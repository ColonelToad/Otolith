"""Joint reactions by statics on the CoM-balanced contact force.

WHY NOT `qfrc_constraint`
========================
The obvious move is to read MuJoCo's constraint forces, since the puppet zeroes
`qvel` and calls `mj_forward`, so the pose is static and the forces look like a
clean measurement. They are not. Checked directly:

    total contact force from mj_forward   219.09 N
    total contact force from CoM balance  166.07 N
    relative residual                      36.0%

A 36% imbalance means the prescribed motion is not dynamically consistent, so
the constraint solver's forces are not reactions to anything real. The puppet
drives positions directly and never integrates dynamics, so its legs are not
holding the body up -- they are being *placed*. Reading `qfrc_constraint` here
would produce plausible numbers with no physical referent, which is the failure
mode this repo keeps paying for.

So the force input is the CoM momentum balance instead: the one contact-force
method in this repo with a verified conservation check (0.998661 W,
docs/V06_LOAD_CASES.md). It answers "what force does the prescribed motion
demand", and statics then distributes it into the joints.

WHAT THIS DOES AND DOES NOT GIVE
================================
Gives: the bearing force at each hip/knee, and the torque about each joint axis,
with shear and torsion included -- which is what phase C flagged as missing
from the load case (it resolved a vertical force into the thigh frame by stance
angle, a model rather than a measurement).

Does not give: the distribution of load *between* stance feet, which the CoM
wrench cannot determine. In a diagonal trot the stance pair is a mirror pair, so
an equal split is exact for FL+RR and FR+RL and is assumed here. A gait with
asymmetric loading would need the distribution solved from the stance geometry
instead, which means solving a separate problem per stance pair.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from otolith_sim.puppet import Go2Puppet

GRAVITY = 9.81
LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("hip", "thigh", "calf")


@dataclass
class JointReactionSeries:
    """Per-joint bearing loads over a run. Newtons and newton-metres."""
    t: np.ndarray              # (N,)
    force: np.ndarray          # (N, 4, 3)  |force| per leg per joint, world
    force_vec: np.ndarray      # (N, 4, 3, 3) world reaction vector
    axis_torque: np.ndarray    # (N, 4, 3)  torque about each joint's own axis
    stance: np.ndarray         # (N, 4) bool
    smooth_steps: int          # edge cells lost to the NaN-producing smoother


def com_series(model, puppet, n_steps, dt):
    """Whole-robot CoM position (m) on the dt grid, from FK on prescribed qpos.

    The free-floating trunk is subtree 0, so its CoM is the whole robot.
    """
    out = np.empty((n_steps, 3))
    for k in range(n_steps):
        q = puppet.sample(model, mujoco.MjData(model), k * dt, dt).qpos
        d = mujoco.MjData(model)
        d.qpos[:] = q
        mujoco.mj_kinematics(model, d)
        mujoco.mj_comPos(model, d)
        out[k] = d.subtree_com[0]
    return out


def _accel(series, dt, window):
    """Second derivative by local-quadratic fit. NaN at both edges."""
    out = np.full(series.shape, np.nan)
    x = np.arange(-window, window + 1) * dt
    for i in range(window, series.shape[0] - window):
        for c in range(series.shape[1]):
            out[i, c] = 2.0 * np.polyfit(x, series[i - window:i + window + 1, c], 2)[0]
    return out


def _leg_structure(model):
    """Body ids distal to each joint, the joint origin, and its axis."""
    out = {}
    for leg in LEGS:
        bodies = [model.body(f"{leg}_hip").id,
                  model.body(f"{leg}_thigh").id,
                  model.body(f"{leg}_calf").id]
        joints = []
        for k, jname in enumerate(JOINTS):
            jid = model.joint(f"{leg}_{jname}_joint").id
            joints.append({
                "dof": int(model.jnt_dofadr[jid]),
                "axis_world": np.array(model.jnt_axis[jid]),
                # everything from this joint's own body down the chain
                "distal": bodies[k:],
                "origin_body": bodies[k],
            })
        out[leg] = joints
    return out


def record_joint_reactions(model, duration_s, dt=None, cfg=None,
                           smooth_steps=18):
    """Bearing loads at every hip/knee over `duration_s` of the gait."""
    dt = dt or model.opt.timestep
    puppet = Go2Puppet(model, cfg)
    n = int(round(duration_s / dt))
    mass = float(model.body_mass[1:].sum())

    a = _accel(com_series(model, puppet, n, dt), dt, smooth_steps)
    # Total GROUND force on the robot = m*(a_com - g_vec) = m*(a_com + 9.81 z).
    # Plus, not minus: the first version had `a - [0,0,GRAVITY]`, which flipped
    # the contact sign. Two sign errors then cancelled in the force SUM -- a
    # swinging leg still showed exactly its own weight, so the obvious check
    # passed -- but every stance number came out ~6% high and the reported
    # components had the wrong signs. Caught by comparing against grf's fz_feet.
    f_total = mass * (a + np.array([0.0, 0.0, GRAVITY]))

    data = mujoco.MjData(model)
    stance = np.zeros((n, 4), dtype=bool)
    for k in range(n):
        stance[k] = puppet.sample(model, data, k * dt, dt).contacts

    counts = stance.sum(axis=1, keepdims=True)                  # (n,1)
    # Equal split across stance feet (documented at module level). The repeat is
    # needed to get (n,4,3): f_total[:,None,:]/counts alone broadcasts to (n,1,3)
    # and then silently fails the stance mask.
    f_feet = (np.repeat(f_total[:, None, :], 4, axis=1)
              / np.maximum(counts, 1)[:, :, None])             # (n,4,3)
    f_feet[~stance] = 0.0

    struct = _leg_structure(model)
    force = np.zeros((n, 4, 3))
    force_vec = np.zeros((n, 4, 3, 3))
    axis_torque = np.zeros((n, 4, 3))

    foot_geom = {leg: model.geom(leg).id for leg in LEGS}
    foot_body = {leg: model.geom_bodyid[foot_geom[leg]] for leg in LEGS}

    for k in range(n):
        for li, leg in enumerate(LEGS):
            for ji, j in enumerate(struct[leg]):
                origin = data.xpos[j["origin_body"]]
                fsum = np.zeros(3)
                msum = np.zeros(3)
                for b in j["distal"]:
                    w = np.array([0.0, 0.0, -model.body_mass[b] * GRAVITY])
                    r = data.xipos[b] - origin
                    fsum += w
                    msum += np.cross(r, w)
                    if b == foot_body[leg]:
                        fc = f_feet[k, li]
                        p = data.geom_xpos[foot_geom[leg]] - origin
                        fsum += fc
                        msum += np.cross(p, fc)
                # The joint must supply exactly the negative of everything else.
                rj = -fsum
                force_vec[k, li, ji] = rj
                force[k, li, ji] = np.linalg.norm(rj)
                axis_torque[k, li, ji] = float(np.dot(-msum, j["axis_world"]))

    return JointReactionSeries(t=np.arange(n) * dt, force=force,
                               force_vec=force_vec, axis_torque=axis_torque,
                               stance=stance, smooth_steps=int(smooth_steps))


def summarise(series: JointReactionSeries, warmup_cycles=2, cycle_s=0.7):
    """Peak and mean bearing loads, dropping warm-up cycles.

    Only stance-foot rows are meaningful: a swing foot carries no contact force,
    so its "reaction" is just the leg's own weight hanging from the hip.
    """
    dt = series.t[1] - series.t[0]
    per = int(round(cycle_s / dt))
    # Trim the warm-up cycles AND the smoother's NaN edges. Dropping only the
    # former leaves NaNs in, and max() over a NaN is NaN, so the gate would
    # silently report nothing (the same trap grf.load_summary documents).
    lo = warmup_cycles * per
    hi = series.t.size - series.smooth_steps
    if hi - lo < per:
        raise ValueError(
            f"run too short: {series.t.size} steps leaves {hi - lo} after "
            f"trimming {warmup_cycles} warm-up cycles plus "
            f"{series.smooth_steps} edge cells")
    w = slice(lo, hi)

    out = {}
    for li, leg in enumerate(LEGS):
        st = series.stance[w][:, li]
        for ji, jname in enumerate(JOINTS):
            # (n,4,3): slice the window first, THEN index leg and joint.
            # series.force[w][li, ji] indexes before selecting the window and
            # silently returns the wrong thing.
            sel = series.force[w][:, li, ji][st]
            tor = series.axis_torque[w][:, li, ji][st]
            if sel.size == 0:
                out[f"{leg}_{jname}"] = {"peak_N": float("nan"),
                                         "mean_N": float("nan"),
                                         "peak_Nm": float("nan"),
                                         "samples": 0}
                continue
            out[f"{leg}_{jname}"] = {
                "peak_N": float(sel.max()),
                "mean_N": float(sel.mean()),
                "peak_Nm": float(np.abs(tor).max()),
                "samples": int(sel.size),
            }
    return out
