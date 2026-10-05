"""v0.6 load cases: ground reaction forces from the prescribed kinematics.

WHY NOT MUJOCO'S CONTACT SOLVER (measured, not assumed)
-------------------------------------------------------
FEA needs the load at the foot-ground interface. The obvious source is
MuJoCo's contact solver. It does not work here, for three separate reasons:

1. The puppet is kinematic by construction (`puppet.py` calls `mj_forward` and
   zeroes `qvel`), so it is a prescribed path, not a simulation. There are no
   constraint forces to read -- `data.cfrc_ext` is identically zero.
2. MuJoCo's constraint solver is a FORWARD solver. With `mjENBL_FWDINV` set and
   a prescribed `qacc` of +g (free fall), the static home pose still reported
   160.68 N instead of 0. FWDINV does not invert the contact set for a
   prescribed acceleration; it changes how `qacc` is derived, not which contact
   set is found.
3. Driving the model dynamically needs real actuators, and the Menagerie Go2
   declares `<motor>` actuators with `biastype=mjBIAS_NONE`, `gainprm[0]=1`,
   `biasprm=0`. That makes actuator force = `gainprm[0] * length` = the joint
   ANGLE in radians, and **ctrl is ignored entirely**. Forcing a proper
   affine-bias position servo (`biastype=mjBIAS_AFFINE`,
   `biasprm = [0, -kp, -kv]`) raised mean GRF from 0.66 W to 0.88 W and held
   base height to -0.21 mm drift, but the result stayed bit-identical across a
   100x gain sweep -- the commanded torque still does not reach the plant.

The check that kills all three: a rigid body at constant height moving at
constant speed has zero net vertical acceleration, so its ground reaction must
equal its weight, `sum(F_z) == mass * g`. The rollout gives 0.88 W. Feeding
that to FEA would mean inventing data.

WHAT IS EXACT INSTEAD
---------------------
For prescribed kinematics the total external force follows from Newton's second
law with no plant model at all:

    F_z_total(t) = mass * (g + a_com_z(t))

`a_com_z` comes from forward kinematics on the prescribed joint angles, forming
the centre of mass from the link masses, and differentiating twice. Because
every link's mass enters the CoM, this captures the swing legs' inertial
contribution automatically -- which matters, because the legs are 8.29 kg of
the 15.21 kg total (54%). A crude `m*(g + a_base)` would miss most of it.

No actuator model, no contact solver, no penalty stiffness, no penetration depth.

SMOOTHING IS MANDATORY, AND WHY
-------------------------------
The swing arc is `step_height * sin(pi * s)`, which is not tangent to the
ground at touchdown: the foot arrives with a finite vertical velocity that is
instantly clamped to zero when stance begins. The prescribed CoM height is
therefore only C0, with a velocity kink at each footfall, and the true
acceleration there contains a delta. Differentiating twice amplifies that as
1/h^2. Sampling off the dt grid at `t +/- h` (h = dt/10) made it far worse --
1808 N, 12.1 W -- because those samples land at arbitrary phases.

The fix is a local quadratic least-squares fit (Savitzky-Golay) over a window
of the dt-grid series, taking the fit's second derivative. Two independent
checks say this is right:

  * conservation: mean GRF/W = 0.9993 and it stays in 0.9992..0.9998 across a
    9x range of window sizes;
  * amplitude: peak GRF = 1.24 W at an 18-step window, against 1.26 W predicted
    analytically from the 2.86 Hz bob of +/- 7.8 mm -- whereas an unsmoothed
    central difference gives 2.11 W, which is the kinks, not the physics.

ASSUMPTION, STATED: the split across stance feet is by symmetry. This gait is a
diagonal trot, so FL+RR and FR+RL are mirror pairs and share equally. That is
exact for these pairs and is not a general result; a gait with asymmetric
loading would need the distribution solved from the stance geometry instead.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass

import mujoco
import numpy as np

from otolith_sim.puppet import Go2Puppet

GRAVITY = 9.81
# 18 steps at dt=2 ms = 36 ms. The bob is 2.86 Hz (350 ms period), so this
# resolves it with ~10 samples per period while flattening the footfall kinks.
# The sweep in docs/V06_LOAD_CASES.md shows 18..162 steps all give mean 0.999 W.
DEFAULT_SMOOTH_STEPS = 18


@dataclass
class GrfSeries:
    """Ground reaction resolved per foot over a run. World frame, newtons."""
    t: np.ndarray            # (N,)   s
    fz_total: np.ndarray     # (N,)   sum over stance feet
    fz_feet: np.ndarray      # (N,4)  FL FR RL RR
    stance: np.ndarray       # (N,4)  bool
    mass: float
    puppet_cycle_s: float
    smooth_steps: int

    @property
    def weight(self):
        return self.mass * GRAVITY


def com_height_series(model, puppet, n_steps, dt):
    """CoM height (m) on the dt grid, from FK on prescribed qpos."""
    z = np.empty(n_steps)
    data = mujoco.MjData(model)
    for k in range(n_steps):
        qpos = puppet.sample(model, data, k * dt, dt).qpos
        d = mujoco.MjData(model)
        d.qpos[:] = qpos
        mujoco.mj_kinematics(model, d)
        mujoco.mj_comPos(model, d)
        # subtree 0 is the free-floating trunk, so its CoM is the whole robot.
        z[k] = d.subtree_com[0][2]
    return z


def _smoothed_accel(z, dt, window):
    """Local-quadratic (Savitzky-Golay) second derivative, same length as z.

    Edges get NaN rather than a one-sided estimate: they are dropped by the
    caller, and a fabricated endpoint value would show up in the peak.
    """
    a = np.full(z.shape, np.nan)
    x = np.arange(-window, window + 1) * dt
    for i in range(window, z.size - window):
        a[i] = 2.0 * np.polyfit(x, z[i - window:i + window + 1], 2)[0]
    return a


def record_grf(model, duration_s, dt=None, cfg=None,
                smooth_steps=DEFAULT_SMOOTH_STEPS):
    """Compute the GRF series for `duration_s` of the puppet's gait."""
    dt = dt or model.opt.timestep
    puppet = Go2Puppet(model, cfg)
    n = int(round(duration_s / dt))
    mass = float(model.body_mass[1:].sum())   # exclude the static world body

    z = com_height_series(model, puppet, n, dt)
    a_com = _smoothed_accel(z, dt, smooth_steps)
    fz = mass * (GRAVITY + a_com)

    data = mujoco.MjData(model)
    stance = np.zeros((n, 4), dtype=bool)
    for k in range(n):
        stance[k] = puppet.sample(model, data, k * dt, dt).contacts

    fz_feet = np.zeros((n, 4))
    counts = stance.sum(axis=1, keepdims=True)
    np.divide(fz[:, None], np.maximum(counts, 1), out=fz_feet)
    fz_feet[~stance] = 0.0

    return GrfSeries(t=np.arange(n) * dt,
                     fz_total=fz,
                     fz_feet=fz_feet,
                     stance=stance,
                     mass=mass,
                     puppet_cycle_s=float(puppet.cfg.cycle_s),
                     smooth_steps=int(smooth_steps))


def load_summary(series: GrfSeries, warmup_cycles=2):
    """FEA-facing summaries. Drops `warmup_cycles` so start-up bias is out."""
    dt = series.t[1] - series.t[0]
    per = int(round(series.puppet_cycle_s / dt))
    # The smoother leaves NaN in `smooth_steps` cells at each end; trim those
    # too, or max()/mean() propagate NaN and the gate silently stops gating.
    lo = warmup_cycles * per + series.smooth_steps
    hi = series.fz_total.size - series.smooth_steps
    if hi - lo < per:
        raise ValueError(
            f"run too short: {series.fz_total.size} steps leaves {hi - lo} after "
            f"trimming {warmup_cycles} warm-up cycles plus {series.smooth_steps} "
            f"edge cells; need at least {(warmup_cycles + 1) * per + 2 * series.smooth_steps}")
    w = slice(lo, hi)
    fz, feet, stance = series.fz_total[w], series.fz_feet[w], series.stance[w]
    stance_idx = [i for i in range(4)]

    return {
        "mass_kg": series.mass,
        "weight_N": series.weight,
        "mean_fz_N": float(fz.mean()),
        "mean_fz_over_weight": float(fz.mean() / series.weight),
        "peak_fz_N": float(fz.max()),
        "peak_fz_over_weight": float(fz.max() / series.weight),
        "rms_fz_N": float(np.sqrt((fz ** 2).mean())),
        "per_foot_peak_N": [float(feet[:, i].max()) for i in stance_idx],
        # impulse carried by one foot per stance: sum Fz*dt over contiguous
        # stance runs, which is the number a fatigue spectrum actually wants.
        "per_foot_stance_impulse_Ns": _stance_impulses(feet, stance, dt),
        "duty_measured": float(stance.mean()),
    }


def _stance_impulses(feet, stance, dt):
    out = []
    for i in range(4):
        on = stance[:, i].astype(np.int8)
        edges = np.diff(np.concatenate(([0], on, [0])))
        starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
        vals = [float(feet[s:e, i].sum() * dt) for s, e in zip(starts, ends)
                if e - s > 2]          # ignore 1-2 step touchdown blips
        out.append(vals)
    return out