"""A kinematic puppet for the G1 biped: 6-DoF legs, quasi-static gait.

WHY A SEPARATE CLASS
====================
`Go2Puppet` is a 4-legged trot with closed-form 2R IK. Neither half transfers:
a biped cannot trot (no diagonal pairs), and the 2R IK is exactly the model the
G1 leg cannot use (see sim/otolith_sim/leg_model.py). Go2Puppet is left alone
because every recorded result in the repo was produced by it.

THE GAIT IS DELIBERATELY BORING
================================
stand, lateral weight shift, squat. Every falsification target v0.5 cares about
is exercised quasi-statically:

  * the weight shift IS the 9.2 deg lateral leg tilt that breaks the planar
    model, so a locomotion gait would not test it any harder;
  * two-foot contact exercises the load split, which `grf.py` assumes is equal
    across the stance set -- exact for a trot's mirror pairs, NOT exact here;
  * the squat sweeps the joint ranges against the CORDIC fold limit.

Writing a kinematic biped *walk* is mostly gait authoring -- footstep
scheduling, CoM sway, singularity management in the numeric IK -- and that
tests the gait author rather than the filter. Walking is a stretch goal, not a
prerequisite. See docs/V05_HUMANOID.md.

THE ONE HARD PROPERTY
=====================
Stance feet must be EXACTLY stationary. The whole eval rests on it: if a
planted foot creeps, leg odometry reports motion that the filter then attributes
to the robot, and every RMSE downstream is contaminated. Go2Puppet gets a
constant 2.000 mm residual (the foothold inset); this one is gated tighter.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from otolith_sim.leg_model import LegModel, load_g1, chain_fk, sole_world

GRAVITY = 9.81


@dataclass
class G1GaitConfig:
    """Quasi-static gait. Units: metres, radians, seconds."""
    cycle_s: float = 2.0         # one full stand-shift-stand cycle
    step_height: float = 0.02     # small lift, so a foot can be re-placed
    # NOT the vendor `stand` height of 0.793. That pose is the model ZERO pose --
    # a straight leg -- which reaches 0.8021 m at best, leaving 9 mm of margin
    # for any forward or lateral offset. Sampling 40k legal poses gives the
    # reachable envelope; the sole spans z = -0.8021..+0.5805. Sitting at 0.79
    # means the gait is one foothold offset away from being unreachable, which is
    # exactly the failure the DLS solver papers over with a 38 mm residual.
    base_height: float = 0.70
    squat_amplitude: float = 0.02 # vertical bob
    sway_amplitude: float = 0.08  # lateral CoM shift over the stance foot
    stride: float = 0.10          # forward travel per cycle
    knee_nominal: float = 0.6     # posture target, keeps the knee bent
    hip_pitch_nominal: float = -0.3

    @property
    def speed(self) -> float:
        return self.stride / self.cycle_s


@dataclass
class G1Sample:
    t: float
    qpos: np.ndarray
    base_pos: np.ndarray
    base_quat: np.ndarray
    base_rpy_rate: np.ndarray
    base_accel: np.ndarray
    contacts: np.ndarray          # (2,) bool, left, right
    foot_targets: np.ndarray      # (2,3) world


def _leg_jacobian(lm: LegModel, leg: str, q, root_pos, root_quat, eps=1e-5):
    """d(sole position)/d(joint angles) by central differences, base frame.

    Numerical on purpose: the analytic Jacobian for a 6-DoF chain with
    off-diagonal frame rotations is easy to get subtly wrong, and a wrong
    Jacobian still converges to *something*, which is the dangerous failure. The
    gate that the IK residual is small is what catches that.
    """
    chain = lm.chain(leg)
    sole = lm.sole[leg]
    base = sole_world(chain, q, root_pos, root_quat, sole)
    J = np.zeros((3, chain.dof))
    for i in range(chain.dof):
        qp = np.array(q, dtype=float); qp[i] += eps
        qm = np.array(q, dtype=float); qm[i] -= eps
        J[:, i] = (sole_world(chain, qp, root_pos, root_quat, sole)
                   - sole_world(chain, qm, root_pos, root_quat, sole)) / (2 * eps)
    return J, base


def solve_leg_ik(lm: LegModel, leg: str, target_base, q_init, root_pos, root_quat,
                 cfg: G1GaitConfig, iters=80, tol=1e-6, damping=1e-3,
                 posture_gain=0.02, limits=None):
    """Damped least squares onto a 6-DoF leg, with a posture null-space term.

    Damped because the leg passes near a singularity when the foot is directly
    under the hip, which is roughly where a biped's stance foot is; plain LS
    would take an enormous step there.

    Two things were needed to make this stable, and both failed silently:

    * An SVD-built null-space projector, NOT the damped normal equations.
      `I - J^T (J J^T + lambda^2 I)^-1 J` is only a projector when lambda is
      zero, so the damped version leaks task motion into the posture step and the
      solve diverged -- 1190 mm of residual, with the foot nowhere near the
      target.
    * **Joint limit clamping.** A true projector means the posture term cannot
      move the foot, which is what makes it safe to leave on -- and also means
      it can wind the joints without bound. Without clamping the gait reached
      ankle_roll = 102 rad against a +-0.26 rad limit, and the FK still put the
      foot exactly on target, so the residual stayed at 1e-5 mm while the pose
      was physically absurd. A converged solve is not a valid pose.
    """
    joints = lm.chain(leg).joints
    nominal = np.zeros(len(joints))
    for i, jn in enumerate(joints):
        if "knee" in jn:
            nominal[i] = cfg.knee_nominal
        elif "hip_pitch" in jn:
            nominal[i] = cfg.hip_pitch_nominal
    q = np.array(q_init, dtype=float)
    target = np.asarray(target_base, dtype=float)
    chain = lm.chain(leg)
    sole = lm.sole[leg]
    if limits is None:
        limits = np.tile(np.array([[-np.pi, np.pi]]), (chain.dof, 1))
    lo = np.asarray(limits)[:, 0]
    hi = np.asarray(limits)[:, 1]
    for _ in range(iters):
        J, cur = _leg_jacobian(lm, leg, q, root_pos, root_quat)
        err = target - cur
        if np.linalg.norm(err) < tol:
            break
        JT = J.T
        H = J @ JT + (damping ** 2) * np.eye(3)
        dq = JT @ np.linalg.solve(H, err)
        U, S, Vt = np.linalg.svd(J, full_matrices=True)
        N = np.eye(len(q)) - Vt.T @ Vt          # orthogonal null-space projector
        dq = dq + N @ (posture_gain * (nominal - q))
        q = np.clip(q + dq, lo, hi)
    resid = float(np.linalg.norm(target - sole_world(chain, q, root_pos, root_quat, sole)))
    return q, resid


def joint_limits(model, leg: str, lm: LegModel):
    """(DOF, 2) array of the leg's real joint ranges, radians."""
    out = []
    for jn in lm.chain(leg).joints:
        r = model.jnt_range[model.joint(jn).id]
        out.append([float(r[0]), float(r[1])])
    return np.array(out)


class G1Puppet:
    """Prescribes a quasi-static G1 gait. Ground truth is the prescription."""

    def __init__(self, model, data=None, cfg: G1GaitConfig | None = None):
        self.model = model
        self.data = data or mujoco.MjData(model)
        self.cfg = cfg or G1GaitConfig()
        self.lm = load_g1(model)
        mujoco.mj_forward(self.model, self.data)
        # Hip lateral positions in the world at the home pose; the nominal
        # foothold for each foot sits under its own hip. Read AFTER a forward
        # pass, since body positions are only populated once kinematics has run.
        self.hip_y = np.array([self.data.xpos[self.model.body(f"{l}_hip_roll_link").id][1]
                               for l in self.lm.legs])
        self.q_home = self._home_q()
        self._prev_pos = None
        self._prev_vel = None

    def _nominal_q(self, leg: str):
        """Posture prior for one leg: bent knee, slight hip pitch. The IK seed."""
        ang = np.zeros(len(self.lm.chain(leg).joints))
        for i, jn in enumerate(self.lm.chain(leg).joints):
            if "knee" in jn:
                ang[i] = self.cfg.knee_nominal
            elif "hip_pitch" in jn:
                ang[i] = self.cfg.hip_pitch_nominal
        return ang

    def _home_q(self):
        """Joint angles that put both feet flat on the ground at `base_height`.

        Solved rather than taken from the vendor `stand` keyframe, which is all
        zeros: that is the model zero pose, a straight leg, and its feet sit
        1.86 mm BELOW the floor.
        """
        q = np.zeros(self.model.nq)
        base = np.array([0.0, 0.0, self.cfg.base_height])
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        out = np.zeros(self.model.nq)
        for li, leg in enumerate(self.lm.legs):
            nominal = self._nominal_q(leg)
            # nominal sole position in base frame: under the hip, on the floor
            tgt = np.array([0.0, self.hip_y[li], 0.0 - self.cfg.base_height])
            ang, _ = solve_leg_ik(self.lm, leg, tgt, nominal, base, quat, self.cfg,
                                  limits=joint_limits(self.model, leg, self.lm))
            for i, jn in enumerate(self.lm.chain(leg).joints):
                jid = self.model.joint(jn).id
                out[self.model.jnt_qposadr[jid]] = ang[i]
        q[:] = out
        q[2] = self.cfg.base_height
        return q

    def _phases(self, t):
        """Per-leg cycle phase in [0,1). Left leads the right by half a cycle."""
        u = (t / self.cfg.cycle_s) % 1.0
        return np.array([u, (u + 0.5) % 1.0])

    def _foothold(self, leg: str, li: int, t: float):
        """World foothold and contact flag, held EXACTLY fixed during stance.

        The offset enters `floor` with the SAME sign it enters the position.
        An earlier version floored on `t/T - 0.5` while placing on `k - 0.5`,
        which put the right foot a full stride behind its hip at t=0: 0.1225 m
        back and 0.1165 m lateral, 0.80 m from a 0.80 m leg. Unreachable, and it
        showed up only as a 38 mm IK residual rather than as an error.
        """
        cfg = self.cfg
        off = 0.0 if li == 0 else 0.5
        u = self._phases(t)[li]
        k = int(np.floor(t / cfg.cycle_s + off))
        duty = 0.55
        nominal_y = float(self.hip_y[li])
        foothold_x = (k + off + duty / 2.0) * cfg.stride
        if u < duty:
            return np.array([foothold_x, nominal_y, 0.0]), True, u
        s = (u - duty) / (1.0 - duty)
        target = np.array([foothold_x + cfg.stride * s, nominal_y,
                           cfg.step_height * np.sin(np.pi * s)])
        return target, False, s

    def _base_motion(self, t: float):
        """Base pose: forward travel, lateral sway toward the stance foot, squat."""
        cfg = self.cfg
        ph = self._phases(t)
        stance = [li for li in range(2) if self._foothold(self.lm.legs[li], li, t)[1]]
        # Sway toward the mean of whichever feet are planted; with both down,
        # stay centred.
        y_off = float(np.mean(self.hip_y[stance])) if stance else 0.0
        z = cfg.base_height - cfg.squat_amplitude * (0.5 - 0.5 * np.cos(
            2 * np.pi * t / cfg.cycle_s))
        pos = np.array([cfg.speed * t, y_off, z])
        roll = 0.0
        pitch = 0.02 * np.sin(2 * np.pi * t / cfg.cycle_s)
        yaw = 0.0
        quat = _rpy_to_quat(roll, pitch, yaw)
        return pos, quat, np.array([roll, pitch, yaw])

    def sample(self, t: float, dt: float) -> G1Sample:
        cfg = self.cfg
        base_pos, base_quat, rpy = self._base_motion(t)
        R = _quat_to_mat(base_quat)
        q = self.q_home.copy()
        q[0:3] = base_pos
        q[3:7] = base_quat

        contacts = np.zeros(2, dtype=bool)
        targets = np.zeros((2, 3))
        self._resid = {}
        for li, leg in enumerate(self.lm.legs):
            tgt, contact, _ = self._foothold(leg, li, t)
            targets[li] = tgt
            contacts[li] = contact
            p_base = R.T @ (tgt - base_pos)
            # Seed from the NOMINAL posture every sample, never from the previous
            # solution. Warm-starting looks faster and is strictly worse: the knee
            # range starts at -0.087 rad, so a warm start can leave the solver
            # against a rail where the task step is clipped, and it then stalls at
            # 105-247 mm. From nominal it converges to 1e-7 m with no violations.
            q_init = self._nominal_q(leg)
            ang, res = solve_leg_ik(self.lm, leg, p_base, q_init,
                                    np.zeros(3), np.array([1.0, 0, 0, 0]), cfg,
                                    limits=joint_limits(self.model, leg, self.lm))
            self._resid[leg] = res
            for i, jn in enumerate(self.lm.chain(leg).joints):
                jid = self.model.joint(jn).id
                q[self.model.jnt_qposadr[jid]] = ang[i]

        if self._prev_pos is None:
            vel = np.zeros(3); accel = np.zeros(3); rpy_rate = np.zeros(3)
        else:
            vel = (base_pos - self._prev_pos) / dt
            accel = (vel - self._prev_vel) / dt
            rpy_rate = (rpy - self._prev_rpy) / dt if hasattr(self, "_prev_rpy") else np.zeros(3)
        self._prev_pos = base_pos.copy()
        self._prev_vel = vel.copy()
        self._prev_rpy = rpy.copy()

        self.data.qpos[:] = q
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return G1Sample(t=t, qpos=q.copy(), base_pos=base_pos, base_quat=base_quat,
                         base_rpy_rate=rpy_rate, base_accel=accel, contacts=contacts,
                         foot_targets=targets)


def _quat_to_mat(wxyz):
    w, x, y, z = wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = np.cos(roll) / 2, np.sin(roll) / 2
    cp, sp = np.cos(pitch) / 2, np.sin(pitch) / 2
    cy, sy = np.cos(yaw) / 2, np.sin(yaw) / 2
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])