"""A kinematic puppet for 6-DoF biped legs: quasi-static gait.

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

from otolith_sim.leg_model import LegModel, chain_fk, sole_world

GRAVITY = 9.81


@dataclass
class BipedGaitConfig:
    """Quasi-static gait. Units: metres, radians, seconds."""
    cycle_s: float = 2.0         # one full stand-shift-stand cycle
    step_height: float = 0.02     # small lift, so a foot can be re-placed
    # Seeded from the vendor `knees_bent` keyframe in scene_mjx.xml by
    # `_vendor_stance()`. NOT the `stand` keyframe of g1.xml, which is the model
    # ZERO pose -- a straight leg -- reaching 0.8021 m at best and leaving 9 mm
    # of margin for any offset. Sampling 40k legal poses gives the envelope.
    base_height: float = 0.7550        # knees_bent qpos[2]
    knee_nominal_hint: float = 0.669    # knees_bent knee angle, radians
    squat_amplitude: float = 0.02 # vertical bob
    sway_amplitude: float = 0.08  # lateral CoM shift over the stance foot
    stride: float = 0.10          # forward travel per cycle
    # Posture prior; overridden from the vendor stance at construction.
    knee_nominal: float = 0.6
    hip_pitch_nominal: float = -0.3
    # Joint-name vocabulary differs per robot and must not be guessed: G1 has
    # `knee` and `hip_pitch`, Apollo has `knee_fe` and `hip_fe`. Matching on the
    # substring "knee" happens to work for both, but matching on "hip_pitch"
    # silently misses Apollo's hip and leaves its posture prior unset -- a
    # plausible-looking pose from an unseeded start.
    knee_match: tuple = ("knee",)
    hip_pitch_match: tuple = ("hip_pitch",)

    @property
    def speed(self) -> float:
        return self.stride / self.cycle_s


@dataclass
class BipedSample:
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
                 cfg: BipedGaitConfig, iters=25, tol=1e-6, damping=1e-3,
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
    # 25, not 80: from the vendor stance seed this converges in 4.2 iterations on
    # average (p95 = 4). The rest of the old budget went entirely into stalls on
    # unreachable targets, where more iterations buy nothing and 80 x 13 Jacobian
    # evaluations per solve is most of the puppet's runtime.
    last_stall = False
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
    if resid > tol:
        # A target outside the reachable envelope stalls rather than diverging, so
        # grinding the iteration cap hides it behind a plausible-looking pose. The
        # residual is returned either way and the gates check it, but say so.
        last_stall = True
    return q, resid


def joint_limits(model, leg: str, lm: LegModel):
    """(DOF, 2) array of the leg's real joint ranges, radians.

    An UNLIMITED hinge reports jnt_range [0, 0], and that is not "the joint sits at
    zero" -- it is "there is no constraint". Handing it back verbatim makes
    solve_leg_ik's clamp pin every joint at exactly 0, so the IK silently cannot
    move and returns a 41 mm residual that looks like a convergence failure rather
    than a missing bounds check. OP3 has no limited joints at all, so every one of
    its twelve went to zero and the gait never bent a joint.

    jnt_limited is the only thing that distinguishes the two cases, so it is what
    this asks. +/-pi is the natural bound for a free hinge and keeps the clamp
    meaningful.
    """
    out = []
    for jn in lm.chain(leg).joints:
        jid = model.joint(jn).id
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
        else:
            lo, hi = -np.pi, np.pi
        out.append([float(lo), float(hi)])
    return np.array(out)


class BipedPuppet:
    """Prescribes a quasi-static G1 gait. Ground truth is the prescription."""

    def __init__(self, model, lm, cfg: BipedGaitConfig | None = None, data=None):
        self.model = model
        self.lm = lm
        self.data = data or mujoco.MjData(model)
        self.cfg = cfg or BipedGaitConfig()
        # Hip lateral positions in the world at the default pose; the nominal
        # foothold for each foot sits under its own hip.
        #
        # Established BEFORE the vendor stance, because OP3's seed bisects the base
        # height on the sole constraint and `_home_q` needs hip_y -- and hip_y
        # depends only on the model, not on the seed. It does need a forward pass
        # first, since body positions are populated only once kinematics has run.
        mujoco.mj_forward(self.model, self.data)
        self.hip_y = np.array([self.data.xpos[self.model.body(
            self.lm.chain(l).bodies[1]).id][1] for l in self.lm.legs])

        # Robot-specific seeding. G1 reads knees_bent out of scene_mjx.xml because
        # g1.xml's only keyframe is the all-zeros zero pose; Apollo reads its own
        # `stand`; OP3 has no keyframes at all and has to solve one. All three are
        # overridden per-robot, so the gait itself never needs to know which robot
        # it is driving.
        self._vendor_stance()
        self.q_home = self._home_q()
        self._prev_pos = None
        self._prev_vel = None

    def _vendor_stance(self):
        """Seed posture and base height from a vendor stance pose. Subclass hook.

        Returns nothing; sets self._vendor_q (leg -> joint angles) and the cfg
        base_height / knee_nominal / hip_pitch_nominal. The base implementation
        does nothing, which is correct for a robot whose defaults are already
        sane -- but every robot here needs it, because the nominal is a seed and
        the seed decides whether the solver converges or stalls against a rail.
        """

    def _nominal_q(self, leg: str):
        """Posture prior. The vendor stance if available, else the config hint."""
        if hasattr(self, "_vendor_q") and leg in self._vendor_q:
            return np.array(self._vendor_q[leg], dtype=float)
        """Posture prior for one leg: bent knee, slight hip pitch. The IK seed."""
        ang = np.zeros(len(self.lm.chain(leg).joints))
        for i, jn in enumerate(self.lm.chain(leg).joints):
            if any(k in jn for k in self.cfg.knee_match):
                ang[i] = self.cfg.knee_nominal
            elif any(k in jn for k in self.cfg.hip_pitch_match):
                ang[i] = self.cfg.hip_pitch_nominal
        return ang

    def _home_q(self):
        """Joint angles that put both feet flat on the ground at `base_height`.

        Solved rather than taken from the vendor `stand` keyframe, which is all
        zeros: that is the model zero pose, a straight leg, and its feet sit
        1.86 mm BELOW the floor.
        """
        out = np.zeros(self.model.nq)
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        # ROOT AT THE ORIGIN, so both the target and the result are base-frame --
        # the same convention `sample()` uses, and the only self-consistent one.
        #
        # This used to pass root_pos = (0, 0, base_height) with a target of
        # (0, hip_y, -base_height), which mixes a world-frame target with a
        # world-frame root and asks the leg to reach 2*base_height below its own
        # root: 1.51 m for a 0.80 m leg. It went unnoticed because the residual here
        # is DISCARDED and `q_home` is only ever read for its non-leg entries --
        # every leg joint is overwritten by the per-sample IK. OP3 turned it into a
        # real fault, because its posture seed is derived from this solve.
        base = np.zeros(3)
        for li, leg in enumerate(self.lm.legs):
            nominal = self._nominal_q(leg)
            # nominal sole position in base frame: under the hip, on the floor
            tgt = np.array([0.0, self.hip_y[li], 0.0 - self.cfg.base_height])
            ang, res = solve_leg_ik(self.lm, leg, tgt, nominal, base, quat, self.cfg,
                                   limits=joint_limits(self.model, leg, self.lm))
            # Checked rather than discarded, precisely because being discarded is
            # what hid this for two robots.
            if res > 1e-4:
                raise ValueError(
                    f"{leg}: home pose IK residual {res * 1000:.3f} mm at base height "
                    f"{self.cfg.base_height:.4f}. The target is unreachable -- the "
                    "stance is too tall for the leg, or the posture seed is wrong.")
            for i, jn in enumerate(self.lm.chain(leg).joints):
                jid = self.model.joint(jn).id
                out[self.model.jnt_qposadr[jid]] = ang[i]
        q = out
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
        # Lateral weight shift, and it has to be CONTINUOUS.
        #
        # The first version snapped y_off to the mean hip position of whichever
        # feet were planted. With only 10% double-support that meant y_off jumped
        # 0.233 m at every transfer -- 116 m/s -- and the base y signal was a
        # staircase: median |vy| 0.0, max 58.2 m/s. A robot does not teleport.
        #
        # That one line was ALL of G1's sigma(r_dot): 3.63 m/s, 12x the budget and
        # essentially all of it in y, which is the sway axis. Stance feet are
        # world-fixed (see _foothold), so r_dot in the base frame is exactly
        # -v_base: any fake base velocity shows up one-for-one in leg odometry.
        #
        # Peak lateral speed is now 2*pi*A/cycle_s = 0.25 m/s for the default
        # 0.08 m amplitude over a 2 s cycle, which is inside the Go2 band.
        y_off = cfg.sway_amplitude * np.sin(2 * np.pi * t / cfg.cycle_s)
        z = cfg.base_height - cfg.squat_amplitude * (0.5 - 0.5 * np.cos(
            2 * np.pi * t / cfg.cycle_s))
        pos = np.array([cfg.speed * t, y_off, z])
        roll = 0.0
        pitch = 0.02 * np.sin(2 * np.pi * t / cfg.cycle_s)
        yaw = 0.0
        quat = _rpy_to_quat(roll, pitch, yaw)
        return pos, quat, np.array([roll, pitch, yaw])

    def sample(self, t: float, dt: float) -> BipedSample:
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
            q_init = self._vendor_q.get(leg, None)
            q_init = self._nominal_q(leg) if q_init is None else q_init
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
        return BipedSample(t=t, qpos=q.copy(), base_pos=base_pos, base_quat=base_quat,
                         base_rpy_rate=rpy_rate, base_accel=accel, contacts=contacts,
                         foot_targets=targets)


def _quat_to_mat(wxyz):
    w, x, y, z = wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _rpy_to_quat(roll, pitch, yaw):
    # cos(angle/2), NOT cos(angle)/2. The latter is not a unit quaternion -- for
    # a 0.02 rad pitch it returns norm 0.125 -- and MuJoCo then applies a
    # malformed base orientation while the IK target was computed with the
    # correct R. The stance foot then advanced with the base at exactly the base's
    # rate: 173 um of creep, 0.087 m/s of apparent foot velocity against
    # sigma_leg = 0.3 m/s, while the base-frame IK residual stayed at 1e-7 m.
    # Only visible for a NON-identity attitude; verified at zero it looked fine.
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])