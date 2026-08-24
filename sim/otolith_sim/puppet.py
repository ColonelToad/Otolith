"""Otolith sensor-simulation layer (pure numpy, no ROS imports).

The kinematic puppet prescribes a physically consistent trot: footholds are
fixed in the world, the base advances at matching speed, and per-leg closed
form IK produces joint angles. Consequences, by construction:
  - "contact" flags are true when a foot is in its stance phase
  - stance feet are stationary in the world frame
so the fusion EKF is evaluated under sensor noise only, not puppet artifacts.

Link constants are introspected from the model at runtime (hip/thigh/calf
frame offsets, foot geom positions) rather than hardcoded.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

GRAVITY = 9.81


# ---------------------------------------------------------------------------
# model introspection
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LegGeom:
    """Per-leg constants derived from the model, plus qpos addresses."""
    name: str                 # "FL", "FR", "RL", "RR"
    hip_base: np.ndarray      # hip joint position in base frame
    side: int                 # +1 = left, -1 = right
    a_offset: float           # lateral offset of the leg plane (hip frame |y|)
    x_offset: float           # fore/aft offset of the pitch axis (hip frame |x|)
    L1: float                 # hip(pitch) -> knee
    L2: float                 # knee -> foot center
    qpos_adr: tuple[int, int, int]   # (hip, thigh, calf) qpos indices
    foot_geom_id: int         # foot sphere geom (named e.g. "FL")


def _leg_geoms(model) -> dict[str, LegGeom]:
    legs = {}
    for side_name, side in (("FL", 1), ("FR", -1), ("RL", 1), ("RR", -1)):
        hip_id = model.joint(f"{side_name}_hip_joint").id
        thigh_id = model.joint(f"{side_name}_thigh_joint").id
        calf_id = model.joint(f"{side_name}_calf_joint").id
        hip_body = model.body(f"{side_name}_hip").id
        thigh_body = model.body(f"{side_name}_thigh").id
        calf_body = model.body(f"{side_name}_calf").id
        foot_gid = model.geom(side_name).id

        # frame offsets, introspected rather than hardcoded
        hip_base = model.body_pos[hip_body].copy()
        thigh_in_hip = model.body_pos[thigh_body].copy()
        calf_in_thigh = model.body_pos[calf_body].copy()
        foot_in_calf = model.geom_pos[foot_gid].copy()

        L1 = float(np.linalg.norm(calf_in_thigh))
        L2 = float(np.linalg.norm(foot_in_calf))
        legs[side_name] = LegGeom(
            name=side_name,
            hip_base=hip_base,
            side=side,
            a_offset=float(abs(thigh_in_hip[1])) or 0.0,
            x_offset=float(thigh_in_hip[0]),
            L1=L1,
            L2=L2,
            qpos_adr=(model.jnt_qposadr[hip_id],
                      model.jnt_qposadr[thigh_id],
                      model.jnt_qposadr[calf_id]),
            foot_geom_id=foot_gid,
        )
    return legs


def foot_geom_ids(model) -> dict[str, int]:
    return {name: model.geom(name).id for name in ("FL", "FR", "RL", "RR")}


# ---------------------------------------------------------------------------
# closed-form leg IK
# ---------------------------------------------------------------------------

def leg_ik(leg: LegGeom, target_base: np.ndarray):
    """Joint angles (hip, thigh, calf) placing the foot center at
    `target_base` (base-frame). Returns None if unreachable.

    Verified against the Go2 home keyframe (theta = 0, 0.9, -1.8).

    Chain per leg: hip rolls about x; the pitch plane sits at lateral
    offset a = side * |thigh_y| in the rolled frame; two links L1, L2
    (pitch about y) reach the foot center.
    """
    d = np.asarray(target_base, dtype=float) - leg.hip_base
    dy, dz = d[1], d[2]

    # hip roll: Rx(theta1) must carry (dy, dz) onto the leg plane,
    # i.e. cos(theta1 - phi) * rho = side * a_offset, phi = atan2(dz, dy)
    rho = np.hypot(dy, dz)
    a = leg.side * leg.a_offset
    if rho < abs(a) or rho == 0.0:
        return None
    phi = np.arctan2(dz, dy)
    base_ang = np.arccos(np.clip(a / rho, -1.0, 1.0))
    # two roll solutions; pick the one nearer hip angle 0 (Go2 stance)
    cand = (phi - base_ang, phi + base_ang)
    theta1 = min(cand, key=lambda th: abs(th))

    # in-plane coordinates after the roll
    x_in = d[0]
    z_in = -np.sin(theta1) * dy + np.cos(theta1) * dz

    r = np.hypot(x_in, z_in)
    if r > leg.L1 + leg.L2 or r < abs(leg.L1 - leg.L2) or r == 0.0:
        return None
    # planar 2-link in (x, z): links point "down" at zero pitch, so the
    # link angle is (theta2 - pi) measured from +z; Go2 uses the
    # knee-back branch (home: theta = 0, 0.9, -1.8 reproduces exactly)
    psi = np.arctan2(x_in, z_in)
    gamma = np.arccos(np.clip(
        (r * r + leg.L1 * leg.L1 - leg.L2 * leg.L2) / (2.0 * r * leg.L1),
        -1.0, 1.0))
    theta2 = psi + np.pi + gamma
    theta2 = (theta2 + np.pi) % (2.0 * np.pi) - np.pi

    v = np.arctan2(-(x_in + leg.L1 * np.sin(theta2)),
                   -(z_in + leg.L1 * np.cos(theta2)))
    theta3 = v - theta2
    return (theta1, theta2, theta3)


# ---------------------------------------------------------------------------
# gait generation
# ---------------------------------------------------------------------------

@dataclass
class GaitConfig:
    cycle_s: float = 0.7        # full gait cycle
    duty: float = 0.55          # fraction of cycle in stance
    stride: float = 0.14        # base advance per cycle
    step_height: float = 0.06
    base_height: float = 0.27
    bob: float = 0.008          # vertical wobble amplitude
    roll_wobble: float = 0.02   # rad
    pitch_wobble: float = 0.015 # rad

    @property
    def speed(self) -> float:
        return self.stride / self.cycle_s


# trot: diagonal pairs move together (FL+RR at phase 0, FR+RL offset by half)
TROT_PHASES = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}


@dataclass
class GaitSample:
    t: float
    qpos: np.ndarray            # (7 + 12,) full puppet state
    base_pos: np.ndarray        # world
    base_quat: np.ndarray       # world (wxyz)
    base_rpy_rate: np.ndarray   # (roll, pitch, yaw) rates for the IMU
    base_accel: np.ndarray      # world-frame linear acceleration (IMU input)
    contacts: np.ndarray        # (4,) bool, order FL FR RL RR
    foot_targets: np.ndarray    # (4, 3) world foot centers


class Go2Puppet:
    """Kinematic trot: footholds fixed in the world, base advancing in lockstep.

    Ground truth (base pose/rates/accel) comes from the prescribed path, so it
    is exact up to finite differencing.
    """

    def __init__(self, model, cfg: GaitConfig | None = None):
        self.model = model
        self.cfg = cfg or GaitConfig()
        self.legs = _leg_geoms(model)
        kid = model.key("home").id
        self.q_home = model.key_qpos[kid].copy()
        # nominal footholds: feet under the hips at stance height
        self._nominal = {
            name: np.array([self.legs[name].hip_base[0],
                            self.legs[name].hip_base[1],
                            0.0])
            for name in self.legs
        }
        self._prev_rpy = None
        self._prev_pos = None

    # -- foothold schedule ---------------------------------------------------
    def _foothold(self, name: str, t: float):
        """World foot target, contact flag for leg `name` at time t.

        Footholds are world-fixed on a stride grid: foothold_k sits under the
        nominal hip at mid-stance, so stance feet are exactly stationary and
        leg odometry is consistent by construction.
        """
        cfg = self.cfg
        p = TROT_PHASES[name]
        u = t / cfg.cycle_s + p              # unrolled cycle index
        k = np.floor(u)
        local = u - k                        # 0..1 within this leg's cycle
        nominal_x, nominal_y, _ = self._nominal[name]

        # world x of this foot's current foothold (and the next one)
        foothold_x = nominal_x + (k - p + cfg.duty / 2.0) * cfg.stride
        next_x = foothold_x + cfg.stride

        if local < cfg.duty:
            # stance: planted
            return (np.array([foothold_x, nominal_y, 0.0]), True, 0.0)
        # swing: arc from foothold to the next foothold
        s = (local - cfg.duty) / (1.0 - cfg.duty)
        target = np.array([foothold_x + (next_x - foothold_x) * s,
                           nominal_y,
                           cfg.step_height * np.sin(np.pi * s)])
        return (target, False, s)

    # -- base motion ---------------------------------------------------------
    def _base_motion(self, t: float):
        cfg = self.cfg
        pos = np.array([cfg.speed * t, 0.0, cfg.base_height
                        + cfg.bob * np.sin(2 * np.pi * 2.0 * t / cfg.cycle_s)])
        roll = cfg.roll_wobble * np.sin(2 * np.pi * t / cfg.cycle_s)
        pitch = cfg.pitch_wobble * np.sin(2 * np.pi * t / cfg.cycle_s + 0.7)
        yaw = 0.0
        quat = _rpy_to_quat(roll, pitch, yaw)
        return pos, quat, np.array([roll, pitch, yaw])

    # -- sample --------------------------------------------------------------
    def sample(self, model, data, t: float, dt: float) -> GaitSample:
        cfg = self.cfg
        base_pos, base_quat, rpy = self._base_motion(t)
        R = _quat_to_mat(base_quat)

        q = self.q_home.copy()
        q[0:3] = base_pos
        q[3:7] = base_quat

        contacts = np.zeros(4, dtype=bool)
        targets = np.zeros((4, 3))
        order = ("FL", "FR", "RL", "RR")

        # rates/accel by finite difference against the previous sample
        # (first sample reports zeros; documented in the node)
        if self._prev_pos is None or self._prev_rpy is None:
            rpy_rate = np.zeros(3)
            accel_world = np.zeros(3)
            vel_world = np.zeros(3)
        else:
            vel_world = (base_pos - self._prev_pos) / dt
            accel_world = (vel_world - self._prev_vel) / dt if hasattr(self, "_prev_vel") else np.zeros(3)
            rpy_rate = (rpy - self._prev_rpy) / dt
        self._prev_pos = base_pos.copy()
        self._prev_rpy = rpy.copy()
        self._prev_vel = vel_world.copy()

        for i, name in enumerate(order):
            target, contact, _ = self._foothold(name, t)
            targets[i] = target
            contacts[i] = contact
            # target in base frame (base yaw is zero; roll/pitch are small but
            # included exactly)
            p_base = R.T @ (target - base_pos)
            sol = leg_ik(self.legs[name], p_base)
            if sol is None:
                raise ValueError(f"{name}: unreachable foot target {target}")
            adr = self.legs[name].qpos_adr
            q[adr[0]:adr[2] + 1] = sol

        data.qpos[:] = q
        data.qvel[:] = 0.0
        mujoco_forward(model, data)
        return GaitSample(t=t, qpos=q, base_pos=base_pos, base_quat=base_quat,
                          base_rpy_rate=rpy_rate.copy(),
                          base_accel=accel_world.copy(), contacts=contacts,
                          foot_targets=targets)


# ---------------------------------------------------------------------------
# small rotation helpers (kept dependency-free and exact)
# ---------------------------------------------------------------------------

def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def _quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mujoco_forward(model, data):
    import mujoco
    mujoco.mj_forward(model, data)
