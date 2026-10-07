"""Binary log contract for Otolith sensor + ground-truth streams.

Format (little-endian, x86):
  Header (32 B):
    magic      : 4 B  b"OTLG"
    version    : u32  (1)
    row_bytes  : u32  (288)
    reserved   : u32  (0)
    dt         : f64  sim step (s)
    row_count  : u64  derived as (file_size - 32) // 288 by readers;
                     writer patches this field on close if possible,
                     but readers must not rely on it (compute from size).

  Row (288 B, packed, 8-byte aligned):
    t              f64       sim time (s)
    gyro[3]        f64[3]    measured gyro body (rad/s)
    accel[3]       f64[3]    measured accel body (m/s^2, gravity included)
    qj[12]         f64[12]   measured joints FL/RRL hip,thigh,calf (rad)
    contacts[4]    u8[4]     measured contacts FL,FR,RL,RR (0/1)
    gt_contacts[4] u8[4]     ground-truth contacts (same in v0.1; kept distinct)
    gt_pos[3]      f64[3]    GT base pos world (m)
    gt_quat[4]     f64[4]    GT base quat world wxyz
    gt_vel[3]      f64[3]    GT base lin vel world (m/s)
    gt_rpy_rate[3] f64[3]    GT rpy rates (rad/s, same as gyro truth pre-bias)
    gt_accel[3]    f64[3]    GT base accel world (m/s^2)

All floats are LE. Integers are raw bytes. No padding beyond explicit fields
except the natural 8-byte alignment is preserved by the field order.

Python writer/reader uses struct with explicit offsets; C++ uses
#pragma pack(push,1) with matching layout and static_assert(sizeof==288).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAGIC = b"OTLG"
VERSION = 1
HEADER_SIZE = 32
ROW_BYTES = 288

# little-endian struct formats
_HEADER_FMT = "<4s I I I d Q"  # magic, version, row_bytes, reserved, dt, row_count
assert struct.calcsize(_HEADER_FMT) == HEADER_SIZE

_ROW_FMT = "<d 3d 3d 12d 4B 4B 3d 4d 3d 3d 3d"
assert struct.calcsize(_ROW_FMT) == ROW_BYTES

ORDER = ("FL", "FR", "RL", "RR")


@dataclass
class LogRow:
    t: float
    gyro: np.ndarray       # (3,)
    accel: np.ndarray      # (3,)
    qj: np.ndarray         # (12,)
    contacts: np.ndarray   # (4,) uint8
    gt_contacts: np.ndarray
    gt_pos: np.ndarray     # (3,)
    gt_quat: np.ndarray    # (4,) wxyz
    gt_vel: np.ndarray     # (3,)
    gt_rpy_rate: np.ndarray
    gt_accel: np.ndarray


def write_header(f, dt: float, row_count: int = 0):
    f.write(struct.pack(_HEADER_FMT, MAGIC, VERSION, ROW_BYTES, 0, float(dt), int(row_count)))


def _pack_row(row: LogRow) -> bytes:
    return struct.pack(
        _ROW_FMT,
        float(row.t),
        *map(float, row.gyro), *map(float, row.accel), *map(float, row.qj),
        *map(int, row.contacts), *map(int, row.gt_contacts),
        *map(float, row.gt_pos), *map(float, row.gt_quat),
        *map(float, row.gt_vel), *map(float, row.gt_rpy_rate), *map(float, row.gt_accel),
    )


def _unpack_row(buf: bytes) -> LogRow:
    vals = struct.unpack(_ROW_FMT, buf)
    # vals layout: t, gyro0..2, accel0..2, qj0..11, c0..3, gtc0..3, pos0..2, quat0..3, vel0..2, rpy0..2, acc0..2
    off = 0
    t = vals[off]; off += 1
    gyro = np.array(vals[off:off+3]); off += 3
    accel = np.array(vals[off:off+3]); off += 3
    qj = np.array(vals[off:off+12]); off += 12
    contacts = np.array(vals[off:off+4], dtype=np.uint8); off += 4
    gt_contacts = np.array(vals[off:off+4], dtype=np.uint8); off += 4
    gt_pos = np.array(vals[off:off+3]); off += 3
    gt_quat = np.array(vals[off:off+4]); off += 4
    gt_vel = np.array(vals[off:off+3]); off += 3
    gt_rpy_rate = np.array(vals[off:off+3]); off += 3
    gt_accel = np.array(vals[off:off+3]); off += 3
    return LogRow(t=t, gyro=gyro, accel=accel, qj=qj,
                  contacts=contacts, gt_contacts=gt_contacts,
                  gt_pos=gt_pos, gt_quat=gt_quat,
                  gt_vel=gt_vel, gt_rpy_rate=gt_rpy_rate, gt_accel=gt_accel)


class LogWriter:
    """Streaming writer: patches row_count on close."""

    def __init__(self, path: str | Path, dt: float):
        self.path = Path(path)
        self.dt = float(dt)
        self._f = open(self.path, "wb")
        write_header(self._f, self.dt, 0)
        self.count = 0

    def write(self, row: LogRow):
        self._f.write(_pack_row(row))
        self.count += 1

    def close(self):
        if self._f is None:
            return
        self._f.flush()
        # patch row_count at offset 24 (after magic+version+row_bytes+reserved+dt)
        try:
            self._f.seek(24)
            self._f.write(struct.pack("<Q", self.count))
        except Exception:
            pass
        self._f.close()
        self._f = None  # type: ignore

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


def read_log(path: str | Path):
    """Return (dt, list[LogRow]). Validates magic/version/row_bytes."""
    p = Path(path)
    data = p.read_bytes()
    if len(data) < HEADER_SIZE:
        raise ValueError(f"file too short: {len(data)}")
    magic, version, row_bytes, _res, dt, _count = struct.unpack(_HEADER_FMT, data[:HEADER_SIZE])
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic!r}")
    if version != VERSION:
        raise ValueError(f"unsupported version {version}")
    if row_bytes != ROW_BYTES:
        raise ValueError(f"row_bytes mismatch {row_bytes} != {ROW_BYTES}")
    # derive count from file size (header may be stale if writer crashed)
    n = (len(data) - HEADER_SIZE) // ROW_BYTES
    trunc = (len(data) - HEADER_SIZE) % ROW_BYTES
    if trunc != 0:
        raise ValueError(f"truncated row: {trunc} leftover bytes")
    rows = []
    off = HEADER_SIZE
    for _ in range(n):
        rows.append(_unpack_row(data[off:off+ROW_BYTES]))
        off += ROW_BYTES
    return dt, rows


def record_puppet_log(path: str | Path, duration_s: float = 2.0, dt: float = 1.0/500,
                      seed_imu: int = 0, seed_enc: int = 1):
    """Generate a log by driving the puppet + sensors in-process (no ROS).

    Useful for eval harness dev; writes `duration_s` seconds at 1/dt.
    """
    import mujoco
    from otolith_sim.puppet import Go2Puppet, GaitConfig, _quat_to_mat, GRAVITY
    from otolith_sim.sensors import ImuNoise, EncoderNoise, contacts_exact

    model = mujoco.MjModel.from_xml_path("third_party/menagerie/unitree_go2/scene.xml")
    data = mujoco.MjData(model)
    puppet = Go2Puppet(model, GaitConfig())
    imu = ImuNoise(seed=seed_imu)
    enc = EncoderNoise(seed=seed_enc)

    order = ORDER
    n = int(duration_s / dt)

    with LogWriter(path, dt) as w:
        t = 0.0
        for _ in range(n):
            sample = puppet.sample(model, data, t, dt)
            R = _quat_to_mat(sample.base_quat)
            # truth in body frame (same math as sim_node.py)
            gyro_truth = sample.base_rpy_rate.copy()
            accel_truth = R.T @ (sample.base_accel + np.array([0.0, 0.0, GRAVITY]))
            gyro_m, accel_m = imu.step(dt, gyro_truth, accel_truth)
            q_true = np.array([sample.qpos[adr]
                               for leg in order
                               for adr in puppet.legs[leg].qpos_adr])
            qj_m = enc.step(q_true)
            # vel: finite diff already in puppet's _prev_pos; recompute here for log clarity
            # derive gt_vel from gt_pos delta against previous row (keep simple: use 0 for row 0, else delta)
            # Instead use puppet's velocity if available; fallback to 0
            # We'll compute incremental below.
            contacts = contacts_exact(sample.contacts).astype(np.uint8)
            # carry vel from previous sample's derived velocity (puppet stores _prev_vel)
            gt_vel = getattr(puppet, "_prev_vel", np.zeros(3)).copy()
            row = LogRow(
                t=t,
                gyro=gyro_m, accel=accel_m, qj=qj_m,
                contacts=contacts, gt_contacts=contacts.copy(),
                gt_pos=sample.base_pos.copy(),
                gt_quat=sample.base_quat.copy(),
                gt_vel=gt_vel,
                gt_rpy_rate=sample.base_rpy_rate.copy(),
                gt_accel=sample.base_accel.copy(),
            )
            w.write(row)
            t += dt


def record_biped_log(path: str | Path, scene: str | Path | None = None,
                     puppet_factory=None, duration_s: float = 6.0,
                     dt: float = 1.0 / 500, seed_imu: int = 0, seed_enc: int = 1,
                     sigma_enc: float | None = None):
    """Generate a log from any 6-DoF biped puppet, in the same OTLG format.

    The format is already robot-agnostic: 12 joint DoF and 4 contact slots fit a
    biped exactly (2 legs x 6 DoF = 12; only 2 slots used). Nothing about the
    on-disk layout changes, which is why the C++ filter needed no log changes for
    v0.5 -- only `robot_spec(name)` and a stride that comes from the descriptor
    instead of the literal 3.

    Contact slots 0 and 1 are left and right; 2 and 3 stay zero. That is a
    convention, not a coincidence, so it is stated here rather than left to the
    reader: `sigma_study --robot <biped>` relies on it.
    """
    import mujoco
    from otolith_sim.sensors import ImuNoise, EncoderNoise
    from otolith_sim.puppet import GRAVITY
    from otolith_sim.biped_puppet import _quat_to_mat

    if puppet_factory is None:
        from otolith_sim.g1_puppet import G1Puppet, G1GaitConfig
        scene = scene or ".work/g1scene/scene.xml"

        def puppet_factory(model):
            return G1Puppet(model, cfg=G1GaitConfig())
    model = mujoco.MjModel.from_xml_path(str(scene))
    puppet = puppet_factory(model)
    imu = ImuNoise(seed=seed_imu)
    # sigma_enc=None keeps the simulator default (0.002 rad), which is what G1 and
    # Apollo were measured at. OP3 has real hardware with much finer encoders, so it
    # needs both: the 0.002 figure to be comparable with the other two, and the
    # Dynamixel figure to test the lever-arm prediction on the robot it was made for.
    enc = EncoderNoise(seed=seed_enc) if sigma_enc is None else \
        EncoderNoise(seed=seed_enc, sigma=sigma_enc)
    n = int(duration_s / dt)

    with LogWriter(path, dt) as w:
        t = 0.0
        for _ in range(n):
            s = puppet.sample(t, dt)
            R = _quat_to_mat(s.base_quat)
            gyro_truth = s.base_rpy_rate.copy()
            accel_truth = R.T @ (s.base_accel + np.array([0.0, 0.0, GRAVITY]))
            gyro_m, accel_m = imu.step(dt, gyro_truth, accel_truth)
            q_true = np.array([s.qpos[model.jnt_qposadr[model.joint(j).id]]
                               for leg in puppet.lm.legs
                               for j in puppet.lm.chain(leg).joints])
            qj_m = enc.step(q_true)
            contacts = np.zeros(4, dtype=np.uint8)
            contacts[:2] = s.contacts.astype(np.uint8)
            # sample() updates _prev_vel with this step's finite difference, so
            # it is read after the call. The sample carries no vel field.
            vel_world = puppet._prev_vel.copy()
            w.write(LogRow(
                t=t, gyro=gyro_m, accel=accel_m, qj=qj_m,
                contacts=contacts, gt_contacts=contacts.copy(),
                gt_pos=s.base_pos, gt_quat=s.base_quat,
                gt_vel=vel_world, gt_rpy_rate=s.base_rpy_rate,
                gt_accel=s.base_accel))
            t += dt


def record_g1_log(path, scene=None, duration_s=6.0, dt=1.0 / 500,
                  seed_imu=0, seed_enc=1):
    """G1 convenience wrapper. See record_biped_log."""
    return record_biped_log(path, scene or ".work/g1scene/scene.xml", None,
                            duration_s, dt, seed_imu, seed_enc)


# OP3's real actuator: DYNAMIXEL XM430-W350, 4096 counts over 360 deg, so
# 0.0879 deg = 0.001534 rad per tick. Quantisation noise on a uniform distribution
# of width q is q/sqrt(12), giving sigma_q = 0.000443 rad -- 4.5x FINER than the
# 0.002 rad the simulator assumes for every other robot. That is what makes OP3 the
# one robot whose sigma_leg could FALSIFY the lever-arm argument rather than merely
# extend it: its lever is short AND its encoder is good, so the two effects push the
# same way and the prediction can fail on its own terms.
OP3_ENCODER_SIGMA = 0.001534 / np.sqrt(12.0)


def record_op3_log(path: str | Path, duration_s: float = 6.0, dt: float = 1.0 / 500,
                   seed_imu: int = 0, seed_enc: int = 1,
                   sigma_enc: float | None = None,
                   scene: str | Path = ".work/op3scene/scene.xml"):
    """OP3 convenience wrapper. `sigma_enc` defaults to the simulator's 0.002 rad;
    pass OP3_ENCODER_SIGMA for the real Dynamixel figure."""
    from otolith_sim.op3_puppet import OP3Puppet, OP3GaitConfig

    return record_biped_log(
        path, scene,
        lambda model: OP3Puppet(model, cfg=OP3GaitConfig()),
        duration_s, dt, seed_imu, seed_enc, sigma_enc)


def record_apollo_log(path: str | Path, duration_s: float = 6.0, dt: float = 1.0 / 500,
                      seed_imu: int = 0, seed_enc: int = 1,
                      scene: str | Path = "third_party/menagerie/apptronik_apollo/scene.xml"):
    """Apollo convenience wrapper over the same recorder."""
    from otolith_sim.apollo_puppet import ApolloPuppet, ApolloGaitConfig

    return record_biped_log(
        path, scene,
        lambda model: ApolloPuppet(model, cfg=ApolloGaitConfig()),
        duration_s, dt, seed_imu, seed_enc)
