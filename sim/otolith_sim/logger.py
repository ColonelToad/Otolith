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
