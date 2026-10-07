"""Attitude/position metrics per robot, for before/after comparison.

Written because the v0.5 attitude-Jacobian fix had to be judged, and judging it
needs a number recorded BEFORE the change. `eval/evaluate.py` reports a single
scalar attitude RMSE, which is not enough: the Jacobian block under test is the
attitude one, and roll in particular is the axis that diverged on G1. A single
blended scalar would hide exactly the thing being measured.

So this reports roll/pitch/yaw separately, because:
  * roll is the axis the wrong block controls, and the one that diverged;
  * yaw is unobservable in every configuration here, so lumping it in dilutes the
    other two and can make a change look neutral when it is not;
  * the three robots differ in stance width by 4x, so they do not respond alike.

Usage:  pixi run python eval/robot_metrics.py [out.json]
"""
from __future__ import annotations

import json
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))

ROBOTS = {
    # name: (log, fuse_log args)
    "go2": ("/tmp/m_go2.otlg", []),
    "g1": ("/tmp/m_g1.otlg", ["--robot", "g1", "--sigma-leg-from-robot"]),
    "apollo": ("/tmp/m_apollo.otlg", ["--robot", "apollo", "--sigma-leg-from-robot"]),
    "op3": ("/tmp/m_op3.otlg", ["--robot", "op3", "--sigma-leg-from-robot"]),
}


def read_est(path):
    """(rows, 15) of [t, p(3), quat(4) wxyz, v(3)] straight from the .est file."""
    d = Path(path).read_bytes()
    _m, _v, row_bytes, _r, _dt, count = struct.unpack_from("<4sIIIdQ", d, 0)
    a = np.frombuffer(d[32:32 + count * row_bytes], dtype=np.float64)
    a = a.reshape(count, -1)
    return a[:, 0], a[:, 1:4], a[:, 4:8], a[:, 8:11]


def qmul(A, B):
    return np.stack([
        A[:, 0] * B[:, 0] - A[:, 1] * B[:, 1] - A[:, 2] * B[:, 2] - A[:, 3] * B[:, 3],
        A[:, 0] * B[:, 1] + A[:, 1] * B[:, 0] + A[:, 2] * B[:, 3] - A[:, 3] * B[:, 2],
        A[:, 0] * B[:, 2] - A[:, 1] * B[:, 3] + A[:, 2] * B[:, 0] + A[:, 3] * B[:, 1],
        A[:, 0] * B[:, 3] + A[:, 1] * B[:, 2] - A[:, 2] * B[:, 1] + A[:, 3] * B[:, 0]], 1)


def qconj(Q):
    return np.stack([Q[:, 0], -Q[:, 1], -Q[:, 2], -Q[:, 3]], 1)


def rpy_err(q_est, q_gt):
    d = qmul(qconj(q_gt), q_est)
    w, x, y, z = d.T
    return np.degrees(np.stack([
        np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y + z * z)),
        np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
        np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z + x * x))], 1))


def rmse(a):
    return float(np.sqrt(np.mean(np.asarray(a) ** 2)))


def metrics_for(est_path, log_path):
    from otolith_sim.logger import read_log
    _dt, rows = read_log(log_path)
    _, pos, quat, vel = read_est(est_path)
    n = min(len(rows), len(pos))
    gt_pos = np.array([r.gt_pos for r in rows[:n]])
    gt_quat = np.array([r.gt_quat for r in rows[:n]])
    gt_vel = np.array([r.gt_vel for r in rows[:n]])
    e = rpy_err(quat[:n], gt_quat)
    return {
        "n": n,
        "att_rmse_rpy_deg": [round(rmse(e[:, i]), 4) for i in range(3)],
        "att_final_rpy_deg": [round(float(e[-1, i]), 3) for i in range(3)],
        "att_max_abs_deg": round(float(np.abs(e).max()), 3),
        "pos_rmse_xyz_m": [round(rmse(pos[:n, i] - gt_pos[:, i]), 4) for i in range(3)],
        "vel_rmse_xyz_mps": [round(rmse(vel[:n, i] - gt_vel[:, i]), 4) for i in range(3)],
    }


def main():
    out = {}
    for name, (log, args) in ROBOTS.items():
        est = f"/tmp/m_{name}.est"
        if not (Path(log).exists() and Path(est).exists()):
            print(f"  {name:7s} SKIP (missing {log if not Path(log).exists() else est})")
            continue
        out[name] = metrics_for(est, log)
        md5 = subprocess.run(["md5sum", est], capture_output=True, text=True).stdout.split()[0]
        out[name]["est_md5"] = md5
    if not out:
        print("no logs found; run eval/make_logs.py first", file=sys.stderr)
        return 2
    hdr = (f"{'robot':7s} {'att r/p/y (deg)':>26s} {'final r/p/y':>22s} "
           f"{'pos x/y/z (m)':>24s} {'vel x/y/z (m/s)':>24s}")
    print(hdr)
    print("-" * len(hdr))
    for name, m in out.items():
        print(f"{name:7s} {str(m['att_rmse_rpy_deg']):>26s} "
              f"{str(m['att_final_rpy_deg']):>22s} "
              f"{str(m['pos_rmse_xyz_m']):>24s} {str(m['vel_rmse_xyz_mps']):>24s}")
    print()
    for name, m in out.items():
        print(f"  {name:7s} est md5 {m['est_md5']}")
    dest = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/robot_metrics.json")
    dest.write_text(json.dumps(out, indent=2))
    print(f"\nwrote {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())