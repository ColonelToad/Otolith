#!/usr/bin/env python3
"""Offline evaluator: OTLG (GT) + ESTM (estimates) -> RMSE table + plots.

Usage:
  PYTHONPATH=sim pixi run python eval/evaluate.py [--in /tmp/in.otlg] [--est /tmp/out.estm]
  If no args, generates a 5s puppet log and runs fuse_log automatically.

Outputs to eval/out/: report.md, traj.png, vel.png, rpy_err.png
"""

from __future__ import annotations
import argparse, struct, subprocess, sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "sim"))

from otolith_sim.logger import read_log

EST_MAGIC = b"ESTM"
EST_VER = 1
EST_ROW = 136
EST_HDR = struct.Struct("<4s I I I d Q")

def read_est(path):
    p = Path(path)
    data = p.read_bytes()
    magic, ver, row_bytes, _, dt, _cnt = EST_HDR.unpack(data[:32])
    assert magic == EST_MAGIC and ver == EST_VER and row_bytes == EST_ROW
    n = (len(data)-32)//EST_ROW
    rows = []
    off=32
    for _ in range(n):
        t, px,py,pz, qw,qx,qy,qz, vx,vy,vz, bgx,bgy,bgz, bax,bay,baz = struct.unpack("<d 3d 4d 3d 3d 3d", data[off:off+136])
        rows.append(dict(t=t, p=np.array([px,py,pz]), q=np.array([qw,qx,qy,qz]), v=np.array([vx,vy,vz]), bg=np.array([bgx,bgy,bgz]), ba=np.array([bax,bay,baz])))
        off+=136
    return dt, rows

def quat_angle_error_deg(q_est, q_gt):
    # q are wxyz
    # error = q_gt^{-1} * q_est ; angle = 2*acos(|w|)
    # conjugate of gt: [-x,-y,-z,w] normalized
    def qmul(a,b):
        aw,ax,ay,az=a
        bw,bx,by,bz=b
        return np.array([aw*bw - ax*bx - ay*by - az*bz,
                         aw*bx + ax*bw + ay*bz - az*by,
                         aw*by - ax*bz + ay*bw + az*bx,
                         aw*bz + ax*by - ay*bx + az*bw])
    def qconj(q):
        w,x,y,z=q
        return np.array([w,-x,-y,-z])
    err = qmul(qconj(q_gt), q_est)
    err = err/np.linalg.norm(err)
    angle = 2*np.arccos(np.clip(abs(err[0]), -1, 1))
    return np.degrees(angle)

def evaluate(in_path, est_path, out_dir):
    dt_gt, gt_rows = read_log(in_path)
    dt_est, est_rows = read_est(est_path)
    assert len(gt_rows)==len(est_rows), f"{len(gt_rows)} vs {len(est_rows)}"
    n=len(gt_rows)
    # optionally skip first 0.2s warmup for metrics (but we init from GT so keep all)
    p_err = np.zeros(n); v_err=np.zeros(n); ang_err=np.zeros(n)
    dp = np.zeros((n,3)); dv=np.zeros((n,3))
    for i in range(n):
        gt=gt_rows[i]; es=est_rows[i]
        dp[i]=es['p']-gt.gt_pos
        dv[i]=es['v']-gt.gt_vel
        p_err[i]=np.linalg.norm(dp[i])
        v_err[i]=np.linalg.norm(dv[i])
        ang_err[i]=quat_angle_error_deg(es['q'], gt.gt_quat)

    # RMSE
    def rmse(a): return np.sqrt(np.mean(a*a))
    p_rmse=rmse(p_err); v_rmse=rmse(v_err); a_rmse=rmse(ang_err)
    # per-axis
    px_rmse, py_rmse, pz_rmse = [rmse(dp[:,k]) for k in range(3)]
    vx_rmse, vy_rmse, vz_rmse = [rmse(dv[:,k]) for k in range(3)]
    final_p = p_err[-1]
    dist = np.linalg.norm(gt_rows[-1].gt_pos - gt_rows[0].gt_pos)
    drift_pct = 100*final_p/max(dist,1e-9)

    # jitter not measured here (offline), report dt

    report = f"""# Otolith M3 — first honest RMSE (trot, 500 Hz, IMU+leg odometry)

*Generated from `{in_path}` → `{est_path}` — {n} samples, dt={dt_gt:.4f}s, duration {n*dt_gt:.2f}s*

| metric | RMSE | per-axis (x / y / z) |
|---|---|---|
| position (m) | `{p_rmse:.4f}` | `{px_rmse:.4f} / {py_rmse:.4f} / {pz_rmse:.4f}` |
| velocity (m/s) | `{v_rmse:.4f}` | `{vx_rmse:.4f} / {vy_rmse:.4f} / {vz_rmse:.4f}` |
| attitude (deg) | `{a_rmse:.4f}` | — |
| final pos err | `{final_p:.4f} m` | drift `{drift_pct:.2f}%` of travel ({dist:.2f} m) |

Raw: p_err mean {p_err.mean():.4f} max {p_err.max():.4f}, v_err max {v_err.max():.4f}, ang max {ang_err.max():.2f} deg.

*Notes:* puppet is kinematic trot with world-fixed footholds + AR(1) IMU bias; estimator is MEKF@500 Hz init from first GT, zero-alloc fixed-size Eigen. Measurement noise `σ_leg=0.3 m/s` per foot (inflated to cover encoder-noise-amplified r_dot via finite difference). This is the baseline to beat — y drift and yaw remain the weak axes.
"""
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    (out/"report.md").write_text(report)
    print(report)

    # plots
    t=np.array([r.t for r in gt_rows])
    # traj xy
    fig, ax = plt.subplots(figsize=(6,4))
    gt_xy = np.array([r.gt_pos[:2] for r in gt_rows])
    es_xy = np.array([r['p'][:2] for r in est_rows])
    ax.plot(gt_xy[:,0], gt_xy[:,1], label="GT")
    ax.plot(es_xy[:,0], es_xy[:,1], label="est", alpha=0.8)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(); ax.set_title("Trajectory (top-down)")
    fig.tight_layout(); fig.savefig(out/"traj.png", dpi=150); plt.close(fig)
    # velocity
    fig, ax = plt.subplots(figsize=(6,3))
    gt_v = np.array([r.gt_vel for r in gt_rows])
    es_v = np.array([r['v'] for r in est_rows])
    for k,lbl in enumerate(["vx","vy","vz"]):
        ax.plot(t, gt_v[:,k], ls="--", label=f"GT {lbl}")
        ax.plot(t, es_v[:,k], label=f"est {lbl}", alpha=0.8)
    ax.set_xlabel("t (s)"); ax.set_ylabel("m/s"); ax.legend(ncol=3, fontsize=7)
    ax.set_title("Velocity"); fig.tight_layout(); fig.savefig(out/"vel.png", dpi=150); plt.close(fig)
    # rpy error = ang_err
    fig, ax = plt.subplots(figsize=(6,2.5))
    ax.plot(t, ang_err); ax.set_xlabel("t (s)"); ax.set_ylabel("deg"); ax.set_title("Attitude error")
    fig.tight_layout(); fig.savefig(out/"rpy_err.png", dpi=150); plt.close(fig)
    print(f"wrote {out/'report.md'} and plots")

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", default=None)
    ap.add_argument("--est", dest="est", default=None)
    ap.add_argument("--out", default="eval/out")
    args=ap.parse_args()
    inp=args.inp; est=args.est
    if inp is None or est is None:
        # auto-generate
        tmp_in = Path("/tmp/otolith_auto.otlg")
        tmp_est = Path("/tmp/otolith_auto.estm")
        print(f"generating puppet log -> {tmp_in}")
        from otolith_sim.logger import record_puppet_log
        record_puppet_log(tmp_in, duration_s=5.0, dt=1/500)
        print(f"running fuse_log -> {tmp_est}")
        subprocess.check_call([str(ROOT/"fusion/build/fuse_log"), str(tmp_in), str(tmp_est)])
        inp, est = str(tmp_in), str(tmp_est)
    evaluate(inp, est, args.out)
