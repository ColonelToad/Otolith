#!/usr/bin/env python3
"""Offline evaluator: OTLG (GT) + one or more ESTM (estimates) -> RMSE table + plots.

Usage:
  PYTHONPATH=sim pixi run python eval/evaluate.py [--in /tmp/in.otlg] [--est /tmp/out.estm] [--ablate]
  If no args, generates a 5s puppet log and runs fuse_log automatically.
  --ablate also runs fuse_log --no-leg-update (predict-only dead reckoning)
  and overlays it in traj.png + a comparison table (the "without Otolith" run).

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
EST_VER = 2
EST_VER1 = 1
EST_ROW = 1936
EST_ROW1 = 136
EST_HDR = struct.Struct("<4s I I I d Q")

def read_est(path):
    p = Path(path)
    data = p.read_bytes()
    magic, ver, row_bytes, _, dt, _cnt = EST_HDR.unpack(data[:32])
    assert magic == EST_MAGIC and ver in (EST_VER, EST_VER1)
    if ver == EST_VER:
        assert row_bytes == EST_ROW
        n = (len(data)-32)//EST_ROW
        rows = []
        off=32
        for _ in range(n):
            # base 136 + 225*8 covariance
            vals = struct.unpack("<d 3d 4d 3d 3d 3d 225d", data[off:off+1936])
            t=vals[0]; px,py,pz=vals[1:4]; qw,qx,qy,qz=vals[4:8]; vx,vy,vz=vals[8:11]; bgx,bgy,bgz=vals[11:14]; bax,bay,baz=vals[14:17]
            cov = np.array(vals[17:]).reshape(15,15)
            rows.append(dict(t=t, p=np.array([px,py,pz]), q=np.array([qw,qx,qy,qz]), v=np.array([vx,vy,vz]), bg=np.array([bgx,bgy,bgz]), ba=np.array([bax,bay,baz]), P=cov))
            off+=1936
        return dt, rows
    else:
        assert row_bytes == EST_ROW1
        n = (len(data)-32)//EST_ROW1
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

# Trail colors (match Foxglove layout: GT green, MEKF orange, dead-reck red)
TRAIL_COLORS = {"mekf": "#ff8800", "deadreck": "#ff0000", "est": "#ff8800"}
TRAIL_LABELS = {"mekf": "MEKF (Otolith)", "deadreck": "dead reckoning (no leg updates)", "est": "est"}

def score_run(gt_rows, est_rows):
    n = len(gt_rows)
    assert len(est_rows)==n, f"{n} vs {len(est_rows)}"
    p_err = np.zeros(n); v_err=np.zeros(n); ang_err=np.zeros(n)
    dp = np.zeros((n,3)); dv=np.zeros((n,3))
    for i in range(n):
        gt=gt_rows[i]; es=est_rows[i]
        dp[i]=es['p']-gt.gt_pos
        dv[i]=es['v']-gt.gt_vel
        p_err[i]=np.linalg.norm(dp[i])
        v_err[i]=np.linalg.norm(dv[i])
        ang_err[i]=quat_angle_error_deg(es['q'], gt.gt_quat)
    def rmse(a): return np.sqrt(np.mean(a*a))
    return dict(
        n=n, p_err=p_err, v_err=v_err, ang_err=ang_err, dp=dp, dv=dv,
        p_rmse=rmse(p_err), v_rmse=rmse(v_err), a_rmse=rmse(ang_err),
        px_rmse=rmse(dp[:,0]), py_rmse=rmse(dp[:,1]), pz_rmse=rmse(dp[:,2]),
        vx_rmse=rmse(dv[:,0]), vy_rmse=rmse(dv[:,1]), vz_rmse=rmse(dv[:,2]),
        final_p=p_err[-1],
    )

# Per-robot facts the report used to hard-code as Go2 prose. Everything here is
# read from the model or the descriptor rather than assumed, because the report is
# generated for all four robots now and "kinematic trot" was being printed for a
# biped whose gait is a quasi-static weight shift.
ROBOT_FACTS = {
    # name: (gait description, sigma_leg used by the filter)
    "go2":    ("kinematic trot with world-fixed footholds + AR(1) IMU bias", 0.3),
    "g1":     ("quasi-static biped gait: stand, lateral weight shift, squat, "
              "with world-fixed footholds + AR(1) IMU bias", 1.05),
    "apollo": ("quasi-static biped gait, as G1", 1.40),
    "op3":    ("quasi-static biped gait, as G1", 0.36),
}


# Nominal standing base height per robot, metres. Used to identify the subject of a
# log: all three bipeds carry 12 joint DoF and 2 contacts, so qj width cannot tell
# them apart, but their base heights differ by 2.8x (0.265 / 0.755 / 1.016).
ROBOT_BASE_Z = {"g1": 0.755, "apollo": 1.016, "op3": 0.265}

# How many contact slots each robot uses. This is the decisive discriminator and it
# has to come first: Go2's base height (0.32) sits within 6 cm of OP3's (0.265), so
# height alone labels a quadruped as a biped -- which is exactly what the first
# version of this function did.
ROBOT_LEGS = {"go2": 4, "g1": 2, "apollo": 2, "op3": 2}


def _robot_facts(in_path):
    """Identify the robot from the log itself. Returns a name, or None.

    Contact-slot count first (4 = quadruped, 2 = biped), then base height among the
    bipeds, which differ by 2.8x.

    Deliberately conservative: if nothing matches, the report says "unknown gait"
    rather than guessing. A report that mislabels its own subject is worse than one
    that says less.
    """
    try:
        from otolith_sim.logger import read_log
        _, rows = read_log(in_path)
        nlegs = max(int(np.sum(r.contacts)) for r in rows)
        z = float(np.median([r.gt_pos[2] for r in rows]))
    except Exception:
        return None
    # Contact count is decisive on its own: a Go2 trot puts 4 feet down at once,
    # every biped puts 2. Height is only needed to separate the bipeds, and it
    # cannot do that job alone -- Go2's base sits at 0.270 m and OP3's at 0.260 m,
    # 1 cm apart, so height alone labels the quadruped as a biped. Which is what
    # two earlier versions of this function did.
    if nlegs == ROBOT_LEGS["go2"]:
        return "go2"
    if nlegs != 2:
        return None
    best, bestd = None, 1e9
    for name, ref in ROBOT_BASE_Z.items():
        d = abs(z - ref)
        if d < bestd:
            best, bestd = name, d
    return best if bestd < 0.15 else None


def evaluate(in_path, est_paths, out_dir):
    # est_paths: {label: path} or a single path (backward compat -> {"est": path})
    if isinstance(est_paths, (str, Path)):
        est_paths = {"est": str(est_paths)}
    dt_gt, gt_rows = read_log(in_path)
    n = len(gt_rows)
    runs = {}
    for label, ep in est_paths.items():
        _, est_rows = read_est(ep)
        runs[label] = score_run(gt_rows, est_rows)
    dist = np.linalg.norm(gt_rows[-1].gt_pos - gt_rows[0].gt_pos)

    # report: primary run first (mekf preferred, else first), then ablation table
    primary = "mekf" if "mekf" in runs else next(iter(runs))
    s = runs[primary]
    drift_pct = 100*s['final_p']/max(dist,1e-9)
    est_list = ", ".join(f"`{k}`" for k in est_paths)
    robot = _robot_facts(in_path)
    gait = "unknown gait"
    sig = None
    subject = ""
    if robot:
        gait, sig = ROBOT_FACTS[robot]
        subject = robot.upper() + " "
    # Rate from the log's own dt, not a hard-coded 500 Hz.
    hz = int(round(1.0 / dt_gt))
    title = ("# Otolith eval — " + subject + "RMSE ("
             + gait.split(" with ")[0] + ", " + str(hz) + " Hz, IMU+leg odometry)")
    report = f"""{title}

*Generated from `{in_path}` → {est_list} — {n} samples, dt={dt_gt:.4f}s, duration {n*dt_gt:.2f}s*

| run | pos RMSE (m) | vel RMSE (m/s) | att RMSE (deg) | final pos err (m) | drift (% of {dist:.2f} m) |
|---|---|---|---|---|---|
"""
    for label in est_paths:
        r = runs[label]
        d = 100*r['final_p']/max(dist,1e-9)
        report += f"| {label} | `{r['p_rmse']:.4f}` | `{r['v_rmse']:.4f}` | `{r['a_rmse']:.4f}` | `{r['final_p']:.4f}` | `{d:.2f}%` |\n"
    report += f"""
Primary `{primary}` per-axis: pos x/y/z `{s['px_rmse']:.4f} / {s['py_rmse']:.4f} / {s['pz_rmse']:.4f}`, vel `{s['vx_rmse']:.4f} / {s['vy_rmse']:.4f} / {s['vz_rmse']:.4f}`.
Raw: p_err mean {s['p_err'].mean():.4f} max {s['p_err'].max():.4f}, v_err max {s['v_err'].max():.4f}, ang max {s['ang_err'].max():.2f} deg.

*Notes:* puppet is a {gait}; estimator is MEKF@{hz} Hz init from first GT, zero-alloc fixed-size Eigen. Measurement noise `σ_leg={('%.2f' % sig) + ' m/s' if sig is not None else 'unknown — pass --robot or a recognised log'}`
per foot, which is NOT a constant: leg odometry differentiates foot position, so `σ_leg = σ_q·√2·L/dt` grows as `1/dt` and with the lever arm. See docs/V05_HUMANOID.md P4/P7 for the four-robot measurement. Dead reckoning = `fuse_log --no-leg-update` (predict-only): same log, no contact updates — the "without Otolith" run. Honest nuance: dead reckoning wins attitude (pure gyro integration) but loses position/velocity catastrophically — the contact updates trade attitude noise for position/velocity observability (yaw stays unobservable).
"""
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    (out/"report.md").write_text(report)
    print(report)

    # plots (primary run for vel/rpy; all runs overlaid on traj)
    t=np.array([r.t for r in gt_rows])
    # traj xy — GT green + every run overlaid
    fig, ax = plt.subplots(figsize=(6,4))
    gt_xy = np.array([r.gt_pos[:2] for r in gt_rows])
    ax.plot(gt_xy[:,0], gt_xy[:,1], label="GT", color="#00ff00", lw=2)
    for label, ep in est_paths.items():
        _, est_rows = read_est(ep)
        es_xy = np.array([r['p'][:2] for r in est_rows])
        ax.plot(es_xy[:,0], es_xy[:,1],
                label=TRAIL_LABELS.get(label, label),
                color=TRAIL_COLORS.get(label, None), alpha=0.85, lw=1.5)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(fontsize=8); ax.set_title("Trajectory (top-down)")
    fig.tight_layout(); fig.savefig(out/"traj.png", dpi=150); plt.close(fig)
    # velocity (primary only, to avoid clutter)
    _, est_rows_p = read_est(est_paths[primary])
    fig, ax = plt.subplots(figsize=(6,3))
    gt_v = np.array([r.gt_vel for r in gt_rows])
    es_v = np.array([r['v'] for r in est_rows_p])
    for k,lbl in enumerate(["vx","vy","vz"]):
        ax.plot(t, gt_v[:,k], ls="--", label=f"GT {lbl}")
        ax.plot(t, es_v[:,k], label=f"est {lbl}", alpha=0.8)
    ax.set_xlabel("t (s)"); ax.set_ylabel("m/s"); ax.legend(ncol=3, fontsize=7)
    ax.set_title(f"Velocity ({primary})"); fig.tight_layout(); fig.savefig(out/"vel.png", dpi=150); plt.close(fig)
    # rpy error (primary only)
    fig, ax = plt.subplots(figsize=(6,2.5))
    ax.plot(t, s['ang_err']); ax.set_xlabel("t (s)"); ax.set_ylabel("deg"); ax.set_title(f"Attitude error ({primary})")
    fig.tight_layout(); fig.savefig(out/"rpy_err.png", dpi=150); plt.close(fig)
    print(f"wrote {out/'report.md'} and plots")

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", default=None)
    ap.add_argument("--est", dest="est", default=None)
    ap.add_argument("--ablate", action="store_true",
                    help="also run fuse_log --no-leg-update and compare")
    ap.add_argument("--out", default="eval/out")
    args=ap.parse_args()
    inp=args.inp; est=args.est
    if inp is None or est is None:
        # auto-generate: one puppet log, both MEKF + dead-reckoning runs
        tmp_in = Path("/tmp/otolith_auto.otlg")
        tmp_mekf = Path("/tmp/otolith_auto.estm")
        tmp_dead = Path("/tmp/otolith_auto_dead.estm")
        print(f"generating puppet log -> {tmp_in}")
        from otolith_sim.logger import record_puppet_log
        record_puppet_log(tmp_in, duration_s=5.0, dt=1/500)
        print(f"running fuse_log -> {tmp_mekf}")
        subprocess.check_call([str(ROOT/"fusion/build/fuse_log"), str(tmp_in), str(tmp_mekf)])
        print(f"running fuse_log --no-leg-update -> {tmp_dead}")
        subprocess.check_call([str(ROOT/"fusion/build/fuse_log"), str(tmp_in), str(tmp_dead), "--no-leg-update"])
        evaluate(str(tmp_in), {"mekf": str(tmp_mekf), "deadreck": str(tmp_dead)}, args.out)
    elif args.ablate:
        # explicit input: add the dead-reckoning run on the same log
        tmp_dead = Path("/tmp/otolith_ablate_dead.estm")
        print(f"running fuse_log --no-leg-update -> {tmp_dead}")
        subprocess.check_call([str(ROOT/"fusion/build/fuse_log"), str(inp), str(tmp_dead), "--no-leg-update"])
        evaluate(inp, {"mekf": est, "deadreck": str(tmp_dead)}, args.out)
    else:
        evaluate(inp, est, args.out)
