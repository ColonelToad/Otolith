"""Record a log and an estimate for all four robots, for before/after comparison.

Kept separate from `robot_metrics.py` so the logs are regenerated on demand but the
metrics can be recomputed from existing artefacts without re-running the puppets
(which take minutes each, mostly the IK).

Usage:  pixi run python eval/make_logs.py
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))

FUSE = str(ROOT / "fusion" / "build" / "fuse_log")

# (name, log path, recorder, fuse_log args)
PLAN = [
    ("go2", "/tmp/m_go2.otlg", "go2", []),
    ("g1", "/tmp/m_g1.otlg", "g1", ["--robot", "g1", "--sigma-leg-from-robot"]),
    ("apollo", "/tmp/m_apollo.otlg", "apollo",
     ["--robot", "apollo", "--sigma-leg-from-robot"]),
    ("op3", "/tmp/m_op3.otlg", "op3", ["--robot", "op3", "--sigma-leg-from-robot"]),
]


def main():
    from otolith_sim.logger import record_apollo_log, record_g1_log, record_op3_log, \
        record_puppet_log
    rec = {
        "go2": lambda p: record_puppet_log(p, duration_s=6.0, dt=1 / 500),
        "g1": lambda p: record_g1_log(p, duration_s=6.0, dt=1 / 500),
        "apollo": lambda p: record_apollo_log(p, duration_s=6.0, dt=1 / 500),
        "op3": lambda p: record_op3_log(p, duration_s=6.0, dt=1 / 500),
    }
    for name, log, kind, args in PLAN:
        print(f"  recording {name} ...", flush=True)
        rec[kind](log)
        est = f"/tmp/m_{name}.est"
        r = subprocess.run([FUSE, log, est] + args, capture_output=True, text=True)
        if r.returncode != 0:
            print(f"  {name}: fuse_log failed: {r.stdout}{r.stderr}", file=sys.stderr)
            return 1
        print(f"    {r.stdout.strip()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())