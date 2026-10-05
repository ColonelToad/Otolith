"""Emit v0.6 FEA load cases from the sim's prescribed kinematics.

    pixi run python mech/emit_loads.py --duration 3.5 --out mech/out/loads.json

What comes out is what a link FEA actually needs:

  * the per-foot time series, so a transient/dynamic stress run can be driven
    by the real signal rather than a guessed ramp;
  * per-foot peak force, for a static allow/ultimate check;
  * per-stance impulse, which is the fatigue input -- mean and spread, because
    a fatigue life estimate needs the distribution, not the peak.

Deliberately NOT written into the OTLG `LogRow`. That struct is 288 B and
ADR-0005 pins the transport bake-off (and ADR-0006's Rust `BenchMsg`) to that
size. Contact force is a mechanical-analysis channel that no estimator reads,
so growing the estimator's input contract to carry it would invalidate two
Accepted ADRs' numbers for nothing. This is a sidecar.

Frame note: forces are world +z only. A real link FEA also wants shear and the
joint reaction distribution; see docs/V06_LOAD_CASES.md "Still open".
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "sim")

MENAGERIE = "third_party/menagerie/unitree_go2/scene.xml"
FOOT_NAMES = ("FL", "FR", "RL", "RR")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=3.5,
                    help="seconds of gait (default 3.5 = 5 cycles at cycle_s=0.7)")
    ap.add_argument("--out", type=Path, default=None,
                    help="write JSON here; omit to print the summary only")
    ap.add_argument("--model", default=MENAGERIE)
    ap.add_argument("--smooth-steps", type=int, default=None,
                    help="Savitzky-Golay half-window in steps (default 18)")
    args = ap.parse_args()

    import mujoco
    from otolith_sim.grf import record_grf, load_summary

    model_path = Path(args.model)
    if not model_path.exists():
        sys.exit(f"{model_path} missing -- see CLAUDE.md for the menagerie symlink")
    model = mujoco.MjModel.from_xml_path(str(model_path))

    kwargs = {"duration_s": args.duration}
    if args.smooth_steps is not None:
        kwargs["smooth_steps"] = args.smooth_steps
    series = record_grf(model, **kwargs)
    summary = load_summary(series)

    print(f"mass {summary['mass_kg']:.4f} kg   weight {summary['weight_N']:.3f} N")
    print(f"conservation: mean GRF/W = {summary['mean_fz_over_weight']:.6f}  (gate: 1.0)")
    print(f"peak {summary['peak_fz_N']:.2f} N = {summary['peak_fz_over_weight']:.3f} W   "
          f"rms {summary['rms_fz_N']:.2f} N   duty {summary['duty_measured']:.4f}")
    print("\nper-foot peak normal force (N):")
    for name, v in zip(FOOT_NAMES, summary["per_foot_peak_N"]):
        print(f"  {name}: {v:7.2f}")
    print("\nper-stance vertical impulse (N.s) -- the fatigue input:")
    for name, runs in zip(FOOT_NAMES, summary["per_foot_stance_impulse_Ns"]):
        if not runs:
            print(f"  {name}: (never in stance)")
            continue
        a = np.array(runs)
        print(f"  {name}: {len(runs)} runs  mean {a.mean():6.3f}  "
              f"min {a.min():6.3f}  max {a.max():6.3f}  sd {a.std():6.3f}")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "provenance": {
                "source": "otolith_sim.grf -- CoM momentum balance on prescribed "
                          "kinematics (NOT MuJoCo's contact solver)",
                "method": "F_z_total(t) = mass * (g + a_com_z(t)); per-foot split "
                          "is equal across stance feet by diagonal symmetry",
                "model": str(model_path),
                "duration_s": args.duration,
                "smooth_steps": series.smooth_steps,
                "smoothing": "Savitzky-Golay local quadratic, second derivative",
                "frame": "world, +z up, newtons",
                "assumption": "equal split across stance feet; exact for the "
                              "FL+RR / FR+RL diagonal pairs of this trot only",
            },
            "summary": summary,
            "t": series.t.tolist(),
            "fz_total": series.fz_total.tolist(),
            "fz_feet": {n: series.fz_feet[:, i].tolist()
                        for i, n in enumerate(FOOT_NAMES)},
            "stance": {n: series.stance[:, i].astype(int).tolist()
                       for i, n in enumerate(FOOT_NAMES)},
        }
        args.out.write_text(json.dumps(payload))
        print(f"\nwrote {args.out} ({args.out.stat().st_size/1024:.1f} KiB)")


if __name__ == "__main__":
    main()