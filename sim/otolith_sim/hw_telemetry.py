"""Hardware-utilisation telemetry for the demo's bottom-right panel.

WHAT THIS IS, PRECISELY
=======================
A readout of MEASURED silicon figures plus a DERIVED cycle estimate. It is not a
live Verilator co-simulation -- that is the follow-on, and the topic field `live`
below is false until it lands. Do not present this panel as a measurement of the
running filter's silicon cost; it is the cost we already measured, per robot.

The split matters because the two halves of the demo answer different questions:

  top-right    /otolith/perf      what the SOFTWARE is doing right now
  bottom-right /otolith/perf_hw   what the SILICON would cost to do it

So the left half of the comparison is live and the right half is a cost model, and
the panel says so.

THE NUMBERS
============
Measured, from hdl/M5_UPDATE_STUDY.md and ADR-0007:

  predict_core (P <- Phi P Phi^T + Qd, 15x15)
      ECP5 40,337 LUT (48%), 0 DSP, Fmax 65.8 MHz
  ldl_kernel (LDL' factor + solve)
      bit-parity 9002/9002 on real captured covariances
      ECP5 Fmax 13.91 MHz, 91 us per factorisation = 22x inside the 2 ms budget
  mul48_kernel
      0.119 mm^2 SKY130, DRC/antenna clean
  full fixed-point filter
      0.3072 m vs 0.3083 m float-update, 0 saturations

WHY ROWS MATTER
===============
The update's cost is dominated by the 15x15x15 Joseph product, which is
width-INDEPENDENT, plus terms that scale with the number of stance feet. A biped
has 2, a trot has 4, and the update table runs 12,500 MACs at rows=6 against 22,390
at rows=12 -- so a biped is the CHEAPER case and the panel must not imply parity.
That is also the honest reason the silicon/software partition probably survives
bipeds: the dominant term is robot-independent.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))

# Measured, not estimated. See the module docstring.
PPA = {
    "predict_lut": 40337,
    "predict_lut_pct": 48.0,
    "predict_dsp": 0,
    "predict_fmax_mhz": 65.8,
    "ldl_fmax_mhz": 13.91,
    "ldl_us": 91.0,
    "mul48_mm2": 0.119,
    "fixed_pos_m": 0.3072,
    "float_pos_m": 0.3083,
}

# MAC counts per stance row from M5_UPDATE_STUDY.md. rows=6 is two stance feet,
# rows=12 is four.
UPDATE_MACS = {3: 12500 // 2, 6: 12500, 9: 16833, 12: 22390}
JOSEPH_SHARE = 0.54          # width-independent 15x15x15 APA^T at rows=6

# Per-robot descriptor facts, matching leg_kin.cpp's SigmaLegModel.
ROBOTS = {
    "go2":    dict(legs=4, dof=3, sigma_leg=0.3546, deadline_us=2000.0),
    "g1":     dict(legs=2, dof=6, sigma_leg=1.0793, deadline_us=2000.0),
    "apollo": dict(legs=2, dof=6, sigma_leg=1.3995, deadline_us=2000.0),
    "op3":    dict(legs=2, dof=6, sigma_leg=0.1571, deadline_us=2000.0),
}


def estimate(rows: int, deadline_us: float) -> dict:
    """Cycle/area estimate for one update step at `rows` measurement rows."""
    macs = UPDATE_MACS.get(rows, 12500)
    # 1 MAC per cycle is the optimistic end; the reported figure is deliberately
    # labelled as a lower bound in the panel title.
    cycles = macs
    us = cycles / PPA["ldl_fmax_mhz"]
    return dict(macs=float(macs), cycles=float(cycles), us=us,
                frac_deadline=us / deadline_us,
                joseph_macs=float(macs * JOSEPH_SHARE))


def main() -> int:
    import rclpy
    from rclpy.node import Node
    # std_msgs, not sensor_msgs. The C++ side made the same mistake and the
    # compiler said so; in Python it is an ImportError at runtime instead.
    from std_msgs.msg import Float64MultiArray

    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="g1", choices=sorted(ROBOTS))
    ap.add_argument("--rate", type=float, default=1.0,
                    help="publish rate Hz; this is a static cost model, 1 Hz is plenty")
    args = ap.parse_args()
    spec = ROBOTS[args.robot]
    # Two stance feet for a biped (3 rows each), four for a trot.
    rows = 3 * spec["legs"]
    est = estimate(rows, spec["deadline_us"])

    class HwNode(Node):
        def __init__(self):
            super().__init__("otolith_hw_telemetry")
            self.pub = self.create_publisher(Float64MultiArray, "/otolith/perf_hw", 10)
            self.t = 0.0
            self.timer = self.create_timer(1.0 / args.rate, self.tick)
            self.get_logger().info(
                f"hw telemetry: robot={args.robot} rows={rows} "
                f"update={est['macs']:.0f} MAC ~{est['us']:.1f} us "
                f"({100 * est['frac_deadline']:.1f}% of a {spec['deadline_us']:.0f} us "
                f"budget) | predict_core ECP5 {PPA['predict_lut']} LUT "
                f"({PPA['predict_lut_pct']:.0f}%), Fmax {PPA['predict_fmax_mhz']} MHz. "
                "STATIC cost model, not a live co-simulation.")

        def tick(self):
            m = Float64MultiArray()
            # Indices mirror /otolith/perf where they overlap, so the two panels can
            # share a mental model; indices 8+ are hardware-only.
            m.data = [
                PPA["predict_fmax_mhz"],          # 0 predict Fmax MHz
                PPA["ldl_fmax_mhz"],               # 1 update-kernel Fmax MHz
                PPA["ldl_us"],                     # 2 ldl us per factorisation
                spec["deadline_us"],               # 3 budget us
                est["frac_deadline"],              # 4 update as fraction of budget
                float(rows),                      # 5 measurement rows
                spec["sigma_leg"],                # 6 sigma_leg in use
                float(spec["legs"]),               # 7 legs
                est["macs"],                       # 8 update MACs
                est["joseph_macs"],                # 9 of which Joseph (width-indep)
                PPA["predict_lut"],                # 10 predict LUT
                PPA["predict_lut_pct"],            # 11 predict LUT %
                PPA["predict_dsp"],                # 12 predict DSP
                PPA["mul48_mm2"],                  # 13 mul48 mm^2
                PPA["fixed_pos_m"],                # 14 fully fixed-point pos RMSE
                PPA["float_pos_m"],                # 15 float-update pos RMSE
                0.0,                               # 16 live co-simulation? NO
            ]
            self.pub.publish(m)

    rclpy.init()
    rclpy.spin(HwNode())
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())