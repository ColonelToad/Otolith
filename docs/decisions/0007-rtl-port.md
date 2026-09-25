# 0007 — v0.4 RTL Port: Fixed-Point Predict Pipeline

Date: 2026-09-23
Status: M4-closed (kernel GDS + full-core cost; Cholesky/docs M5–M6 open)

## Context

ADR-0001§5 promises the EKF hot path "as the same fixed-point pipeline
as RTL" with metrics latency/jitter/RMSE/area/power. A full 15-state
MEKF in RTL (15×15 matmuls + up-to-12×12 `S.inverse()`) is not one
phase — the inverse alone is a Cholesky+solve project. v0.4 scopes to
the slice that is still honestly the hot path: the per-step predict
pipeline, which runs every IMU sample (update runs only on stance).

Tooling is verified live, not assumed: oss-cad-suite (Yosys 0.68,
Verilator 5.051, nextpnr) on PATH; `docker` daemon responds and
`iic-osic-tools:2026.07` (15.9 GB) is pulled — the counter→GDS smoke
flow applies with known gotchas (pin PDK + `STD_CELL_LIBRARY`, absolute
die area). PPA runs in-phase, not as a stretch.

## Decision

1. **Primitive**: predict-step covariance pipeline `P ← ΦPΦᵀ + Qd`
   (15×15) + quaternion `exp`/`normalize` glue. No division, no
   inverse; MAC-array friendly; runs every sample — the hot loop core.
2. **Model language C++**: shortest trust path (shares headers with
   `fusion.cpp`, in-process float-vs-fixed differential, zero new
   toolchain). Rust explicitly non-goal for v0.4 (revisit only if the
   RTL needs a driver harness — not foreseen).
3. **Formats from range analysis + hybrid divergence (M0–M1)**:
   heterogeneous, not uniform. States/Phi/inputs stay Q8.24 (±128,
   LSB 6e-8); covariance P is Q16.48 (LSB 3.6e-15, rail 32768) with
   128-bit accumulators (overflow-freedom proved: products ≤ 2^80,
   ×15 terms ≤ 2^84 << 2^127). Reason, measured: P spans ~1e-9
   (gain-structural correlations) to ~2 (predict-only growth) = 11
   decades — no 32-bit format covers both. Q8.24 destroys
   sub-1e-7 correlations → Joseph gains go wrong → hybrid RMSE 3.87 m
   vs 0.106 m; x256 block-scale rails at 0.5 on growth. With Q16.48
   P: hybrid 0.1023/0.0722/12.18 vs float 0.1064/0.0674/13.11 (same
   trajectory, corr 0.999984, ±7% — quantization + linearization
   wander, no structural divergence). M0 range table: q 1.0, p 0.96,
   v 0.28, bg/ba 0.07, P 0.10 fused, F 107, Phi 1.0, w 1.03, a 104,
   Qd 2e-13..4.5e-5 (all Qd blocks now exactly representable — the
   M0 provisional zeroing is revisited away). Rounding: half-away
   everywhere; overflow saturates and counts (accumulators saturating,
   products exact). F/a near the ±128 rail noted; position range caps
   the envelope at ~100 m; update-path ranges deferred to Cholesky.
4. **Parity anchor**: Verilator testbench asserts **bit-parity vs the
   C++ model**; float RMSE is the *error-bound* reference
   (quantization impact on the 0.1064 m baseline becomes a measured
   number — ADR-0001's "error" metric).
5. **Determinism finding**: fixed cycle count per step is expected —
   the jitter column for silicon is "none by construction", itself a
   result against the CPU histograms.
6. **PPA via the proven counter flow**: headless `iic-osic-tools`,
   sky130A + `sky130_fd_sc_hd` pinned, absolute die area; reports area,
   critical path, power.
7. **Cholesky stretch**: update-step `S.inverse()` (Cholesky +
   triangular solve) starts only on predict-early-close, same
   model→RTL→parity pattern.

## Consequences

- `hdl/` stops being a placeholder (`rtl/`, `tb/`, `synth/`,
  `openlane/`); phase docs move v0.4 to in-progress.
- The C++ fixed-point model lives beside the float path
  (`fusion/fixed/`, test/model-only) — never inside the deterministic
  loop; bit-parity bench + error bound join the standing anchors.
- `fusion/` float, the v0.2 bench, and `rust/` are frozen reference
  points.
- Rejected: full-MEKF RTL in one phase (inverse smuggles a second
  project inside), leg-FK as the primitive (synthesizes easier but is
  not the hot path), assumed Q-format (range analysis is cheap,
  re-spins are not), board bring-up (nextpnr numbers suffice per
  ADR-0001 consequences), Rust model (cross-language differential +
  new deps for no measured gain at this stage).

## M4 outcome (2026-09-25): kernel GDS closed, full core costed

Full-core SKY130 is a measured dead end, not a failure to report
around: 297,819 cells / 3.40 mm² vs 2.53 mm² core = 134% util
(unplaceable), and the pre-PnR STA reporting (`-slack_max -0.01` ×
`group_path_count 1000` × `full_clock_expanded`) OOMs the 12 GB box
on the all-violated netlist. The −8402 ns pre-repair slack was a
buffering artifact (one min-drive mux on a fanout-2979 net; the path
is 188 cells and hold was clean), but area alone kills the config.
Kept as the headline cost estimate: the fixed-point predict core is
~3.4 mm² of `sky130_fd_sc_hd`.

The closed loop runs on `mul48_kernel` (registered `s_mul48`,
Q8.24×Q16.48→Q16.48 — the dominant operator in QPROD/P/VPSEQ/ISQ):
2228-vector bit-parity PASS vs `fusion/fixed/`; LibreLane P&R clean;
magic GDS 21.9 MB with klayout XOR 0 diffs; route DRC 0; antenna 0;
hold MET all corners. Setup @10 ns: TT −4.86 / SS −18.1 / FF +0.38 ns
→ honest Fmax ~67 MHz TT / ~36 MHz SS. Area 0.119 mm² @19.3% util,
0.297 W. M5 (Cholesky) and M6 (docs close-out) remain.
