# 0007 — v0.4 RTL Port: Fixed-Point Predict Pipeline

Date: 2026-09-23
Status: Proposed (M0 kickoff; Accepted at M6 with numbers)

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
3. **Q-format from range analysis** (M0, measured 2026-09-23, 5 s
   seeded trot, post-update states): uniform **Q8.24 storage** (range
   ±128, LSB 6e-8) + **64-bit accumulators** for all matmul/sum
   intermediates, round-half-up on narrowing, saturate on overflow.
   Measured maxabs: q 1.0, p 0.96, v 0.28, bg/ba 0.07, P 0.10,
   F 107, Phi 1.0, w 1.03, a 104, Qd 2e-13..4.5e-5. Q8.24 covers
   everything except the bg/barw Qd blocks (2e-13, 2e-11/step), which
   quantize to zero — accepted provisionally, RMSE impact measured in
   M1 (predicted nil: 2500 steps accumulate 5e-10 against P≈1e-2).
   Caveats locked: F/a near the ±128 rail (saturation counted in the
   error bound); position range caps the operating envelope at ~100 m
   (matches eval scope); update-path ranges deferred to the Cholesky
   stretch, which re-runs this analysis for H/S/K.
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
