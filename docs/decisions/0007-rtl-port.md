# 0007 — v0.4 RTL Port: Fixed-Point Predict Pipeline

Date: 2026-09-23
Status: **Accepted** (M4 kernel GDS + full-core cost; M5 update kernel 9002/9002
and the fully fixed-point filter; see M5 outcome below)

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
0.297 W. M5 also closed the update path's kernel. `hdl/rtl/ldl_kernel.sv` (WIDTH=6,
the dominant real width) factorizes `S` by unpivoted LDL' and solves `S^-1 b`
with one shared 64x64 multiplier and one 128-bit accumulator. Bit-parity is
**9002/9002** against `fixed_update.hpp` on the real innovation covariances
captured from the 10 s trot log, not synthetic matrices. ECP5-85F via nextpnr:
40,337 LUT (48%), 4,396 FF (5%), 0 DSP, 0 BRAM, **Fmax 13.91 MHz** against a
50 MHz constraint (not met), **1,262 cycles = 91 us per factorization+solve**,
which is 22x inside the 2 ms update budget -- so the kernel meets the timing
requirement it actually has while missing the clock target. Two structural
changes were kept only after they moved the measured number and parity was
re-verified: registering the multiplier output (7.36 -> 10.73 MHz) and splitting
the reciprocal rescale's 128-bit barrel shift from its overflow compare
(10.73 -> 13.91 MHz); both were diagnosed from nextpnr's critical-path report
after a first guess proved wrong. **No sky130 GDS for this kernel**: LibreLane's
ABC does not converge (65,860-cell input, killed at 2 h each at 10 ns and
20 ns, memory flat, strategy already `AREA 0`), unlike the mul48 kernel.
## M5 outcome (2026-10-05): the whole filter is fixed point

The measurement model is now fixed point too, so predict, measure and correct
are all fixed and the numerical error is attributable end to end.
`fusion/fixed/fixed_meas.hpp` mirrors `fusion.cpp:71-103` and the `leg_kin.cpp`
Go2 statement for statement: leg geometry in Q8.24, the planar 2R FK, `skew`,
`R*[omega x r]` and `R*[r]` for `H`, and `R*(omega x r + rdot)` for `y` with
`rdot = (r_base - r_prev)/dt`.

Same 10 s trot log, same harness (ESTM-v2, `evaluate.py`):

| | pos RMSE | vel RMSE | att RMSE | saturations |
|---|---|---|---|---|
| float | 0.3334 m | 0.0946 m/s | 24.21 deg | - |
| M1 hybrid (fixed predict, float update) | 0.3083 m | 0.0936 m/s | 22.22 deg | - |
| **M5 fully fixed** | **0.3074 m** | **0.0934 m/s** | **22.19 deg** | **0** |

**Accepted on this basis: the fixed-point update costs essentially nothing over
the float update** (0.3083 -> 0.3074 m, inside run-to-run noise), and
Q8.24/Q16.48 needs no wider format. `H` matches float to 3.9e-07 and `y` to
~4e-04 element-wise from identical states. 4999/4999 updates, 0 non-PD, 0
saturations anywhere.

### The sigma_leg floor

`fuse_update_study --sigma-leg` sweeps the one assumption the pivot floor rests
on. `min|D|` tracks `sigma^2` exactly (`Rmat = sigma^2 I` dominates S's
diagonal); the cliff is between 1.5e-05 and 1.2e-04, which is exactly where
`1/D` leaves the Q16.48 rail (`1/D <= 32768` needs `D >= 3.05e-05`). So the
update requires **sigma_leg >= ~0.0055**, and the shipped 0.3 has ~1000x
margin. Below the cliff it is always the *reciprocal* that saturates, never the
factorization -- non-PD is 0/4999 at sigma = 0.001 too. A tighter noise
assumption needs a wider reciprocal, not a better factorization.

### Three bugs, all found by measurement

1. **The CORDIC's range is smaller than the Go2's knee.** `thigh + calf`
   reaches ~2.8 rad against +/-1.7433 rad convergence, so `cordic_sincos`
   saturated to the rail and returned garbage (44 saturations, 0.14 m of FK
   error). `sin_cos_wide` folds into [-pi/2, pi/2] first; fixed-iteration,
   exact for |theta| <= 3*pi. **This is a robot-specific constraint, not a
   generic precision issue** -- any port of `cordic_sincos` must check its
   input range against the joint limits, not just its precision.
2. **`prev_qj` was overwritten before `r_prev` was read from it**, silently
   zeroing every `rdot` because `r_base == r_prev` exactly.
3. **`mat3_mul_fixed(R, sum, tmp)` was handed a 3-element `sum`.** That helper
   indexes `B[0..8]`; C arrays decay to pointers, so neither the compiler nor a
   reader catches it. It presented as `h[0] = 4.18` against a true -0.014 and
   a wholesale divergence (pos RMSE 119 m). Fixed with a dedicated
   `mat3_vec_fixed`. Lesson recorded because it is invisible by construction:
   the 9-vs-3 argument mismatch is a legal call and the read is only ~2x past
   the end, so it survives review and returns plausible-shaped garbage.

### Consequences of accepting this

- **The update path is portable to RTL, but is not yet in RTL.** `ldl_kernel`
  covers the factorization and the solve; the measurement model, `apply_dx_fixed`
  and the 15x15 Joseph form (54% of the update cost at rows=6, per the M5-C
  study) are model-only. The CORDIC needs no multiplier, so the measurement
  side ports cheaply; the Joseph form is the expensive remaining slice and has
  no parity bench yet.
- **The M5-C cost table is a projection, not a measurement.** It excludes the
  Joseph form and the measurement model, so 10.19 mm² full fixed MEKF should be
  read as a lower bound.
