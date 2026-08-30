# 0004 — Eval Harness: Scenarios, NEES, Fault Injection, Jitter

Date: 2026-08-29
Status: Accepted

## Context

M3 shipped the first honest RMSE (5 s trot, 0.106 m pos, 13 deg yaw) but the
evaluation was a single ad-hoc run (`eval/evaluate.py`). The plan requires a
full testing pyramid beyond C++ unit tests: statistical scenario suites,
consistency checks, fault injection, and timing — the same discipline that
makes the filter's claims credible and the later compute bake-off comparable.

The filter now logs covariance (ESTM v2, `estimate_log.hpp:9`) to enable NEES;
the leg velocity measurement's `r_dot` noise dominates (`σ_enc=0.002` →
`r_dot` noise ~0.28 m/s, so `σ_leg` was inflated to 0.3 m/s in 0003's fix).

## Decision

1. **ESTM v2** (`estimate_log.hpp:9`): `ESTM` v2 row `1936 B` = base `136 B` +
   `P` `225*8 B` row-major `15×15`. v1 (`136 B`) still readable (reader
   dispatches on version). `fuse_log` now writes v2 (`fuse_log.cpp:1`) with
   `P` per row; `evaluate.py` reads both.

2. **Metrics module** (`eval/metrics.py:1`): `rmse`, `quat_angle_error_deg`,
   `compute_nees` (`err^T P^{-1} err`), `chi2_bounds`. Shared by all eval
   tests.

3. **Scenario suite** (`eval/tests/test_scenarios.py:1`): parametrized
   `pytest` over seeds `0,1,2` (2 s trot, 500 Hz, `ImuNoise`/`EncoderNoise`
   varying) + a novel gait (`stride=0.18`). Each generates a puppet log via
   `LogWriter`, runs `fuse_log`, reads both logs, asserts `pos RMSE <0.25`,
   `vel <0.30`, final drift `<50%`. Bounds are loose enough for seed variance,
   tight enough to catch regressions.

4. **Fault injection** (`eval/tests/test_fault.py:1`): `test_dropout_mid_run`
   (0.2 s contacts dropped), `test_imu_bias_jump` (0.1 rad/s), `test_stuck_encoder`
   (0.2 s frozen `qj`). Each must not crash, must stay finite, and `RMSE <0.6`.

5. **NEES consistency** (`eval/tests/test_nees.py:1`): mean position NEES
   (`6:9` block) over 800 samples, `0.1 < mean <15` for `dof=3` (chi2
   `0.35–7.81` ideal, relaxed for yaw-unobservable trot and `r_dot` noise).

6. **Jitter** (`fusion/tests/test_jitter.cpp:1`): `5000× predict+update`,
   histogram `p50<200µs`, `p99<500µs`, `max<5ms` on WSL (no `PREEMPT_RT`,
   `MatrixXd` allocates — generous bounds, but the harness now exists for
   the later pinned-core/zero-alloc work).

## Consequences

- `pixi run pytest sim/tests eval/tests -q` runs the full pyramid
  (`7` sim + `8` eval = `15` passed); `ctest --test-dir fusion/build`
  runs `11` tests (including jitter) — all green.
- The harness is the regression gate for v0.2+ (transport), v0.3 (Rust),
  v0.4 (RTL): any change to the filter or log contract breaks the suite.
- Rejected: separate `cov.log` file (extra file to keep in sync), storing
  only diagonal `P` (insufficient for NEES), hard `chi2` pass (too brittle
  for this `r_dot` noise model).
