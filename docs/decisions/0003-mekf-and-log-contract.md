# 0003 — MEKF Fusion Core and Log Contract

Date: 2026-08-29
Status: Accepted

## Context

M1 defined a typed, fixed-size binary log (OTLG v1, 288 B rows) shared between
Python and C++. M2 needs the contact-aided estimator that consumes it.

Options for the filter:

- **Euler-angle EKF**: simpler math, singularities at pitch ±90° (fine for flat-ground trot) but not credible for v0.5 humanoid and not comparable to CAPO/invariant literature.
- **Error-state MEKF (15-dim)**: quaternion nominal + 3-dim attitude error, industry-standard singularity-free, ports cleanly to fixed-point RTL, and matches the bibliography (CAPO, invariant-EKF). More math up front.

## Decision

1. **15-state error-state MEKF** (`fusion/include/otolith/fusion.hpp:12`):
   error `[dtheta(3), dv(3), dp(3), dbg(3), dba(3)]`, nominal `q/p/v/bg/ba + P(15×15)`.
   IMU predict at sensor rate (~500 Hz, reported honestly; 1 kHz target is a bake-off metric, not a puppet requirement). Stance-foot leg-odometry update: foot velocity in world = `v + R*(omega x r)` should be zero; residual `-h`, Jacobian `[-R[omega x r]_x, I, 0, R[r]_x, 0]`, stacked over `m` stance feet with Joseph-form covariance update. Gravity `9.81` explicit, `exp_quat(w*dt)` for quaternion, Phi=`I+F*dt` first-order.

2. **Leg FK** (`fusion/include/otolith/leg_kin.hpp:10`): closed-form `foot = hip_base + [-L1 s2 -L2 s23, R_x(th1)*[side*a, z_in]]`, constants introspected from the Go2 model (hip 0.1934×0.0465, `a=0.0955`, `L1=0.213`, `L2=0.213009…`). Validated against MuJoCo FK to 2 mm mean, round-trips through the Python `leg_ik` (see tests).

3. **Library split**: `otolith_leg` (FK), `otolith_log` (M1), `otolith_fusion` (MEKF) — each header-only or small `.cpp`, plain CMake, pixi-owned Eigen 5 / Catch2 3. No ROS in the hot path.

## Consequences

- `fusion/tests/test_fusion.cpp` asserts: predict grows covariance but stays PSD, constant-yaw integrates correctly, leg update reduces trace, 500-step run stays finite/PSD, zero-stance is no-op.
- Fixed-size Eigen types only (`Vector3d`, `Matrix<double,15,15>`), no allocation in predict/update — ready for zero-alloc and jitter harnesses in M4.
- The MEKF math is the artifact ported to Rust and RTL; Euler would have been throwaway.
- Rejected Euler path remains documented for readers expecting the simpler start, with rationale tied to v0.5 reuse.
