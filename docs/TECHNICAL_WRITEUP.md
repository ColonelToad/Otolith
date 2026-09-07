# Otolith — Technical Writeup (through Go2 demo)

*Go2 is the first checkpoint: the deterministic perception stack of a quadruped, live at 500 Hz, with the same hot path that will be ported to Rust, RTL→FPGA and ASIC.*

## 1. Why Otolith, why Go2 first

Every mobile robot needs to know where it is when vision fails — a proprioceptive stack that fuses IMU, joints and contacts into base pose/velocity at control rate, deterministically, with honest covariance. Otolith builds that stack for a simulated Unitree Go2 in MuJoCo and carries the *same* fixed-size math through a measured compute bake-off: C++ on pinned cores → Rust over `iceoryx2` shared memory → the same fixed-point pipeline as RTL via Yosys/nextpnr → LibreLane/SKY130 PPA. The point up to Go2 is to prove the stack is real before the bake-off; the point after is to answer “what belongs in software vs silicon” with data, not slides.

Go2 before G1: quadruped trot is the canonical legged-odometry problem (CAPO arXiv:2602.17393 is the benchmark), 12 DoF vs 23+, contact is unambiguous, and the evaluation (stance feet world-fixed by construction) is clean. G1 humanoid reuses the same sensor layer, fusion and eval — it’s a harder plant, not a rewrite.

No hardware, no `PREEMPT_RT` in WSL2, no RTX for Isaac — so all claims are “deterministic by design” backed by jitter histograms and RMSE vs ground truth.

## 2. System sketch

```
MuJoCo Go2 (1 kHz) ─┬─ IMU gyro+accel (AR(1) bias + white) ─┐
                    ├─ 12 joint encoders (0.002 rad white) ─┼─► OTLG log (288 B rows, 500 Hz) ─► MEKF 15-state ─► ESTM v2 (1936 B rows, P 15×15) ─► eval (RMSE/NEES/fault)
                    └─ foot contacts (4 bool, world-fixed) ─┘                ▲
                ROS 2 edges (rmw_zenoh, BEST_EFFORT) ─────────────────────────┘
Foxglove (Windows :8765) ◄── foxglove_bridge ── /otolith/state_estimate (Odometry) + /tf (world→base) + /robot_description (latched URDF) + Paths/Markers
```

The hot path (`fusion/`) never touches ROS middleware; ROS lives at the edges (`ros2/otolith_fusion`, `ros2/otolith_viz`). Every boundary is a typed, fixed-size struct.

## 3. Puppet — a consistent trot

`sim/otolith_sim/puppet.py:168` prescribes `qpos` directly. Footholds are world-fixed on a stride grid (`stride 0.14 m`, `cycle 0.7 s`, `duty 0.55`), base advances at `stride/cycle = 0.2 m/s` with 8 mm bob + 0.02 rad wobble, swing feet follow `sin(πs)` arcs, per-leg closed-form IK (`leg_ik`) with constants introspected from the model (`hip 0.1934×0.0465`, `a 0.0955`, `L1=L2=0.213`). Consequences: contact flags and leg odometry are true by construction — error is sensor noise, not puppet artifacts (ADR 0002).

`ImuNoise` (`sensors.py:16`): gyro `σ_white 0.01`, accel `0.15`, bias AR(1) `σ_bg 0.001 τ 200 s`, `σ_ba 0.02 τ 300 s`. `EncoderNoise` `σ 0.002`. Verified: IK vs MuJoCo FK ≤2 mm, 3 puppet tests pass, publisher 455 Hz of 500 Hz target.

## 4. MEKF — the hot primitive

`fusion/include/otolith/fusion.hpp:12` — 15-state error-state (`dθ 3, dv 3, dp 3, dbg 3, dba 3`), nominal `q/p/v/bg/ba + P 15×15`, `exp_quat(w·dt)` quaternion, `Φ=I+F·dt` with

```
F = [ -[w]×  0    0  -I   0
      -R[a]× 0    0   0  -R
       0     I    0   0   0 ]
Qd = diag(σ_g²·dt, σ_a²·dt, σ_bg²·dt, σ_ba²·dt)  (σ_g 0.01, σ_a 0.15, σ_bg 1e-5, σ_ba 1e-4)
```

Leg odometry (`fusion.cpp:82`): stance foot world velocity should be zero:

```
h = v + R·(w × r + r_dot) ,   r = FK(qj),  r_dot = (r_k − r_{k−1})/dt
y = −h ,  H = [ −R[w×r]×  I  0  R[r]×  0 ]  stacked over m stance feet,  R = σ_leg²·I
```

`r_dot` is the fix that cut velocity bias from 0.22 m/s to ~0.0001; its encoder-noise amplification (`σ_enc 0.002 → r_dot noise ≈0.28 m/s` at `dt 0.002`) forces `σ_leg 0.3 m/s` (inflated, documented). Joseph form `P = (I−KH)P(I−KH)ᵀ + KRKᵀ`, fixed-size Eigen, no alloc in the predict/update math (the ROS wrapper allocates via `MatrixXd` — noted, later zero-alloc work).

Leg FK (`leg_kin.hpp:10`): `x=−L₁s₂−L₂s₂₃, z=−L₁c₂−L₂c₂₃, y=Rₓ(th₁)·[side·a, z]`, `a 0.0955`, validated 2 mm mean vs MuJoCo.

ADR 0003 records MEKF vs Euler choice (Euler rejected — singularities at G1).

## 5. Logs — typed contracts

OTLG v1 (`sim/otolith_sim/logger.py:1` ↔ `fusion/include/otolith/log.hpp:12`): header `OTLG v1 32 B` (`dt`, `row_bytes 288`) + rows `t, gyro 3, accel 3, qj 12, contacts 4, gt_contacts 4, gt_pos 3, gt_quat 4, gt_vel 3, gt_rpy_rate 3, gt_accel 3`. Little-endian, 8-byte aligned, `P` not in OTLG — that’s offline truth. ESTM v2 (`estimate_log.hpp:9`): header `ESTM v2 32 B` + rows `base 136 B (t,p,quat,v,bg,ba) + P 225·8 =1936 B` row-major `15×15`. v1 still readable.

`record_puppet_log` drives puppet+ sensors in-process (no ROS) → OTLG → `fuse_log` (C++ `read_log` → `FusionEKF` → `write_estimate_v2`) → eval. Cross-lang proven: Python writes OTLG, C++ `log_check` reads it.

## 6. Eval — first honest numbers

`eval/M3_REPORT.md:1` (5 s trot @500 Hz, 2500 rows, `σ_leg 0.3`):

| pos RMSE | vel RMSE | att RMSE | final | drift |
|---|---|---|---|---|
| 0.106 m (x 0.021 y 0.104 z 0.004) | 0.067 m/s | 13.1 deg | 0.207 m | 20.7% of 1.00 m travel |

`y` and yaw dominate — yaw is unobservable with only leg velocity. Perfect sensors (no noise) give 0.006 m pos, 0.02 m/s vel.

`eval/tests/`: `test_scenarios.py:1` (3 seeded 2-s trots + novel `stride 0.18`, bounds `pos<0.25 vel<0.30 drift<50%`), `test_fault.py:1` (dropout 0.2 s, bias jump 0.1 rad/s, stuck encoder — all finite `RMSE<0.6`), `test_nees.py:1` (mean position NEES `0.1–15` for `dof 3`, relaxed for `r_dot` noise), `test_jitter.cpp:1` (5000× predict+update `p50<200µs p99<500µs max<5ms` on WSL — `MatrixXd` still allocates, noted).

## 7. Live path — two-OS co-design

`sim_node.py:1` (rclpy, `BEST_EFFORT` depth 1) publishes `/otolith/imu|joint_states|foot_contacts @500 Hz` + `/otolith/ground_truth @100 Hz` via `rmw_zenoh` (DDS multicast does not survive the WSL boundary). `ros2/otolith_fusion` (`fusion_node.cpp:14`) — `rclcpp` edge node, `dt` from `Imu` header, init `q` from first `Imu.orientation`, `predict` on every `imu`, `update_legs(qj,contacts,gyro,dt)` when both available, publishes `/otolith/state_estimate` (`Odometry` with `pose.covariance` `P[6:9]` + `twist.covariance` `P[3:6]`), 1 s watchdogs for stale `imu/joints/contacts`.

Foxglove: `ros2/otolith_description` (URDF `go2.urdf` + `meshes/*.obj` from Menagerie, latched `/robot_description`) + `robot_state_publisher` (from `joint_states` → `tf` for 12 joints) + `otolith_viz` (`viz_node.py:1`: `world→base` (est) and `world→base_gt` `tf`, `gt_path`/`est_path` `Path`, covariance ellipsoid `MarkerArray` at 5 Hz) → `foxglove_bridge` on `:8765` → Foxglove Studio (Windows) `foxglove/go2_demo.json` (3D URDF + TF + Paths + Markers, Plots for `vx`, State for contacts). Heaviness reduction: URDF meshes are sent once (latched `transient_local`), `tf`+`Path` are throttled to 10–30 Hz (not 500 Hz), `Marker` at 5 Hz; the 500 Hz `state_estimate` stays on `BEST_EFFORT`. Full mesh is shown without per-frame mesh bytes.

`scripts/go2_demo.sh:1` builds (`colcon` with `LIBRARY_PATH` workaround for `lttng-ust`), sources `install/setup.bash`, runs `sim_node &`, `ros2 launch otolith_viz viz_launch.py`, waits, prints `ros2 topic list`, expects `ws://localhost:8765` open.

Hardware (`~/Projects/hardware`): `oss-cad-suite` (Yosys/Verilator/nextpnr/GTKWave) + `iic-osic-tools:2026.07` (LibreLane 3, SKY130/GF180) already verified via `counter` → `GDS` smoke test (`hardware/README.md`). Not used until v0.4 — the Go2 demo is software-only and honest about it.

## 8. What’s next (after Go2)

v0.2 transport bake-off (ROS topics vs `iceoryx2` shared memory vs typed contracts), v0.3 Rust port, v0.4 RTL (same fixed-point pipeline `hdl/` → Yosys → LibreLane PPA), v0.5 G1 reuse. The 5-s vs fault-injection distinction for the video: clean trot is the gating artifact (does it track?); a 10-s second clip with 0.2 s dropout + bias jump is supplemental proof of watchdogs/NEES — keep it separate, not blocking the demo.

