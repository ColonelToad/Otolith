# CLAUDE.md

Instructions for AI agents working in this repository.

---

## What Otolith Is

A deterministic perception stack for a simulated Unitree Go2 quadruped: contact-aided EKF state estimation at 1kHz from simulated IMU/joint/contact sensors, with the hot path implemented across compute targets (C++, Rust, RTL→FPGA→ASIC-model) and measured at every seam. Full goals and phases live in `README.md`; architecture decisions live in `docs/decisions/`.

## Environment (WSL2 Debian — read before running anything)

- **Everything builds and runs inside WSL2 Debian via pixi.** ROS 2 Jazzy is installed through RoboStack (`pixi.toml`), never apt, never Windows.
- Activate with `pixi shell` (or prefix commands with `pixi run …`). ROS 2 environment comes from the pixi env — do not source `/opt/ros/*`.
- Default RMW is `rmw_zenoh_cpp` (set `RMW_IMPLEMENTATION=rmw_zenoh_cpp` if a tool ignores pixi env). Do not use DDS multicast discovery — it does not survive the Windows boundary.
- **The repo lives on the Linux filesystem** (`~/Projects/otolith`). Never build from `/mnt/c`, never create a second CMake cache from Windows. Windows-side tooling (Foxglove Studio, VS Code Remote-WSL) only *reads*.
- MuJoCo Menagerie is **not** vendored: it is symlinked at `third_party/menagerie` → `../rally/mujoco_menagerie` (gitignored). If missing: `mkdir -p third_party && ln -s ../../rally/mujoco_menagerie third_party/menagerie`. Scene XMLs include models with paths relative to the scene file.
- Foxglove Studio (Windows) connects to `ws://localhost:8765` via `foxglove-bridge`. That WebSocket is the only sanctioned WSL↔Windows crossing — no DDS over the boundary, no multicast.

## Commands

```bash
pixi shell                                              # activate env
PYTHONPATH=sim pixi run python -m otolith_sim.sim_node  # sensor sim (puppet) -> /otolith/imu|joint_states|foot_contacts
pixi run python -m pytest sim/tests eval/tests mech/tests -q       # full pyramid (41 + 17 skipped)
pixi run -e cad python -m pytest mech/tests -q                    # CAD+FEA gates (35), needs cadquery
ctest --test-dir fusion/build                           # fusion unit + jitter (11 tests)
# ROS edge (needs LIBRARY_PATH for lttng-ust from pixi):
LIBRARY_PATH=$CONDA_PREFIX/lib:$LIBRARY_PATH pixi run colcon build --packages-select otolith_fusion --cmake-args -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
source install/setup.bash; ros2 run otolith_fusion fusion_node  # -> /otolith/state_estimate (+ watchdogs)
ros2 launch otolith_fusion otolith_launch.py            # fusion + foxglove_bridge :8765
PYTHONPATH=sim pixi run python eval/evaluate.py         # offline 5 s trot -> eval/out/report.md + plots
./fusion/build/fuse_log /tmp/in.otlg /tmp/out.estm       # offline runner (needs fusion/build)
```

## Conventions (carried from Rally — non-negotiable in the hot path)

1. **Typed, fixed-size structs at every boundary.** No JSON, no untyped blobs, no allocation in the EKF/loop path.
2. **Zero dynamic allocation in deterministic loops.** Fixed-size Eigen types, pre-allocated buffers; ASan/UBSan clean.
3. **Deterministic-by-design, measured honestly.** Busy-spin + pinned cores + jitter telemetry; WSL2 has no PREEMPT_RT, so claims are backed by histograms, not adjectives.
4. **Sim-first with honest noise.** Sensors are ground truth + explicit bias/noise models; the eval harness scores estimates against sim ground truth (RMSE) rather than eyeballing.
5. **Every failure mode gets a watchdog** (frame health, stale sensors, estimator divergence).
6. **Architecture decisions get an ADR** in `docs/decisions/NNNN-*.md` — short, dated, with the rejected alternatives.
7. **ROS 2 is an edge technology.** The hot path is typed shared memory/contracts. If a change pushes ROS middleware into the deterministic path, stop and reconsider.

## Phase Map (what exists vs what's planned)

- v0.1: **done** — `sim/` puppet+sensors (455 Hz), `fusion/` 15-state MEKF+leg FK+r_dot (silicon-ready, σ_leg 0.3), `eval/` offline runner + 5 s RMSE 0.106 m + scenario/NEES/fault/jitter harness, `ros2/otolith_fusion` edge node + `foxglove_bridge` on `:8765` (live smoke passed)
- v0.2: transport bake-off DONE (ADR-0005 Accepted) — C ring wins (p50 ~0.5µs, 0 drops); ROS/zenoh stays edge; iceoryx2 measured (p50 ~4-8µs, generality tax). Fixture: `fusion/bench/` + `run_bench.py`, results gitignored in `eval/out/bench-*/`.
- v0.3: Rust port DONE (ADR-0006 Accepted) — `rust/` workspace (`otolith-transport` Miri-gated ring + iceoryx2 backends, `otolith-fusion` forbid-unsafe MEKF), 1e-9 differential, 1.3e-13 parity, 6-contender matrix (C > E > D > B ≈ F > A). Pixi-managed Rust toolchain.
- v0.4: **done** (ADR-0007 Accepted) — predict core: fixed-point pipeline (P←ΦPΦᵀ+Qd + quat glue), C++ model, Q8.24+Q16.48 locked by range analysis, Verilator bit-parity 5000/5000, ECP5 Fmax 65.8 MHz, mul48 kernel GDS (0.119 mm², DRC/antenna clean). Full-core SKY130 = 3.40 mm² = 134% util → documented dead end. Update path: `ldl_kernel` (LDL' factor+solve) bit-parity **9002/9002** on real captured covariances, ECP5 40,337 LUT (48%), 0 DSP, Fmax 13.91 MHz, 91 µs = 22× inside the 2 ms budget; SKY130 ABC does not converge on this netlist, so ECP5 PPA is a **substituted number, not a passed check**. **Fully fixed-point filter 0.3074 m pos vs 0.3083 m float-update, 0 saturations**; σ_leg floor ≈0.0055 (~1000× margin at 0.3). `hdl/` (`rtl/ tb/ synth/ openlane/`), report `hdl/M5_UPDATE_STUDY.md`.
- v0.4 option B (**not** pursued, declined on measured cost): the update path outside `ldl_kernel` is model-only — measurement model (`cordic_sincos`, needs `sin_cos_wide` folding), `apply_dx_fixed`, and the 15×15 Joseph form (54% of update MACs at rows=6). Reason: ~10.19 mm² complete fixed MEKF = 3× a predict core already unplaceable in a 1600×1600 die. See C5 in `hdl/M5_UPDATE_STUDY.md`. The 10.19 mm² figure **excludes** those three — read it as a lower bound.
- v0.5: humanoid reuse (Unitree G1) — **not started**. Highest-information next step: the `sin_cos_wide` bug (Go2 knee reaches ~2.8 rad vs CORDIC's ±1.7433 rad convergence) is direct evidence that per-robot constants are where the wrong assumptions live, so a biped with different geometry/stance-width/biases is a real falsification test.
- v0.6: mechanical (CAD + FEA) — **parked**, and CAD is the half with room. **All 3 prerequisites closed 2026-10-05** (`docs/V06_PREREQS_CLOSED.md`): L2 provenance resolved (my flag was wrong — it is the foot contact sphere centre, `hypot(0.002,0.213)`, bit-identical to MuJoCo); contact forces via exact CoM momentum balance (conservation 0.998661 W, peak 1.238 W) after MuJoCo's contact solver failed three ways; materials recorded and wired into the FEA harness. **Still open: the mass budget** — model 15.2064 kg vs published ~12.4 kg (+22.6%), a 1.2263x multiplier on every load case. Readiness: PCB has **no** prerequisites in this repo (no BOM, MCU, IMU part, power budget, rail or interface spec anywhere — nothing in the sim/fusion stack justifies a custom board today); CAD/FEA is ready to start after prerequisite work. Prereqs: (1) resolve the unrecorded `L2` geometry provenance in `fusion/src/leg_kin.cpp`, (2) log **contact forces** from the sim — the OTLG records `contacts` as binary flags only, so FEA currently has no load cases and no quantity a mechanical phase needs has ever been measured, (3) record material properties. Then: the sim's calf collision is a `0.1065 × 0.01225 × 0.017` m **box** (a slab), so current contact/friction behaviour is set by a placeholder. Tooling verified on this box — cadquery 2.8 / build123d 0.9 / gmsh 4.15 / CalculiX 2.23 via conda-forge in-pixi; KiCad 9.0.2 / ngspice 44.2 / FreeCAD 1.0 / OpenSCAD 2021.01 apt.

When a phase starts, its directory stops being a placeholder — delete its `.gitkeep` and update this file.
