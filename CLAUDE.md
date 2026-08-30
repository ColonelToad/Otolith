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
pixi run python -m pytest sim/tests eval/tests -q       # full pyramid (15 tests)
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
- v0.2: transport bake-off (ROS 2 topics vs shared memory vs contracts)
- v0.3: Rust port (`rust/`, iceoryx2)
- v0.4: RTL port (`hdl/`): Verilator → Yosys; LibreLane + SKY130 PPA study — **tooling ready** in `~/Projects/hardware/` (oss-cad-suite + `iic-osic-tools:2026.07`, counter GDS passed)
- v0.5: humanoid reuse (Unitree G1)

When a phase starts, its directory stops being a placeholder — delete its `.gitkeep` and update this file.
