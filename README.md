# Otolith

A deterministic perception stack for legged robots — and a measured answer to the question *"what belongs in software, and what belongs in silicon?"*

A Unitree Go2 quadruped, simulated in MuJoCo, streams biased, noisy IMU, joint-encoder, and foot-contact data at real rates into a contact-aided extended Kalman filter estimating base orientation, velocity, and position at 1kHz. That hot path is implemented several times — C++ on pinned cores, Rust over shared memory, and the same fixed-point pipeline as synthesizable RTL carried through FPGA synthesis and an open ASIC flow — with latency, jitter, error, area, and power compared side by side. The stack is co-designed across operating systems from day one: ROS 2 over zenoh and the simulator run in WSL2 Debian, Foxglove and the observability tooling run native on Windows, and every boundary is a typed contract.

Otolith is the nervous system of a quadruped, built the way trading systems are built: deterministic by design, measured at every seam.

## Pillars

1. **Real-time sensors first** — simulated IMU/encoders/contacts with honest noise models, hard timestamp discipline, fixed-rate fusion. State estimation is the flagship problem, not an afterthought.
2. **Mixture of chips, measured** — the same hot primitive implemented as C++, Rust, and RTL, with CPU / FPGA-target / ASIC-target (OpenLane + SKY130) comparisons. The partitioning argument is made with data.
3. **Two-OS co-design** — Windows owns visualization and observability (Foxglove, native); WSL2 Debian owns the engine (ROS 2 Jazzy via RoboStack, MuJoCo, the pipeline). Boundaries are designed for the crossing, not patched after.

## Status & Phases

| Phase | Scope | Status |
|-------|-------|--------|
| v0.1 | Go2 scene + sensor simulation layer (IMU/joints/contacts, noise models), C++ contact-aided EKF (15-state MEKF, leg FK, `r_dot` fix, σ_leg 0.3), eval harness (RMSE, NEES, fault injection, jitter), Foxglove wiring | **done** — `f95c5b4` log contract, `3d5ee82` MEKF, `4fe2f1d` offline runner (0.106 m pos, 13 deg @5 s), `9fd2432` scenario/NEES/fault/jitter, ROS `otolith_fusion` node + `foxglove_bridge` on `:8765` |
| v0.2 | Transport bake-off: ROS 2 topics vs shared memory vs typed contracts | planned |
| v0.3 | Rust port of the fusion node (iceoryx2 shared memory) | planned |
| v0.4 | RTL port of the hot primitive: Verilator → Yosys; LibreLane + SKY130 PPA study | planned — tooling ready (`~/Projects/hardware/oss-cad-suite` + `iic-osic-tools:2026.07`) |
| v0.5 | Humanoid reuse (Unitree G1): same sensor layer, same fusion, harder plant | planned |

## Architecture (sketch)

```
MuJoCo (Go2, 1kHz physics)
  ├─ IMU gyro+accel (bias + noise)      ─┐
  ├─ 12× joint encoders                 ─┼─► timestamp/align ─► contact-aided EKF ─► base state @1kHz
  └─ foot contacts                      ─┘         (typed structs,        [hot path: C++ → Rust → RTL]
                                                    zero-alloc)
ROS 2 edges (rmw_zenoh): sensor drivers, bridge nodes          │
Foxglove (Windows, native) ◄── foxglove-bridge WebSocket ──────┘
```

The hot path never touches ROS 2 middleware. ROS 2 lives at the edges for ecosystem fluency; the deterministic path is typed shared memory and contracts, the same pattern commercial RT middleware (iceoryx2, HORUS) is built on.

## Layout

```
otolith/
├── sim/        MuJoCo scenes + sensor simulation layer (noise models, rates)
├── fusion/     Contact-aided EKF hot path (C++17, Catch2, zero-alloc)
├── ros2/       ROS 2 wrapper packages (colcon; edges only, never the hot path)
├── eval/       RMSE vs ground truth, jitter histograms, plots
├── rust/       v0.3 iceoryx2 port
├── hdl/        v0.4 RTL: SystemVerilog, testbenches, yosys/OpenLane configs
├── docs/       Design notes and decision records
└── third_party/menagerie → symlink to ../rally/mujoco_menagerie (gitignored)
```

## Environment

Two-OS co-design (see `CLAUDE.md` for agent-level detail and exact commands):

- **WSL2 Debian** — everything that builds or runs: pixi (RoboStack ROS 2 Jazzy, `rmw_zenoh`, `foxglove-bridge`), MuJoCo, C++/Rust/RTL toolchains. The repo lives on the Linux filesystem; builds never touch `/mnt/c`.
- **Windows 11** — Foxglove Studio (native) pointed at `localhost:8765`; VS Code via WSL Remote. Nothing robotics-native is installed here.

## References

- [CAPO: Contact-Anchored Proprioceptive Odometry for Quadruped Robots](https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry) (arXiv:2602.17393) — prior art and benchmark target for the estimator; Otolith's differentiator is the compute-partitioning bake-off, not estimator novelty
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) — Unitree Go2 / G1 models
- [RoboStack](https://robostack.github.io/) — ROS 2 Jazzy on Debian via pixi
- [rmw_zenoh](https://github.com/ros2/rmw_zenoh) — ROS 2's non-DDS middleware (no multicast discovery)
- [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) — shared-memory RT middleware (Rust)
- [OpenLane2](https://github.com/efabless/openlane2) + [SKY130 PDK](https://skywater-pdk.readthedocs.io/) — open RTL-to-GDS flow for the ASIC-target study
