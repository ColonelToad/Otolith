# 0001 — Project Foundation

Date: 2026-08-24
Status: Accepted

## Context

Otolith is the successor to Rally (dual-arm robotic table tennis). Rally proved
the value of: typed fixed-size contracts at process boundaries, deterministic
busy-spin loops with jitter telemetry, watchdogs for every observed failure
mode, sim-first validation, and a deliberate Windows/WSL co-design split. It
also surfaced hard constraints: WSL2 has no PREEMPT_RT, DDS multicast discovery
does not survive the Windows boundary, GPU-accelerated simulators (Isaac) need
hardware this project does not have, and llama/RT middleware ecosystems have
real thread-safety traps.

## Decision

1. **Embodiment**: Unitree Go2 quadruped in MuJoCo (model already available via
   the local Menagerie clone reused from Rally). Humanoid (Unitree G1) is a
   later phase reusing the same sensor/fusion layers. Wheeled was rejected:
   different hiring track (SLAM/nav) and weaker fit for the compute story.
2. **Flagship problem**: contact-aided proprioceptive state estimation
   (IMU + joint encoders + foot contacts → base orientation/velocity/position
   at 1kHz). Deterministic, benchmarkable against sim ground truth, and the
   ideal hot primitive for the compute-partitioning study. Prior art exists
   (CAPO, arXiv:2602.17393) and is treated as a benchmark, not a competitor —
   the novel contribution is the measured compute bake-off.
3. **Simulator**: MuJoCo. Isaac Sim/Lab requires an RTX GPU (not available);
   MuJoCo is CPU-only, already known, and the legged-locomotion reference.
4. **ROS 2**: Jazzy via RoboStack (pixi) on WSL2 Debian, with `rmw_zenoh_cpp`
   as the RMW. ROS 2 lives at the edges only; the deterministic path uses
   typed shared memory/contracts. Rationale: ecosystem/hiring fluency without
   letting DDS into the hot path; zenoh avoids the multicast discovery
   failure mode documented across WSL2 setups.
5. **Compute bake-off** (the differentiator): the EKF hot path implemented as
   (a) C++ on pinned cores, (b) Rust over iceoryx2 shared memory, (c) the same
   fixed-point pipeline as RTL — synthesized for FPGA targets (Yosys) and
   taken through OpenLane2 + SKY130 for an ASIC-target PPA report. Metrics:
   latency, jitter, RMSE vs ground truth, utilization/area, power.
6. **OS split**: WSL2 Debian owns everything that builds or runs; Windows 11
   owns Foxglove and editing. The only sanctioned crossing is the Foxglove
   WebSocket bridge. Repo lives on the Linux filesystem exclusively.

## Consequences

- No real hardware is required for any phase; "sim-to-real" claims are limited
  to sim-side skills (noise modeling, domain-randomization awareness) and
  stated as such.
- WSL2 lacks PREEMPT_RT: real-time claims are "deterministic by design" and
  must carry jitter histograms.
- HDL tooling (Verilator/Yosys/OpenLane) is deferred to v0.4; versions must be
  pinned when installed (PDK/tool coupling is the known pain point).
- The estimator has prior art; scope control matters. If a phase drifts toward
  estimator novelty, stop and return to the bake-off.
