# 0006 — v0.3 Rust Port: Full Filter, Transport Trait, Two Backends

Date: 2026-09-23
Status: Proposed (M0 kickoff; Accepted at M6 with numbers)

## Context

ADR-0005 settled the transport (C ring wins: p50 ~0.5 µs, 0 drops) and
left v0.3 scoping open: the phase map's "Rust port (iceoryx2)" was written
pre-measurement, and iceoryx2 trails the ring ~10× on generality tax
(dynamic peers, chunk lifecycle, control working set — verified in
0.10.0 source, recorded in 0005). The v0.3 decision: port the ring too,
and let the second bake-off — plus a soundness argument about `unsafe` —
decide what the Rust future looks like.

The filter core is small and stable: 134-line MEKF (`fusion.cpp`) +
44-line leg FK (`leg_kin.cpp`), Eigen-based, regression-gated by 0004's
pyramid (15 pytest + 11 ctest) and the M3 anchor (0.1064 m / 0.0674 m/s /
13.1°). Small enough to port whole; stable enough to parity-gate.

## Decision

1. **Full port, one phase**: `rust/` workspace with `otolith-transport`
   (trait + ring backend + iceoryx2 backend) and `otolith-fusion`
   (MEKF + leg FK, Eigen → nalgebra). No transport-only split.
2. **Pixi-managed Rust**: reverses the bench-era option-1 shortcut
   deliberately. A real phase gets a hermetic toolchain (`pixi.toml`,
   conda-forge rust ≥1.89 to satisfy iceoryx2 0.10's MSRV);
   `pixi run cargo` must resolve to it, not `~/.cargo`. Known split:
   Miri needs nightly rustc internals, so the Miri gate — and only the
   Miri gate — runs on rustup nightly; all builds/tests run on pixi
   stable. Both toolchains are pinned/documented, not ambient.
3. **`Transport::{send, try_recv}` trait** over the shared 288 B
   `BenchMsg` layout (same bytes as `fusion/bench/common.hpp`, both
   languages). Backends: hand-rolled SHM ring (faithful port of
   `ring.hpp`: CAP 1024, claim/publish/free protocol, producer never
   blocks) and iceoryx2 (reuse of the proven `fusion/bench/iox2`
   pattern: `send_copy`, buffer 1024, spin).
4. **Soundness split (the `unsafe` argument)**: sequence/state logic in a
   pure-safe module — unit tests + **Miri gate**; the SHM-mapping shim
   (`shm_open`/`mmap`, unsupported by Miri) held to ~30 auditable lines,
   each mapped to a written invariant (size-checked `fstat`,
   `MAP_SHARED`, single mapping per role). Standing rule: new `unsafe`
   lands in the shim with an invariant or is rejected.
5. **1% parity bar**: Rust estimates through the *unchanged* eval
   harness — pos/vel/att RMSE within 1% of M3, scenario/NEES/fault
   suites green, ablation reproduces 0.528 m dead-reckoning.
   Tolerance-based (float-op ordering differs across compilers);
   bitwise identity explicitly not required.
6. **Second bake-off**: `run_bench.py` gains `e` (rust-ring) and `f`
   (rust-iox2); predictions — `e` within noise of C++ C, `f` within
   noise of v0.2 B (validates the trait added zero overhead). The parity
   suite + bake-off stay as the regression anchor for all future
   improvements.
7. **ROS stays C++**: no `rclrs`/`ros2-client`; "ROS at the edges"
   (conventions §7) covers the port — filter + SHM transports only.

## Consequences

- `rust/.gitkeep` deleted at kickoff; `CLAUDE.md` phase map + README
  table move v0.3 to in-progress.
- `fusion/` C++ and the v0.2 bench are frozen reference points —
  baselines, not renovation targets.
- Rejected: transport-only v0.3a (splits the parity argument across
  phases for no reason at this code size), staying on rustup (fast but
  non-hermetic — wrong default for a full port), iceoryx2-only port
  (assumes away the measured tradeoff), Miri-skipped (removes the
  soundness argument this phase exists to make), bitwise parity
  (dishonest bar across compilers).
