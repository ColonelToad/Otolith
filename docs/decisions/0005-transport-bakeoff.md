# 0005 — v0.2 Transport Bake-off: Four Contenders, One Decision

Date: 2026-09-23
Status: Accepted (2026-09-23 — measurements in, decision recorded below)

## Context

v0.1 is done and the Go2 demo shipped (video + GIF + ablation, `493e4d9`).
The project's load-bearing question — *what belongs in software, and what
belongs in silicon?* — needs its first measured answer at the transport
layer: how sensor data crosses from producer to the deterministic fusion
path. Per-repo convention this decision gets an ADR before any benchmark
code lands; the eval harness (0004) is the regression gate everything must
pass through.

Env survey (`pixi list`, 2026-09-23):

- ROS 2 Jazzy + `rmw_zenoh_cpp` 0.2.9: present, the current edge transport.
- `iceoryx` **v1** (`posh`/`hoofs`/`binding_c` 2.0.6, headers + `.so` in
  `$CONDA_PREFIX`): present **transitively** via `ros-jazzy-desktop`.
  This is the C++ iceoryx, not the Rust `iceoryx2` v0.3 names.
- `iceoryx2` (Rust crate): **absent** from the pixi env.
- Rust toolchain: `rustc`/`cargo` 1.90.0 exist via user-level rustup
  (`~/.cargo/bin`), visible inside `pixi run` but **not pixi-managed**
  (not in `pixi.toml`, not reproducible via `pixi.lock`).
- POSIX SHM (`shm_open`/`mmap`, futex/sem): libc only, always available.

## Decision

Bench **four** transports, same workload, same metrics. Contenders:

- **A. ROS 2 topics (zenoh).** Status quo edge path. Expected to lose on
  latency/jitter; kept as the honest baseline the bake-off must beat.
- **B. iceoryx2 shared memory.** The v0.3-named transport. **Gated on an
  env decision** (see Consequences): not in pixi env; fetch via cargo
  (non-hermetic, rustup-managed toolchain) or add Rust to `pixi.toml`
  first. Does not block A/C/D.
- **C. Typed-contract ring buffer (in-repo, header-only C++).** The
  architecture-sketch favorite: fixed-size structs, single-producer /
  single-consumer, zero-alloc, no broker, no daemon. Expected winner;
  the bake-off exists to prove it, not assert it.
- **D. POSIX SHM baseline (`shm_open` + `mmap` + semaphore).** No
  framework, libc only. Answers "how much does a framework cost over
  raw shared memory?" — if C ever loses to D by more than noise, the
  ring design is wrong, not the concept.

Workload: cross-process producer → consumer loop carrying the real
payload shape — `LogRow`, fixed `288 B` (`log.hpp:42`, asserted in
`test_log.cpp:11`) — at 500 Hz (sim rate) plus a stress rate TBD in the
bench (e.g. 5 kHz, 10× headroom probe, still below the fusion jitter
bounds from 0004). Same-process loopback is out of scope: the point of
a transport is crossing a process boundary.

Metrics (per contender, WSL2 caveats from 0004 apply — histograms, not
adjectives): latency `p50/p99/max`, jitter histogram, copies per
message (0 demanded of C/D, counted for A/B), allocations in the hot
loop (0 demanded of C/D), setup/teardown cost, and failure behavior
(stale consumer, slow consumer, producer death — the watchdog
discipline from conventions §5 applies to the harness too).

Harness shape (design only — no code until sign-off): a `transport_bench`
binary in `fusion/` (Catch2-adjacent, same build), one driver per
contender behind a common timing harness, results to `eval/out/`
(gitignored) + a summary table for the write-up. Pass criteria for the
phase: all four (or A/C/D + B-gated) run green, numbers recorded, ADR
updated to Accepted with the v0.3/v0.4 transport choice.

## Results (2026-09-23, `eval/out/bench-20260923-145600/`, 5 s per cell)

288 B payload, cross-process, CLOCK_MONOTONIC latency. p99/max carry the
WSL2-no-PREEMPT_RT scheduling tax on every contender; p50 is the handoff.

| | 500 Hz: p50 / p99 / drops | 5 kHz: p50 / p99 / drops |
|---|---|---|
| C ring | 0.6 µs / 1.7 ms / 0% | 0.4 µs / 1.1 ms / 0% |
| D mailbox | 1.1 µs / 1.3 ms / 4.6% prod | 0.7 µs / 101 µs / 5.3% prod |
| A ROS/zenoh | 408 µs / 1.5 ms / 0% | 297 µs / 1.6 ms / 0.26% transport |
| B iceoryx2 | 7.9 µs / 755 µs / 0% | 3.5 µs / 1.0 ms / 0% |

## Decision

**C — the in-repo typed SPSC ring — is the v0.2→v0.5 transport.**
Sub-µs steady state, zero drops at both rates, zero syscalls in the loop,
zero new dependencies. A stays at the edge exactly as deployed (router
hairpin included in its number, honestly). D retires as a baseline having
done its job: cap-1 drops under descheduling (all producer-side, zero
transport losses) validate the ring's depth-1024 choice. B is measured for
the record and frames the v0.3 scoping question below.

Why B trails C ~10× (source-read, iceoryx2 0.10.0, not a profiler claim):
per `send_sample` (`src/port/publisher.rs:373`) every send runs
`update_connections` (dynamic peer-list maintenance), history bookkeeping,
chunk loan/refcount/return lifecycle, and a multi-line control working set
(registry, dynamic storage, handles, queues) — vs the ring's one acquire
load + memcpy + one release store. iceoryx2 supports dynamic MPMC peers,
monitoring, and lifecycle; the ring supports exactly one static SPSC pair.
The gap is generality tax, not language: `send_copy`'s 288 B memcpy costs
~50 ns, three orders below the 8 µs — a zero-copy loan would shave
nanoseconds, not the gap. (No CPU pinning in the fixture, per "as
deployed"; WSL2 vCPU migration punishes the larger control working set
most, which also explains the ms-scale tails everywhere.)

## Consequences (updated)

- B's env question is settled as run, not as specified: cargo-fetch
  against rustup 1.90.0 worked first try (`libc` pinned to 0.2 — 1.x has
  no stable release on crates.io; `subscriber_max_buffer_size` raised to
  1024 to match C; `/tmp/iceoryx2` wiped per run).
- v0.3 scoping is OPEN (was: "Rust port, iceoryx2"): port-the-ring vs
  adopt-iceoryx2 is now a measured tradeoff, not an assumption. See
  v0.3 kickoff discussion; this ADR does not pre-decide it.
- Fixture notes kept in `fusion/bench/README.md`: zenoh peers here need
  `rmw_zenohd` (multicast scouting dead — the router is part of A's
  fixture); all cmake/cargo invocations must go through `pixi run`
  (the bare shell has no pixi activation, which once silently emptied
  `CMAKE_PREFIX_PATH`).
- Rejected (unchanged): DDS multicast, TCP loopback, same-process
  benchmarks, iceoryx v1 as SHM representative.
