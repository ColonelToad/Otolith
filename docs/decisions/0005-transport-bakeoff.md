# 0005 — v0.2 Transport Bake-off: Four Contenders, One Decision

Date: 2026-09-23
Status: Proposed (measurement + decision gated on sign-off)

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

## Consequences

- A/C/D can be built and measured **now**: zero new environment deps
  (ROS edge already in env, C/D are libc + in-repo headers).
- B (iceoryx2) needs one of, decided at sign-off (2026-09-23: **option 1
  chosen** — cargo-fetch the `iceoryx2` crate against the rustup
  1.90.0 toolchain; non-hermetic `~/.cargo` accepted for the bench,
  hermetic pixi Rust deferred to v0.3 kickoff):
  1. cargo-fetch the `iceoryx2` crate against the rustup toolchain
     (fast, non-hermetic, `~/.cargo` outside `pixi.lock`);
  2. add Rust to `pixi.toml` first (hermetic, slower, touches the env
     every agent command depends on);
  3. defer B to v0.3 kickoff and let A/C/D decide the interim
     (risk: v0.3 starts without its headline number).
- Rejected: DDS multicast (doesn't survive the Windows boundary —
  settled repo-wide), TCP loopback as a contender (measures the kernel
  stack, not a deterministic-path candidate), same-process benchmarks
  (prove nothing about transport), iceoryx v1 as the SHM representative
  (v0.3 names iceoryx2; benching v1 answers a question nobody asked —
  though its incidental presence is a fallback if iceoryx2 proves
  uninstallable).
- Demo leftovers swept in the same pass: root-level `*.mp4` + 
  `*.Zone.Identifier` gitignored (shipped artifacts in `assets/` are
  tracked and unaffected); raw takes stay untracked.
