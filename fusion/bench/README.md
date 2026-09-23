# Transport bake-off bench (v0.2, ADR-0005)

Cross-process producer → consumer latency for four transports moving the
same payload: a 288 B message size-locked to the `LogRow` contract
(`log.hpp:42`), send stamp + seq in the first 16 bytes (`common.hpp`).

| Contender | Mechanism | Buffer | Handoff | Env |
|-----------|-----------|--------|---------|-----|
| A `bench_a_*` | ROS 2 topics (`ByteMultiArray`, best-effort, keep-last 10) via `rmw_zenoh_cpp` | 10 | router hairpin | pixi (ROS Jazzy) |
| B `iox2/bench-iox2` | iceoryx2 0.10 `publish_subscribe<[u8;288]>`, `send_copy` | 1024 | zero-copy loan pool + copy in | cargo-fetch (rustup 1.90, **not** pixi-managed) |
| C `bench_c` | in-repo SPSC sequence ring in POSIX SHM, consumer spins | 1024 slots | lock-free, no syscalls steady-state | libc only |
| D `bench_d` | POSIX SHM single-slot mailbox + 2 semaphores, consumer blocks | 1 slot | syscall (futex) per message | libc only |

## Run

```bash
# C++ bench (needs the pixi env for configure-time prefix):
pixi run bash -c 'cmake -S fusion -B fusion/build -DCMAKE_PREFIX_PATH=$CONDA_PREFIX'
pixi run bash -c 'LIBRARY_PATH=$CONDA_PREFIX/lib:$LIBRARY_PATH cmake --build fusion/build --target bench_c bench_d bench_a_pub bench_a_sub -j$(nproc)'

# Rust bench (first build fetches iceoryx2 from crates.io):
pixi run cargo build --release --manifest-path fusion/bench/iox2/Cargo.toml

# Full matrix (all contenders x 500 Hz + 5 kHz, 5 s each):
pixi run python fusion/bench/run_bench.py
# Smoke: pixi run python fusion/bench/run_bench.py --quick
```

Results land in `eval/out/bench-<timestamp>/` (gitignored): raw `.bin`
(seq, latency) pairs, `.pubinfo` producer sidecars, `SUMMARY.md` table.

**Stop `go2_demo.sh` before benching** (default ROS domain; crosstalk).

## Reading the numbers

- `p50` = steady-state handoff cost. `p99`/`max` = scheduling reality on
  WSL2 (no PREEMPT_RT) — histograms, not adjectives (cf. ADR-0004).
- `drops (prod+transport)`: producer drops = pacer skipped (ring/mailbox
  full — by design, pacing stays sacred); transport drops = sent but never
  received (middleware eviction). A cap-1 design (D) is *expected* to drop
  under descheduling; that is the signal, not a bug.
- A runs pub → `rmw_zenohd` → sub: zenoh peers in this env do not discover
  router-less (multicast scouting dead), so the router is part of the
  as-deployed fixture. Its forward cost is inside A's latency — honestly,
  since the deployed edge pays it too.
- B's `subscriber_max_buffer_size` = 1024 matches C's ring depth so the
  comparison isolates mechanism, not buffering. `/tmp/iceoryx2` is wiped
  per run (stale registry would reject the static config).

## Layout

- `common.hpp` — CLI, 288 B payload, `CLOCK_MONOTONIC` pacing helpers.
- `ring.hpp` / `mailbox.hpp` — header-only C/D transports.
- `bench_{c,d,a_pub,a_sub}.cpp` — role-selected mains.
- `iox2/` — cargo crate for B (`Cargo.lock` committed; `target/` ignored).
- `run_bench.py` — orchestrator + stats (router lifecycle included).
