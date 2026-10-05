# hdl/ — v0.4 RTL port (ADR-0007)

Fixed-point predict pipeline (`P ← ΦPΦᵀ + Qd`, 15×15, + quaternion
`exp`/`normalize` glue) as synthesizable SystemVerilog, carried through
Verilator parity → Yosys synthesis → LibreLane/SKY130 PPA.

## Layout

- `rtl/` — synthesizable SystemVerilog (M2): `fixed_pkg.sv` (exact
  Q8.24/Q16.48 op mirrors), `predict_core.sv` (comb nominal + sequential
  15×15 engine, 6978 cycles/step deterministic). No test code.
- `tb/` — Verilator lockstep bench (M2): `tb_predict.cpp` drives DUT +
  C++ model side-by-side over the 2500-step trot, bit-compares
  q/p/v/P/sat every step. `make run OTLG=/tmp/parity.otlg`.
  Self-checks: write-read bus verify, `--selftest` 1-step mode,
  `TB_DUMPALL` full-array dumps. **Status: 2500/2500 PASS.**
- `synth/` — Yosys scripts + constraints (M3). Pinned: Yosys 0.68.
  `predict.ys` (full flow), `stat.ys` (fast inventory).
- `openlane/` — LibreLane configs (M4). Pinned: `iic-osic-tools:2026.07`,
  `PDK=sky130A`, `STD_CELL_LIBRARY=sky130_fd_sc_hd`, absolute die area
  (see `~/Projects/hardware/README.md` gotchas).

## M3 results (measured 2026-09-24)

- Yosys synth: 45,129 cells (108 s), zero warnings-are-errors.
- nextpnr ECP5-85F @50 MHz target: **Fmax 65.8 MHz** (constraint met),
  **LUT 21,288/83,640 (25%)**, FF 15,485/83,640 (19%), DSP 0/156
  (mults in LUTs — DSP inference follow-up open), BRAM 0/208.
- Critical path: control/state → Phi/P clock-enables (routing-heavy),
  NOT the MAC datapath. Debug bus muxes visible on async paths only.
- Wall time per predict step @65.8 MHz: 7713 cycles = **117 µs**
  (17× inside the 2 ms budget; jitter: none by construction).
- Fits ECP5-45F comfortably (~48%); 25F at ~88% (tight, routable?).

## M5-A results: ldl_kernel (measured 2026-10-05)

The update path's factorization/solve kernel — `rtl/ldl_kernel.sv`, WIDTH=6
(the dominant real width: 2 stance feet). Unpivoted LDL', `S = L D L'` with
unit-diagonal `L`, then `S^-1 b` by forward substitution / `1/D` scaling /
back substitution. One 64x64 multiplier and one 128-bit accumulator, shared
by every dot product and every Newton-Raphson iteration (calling `s_mulq48` at
each of the seven sites made Yosys build a multiplier per site: 823,823
primitives vs 161,276). Serialized 64-bit word bus for load/dump — a flat
6x6 q48 port would be ~1700 pads and would swamp the core area.

**Parity: 9002/9002 PASS.** Not synthetic — those are the real innovation
covariances captured from the 10 s trot log (`fuse_update_study --dump-s`),
each run twice: once with a unit-vector RHS (the columns of `S^-1`, which is
what `K` needs) and once with the real measurement residual. `L`, `D`, `x` match
bit-for-bit plus the `pd`/`sat` flags.

**PPA — ECP5-85F via nextpnr** (`make -C hdl/synth ldl-report`):

| | |
|---|---|
| synthesis | 61,933 cells -> 40,337 LUT4, 4,396 FF |
| utilization | LUT **48%** (40,338/83,640), FF 5%, IO 38%, DSP 0/156, BRAM 0/208 |
| **Fmax** | **13.91 MHz** (constraint 50 MHz — **not met**) |
| latency | 1,262 cycles = **91 µs**, vs the 2 ms update budget = **22x inside** |
| sky130 estimate | 161,276 primitives ~ 0.18 mm² (projection, not built) |

Two timing changes, each kept only because it moved the measured number and
parity was re-verified after each: registering the multiplier output
(7.36 -> 10.73 MHz) and splitting the reciprocal rescale's 128-bit barrel
shift from its overflow compare (10.73 -> 13.91 MHz). Both were diagnosed from
nextpnr's critical-path report; the first guess (that the multiplier dominated)
was wrong, and only the report settled it. The final critical path starts at
the **`out_ready` input pad** and runs through the wide register-file address
muxing — control/routing, not the MAC datapath, the same conclusion M3 reached
for `predict_core`. With no input-delay constraint the pad delay falls inside
the clock period, so 13.91 MHz is pessimistic about the internal logic.

**No sky130 GDS for this kernel.** LibreLane's ABC does not converge: a
65,860-cell input, killed at 2 h each at 10 ns and 20 ns, memory flat
(grinding, not OOM), and the resolved strategy is already `AREA 0` so there
was no cheaper script. Relaxing the clock period changed nothing, which is what
distinguishes this from M4's full-core stall. Documented in
`openlane/ldl/config.yaml`; the PPA above comes from the nextpnr path instead.

## M5-B: the update path is model-only (2026-10-05)

`ldl_kernel` covers the factorization and the solve, and nothing else. Still
in C++ and still unported:

- **The measurement model** (`fusion/fixed/fixed_meas.hpp`): leg FK,
  `cordic_sincos`, `rdot`, `H`, `y`. `cordic_sincos` is 20 shift-add
  iterations with no multiplier, so it ports cheaply — but note it converges
  only to +/-1.7433 rad, and the Go2 knee reaches ~2.8 rad, so the RTL port
  needs the same `sin_cos_wide` quadrant folding the model now has.
- **`apply_dx_fixed`**: `K*d` at 15xN, then the log-odds quaternion add. No
  multiplier needed (the 2^48 shift is the log-odds scale), but it is a state
  write on the hot path and has no parity bench yet.
- **The 15x15 Joseph form**: `P' = P - K*S*K' + K*Rmat*K'`. Per the M5-C cost
  table this is **54% of the update cost at rows=6** — the largest remaining
  slice. It is a rank-N update and could be restructured as
  `(I - K*H)*P*(I - K*H)' + K*Rmat*K'`, which trades 2 matmuls for 2 matmuls
  but is 15 wide instead of N, so it may not help; measure before assuming.

The 10.19 mm² "full fixed-point MEKF" projection in `hdl/M5_UPDATE_STUDY.md`
excludes all three. Read it as a lower bound.

## M4 results (measured 2026-09-25, rescoped to kernel)

Full-core SKY130 is a documented dead end: post-synth stat 297,819
cells / 3.40 mm² vs the 2.53 mm² core = 134% util (unplaceable), and
`report_checks -slack_max -0.01` × `group_path_count 1000` OOMs the
container on the all-violated netlist (exit 255, twice). Recorded as
the honest cost estimate; see `openlane/config.yaml` note. The closed
GDS loop runs on `mul48_kernel` (registered `s_mul48`: Q8.24×Q16.48 →
Q16.48, the dominant operator) — `rtl/mul48_kernel.sv`,
`tb/tb_mul48.cpp` (**2228 vectors bit-parity PASS** vs
`fusion/fixed/`), `synth/mul48.ys` (106,555 pre-map primitives),
`openlane/mul48/config.yaml` (10 ns, 800×800 die sized from measurement).

- Flow `RUN_2026-09-25_16-01-05`: P&R clean through detailed routing.
- **GDS**: magic 21.9 MB; klayout independent streamout XORs with
  **0 differences** (verified manually — see container gotchas).
- **DRC**: route 0 (174→12→7→0 across iters). **Antenna**: 0.
- **Timing @10 ns**: setup TT −4.86 / SS −18.1 / **FF +0.38 ns (MET)**;
  hold MET all corners (+0.33…+0.62 ns); skew 0.29 ns. Honest Fmax:
  **~67 MHz TT / ~36 MHz SS** (one relaxation rerun declined — the
  number is the result).
- **Area**: 20,391 instances (19,819 stdcell), 119,244 µm² = 0.119 mm²
  @19.3% util. **Power**: 0.297 W total.

## Container gotchas (paid for, write down)

- Image entrypoint rejects `sleep infinity`: launch with `--wait`
  (`docker run -d --name iic-otolith -v … -v … image --wait`).
- Docker Desktop restart **breaks WSL bind mounts** (become empty
  tmpfs): symptom `cd: /headless/otolith/…: No such file`; fix is
  `docker rm -f` + recreate. PDK (`/foss/pdks`, image layer) survives;
  anything `apt-get install`ed (verilator 5.020 for step 01) does not.
- `klayout` binary lives at `/foss/tools/klayout/klayout`, off the
  default PATH — step 62-xor fails with `No such file or directory`;
  rerun the `ruby xor.drc …` command from its COMMANDS file manually
  with PATH fixed.
- **The container's `/tmp` is not the host's `/tmp`.** WSL2 + Docker Desktop
  do not share it, so the bench defaults (`OTLG ?= /tmp/parity.otlg`,
  `SCAP ?= /tmp/s_cap.txt`) fail inside the container with `bad log` even
  though the file plainly exists on the host. Stage inputs in the repo
  instead — `.work/` is gitignored for exactly this — and override:
  `make run OTLG=/work/.work/<log>` and `make run-ldl SCAP=/work/.work/<cap>`.
- OpenSTA's Verilog reader rejects `signed` port declarations
  (`input signed [31:0]` = syntax error at STA): keep signedness on
  internal regs only, ports plain `logic [N:0]` (bit-safe, parity
  re-verified).

## DUT observability (read-only, no datapath effect)

- `0x000` done, `0x001` sat_count (sticky, cleared on start).
- `0x1F0–0x1F7`: nominal snapshot (R0/R4/R8, eq0, nq0, Phi0, qdg, Phi9).
- `0x1F8/0x1F9/0x1FA/0x1FB`: indirected index + P1/P2/Phi reads.
  Kept deliberately: found the F00 sign bug (transposed skew) and the
  `>>>` vs `>>` symmetrize bug during M2 bringup.

## Toolchain (all verified live)

- oss-cad-suite: Yosys 0.68, Verilator 5.051, nextpnr (ECP5 target).
- Docker `iic-osic-tools:2026.07` (15.9 GB image present, daemon responds).
- Everything runs from WSL2 Debian; repo stays on the Linux filesystem.

## Formats (locked M0, amended M1)

States/Phi Q8.24 (±128, LSB 6e-8); covariance P Q16.48 (LSB 3.6e-15);
128-bit accumulators (overflow-free by proof); round-half-away;
saturate-and-count. Bit-parity vs `fusion/fixed/` model; float RMSE is
the error-bound reference (0.1023 vs 0.1064 m, same trajectory).

## Toolchain (all verified live)

- oss-cad-suite: Yosys 0.68, Verilator 5.051, nextpnr (ECP5 target).
- Docker `iic-osic-tools:2026.07` (15.9 GB image present, daemon responds).
- Everything runs from WSL2 Debian; repo stays on the Linux filesystem.

## Formats (locked M0, see ADR-0007)

Q8.24 storage (±128, LSB 6e-8), 64-bit accumulators, round-half-up on
narrowing, saturate on overflow. Bit-parity vs `fusion/fixed/` model;
float RMSE is the error-bound reference.
