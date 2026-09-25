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
