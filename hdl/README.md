# hdl/ — v0.4 RTL port (ADR-0007)

Fixed-point predict pipeline (`P ← ΦPΦᵀ + Qd`, 15×15, + quaternion
`exp`/`normalize` glue) as synthesizable SystemVerilog, carried through
Verilator parity → Yosys synthesis → LibreLane/SKY130 PPA.

## Layout

- `rtl/` — synthesizable SystemVerilog (M2): `fixed_pkg.sv` (exact
  Q8.24/Q16.48 op mirrors), `predict_core.sv` (comb nominal + sequential
  15×15 engine, ~6754 cycles/step deterministic). No test code.
- `tb/` — Verilator lockstep bench (M2): `tb_predict.cpp` drives DUT +
  C++ model side-by-side over the 2500-step trot, bit-compares
  q/p/v/P/sat every step. `make run OTLG=/tmp/parity.otlg`.
  Self-checks: write-read bus verify, `--selftest` 1-step mode,
  `TB_DUMPALL` full-array dumps. **Status: 2500/2500 PASS.**
- `synth/` — Yosys scripts + constraints (M3). Pinned: Yosys 0.68.
- `openlane/` — LibreLane configs (M4). Pinned: `iic-osic-tools:2026.07`,
  `PDK=sky130A`, `STD_CELL_LIBRARY=sky130_fd_sc_hd`, absolute die area
  (see `~/Projects/hardware/README.md` gotchas).

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
