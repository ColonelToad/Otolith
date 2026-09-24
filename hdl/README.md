# hdl/ — v0.4 RTL port (ADR-0007)

Fixed-point predict pipeline (`P ← ΦPΦᵀ + Qd`, 15×15, + quaternion
`exp`/`normalize` glue) as synthesizable SystemVerilog, carried through
Verilator parity → Yosys synthesis → LibreLane/SKY130 PPA.

## Layout

- `rtl/` — synthesizable SystemVerilog (M2). No test code, no `$display`.
- `tb/` — Verilator testbenches + bit-parity drivers vs the C++ model (M2).
- `synth/` — Yosys scripts + constraints (M3). Pinned: Yosys 0.68.
- `openlane/` — LibreLane configs (M4). Pinned: `iic-osic-tools:2026.07`,
  `PDK=sky130A`, `STD_CELL_LIBRARY=sky130_fd_sc_hd`, absolute die area
  (see `~/Projects/hardware/README.md` gotchas).

## Toolchain (all verified live)

- oss-cad-suite: Yosys 0.68, Verilator 5.051, nextpnr (ECP5 target).
- Docker `iic-osic-tools:2026.07` (15.9 GB image present, daemon responds).
- Everything runs from WSL2 Debian; repo stays on the Linux filesystem.

## Formats (locked M0, see ADR-0007)

Q8.24 storage (±128, LSB 6e-8), 64-bit accumulators, round-half-up on
narrowing, saturate on overflow. Bit-parity vs `fusion/fixed/` model;
float RMSE is the error-bound reference.
