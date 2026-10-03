// Test wrapper exposing fixed_pkg's Q16.48 primitives as ports so Verilator
// can drive them (a package cannot be a top module). Test-only: not part of
// the synthesized RTL, and not read by hdl/synth/*.ys.
module fixed_pkg_wrap (
  input  logic [63:0] clz_x,
  input  logic [63:0] recip_x,
  input  logic [63:0] mul_a,
  input  logic [63:0] mul_b,
  output logic [6:0]  clz_out,
  output logic [63:0] recip_out,
  output logic        recip_sat,
  output logic [63:0] mul_out,
  output logic        mul_sat
);
  logic [64:0] rr, rm;
  assign clz_out = fixed_pkg::s_clz64(clz_x);
  assign rr = fixed_pkg::s_recip48(fixed_pkg::q48_t'(recip_x));
  assign recip_out = rr[63:0];
  assign recip_sat = rr[64];
  assign rm = fixed_pkg::s_mulq48(fixed_pkg::q48_t'(mul_a), fixed_pkg::q48_t'(mul_b));
  assign mul_out = rm[63:0];
  assign mul_sat = rm[64];
endmodule