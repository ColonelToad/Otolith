// 32x64 multiply-narrow kernel (M4 P&R target).
// Wraps fixed_pkg::s_mul48 (Q8.24 x Q16.48 -> Q16.48, round-half-away +
// saturate) with registered I/O: 1-cycle latency, so the multiplier tree
// is the combinational path between real start/endpoints for STA.
// Same Yosys-0.67-safe style as predict_core: package-scope fixed_pkg::
// calls, no `import`, packed {sat,value} return vector.
module mul48_kernel (
  input  logic               clk,
  input  logic               rst_n,
  input  logic               valid,
  input  logic [31:0]        a,      // Q8.24 (signed two's complement bits)
  input  logic [63:0]        b,      // Q16.48 (signed two's complement bits)
  output logic               ready,  // registered valid, 1 cycle later
  output logic [63:0]        p,      // Q16.48 product
  output logic               sat     // sticky-per-vector saturate flag
);
  logic signed [31:0] a_r;
  logic signed [63:0] b_r;
  logic               v_r;
  logic [64:0]        r; // {sat, value}, combinational

  assign r = fixed_pkg::s_mul48(a_r, b_r);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a_r   <= '0;
      b_r   <= '0;
      v_r   <= 1'b0;
      p     <= '0;
      sat   <= 1'b0;
      ready <= 1'b0;
    end else begin
      a_r   <= a;
      b_r   <= b;
      v_r   <= valid;
      p     <= r[63:0];
      sat   <= r[64];
      ready <= v_r;
    end
  end
endmodule
