// Fixed-point package — exact RTL mirror of fusion/fixed/fixed.hpp.
// Q8.24 storage (states/Phi/inputs), Q16.48 covariance, 128-bit accums.
// Rounding: half away from zero on narrowing. Overflow: saturate +
// sticky flag (the testbench compares ever-saturated vs the model's
// per-step sat_count). All functions pure combinational.

package fixed_pkg;

  typedef logic signed [31:0]  q24_t;    // Q8.24
  typedef logic signed [63:0]  q48_t;    // Q16.48 (P)
  typedef logic signed [127:0] acc128_t; // mixed-product accumulator

  // {sat, value} bundles: single logic cone per op, flag ORed by the FSM.
  typedef struct packed {
    logic               sat;
    logic signed [31:0] v;
  } sat24_t;

  typedef struct packed {
    logic               sat;
    logic signed [63:0] v;
  } sat48_t;

  localparam q24_t Q24_MAX = 32'sh7fffffff;
  localparam q24_t Q24_MIN = 32'sh80000000;
  localparam q48_t Q48_MAX = 64'sh7fffffffffffffff;
  localparam q48_t Q48_MIN = 64'sh8000000000000000;

  // Q16.48 (int64) -> Q8.24, round-half-away + saturate. Mirrors narrow().
  function automatic sat24_t s_narrow(input logic signed [63:0] x);
    logic signed [63:0] half;
    logic signed [63:0] shifted;
    logic sat;
    q24_t v;
    half = (x >= 0) ? 64'd8388608 : -64'd8388608; // 2^23
    shifted = (x + half) >>> 24; // arithmetic shift; x+half cannot
    // overflow: |x| <= ~2^62 in our uses (products of int32s)
    sat = (shifted > 64'sd2147483647) || (shifted < -64'sd2147483648);
    if (shifted > 64'sd2147483647) v = Q24_MAX;
    else if (shifted < -64'sd2147483648) v = Q24_MIN;
    else v = shifted[31:0];
    s_narrow = {sat, v};
  endfunction

  // Q24.72 (128-bit) -> Q16.48, round-half-away + saturate (genuine rail
  // only — overflow-freedom proved in fixed.hpp). Mirrors narrow48().
  function automatic sat48_t s_narrow48(input acc128_t x);
    acc128_t half;
    acc128_t shifted;
    half = (x >= 0) ? (acc128_t'(1) << 23) : -(acc128_t'(1) << 23);
    shifted = (x + half) >>> 24;
    begin
      logic sat;
      q48_t v;
      sat = (shifted > acc128_t'(Q48_MAX)) || (shifted < acc128_t'(Q48_MIN));
      if (shifted > acc128_t'(Q48_MAX)) v = Q48_MAX;
      else if (shifted < acc128_t'(Q48_MIN)) v = Q48_MIN;
      else v = shifted[63:0];
      s_narrow48 = {sat, v};
    end
  endfunction

  // Saturating Q8.24 add/sub (mirror add()/sub()).
  // NOTE: return value assigned WHOLE (concat) — the -sv frontend
  // cannot width-resolve field-wise assignment to struct returns.
  function automatic sat24_t s_add24(input q24_t a, b);
    logic signed [32:0] s;
    logic sat;
    q24_t v;
    s = 33'(a) + 33'(b);
    sat = (s > 33'sd2147483647) || (s < -33'sd2147483648);
    if (s > 33'sd2147483647) v = Q24_MAX;
    else if (s < -33'sd2147483648) v = Q24_MIN;
    else v = s[31:0];
    s_add24 = {sat, v};
  endfunction

  function automatic sat24_t s_sub24(input q24_t a, b);
    logic signed [32:0] d;
    d = 33'(a) - 33'(b);
    begin
      logic sat;
      q24_t v;
      sat = (d > 33'sd2147483647) || (d < -33'sd2147483648);
      if (d > 33'sd2147483647) v = Q24_MAX;
      else if (d < -33'sd2147483648) v = Q24_MIN;
      else v = d[31:0];
      s_sub24 = {sat, v};
    end
  endfunction

  // Saturating Q16.48 add (mirror padd()).
  function automatic sat48_t s_add48(input q48_t a, b);
    // Same-sign overflow test on wraparound (mirrors acc_add/padd).
    logic signed [63:0] r;
    r = a + b; // wraps per SV semantics; test below detects it
    begin
      logic sat;
      q48_t v;
      sat = ((a >= 0 && b >= 0 && r < 0) || (a < 0 && b < 0 && r >= 0));
      if ((a >= 0 && b >= 0 && r < 0)) v = Q48_MAX;
      else if ((a < 0 && b < 0 && r >= 0)) v = Q48_MIN;
      else v = r;
      s_add48 = {sat, v};
    end
  endfunction

  // Saturating 64-bit add (mirror acc_add()).
  function automatic sat48_t s_add64(input logic signed [63:0] a, b);
    logic signed [63:0] r;
    r = a + b;
    begin
      logic sat;
      q48_t v;
      sat = ((a >= 0 && b >= 0 && r < 0) || (a < 0 && b < 0 && r >= 0));
      if ((a >= 0 && b >= 0 && r < 0)) v = Q48_MAX;
      else if ((a < 0 && b < 0 && r >= 0)) v = Q48_MIN;
      else v = r;
      s_add64 = {sat, v};
    end
  endfunction

  // Negate with -MIN saturation (mirror neg()).
  function automatic sat24_t s_neg24(input q24_t a);
    s_neg24 = {(a == Q24_MIN), (a == Q24_MIN) ? Q24_MAX : -a};
  endfunction

  // Multiply helpers (mirror mul() and the Q8.24xQ16.48 product in P1/P2).
  // Operands are explicitly widened BEFORE multiplying (SV result-width
  // rule would otherwise truncate 32x32 to 32 bits — a real bug caught
  // during M2 development, fixed here).
  function automatic sat24_t s_mul24(input q24_t a, b);
    s_mul24 = s_narrow(64'(a) * 64'(b));
  endfunction

  function automatic sat48_t s_mul48(input q24_t a, input logic signed [63:0] b);
    s_mul48 = s_narrow48(acc128_t'(a) * acc128_t'(b));
  endfunction

  // Halve with round-half-away (mirror halve_q()).
  function automatic sat24_t s_halve24(input q24_t v);
    logic signed [32:0] t;
    t = 33'(v) + ((v >= 0) ? 33'sd1 : -33'sd1);
    s_halve24 = {1'b0, t[32:1]}; // shrinking: cannot overflow
  endfunction

  // Unrolled N-R inverse sqrt, 6 iterations, seed 1.0 (mirror invsqrt_nr).
  // NOTE: loop var hoisted (not for-init declared) — the -sv frontend
  // mis-elaborates for-init decls inside functions (Yosys rtlil assert).
  function automatic sat24_t invsqrt6(input q24_t x);
    q24_t y, t1, t2, half;
    logic s;
    q24_t THREE;
    int i;
    THREE = 32'sd50331648; // 3.0 exact
    y = 32'sd16777216;     // 1.0
    s = 1'b0;
    for (i = 0; i < 6; i++) begin
      sat24_t r1, r2, r3, r4;
      r1 = s_mul24(x, y); s = s | r1.sat; t1 = r1.v;
      r2 = s_mul24(t1, y); s = s | r2.sat; t2 = r2.v;
      r4 = s_sub24(THREE, t2); s = s | r4.sat;
      r3 = s_halve24(r4.v); half = r3.v; // halve never saturates
      r1 = s_mul24(y, half); s = s | r1.sat; y = r1.v;
    end
    invsqrt6 = {s, y};
  endfunction

endpackage
