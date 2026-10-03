// Fixed-point package — exact RTL mirror of fusion/fixed/fixed.hpp.
// Q8.24 storage (states/Phi/inputs), Q16.48 covariance, 128-bit accums.
// Rounding: half away from zero on narrowing. Overflow: saturate +
// sticky flag (the testbench compares ever-saturated vs the model's
// per-step sat_count). All functions pure combinational.

package fixed_pkg;

  typedef logic signed [31:0]  q24_t;    // Q8.24
  typedef logic signed [63:0]  q48_t;    // Q16.48 (P)
  typedef logic signed [127:0] acc128_t; // mixed-product accumulator

  // Packed {sat, value} returns: logic [32:0] (24-bit value) and
  // logic [64:0] (48-bit value). Struct returns + field access on temps
  // break Yosys 0.67's frontend (LibreLane flow); packed vectors work
  // everywhere. Single logic cone per op; flag ORed by the FSM.
  localparam q24_t Q24_MAX = 32'sh7fffffff;
  localparam q24_t Q24_MIN = 32'sh80000000;
  localparam q48_t Q48_MAX = 64'sh7fffffffffffffff;
  localparam q48_t Q48_MIN = 64'sh8000000000000000;

  // Q16.48 (int64) -> Q8.24, round-half-away + saturate. Mirrors narrow().
  function automatic logic [32:0] s_narrow(input logic signed [63:0] x);
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
  function automatic logic [64:0] s_narrow48(input acc128_t x);
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
  function automatic logic [32:0] s_add24(input q24_t a, b);
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

  function automatic logic [32:0] s_sub24(input q24_t a, b);
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
  function automatic logic [64:0] s_add48(input q48_t a, b);
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
  function automatic logic [64:0] s_add64(input logic signed [63:0] a, b);
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
  function automatic logic [32:0] s_neg24(input q24_t a);
    s_neg24 = {(a == Q24_MIN), (a == Q24_MIN) ? Q24_MAX : -a};
  endfunction

  // Multiply helpers (mirror mul() and the Q8.24xQ16.48 product in P1/P2).
  // Operands are explicitly widened BEFORE multiplying (SV result-width
  // rule would otherwise truncate 32x32 to 32 bits — a real bug caught
  // during M2 development, fixed here).
  function automatic logic [32:0] s_mul24(input q24_t a, b);
    s_mul24 = s_narrow(64'(a) * 64'(b));
  endfunction

  function automatic logic [64:0] s_mul48(input q24_t a, input logic signed [63:0] b);
    s_mul48 = s_narrow48(acc128_t'(a) * acc128_t'(b));
  endfunction

  // Halve with round-half-away (mirror halve_q()).
  function automatic logic [32:0] s_halve24(input q24_t v);
    logic signed [32:0] t;
    t = 33'(v) + ((v >= 0) ? 33'sd1 : -33'sd1);
    s_halve24 = {1'b0, t[32:1]}; // shrinking: cannot overflow
  endfunction

  // -------------------------------------------------------------------------
  // Q16.48 x Q16.48 primitives (ADR-0007 M5-A; mirror fixed_update.hpp).
  //
  // FRACTIONAL-UNIT DISCIPLINE: a q48 x q48 product carries 96 fractional
  // bits, so returning to q48 means shifting 48 — NOT the 24 that s_narrow48
  // applies to a 72-fractional-bit (q24 x q48) product. Mixing those two was
  // one of five bugs the M5-C differential caught; the names now encode the
  // width each one consumes.
  // -------------------------------------------------------------------------

  // 96 fractional bits -> Q16.48, round-half-away + saturate (mirror
  // narrow96to48). Arithmetic shift so negatives floor, matching C++20.
  function automatic logic [64:0] s_narrow96to48(input acc128_t x);
    acc128_t half;
    acc128_t shifted;
    logic sat;
    q48_t v;
    half = (x >= 0) ? (acc128_t'(1) << 47) : -(acc128_t'(1) << 47);
    shifted = (x + half) >>> 48;
    sat = (shifted > acc128_t'(Q48_MAX)) || (shifted < acc128_t'(Q48_MIN));
    if (shifted > acc128_t'(Q48_MAX)) v = Q48_MAX;
    else if (shifted < acc128_t'(Q48_MIN)) v = Q48_MIN;
    else v = shifted[63:0];
    s_narrow96to48 = {sat, v};
  endfunction

  // q48 x q48 -> q48 (mirror div48's Q48-mode numerator / the LDL' dots).
  function automatic logic [64:0] s_mulq48(input q48_t a, input q48_t b);
    s_mulq48 = s_narrow96to48(acc128_t'(a) * acc128_t'(b));
  endfunction

  // Saturating Q16.48 subtract, 128-bit intermediate (mirror psub).
  // NOT a narrowing: both operands are already Q16.48, so there is no
  // fractional-bit rescale here. Using s_narrow96to48 for this collapses the
  // result toward zero (it shifts 48) and drove the N-R reciprocal's
  // recurrence to r = 0.
  function automatic logic [64:0] s_psub48(input q48_t a, input q48_t b);
    acc128_t d;
    logic sat;
    q48_t v;
    d = acc128_t'(a) - acc128_t'(b);
    sat = (d > acc128_t'(Q48_MAX)) || (d < acc128_t'(Q48_MIN));
    if (d > acc128_t'(Q48_MAX)) v = Q48_MAX;
    else if (d < acc128_t'(Q48_MIN)) v = Q48_MIN;
    else v = d[63:0];
    s_psub48 = {sat, v};
  endfunction

  // Count leading zeros, 0..64 (mirror clz64). Iterating upward lets the
  // HIGHEST set bit win, giving 63-p as the C++ does.
  function automatic logic [6:0] s_clz64(input logic [63:0] x);
    int i;
    begin
      s_clz64 = 7'd64;
      for (i = 0; i < 64; i = i + 1)
        if (x[i]) s_clz64 = 7'(63 - i);
    end
  endfunction

  // 1/x in Q16.48: clz64 normalize into [0.5, 1), 8 fixed N-R iterations
  // (seed 1.0, e0 <= 0.5 so the count is format-limited not iteration-limited),
  // then the 2^s rescale. x == 0 returns 0; callers guard and count.
  // Mirrors recip48() statement for statement, including the saturating
  // rescale (genuine rail only: 1/x overflows Q16.48 below x ~ 2^-48).
  function automatic logic [64:0] s_recip48(input q48_t x);
    logic neg_out;
    logic [63:0] ax;
    logic [6:0] clz;
    logic signed [8:0] s, sn;
    logic [5:0] amt;
    q48_t m, r, mr, two, one48, e, v;
    acc128_t rs;
    logic sat;
    int i;
    begin
      neg_out = (x < 0);
      ax = neg_out ? (~x + 64'd1) : x; // |x| unsigned; INT64_MIN safe
      clz = s_clz64(ax);
      s = 9'(clz) - 9'sd16;            // m lands in [2^47, 2^48)
      sn = -s;
      amt = (s >= 0) ? s[5:0] : sn[5:0];
      m = (s >= 0) ? q48_t'(ax << amt) : q48_t'(ax >> amt);
      two = 64'sh0002000000000000;     // 2.0 in Q16.48 == 2^49
      one48 = 64'sh0001000000000000;    // 1.0 in Q16.48 == 2^48
      r = one48;
      sat = 1'b0;
      for (i = 0; i < 8; i = i + 1) begin
        logic [64:0] p1, p2, p3;
        p1 = s_mulq48(m, r);           // m*r
        mr = p1[63:0];
        sat = sat | p1[64];
        // e = 2.0 - m*r: plain saturating q48 subtract (s_psub48), NOT a
        // narrowing — both operands are already Q16.48.
        p2 = s_psub48(two, mr);
        e = p2[63:0];
        sat = sat | p2[64];
        p3 = s_mulq48(r, e);           // r*e
        r = p3[63:0];
        sat = sat | p3[64];
      end
      // 1/|x| = 2^s / m: the rescale MUST be applied (the C++ bug that made
      // every divisor < 1.0 return 1/m).
      if (s >= 0) begin
        rs = acc128_t'(r) << amt;
        if (rs > acc128_t'(Q48_MAX)) begin
          v = Q48_MAX; sat = 1'b1;
        end else begin
          v = q48_t'(rs[63:0]);
        end
      end else begin
        v = q48_t'(r >> amt); // shrinks, cannot overflow
      end
      if (x == 0) begin
        // Mirror recip48()'s guard. Without it the N-R recurrence would see
        // m=0, double r every iteration, and saturate on the rescale.
        s_recip48 = {1'b0, 64'd0};
      end else if (!neg_out) begin
        s_recip48 = {sat, v};
      end else if (v == Q48_MIN) begin
        s_recip48 = {1'b1, Q48_MAX}; // -MIN saturates, counted
      end else begin
        s_recip48 = {sat, ~v + 64'd1};
      end
    end
  endfunction

  // Unrolled N-R inverse sqrt, 6 iterations, seed 1.0 (mirror invsqrt_nr).
  // NOTE: loop var hoisted (not for-init declared) — the -sv frontend
  // mis-elaborates for-init decls inside functions (Yosys rtlil assert).
  function automatic logic [32:0] invsqrt6(input q24_t x);
    q24_t y, t1, t2, half;
    logic s;
    q24_t THREE;
    int i;
    THREE = 32'sd50331648; // 3.0 exact
    y = 32'sd16777216;     // 1.0
    s = 1'b0;
    for (i = 0; i < 6; i++) begin
      logic [32:0] r1, r2, r3, r4;
      r1 = s_mul24(x, y); s = s | r1[32]; t1 = r1[31:0];
      r2 = s_mul24(t1, y); s = s | r2[32]; t2 = r2[31:0];
      r4 = s_sub24(THREE, t2); s = s | r4[32];
      r3 = s_halve24(r4[31:0]); half = r3[31:0]; // halve never saturates
      r1 = s_mul24(y, half); s = s | r1[32]; y = r1[31:0];
    end
    invsqrt6 = {s, y};
  endfunction

endpackage
