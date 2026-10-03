// Sequenced LDL' factor + solve kernel (ADR-0007 M5-A).
//
// Bit-exact mirror of fixed_update.hpp's ldl_factor() then ldl_solve():
//   S = L D L'   (L unit-lower, D diagonal)
//   S^-1 b via forward substitution, 1/D scaling, back substitution.
// LDL' rather than Cholesky: S is SPD by construction and unit-diagonal L
// removes every square root and every solve-side division.
//
// WIDTH is the dominant real case (W=6 = 3 rows x 2 stance feet). W=3 and
// W=12 are the same code with a different constant; the M5-C study measured
// rows=6 at 90% of updates.
//
// ONE 64x64 multiplier, ONE 128-bit accumulator, reused by every dot product
// and every Newton-Raphson iteration. The study showed the reciprocal needs
// the same 64x64 width as the LDL' dots, so it costs only cycles, not area.
//
// S is stored lower-triangle only and L strict-lower only (both index with
// lt_idx/sl_idx, so no inverse mapping is ever needed except on load).
// I/O is a serialized 64-bit word bus: a flat 6x6 q48 port would be ~1700
// pads, which at sky130 I/O density would swamp the core and make the PPA
// number meaningless. Core cell area is the figure to compare.
module ldl_kernel #(
  parameter int W = 6
) (
  input  logic        clk,
  input  logic        rst_n,

  // Serialized load: W*(W+1)/2 words of S (lower triangle, row-major) then
  // W words of b. in_ready is high while the kernel is idle.
  input  logic        in_valid,
  input  logic [63:0] in_data,
  output logic        in_ready,

  input  logic        start,      // begin factor + solve

  // Serialized dump: W*(W-1)/2 words of L (strict lower, row-major) then
  // W words of D then W words of x.
  output logic        out_valid,
  output logic [63:0] out_data,
  input  logic        out_ready,

  output logic        busy,
  output logic        done,       // sticky until next start
  output logic        pd,         // every D[i] > 0
  output logic        sat         // any saturation observed
);
  // Lower-triangle index, i >= j. Used for S.
  function automatic int lt_idx(input int i, input int j);
    lt_idx = i * (i + 1) / 2 + j;
  endfunction
  // Strict-lower index, i > j. Used for L (every read and write has i > j,
  // including the transposed forms L[j][k], L[i][k] and L[k][i]).
  function automatic int sl_idx(input int i, input int j);
    sl_idx = i * (i - 1) / 2 + j;
  endfunction

  localparam int WT = W * (W + 1) / 2;   // S lower-triangle words
  localparam int WL = W * (W - 1) / 2;   // L strict-lower words
  localparam int NW_IN  = WT + W;
  localparam int NW_OUT = WL + W + W;

  typedef enum logic [4:0] {
    S_IDLE, S_LOAD, S_DINIT, S_T1, S_T2, S_LENDQ, S_DSTORE, S_LINIT,
    S_DVSET, S_DVMR, S_DVR, S_DVSCALE, S_DVFIN, S_DVMUL, S_DIVSTORE,
    S_FINIT, S_FTERM, S_FSTORE, S_SCSET,
    S_BINIT, S_BTERM, S_BSTORE, S_FIN, S_DUMP
  } st_e;
  st_e st;

  // ---- state ----
  // q48_t (SIGNED), not logic [63:0]: the accumulators form
  // acc128_t'(a) * acc128_t'(b), and casting an UNSIGNED 64-bit value to a
  // signed 128-bit one ZERO-extends in Verilog while the C++ casts int64_t and
  // SIGN-extends. With unsigned storage every negative L/S/x entry became a
  // ~2^64 positive number, so each dot-product term was ~2^64 too large and D
  // came out at 3.2 instead of 0.0927. Ports stay unsigned (OpenSTA rejects
  // `signed` port declarations -- see hdl/README).
  fixed_pkg::q48_t S_r [0:WT-1];   // lower triangle
  fixed_pkg::q48_t L_r [0:WL-1];   // strict lower
  fixed_pkg::q48_t D_r [0:W-1];
  fixed_pkg::q48_t B_r [0:W-1];   // load vector, kept across the factorization
  fixed_pkg::q48_t X_r [0:W-1];   // z then x, in place

  int ld_i, jj, ii, ri, rj, nb, kk, it;
  logic div_ld;                // division destination: 0 -> L, 1 -> x
  logic div_sel;               // divisor select: 0 -> D[jj] (factor), 1 -> D[ii]
  fixed_pkg::q48_t absd;

  fixed_pkg::acc128_t acc;
  fixed_pkg::q48_t  num, recip, m_r, r_r, lt_r, mr_r, e_r;
  logic [64:0] p1;
  logic sat_r;

  // Seed the accumulator with a Q16.48 value promoted to the 96-fractional-bit
  // working unit. Written as a concatenation, NOT `acc128_t'(x) << 48`: the
  // cast-then-shift form did not size to 128 bits and left the accumulator's
  // high bits zero, which made every pivot read as non-positive.
  function automatic fixed_pkg::acc128_t acc_seed48(input logic [63:0] x);
    // 16 + 64 + 48 = 128 exactly (a bare {x, 48'd0} is a 112-bit replicate and
    // trips WIDTHEXPAND, which this project treats as an error).
    //
    // SIGN-EXTEND before shifting, which a zero-extended concatenation does
    // NOT do: the C++ is `acc128(x) << FRAC48`, and acc128() sign-extends a
    // negative int64 to 128 bits BEFORE the shift, so bits [127:112] carry x's
    // sign. With zeros there, every negative S entry produced a POSITIVE
    // accumulator and narrow96to48 saturated to the rail -- which is how the
    // first real-matrix run failed with L/D/x all at7fff... . Written as an
    // assignment (not {16{x[63]},...}) because Verilator rejects the nested
    // replication in that position.
    fixed_pkg::acc128_t sx;
    sx = 128'($signed(x));   // explicit sign-extending size cast
    acc_seed48 = sx << 48;
  endfunction

  // acc = acc - a*b, raw 96-fractional-bit product, no narrowing (the dot
  // products narrow exactly once at the end, matching the C++).
  function automatic void acc_sub(input fixed_pkg::q48_t a,
                                 input fixed_pkg::q48_t b);
    acc = acc - fixed_pkg::acc128_t'(a) * fixed_pkg::acc128_t'(b);
  endfunction

  // ---- division setup, all combinational from the selected pivot ----
  // Named temps throughout: SystemVerilog forbids part-selects of
  // expressions like (-sh)[5:0], and Verilator rejects them outright.
  logic [63:0]        dv_abs;
  logic [6:0]         dv_clz;
  logic signed [8:0]  dv_sh, dv_shn;
  logic [5:0]         dv_amt;
  logic [63:0]        dv_m;
  logic signed [63:0] dv_den;   // q48: signed, so the < 0 test is meaningful
  always_comb begin
    dv_den = div_sel ? D_r[ii] : D_r[jj];
    dv_abs = (dv_den < 0) ? (~dv_den + 64'd1) : dv_den;  // |den| unsigned
    dv_clz = fixed_pkg::s_clz64(dv_abs);
    dv_sh  = 9'(dv_clz) - 9'sd16;       // m lands in [2^47, 2^48)
    dv_shn = -dv_sh;
    dv_amt = (dv_sh >= 0) ? dv_sh[5:0] : dv_shn[5:0];
    dv_m   = (dv_sh >= 0) ? (dv_abs << dv_amt) : (dv_abs >> dv_amt);
  end
  // 1/|den| = 2^sh / m, with saturation on the way up.
  logic [63:0]        rs_h;
  logic               rs_sat;
  always_comb begin
    if (dv_sh >= 0) begin
      if ((fixed_pkg::acc128_t'(r_r) << dv_amt) >
          fixed_pkg::acc128_t'(64'sh7fffffffffffffff)) begin
        rs_h   = 64'sh7fffffffffffffff;
        rs_sat = 1'b1;
      end else begin
        rs_h   = fixed_pkg::q48_t'((fixed_pkg::acc128_t'(r_r) << dv_amt));
        rs_sat = 1'b0;
      end
    end else begin
      rs_h   = fixed_pkg::q48_t'(r_r >> dv_amt);  // shrinks: cannot overflow
      rs_sat = 1'b0;
    end
  end

  // narrow96to48(acc) as a WIRE. S_DSTORE needs to test the sign of this value
  // in the same cycle it stores it; reading a nonblocking-assigned register
  // there would test the PREVIOUS value (0 on entry), so every pivot looked
  // non-positive and the kernel bailed to non-PD on the first column.
  logic [64:0] narrow_acc;
  assign narrow_acc = fixed_pkg::s_narrow96to48(acc);

  // Load-word index -> (row, col) in the lower triangle.
  int ld_row, ld_col;
  always_comb begin
    ld_row = 0;
    ld_col = ld_i;
    for (int i = 0; i < W; ++i)
      if ((ld_i >= i * (i + 1) / 2) && (ld_i < (i + 1) * (i + 2) / 2)) begin
        ld_row = i;
        ld_col = ld_i - i * (i + 1) / 2;
      end
  end

  // ---- the single 64x64 multiplier's current operands, per state ----
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE;
      ld_i <= 0; jj <= 0; ii <= 0; ri <= 0; rj <= 0; nb <= 0; kk <= 0; it <= 0;
      div_ld <= 1'b0; div_sel <= 1'b0; absd <= 64'd0;
      acc <= '0; num <= '0; recip <= '0; m_r <= '0; r_r <= '0;
      lt_r <= '0; mr_r <= '0; e_r <= '0; p1 <= '0; sat_r <= 1'b0;
      busy <= 1'b0; done <= 1'b0; pd <= 1'b0; sat <= 1'b0;
      out_valid <= 1'b0;
    end else begin
      sat_r <= 1'b0;
      case (st)
        // ------------------------------------------------------ load / idle
        S_IDLE: begin
          out_valid <= 1'b0;
          if (start) begin
            ld_i <= 0; jj <= 0; ii <= 0; kk <= 0; it <= 0;
            pd <= 1'b1; sat <= 1'b0; busy <= 1'b1; done <= 1'b0;
            st <= S_LOAD;
          end
        end
        S_LOAD: begin
          if (in_valid) begin
            if (ld_i < WT) S_r[ld_i] <= in_data;
            else begin
              B_r[ld_i - WT] <= in_data;
              // Seed X_r too: on the non-PD path the kernel skips the solve,
              // and ldl_factor()'s caller then sees x == b unchanged.
              X_r[ld_i - WT] <= in_data;
            end
            if (ld_i == NW_IN - 1) begin
              ld_i <= 0; jj <= 0; ii <= 0; kk <= 0;
              acc <= acc_seed48(S_r[lt_idx(0,0)]);
              // D[0] = narrow96(S[0][0]<<48) with an EMPTY correction sum, so
              // skip the term loop entirely (entering it would subtract one
              // bogus L[0][0]*D[0] term from never-written storage).
              st <= (W > 1) ? S_DSTORE : S_FINIT;
            end else begin
              ld_i <= ld_i + 1;
            end
          end
        end

        // -------------------- D[j] = narrow96(S[j][j]<<48 - sum_k L[j][k]*ld)
        // L[i][j] = div48(narrow96(S[i][j]<<48 - sum_k L[i][k]*ld_k*L[j][k]), D[j])
        // with ld_k = narrow96(L[ri][k] * D[k]). Note the ld uses row ri but the
        // SUBTRACTION uses row rj, and the term count is jj in both cases --
        // the C++ is explicit about this and getting it wrong silently
        // changes the sum length from jj to ii.
        S_DINIT: begin
          acc  <= acc_seed48(S_r[lt_idx(jj,jj)]);
          ri   <= jj; rj <= jj; nb <= jj; kk <= 0;
          st   <= (jj > 0) ? S_T1 : S_DSTORE;
        end
        S_T1: begin
          p1 <= fixed_pkg::s_mulq48(L_r[sl_idx(ri,kk)], D_r[kk]);
          st <= S_T2;
        end
        S_T2: begin
          // acc -= L[rj][k] * narrow96(L[ri][k]*D[k])
          // lt = narrow96(L[ri][k]*D[k]) can be NEGATIVE, and p1[63:0] is an
          // unsigned part-select: casting it straight to acc128_t zero-extends.
          // $signed first, so the cast sign-extends as the C++ int64_t cast does.
          lt_r <= $signed(p1[63:0]);
          acc  <= acc - fixed_pkg::acc128_t'(L_r[sl_idx(rj,kk)])
                             * fixed_pkg::acc128_t'($signed(p1[63:0]));
          sat_r <= p1[64];
          kk <= kk + 1;
          st <= (kk + 1 < nb) ? S_T1 : ((ri == rj) ? S_DSTORE : S_LENDQ);
        end
        // common tail: the corrections are done, narrow and divide
        S_LENDQ: begin
          st <= S_DVSET;   // acc already holds S[i][j]<<48 - corrections
        end
        S_DSTORE: begin
          D_r[jj] <= fixed_pkg::q48_t'(narrow_acc[63:0]);
          sat_r <= narrow_acc[64];
          if (!(narrow_acc[63:0] > 0)) begin
            pd <= 1'b0;            // mirror ldl_factor's early return
            st <= S_FIN;
          end else if (jj + 1 < W) begin
            ii <= jj + 1; kk <= 0;
            st <= S_LINIT;         // L[i][jj] for i = jj+1 .. W-1
          end else begin
            jj <= W; kk <= 0;
            st <= S_FINIT;         // factorization complete
          end
        end

        // ---------------- L[i][j] = div48(narrow96(S[i][j]<<48 - sum), D[j])
        S_LINIT: begin
          acc <= acc_seed48(S_r[lt_idx(ii,jj)]);
          ri <= ii; rj <= jj; nb <= jj; kk <= 0;
          st <= (jj > 0) ? S_T1 : S_LENDQ;
        end

        // ------------------------- division: recip48 then multiply
        S_DVSET: begin
          num <= fixed_pkg::q48_t'(narrow_acc[63:0]);
          sat_r <= narrow_acc[64];
          r_r  <= 64'sh0001000000000000;   // 1.0 in Q16.48
          it   <= 0;
          div_ld <= 1'b0; div_sel <= 1'b0;  // divisor = D[jj]
          st <= S_DVMR;
        end
        S_DVMR: begin
          // iteration: mr = m * r  (m resolved combinationally from dv_*)
          m_r  <= dv_m;
          absd <= dv_abs;
          mr_r <= fixed_pkg::q48_t'(fixed_pkg::s_mulq48(dv_m, r_r)[63:0]);
          st <= S_DVR;
        end
        S_DVR: begin
          // e = 2.0 - m*r is a plain saturating q48 subtract (NOT a narrowing),
          // then the single multiplier forms r*e in the same cycle.
          e_r <= fixed_pkg::q48_t'(fixed_pkg::s_psub48(
                   64'sh0002000000000000, mr_r)[63:0]);
          p1  <= fixed_pkg::s_mulq48(
                   r_r, fixed_pkg::q48_t'(fixed_pkg::s_psub48(
                     64'sh0002000000000000, mr_r)[63:0]));
          st <= S_DVSCALE;
        end
        S_DVSCALE: begin
          if (it < 7) begin
            r_r <= fixed_pkg::q48_t'(p1[63:0]);   // next N-R iterate
            it  <= it + 1;
            st  <= S_DVMR;
          end else begin
            r_r <= fixed_pkg::q48_t'(p1[63:0]);
            st <= S_DVFIN;                        // rescale is combinational
          end
        end
        S_DVFIN: begin
          recip <= fixed_pkg::q48_t'(rs_h);
          sat_r <= sat_r | rs_sat;
          st <= S_DVMUL;
        end
        S_DVMUL: begin
          p1 <= fixed_pkg::s_mulq48(num, recip);
          sat_r <= sat_r | p1[64];
          st <= S_DIVSTORE;
        end
        S_DIVSTORE: begin
          sat_r <= p1[64];
          if (!div_ld) begin
            L_r[sl_idx(ii,jj)] <= fixed_pkg::q48_t'(p1[63:0]);
            if (ii + 1 < W) begin
              ii <= ii + 1;
              st <= S_LINIT;
            end else begin
              if (jj + 1 < W) begin
                jj <= jj + 1;
                st <= S_DINIT;
              end else begin
                jj <= W; kk <= 0;
                st <= S_FINIT;
              end
            end
          end else begin
            // scale step: x[i] = z[i] / D[i]
            X_r[ii] <= fixed_pkg::q48_t'(p1[63:0]);
            if (ii + 1 < W) begin
              ii <= ii + 1;
              st <= S_SCSET;
            end else begin
              ii <= W - 1;
              st <= S_BINIT;
            end
          end
        end

        // ------------------------- forward substitution L z = b
        S_FINIT: begin
          ii <= 0; kk <= 0;
          acc <= acc_seed48(B_r[0]);
          st <= (W > 1) ? S_FTERM : S_FSTORE;
          // (i=0 enters S_FTERM with kk=0 and must skip the sum; handled by
          // S_FTERM's own guard below)
        end
        S_FTERM: begin
          // acc -= L[i][k] * X[k]  (no intermediate narrowing, as in C++)
          if (kk >= ii) begin
            st <= S_FSTORE;           // i == 0: empty sum
          end else begin
            acc <= acc - fixed_pkg::acc128_t'(L_r[sl_idx(ii,kk)])
                         * fixed_pkg::acc128_t'(X_r[kk]);
            kk <= kk + 1;
            st <= (kk + 1 < ii) ? S_FTERM : S_FSTORE;
          end
        end
        S_FSTORE: begin
          // narrow_acc (wire), not a freshly nonblocking-assigned p1: storing
          // p1[63:0] in the same cycle it is assigned stores the PREVIOUS
          // value, which is what left x close to but not equal to the model.
          sat_r <= narrow_acc[64];
          X_r[ii] <= fixed_pkg::q48_t'(narrow_acc[63:0]);
          if (ii + 1 < W) begin
            ii <= ii + 1; kk <= 0;
            acc <= acc_seed48(B_r[ii+1]);
            st <= S_FTERM;
          end else begin
            ii <= 0;
            st <= S_SCSET;
          end
        end
        S_SCSET: begin
          num     <= X_r[ii];
          div_ld  <= 1'b1;
          div_sel <= 1'b1;
          r_r     <= 64'sh0001000000000000;
          it      <= 0;
          st      <= S_DVMR;
        end

        // ------------------------- back substitution L' x = z
        S_BINIT: begin
          acc <= acc_seed48(X_r[ii]);
          kk  <= ii + 1;
          st  <= S_BTERM;
        end
        S_BTERM: begin
          if (kk >= W) begin
            sat_r <= narrow_acc[64];          // wire, for the same reason
            X_r[ii] <= fixed_pkg::q48_t'(narrow_acc[63:0]);
            st <= S_BSTORE;
          end else begin
            acc <= acc - fixed_pkg::acc128_t'(L_r[sl_idx(kk,ii)])
                         * fixed_pkg::acc128_t'(X_r[kk]);
            kk <= kk + 1;
            st <= S_BTERM;
          end
        end
        S_BSTORE: begin
          if (ii == 0) begin
            st <= S_FIN;
          end else begin
            ii <= ii - 1;
            st <= S_BINIT;
          end
        end

        // ------------------------------------------------------ finish / dump
        S_FIN: begin
          busy <= 1'b0;
          ld_i <= 0;
          done <= 1'b1;
          out_valid <= 1'b1;
          st <= S_DUMP;
        end
        S_DUMP: begin
          if (out_ready) begin
            ld_i <= ld_i + 1;
            if (ld_i + 1 >= NW_OUT) begin
              out_valid <= 1'b0;
              ld_i <= 0;
              st <= S_IDLE;
            end
          end
        end
        default: st <= S_IDLE;
      endcase
      sat <= sat | sat_r;
    end
  end

  assign in_ready = (st == S_IDLE);

  // Dump mux: L strict-lower, then D, then x.
  always_comb begin
    if (ld_i < WL)            out_data = L_r[ld_i];
    else if (ld_i < WL + W)   out_data = D_r[ld_i - WL];
    else                      out_data = X_r[ld_i - (WL + W)];
  end
endmodule