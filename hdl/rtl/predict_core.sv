// predict_core: fixed-point MEKF predict pipeline (ADR-0007 M2).
//
// Microarchitecture: nominal path (R, first-order exp + normalize, v/p,
// F, Phi) is COMBINATIONAL, evaluated in one NOM cycle; the two 15x15
// covariance matmuls are SEQUENTIAL through one shared product+accumulate
// datapath (3375 cycles each); Qd+symmetrize is combinational. Total:
// deterministic 6978 cycles/step. Unrolling the cheap stuff +
// sequencing the big loops minimizes bug surface and area. Fallback
// (not implemented): split NOM if M3 timing disappoints.
//
// Bit-exactness contract: every op mirrors fusion/fixed/ fixed.hpp +
// fixed_predict.hpp statement-for-statement, same order, same
// round-half-away, same saturation. 64-bit accumulations use fixed_pkg::s_add64
// chains EXACTLY like the model's acc_add (including rail saturation);
// 128-bit P accumulators use plain adds (overflow-free by the proof in
// fixed.hpp), exactly like the model. hdl/tb asserts STATE-BIT parity
// vs the C++ model every step over the 2500-step trot.
//
// No divider, no CORDIC (first-order exp needs neither — model decision).

module predict_core (
  input  logic        clk,
  input  logic        rst,       // synchronous, active-high
  input  logic        start,     // pulse in IDLE
  output logic        done,      // sticky until next start
  // register bus: sync write (IDLE/DONE only), combinational read
  input  logic [8:0]  bus_addr,
  input  logic [63:0] bus_wdata,
  input  logic        bus_we,
  output logic [63:0] bus_rdata,
  output logic [31:0] sat_count  // sticky event count, cleared on start
);
  // NOTE: compilation-unit import preferred, but the oss-cad-suite
  // frontend requires the import inside the module (kept here; Verilator
  // and Yosys -sv both accept this placement).

  // SIG2 consts, sigma^2 in Q16.48 (mirror of model derivation):
  //   gyro  1e-4   -> 28147497671
  //   accel 0.0225 -> 6333186975990
  //   bg    1e-10  -> 28147
  //   ba    1e-8   -> 2814750
  localparam fixed_pkg::q48_t SIG2_G  = 64'd28147497671;
  localparam fixed_pkg::q48_t SIG2_A  = 64'd6333186975990;
  localparam fixed_pkg::q48_t SIG2_BG = 64'd28147;
  localparam fixed_pkg::q48_t SIG2_BA = 64'd2814750;

  localparam fixed_pkg::q24_t ONE_Q = 32'sd16777216; // 1.0, avoid magic below
  localparam fixed_pkg::q24_t TWO_Q = 32'sd33554432; // 2.0 exact

  // ---------- registers ----------
  fixed_pkg::q24_t gyro[3], accel[3], dtf, gf;       // inputs
  fixed_pkg::q24_t q[4], p[3], v[3], bg[3], ba[3];   // nominal state
  fixed_pkg::q48_t P[225];                            // covariance, Q16.48
  fixed_pkg::q24_t Phi[225];                          // rebuilt every step
  // Hoisted nominal temps (stable snapshot sources for dbg regs).
  fixed_pkg::q24_t R_[9], eq_[4], nq_[4];
  fixed_pkg::q48_t P1[225], P2[225];                  // engine outputs
  fixed_pkg::q48_t qd_g, qd_a, qd_bg, qd_ba;          // Qd consts for this dt

  // DUT observability: snapshot of nominal intermediates, captured in
  // ST_NOM alongside state. Lets the testbench bisect model-vs-RTL
  // divergence without disturbing the datapath (read-only flops).
  fixed_pkg::q24_t dbg_R0, dbg_R4, dbg_R8, dbg_eq0, dbg_nq0, dbg_Phi0;
  fixed_pkg::q48_t dbg_qd_g;
  fixed_pkg::q24_t dbg_Phi9; // Phi[0][9] (F09 block) — P-rail bisect
  logic [7:0] dbg_idx; // indirected P1/P2/Phi read index (0x1f8-0x1fb)

  // matmul engine: counters + 128-bit accumulator
  logic [3:0] mm_i, mm_j, mm_k;
  logic       mm_phase; // 0: P1=Phi*P, 1: P2=P1*Phi'
  fixed_pkg::acc128_t    mm_acc;

  typedef enum logic [2:0] {
    ST_IDLE, ST_NOM, ST_P1, ST_P2, ST_QDSYM, ST_DONE
  } state_t;
  state_t state;

  logic [31:0] sat_cnt;
  logic        done_r;

  // ---------- nominal combinational datapath ----------
  // Wires from regs; captured in ST_NOM. Mirrors step_fixed order.
  fixed_pkg::q24_t  n_q[4], n_p[3], n_v[3];
  fixed_pkg::q24_t  n_Phi[225];
  fixed_pkg::q48_t  n_qd_g, n_qd_a, n_qd_bg, n_qd_ba;
  logic [31:0] nom_sat;

  always_comb begin : nominal
    fixed_pkg::q24_t w[3], av[3];
    fixed_pkg::q24_t ev[3];
    fixed_pkg::q24_t F30[9];
    int i, j, k;
    fixed_pkg::sat24_t r;
    fixed_pkg::sat24_t r2;
    fixed_pkg::sat24_t rn;
    fixed_pkg::sat24_t ri;
    fixed_pkg::sat24_t ri2;
    fixed_pkg::sat48_t r48;
    fixed_pkg::sat48_t r64;
    fixed_pkg::sat48_t rq;
    fixed_pkg::sat48_t r1;
    logic [31:0] sc;
    sc = 0;
    // Default-assign conditionally-written temps (else latches). Values
    // below are never observed: every use-site overwrites first.
    r2 = '0; rq = '0;
    n_qd_g = 0; n_qd_a = 0; n_qd_bg = 0; n_qd_ba = 0;
    // w = gyro-bg, av = accel-ba
    for (i = 0; i < 3; i++) begin
      r = fixed_pkg::s_sub24(gyro[i], bg[i]); sc += {31'b0, r.sat}; w[i] = r.v;
      r = fixed_pkg::s_sub24(accel[i], ba[i]); sc += {31'b0, r.sat}; av[i] = r.v;
    end
    // R = quat_to_mat(q): 9 entries, same formulas as model
    begin
      fixed_pkg::q24_t qw, qx, qy, qz, t, u;
      qw = q[0]; qx = q[1]; qy = q[2]; qz = q[3];
      // R_[0] = 1-2(qy^2+qz^2)
      r = fixed_pkg::s_mul24(qy, qy); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qz, qz); sc += {31'b0, r.sat};
      r = fixed_pkg::s_add24(t, r.v); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_sub24(ONE_Q, t); sc += {31'b0, r.sat}; R_[0] = r.v;
      // R_[1] = 2(qx*qy - qz*qw)
      r = fixed_pkg::s_mul24(qx, qy); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qz, qw); sc += {31'b0, r.sat}; u = r.v;
      r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; R_[1] = r.v;
      // R_[2] = 2(qx*qz + qy*qw)
      r = fixed_pkg::s_mul24(qx, qz); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qy, qw); sc += {31'b0, r.sat}; u = r.v;
      r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; R_[2] = r.v;
      // R_[3] = 2(qx*qy + qz*qw)
      r = fixed_pkg::s_mul24(qx, qy); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qz, qw); sc += {31'b0, r.sat}; u = r.v;
      r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; R_[3] = r.v;
      // R_[4] = 1-2(qx^2+qz^2)
      r = fixed_pkg::s_mul24(qx, qx); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qz, qz); sc += {31'b0, r.sat};
      r = fixed_pkg::s_add24(t, r.v); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_sub24(ONE_Q, t); sc += {31'b0, r.sat}; R_[4] = r.v;
      // R_[5] = 2(qy*qz - qx*qw)
      r = fixed_pkg::s_mul24(qy, qz); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qx, qw); sc += {31'b0, r.sat}; u = r.v;
      r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; R_[5] = r.v;
      // R_[6] = 2(qx*qz - qy*qw)
      r = fixed_pkg::s_mul24(qx, qz); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qy, qw); sc += {31'b0, r.sat}; u = r.v;
      r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; R_[6] = r.v;
      // R_[7] = 2(qy*qz + qx*qw)
      r = fixed_pkg::s_mul24(qy, qz); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qx, qw); sc += {31'b0, r.sat}; u = r.v;
      r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; R_[7] = r.v;
      // R_[8] = 1-2(qx^2+qy^2)
      r = fixed_pkg::s_mul24(qx, qx); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(qy, qy); sc += {31'b0, r.sat};
      r = fixed_pkg::s_add24(t, r.v); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_mul24(TWO_Q, t); sc += {31'b0, r.sat}; t = r.v;
      r = fixed_pkg::s_sub24(ONE_Q, t); sc += {31'b0, r.sat}; R_[8] = r.v;
    end
    nom_sat = sc;
    // --- exp: wdt, halve->ev, en2 (fixed_pkg::s_add64 chain from ONE^2), invsqrt, eq
    begin
      fixed_pkg::q24_t wdt[3], evloc[3], einv;
      logic signed [63:0] en2acc;

      for (i = 0; i < 3; i++) begin
        r = fixed_pkg::s_mul24(w[i], dtf); sc += {31'b0, r.sat}; wdt[i] = r.v;
        r = fixed_pkg::s_halve24(wdt[i]); evloc[i] = r.v; // halve: no sat possible
      end
      en2acc = 64'd281474976710656; // ONE^2 = 2^48. NOTE: powers of
      // two are written as decimal literals ONLY with this verifying
      // comment (a transposed-digits typo here cost an hour: 2^48 =
      // 281474976710656, verified by python3 -c "print(2**48)").
      for (i = 0; i < 3; i++) begin
        r64 = fixed_pkg::s_add64(en2acc, 64'(evloc[i]) * 64'(evloc[i]));
        sc += {31'b0, r64.sat}; en2acc = r64.v;
      end
      r = fixed_pkg::s_narrow(en2acc); sc += {31'b0, r.sat};
      begin

        ri = fixed_pkg::invsqrt6(r.v); sc += {31'b0, ri.sat}; einv = ri.v;
      end
      r = fixed_pkg::s_mul24(ONE_Q, einv); sc += {31'b0, r.sat}; eq_[0] = r.v;
      for (i = 0; i < 3; i++) begin
        r = fixed_pkg::s_mul24(evloc[i], einv); sc += {31'b0, r.sat}; eq_[i+1] = r.v;
      end
      for (i = 0; i < 3; i++) ev[i] = evloc[i];
      // --- qprod nq (Hamilton, left-assoc, mirror of model) ---
      begin
        fixed_pkg::q24_t q0, q1, q2, q3, e0, e1, e2, e3, t, u;
        
        q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3];
        e0 = eq_[0]; e1 = eq_[1]; e2 = eq_[2]; e3 = eq_[3];
        // nq[0] = ((q0*e0 - q1*e1) - q2*e2) - q3*e3
        r = fixed_pkg::s_mul24(q0, e0); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q1, e1); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q2, e2); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q3, e3); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; nq_[0] = r.v;
        // nq[1] = ((q0*e1 + q1*e0) + q2*e3) - q3*e2
        r = fixed_pkg::s_mul24(q0, e1); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q1, e0); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q2, e3); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q3, e2); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; nq_[1] = r.v;
        // nq[2] = ((q0*e2 + q2*e0) + q3*e1) - q1*e3
        r = fixed_pkg::s_mul24(q0, e2); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q2, e0); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q3, e1); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q1, e3); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; nq_[2] = r.v;
        // nq[3] = ((q0*e3 + q3*e0) + q1*e2) - q2*e1
        r = fixed_pkg::s_mul24(q0, e3); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q3, e0); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q1, e2); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_add24(t, u); sc += {31'b0, r.sat}; t = r.v;
        r = fixed_pkg::s_mul24(q2, e1); sc += {31'b0, r.sat}; u = r.v;
        r = fixed_pkg::s_sub24(t, u); sc += {31'b0, r.sat}; nq_[3] = r.v;
        // --- qnorm: qn2 chain, invsqrt, scale ---
        begin
          logic signed [63:0] qn2;

          qn2 = 0;
          for (i = 0; i < 4; i++) begin
            r64 = fixed_pkg::s_add64(qn2, 64'(nq_[i]) * 64'(nq_[i]));
            sc += {31'b0, r64.sat}; qn2 = r64.v;
          end
          r = fixed_pkg::s_narrow(qn2); sc += {31'b0, r.sat};
          ri2 = fixed_pkg::invsqrt6(r.v); sc += {31'b0, ri2.sat};
          for (i = 0; i < 4; i++) begin
            r = fixed_pkg::s_mul24(nq_[i], ri2.v); sc += {31'b0, r.sat}; n_q[i] = r.v;
          end
        end
      end
      // --- v/p: Ra = R*av (fixed_pkg::s_add64 chains), dv/dp ---
      begin
        fixed_pkg::q24_t Ra[3], dv, dpv, gvv;
        gvv = 0;
        for (i = 0; i < 3; i++) begin
          logic signed [63:0] sum;

          sum = 0;
          for (j = 0; j < 3; j++) begin
            r64 = fixed_pkg::s_add64(sum, 64'(R_[i*3+j]) * 64'(av[j]));
            sc += {31'b0, r64.sat}; sum = r64.v;
          end
          r = fixed_pkg::s_narrow(sum); sc += {31'b0, r.sat}; Ra[i] = r.v;
        end
        // gvec = (0,0,-gf)
        for (i = 0; i < 3; i++) begin
          fixed_pkg::q24_t gi;
          if (i == 2) begin

            rn = fixed_pkg::s_neg24(gf); sc += {31'b0, rn.sat}; gi = rn.v;
          end else gi = 32'sd0;
          r = fixed_pkg::s_add24(Ra[i], gi); sc += {31'b0, r.sat};
          r = fixed_pkg::s_mul24(r.v, dtf); sc += {31'b0, r.sat}; dv = r.v;
          r = fixed_pkg::s_add24(v[i], dv); sc += {31'b0, r.sat};
          gvv = r.v; // reuse temp: updated v
          r = fixed_pkg::s_mul24(gvv, dtf); sc += {31'b0, r.sat}; dpv = r.v;
          r = fixed_pkg::s_add24(p[i], dpv); sc += {31'b0, r.sat};
          n_v[i] = gvv;
          n_p[i] = r.v;
        end
      end
      // --- F30 = -R*skew(av): 27 MACs via fixed_pkg::s_add64 chains ---
      begin
        fixed_pkg::q24_t Sa[9];
        // skew(av): [0,-az,ay; az,0,-ax; -ay,ax,0]
        Sa[0] = 0; Sa[4] = 0; Sa[8] = 0;
      begin
          rn = fixed_pkg::s_neg24(av[2]); sc += {31'b0, rn.sat}; Sa[1] = rn.v;
          Sa[2] = av[1];
          Sa[3] = av[2];
          rn = fixed_pkg::s_neg24(av[0]); sc += {31'b0, rn.sat}; Sa[5] = rn.v;
          rn = fixed_pkg::s_neg24(av[1]); sc += {31'b0, rn.sat}; Sa[6] = rn.v;
          Sa[7] = av[0];
        end
        for (i = 0; i < 3; i++) begin
          for (j = 0; j < 3; j++) begin
            logic signed [63:0] fsum;

            fixed_pkg::sat24_t rn2;
            fsum = 0;
            for (k = 0; k < 3; k++) begin
              r64 = fixed_pkg::s_add64(fsum, 64'(R_[i*3+k]) * 64'(Sa[k*3+j]));
              sc += {31'b0, r64.sat}; fsum = r64.v;
            end
            rn2 = fixed_pkg::s_narrow(fsum); sc += {31'b0, rn2.sat};
            rn2 = fixed_pkg::s_neg24(rn2.v); sc += {31'b0, rn2.sat};
            F30[(i*3+j)] = rn2.v;
          end
        end
      end
      // --- Phi: F select + I + F*dt (mirror of model F layout) ---
      begin
        fixed_pkg::q24_t F00[9], F312[9];
        // F00 = -skew(w), mirroring the model's DOUBLE negation
        // neg(Sw[.]) where Sw itself contains neg() (exact corner
        // behavior at QMIN preserved: neg(neg(x)) != x there).
        // Layout: [0,+wz,-wy; -wz,0,+wx; +wy,-wx,0].
        F00[0] = 0;
      begin
          rn = fixed_pkg::s_neg24(w[2]); sc += {31'b0, rn.sat};
          rn = fixed_pkg::s_neg24(rn.v); sc += {31'b0, rn.sat}; F00[1] = rn.v;
          rn = fixed_pkg::s_neg24(w[1]); sc += {31'b0, rn.sat}; F00[2] = rn.v;
          rn = fixed_pkg::s_neg24(w[2]); sc += {31'b0, rn.sat}; F00[3] = rn.v;
          rn = fixed_pkg::s_neg24(w[0]); sc += {31'b0, rn.sat};
          rn = fixed_pkg::s_neg24(rn.v); sc += {31'b0, rn.sat}; F00[5] = rn.v;
          rn = fixed_pkg::s_neg24(w[1]); sc += {31'b0, rn.sat};
          rn = fixed_pkg::s_neg24(rn.v); sc += {31'b0, rn.sat}; F00[6] = rn.v;
          rn = fixed_pkg::s_neg24(w[0]); sc += {31'b0, rn.sat}; F00[7] = rn.v;
          F00[4] = 0; F00[8] = 0;
        end
        // F312 = -R
        for (i = 0; i < 9; i++) begin

          rn = fixed_pkg::s_neg24(R_[i]); sc += {31'b0, rn.sat}; F312[i] = rn.v;
        end
        for (i = 0; i < 15; i++) begin
          for (j = 0; j < 15; j++) begin
            fixed_pkg::q24_t f;
            logic fz;
            f = 0; fz = 1'b1;
            if (i < 3 && j < 3) begin f = F00[i*3+j]; fz = (f == 0); end
            else if (i < 3 && j >= 9 && j < 12 && (j-9) == i) begin
              f = 32'shff000000; fz = 1'b0; // -1.0 (-I block)
            end
            else if (i >= 3 && i < 6 && j < 3) begin f = F30[(i-3)*3+j]; fz = (f == 0); end
            else if (i >= 3 && i < 6 && j >= 12) begin f = F312[(i-3)*3+(j-12)]; fz = (f == 0); end
            else if (i >= 6 && i < 9 && j >= 3 && j < 6 && (j-3) == (i-6)) begin
              f = ONE_Q; fz = 1'b0; // +I block
            end
            if (fz) begin
              if (i == j) n_Phi[i*15+j] = ONE_Q;
              else n_Phi[i*15+j] = 0;
            end else begin
              r = fixed_pkg::s_mul24(f, dtf); sc += {31'b0, r.sat};
              if (i == j) begin

                r2 = fixed_pkg::s_add24(ONE_Q, r.v); sc += {31'b0, r2.sat};
                n_Phi[i*15+j] = r2.v;
              end else n_Phi[i*15+j] = r.v;
            end
          end
        end
      end
      // --- Qd consts: narrow48(SIG2*dtf), mirror of model ---
      begin

        rq = fixed_pkg::s_narrow48(fixed_pkg::acc128_t'(SIG2_G) * fixed_pkg::acc128_t'(dtf)); sc += {31'b0, rq.sat}; n_qd_g = rq.v;
        rq = fixed_pkg::s_narrow48(fixed_pkg::acc128_t'(SIG2_A) * fixed_pkg::acc128_t'(dtf)); sc += {31'b0, rq.sat}; n_qd_a = rq.v;
        rq = fixed_pkg::s_narrow48(fixed_pkg::acc128_t'(SIG2_BG) * fixed_pkg::acc128_t'(dtf)); sc += {31'b0, rq.sat}; n_qd_bg = rq.v;
        rq = fixed_pkg::s_narrow48(fixed_pkg::acc128_t'(SIG2_BA) * fixed_pkg::acc128_t'(dtf)); sc += {31'b0, rq.sat}; n_qd_ba = rq.v;
      end
      nom_sat = sc;
    end
  end

  // ---------- QDSYM single-entry datapath (sequential, 225 cycles) ----------
  // Mirror of model Qd+symmetrize for ONE entry (mm_i, mm_j). Sequential,
  // not combinationally unrolled: the unrolled 225-entry version
  // elaborates 450 wide read-muxes and stalls Yosys opt for 10+ minutes
  // (measured). +224 cycles/step, negligible next to 6750 matmul cycles.
  fixed_pkg::q48_t qd_res;
  logic [31:0] qd_sat1;
  always_comb begin : qdsym1
    fixed_pkg::sat48_t r1;
    fixed_pkg::q48_t vv, tt;
    fixed_pkg::acc128_t halves, h, sym;
    logic [31:0] sc;
    sc = 0;
    r1 = '0; vv = 0; tt = 0; halves = 0; h = 0; sym = 0;
    vv = P2[mm_i*15+mm_j];
    if (mm_i == mm_j) begin
      if (mm_i < 3) begin r1 = fixed_pkg::s_add48(vv, qd_g); sc += {31'b0, r1.sat}; vv = r1.v; end
      else if (mm_i < 6) begin r1 = fixed_pkg::s_add48(vv, qd_a); sc += {31'b0, r1.sat}; vv = r1.v; end
      else if (mm_i < 9) begin end // dp: none
      else if (mm_i < 12) begin r1 = fixed_pkg::s_add48(vv, qd_bg); sc += {31'b0, r1.sat}; vv = r1.v; end
      else begin r1 = fixed_pkg::s_add48(vv, qd_ba); sc += {31'b0, r1.sat}; vv = r1.v; end
    end
    tt = P2[mm_j*15+mm_i];
    halves = fixed_pkg::acc128_t'(vv) + fixed_pkg::acc128_t'(tt);
    h = (halves >= 0) ? fixed_pkg::acc128_t'(1) : -fixed_pkg::acc128_t'(1);
    // ARITHMETIC shift (see bug note in git history: >> rails negatives).
    sym = (halves + h) >>> 1;
    if (sym > fixed_pkg::acc128_t'(fixed_pkg::Q48_MAX)) begin sc += 1; qd_res = fixed_pkg::Q48_MAX; end
    else if (sym < fixed_pkg::acc128_t'(fixed_pkg::Q48_MIN)) begin sc += 1; qd_res = fixed_pkg::Q48_MIN; end
    else qd_res = sym[63:0];
    qd_sat1 = sc;
  end

  // ---------- matmul engine operand mux (comb) ----------
  // P1[i][j] = sum_k Phi[i][k]*P[k][j]; P2[i][j] = sum_k P1[i][k]*Phi[j][k].
  // a32 is always the Phi side (Q8.24), b64 the P side (Q16.48).
  fixed_pkg::q24_t eng_a;
  fixed_pkg::q48_t eng_b;
  always_comb begin
    if (mm_phase == 0) begin eng_a = Phi[mm_i*15+mm_k]; eng_b = P[mm_k*15+mm_j]; end
    else begin eng_a = Phi[mm_j*15+mm_k]; eng_b = P1[mm_i*15+mm_k]; end
  end

  // ---------- bus read (comb) ----------
  always_comb begin
    bus_rdata = 64'b0;
    if (bus_addr == 9'h000) bus_rdata = {63'b0, done_r};
    else if (bus_addr == 9'h001) bus_rdata = {32'b0, sat_cnt};
    else if (bus_addr >= 9'h010 && bus_addr <= 9'h012) bus_rdata = {32'b0, gyro[2'(bus_addr-9'h010)]};
    else if (bus_addr >= 9'h013 && bus_addr <= 9'h015) bus_rdata = {32'b0, accel[2'(bus_addr-9'h013)]};
    else if (bus_addr == 9'h016) bus_rdata = {32'b0, dtf};
    else if (bus_addr == 9'h017) bus_rdata = {32'b0, gf};
    else if (bus_addr >= 9'h020 && bus_addr <= 9'h023) bus_rdata = {32'b0, q[2'(bus_addr-9'h020)]};
    else if (bus_addr >= 9'h024 && bus_addr <= 9'h026) bus_rdata = {32'b0, p[2'(bus_addr-9'h024)]};
    else if (bus_addr >= 9'h027 && bus_addr <= 9'h029) bus_rdata = {32'b0, v[2'(bus_addr-9'h027)]};
    else if (bus_addr >= 9'h02a && bus_addr <= 9'h02c) bus_rdata = {32'b0, bg[2'(bus_addr-9'h02a)]};
    else if (bus_addr >= 9'h02d && bus_addr <= 9'h02f) bus_rdata = {32'b0, ba[2'(bus_addr-9'h02d)]};
    else if (bus_addr >= 9'h100 && bus_addr < 9'h100+225) bus_rdata = P[8'(bus_addr-9'h100)];
    else if (bus_addr == 9'h1f8) bus_rdata = {56'b0, dbg_idx};
    else if (bus_addr == 9'h1f9) bus_rdata = P1[dbg_idx];
    else if (bus_addr == 9'h1fa) bus_rdata = P2[dbg_idx];
    else if (bus_addr == 9'h1fb) bus_rdata = {32'b0, Phi[dbg_idx[7:0]]};
    // NOTE 0x1fb: Phi has 225 entries; idx >= 225 wraps mod 256
    // (debug-only; testbench keeps idx < 225).
    else if (bus_addr >= 9'h1f0 && bus_addr <= 9'h1f7) begin
      // DUT observability (nominal snapshot from ST_NOM)
      case (bus_addr[2:0])
        3'd0: bus_rdata = {32'b0, dbg_R0};
        3'd1: bus_rdata = {32'b0, dbg_R4};
        3'd2: bus_rdata = {32'b0, dbg_R8};
        3'd3: bus_rdata = {32'b0, dbg_eq0};
        3'd4: bus_rdata = {32'b0, dbg_nq0};
        3'd5: bus_rdata = {32'b0, dbg_Phi0};
        3'd7: bus_rdata = {32'b0, dbg_Phi9};
        default: bus_rdata = dbg_qd_g;
      endcase
    end
  end

  // ---------- sequential: bus writes, FSM, engine ----------
  always_ff @(posedge clk) begin
    if (rst) begin
      state <= ST_IDLE; done_r <= 1'b0; sat_cnt <= 32'b0;
      mm_i <= 0; mm_j <= 0; mm_k <= 0; mm_phase <= 0; mm_acc <= 0;
    end else begin
      // bus writes honored in IDLE only (testbench discipline)
      if (bus_we && state == ST_IDLE) begin
        if (bus_addr == 9'h1f8) dbg_idx <= bus_wdata[7:0];
        else if (bus_addr >= 9'h010 && bus_addr <= 9'h012) gyro[2'(bus_addr-9'h010)] <= bus_wdata[31:0];
        else if (bus_addr >= 9'h013 && bus_addr <= 9'h015) accel[2'(bus_addr-9'h013)] <= bus_wdata[31:0];
        else if (bus_addr == 9'h016) dtf <= bus_wdata[31:0];
        else if (bus_addr == 9'h017) gf <= bus_wdata[31:0];
        else if (bus_addr >= 9'h020 && bus_addr <= 9'h023) q[2'(bus_addr-9'h020)] <= bus_wdata[31:0];
        else if (bus_addr >= 9'h024 && bus_addr <= 9'h026) p[2'(bus_addr-9'h024)] <= bus_wdata[31:0];
        else if (bus_addr >= 9'h027 && bus_addr <= 9'h029) v[2'(bus_addr-9'h027)] <= bus_wdata[31:0];
        else if (bus_addr >= 9'h02a && bus_addr <= 9'h02c) bg[2'(bus_addr-9'h02a)] <= bus_wdata[31:0];
        else if (bus_addr >= 9'h02d && bus_addr <= 9'h02f) ba[2'(bus_addr-9'h02d)] <= bus_wdata[31:0];
        else if (bus_addr >= 9'h100 && bus_addr < 9'h100+225) P[8'(bus_addr-9'h100)] <= bus_wdata;
      end
      case (state)
        ST_IDLE: if (start) begin
          done_r <= 1'b0; sat_cnt <= 32'b0;
          mm_i <= 0; mm_j <= 0; mm_k <= 0; mm_phase <= 1'b0; mm_acc <= 0;
          state <= ST_NOM;
        end
        ST_NOM: begin
          for (int n = 0; n < 4; n++) q[n] <= n_q[n];
          for (int n = 0; n < 3; n++) begin
            p[n] <= n_p[n]; v[n] <= n_v[n];
          end
          // bg/ba untouched by predict (no update in this pipeline)
          for (int n = 0; n < 225; n++) Phi[n] <= n_Phi[n];
          qd_g <= n_qd_g; qd_a <= n_qd_a; qd_bg <= n_qd_bg; qd_ba <= n_qd_ba;
          // observability snapshot (read-only debug, no datapath effect)
          dbg_R0 <= R_[0]; dbg_R4 <= R_[4]; dbg_R8 <= R_[8];
          dbg_eq0 <= eq_[0]; dbg_nq0 <= nq_[0];
          dbg_Phi0 <= n_Phi[0]; dbg_qd_g <= n_qd_g;
          dbg_Phi9 <= n_Phi[9];
          sat_cnt <= sat_cnt + nom_sat;
          state <= ST_P1;
        end
        ST_P1, ST_P2: begin
          // one MAC per cycle; k==14 also writes back + advances
          fixed_pkg::acc128_t prod;
          fixed_pkg::sat48_t nr;
          prod = fixed_pkg::acc128_t'(eng_a) * fixed_pkg::acc128_t'(eng_b);
          if (mm_k == 14) begin
            nr = fixed_pkg::s_narrow48(mm_acc + prod);
            if (mm_phase == 0) P1[mm_i*15+mm_j] <= nr.v;
            else P2[mm_i*15+mm_j] <= nr.v;
            sat_cnt <= sat_cnt + {31'b0, nr.sat};
            mm_k <= 0;
            if (mm_j == 14) begin
              mm_j <= 0;
              if (mm_i == 14) begin
                mm_i <= 0;
                if (mm_phase == 0) mm_phase <= 1'b1;
                else state <= ST_QDSYM;
              end else mm_i <= mm_i + 1;
            end else mm_j <= mm_j + 1;
          end else begin
            mm_acc <= (mm_k == 0) ? prod : mm_acc + prod;
            mm_k <= mm_k + 1;
          end
        end
        ST_QDSYM: begin
          P[mm_i*15+mm_j] <= qd_res;
          sat_cnt <= sat_cnt + qd_sat1;
          if (mm_j == 14) begin
            mm_j <= 0;
            if (mm_i == 14) begin
              mm_i <= 0;
              state <= ST_DONE;
            end else mm_i <= mm_i + 1;
          end else mm_j <= mm_j + 1;
        end
        ST_DONE: begin
          done_r <= 1'b1;
          state <= ST_IDLE;
        end
        default: state <= ST_IDLE;
      endcase
    end
  end

endmodule
