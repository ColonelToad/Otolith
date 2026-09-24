// predict_core: fixed-point MEKF predict pipeline (ADR-0007 M2 rev2).
//
// Microarchitecture rev2 (M3 area-driven redesign): rev1's fully-unrolled
// nominal path (~200 multipliers, ~1.4M cells) cannot fit any ECP5
// (~500k+ LUTs needed vs 84k). This revision sequences ALL multiplies
// through small per-block shared datapaths (~12 mults total); register
// files, bus map, start/done/sat semantics, debug interface, P1/P2
// engine, and QDSYM loop are UNCHANGED. Deterministic ~7.7k cycles/step
// (matmuls still dominate). Bit-parity re-verified after rewrite.
//
// Per-block pattern: entry/phase counters + operand-select comb driving
// the block's 1-2 multipliers; results captured to temps/state. Every
// micro-step mirrors one model op (same function, same order, same
// rounding/saturation). F00/F312/skew arrangements stay combinational
// wires (no mults); flags counted once in FARR state. Shared scratch
// temps t0-t3/t64 (blocks exclusive). ISQ shared invsqrt subroutine
// (call/return via retst). No divider, no CORDIC.

module predict_core (
  input  logic        clk,
  input  logic        rst,       // synchronous, active-high
  input  logic        start,     // pulse in IDLE
  output logic        done,      // sticky until next start
  // register bus: sync write (IDLE only), combinational read
  input  logic [8:0]  bus_addr,
  input  logic [63:0] bus_wdata,
  input  logic        bus_we,
  output logic [63:0] bus_rdata,
  output logic [31:0] sat_count  // sticky event count, cleared on start
);
  // SIG2 consts, sigma^2 in Q16.48 (mirror of model derivation):
  //   gyro  1e-4   -> 28147497671
  //   accel 0.0225 -> 6333186975990
  //   bg    1e-10  -> 28147
  //   ba    1e-8   -> 2814750
  localparam logic signed [63:0] SIG2_G  = 64'd28147497671;
  localparam logic signed [63:0] SIG2_A  = 64'd6333186975990;
  localparam logic signed [63:0] SIG2_BG = 64'd28147;
  localparam logic signed [63:0] SIG2_BA = 64'd2814750;

  localparam logic signed [31:0] ONE_Q = 32'sd16777216;
  localparam logic signed [31:0] TWO_Q = 32'sd33554432;
  localparam logic signed [31:0] THREE_Q = 32'sd50331648;

  // ---------- registers ----------
  logic signed [31:0] gyro[3], accel[3], dtf, gf;   // inputs
  logic signed [31:0] q[4], p[3], v[3], bg[3], ba[3]; // nominal state
  logic signed [63:0] P[225];                        // covariance Q16.48
  logic signed [31:0] Phi[225];                      // rebuilt every step
  logic signed [63:0] P1[225], P2[225];              // engine outputs
  logic signed [31:0] Rr[9];                         // rotation (sequenced)
  logic signed [31:0] F30r[9];                       // F30 block (sequenced)
  logic signed [31:0] eqr[4];                        // exp result (sequenced)
  logic signed [31:0] nqr[4];                        // quat product (sequenced)
  logic signed [31:0] w_[3], av_[3];                 // bias-corrected
  logic signed [31:0] ev_[3];                        // half-angles
  logic signed [63:0] qd_g, qd_a, qd_bg, qd_ba;      // Qd consts

  // shared scratch (blocks exclusive by construction)
  logic signed [31:0] t0, t1, t2, t3, gv_;
  logic signed [63:0] t64, t64b;
  logic signed [31:0] isqx, isqy; // ISQ in/out
  logic [2:0] isqi, isqp;         // ISQ dedicated counters

  // DUT observability (unchanged interface).
  logic signed [31:0] dbg_R0, dbg_R4, dbg_R8, dbg_eq0, dbg_nq0, dbg_Phi0;
  logic signed [63:0] dbg_qd_g;
  logic signed [31:0] dbg_Phi9;
  logic [7:0] dbg_idx;

  // matmul engine state (unchanged from rev1)
  logic [3:0] mm_i, mm_j, mm_k;
  logic       mm_phase;
  logic signed [127:0] mm_acc;

  // nominal sequencer counters (shared, blocks exclusive)
  logic [3:0] ne;   // entry/index (0..14)
  logic [3:0] nph;  // phase (0..11: VP rows need 12 steps)

  typedef enum logic [4:0] {
    ST_IDLE, ST_WSEQ, ST_RSEQ, ST_EXPSEQ, ST_EXP2, ST_QPRODSEQ,
    ST_QNORMSEQ, ST_QNORM2, ST_VPSEQ, ST_F30SEQ, ST_FARR, ST_PHISEQ,
    ST_ISQ, ST_P1, ST_P2, ST_QDSYM, ST_DONE
  } state_t;
  state_t state;
  state_t retst; // ISQ return state

  logic [31:0] sat_cnt;
  logic        done_r;

  // F-arrangement + skew comb wires (no mults; from stable regs w_/R_/av_).
  // Mirror of model sk() + F00/F312 construction EXACTLY, including
  // double negation (neg(neg(x)) != x at QMIN) and per-call flags:
  //   Sw = skew(w): 3 negs; F00[i] = neg(Sw[i]): 6 negs.
  //   Sa = skew(av): 3 negs.  F312[i] = neg(R[i]): 9 negs.
  // Flags (21 total) exposed for once-per-step counting in ST_FARR.
  logic signed [31:0] Sw_[9], F00w[9], Saw_[9], F312w[9];
  logic [20:0] farr_flags;
  always_comb begin
    fixed_pkg::sat24_t rn;
    Sw_[0] = 0;
    rn = fixed_pkg::s_neg24(w_[2]); Sw_[1] = rn.v; farr_flags[0] = rn.sat;
    Sw_[2] = w_[1];
    Sw_[3] = w_[2];
    Sw_[4] = 0;
    rn = fixed_pkg::s_neg24(w_[0]); Sw_[5] = rn.v; farr_flags[1] = rn.sat;
    rn = fixed_pkg::s_neg24(w_[1]); Sw_[6] = rn.v; farr_flags[2] = rn.sat;
    Sw_[7] = w_[0];
    Sw_[8] = 0;
    rn = fixed_pkg::s_neg24(Sw_[1]); F00w[1] = rn.v; farr_flags[3] = rn.sat;
    rn = fixed_pkg::s_neg24(Sw_[2]); F00w[2] = rn.v; farr_flags[4] = rn.sat;
    rn = fixed_pkg::s_neg24(Sw_[3]); F00w[3] = rn.v; farr_flags[5] = rn.sat;
    rn = fixed_pkg::s_neg24(Sw_[5]); F00w[5] = rn.v; farr_flags[6] = rn.sat;
    rn = fixed_pkg::s_neg24(Sw_[6]); F00w[6] = rn.v; farr_flags[7] = rn.sat;
    rn = fixed_pkg::s_neg24(Sw_[7]); F00w[7] = rn.v; farr_flags[8] = rn.sat;
    F00w[0] = 0; F00w[4] = 0; F00w[8] = 0;
    Saw_[0] = 0;
    rn = fixed_pkg::s_neg24(av_[2]); Saw_[1] = rn.v; farr_flags[9] = rn.sat;
    Saw_[2] = av_[1];
    Saw_[3] = av_[2];
    Saw_[4] = 0;
    rn = fixed_pkg::s_neg24(av_[0]); Saw_[5] = rn.v; farr_flags[10] = rn.sat;
    rn = fixed_pkg::s_neg24(av_[1]); Saw_[6] = rn.v; farr_flags[11] = rn.sat;
    Saw_[7] = av_[0];
    Saw_[8] = 0;
    for (int m = 0; m < 9; m++) begin
      rn = fixed_pkg::s_neg24(Rr[m]); F312w[m] = rn.v; farr_flags[12+m] = rn.sat;
    end
  end
  // Qd consts comb (captured in ST_FARR with flags).
  fixed_pkg::q48_t Qdw_g, Qdw_a, Qdw_bg, Qdw_ba;
  logic [3:0] qd_flags;
  always_comb begin
    fixed_pkg::sat48_t rq;
    rq = fixed_pkg::s_narrow48(fixed_pkg::acc128_t'(SIG2_G) * fixed_pkg::acc128_t'(dtf));
    Qdw_g = rq.v; qd_flags[0] = rq.sat;
    rq = fixed_pkg::s_narrow48(fixed_pkg::acc128_t'(SIG2_A) * fixed_pkg::acc128_t'(dtf));
    Qdw_a = rq.v; qd_flags[1] = rq.sat;
    rq = fixed_pkg::s_narrow48(fixed_pkg::acc128_t'(SIG2_BG) * fixed_pkg::acc128_t'(dtf));
    Qdw_bg = rq.v; qd_flags[2] = rq.sat;
    rq = fixed_pkg::s_narrow48(fixed_pkg::acc128_t'(SIG2_BA) * fixed_pkg::acc128_t'(dtf));
    Qdw_ba = rq.v; qd_flags[3] = rq.sat;
  end

  // Flag-sum wires for ST_FARR (combinational popcount of the 21
  // arrangement-neg flags + 4 Qd narrow flags; exact model-flag parity).
  logic [4:0] farr_sum;
  logic [2:0] qdsum;
  always_comb begin
    int m;
    farr_sum = 0;
    for (m = 0; m < 21; m++) farr_sum += {4'b0, farr_flags[m]};
    qdsum = 0;
    for (m = 0; m < 4; m++) qdsum += {2'b0, qd_flags[m]};
  end

  // ---------- nominal micro-sequencer + engine + bus ----------
  // Every micro-step mirrors one model op (same function, same order,
  // same rounding/saturation). Counters reset on every block transition.
  // sat flags added per micro-step; shared temps (t0-t3/t64/t64b) are
  // exclusive per block-phase by construction.
  always_ff @(posedge clk) begin
    if (rst) begin
      state <= ST_IDLE; done_r <= 1'b0; sat_cnt <= 32'b0;
      mm_i <= 0; mm_j <= 0; mm_k <= 0; mm_phase <= 0; mm_acc <= 0;
      ne <= 0; nph <= 0; isqi <= 0; isqp <= 0;
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
          ne <= 0; nph <= 0; isqi <= 0; isqp <= 0;
          state <= ST_WSEQ;
        end
        // WSEQ: w/av bias correction + gvec const (7 cycles)
        ST_WSEQ: begin
          fixed_pkg::sat24_t r;
          if (ne < 3) begin
            r = fixed_pkg::s_sub24(gyro[ne[1:0]], bg[ne[1:0]]);
            w_[ne[1:0]] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; ne <= ne + 1;
          end else if (ne < 6) begin
            r = fixed_pkg::s_sub24(accel[ne-3], ba[ne-3]);
            av_[ne-3] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; ne <= ne + 1;
          end else begin
            r = fixed_pkg::s_neg24(gf);
            gv_ <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
            ne <= 0; nph <= 0; state <= ST_RSEQ;
          end
        end
        // RSEQ: R[9], 5 phases x 9 entries = 45 cycles.
        // Entry map (a1,b1,a2,b2,addsub,finalsub) mirrors R formulas:
        //   R0:(qy,qy),(qz,qz)+1- R1:(qx,qy),(qz,qw)-+ R2:(qx,qz),(qy,qw)++
        //   R3:(qx,qy),(qz,qw)++ R4:(qx,qx),(qz,qz)+1- R5:(qy,qz),(qx,qw)-+
        //   R6:(qx,qz),(qy,qw)-+ R7:(qy,qz),(qx,qw)++ R8:(qx,qx),(qy,qy)+1-
        ST_RSEQ: begin
          fixed_pkg::sat24_t r;
          logic signed [31:0] a1, b1, a2, b2;
          logic sadd, fsub;
          a1 = 0; b1 = 0; a2 = 0; b2 = 0; sadd = 0; fsub = 0;
          case (ne)
            // Convention: q[0]=w,q[1]=x,q[2]=y,q[3]=z (wxyz storage).
            // Formulas verified against quat_to_mat (Eigen convention).
            0: begin a1=q[2]; b1=q[2]; a2=q[3]; b2=q[3]; sadd=0; fsub=0; end // R0
            1: begin a1=q[1]; b1=q[2]; a2=q[3]; b2=q[0]; sadd=1; fsub=1; end // R1
            2: begin a1=q[1]; b1=q[3]; a2=q[2]; b2=q[0]; sadd=0; fsub=1; end // R2
            3: begin a1=q[1]; b1=q[2]; a2=q[3]; b2=q[0]; sadd=0; fsub=1; end // R3
            4: begin a1=q[1]; b1=q[1]; a2=q[3]; b2=q[3]; sadd=0; fsub=0; end // R4
            5: begin a1=q[2]; b1=q[3]; a2=q[1]; b2=q[0]; sadd=1; fsub=1; end // R5
            6: begin a1=q[1]; b1=q[3]; a2=q[2]; b2=q[0]; sadd=1; fsub=1; end // R6
            7: begin a1=q[2]; b1=q[3]; a2=q[1]; b2=q[0]; sadd=0; fsub=1; end // R7
            8: begin a1=q[1]; b1=q[1]; a2=q[2]; b2=q[2]; sadd=0; fsub=0; end // R8
            default: begin end
          endcase
          case (nph)
            0: begin r = fixed_pkg::s_mul24(a1, b1); t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 1; end
            1: begin r = fixed_pkg::s_mul24(a2, b2); t1 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 2; end
            2: begin
                 if (sadd) r = fixed_pkg::s_sub24(t0, t1);
                 else r = fixed_pkg::s_add24(t0, t1);
                 t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 3;
               end
            3: begin r = fixed_pkg::s_mul24(TWO_Q, t0); t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 4; end
            4: begin
                 if (fsub) begin Rr[ne] <= t0; end
                 else begin r = fixed_pkg::s_sub24(ONE_Q, t0); Rr[ne] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; end
                 nph <= 0;
                 if (ne == 8) begin ne <= 0; state <= ST_EXPSEQ; end
                 else ne <= ne + 1;
               end
            default: begin nph <= 0; end
          endcase
        end
        // EXPSEQ: wdt/ev loop, en2 chain, narrow, ISQ call.
        // ne: 0..2 wdt/ev (nph 0..1), 3 en2-init, 4..6 en2 chain
        // (nph 0..1), 7 narrow, 8 ISQ call. Total 6+1+6+1+1 = 15.
        ST_EXPSEQ: begin
          fixed_pkg::sat24_t r;
          fixed_pkg::sat48_t r64;
          if (ne < 3) begin
            if (nph == 0) begin
              r = fixed_pkg::s_mul24(w_[ne[1:0]], dtf);
              t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 1;
            end else begin
              r = fixed_pkg::s_halve24(t0);
              ev_[ne[1:0]] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
              ne <= ne + 1; nph <= 0;
            end
          end else if (ne == 3) begin
            t64 <= 64'd281474976710656; // ONE^2 = 2^48 exact (verified literal)
            ne <= 4;
          end else if (ne < 7) begin
            if (nph == 0) begin
              t64b <= 64'(ev_[ne-4]) * 64'(ev_[ne-4]);
              nph <= 1;
            end else begin
              r64 = fixed_pkg::s_add64(t64, t64b);
              t64 <= r64.v; sat_cnt <= sat_cnt + {31'b0, r64.sat};
              ne <= ne + 1; nph <= 0;
            end
          end else if (ne == 7) begin
            r = fixed_pkg::s_narrow(t64);
            t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; ne <= 8;
          end else begin // ne == 8: ISQ call
            isqx <= t0; retst <= ST_EXP2;
            isqi <= 0; isqp <= 0;
            ne <= 0; nph <= 0; state <= ST_ISQ;
          end
        end
        // EXP2: eq loop (ne 0..3): eq[0]=1*einv, eq[1..3]=ev*einv.
        ST_EXP2: begin
          fixed_pkg::sat24_t r;
          if (ne == 0) begin
            r = fixed_pkg::s_mul24(ONE_Q, isqy);
            eqr[0] <= r.v;
          end else begin
            r = fixed_pkg::s_mul24(ev_[ne-1], isqy);
            eqr[ne[1:0]] <= r.v;
          end
          sat_cnt <= sat_cnt + {31'b0, r.sat};
          if (ne == 3) begin ne <= 0; nph <= 0; state <= ST_QPRODSEQ; end
          else ne <= ne + 1;
        end
        // ISQ: shared N-R invsqrt (6 iters x 5 ops + init + exit = 32).
        // In: isqx. Out: isqy. Clobbers t0-t3 (callers hold nothing live).
        ST_ISQ: begin
          fixed_pkg::sat24_t r;
          if (isqi == 0) begin
            t0 <= ONE_Q; // y init = 1.0
            isqi <= 1; isqp <= 0;
          end else if (isqi < 7) begin
            case (isqp)
              0: begin r = fixed_pkg::s_mul24(isqx, t0); t1 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; isqp <= 1; end
              1: begin r = fixed_pkg::s_mul24(t1, t0); t2 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; isqp <= 2; end
              2: begin r = fixed_pkg::s_sub24(THREE_Q, t2); t3 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; isqp <= 3; end
              3: begin r = fixed_pkg::s_halve24(t3); t3 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; isqp <= 4; end
              4: begin r = fixed_pkg::s_mul24(t0, t3); t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; isqp <= 0; isqi <= isqi + 1; end
              default: begin isqp <= 0; end
            endcase
          end else begin // isqi == 7: exit
            isqy <= t0;
            ne <= 0; nph <= 0;
            state <= retst;
          end
        end
        // QPRODSEQ: nq[4], left-assoc Hamilton, 7 ops x 4 = 28 cycles.
        // Per entry (mirror of model, verified formulas):
        //   nq0: (q0e0-q1e1)-... : (q0,e0),(q1,e1)-,(q2,e2)-,(q3,e3)-
        //   nq1: (q0,e1),(q1,e0)+,(q2,e3)+,(q3,e2)-
        //   nq2: (q0,e2),(q2,e0)+,(q3,e1)+,(q1,e3)-
        //   nq3: (q0,e3),(q3,e0)+,(q1,e2)+,(q2,e1)-
        // Ops: p0-1 mults, p2 add/sub, p3 mult, p4 add/sub, p5 mult,
        // p6 add/sub + writeback.
        ST_QPRODSEQ: begin
          fixed_pkg::sat24_t r;
          logic signed [31:0] m0a, m0b, m1a, m1b, m2a, m2b, m3a, m3b;
          logic s1, s2, s3; // 0=add,1=sub for the three combines
          m0a = 0; m0b = 0; m1a = 0; m1b = 0;
          m2a = 0; m2b = 0; m3a = 0; m3b = 0;
          s1 = 0; s2 = 0; s3 = 0;
          case (ne)
            0: begin m0a=q[0]; m0b=eqr[0]; m1a=q[1]; m1b=eqr[1]; s1=1;
                     m2a=q[2]; m2b=eqr[2]; s2=1; m3a=q[3]; m3b=eqr[3]; s3=1; end
            1: begin m0a=q[0]; m0b=eqr[1]; m1a=q[1]; m1b=eqr[0]; s1=0;
                     m2a=q[2]; m2b=eqr[3]; s2=0; m3a=q[3]; m3b=eqr[2]; s3=1; end
            2: begin m0a=q[0]; m0b=eqr[2]; m1a=q[2]; m1b=eqr[0]; s1=0;
                     m2a=q[3]; m2b=eqr[1]; s2=0; m3a=q[1]; m3b=eqr[3]; s3=1; end
            3: begin m0a=q[0]; m0b=eqr[3]; m1a=q[3]; m1b=eqr[0]; s1=0;
                     m2a=q[1]; m2b=eqr[2]; s2=0; m3a=q[2]; m3b=eqr[1]; s3=1; end
            default: begin end
          endcase
          case (nph)
            0: begin r = fixed_pkg::s_mul24(m0a, m0b); t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 1; end
            1: begin r = fixed_pkg::s_mul24(m1a, m1b); t1 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 2; end
            2: begin
                 if (s1) r = fixed_pkg::s_sub24(t0, t1);
                 else r = fixed_pkg::s_add24(t0, t1);
                 t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 3;
               end
            3: begin r = fixed_pkg::s_mul24(m2a, m2b); t1 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 4; end
            4: begin
                 if (s2) r = fixed_pkg::s_sub24(t0, t1);
                 else r = fixed_pkg::s_add24(t0, t1);
                 t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 5;
               end
            5: begin r = fixed_pkg::s_mul24(m3a, m3b); t1 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 6; end
            6: begin
                 if (s3) r = fixed_pkg::s_sub24(t0, t1);
                 else r = fixed_pkg::s_add24(t0, t1);
                 nqr[ne[1:0]] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
                 nph <= 0;
                 if (ne == 3) begin ne <= 0; state <= ST_QNORMSEQ; end
                 else ne <= ne + 1;
               end
            default: begin nph <= 0; end
          endcase
        end
        // QNORMSEQ: qn2 chain (ne 0..3 x 2 steps + narrow + call).
        ST_QNORMSEQ: begin
          fixed_pkg::sat24_t r;
          fixed_pkg::sat48_t r64;
          if (ne < 4) begin
            if (nph == 0) begin
              t64b <= 64'(nqr[ne[1:0]]) * 64'(nqr[ne[1:0]]);
              nph <= 1;
            end else begin
              if (ne == 0) begin t64 <= t64b; end
              else begin r64 = fixed_pkg::s_add64(t64, t64b); t64 <= r64.v; sat_cnt <= sat_cnt + {31'b0, r64.sat}; end
              nph <= 0; ne <= ne + 1;
            end
          end else if (ne == 4) begin
            r = fixed_pkg::s_narrow(t64);
            t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; ne <= 5;
          end else begin // ne == 5: ISQ call
            isqx <= t0; retst <= ST_QNORM2;
            isqi <= 0; isqp <= 0;
            ne <= 0; nph <= 0; state <= ST_ISQ;
          end
        end
        // QNORM2: scale (ne 0..3): q <= MUL(nqr, isqy).
        ST_QNORM2: begin
          fixed_pkg::sat24_t r;
          r = fixed_pkg::s_mul24(nqr[ne[1:0]], isqy);
          q[ne[1:0]] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
          if (ne == 3) begin
            // gv_ (gvec const neg(gf)) precomputed here: 1 extra write,
            // zero extra cycles. Model computes neg(gf) once per step.
            r = fixed_pkg::s_neg24(gf);
            gv_ <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
            ne <= 0; nph <= 0; state <= ST_VPSEQ;
          end
          else ne <= ne + 1;
        end
        // VPSEQ: gv neg (ne==3, 1 step) + rows (ne 0..2, nph 0..11).
        // Per row: Ra chain (3x{mult,add} + narrow = 7) + dv/dp (4).
        // Ra[i] = sum_k R[i][k]*av[k]; dv=(Ra+gvec)*dt; v+=dv; dp=v*dt; p+=dp.
        // VPSEQ: Ra rows (ne 0..2) + dv/dp. gv_ (gvec const) was
        // precomputed in QNORM2 tail (1 extra write, zero extra cycles).
        ST_VPSEQ: begin
          fixed_pkg::sat24_t r;
          fixed_pkg::sat48_t r64;
          // Ra chain: nph 0..5 MAC (k=nph>>1), nph 6 narrow.
          // (gv_ precomputed in QNORM2 tail; gvec inlined below.)
          if (nph < 6) begin
            if (nph[0] == 0) begin
              t64b <= 64'(Rr[ne*3+(nph>>1)]) * 64'(av_[2'(nph>>1)]);
              nph <= nph + 1;
            end else begin
              if (nph == 1) begin t64 <= t64b; end
              else begin r64 = fixed_pkg::s_add64(t64, t64b); t64 <= r64.v; sat_cnt <= sat_cnt + {31'b0, r64.sat}; end
              nph <= nph + 1;
            end
          end else if (nph == 6) begin
            r = fixed_pkg::s_narrow(t64);
            t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
            nph <= nph + 1;
          end else if (nph < 11) begin
            if (nph == 7) begin
              r = fixed_pkg::s_add24(t0, (ne == 2) ? gv_ : 32'sd0);
              t1 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= nph + 1;
            end else if (nph == 8) begin
              r = fixed_pkg::s_mul24(t1, dtf);
              t1 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= nph + 1;
            end else if (nph == 9) begin
              r = fixed_pkg::s_add24(v[ne[1:0]], t1);
              t2 <= r.v; v[ne[1:0]] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= nph + 1;
            end else if (nph == 10) begin
              // dp = v_new*dt (MUST be its own cycle: t2 holds v_new from
              // nph==9; reading t2 in the same cycle as writing it would
              // use the stale value — a real bug caught by parity (500x)).
              r = fixed_pkg::s_mul24(t2, dtf);
              t2 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
              nph <= nph + 1;
            end
          end else if (nph == 11) begin
            // p += dp (t2 now holds dp from the nph==10 cycle above)
            r = fixed_pkg::s_add24(p[ne[1:0]], t2);
            p[ne[1:0]] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
            nph <= 0;
            if (ne == 2) begin ne <= 0; state <= ST_F30SEQ; end
            else ne <= ne + 1;
          end else begin // unreachable guard
            nph <= 0;
          end
        end
        // F30SEQ: F30r[9], 9 entries x 8 steps = 72 cycles.
        // Per entry (ne 0..8, F30 index directly): 3x{mult,add} + narrow + neg.
        // R_/Saw_ operands: Rr[row*3+k], Saw_[k*3+col] with (row,col) from ne.
        // ne = row*3+col: row = ne/3, col = ne%3 — NO division: maintain
        // separate (fi,fj) via mm_i/mm_j (free; reset on entry).
        ST_F30SEQ: begin
          fixed_pkg::sat24_t r;
          fixed_pkg::sat48_t r64;
          if (nph < 6) begin
            if (nph[0] == 0) begin
              t64b <= 64'(Rr[mm_i*3+(nph>>1)]) * 64'(Saw_[(nph>>1)*3+mm_j]);
              nph <= nph + 1;
            end else begin
              if (nph == 1) begin t64 <= t64b; end
              else begin r64 = fixed_pkg::s_add64(t64, t64b); t64 <= r64.v; sat_cnt <= sat_cnt + {31'b0, r64.sat}; end
              nph <= nph + 1;
            end
          end else if (nph == 6) begin
            r = fixed_pkg::s_narrow(t64);
            t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 7;
          end else begin // nph == 7
            r = fixed_pkg::s_neg24(t0);
            F30r[mm_i*3+mm_j] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
            nph <= 0;
            if (mm_j == 2) begin
              mm_j <= 0;
              if (mm_i == 2) begin mm_i <= 0; state <= ST_FARR; end
              else mm_i <= mm_i + 1;
            end else mm_j <= mm_j + 1;
          end
        end
        // FARR: capture F00/F312/Sa-neg flags (21) + Qd consts (4) once.
        // F-arrange wires are stable (w_/R_/av_ regs settled long ago);
        // counting here matches the model exactly (each neg once).
        ST_FARR: begin
          sat_cnt <= sat_cnt + {27'b0, farr_sum} + {29'b0, qdsum};
          qd_g <= Qdw_g; qd_a <= Qdw_a; qd_bg <= Qdw_bg; qd_ba <= Qdw_ba;
          mm_i <= 0; mm_j <= 0;
          ne <= 0; nph <= 0;
          state <= ST_PHISEQ;
        end
        // PHISEQ: Phi[225], 2 steps x 225 = 450 cycles.
        // F-select mirrors model F layout (F00/F312 wires, F30 regs,
        // consts). ALWAYS mult (even F==0: mult(0)=0, no sat — proven
        // equivalent to model's skip). diag adds ONE.
        ST_PHISEQ: begin
          fixed_pkg::sat24_t r;
          logic signed [31:0] f;
          f = 0;
          if (mm_i < 3 && mm_j < 3) f = F00w[mm_i*3+mm_j];
          else if (mm_i < 3 && mm_j >= 9 && mm_j < 12 && (mm_j-9) == mm_i) f = 32'shff000000;
          else if (mm_i >= 3 && mm_i < 6 && mm_j < 3) f = F30r[(mm_i-3)*3+mm_j];
          else if (mm_i >= 3 && mm_i < 6 && mm_j >= 12) f = F312w[(mm_i-3)*3+(mm_j-12)];
          else if (mm_i >= 6 && mm_i < 9 && mm_j >= 3 && mm_j < 6 && (mm_j-3) == (mm_i-6)) f = ONE_Q;
          if (nph == 0) begin
            r = fixed_pkg::s_mul24(f, dtf);
            t0 <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat}; nph <= 1;
          end else begin
            if (mm_i == mm_j) begin
              r = fixed_pkg::s_add24(ONE_Q, t0);
              Phi[mm_i*15+mm_j] <= r.v; sat_cnt <= sat_cnt + {31'b0, r.sat};
            end else Phi[mm_i*15+mm_j] <= t0;
            nph <= 0;
            if (mm_j == 14) begin
              mm_j <= 0;
              if (mm_i == 14) begin
                mm_i <= 0; mm_k <= 0; mm_phase <= 1'b0; mm_acc <= 0;
                state <= ST_P1;
              end else mm_i <= mm_i + 1;
            end else mm_j <= mm_j + 1;
          end
        end
        ST_P1, ST_P2: begin
          // one MAC per cycle; k==14 also writes back + advances.
          // (Unchanged from rev1: proven by parity.)
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
          // observability snapshot (all sources stable post-step)
          dbg_R0 <= Rr[0]; dbg_R4 <= Rr[4]; dbg_R8 <= Rr[8];
          dbg_eq0 <= eqr[0]; dbg_nq0 <= nqr[0];
          dbg_Phi0 <= Phi[0]; dbg_qd_g <= qd_g;
          dbg_Phi9 <= Phi[9];
          state <= ST_IDLE;
        end
        default: state <= ST_IDLE;
      endcase
    end
  end

  // ---------- matmul engine operand mux (comb, unchanged rev1) ----------
  fixed_pkg::q24_t eng_a;
  fixed_pkg::q48_t eng_b;
  always_comb begin
    if (mm_phase == 0) begin eng_a = Phi[mm_i*15+mm_k]; eng_b = P[mm_k*15+mm_j]; end
    else begin eng_a = Phi[mm_j*15+mm_k]; eng_b = P1[mm_i*15+mm_k]; end
  end

  // ---------- QDSYM single-entry datapath (unchanged rev1) ----------
  fixed_pkg::q48_t qd_res;
  logic [31:0] qd_sat1;
  always_comb begin
    fixed_pkg::sat48_t r1;
    fixed_pkg::q48_t vv, tt;
    fixed_pkg::acc128_t halves, h, sym;
    logic s1, s2;
    r1 = '0; // default (only diag branches reassign; value unused off-diag)
    vv = P2[mm_i*15+mm_j];
    s1 = 1'b0;
    if (mm_i == mm_j) begin
      if (mm_i < 3) begin r1 = fixed_pkg::s_add48(vv, qd_g); vv = r1.v; s1 = r1.sat; end
      else if (mm_i < 6) begin r1 = fixed_pkg::s_add48(vv, qd_a); vv = r1.v; s1 = r1.sat; end
      else if (mm_i < 9) begin end // dp: none
      else if (mm_i < 12) begin r1 = fixed_pkg::s_add48(vv, qd_bg); vv = r1.v; s1 = r1.sat; end
      else begin r1 = fixed_pkg::s_add48(vv, qd_ba); vv = r1.v; s1 = r1.sat; end
    end
    tt = P2[mm_j*15+mm_i];
    halves = fixed_pkg::acc128_t'(vv) + fixed_pkg::acc128_t'(tt);
    h = (halves >= 0) ? fixed_pkg::acc128_t'(1) : -fixed_pkg::acc128_t'(1);
    sym = (halves + h) >>> 1; // ARITHMETIC (bug note in history)
    s2 = (sym > fixed_pkg::acc128_t'(fixed_pkg::Q48_MAX)) ||
         (sym < fixed_pkg::acc128_t'(fixed_pkg::Q48_MIN));
    qd_sat1 = {31'b0, s1} + {31'b0, s2}; // exact event count 0..2
    if (sym > fixed_pkg::acc128_t'(fixed_pkg::Q48_MAX)) qd_res = fixed_pkg::Q48_MAX;
    else if (sym < fixed_pkg::acc128_t'(fixed_pkg::Q48_MIN)) qd_res = fixed_pkg::Q48_MIN;
    else qd_res = sym[63:0];
  end

  // ---------- bus read (comb, unchanged rev1 map) ----------
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
    else if (bus_addr >= 9'h1f0 && bus_addr <= 9'h1f7) begin
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

endmodule
