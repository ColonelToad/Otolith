#pragma once
// Fixed-point predict pipeline — bit-model of fusion.cpp:25-59 (ADR-0007 M1).
//
// Mirrors the float predict statement-for-statement in Q8.24 with 64-bit
// accumulators: nominal q/v/p (CORDIC exp, N-R normalize) + covariance
// Phi*P*Phi' + Qd. Inputs convert double->fixed at the boundary (the
// future ADC boundary); outputs convert back for the differential test.
// Model-only: beside the float path, never inside it.
//
// Rounding-order note: fixed-point association (e.g. quaternion product
// parenthesization) is the MODEL's defined order, documented here; RTL
// copies it. Float-vs-fixed diffs absorb association differences — the
// error bound judges, exactness with Eigen's vectorized order is neither
// claimed nor needed.
//
// Qd constants in Q16.48 (all representable now — the M0 provisional
// zeroing of bg/barw is REVISITED: 2e-13 -> 56 LSBs, 2e-11 -> 5629
// LSBs, both exact; gyro 2e-7 -> 56295, accel 4.5e-5 -> 12676563).
// Full Qd injection, no exceptions.

#include "fixed/fixed.hpp"

namespace otolith {
namespace fixed {

// Q16.48 accumulator -> Q8.24 storage with round-half-away + saturation.
// CRITICAL: the argument must be a raw Q16.48 product-sum (e.g. from
// acc(a)*acc(b) accumulation). NEVER pre-shift before calling —
// narrow() performs the single >>FRAC itself; double-shifting zeroes
// the value (a real bug caught by the differential test, fixed here).
inline q24 q1648_to_q24(acc x) { return narrow(x); }

// P storage: Q16.48 (int64). Rationale: P spans ~1e-9 (gain-structural
// correlations) to ~2 (predict-only growth/transients) = 11 decades —
// no 32-bit format covers both (Q8.24 destroys correlations -> measured
// hybrid divergence 3.87 m; x256 block-scale rails at 0.5 on growth).
// Q16.48 (LSB 3.6e-15, rail 32768) covers everything with margin.
// Heterogeneous datapath (states Q8.24, P Q16.48, 128-bit accumulators)
// is what real fixed-point Kalman hardware looks like; the M3/M4 area
// numbers will show its cost honestly.
inline double p_to_double(q48 x) { return double(x) / 281474976710656.0; }
inline q48 p_from_double(double x) {
    double s = x * 281474976710656.0; // 2^48 exact in double
    if (s >= 9.223372036854776e18) { ++sat_count(); return std::numeric_limits<q48>::max(); }
    if (s <= -9.223372036854776e18) { ++sat_count(); return std::numeric_limits<q48>::min(); }
    return q48(std::llround(s)); // half-away, matches narrow()
}

// Saturating 64-bit add for the P pipeline (genuine rail only, counted).
inline q48 padd(q48 a, q48 b) {
    q48 r = q48(uint64_t(a) + uint64_t(b)); // wrap well-defined unsigned
    if ((a >= 0 && b >= 0 && r < 0) || (a < 0 && b < 0 && r >= 0)) {
        ++sat_count();
        return (a >= 0) ? std::numeric_limits<q48>::max() : std::numeric_limits<q48>::min();
    }
    return r;
}

struct FixedState {
    q24 q[4];    // w,x,y,z nominal quaternion (unit)
    q24 p[3];
    q24 v[3];
    q24 bg[3];
    q24 ba[3];
    q48 P[225];  // row-major 15x15, Q16.48
};

inline FixedState default_state() {
    FixedState s{};
    s.q[0] = ONE;
    // P = 0.01*I, x10 on dtheta/dv blocks (mirrors make_default_state),
    // stored x256 (see P_SCALE).
    q48 base = p_from_double(0.01);
    q48 big = p_from_double(0.1);
    for (int i = 0; i < 15; ++i)
        s.P[i * 15 + i] = (i < 6) ? big : base;
    return s;
}

struct FixedPredict {
    FixedState s = default_state();
    double gravity = 9.81;

    // One predict step. Inputs in double (boundary conversion inside).
    void step(double dt, const double gyro_m[3], const double accel_m[3]) {
        q24 g[3], a[3];
        for (int i = 0; i < 3; ++i) {
            g[i] = from_double(gyro_m[i]);
            a[i] = from_double(accel_m[i]);
        }
        step_fixed(from_double(dt), g, a, from_double(gravity));
    }

    // Fixed-point core (all Q8.24). Separated so the RTL maps 1:1.
    void step_fixed(q24 dtf, const q24 gyro_m[3], const q24 accel_m[3], q24 gf) {
        const q24 TWO = ONE * 2; // exact
        // w = gyro - bg, a = accel - ba
        q24 w[3], av[3];
        for (int i = 0; i < 3; ++i) {
            w[i] = sub(gyro_m[i], s.bg[i]);
            av[i] = sub(accel_m[i], s.ba[i]);
        }
        // R = quat_to_mat(q), standard wxyz formula
        q24 qw = s.q[0], qx = s.q[1], qy = s.q[2], qz = s.q[3];
        auto sq = [](q24 v) { return narrow(acc(v) * acc(v)); };
        q24 R[9];
        R[0] = sub(ONE, mul(TWO, add(sq(qy), sq(qz))));
        R[1] = mul(TWO, sub(mul(qx, qy), mul(qz, qw)));
        R[2] = mul(TWO, add(mul(qx, qz), mul(qy, qw)));
        R[3] = mul(TWO, add(mul(qx, qy), mul(qz, qw)));
        R[4] = sub(ONE, mul(TWO, add(sq(qx), sq(qz))));
        R[5] = mul(TWO, sub(mul(qy, qz), mul(qx, qw)));
        R[6] = mul(TWO, sub(mul(qx, qz), mul(qy, qw)));
        R[7] = mul(TWO, add(mul(qy, qz), mul(qx, qw)));
        R[8] = sub(ONE, mul(TWO, add(sq(qx), sq(qy))));
        // exp_quat(w*dt), first-order: eq = normalize([1, w*dt/2]).
        // Exact to O(h^3) with h = |w*dt|/2; in-envelope h <= 0.01 gives
        // err <= 2e-7 << LSB, 1000x margin. This deliberately avoids the
        // axis-angle path (angle = |w*dt| via sum-of-squares UNDERFLOWS
        // Q8.24 for typical rates, and axis = w/angle amplifies input
        // quantization 3000x). No division, no CORDIC in the quat path;
        // degrades gracefully (smoothly, no rails) out of envelope.
        // div_scaled/cordic_sincos stay as tested utilities (Cholesky
        // stretch, future trig) — NOT used here by design.
        q24 wdt[3] = {mul(w[0], dtf), mul(w[1], dtf), mul(w[2], dtf)};
        q24 ev[3] = {halve_q(wdt[0]), halve_q(wdt[1]), halve_q(wdt[2])};
        acc en2 = acc(ONE) * acc(ONE);
        for (int i = 0; i < 3; ++i) en2 = acc_add(en2, acc(ev[i]) * acc(ev[i]));
        q24 einv = invsqrt_nr(narrow(en2)); // en2 ~= 2^48: healthy input
        q24 eq[4] = {mul(ONE, einv), mul(ev[0], einv), mul(ev[1], einv), mul(ev[2], einv)};
        // q = normalize(q * eq), Hamilton product, left-assoc (model order)
        q24 q0 = s.q[0], q1 = s.q[1], q2 = s.q[2], q3 = s.q[3];
        q24 e0 = eq[0], e1 = eq[1], e2 = eq[2], e3 = eq[3];
        q24 nq[4];
        nq[0] = sub(sub(sub(mul(q0, e0), mul(q1, e1)), mul(q2, e2)), mul(q3, e3));
        nq[1] = sub(add(add(mul(q0, e1), mul(q1, e0)), mul(q2, e3)), mul(q3, e2));
        nq[2] = sub(add(add(mul(q0, e2), mul(q2, e0)), mul(q3, e1)), mul(q1, e3));
        nq[3] = sub(add(add(mul(q0, e3), mul(q3, e0)), mul(q1, e2)), mul(q2, e1));
        acc qn2 = 0;
        for (int i = 0; i < 4; ++i) qn2 = acc_add(qn2, acc(nq[i]) * acc(nq[i]));
        q24 inv = invsqrt_nr(q1648_to_q24(qn2)); // qn2 is Q16.48
        for (int i = 0; i < 4; ++i) s.q[i] = mul(nq[i], inv);
        // v += (R*a + g)*dt with g_vec = (0,0,-g); p += v*dt (semi-implicit)
        q24 gvec[3] = {0, 0, neg(gf)};
        for (int i = 0; i < 3; ++i) {
            acc sum = 0;
            for (int j = 0; j < 3; ++j) sum = acc_add(sum, acc(R[i * 3 + j]) * acc(av[j]));
            q24 dv = mul(add(narrow(sum), gvec[i]), dtf);
            s.v[i] = add(s.v[i], dv);
            s.p[i] = add(s.p[i], mul(s.v[i], dtf));
        }
        // F blocks (mirror fusion.cpp): F00=-skew(w), F09=-I,
        // F30=-R*skew(a), F312=-R, F63=I. skew = arrangement + neg (exact).
        q24 F[225] = {0};
        auto sk = [&](const q24 u[3], q24 S[9]) {
            S[0] = 0; S[1] = neg(u[2]); S[2] = u[1];
            S[3] = u[2]; S[4] = 0; S[5] = neg(u[0]);
            S[6] = neg(u[1]); S[7] = u[0]; S[8] = 0;
        };
        q24 Sw[9], Sa[9];
        sk(w, Sw);
        sk(av, Sa);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                F[i * 15 + j] = neg(Sw[i * 3 + j]); // F00
                if (i == j) F[i * 15 + 9 + j] = neg(ONE); // F09
                acc f30 = 0; // F30 = -R*skew(a), single narrow
                for (int k = 0; k < 3; ++k) f30 = acc_add(f30, acc(R[i * 3 + k]) * acc(Sa[k * 3 + j]));
                F[(3 + i) * 15 + j] = neg(narrow(f30));
                F[(3 + i) * 15 + 12 + j] = neg(R[i * 3 + j]); // F312
                if (i == j) F[(6 + i) * 15 + 3 + j] = ONE; // F63
            }
        // Phi = I + F*dt
        q24 Phi[225];
        for (int i = 0; i < 15; ++i)
            for (int j = 0; j < 15; ++j) {
                q24 fdt = (F[i * 15 + j] == 0) ? q24(0) : mul(F[i * 15 + j], dtf);
                Phi[i * 15 + j] = (i == j) ? add(ONE, fdt) : fdt;
            }
        // Qd consts in Q16.48 (self-deriving): gyro 2e-7, accel 4.5e-5,
        // bg-rw 2e-13, ba-rw 2e-11 — ALL representable now (bg -> 56
        // LSBs), so full injection, no provisional zeroing.
        // P2 = Phi*P*Phi': Q8.24 x Q16.48 products in 128-bit accums
        // (overflow-freedom proved in fixed.hpp), narrow once per entry
        // to Q16.48, +Qd, symmetrize with rounded halving in 64-bit.
        // Qd = sigma^2*dt per block, computed EXACTLY as the RTL does
        // (narrow48(SIG2*dtf)) so model<->RTL agree by construction.
        // SIG2 consts are sigma^2 in Q16.48 (offline derivation below);
        // dt is the step's own dtf, as in C++. dtf quantization shifts
        // Qd ~1e-5 relative vs float64 — nil impact, re-verified by the
        // error bound (predict: gyro/accel Qd ~1e-7/1e-5 vs P ~1e-2).
        // SIG2 derivation (sigma^2 * 2^48, half-away):
        //   gyro  1e-4   -> 28147497671
        //   accel 0.0225 -> 6333186975990
        //   bg    1e-10  -> 28147
        //   ba    1e-8   -> 2814750
        // (sigmas mirror FusionConfig defaults)
        const q48 QDG = narrow48(acc128(q48(28147497671LL)) * acc128(dtf));
        const q48 QDA = narrow48(acc128(q48(6333186975990LL)) * acc128(dtf));
        const q48 QDBG = narrow48(acc128(q48(28147)) * acc128(dtf));
        const q48 QDBA = narrow48(acc128(q48(2814750)) * acc128(dtf));
        q48 P1[225] = {0}, P2[225] = {0};
        for (int i = 0; i < 15; ++i)
            for (int j = 0; j < 15; ++j) {
                acc128 sum = 0;
                for (int k = 0; k < 15; ++k)
                    sum += acc128(Phi[i * 15 + k]) * acc128(s.P[k * 15 + j]);
                P1[i * 15 + j] = narrow48(sum);
            }
        for (int i = 0; i < 15; ++i)
            for (int j = 0; j < 15; ++j) {
                acc128 sum = 0;
                for (int k = 0; k < 15; ++k)
                    sum += acc128(P1[i * 15 + k]) * acc128(Phi[j * 15 + k]);
                P2[i * 15 + j] = narrow48(sum);
            }
        for (int i = 0; i < 15; ++i)
            for (int j = 0; j < 15; ++j) {
                q48 v = P2[i * 15 + j];
                if (i == j) {
                    if (i < 3) v = padd(v, QDG); // gyro
                    else if (i < 6) v = padd(v, QDA); // accel
                    else if (i < 9) {} // dp: none
                    else if (i < 12) v = padd(v, QDBG); // bg-rw
                    else v = padd(v, QDBA); // ba-rw
                }
                q48 t = P2[j * 15 + i];
                acc128 halves = acc128(v) + acc128(t);
                acc128 h = (halves >= 0) ? acc128(1) : acc128(-1);
                acc128 sym = (halves + h) >> 1;
                if (sym > acc128(std::numeric_limits<q48>::max())) { ++sat_count(); sym = acc128(std::numeric_limits<q48>::max()); }
                if (sym < acc128(std::numeric_limits<q48>::min())) { ++sat_count(); sym = acc128(std::numeric_limits<q48>::min()); }
                s.P[i * 15 + j] = q48(sym);
            }
    }

private:
    static q24 halve_q(q24 v) {
        return q24((acc(v) + (v >= 0 ? acc(1) : acc(-1))) >> 1);
    }
};

} // namespace fixed
} // namespace otolith
