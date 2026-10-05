#pragma once
// Fixed-point MEASUREMENT model — bit-model of fusion.cpp:71-103 (ADR-0007 M5).
//
// The M5-C study injected H and y from the float path; this builds them in
// fixed point so the whole update (predict, measure, correct) is fixed-point
// and the error can be attributed end to end.
//
// Everything mirrors an existing function statement for statement:
//   foot_pos_base_fixed  <-> leg_kin.cpp:22-34  (planar 2R FK)
//   leg_geom_fixed       <--  leg_kin.cpp:6-17  (Go2 constants)
//   quat_to_mat_fixed    <--  fixed_predict.hpp  (shared, one definition)
//   FixedUpdateFixed     <->  fusion.cpp:61-131
//
// sin/cos come from the Q8.24 CORDIC in fixed.hpp. Its convergence range is
// |theta| <= 1.7433 rad (CORDIC_LIM) and inputs beyond it saturate to the rail
// and are COUNTED; trot joint angles stay well inside, so the count stays zero
// and the bench asserts that.
//
// PRECISION (the part M5-C flagged as unmeasured): r_base is ~0.3 m while
// r_base - r_prev is only ~1e-4 m, and dividing by dt = 2 ms amplifies the
// Q8.24 quantization of that difference by 500x. At sigma_leg = 0.3 this is
// ~3e-5 m/s against a 0.3 m/s noise floor, but the margin scales as
// 1/sigma_leg -- see the sigma sweep in hdl/M5_UPDATE_STUDY.md.

#include <cstdint>

#include "fixed/fixed.hpp"
#include "fixed/fixed_predict.hpp"
#include "fixed/fixed_update.hpp"

namespace otolith {
namespace fixed {

// ---------------------------------------------------------------------------
// Leg geometry, Q8.24 (leg_kin.cpp:6-17). Indexed by contact-bit position so
// FL, FR, RL, RR == 0,1,2,3.
// ---------------------------------------------------------------------------
struct LegGeomFixed {
    q24 a_offset, L1, L2;
    int side;
    q24 hip_x, hip_y, hip_z;
};

inline LegGeomFixed leg_geom_fixed(int leg) {
    LegGeomFixed g{};
    g.a_offset = from_double(0.0955);
    g.L1 = from_double(0.213);
    g.L2 = from_double(0.21300938946440834);
    g.hip_z = 0;
    switch (leg) {
        case 0: g.side = +1; g.hip_x =  from_double(0.1934); g.hip_y =  from_double(0.0465); break;
        case 1: g.side = -1; g.hip_x =  from_double(0.1934); g.hip_y = -from_double(0.0465); break;
        case 2: g.side = +1; g.hip_x = -from_double(0.1934); g.hip_y =  from_double(0.0465); break;
        default: g.side = -1; g.hip_x = -from_double(0.1934); g.hip_y = -from_double(0.0465); break;
    }
    return g;
}

// Planar 2R FK + hip roll, Q8.24 (leg_kin.cpp:22-34). q = [hip, thigh, calf].
inline void foot_pos_base_fixed(const LegGeomFixed& leg, const q24 q[3],
                                q24 r[3]) {
    // sin_cos_wide, not cordic_sincos: the knee angle (thigh + calf) reaches
    // ~2.8 rad on the Go2, past the CORDIC's +/-1.7433 rad convergence.
    const SinCos st = sin_cos_wide(q[1]);              // thigh
    const SinCos sc = sin_cos_wide(add(q[1], q[2]));   // thigh + calf
    const q24 x_in = sub(neg(mul(leg.L1, st.s)), mul(leg.L2, sc.s));
    const q24 z_in = sub(neg(mul(leg.L1, st.c)), mul(leg.L2, sc.c));
    const q24 y_in = (leg.side > 0) ? leg.a_offset : neg(leg.a_offset);
    const SinCos sh = sin_cos_wide(q[0]);              // hip roll
    const q24 dy = sub(mul(sh.c, y_in), mul(sh.s, z_in));
    const q24 dz = add(mul(sh.s, y_in), mul(sh.c, z_in));
    r[0] = add(leg.hip_x, x_in);
    r[1] = add(leg.hip_y, dy);
    r[2] = add(leg.hip_z, dz);
}

// skew(u), arrangement only -- the signs are applied by the caller exactly as
// fusion.cpp:91-97 does (H = -R*[omega x r] and H = R*[r]).
inline void skew_fixed(const q24 u[3], q24 S[9]) {
    S[0] = 0;     S[1] = neg(u[2]); S[2] = u[1];
    S[3] = u[2];  S[4] = 0;       S[5] = neg(u[0]);
    S[6] = neg(u[1]); S[7] = u[0]; S[8] = 0;
}

// 3x3 TIMES A 3-VECTOR, Q8.24. Deliberately NOT mat3_mul_fixed() below:
// that indexes B[k*3+j] for k,j < 3, i.e. B[0..8], so handing it a 3-element
// vector reads six elements past the end. Arrays decay to pointers, so neither
// the compiler nor a reader catches it -- it showed up as h[0] = 4.18 against
// a true -0.014, i.e. a wholesale divergence of the fully fixed filter.
// R * v, matching Eigen's R * (omega_cross_r + r_dot) column-vector product.
inline void mat3_vec_fixed(const q24 A[9], const q24 v[3], q24 out[3]) {
    for (int i = 0; i < 3; ++i) {
        acc sum = 0;
        for (int k = 0; k < 3; ++k) sum = acc_add(sum, acc(A[i * 3 + k]) * acc(v[k]));
        out[i] = narrow(sum);
    }
}

// 3x3 q24 product, one narrowing per entry (same idiom as the predict path's
// F30/R30 blocks). B and C must be 9 elements.
inline void mat3_mul_fixed(const q24 A[9], const q24 B[9], q24 C[9]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            acc sum = 0;
            for (int k = 0; k < 3; ++k)
                sum = acc_add(sum, acc(A[i * 3 + k]) * acc(B[k * 3 + j]));
            C[i * 3 + j] = narrow(sum);
        }
}

// ---------------------------------------------------------------------------
// The measurement model + full fixed-point update (fusion.cpp:61-131).
// ---------------------------------------------------------------------------
struct FixedUpdateFixed {
    // Owns a FixedPredict so the FULLY fixed filter reuses the one verified
    // predict implementation (5000-step RTL bit-parity) instead of a copy.
    FixedPredict fp;
    FixedState& s = fp.s;
    q24 prev_qj[12] = {0};
    bool has_prev = false;

    // The M1 predict step, on the same state. Boundary conversion inside.
    void predict(double dt, const q24 gyro[3], const q24 accel[3],
                 double gravity = 9.81) {
        fp.step_fixed(from_double(dt), gyro, accel, from_double(gravity));
    }

    // Returns the number of stance legs used (0 => no update this step).
    // Fills H (rows x 15), y (rows), Rmat (rows x rows, Q16.48).
    int measure(const q24 qj[12], const uint8_t contacts[4],
                const q24 gyro_m[3], q24 dtf, double sigma_leg,
                q24* H, q24* y, q48* Rmat, int max_rows) {
        int stance[4];
        int m = 0;
        for (int i = 0; i < 4; ++i)
            if (contacts[i]) stance[m++] = i;

        // Snapshot the PREVIOUS sample before overwriting: r_dot is a
        // difference of consecutive foot positions, so updating prev_qj first
        // would make r_prev == r_base and silently zero every r_dot.
        q24 qprev[12];
        for (int i = 0; i < 12; ++i) qprev[i] = prev_qj[i];
        const bool fresh = (!has_prev);
        for (int i = 0; i < 12; ++i) prev_qj[i] = qj[i];
        has_prev = true;
        if (m == 0 || fresh || 3 * m > max_rows) return 0;

        const int rows = 3 * m;
        q24 R[9];
        quat_to_mat_fixed(s.q, R);
        q24 w[3];
        for (int i = 0; i < 3; ++i) w[i] = sub(gyro_m[i], s.bg[i]);

        const q48 sig2 = p_from_double(sigma_leg * sigma_leg);
        for (int k = 0; k < m; ++k) {
            const int leg = stance[k];
            const LegGeomFixed g = leg_geom_fixed(leg);
            q24 r_base[3], r_prev[3];
            foot_pos_base_fixed(g, qj + leg * 3, r_base);
            foot_pos_base_fixed(g, qprev + leg * 3, r_prev);

            q24 r_dot[3];
            for (int i = 0; i < 3; ++i)
                // (r_base - r_prev) / dt. div_scaled normalizes the divisor
                // and rescales, so num << den is fine -- checked, not assumed.
                r_dot[i] = div_scaled(sub(r_base[i], r_prev[i]), dtf);

            q24 ocr[3]; // omega x r_base
            ocr[0] = sub(mul(w[1], r_base[2]), mul(w[2], r_base[1]));
            ocr[1] = sub(mul(w[2], r_base[0]), mul(w[0], r_base[2]));
            ocr[2] = sub(mul(w[0], r_base[1]), mul(w[1], r_base[0]));

            q24 sum[3], h[3], tmp[3];
            for (int i = 0; i < 3; ++i) sum[i] = add(ocr[i], r_dot[i]);
            mat3_vec_fixed(R, sum, tmp);
            for (int i = 0; i < 3; ++i) h[i] = add(s.v[i], tmp[i]);
            for (int i = 0; i < 3; ++i) y[k * 3 + i] = neg(h[i]);

            q24 wr[9], rr[9], Rw[9], Rr[9];
            skew_fixed(ocr, wr);
            skew_fixed(r_base, rr);
            mat3_mul_fixed(R, wr, Rw);
            mat3_mul_fixed(R, rr, Rr);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) {
                    H[(k * 3 + i) * 15 + j] = neg(Rw[i * 3 + j]);
                    H[(k * 3 + i) * 15 + 3 + j] = (i == j) ? ONE : q24(0);
                    H[(k * 3 + i) * 15 + 9 + j] = Rr[i * 3 + j];
                }
        }
        for (int i = 0; i < rows * rows; ++i) Rmat[i] = 0;
        for (int k = 0; k < m; ++k)
            for (int i = 0; i < 3; ++i)
                Rmat[(k * 3 + i) * rows + (k * 3 + i)] = sig2;
        return rows;
    }

    // One full fixed-point update: measure, then the M5 linear algebra.
    int step(const q24 qj[12], const uint8_t contacts[4],
             const q24 gyro_m[3], q24 dtf, double sigma_leg,
             DivMode mode, int max_rows = 12) {
        q24 H[15 * 12] = {0}, y[12] = {0};
        q48 Rmat[12 * 12] = {0};
        const int rows = measure(qj, contacts, gyro_m, dtf, sigma_leg, H, y,
                                 Rmat, max_rows);
        if (rows == 0) return 0;
        const UpdateOut out = update_linear(s, H, y, Rmat, rows, mode);
        if (!out.pd) return 0;
        apply_dx_fixed(s, out.dx);
        return rows;
    }
};

} // namespace fixed
} // namespace otolith
