// Fixed-point model differential tests (ADR-0007 M1).
// Strategy: unit bounds on primitives + 200-step predict differential
// fixed-vs-float on the SAME input sequence as the Rust golden test
// (fusion.rs differential_vs_cpp_golden), so all three implementations
// (C++ float, Rust, C++ fixed) share one input script.

#include "fixed/fixed.hpp"
#include "fixed/fixed_predict.hpp"
#include "fixed/fixed_update.hpp"
#include "fixed/fixed_meas.hpp"
#include "otolith/fusion.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <vector>

using namespace otolith;
using namespace otolith::fixed;

TEST_CASE("fixed mul rounds to <1 LSB", "[fixed]") {
    q24 m = mul(from_double(0.1), from_double(0.2));
    double err_lsb = std::fabs(to_double(m) - 0.02) / std::ldexp(1.0, -24);
    CHECK(err_lsb < 1.0);
}

TEST_CASE("fixed saturates at rails and counts", "[fixed]") {
    sat_reset();
    CHECK(add(QMAX, ONE) == QMAX);
    CHECK(sub(QMIN, ONE) == QMIN);
    CHECK(mul(QMAX, QMAX) == QMAX);
    CHECK(neg(QMIN) == QMAX);
    CHECK(sat_count() == 4);
}

TEST_CASE("cordic within 40 LSB over [-1.7, 1.7]", "[fixed]") {
    double worst = 0;
    for (double t = -1.7; t <= 1.7001; t += 0.05) {
        SinCos sc = cordic_sincos(from_double(t));
        worst = std::max(worst, std::fabs(to_double(sc.s) - std::sin(t)));
        worst = std::max(worst, std::fabs(to_double(sc.c) - std::cos(t)));
    }
    CHECK(worst < 40 * std::ldexp(1.0, -24));
}

TEST_CASE("div_scaled in-envelope accuracy", "[fixed]") {
    // axis pattern: |num| ~ den (exp_quat use), plus general in-rail cases
    double worst = 0;
    for (double a : {1e-4, 1e-3, 0.01, 0.05, 0.5, 2.0}) {
        q24 r = div_scaled(from_double(0.6 * a), from_double(a));
        worst = std::max(worst, std::fabs(to_double(r) - 0.6));
    }
    for (double n : {0.5, 1.0, -3.0}) {
        for (double d : {0.5, 2.0, 9.81}) {
            q24 r = div_scaled(from_double(n), from_double(d));
            worst = std::max(worst, std::fabs(to_double(r) - n / d));
        }
    }
    // input quantization dominates (~1e-4 on tiny axes); division itself
    // must stay an order below the axis-path budget
    CHECK(worst < 5e-4);
}

TEST_CASE("invsqrt near 1", "[fixed]") {
    for (double x : {0.9, 1.0, 1.1}) {
        q24 r = invsqrt_nr(from_double(x));
        CHECK(std::fabs(to_double(r) - 1.0 / std::sqrt(x)) < 1e-6);
    }
}

TEST_CASE("fixed predict tracks float over 200 steps", "[fixed]") {
    // Predict-path differential: float filter runs predict-ONLY (no
    // updates — the update step is float-only until the Cholesky
    // stretch), fixed model runs the same 200 predicts. Any divergence
    // is quantization alone, not algorithm.
    FusionEKF ekf;
    FixedPredict fx;
    Eigen::Vector3d gm(0.01,-0.01,0.02), am(0.05,-0.03,9.82);
    double g[3] = {0.01,-0.01,0.02}, a[3] = {0.05,-0.03,9.82};
    sat_reset();
    for (int i=0;i<200;++i) {
        ekf.predict(0.002, gm, am);
        fx.step(0.002, g, a);
    }
    const auto& s = ekf.state();
    double dp = 0, dv = 0, dq = 0, dP = 0;
    for (int i=0;i<3;++i) {
        dp = std::max(dp, std::fabs(to_double(fx.s.p[i]) - s.p[i]));
        dv = std::max(dv, std::fabs(to_double(fx.s.v[i]) - s.v[i]));
    }
    dq = std::fabs(to_double(fx.s.q[0]) - s.q.w());
    for (int i=0;i<15;++i) for (int j=0;j<15;++j)
        dP = std::max(dP, std::fabs(p_to_double(fx.s.P[i*15+j]) - s.P(i,j)));
    // In-envelope: no saturation fires on this script (rails are counted).
    CHECK(sat_count() == 0);
    // Bounds calibrated on first green run (commit message records the
    // observed values); the full-log RMSE is the real error gate.
    CHECK(dp < 1e-3);
    CHECK(dv < 1e-3);
    CHECK(dq < 1e-4);
    CHECK(dP < 1e-2);
}

// ---------------------------------------------------------------------------
// M5-C: fixed-point update path (ADR-0007). These are the standing gates for
// the study's findings — they exist because the study's differential caught
// five real bugs that unit bounds alone would not have.
// ---------------------------------------------------------------------------

TEST_CASE("recip48 is exact to <1e-13 relative", "[fixed][update]") {
    double worst = 0.0;
    for (double d : {0.0911319, 0.09, 0.19, 0.19262, 0.25, 0.5, 1.0, 2.0, 3.7}) {
        const double got = p_to_double(recip48(p_from_double(d)));
        worst = std::max(worst, std::fabs(got - 1.0 / d) * d);
    }
    CHECK(worst < 1e-13);
    // Normalization must land m in [0.5,1): straddle 1.0.
    CHECK(p_to_double(recip48(p_from_double(0.999))) > 0.999);
    CHECK(p_to_double(recip48(p_from_double(1.001))) < 1.001);
}

TEST_CASE("div48 both modes track the quotient", "[fixed][update]") {
    double worst_q48 = 0.0, worst_q24 = 0.0;
    for (double d : {0.0911319, 0.19, 0.5, 1.0, 3.7}) {
        for (double n : {0.05, 0.5, 1.0, -0.3}) {
            const q48 D = p_from_double(d), N = p_from_double(n);
            worst_q48 = std::max(worst_q48,
                std::fabs(p_to_double(div48(N, D, DivMode::Q48)) - n / d) / std::fabs(n / d));
            worst_q24 = std::max(worst_q24,
                std::fabs(p_to_double(div48(N, D, DivMode::Q24)) - n / d) / std::fabs(n / d));
        }
    }
    CHECK(worst_q48 < 1e-12);  // full Q16.48 reciprocal
    CHECK(worst_q24 < 1e-5);   // Q8.24 reciprocal: ~24-bit, as expected
}

// LDL' must reconstruct S and S^-1 must invert S, on a DENSE SPD matrix.
TEST_CASE("LDL' reconstructs S and inverts it on dense SPD", "[fixed][update]") {
    constexpr int n = 9;
    // Build via Eigen so SPD-ness is not in question, then convert.
    Eigen::MatrixXd A(n, n);
    unsigned seed = 12345u;
    auto next = [&]() {
        seed = seed * 1103515245u + 12345u;
        return double((seed >> 16) & 0x7fff) / 32768.0 - 1.0;
    };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) A(i, j) = (i == j) ? 0.19 : 0.10 * next();
    Eigen::MatrixXd Sd = A * A.transpose();
    Sd += 0.09 * Eigen::MatrixXd::Identity(n, n);
    q48 S[81];
    for (int i = 0; i < n * n; ++i) S[i] = p_from_double(Sd(i / n, i % n));

    LdlFactor f;
    sat_reset();
    REQUIRE(ldl_factor(S, n, f, DivMode::Q48));
    CHECK(f.pd);
    for (int i = 0; i < n; ++i) CHECK(f.D[i] > 0);

    // (1) reconstruction: L D L' == S
    double rec = 0.0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            acc128 s = 0;
            for (int k = 0; k < n; ++k) {
                const q48 lik = (k < i) ? f.L[i * n + k]
                              : (k == i ? p_from_double(1.0) : q48(0));
                const q48 ljk = (k < j) ? f.L[j * n + k]
                              : (k == j ? p_from_double(1.0) : q48(0));
                // narrow the intermediate: a bare q48^3 triple product needs
                // 144 fractional bits and overflows the 128-bit accumulator.
                s += acc128(narrow96to48(acc128(lik) * acc128(f.D[k]))) *
                     acc128(ljk);
            }
            rec = std::max(rec, std::fabs(p_to_double(narrow96to48(s)) -
                                           Sd(i, j)));
        }
    CHECK(rec < 1e-12);

    // (2) inverse: S * (S^-1 e_c) == e_c for every column
    const Eigen::MatrixXd Sd_inv = Sd.inverse();
    double res = 0.0;
    for (int c = 0; c < n; ++c) {
        q48 e[81] = {0};
        e[c] = p_from_double(1.0);
        ldl_solve(f, e);
        for (int j = 0; j < n; ++j)
            res = std::max(res, std::fabs(p_to_double(e[j]) - Sd_inv(j, c)));
    }
    CHECK(res < 1e-9);
    CHECK(sat_count() == 0);
}

// End-to-end one update against float, on a synthetic update shaped like the
// real one (3 stance legs -> rows=9, per-leg I_3 on the dv block, Rmat =
// 0.09*I). Pins the K and Joseph-form indexing, which bounds on the factor
// alone cannot see: the K = T S^-1 and (K Rmat K')[i][j] = sum_k
// KR[i][k]K[j][k] transposes both slipped through factorization-only tests.
TEST_CASE("update_linear matches float on a synthetic update", "[fixed][update]") {
    constexpr int rows = 9, legs = 3;
    unsigned seed = 777u;
    auto next = [&]() {
        seed = seed * 1103515245u + 12345u;
        return double((seed >> 16) & 0x7fff) / 32768.0 - 1.0; // [-1,1)
    };
    // Covariance: proper SPD (symmetric, positive diagonal), magnitudes like
    // the real filter's (max ~0.1, bulk ~1e-3).
    Eigen::MatrixXd Pd = 0.02 * Eigen::MatrixXd::Identity(15, 15);
    std::vector<double> off(15 * 15);
    for (int i = 0; i < 15; ++i)
        for (int j = i + 1; j < 15; ++j) {
            const double v = 0.004 * next();
            off[i * 15 + j] = off[j * 15 + i] = v;
        }
    for (int i = 0; i < 15; ++i)
        for (int j = 0; j < 15; ++j) Pd(i, j) += (i == j ? 0.0 : off[i * 15 + j]);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, 15);
    for (int leg = 0; leg < legs; ++leg) {
        H.block(leg * 3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3); // dv
        for (int r = 0; r < 3; ++r) {
            H(leg * 3 + r, 0) = 0.3 * next();      // dtheta
            H(leg * 3 + r, 9) = 0.3 * next();      // dp
            H(leg * 3 + r, 12) = 0.1 * next();     // dba
        }
    }
    const Eigen::MatrixXd Rm = 0.09 * Eigen::MatrixXd::Identity(rows, rows);
    Eigen::VectorXd yv(rows);
    for (int i = 0; i < rows; ++i) yv(i) = 0.4 * next();

    const Eigen::MatrixXd Kf = Pd * H.transpose() *
        (H * Pd * H.transpose() + Rm).inverse();
    const Eigen::MatrixXd Af = Eigen::MatrixXd::Identity(15, 15) - Kf * H;
    const Eigen::MatrixXd Pf = Af * Pd * Af.transpose() + Kf * Rm * Kf.transpose();
    const Eigen::VectorXd dxf = Kf * yv;

    FixedState fs;
    for (int i = 0; i < 225; ++i) fs.P[i] = p_from_double(Pd(i / 15, i % 15));
    q24 Hq[15 * 12] = {0}, yq[12] = {0};
    q48 Rq[144] = {0};
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < 15; ++j) Hq[i * 15 + j] = from_double(H(i, j));
        yq[i] = from_double(yv(i));
        for (int j = 0; j < rows; ++j) Rq[i * rows + j] = p_from_double(Rm(i, j));
    }
    sat_reset();
    const UpdateOut out = update_linear(fs, Hq, yq, Rq, rows, DivMode::Q48);
    REQUIRE(out.pd);
    CHECK(sat_count() == 0);

    double kerr = 0.0;
    for (int i = 0; i < 15; ++i)
        kerr = std::max(kerr, std::fabs(to_double(out.dx[i]) - dxf(i)));
    CHECK(kerr < 1e-6);

    double perr = 0.0;
    for (int i = 0; i < 15; ++i)
        for (int j = 0; j < 15; ++j)
            perr = std::max(perr,
                std::fabs(p_to_double(fs.P[i * 15 + j]) - Pf(i, j)));
    CHECK(perr < 1e-5);
}

// ---------------------------------------------------------------------------
// M5: fixed-point measurement model (H and y built in fixed point, not
// injected from float). Gates the leg FK, the r_dot division and the CORDIC's
// convergence range. See hdl/M5_UPDATE_STUDY.md.
// ---------------------------------------------------------------------------
TEST_CASE("fixed leg FK matches the float model over trot joint angles", "[fixed][update]") {
    double worst = 0.0;
    sat_reset();
    for (int leg = 0; leg < 4; ++leg) {
        const LegGeomFixed g = leg_geom_fixed(leg);
        for (int i = 0; i < 64; ++i) {
            // hip in [-0.5,0.5], thigh/calf in [-1.4,1.4] rad: inside the
            // CORDIC's +/-1.7433 rad range and inside a real trot's envelope.
            const double hip = -0.5 + 1.0 * (i / 63.0);
            const double th = -1.4 + 2.8 * ((i * 7 % 64) / 63.0);
            const double cf = -1.4 + 2.8 * ((i * 13 % 64) / 63.0);
            q24 q[3] = {from_double(hip), from_double(th), from_double(cf)};
            q24 r[3];
            foot_pos_base_fixed(g, q, r);
            // Independent double reference: the same 2R planar FK.
            const double x_in = -0.213 * std::sin(th) - 0.21300938946440834 * std::sin(th + cf);
            const double z_in = -0.213 * std::cos(th) - 0.21300938946440834 * std::cos(th + cf);
            const double y_in = (g.side > 0 ? 0.0955 : -0.0955);
            const double c = std::cos(hip), s = std::sin(hip);
            const double dy = c * y_in - s * z_in;
            const double dz = s * y_in + c * z_in;
            const double hx = (leg == 0 || leg == 1) ? 0.1934 : -0.1934;
            const double hy = (leg == 0 || leg == 2) ? 0.0465 : -0.0465;
            const double want[3] = {hx + x_in, hy + dy, dz};
            for (int a = 0; a < 3; ++a)
                worst = std::max(worst, std::fabs(to_double(r[a]) - want[a]));
        }
    }
    // Q8.24 LSB is 6e-8; allow a few LSB for the CORDIC's own error.
    CHECK(worst < 1e-6);
    CHECK(sat_count() == 0);   // no CORDIC rail hits in this envelope
}

TEST_CASE("fixed r_dot recovers a known foot velocity", "[fixed][update]") {
    const LegGeomFixed g = leg_geom_fixed(0);
    const q24 a0[3] = {from_double(0.10), from_double(0.60), from_double(-1.10)};
    const q24 a1[3] = {from_double(0.10), from_double(0.62), from_double(-1.10)};
    q24 r0[3], r1[3];
    foot_pos_base_fixed(g, a0, r0);
    foot_pos_base_fixed(g, a1, r1);
    const q24 dtf = from_double(0.002);
    sat_reset();
    for (int i = 0; i < 3; ++i) {
        const q24 rd = div_scaled(sub(r1[i], r0[i]), dtf);
        // Independent double reference for the same step.
        const double th0 = 0.60, th1 = 0.62, cf = -1.10, hip = 0.10;
        auto foot = [&](double th) {
            const double x_in = -0.213 * std::sin(th) - 0.21300938946440834 * std::sin(th + cf);
            const double z_in = -0.213 * std::cos(th) - 0.21300938946440834 * std::cos(th + cf);
            const double y_in = 0.0955;
            const double c = std::cos(hip), s = std::sin(hip);
            return std::array<double,3>{0.1934 + x_in,
                                        0.0465 + (c * y_in - s * z_in),
                                        (s * y_in + c * z_in)};
        };
        const auto fa = foot(th0), fb = foot(th1);
        const double want = (fb[i] - fa[i]) / 0.002;
        // The DIFFERENCE is ~1e-4 m quantized at 6e-8, then /2 ms: expect
        // ~3e-5 m/s of quantization noise on a ~1 m/s signal.
        CHECK(std::fabs(to_double(rd) - want) < 2e-3);
    }
    CHECK(sat_count() == 0);
}

TEST_CASE("sin_cos_wide covers the Go2 joint envelope", "[fixed][update]") {
    double worst = 0.0;
    sat_reset();
    // Sweep past +/-pi so the 2*pi reduction and both folds are exercised.
    for (int i = 0; i < 400; ++i) {
        const double th = -3.2 + 6.4 * (i / 399.0);
        const SinCos r = sin_cos_wide(from_double(th));
        worst = std::max(worst, std::fabs(to_double(r.s) - std::sin(th)));
        worst = std::max(worst, std::fabs(to_double(r.c) - std::cos(th)));
    }
    CHECK(worst < 5e-6);          // Q8.24 LSB 6e-8, CORDIC ~40 LSB
    CHECK(sat_count() == 0);      // folding keeps every input convergent
}
