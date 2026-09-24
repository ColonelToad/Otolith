// Fixed-point model differential tests (ADR-0007 M1).
// Strategy: unit bounds on primitives + 200-step predict differential
// fixed-vs-float on the SAME input sequence as the Rust golden test
// (fusion.rs differential_vs_cpp_golden), so all three implementations
// (C++ float, Rust, C++ fixed) share one input script.

#include "fixed/fixed.hpp"
#include "fixed/fixed_predict.hpp"
#include "otolith/fusion.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>

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
