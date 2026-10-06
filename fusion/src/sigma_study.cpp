// Measures the measurement noise the filter ACTUALLY sees, which is the number
// sigma_leg is supposed to describe.
//
// WHY THIS IS A C++ TOOL AND NOT A SCRIPT
// ========================================
// r_base comes from `leg_kin::foot_pos_base`, the same planar 2R FK the filter's
// measurement model uses. Re-implementing it in Python to analyse it would create
// a second source of truth that can disagree with the filter by an unknown
// amount -- and this repo has already been bitten twice that way (the `L2`
// provenance, and the `mat3_mul_fixed` sizing bug). The analysis has to run on
// the same arithmetic it is analysing.
//
// WHAT IT REPORTS
// ===============
//   * r_base per foot, from logged qj
//   * r_dot = (r_base - r_prev) / dt, the quantity the measurement model forms
//   * the spread of r_dot, per foot and for stance feet only
//
// `Rmat = sigma_leg^2 * I` is the filter's belief about r_dot's spread. This
// tool measures the actual spread, so the two can be compared directly instead
// of sigma_leg being justified by assertion.
//
// Note the filter computes r_dot from the SAME r_base on consecutive steps, so
// any noise in qj appears in r_base directly and in r_dot amplified by 1/dt.
// That amplification is the whole reason sigma_leg is large, and it is
// measurable here rather than assumed.
//
// --place-err <sigma_mm> adds a per-STANCE placement error to r_base: drawn
// once at touchdown, held constant for the whole stance, re-drawn next stance.
// That is what uneven terrain or a sloppy foothold actually does -- the foot
// lands slightly off and then stays where it landed. It is the same shape of
// error as the foot pad's 10.2 mm constant offset (docs/V06_FOOT_PAD.md), so it
// should produce an OFFSET and not a RATE. This measures which it is, rather
// than assuming.
//
// Usage: sigma_study <in.otlg> [--place-err <sigma_mm>]

#include "otolith/log.hpp"
#include "otolith/leg_kin.hpp"
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <cstdint>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: sigma_study <in.otlg> [--place-err <sigma_mm>]\n");
        return 2;
    }
    double place_err_mm = -1.0;
    for (int a = 2; a + 1 < argc; ++a)
        if (std::string(argv[a]) == "--place-err") place_err_mm = std::atof(argv[a + 1]);
    otolith::LogFile lf;
    try {
        lf = otolith::read_log(argv[1]);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "read_log: %s\n", e.what());
        return 1;
    }
    if (lf.rows.empty()) { std::fprintf(stderr, "empty log\n"); return 1; }
    const double dt = lf.header.dt;
    const char* names[4] = {"FL", "FR", "RL", "RR"};
    otolith::LegGeom legs[4];
    for (int i = 0; i < 4; ++i) legs[i] = otolith::leg_geom(names[i]);

    // Welford accumulators: mean and M2 for the spread of r_dot.
    struct Acc { double n = 0, mean[3] = {0,0,0}, m2[3] = {0,0,0}; };
    Acc all[4], stance[4];
    double prev[4][3] = {{0}};
    bool have_prev = false;
    // A stance foot is world-fixed by construction, so its r_base is CONSTANT
    // and r_dot should be zero apart from noise. Skipping the first sample of
    // each stance avoids the transition step, which is a real discontinuity and
    // not noise.
    int prev_contact[4] = {0,0,0,0};
    // Per-stance placement offset, re-drawn at each touchdown.
    double place[4][3] = {{0}};
    std::uint64_t rng = 0x9E3779B97F4A7C15ull;
    auto gauss = [&]() {
        rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
        const double u = (double)((rng >> 11) & ((1ull << 53) - 1)) / (double)(1ull << 53);
        return (u + u + u - 1.5) * 2.0;  // approx N(0,1), no <random> needed
    };

    for (auto& row : lf.rows) {
        Eigen::Vector3d q(row.qj[0], row.qj[1], row.qj[2]);
        for (int f = 0; f < 4; ++f) {
            Eigen::Vector3d r = otolith::foot_pos_base(
                legs[f], row.qj[3*f], row.qj[3*f+1], row.qj[3*f+2]);
            bool contact = row.contacts[f] != 0;
            if (place_err_mm > 0.0 && contact && !prev_contact[f])
                for (int k = 0; k < 3; ++k)
                    place[f][k] = place_err_mm * 1e-3 * gauss();
            if (place_err_mm > 0.0)
                for (int k = 0; k < 3; ++k) r[k] += place[f][k];
            if (have_prev) {
                double v[3];
                for (int k = 0; k < 3; ++k) v[k] = (r[k] - prev[f][k]) / dt;
                auto push = [&](Acc& a, const double* x) {
                    a.n += 1;
                    for (int k = 0; k < 3; ++k) {
                        const double d = x[k] - a.mean[k];
                        a.mean[k] += d / a.n;
                        a.m2[k] += d * (x[k] - a.mean[k]);
                    }
                };
                push(all[f], v);
                // Only count r_dot from samples that are both in stance now and
                // were in stance previously.
                if (contact && prev_contact[f]) push(stance[f], v);
            }
            for (int k = 0; k < 3; ++k) prev[f][k] = r[k];
            prev_contact[f] = contact ? 1 : 0;
        }
        have_prev = true;
    }

    auto rms = [](const Acc& a, int k) {
        return a.n > 1 ? std::sqrt(a.m2[k] / (a.n - 1)) : 0.0;
    };
    std::printf("log %s: %zu rows, dt=%.4f s\n", argv[1], lf.rows.size(), dt);
    std::printf("\nsigma(r_dot) per foot, m/s  [filter assumes sigma_leg^2 for EACH axis]\n");
    std::printf("%-4s %10s %10s %10s | %10s %10s %10s\n",
                "foot", "all_x", "all_y", "all_z", "st_x", "st_y", "st_z");
    for (int f = 0; f < 4; ++f) {
        std::printf("%-4s %10.4f %10.4f %10.4f | %10.4f %10.4f %10.4f\n",
                    names[f], rms(all[f],0), rms(all[f],1), rms(all[f],2),
                    rms(stance[f],0), rms(stance[f],1), rms(stance[f],2));
    }
    // The stance-only figure is the honest one: a swing foot's r_dot is real
    // motion, not noise, and averaging it in inflates the apparent measurement
    // noise by the swing/stance duty ratio.
    double worst = 0.0;
    for (int f = 0; f < 4; ++f)
        for (int k = 0; k < 3; ++k) worst = std::max(worst, rms(stance[f], k));
    std::printf("\nworst stance sigma(r_dot) = %.4f m/s\n", worst);
    std::printf("shipped sigma_leg          = 0.3 m/s\n");
    std::printf("ratio shipped/measured     = %.4f\n", worst > 0 ? 0.3 / worst : 0.0);
    return 0;
}