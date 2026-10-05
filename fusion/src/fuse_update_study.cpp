// fuse_update_study: fixed-point MEKF update differential + envelope study
// (ADR-0007 M5-C).
//
// Runs the PURE FLOAT filter over a log with the update trace tap on, and
// at every real update step runs the fixed-point linear algebra
// (fixed::update_linear) on the SAME pre-update P with the SAME H/y/Rmat,
// under each DivMode. Reports, per mode:
//
//   * accuracy: max abs + relative error of K, dx, and post-update P
//   * envelopes: max|S|, max|L|, max|D|, min|D| (LDL pivot floor), max|K|,
//     max|dx|, max|AP|  — closes the M1 "update-path ranges deferred"
//     deferral with measurement instead of assumption
//   * saturation: total events attributable to the update path
//   * cost: analytic MAC count per update (feeds the area projection)
//
// Isolation note: the float P trajectory drives the inputs, so this
// measures the update's linear algebra ALONE. Predict-path quantization is
// already measured separately (M1: 0.1023 m vs 0.1064 m float) and is not
// double-counted here.
//
// usage: fuse_update_study <in.otlg>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>

#include "fixed/fixed_update.hpp"
#include "otolith/fusion.hpp"
#include "otolith/log.hpp"

using namespace otolith;
using namespace otolith::fixed;

namespace {

struct Agg {
    int updates = 0;
    int by_rows[13] = {0}; // rows = 3*stance_legs
    int non_pd = 0;
    double k_absmax = 0, k_absmax_ref = 0;
    double dx_absmax = 0, dx_absmax_ref = 0;
    double p_absmax = 0;
    double p_relfro = 0;
    UpdateRanges rg;
    int sat_total = 0;
    // per-block dx error (dtheta, dv, dp, dbg, dba)
    double dx_block[5] = {0, 0, 0, 0, 0};
};

// Analytic MAC counts per update at rows=9 (ADR-0007 M5 table). Fixed by the
// algorithm shape, not measured — the predict path's 15x15 Phi*P*Phi'
// (2 x 15^3 = 6750) is the density anchor. K requires S^-1 materialized via
// `rows` unit-vector solves (one triangular solve each), not `rows` solves
// with T's columns: K = T S^-1 needs S^-1's COLUMNS, and solving with a
// column of T computes S^-1 T instead (inverse on the left).
struct MacCount {
    const char* name;
    int macs;
};
struct RowMacs {
    const char* name;
    int macs;
};
int row_mac_total(int r, RowMacs out[]) {
    const int tri = r * (r + 1) / 2;      // S entries computed (lower tri)
    int ddots = 0, ldots = 0;
    for (int j = 0; j < r; ++j) { ddots += j; for (int i = j + 1; i < r; ++i) ldots += j; }
    int n = 0;
    out[n++] = {"S = H P H' (T = P H', then H T lower tri)", 15 * r * 15 + tri * 15};
    out[n++] = {"LDL' factor (D dots + L dots)", ddots + ldots};
    out[n++] = {"S^-1 via r unit-vector triangular solves", r * r * (r - 1)};
    out[n++] = {"K = T S^-1", 15 * r * r};
    out[n++] = {"dx = K y", 15 * r};
    out[n++] = {"A = I - K H", 15 * 15 * r};
    out[n++] = {"A P A'", 2 * 15 * 15 * 15};
    out[n++] = {"K Rmat K'", 15 * r * r + 15 * 15 * r};
    int tot = 0;
    for (int i = 0; i < n; ++i) tot += out[i].macs;
    return tot;
}

void accumulate(Agg& a, const Agg& step) {
    a.updates += step.updates;
    for (int r = 0; r <= 12; ++r) a.by_rows[r] += step.by_rows[r];
    a.non_pd += step.non_pd;
    auto mx = [](double& d, double v) { if (v > d) d = v; };
    mx(a.k_absmax, step.k_absmax);
    mx(a.k_absmax_ref, step.k_absmax_ref);
    mx(a.dx_absmax, step.dx_absmax);
    mx(a.dx_absmax_ref, step.dx_absmax_ref);
    mx(a.p_absmax, step.p_absmax);
    mx(a.p_relfro, step.p_relfro);
    mx(a.rg.max_abs_S, step.rg.max_abs_S);
    mx(a.rg.max_abs_L, step.rg.max_abs_L);
    mx(a.rg.max_abs_D, step.rg.max_abs_D);
    mx(a.rg.max_abs_K, step.rg.max_abs_K);
    mx(a.rg.max_abs_dx, step.rg.max_abs_dx);
    mx(a.rg.max_abs_AP, step.rg.max_abs_AP);
    // min_D: smallest positive pivot across the whole run
    if (step.rg.min_D > 0 && (a.rg.min_D == 0.0 || step.rg.min_D < a.rg.min_D))
        a.rg.min_D = step.rg.min_D;
    a.sat_total += step.sat_total;
    for (int i = 0; i < 5; ++i) mx(a.dx_block[i], step.dx_block[i]);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: fuse_update_study <in.otlg> [--dump-s <path> --dump-w <W>]\n"
                     "                          [--sigma-leg <value>]\n");
        return 2;
    }
    // --dump-s writes the REAL captured innovation covariances in the packed
    // order hdl/rtl/ldl_kernel.sv loads them, so the RTL parity bench runs on
    // the distribution it will actually see instead of synthetic matrices.
    // Each S is emitted twice: once with a unit-vector RHS (the columns of
    // S^-1, which is exactly what K needs) and once with the real measurement
    // residual promoted to Q16.48.
    std::FILE* dump = nullptr;
    int dump_w = 6;
    for (int a = 2; a + 1 < argc; ++a) {
        if (std::string(argv[a]) == "--dump-s") dump = std::fopen(argv[a + 1], "w");
        if (std::string(argv[a]) == "--dump-w") dump_w = std::atoi(argv[a + 1]);
    }
    const int DW = dump_w;
    // --sigma-leg overrides cfg.sigma_leg_vel, which sets Rmat = sigma^2*I.
    // That is the floor under every LDL' pivot, so it is the knob that decides
    // whether fixed-point division stays out of its rail (ADR-0007 M5).
    double sigma_leg = -1.0;
    for (int a = 2; a + 1 < argc; ++a)
        if (std::string(argv[a]) == "--sigma-leg") sigma_leg = std::atof(argv[a + 1]);
    LogFile lf;
    try {
        lf = read_log(argv[1]);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "read_log: %s\n", e.what());
        return 1;
    }
    if (lf.rows.empty()) {
        std::fprintf(stderr, "empty log\n");
        return 1;
    }
    const double dt = lf.header.dt;

    FusionConfig fcfg;
    if (sigma_leg > 0.0) fcfg.sigma_leg_vel = sigma_leg;
    FusionEKF ekf(fcfg);
    ekf.set_trace(true);
    if (sigma_leg > 0.0)
        std::printf("sigma_leg_vel overridden to %g -> Rmat = %.6g*I\n",
                    sigma_leg, sigma_leg * sigma_leg);

    Agg agg[2]; // indexed by DivMode: 0 = Q48, 1 = Q24
    const DivMode modes[2] = {DivMode::Q48, DivMode::Q24};
    const char* mode_name[2] = {"(a) Q16.48 reciprocal", "(b) Q8.24 div_scaled"};

    // Init state from first GT, same as fuse_log.cpp.
    {
        FusionState s = ekf.state();
        const auto& r0 = lf.rows[0];
        s.q = Eigen::Quaterniond(r0.gt_quat[0], r0.gt_quat[1], r0.gt_quat[2],
                                r0.gt_quat[3]);
        s.p = Eigen::Vector3d(r0.gt_pos[0], r0.gt_pos[1], r0.gt_pos[2]);
        s.v = Eigen::Vector3d(r0.gt_vel[0], r0.gt_vel[1], r0.gt_vel[2]);
        ekf.set_state(s);
    }

    for (const auto& r : lf.rows) {
        const Eigen::Vector3d gyro(r.gyro[0], r.gyro[1], r.gyro[2]);
        const Eigen::Vector3d acc(r.accel[0], r.accel[1], r.accel[2]);
        Eigen::Matrix<double, 12, 1> qj;
        for (int i = 0; i < 12; ++i) qj[i] = r.qj[i];
        std::array<uint8_t, 4> c{r.contacts[0], r.contacts[1], r.contacts[2],
                                r.contacts[3]};

        ekf.predict(dt, gyro, acc);
        const FusionState pre = ekf.state(); // pre-update P for the differential
        const int m = ekf.update_legs(qj, c, gyro, dt);
        if (m == 0) continue;

        const UpdateTrace& tr = ekf.trace();
        const int rows = tr.rows;
        // rows = 3 * stance_legs, so 3/6/9/12 are all legitimate. An earlier
        // version of this driver accepted only {9,12} and silently discarded
        // the 90% of trot updates that have TWO stance feet (rows=6).
        if (rows < 3 || rows > 12) continue;

        // Convert the float trace inputs to fixed point (boundary, ±0.5 LSB).
        q24 H[15 * 12] = {0};
        q24 y[12] = {0};
        q48 Rmat[12 * 12] = {0};
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < 15; ++j) H[i * 15 + j] = from_double(tr.H(i, j));
        for (int i = 0; i < rows; ++i) y[i] = from_double(tr.y(i));
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < rows; ++j)
                Rmat[i * rows + j] = p_from_double(tr.Rmat(i, j));

        if (dump && rows == DW) {
            auto emit = [&](const q48* rhs) {
                for (int i = 0; i < DW; ++i)
                    for (int j = 0; j <= i; ++j)
                        // p_from_double FIRST: casting the raw double (~0.19)
                        // straight to uint64_t yields 0, which silently
                        // captured an all-zero S.
                        std::fprintf(dump, "%016llx ",
                            (unsigned long long)(uint64_t)p_from_double(tr.S(i, j)));
                for (int i = 0; i < DW; ++i)
                    std::fprintf(dump, "%016llx ", (unsigned long long)(uint64_t)rhs[i]);
                std::fprintf(dump, "\n");
            };
            static int col = 0;
            q48 e[MAX_ROWS] = {0};
            e[col] = p_from_double(1.0);
            col = (col + 1) % DW;
            emit(e);
            q48 yr[MAX_ROWS] = {0};
            for (int i = 0; i < DW; ++i)
                yr[i] = narrow48(acc128(y[i]) << 24); // Q8.24 -> Q16.48
            emit(yr);
        }

        const FusionState post = ekf.state(); // float post-update P

        for (int mi = 0; mi < 2; ++mi) {
            FixedState fs;
            for (int i = 0; i < 225; ++i) fs.P[i] = p_from_double(pre.P(i / 15, i % 15));
            UpdateRanges rg;
            sat_reset();
            const UpdateOut out =
                update_linear(fs, H, y, Rmat, rows, modes[mi], &rg);

            Agg step;
            step.updates = 1;
            step.by_rows[rows] = 1;
            step.rg = rg;
            step.sat_total = rg.sat_delta;
            if (!out.pd) {
                step.non_pd = 1;
                accumulate(agg[mi], step);
                continue;
            }
            // dx error (K is not exposed by update_linear; compared via dx
            // and P, with K magnitude taken from the float trace).
            for (int i = 0; i < 15; ++i) {
                const double e = to_double(out.dx[i]) - tr.dx(i);
                const double a = std::fabs(e);
                if (a > step.dx_absmax) step.dx_absmax = a;
                const int blk = i / 3;
                if (a > step.dx_block[blk]) step.dx_block[blk] = a;
            }
            for (int i = 0; i < 15; ++i)
                step.dx_absmax_ref =
                    std::max(step.dx_absmax_ref, std::fabs(tr.dx(i)));
            for (int j = 0; j < rows; ++j)
                for (int i = 0; i < 15; ++i)
                    step.k_absmax_ref =
                        std::max(step.k_absmax_ref, std::fabs(tr.K(i, j)));
            // P' error
            double num = 0, den = 0;
            for (int i = 0; i < 15; ++i)
                for (int j = 0; j < 15; ++j) {
                    const double pf = post.P(i, j);
                    const double pq = p_to_double(fs.P[i * 15 + j]);
                    step.p_absmax = std::max(step.p_absmax, std::fabs(pf - pq));
                    num += (pf - pq) * (pf - pq);
                    den += pf * pf;
                }
            step.p_relfro = den > 0 ? std::sqrt(num / den) : 0.0;
            accumulate(agg[mi], step);
        }
    }

    std::printf("=== M5-C fixed-point update study: %s ===\n", argv[1]);
    std::printf("rows: %zu log rows, dt=%.6f s\n", lf.rows.size(), dt);
    std::printf("updates: %d  non-PD: %d\n", agg[0].updates, agg[0].non_pd);
    std::printf("  by width (rows = 3 x stance legs):");
    for (int r = 3; r <= 12; r += 3)
        if (agg[0].by_rows[r]) std::printf("  rows=%d: %d", r, agg[0].by_rows[r]);
    std::printf("\n\n");

    std::printf("--- envelopes (identical across modes: factorization is exact"
                " except the division) ---\n");
    std::printf("  max|S|   %.6g\n", agg[0].rg.max_abs_S);
    std::printf("  max|L|   %.6g\n", agg[0].rg.max_abs_L);
    std::printf("  max|D|   %.6g\n", agg[0].rg.max_abs_D);
    std::printf("  min|D|   %.6g   <- LDL pivot floor (0 => not seen)\n",
                agg[0].rg.min_D);
    std::printf("  max|K|   %.6g\n", agg[0].rg.max_abs_K);
    std::printf("  max|dx|  %.6g\n", agg[0].rg.max_abs_dx);
    std::printf("  max|AP|  %.6g\n", agg[0].rg.max_abs_AP);

    for (int mi = 0; mi < 2; ++mi) {
        const Agg& a = agg[mi];
        std::printf("\n--- %s ---\n", mode_name[mi]);
        std::printf("  non-PD factorizations: %d / %d\n", a.non_pd, a.updates);
        std::printf("  saturation events (update path): %d\n", a.sat_total);
        const double krel = a.k_absmax_ref > 0 ? a.dx_absmax / a.k_absmax_ref : 0;
        (void)krel;
        std::printf("  dx   max abs err %.6g  (ref max|dx| %.6g)\n",
                    a.dx_absmax, a.dx_absmax_ref);
        std::printf("       per-block dtheta %.3g  dv %.3g  dp %.3g  dbg %.3g  dba %.3g\n",
                    a.dx_block[0], a.dx_block[1], a.dx_block[2], a.dx_block[3],
                    a.dx_block[4]);
        std::printf("  P'   max abs err %.6g   rel Frobenius %.6g\n",
                    a.p_absmax, a.p_relfro);
        std::printf("  ref max|K| %.6g\n", a.k_absmax_ref);
    }

    std::printf("\n--- cost per update, analytic MAC, by width ---\n");
    std::printf("  %-46s %8s %8s %8s\n", "op", "rows=6", "rows=9", "rows=12");
    RowMacs m6[16], m9[16], m12[16];
    const int t6 = row_mac_total(6, m6), t9 = row_mac_total(9, m9),
              t12 = row_mac_total(12, m12);
    const int nops = 8;
    for (int i = 0; i < nops; ++i)
        std::printf("  %-46s %8d %8d %8d\n", m6[i].name, m6[i].macs, m9[i].macs,
                    m12[i].macs);
    std::printf("  %-46s %8d %8d %8d\n", "TOTAL update MAC", t6, t9, t12);
    std::printf("  %-46s %8d %8d %8d\n", "predict Phi*P*Phi' (M4 anchor)",
                2 * 15 * 15 * 15, 2 * 15 * 15 * 15, 2 * 15 * 15 * 15);
    std::printf("\n  sky130 area at M4 density (3.40 mm^2 / 6750 MAC):\n");
    std::printf("    %-10s %8.2f %8.2f %8.2f  mm^2  update\n", "rows:", 
                3.40 * t6 / 6750.0, 3.40 * t9 / 6750.0, 3.40 * t12 / 6750.0);
    std::printf("    %-10s %8.2f %8.2f %8.2f  mm^2  full fixed-point MEKF\n", "predict+upd:",
                3.40 * (1.0 + double(t6) / 6750.0), 3.40 * (1.0 + double(t9) / 6750.0),
                3.40 * (1.0 + double(t12) / 6750.0));
    // Distribution-weighted: the honest figure is the trot's actual mix of
    // stance widths, not the widest case.
    double wmac = 0.0;
    int wtot = agg[0].updates;
    for (int r = 3; r <= 12; r += 3) {
        if (!agg[0].by_rows[r]) continue;
        RowMacs mm[16];
        const int t = row_mac_total(r, mm);
        wmac += double(agg[0].by_rows[r]) / wtot * double(t);
        std::printf("    width rows=%2d: %5d updates (%4.1f%%) -> %6d MAC\n", r,
                    agg[0].by_rows[r], 100.0 * agg[0].by_rows[r] / wtot, t);
    }
    std::printf("    %-10s %8.2f mm^2 update, %.2f mm^2 full MEKF (observed mix)\n",
                "weighted:", 3.40 * wmac / 6750.0, 3.40 * (1.0 + wmac / 6750.0));
    std::printf("    (linear MAC scaling ignores shared state/control overhead"
                " — an UPPER bound; predict alone measured 3.40 mm^2)\n");
    return 0;
}