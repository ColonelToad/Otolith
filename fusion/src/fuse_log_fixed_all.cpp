// fuse_log_fixed_all: the COMPLETE fixed-point filter — predict, measure and
// correct — against the float filter, over the same log (ADR-0007 M5).
//
// fuse_log_fixed (M1) ran fixed predict + FLOAT update. fuse_update_study
// (M5-C) ran the fixed update's linear algebra on float H/y. This closes the
// loop: H and y are now built in fixed point too (fixed_meas.hpp), including
// the leg FK, the r_dot division and the wide-range CORDIC.
//
// Emits the same ESTM-v2 layout as fuse_log.cpp, so evaluate.py scores it
// unchanged and the number is directly comparable to M1's 0.1023 m and to the
// float baseline's 0.1064 m.
//
// usage: fuse_log_fixed_all <in.otlg> <out.estm>
#include <cstdio>
#include <string>

#include "fixed/fixed_meas.hpp"
#include "otolith/estimate_log.hpp"
#include "otolith/fusion.hpp"
#include "otolith/log.hpp"

using namespace otolith;
using namespace otolith::fixed;

namespace {

Eigen::Quaterniond to_eigen_q(const q24 q[4]) {
    return Eigen::Quaterniond(to_double(q[0]), to_double(q[1]), to_double(q[2]),
                              to_double(q[3]));
}
void from_eigen_q(q24 q[4], const Eigen::Quaterniond& e) {
    q[0] = from_double(e.w()); q[1] = from_double(e.x());
    q[2] = from_double(e.y()); q[3] = from_double(e.z());
}
Eigen::Vector3d to_eigen_v(const q24 v[3]) {
    return Eigen::Vector3d(to_double(v[0]), to_double(v[1]), to_double(v[2]));
}
void from_eigen_v(q24 v[3], const Eigen::Vector3d& e) {
    v[0] = from_double(e.x()); v[1] = from_double(e.y()); v[2] = from_double(e.z());
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: fuse_log_fixed_all <in.otlg> <out.estm>\n");
        return 2;
    }
    auto lf = read_log(argv[1]);
    const double dt = lf.header.dt;

    FixedUpdateFixed fx;
    FusionEKF ekf;
    if (!lf.rows.empty()) {
        const auto& r0 = lf.rows[0];
        from_eigen_q(fx.s.q, Eigen::Quaterniond(r0.gt_quat[0], r0.gt_quat[1],
                                                r0.gt_quat[2], r0.gt_quat[3]));
        from_eigen_v(fx.s.p, Eigen::Vector3d(r0.gt_pos[0], r0.gt_pos[1], r0.gt_pos[2]));
        from_eigen_v(fx.s.v, Eigen::Vector3d(r0.gt_vel[0], r0.gt_vel[1], r0.gt_vel[2]));
        FusionState s = ekf.state();
        s.q = Eigen::Quaterniond(r0.gt_quat[0], r0.gt_quat[1], r0.gt_quat[2], r0.gt_quat[3]);
        s.p = Eigen::Vector3d(r0.gt_pos[0], r0.gt_pos[1], r0.gt_pos[2]);
        s.v = Eigen::Vector3d(r0.gt_vel[0], r0.gt_vel[1], r0.gt_vel[2]);
        ekf.set_state(s);
    }

    sat_reset();
    long updates = 0, sat_after_predict = 0;
    std::vector<EstRowV2> est;
    est.reserve(lf.rows.size());

    for (const auto& r : lf.rows) {
        const Eigen::Vector3d gyro(r.gyro[0], r.gyro[1], r.gyro[2]);
        const Eigen::Vector3d accel(r.accel[0], r.accel[1], r.accel[2]);

        // --- fixed side, fully fixed ---
        q24 gq[3], aq[3], qj[12];
        std::array<uint8_t, 4> c{r.contacts[0], r.contacts[1], r.contacts[2],
                                r.contacts[3]};
        for (int i = 0; i < 3; ++i) { gq[i] = from_double(gyro[i]); aq[i] = from_double(accel[i]); }
        for (int i = 0; i < 12; ++i) qj[i] = from_double(r.qj[i]);
        fx.predict(dt, gq, aq);
        const int s_pred = sat_count();
        const int used = fx.step(qj, c.data(), gq, from_double(dt), 0.3, DivMode::Q48);
        if (used > 0) ++updates;   // `used` is 3*stance_legs, i.e. rows, not updates

        // Float side runs too, so the two can be compared state-for-state;
        // only the fixed side is written out.
        ekf.predict(dt, gyro, accel);
        Eigen::Matrix<double, 12, 1> qjd;
        for (int i = 0; i < 12; ++i) qjd[i] = r.qj[i];
        ekf.update_legs(qjd, c, gyro, dt);

        EstRowV2 er{};
        er.base.t = r.t;
        er.base.p[0] = to_double(fx.s.p[0]);
        er.base.p[1] = to_double(fx.s.p[1]);
        er.base.p[2] = to_double(fx.s.p[2]);
        er.base.quat[0] = to_double(fx.s.q[0]);
        er.base.quat[1] = to_double(fx.s.q[1]);
        er.base.quat[2] = to_double(fx.s.q[2]);
        er.base.quat[3] = to_double(fx.s.q[3]);
        er.base.v[0] = to_double(fx.s.v[0]);
        er.base.v[1] = to_double(fx.s.v[1]);
        er.base.v[2] = to_double(fx.s.v[2]);
        for (int i = 0; i < 225; ++i)
            er.P[i] = p_to_double(fx.s.P[i]);
        est.push_back(er);
        sat_after_predict = s_pred;
    }
    write_estimate_v2(argv[2], dt, est);
    std::fprintf(stderr,
                 "rows=%zu fixed-point updates=%ld sat_total=%d "
                 "(cumulative sat_count after predict: %d)\n",
                 lf.rows.size(), updates, sat_count(), sat_after_predict);
    return 0;
}
