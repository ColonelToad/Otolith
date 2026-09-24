// fuse_log_fixed: fixed-point predict + float update hybrid (ADR-0007 M1).
//
// Mirrors fuse_log.cpp, but the predict step runs the Q8.24 bit-model
// (fixed::FixedPredict) while update_legs stays float (Eigen) until the
// Cholesky stretch. State converts fixed->float for update and back per
// row (±0.5 LSB each way — part of the measured bound, and of any real
// deployment that keeps update in float). ESTM-v2 output identical
// layout, so evaluate.py scores it unchanged.
//
// usage: fuse_log_fixed <in.otlg> <out.estm> [dt_override]
#include "fixed/fixed_predict.hpp"
#include "otolith/estimate_log.hpp"
#include "otolith/fusion.hpp"
#include "otolith/log.hpp"
#include <iostream>

namespace {
Eigen::Quaterniond to_eigen_q(const otolith::fixed::q24 q[4]) {
    using namespace otolith::fixed;
    return Eigen::Quaterniond(to_double(q[0]), to_double(q[1]),
                              to_double(q[2]), to_double(q[3]));
}
Eigen::Vector3d to_eigen_v(const otolith::fixed::q24 v[3]) {
    using namespace otolith::fixed;
    return Eigen::Vector3d(to_double(v[0]), to_double(v[1]), to_double(v[2]));
}
void from_eigen_q(otolith::fixed::q24 q[4], const Eigen::Quaterniond& e) {
    using namespace otolith::fixed;
    q[0] = from_double(e.w());
    q[1] = from_double(e.x());
    q[2] = from_double(e.y());
    q[3] = from_double(e.z());
}
void from_eigen_v(otolith::fixed::q24 v[3], const Eigen::Vector3d& e) {
    using namespace otolith::fixed;
    v[0] = from_double(e.x());
    v[1] = from_double(e.y());
    v[2] = from_double(e.z());
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: fuse_log_fixed <in.otlg> <out.estm> [dt_override]\n";
        return 2;
    }
    std::string in = argv[1], out = argv[2];
    try {
        auto lf = otolith::read_log(in);
        double dt = lf.header.dt;
        for (int a = 3; a < argc; ++a) dt = std::stod(argv[a]);

        otolith::fixed::FixedPredict fx;
        // Init from first GT (same as fuse_log.cpp)
        if (!lf.rows.empty()) {
            auto& r0 = lf.rows[0];
            fx.s.q[0] = otolith::fixed::from_double(r0.gt_quat[0]);
            fx.s.q[1] = otolith::fixed::from_double(r0.gt_quat[1]);
            fx.s.q[2] = otolith::fixed::from_double(r0.gt_quat[2]);
            fx.s.q[3] = otolith::fixed::from_double(r0.gt_quat[3]);
            from_eigen_v(fx.s.p, Eigen::Vector3d(r0.gt_pos[0], r0.gt_pos[1], r0.gt_pos[2]));
            from_eigen_v(fx.s.v, Eigen::Vector3d(r0.gt_vel[0], r0.gt_vel[1], r0.gt_vel[2]));
        }
        // Float update side: mirrors C++ filter state, synced per row.
        // P is shared by bulk copy (fixed->float before update,
        // float->fixed after) — conversion churn is part of the bound.
        otolith::FusionEKF ekf_f;
        {
            auto& r0 = lf.rows[0];
            otolith::FusionState s = ekf_f.state();
            s.q = Eigen::Quaterniond(r0.gt_quat[0], r0.gt_quat[1], r0.gt_quat[2], r0.gt_quat[3]);
            s.q.normalize();
            s.p = Eigen::Vector3d(r0.gt_pos[0], r0.gt_pos[1], r0.gt_pos[2]);
            s.v = Eigen::Vector3d(r0.gt_vel[0], r0.gt_vel[1], r0.gt_vel[2]);
            s.bg.setZero();
            s.ba.setZero();
            ekf_f.set_state(s);
        }

        std::vector<otolith::EstRowV2> est;
        est.reserve(lf.rows.size());
        for (auto& row : lf.rows) {
            double gyro[3] = {row.gyro[0], row.gyro[1], row.gyro[2]};
            double accel[3] = {row.accel[0], row.accel[1], row.accel[2]};
            fx.step(dt, gyro, accel);

            // Float update on the fixed-predicted state (convert, update, write back)
            otolith::FusionState fs = ekf_f.state();
            fs.q = to_eigen_q(fx.s.q);
            fs.q.normalize();
            fs.p = to_eigen_v(fx.s.p);
            fs.v = to_eigen_v(fx.s.v);
            fs.bg = to_eigen_v(fx.s.bg);
            fs.ba = to_eigen_v(fx.s.ba);
            for (int r = 0; r < 15; ++r)
                for (int c = 0; c < 15; ++c)
                    fs.P(r, c) = otolith::fixed::p_to_double(fx.s.P[r * 15 + c]);
            ekf_f.set_state(fs);
            Eigen::Matrix<double,12,1> qj;
            for (int i = 0; i < 12; ++i) qj[i] = row.qj[i];
            std::array<uint8_t,4> contacts{row.contacts[0], row.contacts[1], row.contacts[2], row.contacts[3]};
            Eigen::Vector3d gm(gyro[0], gyro[1], gyro[2]);
            ekf_f.update_legs(qj, contacts, gm, dt);
            // write back (state + covariance)
            auto st = ekf_f.state();
            from_eigen_q(fx.s.q, st.q);
            from_eigen_v(fx.s.p, st.p);
            from_eigen_v(fx.s.v, st.v);
            from_eigen_v(fx.s.bg, st.bg);
            from_eigen_v(fx.s.ba, st.ba);
            for (int r = 0; r < 15; ++r)
                for (int c = 0; c < 15; ++c)
                    fx.s.P[r * 15 + c] = otolith::fixed::p_from_double(st.P(r, c));

            otolith::EstRowV2 er{};
            er.base.t = row.t;
            er.base.p[0] = st.p.x(); er.base.p[1] = st.p.y(); er.base.p[2] = st.p.z();
            er.base.quat[0] = st.q.w(); er.base.quat[1] = st.q.x();
            er.base.quat[2] = st.q.y(); er.base.quat[3] = st.q.z();
            er.base.v[0] = st.v.x(); er.base.v[1] = st.v.y(); er.base.v[2] = st.v.z();
            er.base.bg[0] = st.bg.x(); er.base.bg[1] = st.bg.y(); er.base.bg[2] = st.bg.z();
            er.base.ba[0] = st.ba.x(); er.base.ba[1] = st.ba.y(); er.base.ba[2] = st.ba.z();
            for (int r = 0; r < 15; ++r)
                for (int c = 0; c < 15; ++c) er.P[r * 15 + c] = st.P(r, c);
            est.push_back(er);
        }
        otolith::write_estimate_v2(out, dt, est);
        std::cout << "fuse_log_fixed: " << lf.rows.size() << " rows -> " << est.size()
                  << " estimates, dt=" << dt << " (fixed predict + float update)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "fuse_log_fixed error: " << e.what() << "\n";
        return 1;
    }
}
