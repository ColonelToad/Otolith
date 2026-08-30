#include "otolith/fusion.hpp"
#include "otolith/leg_kin.hpp"
#include <cmath>

namespace otolith {

Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
    Eigen::Matrix3d S;
    S << 0, -w.z(), w.y(),
         w.z(), 0, -w.x(),
        -w.y(), w.x(), 0;
    return S;
}
Eigen::Matrix3d quat_to_mat(const Eigen::Quaterniond& q) { return q.toRotationMatrix(); }

static Eigen::Quaterniond exp_quat(const Eigen::Vector3d& w_dt) {
    double angle = w_dt.norm();
    if (angle < 1e-12) return Eigen::Quaterniond::Identity();
    Eigen::Vector3d axis = w_dt / angle;
    double h = angle * 0.5;
    double s = std::sin(h);
    return Eigen::Quaterniond(std::cos(h), axis.x()*s, axis.y()*s, axis.z()*s);
}

void FusionEKF::predict(double dt, const Eigen::Vector3d& gyro_m, const Eigen::Vector3d& accel_m) {
    const Eigen::Vector3d w = gyro_m - state_.bg;
    const Eigen::Vector3d a = accel_m - state_.ba;
    const Eigen::Matrix3d R = state_.q.toRotationMatrix();
    const Eigen::Vector3d g(0,0,-cfg_.gravity);

    // --- nominal propagation (first-order) ---
    state_.q = (state_.q * exp_quat(w * dt)).normalized();
    state_.v += (R * a + g) * dt;
    state_.p += state_.v * dt; // use updated v (semi-implicit)

    // --- covariance propagation: Phi = I + F*dt ---
    constexpr int N = 15;
    Eigen::Matrix<double,N,N> F = Eigen::Matrix<double,N,N>::Zero();
    F.block<3,3>(0,0) = -skew(w);
    F.block<3,3>(0,9) = -Eigen::Matrix3d::Identity();
    F.block<3,3>(3,0) = -R * skew(a);
    F.block<3,3>(3,12) = -R;
    F.block<3,3>(6,3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double,N,N> Phi = Eigen::Matrix<double,N,N>::Identity() + F * dt;

    // Qd = G Q G^T dt
    Eigen::Matrix<double,N,N> Qd = Eigen::Matrix<double,N,N>::Zero();
    double s_g = cfg_.sigma_gyro, s_a = cfg_.sigma_accel;
    double s_bg = cfg_.sigma_bg_rw, s_ba = cfg_.sigma_ba_rw;
    Qd.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * s_g*s_g * dt;
    Qd.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * s_a*s_a * dt;
    Qd.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * s_bg*s_bg * dt;
    Qd.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * s_ba*s_ba * dt;

    state_.P = Phi * state_.P * Phi.transpose() + Qd;
    // enforce symmetry
    state_.P = 0.5 * (state_.P + state_.P.transpose());
}

int FusionEKF::update_legs(const Eigen::Matrix<double,12,1>& qj,
                           const std::array<uint8_t,4>& contacts,
                           const Eigen::Vector3d& gyro_m,
                           double dt) {
    const char* names[4] = {"FL","FR","RL","RR"};
    std::vector<int> stance;
    for (int i=0;i<4;++i) if (contacts[i]) stance.push_back(i);
    if (stance.empty()) { prev_qj_ = qj; has_prev_ = true; return 0; }
    if (!has_prev_) { prev_qj_ = qj; has_prev_ = true; return 0; }

    const Eigen::Matrix3d R = state_.q.toRotationMatrix();
    const Eigen::Vector3d w = gyro_m - state_.bg;
    const int m = (int)stance.size();
    const int rows = 3*m;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, 15);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(rows);
    Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(rows, rows);
    for (int k=0;k<m;++k) {
        int leg = stance[k];
        LegGeom lg = leg_geom(names[leg]);
        Eigen::Vector3d qleg = qj.segment<3>(leg*3);
        Eigen::Vector3d r_base = foot_pos_base(lg, qleg); // base-frame foot position
        Eigen::Vector3d qprev = prev_qj_.segment<3>(leg*3);
        Eigen::Vector3d r_prev = foot_pos_base(lg, qprev);
        Eigen::Vector3d r_dot = (r_base - r_prev) / dt;
        Eigen::Vector3d omega_cross_r = w.cross(r_base);
        Eigen::Vector3d h = state_.v + R * (omega_cross_r + r_dot);
        y.segment<3>(k*3) = -h;

        // Jacobian: H = [-R[omega x r]_x, I, 0, R[r]_x, 0]
        Eigen::Matrix3d wr = skew(omega_cross_r);
        Eigen::Matrix3d rr = skew(r_base);
        H.block<3,3>(k*3, 0) = -R * wr;
        H.block<3,3>(k*3, 3) = Eigen::Matrix3d::Identity();
        // dp block 0
        H.block<3,3>(k*3, 9) = R * rr;
        // dba block 0

        double sigma = cfg_.sigma_leg_vel;
        Rmat.block<3,3>(k*3,k*3) = Eigen::Matrix3d::Identity() * sigma*sigma;
        // Note: we ignore lever-arm scaling; could scale with |r|
        (void)leg; (void)qleg;
    }

    // Kalman gain
    Eigen::MatrixXd S = H * state_.P * H.transpose() + Rmat;
    Eigen::MatrixXd K = state_.P * H.transpose() * S.inverse();

    Eigen::VectorXd dx = K * y;
    Eigen::Vector3d dtheta = dx.segment<3>(0);
    Eigen::Vector3d dv = dx.segment<3>(3);
    Eigen::Vector3d dp = dx.segment<3>(6);
    Eigen::Vector3d dbg = dx.segment<3>(9);
    Eigen::Vector3d dba = dx.segment<3>(12);

    state_.q = (state_.q * exp_quat(dtheta)).normalized();
    state_.v += dv;
    state_.p += dp;
    state_.bg += dbg;
    state_.ba += dba;

    // Joseph form for stability
    constexpr int N=15;
    Eigen::Matrix<double,N,N> I = Eigen::Matrix<double,N,N>::Identity();
    Eigen::Matrix<double,N,N> KH = K * H;
    Eigen::Matrix<double,N,N> A = I - KH;
    state_.P = A * state_.P * A.transpose() + K * Rmat * K.transpose();
    state_.P = 0.5*(state_.P + state_.P.transpose());
    prev_qj_ = qj;
    has_prev_ = true;
    return m;
}

} // namespace otolith
