#pragma once
// Error-state MEKF for contact-aided proprioceptive odometry.
// Nominal: q (world->body? we use body->world), p,v in world, bg,ba in body.
// Error: 15-dim [dtheta(3), dv(3), dp(3), dbg(3), dba(3)]

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace otolith {

struct FusionConfig {
    double sigma_gyro = 0.01;      // rad/s / sqrt(Hz)
    double sigma_accel = 0.15;     // m/s^2 / sqrt(Hz)
    double sigma_bg_rw = 1e-5;     // rad/s per sqrt(s)
    double sigma_ba_rw = 1e-4;     // m/s^2 per sqrt(s)
    double sigma_leg_vel = 0.05;   // m/s leg-odometry per foot
    double gravity = 9.81;
};

struct FusionState {
    Eigen::Quaterniond q;      // body -> world rotation
    Eigen::Vector3d p;         // world position
    Eigen::Vector3d v;         // world velocity
    Eigen::Vector3d bg;        // gyro bias body
    Eigen::Vector3d ba;        // accel bias body
    Eigen::Matrix<double,15,15> P; // error covariance
};

inline FusionState make_default_state() {
    FusionState s;
    s.q = Eigen::Quaterniond::Identity();
    s.p.setZero(); s.v.setZero(); s.bg.setZero(); s.ba.setZero();
    s.P.setIdentity();
    s.P *= 1e-2;
    // larger orientation/velocity uncertainty
    s.P.block<3,3>(0,0) *= 10; // dtheta
    s.P.block<3,3>(3,3) *= 10; // dv
    return s;
}

class FusionEKF {
public:
    explicit FusionEKF(const FusionConfig& cfg = {}, FusionState s = make_default_state())
        : cfg_(cfg), state_(std::move(s)) {}

    const FusionState& state() const { return state_; }
    void set_state(const FusionState& s) { state_ = s; }

    // IMU propagation: gyro_m, accel_m in body frame (gravity included, as sensor gives).
    void predict(double dt, const Eigen::Vector3d& gyro_m, const Eigen::Vector3d& accel_m);

    // Leg-odometry update: qj[12] FL,FR,RL,RR hip/thigh/calf, contacts[4] 0/1, gyro_m needed for omega.
    // Returns number of stance feet used (0 => no update).
    int update_legs(const Eigen::Matrix<double,12,1>& qj,
                    const std::array<uint8_t,4>& contacts,
                    const Eigen::Vector3d& gyro_m);

private:
    FusionConfig cfg_;
    FusionState state_;
};

// helpers exposed for testing
Eigen::Matrix3d quat_to_mat(const Eigen::Quaterniond& q);
Eigen::Matrix3d skew(const Eigen::Vector3d& w);

} // namespace otolith
