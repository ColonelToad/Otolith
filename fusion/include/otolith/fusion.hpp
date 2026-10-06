#pragma once
// Error-state MEKF for contact-aided proprioceptive odometry.
// Nominal: q (world->body? we use body->world), p,v in world, bg,ba in body.
// Error: 15-dim [dtheta(3), dv(3), dp(3), dbg(3), dba(3)]

#include <Eigen/Dense>
#include "otolith/leg_kin.hpp"
#include <Eigen/Geometry>

namespace otolith {

struct FusionConfig {
    double sigma_gyro = 0.01;      // rad/s / sqrt(Hz)
    double sigma_accel = 0.15;     // m/s^2 / sqrt(Hz)
    double sigma_bg_rw = 1e-5;     // rad/s per sqrt(s)
    double sigma_ba_rw = 1e-4;     // m/s^2 per sqrt(s)
    double sigma_leg_vel = 0.3;
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

// Instrumentation tap for the M5 fixed-update study (ADR-0007 M5-C).
// Records exactly the intermediates update_legs computed — same values,
// same step, no second code path — so the fixed-point differential is
// against the filter's real arithmetic rather than a re-derivation.
struct UpdateTrace {
    int rows = 0;
    Eigen::MatrixXd H, y, Rmat, S, K, dx;
};

class FusionEKF {
public:
    explicit FusionEKF(const FusionConfig& cfg = {}, FusionState s = make_default_state())
        : cfg_(cfg), state_(std::move(s)) {}

    const FusionState& state() const { return state_; }
    void set_state(const FusionState& s) { state_ = s; }

    // IMU propagation: gyro_m, accel_m in body frame (gravity included, as sensor gives).
    void predict(double dt, const Eigen::Vector3d& gyro_m, const Eigen::Vector3d& accel_m);

    // Leg-odometry update. qj holds the robot's leg joint angles in descriptor
    // order, contacts one flag per leg (only the first robot.n_legs entries are
    // read), gyro_m is needed for omega, dt the sample period (for r_dot).
    // Returns the number of stance feet used (0 => no update).
    //
    // The 12-wide qj is NOT a quadruped assumption: G1 is a biped with 6 DoF per
    // leg, which is also 12. What varies is the stride and the leg count, and
    // both come from robot_. A robot needing more than 12 leg DoF would need a
    // wider type here -- the binding constraint on generality, and the one place
    // the typed fixed-size contract (AGENTS.md rule 1) has to be widened.
    int update_legs(const Eigen::Matrix<double,12,1>& qj,
                    const std::array<uint8_t,kMaxLegs>& contacts,
                    const Eigen::Vector3d& gyro_m,
                    double dt);

    // Select the robot. Defaults to go2, so every existing caller and every
    // recorded result is unaffected.
    void set_robot(const RobotSpec& spec) { robot_ = spec; }
    const RobotSpec& robot() const { return robot_; }

    void set_trace(bool on) { trace_on_ = on; }
    const UpdateTrace& trace() const { return trace_; }

private:
    FusionConfig cfg_;
    FusionState state_;
    RobotSpec robot_{robot_spec("go2")};
    Eigen::Matrix<double,12,1> prev_qj_;
    bool has_prev_ = false;
    bool trace_on_ = false;
    UpdateTrace trace_;
};

// helpers exposed for testing
Eigen::Matrix3d quat_to_mat(const Eigen::Quaterniond& q);
Eigen::Matrix3d skew(const Eigen::Vector3d& w);

} // namespace otolith
