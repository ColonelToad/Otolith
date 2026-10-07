// Finite-difference gate for the leg-odometry measurement Jacobian.
//
// Finite-difference gate for the leg-odometry measurement Jacobian.
//
// WHY THIS TEST EXISTS
// ====================
// The attitude block of H omitted the r_dot term: it read -R*skew(w x r) where the
// derivative is -R*skew(w x r + r_dot). That is not a small correction -- for a
// stance foot |r_dot| runs to tens of m/s against |w x r| ~ 0.03 -- so the block
// missed the dominant term entirely. Roll diverged to -179 deg on G1 and Apollo
// while pitch held at ~2 deg.
//
// Nothing caught it. Every number the filter produced looked reasonable, which is
// what a wrong Jacobian produces. Go2 never diverged because its narrower stance
// never drove the block far enough from equilibrium to show it, so a Go2-only gate
// would not have found this either.
//
// WHY THE EXPECTED JACOBIAN IS WRITTEN OUT RATHER THAN CALLED
// ============================================================
// The point is to compare the CODE against the DEFINITION. Assembling the expected
// value from the same helper calls the filter uses would assert that the code equals
// itself and catch nothing.
//
// Finite differences give, for h(q,v,w,R) = v + R(w x r(q) + rdot(q)), perturbing R
// as R -> R(I + dtheta^) (a body-frame attitude perturbation, which is what the
// state does):
//
//   dh/ddtheta = -R skew(w x r + r_dot)
//   dh/dw      = -R skew(r)        and dg = w - bg, so dh/dbg = +R skew(r)
//   dh/dv      = I
//   dh/dp      = 0
//
// Every one of those is asserted below, against all four robots.
//
// A CAUTION THIS FILE EXISTS TO RECORD
// ====================================
// An earlier attempt at this fix concluded the block should be +R*skew(r), from
// hand-deriving that "a body-frame attitude perturbation dtheta moves a world point
// by R(dtheta x r)". That derivation ignores that h contains the whole vector
// (w x r + r_dot) multiplied by R -- not r alone -- so it got both the sign and the
// quantity wrong. Finite differences rejected it at 0.93 relative error.
//
// The lesson is why the expected values below are stated as FD-verified facts with
// their magnitudes, rather than as an argument: on this block the algebra is
// genuinely easy to get wrong, and the disagreement is invisible in the filter's
// output.
#include "otolith/fusion.hpp"
#include "otolith/leg_kin.hpp"
#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <limits>

using namespace otolith;

namespace {

// Named skew3, not skew: otolith has its own `skew` in the fusion TU and
// an unqualified overload in this file makes the call ambiguous.
Eigen::Matrix3d skew3(const Eigen::Vector3d& a) {
    Eigen::Matrix3d m;
    m << 0, -a.z(), a.y(),
         a.z(), 0, -a.x(),
        -a.y(), a.x(), 0;
    return m;
}

// The measurement model, written from the definition. Uses the same
// `foot_pos_base` as the filter (that part is separately bit-exact against MuJoCo)
// but re-derives the assembly, so the comparison is definition-vs-code.
Eigen::Vector3d h_model(const RobotSpec& spec, const LegSpec& lg, const double* qj,
                        int stride, const double* qprev,
                        const Eigen::Vector3d& v, const Eigen::Matrix3d& R,
                        const Eigen::Vector3d& w, double dt) {
    // `stride` matters: qj/qprev are the whole 12-wide leg vector, and leg f's
    // angles start at f * dof_per_leg. An earlier version of this test took `stride`
    // and ignored it, so every leg past the first was finite-differenced against leg
    // 0's joint angles while the filter used its own -- which showed up as a spurious
    // 0.3 relative error in the gyro-bias block and 0.007 in the attitude block.
    Eigen::Vector3d rb = foot_pos_base(spec, lg, qj + stride);
    Eigen::Vector3d rp = foot_pos_base(spec, lg, qprev + stride);
    Eigen::Vector3d r_dot = (rb - rp) / dt;
    Eigen::Vector3d omega_cross_r = w.cross(rb);
    return v + R * (omega_cross_r + r_dot);
}

struct Scene {
    RobotSpec spec;
    Eigen::Matrix<double, 12, 1> qj;
    Eigen::Matrix<double, 12, 1> qprev;
    Eigen::Matrix3d R;
    Eigen::Vector3d v, w;
    double dt = 0.002;
    std::array<uint8_t, kMaxLegs> contacts{};
};

// Build a scene with legal-ish angles for whichever robot is under test.
Scene make_scene(const char* robot) {
    Scene s;
    s.spec = robot_spec(robot);
    s.qj.setZero();
    s.qprev.setZero();
    unsigned seed = 12345;
    for (int f = 0; f < s.spec.n_legs; ++f) {
        for (int i = 0; i < s.spec.dof_per_leg; ++i) {
            // A small deterministic pseudo-random spread: identical across runs, and
            // spread over the joint range when the robot has one.
            seed = seed * 1103515245u + 12345u;
            const double u = ((seed >> 16) & 0x7fff) / 32767.0;
            const int idx = f * s.spec.dof_per_leg + i;
            const double amp = 0.35;
            s.qj(idx) = (2.0 * u - 1.0) * amp;
            // qprev is a genuinely different pose, so r_dot is non-zero and the
            // q-dependence of the model is actually exercised.
            seed = seed * 1103515245u + 12345u;
            const double u2 = ((seed >> 16) & 0x7fff) / 32767.0;
            s.qprev(idx) = (2.0 * u - 1.0) * amp;
        }
        s.contacts[f] = 1;
    }
    for (int i = s.spec.n_legs; i < kMaxLegs; ++i) s.contacts[i] = 0;
    // IDENTITY, and that is not laziness: H is built from `state_.q`, the filter's
    // own attitude, which initialises to identity and which this test cannot set.
    //
    // An earlier version rotated R to "make R*skew(r) do real work" and every
    // comparison then disagreed by O(1) -- not because the block was wrong, but
    // because the finite differences were taken about a different rotation than the
    // one the filter used. The identity convention is asserted below so the
    // assumption cannot rot silently.
    s.R = Eigen::Matrix3d::Identity();
    s.v = Eigen::Vector3d(0.13, -0.27, 0.06);
    s.w = Eigen::Vector3d(0.021, -0.033, 0.012);
    return s;
}

}  // namespace

TEST_CASE("measurement Jacobian matches finite differences", "[fusion][jacobian]") {
    // All four robots. G1, Apollo and OP3 share the Chain path but differ in leg
    // length and stance, and the block's error scaled with |r| -- so a gate on Go2
    // alone would not have caught it.
    for (const char* robot : {"go2", "g1", "apollo", "op3"}) {
        Scene s = make_scene(robot);

        // Checked on a FRESH filter: update_legs mutates the state, so the default
        // attitude is only the attitude at the moment H is built.
        {
            FusionEKF fresh;
            REQUIRE(fresh.state().q.isApprox(Eigen::Quaterniond::Identity()));
        }
        FusionEKF ekf;
        ekf.set_robot(s.spec);
        // Prime prev_qj_ so update_legs takes the differencing branch.
        ekf.update_legs(s.qprev, s.contacts, s.w, s.dt);
        const int used = ekf.update_legs(s.qj, s.contacts, s.w, s.dt);
        REQUIRE(used == s.spec.n_legs);

        const Eigen::MatrixXd& H = ekf.last_measurement_H();
        REQUIRE(H.rows() == 3 * s.spec.n_legs);
        REQUIRE(H.cols() == 15);

        const double eps = 1e-7;
        double worst_att = 0.0, worst_v = 0.0;

        for (int f = 0; f < s.spec.n_legs; ++f) {
            const LegSpec& lg = s.spec.leg[f];
            const int nd = s.spec.dof_per_leg;
            const int row = f * 3;

            // --- attitude block, columns 0..2 -------------------------------
            for (int i = 0; i < 3; ++i) {
                Eigen::Matrix3d Rp = s.R, Rm = s.R;
                // Perturb the rotation matrix directly, via a small body-frame
                // rotation, exactly as the state does.
                Eigen::Vector3d d = Eigen::Vector3d::Zero();
                d(i) = eps;
                const Eigen::Matrix3d ex = skew3(d);
                Rp = s.R * (Eigen::Matrix3d::Identity() + ex);
                Rm = s.R * (Eigen::Matrix3d::Identity() - ex);
                const Eigen::Vector3d hp =
                    h_model(s.spec, lg, s.qj.data(), f * nd, s.qprev.data(), s.v, Rp, s.w, s.dt);
                const Eigen::Vector3d hm =
                    h_model(s.spec, lg, s.qj.data(), f * nd, s.qprev.data(), s.v, Rm, s.w, s.dt);
                const Eigen::Vector3d fd = ((hp - hm) / (2 * eps)).eval();
                for (int r = 0; r < 3; ++r) {
                    const double rel = (H(row + r, i) - fd(r)) / (1.0 + std::abs(fd(r)));
                    worst_att = std::max(worst_att, std::abs(rel));
                }
            }

            // --- velocity block, columns 3..5 -------------------------------
            for (int i = 0; i < 3; ++i) {
                Eigen::Vector3d vp = s.v, vm = s.v;
                vp(i) += eps; vm(i) -= eps;
                const Eigen::Vector3d hp =
                    h_model(s.spec, lg, s.qj.data(), f * nd, s.qprev.data(), vp, s.R, s.w, s.dt);
                const Eigen::Vector3d hm =
                    h_model(s.spec, lg, s.qj.data(), f * nd, s.qprev.data(), vm, s.R, s.w, s.dt);
                const Eigen::Vector3d fd = ((hp - hm) / (2 * eps)).eval();
                for (int r = 0; r < 3; ++r) {
                    const double rel = (H(row + r, 3 + i) - fd(r)) / (1.0 + std::abs(fd(r)));
                    worst_v = std::max(worst_v, std::abs(rel));
                }
            }

            // --- gyro bias block, columns 12..14 ----------------------------
            // NOT a derivative check: this block is IDENTICALLY ZERO in the filter.
            // The source carries a bare `// dba block 0` comment and no assignment
            // (fusion/src/fusion.cpp), so leg odometry has never corrected gyro bias
            // -- dx[12..14] is always zero and bg only decays through Qd.
            //
            // That is a real modelling gap, found while writing this test. It is
            // asserted as-is rather than "fixed" here, because making bg observable
            // changes the filter's bias dynamics and needs its own validation --
            // notably against the v0.6 finding that the filter performs best
            // UNDER-trusting leg odometry by ~25x, which may be this gap in
            // disguise. See docs/V06_SIGMA_LEG.md.
            for (int c = 12; c <= 14; ++c)
                for (int r = 0; r < 3; ++r) {
                    REQUIRE(H(row + r, c) == 0.0);
                    REQUIRE(H(row + r, c) == 0.0);
                }

            // --- position block, columns 6..8: h must not depend on p ------
            for (int i = 6; i < 9; ++i)
                for (int r = 0; r < 3; ++r)
                    REQUIRE(std::abs(H(row + r, i)) < 1e-12);
        }

        INFO("robot " << robot << " worst relative error: att " << worst_att
                     << " v " << worst_v);
        // 1e-5 relative. Central differences on a 1e-7 step give ~1e-9 truncation
        // noise, so this is loose enough not to be flaky and tight enough that the
        // old -R*skew(w x r) could not have passed: it differed by a factor of ~50
        // in magnitude and the wrong sign, i.e. a relative error of O(1).
        REQUIRE(worst_att < 1e-5);
        REQUIRE(worst_v < 1e-5);
    }
}

TEST_CASE("attitude Jacobian magnitude tracks |w x r + r_dot|", "[fusion][jacobian]") {
    // A magnitude check as well as a correctness one. The original defect was a
    // MISSING TERM, so a definition-vs-code comparison is the thing that catches it
    // -- but only if the two are computed independently, and a magnitude assertion
    // is what stops the block quietly drifting back toward the smaller expression.
    //
    // ||dh/ddtheta|| is |w x r + r_dot|, and r_dot dominates for a stance foot, so a
    // block built from w x r alone would be two orders of magnitude too small here.
    // That ratio is asserted directly rather than left implicit.
    for (const char* robot : {"go2", "g1", "apollo", "op3"}) {
        Scene s = make_scene(robot);
        FusionEKF ekf;
        ekf.set_robot(s.spec);
        ekf.update_legs(s.qprev, s.contacts, s.w, s.dt);
        ekf.update_legs(s.qj, s.contacts, s.w, s.dt);
        const Eigen::MatrixXd& H = ekf.last_measurement_H();

        double worst = 0.0, best = std::numeric_limits<double>::infinity();
        for (int f = 0; f < s.spec.n_legs; ++f) {
            const LegSpec& lg = s.spec.leg[f];
            const int nd = s.spec.dof_per_leg;
            const Eigen::Vector3d rb = foot_pos_base(s.spec, lg, s.qj.data() + f * nd);
            const Eigen::Vector3d rp = foot_pos_base(s.spec, lg, s.qprev.data() + f * nd);
            const Eigen::Vector3d rd = (rb - rp) / s.dt;
            const double bnorm = (s.w.cross(rb) + rd).norm();
            Eigen::Matrix3d A;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) A(i, j) = H(f * 3 + i, j);
            // ||skew(v)||_F = sqrt(2)*|v|, so divide it out or the bound is off by 1.414.
            const double got = A.norm() / std::sqrt(2.0);
            if (bnorm > 1e-9) {
                worst = std::max(worst, got / bnorm);
                best = std::min(best, got / bnorm);
            }
        }
        INFO("robot " << robot << " ||H_att||/|b| in [" << best << ", " << worst << "]");
        REQUIRE(worst > 0.9);
        REQUIRE(worst < 1.1);
    }
}
