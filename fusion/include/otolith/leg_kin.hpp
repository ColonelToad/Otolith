#pragma once
// Leg forward kinematics for the filter, generalised past the Go2 planar 2R.
//
// WHY TWO MODELS BEHIND ONE INTERFACE
// =====================================
// Go2's leg is genuinely planar, and its planar 2R form is validated: EXACTLY
// 2.000 mm of position error at every pose, against MuJoCo, over the full joint
// range. That number is not noise and not accumulated error -- it is the foot
// sphere's 2 mm lateral offset in the calf frame (pos="-0.002 0 -0.213"),
// which the planar form does not represent because it places the foot directly
// below the calf joint. Constant magnitude, direction rotating with the leg, so
// it cancels in the finite difference the filter forms: r_dot error is 0.0018
// m/s, 0.01x the assumed sigma_leg (measured separately, docs/V05_HUMANOID.md).
//
// Replacing it with the general chain would cost the bit-parity between the float and
// fixed-point paths, which `test_fixed.cpp` asserts over the trot joint angles.
// So the planar form stays for robots that want it and the chain form serves
// robots that cannot use it.
//
// A biped cannot use it. G1's CoM has to shift 0.1165 m over the stance foot,
// which over its 0.719 m leg is a 9.2 deg leg tilt; a planar FK misplaces the
// foot by 115 mm, 0.4x the entire sigma_leg budget. The chain form carries the
// per-joint axis, origin AND the fixed body_quat -- the last one matters, since
// G1's hip_roll_link is tilted 10 deg and its knee_link -10 deg, and omitting
// those rotations stops the chain offsets telescoping (53 mm at q=0).
//
// So `foot_pos_base` is the seam: robot-specific data behind it, robot-agnostic
// filter above it.

#include <Eigen/Dense>
#include <cstdint>
#include <cmath>

namespace otolith {

constexpr int kMaxLegs = 4;
constexpr int kMaxDofPerLeg = 8;

enum class LegModel : std::uint8_t {
    Planar2R,   // Go2: pitch chain of two links in a laterally offset plane
    Chain,      // general serial chain, any DoF
};

// Per-leg descriptor. Every field a robot's leg FK needs, and nothing else.
struct LegSpec {
    Eigen::Vector3d hip_base{0, 0, 0};   // chain root / hip joint, in base frame
    int side = +1;                        // +1 left, -1 right (planar model)
    double a_offset = 0.0;                // |thigh_y|, planar leg-plane offset
    double x_offset = 0.0;                // thigh_x
    double L1 = 0.0, L2 = 0.0;            // planar link lengths
    int dof = 0;
    Eigen::Vector3d axis[kMaxDofPerLeg]{};
    Eigen::Vector3d origin[kMaxDofPerLeg]{};   // joint origin in parent frame
    Eigen::Vector4d quat[kMaxDofPerLeg]{};     // FIXED child frame rotation, wxyz
    Eigen::Vector3d sole{0, 0, 0};             // sole point in foot body frame
};

// sigma_leg is the leg-odometry velocity noise: sigma_q*sqrt(2)*k*L/dt.
//
// It is NOT a constant of the filter, and treating it as one is what made the Go2
// value (0.3) look validated when it was a coincidence -- see P4 in
// docs/V05_HUMANOID.md. Three separate inputs, and all three are per-robot:
//
//   sigma_q  encoder angle quantisation, rad. Real hardware, not the simulator's
//            default: XM430 is 4096 counts/rev = 0.001534 rad/tick, so
//            sigma_q = tick/sqrt(12) = 0.000443.
//   L        nominal lever arm, hip to sole. NOT derivable: the effective lever is
//            the accumulated response of every joint plus the sole offset, so it is
//            measured per robot and lands at 0.72 (go2), 0.94 (g1), 1.09 (apollo),
//            0.86 (op3) of nominal.
//   dt       the sample period. The 1/dt is the whole point: halving dt doubles
//            sigma_leg, so any constant is right at exactly one rate.
//
// The result is quadrature-added with the robot's own body-motion term (the base
// swaying while a foot is planted), which is ~0.14-0.18 m/s across all four robots
// and is NOT part of this formula.
struct SigmaLegModel {
    double sigma_q = 0.0;   // encoder angle quantisation, rad
    double lever_nominal = 0.0;  // hip to sole, m
    double k = 1.0;         // measured effective-lever ratio
    double body_motion = 0.0;    // measured real-motion term, m/s

    // sigma_leg at a given sample period, in quadrature with the motion term.
    double at(double dt) const {
        const double quant = sigma_q * std::sqrt(2.0) * k * lever_nominal / dt;
        return std::sqrt(quant * quant + body_motion * body_motion);
    }
};

struct RobotSpec {
    const char* name = "";
    LegModel model = LegModel::Planar2R;
    int n_legs = 0;
    int dof_per_leg = 0;
    LegSpec leg[kMaxLegs];
    SigmaLegModel sigma_leg{};   // populated by robot_spec; see the struct's docs
};

// Named lookups. leg_names() must match the order of spec->leg[].
RobotSpec robot_spec(const char* name);   // "go2" | "g1"
const LegSpec& leg_by_name(const RobotSpec& spec, const char* name);
// Leg names in spec->leg[] order: 4 for Go2 ("FL","FR","RL","RR"), and
// ("left","right") for both bipeds, G1 and Apollo. A biped's names are shorter and
// singular, so anything that printed leg labels has to ask rather than assume --
// sigma_study did assume and had to be fixed. Keyed on n_legs so the next biped
// needs no edit here.
const char* const* leg_names(const RobotSpec& spec);

// Foot (sole) position in the BASE frame, for a leg's joint angles.
// `q` must hold at least spec.dof_per_leg doubles.
Eigen::Vector3d foot_pos_base(const RobotSpec& spec, int leg, const double* q);
Eigen::Vector3d foot_pos_base(const RobotSpec& spec, const LegSpec& leg, const double* q);

// Planar 2R in isolation, for reference and for the fixed-point twin's test.
// Exposed so `test_fixed.cpp` can keep checking the fixed-point FK against the
// same numbers it always has.
Eigen::Vector3d planar2r_pos_base(const LegSpec& leg, double hip, double thigh, double calf);

} // namespace otolith