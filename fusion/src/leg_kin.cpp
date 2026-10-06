#include "otolith/leg_kin.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

namespace otolith {

// GEOMETRY PROVENANCE (audited 2026-10-05 while scoping the v0.6 CAD phase).
//
// An earlier version of this comment flagged L2 = 0.21300938946440834 as an
// unexplained 9.4 um discrepancy against the MJCF's 0.213, and speculated it
// was an IK fit. THAT WAS WRONG, and the resolution matters for CAD:
//
//   L1 = |calf joint frame in thigh|              = 0.213
//   L2 = |foot contact sphere centre in calf|      = 0.21300938946440834
//
// They are different quantities. The MJCF's collision default adds a
// pos="-0.002 0 -0.213" offset, so L2 = hypot(0.002, 0.213) = the distance to
// the CONTACT SPHERE CENTRE, which is 9.4 um further out than the calf->ankle
// frame. The 17 significant digits are just what hypot() does to a rounded
// decimal; nothing was fitted. Verified: MuJoCo computes bit-for-bit the same
// double from `norm(geom_pos[foot_geom])`, and the C++ and Rust twins both
// match it exactly (relative difference 0.0).
//
// So this is correct as-is, and the correct thing to do is NOTHING here.
// What it does mean for v0.6: L2 is a distance to a collision *proxy sphere*,
// not a vendor link dimension. Leg odometry wants the contact point, which is
// what this is, so it is the right constant for the estimator -- but CAD must
// take link lengths from the visual meshes or a vendor drawing, not from here,
// L2 is the leg's second segment, taken from the KINEMATIC chain.
//
// It used to be 0.21300938946440834 = hypot(0.002, 0.213), the distance to the
// foot contact SPHERE centre, whose position the MJCF collision default offsets
// by -0.002 m in x. leg_kin derived it from norm(geom_pos[foot_geom]) -- read
// live off a collision proxy -- which meant moving the foot collision geom
// moved the estimator's leg length 1:1 (measured to 1e-9 over +-10 mm). The C++
// and Rust twins then hardcoded that value, so editing contact geometry would
// have silently changed every estimator result in the repo.
//
// For this robot the kinematic chain's second segment equals the first, so L2
// now equals L1. That is a property of the Menagerie frame layout, not a
// copy-paste slip, and it happens to remove the 9.4 um contact-sphere offset.
// See docs/V06_FOOT_PAD.md.
LegGeom leg_geom(const char* name) {
    std::string n(name);
    LegGeom lg{};
    lg.a_offset = 0.0955;
    lg.x_offset = 0.0;
    lg.L1 = 0.213;
    lg.L2 = 0.213;
    if (n == "FL") { lg.hip_base = Eigen::Vector3d(0.1934, 0.0465, 0.0); lg.side = +1; }
    else if (n == "FR") { lg.hip_base = Eigen::Vector3d(0.1934, -0.0465, 0.0); lg.side = -1; }
    else if (n == "RL") { lg.hip_base = Eigen::Vector3d(-0.1934, 0.0465, 0.0); lg.side = +1; }
    else if (n == "RR") { lg.hip_base = Eigen::Vector3d(-0.1934, -0.0465, 0.0); lg.side = -1; }
    else throw std::runtime_error("unknown leg " + n);
    return lg;
}

Eigen::Vector3d foot_pos_base(const LegGeom& leg,
                              double hip, double thigh, double calf) {
    // Planar 2R: x_in = -L1 sin(thigh) - L2 sin(thigh+calf)
    //           z_in = -L1 cos(thigh) - L2 cos(thigh+calf)
    // y_in = side*a_offset (constant leg-plane offset)
    // Base-frame dy,dz = R_x(th1)^T * [y_in, z_in] as derived from leg_ik inversion.
    double x_in = -leg.L1 * std::sin(thigh) - leg.L2 * std::sin(thigh + calf);
    double z_in = -leg.L1 * std::cos(thigh) - leg.L2 * std::cos(thigh + calf);
    double y_in = leg.side * leg.a_offset;
    double c = std::cos(hip), s = std::sin(hip);
    double dy = c * y_in - s * z_in;
    double dz = s * y_in + c * z_in;
    Eigen::Vector3d d(x_in, dy, dz);
    return leg.hip_base + d;
}

Eigen::Vector3d foot_pos_base(const LegGeom& leg,
                              const Eigen::Vector3d& q) {
    return foot_pos_base(leg, q[0], q[1], q[2]);
}

} // namespace otolith
