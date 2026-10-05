#include "otolith/leg_kin.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

namespace otolith {

// GEOMETRY PROVENANCE (audited 2026-10-05 while scoping the v0.6 CAD phase).
//
// Five of these six constants are verbatim from the MuJoCo Menagerie Go2 MJCF
// (third_party/menagerie/unitree_go2/go2.xml):
//   hip_base (0.1934, 0.0465)  <- pos="0.1934 0.0465 0"
//   L1 = 0.213                  <- pos="0 0 -0.213"   (thigh)
//   a_offset = 0.0955           <- pos="0 0.0955 0"
//   L2 = 0.213                  <- pos="0 0 -0.213"   (calf)
//
// L2 is the exception and the reason this comment exists: the MJCF says
// 0.213, this file says 0.21300938946440834 -- a 9.4 um discrepancy whose
// origin is NOT recorded anywhere in the repo. The 17 significant digits say
// "fitted", not "published", so someone at some point refined it (most likely
// an IK/differential fit against the sim) and did not write down why.
//
// DO NOT build CAD or FEA against L2 until that is resolved. A 9.4 um error is
// irrelevant to a foot-position estimate and unacceptable in a structural
// model, so the two uses need opposite answers and only one of them can be
// right. Either the real Go2 calf is 0.213 exactly and this constant should
// revert, or the refinement is load-bearing for the estimator and belongs in
// the geometry spec with its derivation attached.
//
// The MJCF is a *derived* model (meshes + primitives), not a vendor drawing, so
// "matches the MJCF" is provenance, not ground truth. Anything mechanical
// needs a real dimension source before it can be trusted.
LegGeom leg_geom(const char* name) {
    std::string n(name);
    LegGeom lg{};
    lg.a_offset = 0.0955;
    lg.x_offset = 0.0;
    lg.L1 = 0.213;
    lg.L2 = 0.21300938946440834;
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
