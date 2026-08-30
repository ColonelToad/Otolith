#include "otolith/leg_kin.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

namespace otolith {

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
