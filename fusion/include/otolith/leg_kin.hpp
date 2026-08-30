#pragma once
// Forward kinematics for Unitree Go2 legs.
// Mirrors the analytic IK in sim/otolith_sim/puppet.py (leg_ik) and is
// validated against MuJoCo FK to <2mm mean error (see test).

#include <Eigen/Dense>

namespace otolith {

struct LegGeom {
    Eigen::Vector3d hip_base; // hip joint position in base frame
    int side;                 // +1 left, -1 right
    double a_offset;          // |thigh_y|
    double x_offset;          // thigh_x (0 for Go2)
    double L1, L2;            // thigh->calf, calf->foot
};

// Go2 constants introspected from menagerie/unitree_go2/scene.xml
// hip_base [±0.1934, ±0.0465, 0], a_offset 0.0955, L1/L2 0.213
LegGeom leg_geom(const char* name);          // "FL","FR","RL","RR"
Eigen::Vector3d foot_pos_base(const LegGeom& leg,
                              double hip, double thigh, double calf);
Eigen::Vector3d foot_pos_base(const LegGeom& leg,
                              const Eigen::Vector3d& q); // [hip,thigh,calf]

} // namespace otolith
