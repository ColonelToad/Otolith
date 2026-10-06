#include "otolith/leg_kin.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

namespace otolith {

namespace {

// Rodrigues rotation about a unit axis. The axis comes from the model and is
// already normalised by MuJoCo's jnt_axis, but normalising again costs nothing
// and removes an assumption that would otherwise be silent.
Eigen::Matrix3d axis_angle(const Eigen::Vector3d& axis_in, double theta) {
    Eigen::Vector3d a = axis_in;
    const double n = a.norm();
    if (n < 1e-12) throw std::runtime_error("leg joint axis is degenerate");
    a /= n;
    Eigen::Matrix3d K;
    K <<     0.0, -a.z(),  a.y(),
          a.z(),     0.0, -a.x(),
         -a.y(),  a.x(),     0.0;
    return Eigen::Matrix3d::Identity() + std::sin(theta) * K
         + (1.0 - std::cos(theta)) * (K * K);
}

Eigen::Matrix3d quat_to_mat(const Eigen::Vector4d& wxyz) {
    const double w = wxyz[0], x = wxyz[1], y = wxyz[2], z = wxyz[3];
    Eigen::Matrix3d R;
    R << 1 - 2*(y*y + z*z), 2*(x*y - w*z),   2*(x*z + w*y),
         2*(x*y + w*z),   1 - 2*(x*x + z*z), 2*(y*z - w*x),
         2*(x*z - w*y),   2*(y*z + w*x),   1 - 2*(x*x + y*y);
    return R;
}

} // namespace

Eigen::Vector3d planar2r_pos_base(const LegSpec& leg, double hip, double thigh, double calf) {
    // x_in = -L1 sin(thigh) - L2 sin(thigh+calf)
    // z_in = -L1 cos(thigh) - L2 cos(thigh+calf)
    // y_in = side*a_offset (constant leg-plane offset)
    // Base-frame dy,dz = R_x(hip)^T * [y_in, z_in], matching the C++ that
    // preceded this file. The values are untouched -- see leg_kin.hpp on why.
    const double x_in = -leg.L1 * std::sin(thigh) - leg.L2 * std::sin(thigh + calf);
    const double z_in = -leg.L1 * std::cos(thigh) - leg.L2 * std::cos(thigh + calf);
    const double y_in = leg.side * leg.a_offset;
    const double c = std::cos(hip), s = std::sin(hip);
    const double dy = c * y_in - s * z_in;
    const double dz = s * y_in + c * z_in;
    return leg.hip_base + Eigen::Vector3d(x_in, dy, dz);
}

Eigen::Vector3d foot_pos_base(const RobotSpec& spec, const LegSpec& leg, const double* q) {
    if (spec.model == LegModel::Planar2R) {
        if (leg.dof != 3)
            throw std::runtime_error("planar2r model needs 3 joints per leg");
        return planar2r_pos_base(leg, q[0], q[1], q[2]);
    }

    // General serial chain, rooted at the base frame (identity pose).
    // The joint origin moves the point; the FIXED body_quat then rotates every
    // LATER offset; the joint rotation is about the joint origin and so does not
    // move `p` itself.
    Eigen::Vector3d p = leg.hip_base;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    for (int i = 0; i < leg.dof; ++i) {
        p += R * leg.origin[i];
        R = R * quat_to_mat(leg.quat[i]);
        R = R * axis_angle(leg.axis[i], q[i]);
    }
    return p + R * leg.sole;
}

Eigen::Vector3d foot_pos_base(const RobotSpec& spec, int leg, const double* q) {
    if (leg < 0 || leg >= spec.n_legs)
        throw std::runtime_error("leg index out of range for " + std::string(spec.name));
    return foot_pos_base(spec, spec.leg[leg], q);
}

// ---------------------------------------------------------------------------
// Go2. Unchanged values, deliberately: this is the model the fixed-point twin
// and every recorded result were built against.
// ---------------------------------------------------------------------------
static RobotSpec make_go2() {
    RobotSpec s;
    s.name = "go2";
    s.model = LegModel::Planar2R;
    s.n_legs = 4;
    s.dof_per_leg = 3;
    const char* names[4] = {"FL", "FR", "RL", "RR"};
    const double hx[4] = { 0.1934,  0.1934, -0.1934, -0.1934 };
    const double hy[4] = {  0.0465, -0.0465,  0.0465, -0.0465 };
    const int side[4] = { +1, -1, +1, -1 };
    for (int i = 0; i < 4; ++i) {
        LegSpec& L = s.leg[i];
        L.hip_base = Eigen::Vector3d(hx[i], hy[i], 0.0);
        L.side = side[i];
        L.a_offset = 0.0955;
        L.x_offset = 0.0;
        L.L1 = 0.213;
        L.L2 = 0.213;
        L.dof = 3;
        (void)names;
    }
    return s;
}

// ---------------------------------------------------------------------------
// G1. Generated from .work/g1scene/scene.xml; see mech/spec/build_g1_spec.py.
// Chain order is the tree topology -- pelvis -> hip_pitch -> hip_roll -> ...
// so hip_pitch is FIRST, the opposite of what a quadruped-derived mental model
// predicts. Regenerate rather than hand-edit: a mis-transcribed axis produces
// plausible numbers rather than an error.
// ---------------------------------------------------------------------------
static RobotSpec make_g1() {
    RobotSpec s;
    s.name = "g1";
    s.model = LegModel::Chain;
    s.n_legs = 2;
    s.dof_per_leg = 6;

    for (int leg = 0; leg < 2; ++leg) {
        LegSpec& L = s.leg[leg];
        L.dof = 6;
        const double sg = leg == 0 ? +1.0 : -1.0;   // left, right
        L.side = leg == 0 ? +1 : -1;
        // Sole point = mean of the foot's four r=5 mm contact spheres, which sit
        // coplanar at z=-0.030 forming a flat 170 x 60 mm sole. A property of the
        // foot, not of the current contact set -- see sim/otolith_sim/leg_model.py.
        L.sole = Eigen::Vector3d(0.035, 0.0, -0.03);

        struct J { double ax, ay, az, ox, oy, oz, qw, qx, qy, qz; const char* n; };
        // Left leg; the right is the same with y negated on the lateral offsets.
        static const J left[6] = {
            { 0, 1, 0,  0.0,       0.064452,  -0.1027,    1, 0, 0, 0,                 "hip_pitch" },
            { 1, 0, 0,  0.0,       0.052,     -0.030465,  0.996178686, 0, -0.0873385724, 0, "hip_roll" },
            { 0, 0, 1,  0.025001,  0.0,       -0.12412,   1, 0, 0, 0,                 "hip_yaw" },
            { 0, 1, 0, -0.078273,  0.0021489, -0.17734,   0.996178686, 0,  0.0873385724, 0, "knee" },
            { 0, 1, 0,  0.0,      -9.4445e-05, -0.30001,   1, 0, 0, 0,                 "ankle_pitch" },
            { 1, 0, 0,  0.0,       0.0,       -0.017558,  1, 0, 0, 0,                 "ankle_roll" },
        };
        for (int i = 0; i < 6; ++i) {
            const J& j = left[i];
            L.axis[i]   = Eigen::Vector3d(j.ax, j.ay, j.az);
            // Only body_pos.y mirrors between legs. The body_quat does NOT:
            // G1's 10 deg frame tilts are rotations about y, which are invariant
            // under the sagittal reflection that swaps left for right. Mirroring
            // the quaternion's y component too cost 331 mm of FK error and looked
            // like a data-entry mistake in the literals rather than a wrong
            // mirroring rule.
            L.origin[i] = Eigen::Vector3d(j.ox, sg * j.oy, j.oz);
            L.quat[i]   = Eigen::Vector4d(j.qw, j.qx, j.qy, j.qz);
            (void)j.n;
        }
        // The chain is rooted at the pelvis, which IS the base frame for a
        // floating-base humanoid, so hip_base is the origin.
        L.hip_base.setZero();
    }
    return s;
}

RobotSpec robot_spec(const char* name) {
    const std::string n = name;
    if (n == "go2") return make_go2();
    if (n == "g1")  return make_g1();
    throw std::runtime_error("unknown robot " + n);
}

const LegSpec& leg_by_name(const RobotSpec& spec, const char* name) {
    const std::string n = name;
    static const char* go2_names[4] = {"FL", "FR", "RL", "RR"};
    static const char* g1_names[2]  = {"left", "right"};
    const bool is_g1 = std::string(spec.name) == "g1";
    const char* const* names = is_g1 ? g1_names : go2_names;
    for (int i = 0; i < spec.n_legs; ++i)
        if (n == names[i]) return spec.leg[i];
    throw std::runtime_error("unknown leg " + n + " for " + spec.name);
}

} // namespace otolith