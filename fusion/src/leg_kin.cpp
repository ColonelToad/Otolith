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


// ---------------------------------------------------------------------------
// Apollo (Apptronik). Generated by mech/spec/build_apollo_spec.py -- regenerate
// rather than hand-edit; a mis-transcribed axis produces plausible numbers
// rather than an error.
//
// BOTH LEGS ARE EMITTED LITERALLY, and that is the point. G1's spec negates y on
// the lateral offsets because only body_pos.y mirrors there. Apollo does not work
// that way: every link's quaternion REFLECTS about y, (w,x,y,z) -> (w,-x,y,-z),
// uniformly across all six. Deriving the right leg from the left with G1's rule
// would leave every link past the hip 180 degrees out, and with an identity-quat
// rule would leave the reflected links 0 degrees out. Neither failure raises an
// error; both just quietly produce the wrong FK. So both legs are read out of the
// model, and mech/tests/test_apollo_leg.py gates the relationship either way.
// ---------------------------------------------------------------------------
static RobotSpec make_apollo() {
    RobotSpec s;
    s.name = "apollo";
    s.model = LegModel::Chain;
    s.n_legs = 2;
    s.dof_per_leg = 6;

    struct J { double ax, ay, az, ox, oy, oz, qw, qx, qy, qz; const char* n; };
    static const J left[6] = {
            { 0, 0, 1,  -0.02, 0.11, -0.16875,  0.957662211, 0.126078028, -0.256605057, 0.0337826075,  "l_hip_ie" },
            { 1, 0, 0,  0, 0, 0,  1, 0, 0, 0,  "l_hip_aa" },
            { 0, 1, 0,  0, 0, 0,  0.957662084, -0.126079011, 0.256605023, -0.033782803,  "l_hip_fe" },
            { 0, 1, 0,  -0.05, 0, -0.425,  1, 0, 0, 0,  "l_knee_fe" },
            { 1, 0, 0,  0.05, 0, -0.425,  0.987672169, 0.0864102147, -0.130029022, 0.0113761019,  "l_ankle_ie" },
            { 0, 1, 0,  0, 0, 0,  0.965925849, 0, 0.25881896, 0,  "l_ankle_pd" }
    };
    static const J right[6] = {
            { 0, 0, 1,  -0.02, -0.11, -0.16875,  0.957662211, -0.126078028, -0.256605057, -0.0337826075,  "r_hip_ie" },
            { 1, 0, 0,  0, 0, 0,  1, 0, 0, 0,  "r_hip_aa" },
            { 0, 1, 0,  0, 0, 0,  0.957662084, 0.126079011, 0.256605023, 0.033782803,  "r_hip_fe" },
            { 0, 1, 0,  -0.05, 0, -0.425,  1, 0, 0, 0,  "r_knee_fe" },
            { 1, 0, 0,  0.05, 0, -0.425,  0.987672169, -0.0864102147, -0.130029022, -0.0113761019,  "r_ankle_ie" },
            { 0, 1, 0,  0, 0, 0,  0.965925849, 0, 0.25881896, 0,  "r_ankle_pd" }
    };

    for (int leg = 0; leg < 2; ++leg) {
        LegSpec& L = s.leg[leg];
        L.dof = 6;
        L.side = leg == 0 ? +1 : -1;
        // Sole = BOTTOM-FACE CENTRE of the 200 x 85 x 18 mm sole box, i.e. the box
        // centre 9 mm lower. Using the box centre would bias every foot position
        // in the log by 9 mm, 3% of sigma_leg, in a plausible-looking direction.
        // 9 significant digits, from build_apollo_spec.py. Typed to 5 decimals this
        // cost 5.6 um of FK error on its own: the sole offset is applied at the very
        // end of the chain, so its rounding lands straight on the output position
        // with nothing to average it against.
        L.sole = leg == 0 ? Eigen::Vector3d(0.0646931, -0.00550529, -0.04743)
                          : Eigen::Vector3d(0.0646931, 0.00550529, -0.04743);

        const J* J6 = (leg == 0) ? left : right;
        for (int i = 0; i < 6; ++i) {
            L.axis[i]   = Eigen::Vector3d(J6[i].ax, J6[i].ay, J6[i].az);
            L.origin[i] = Eigen::Vector3d(J6[i].ox, J6[i].oy, J6[i].oz);
            // LegSpec::quat is wxyz and quat_to_mat reads it positionally as
            // wxyz[0..3]; Eigen's Vector4d four-scalar constructor is (x,y,z,w).
            // Writing (qw,qx,qy,qz) is what lands w at index 0. The conventions
            // cancel, and G1's chain FK is 27 pm because of it -- but using the
            // NAMED accessors (.w(), .x(), ...) anywhere on this field reads the
            // scrambled layout instead. Pinned by test_fusion.cpp.
            L.quat[i]   = Eigen::Vector4d(J6[i].qw, J6[i].qx, J6[i].qy, J6[i].qz);
        }
        // The chain is rooted at base_link, which IS the base frame for a
        // floating-base humanoid. The +/-0.11 m hip offset lives in origin[0].y,
        // the same place G1 keeps its 0.0645.
        L.hip_base.setZero();
    }
    return s;
}


// ---------------------------------------------------------------------------
// OP3 (Robotis). Generated by mech/spec/build_biped_spec.py op3 -- regenerate
// rather than hand-edit.
//
// THIRD MIRROR RULE, AND THE THIRD AXIS RULE TOO. G1 and OP3 share one rule
// (quaternions equal across legs, only body_pos.y mirrors); Apollo has the other
// (quaternions reflected about y). And OP3 additionally NEGATES THE JOINT AXES on
// the right leg -- hip_pitch goes 0,1,0 -> 0,-1,0, knee likewise, ank_pitch the
// other way -- which neither of the other two models does.
//
// So both legs are emitted literally again. Two of the three possible rules would
// be wrong for this robot, and applying either silently misplaces every link past
// the hip without raising an error.
//
// OP3 HAS NO JOINT LIMITS (every jnt_range is [0,0]) and no keyframes, so T4 is
// vacuous for it and the puppet has no vendor stance to seed from. See
// mech/tests/test_op3_leg.py, which records that as a coverage gap rather than a
// clean bill of health.
// ---------------------------------------------------------------------------
static RobotSpec make_op3() {
    RobotSpec s;
    s.name = "op3";
    s.model = LegModel::Chain;
    s.n_legs = 2;
    s.dof_per_leg = 6;

    struct J { double ax, ay, az, ox, oy, oz, qw, qx, qy, qz; const char* n; };
    static const J left[6] = {
            { 0, 0, -1,  0, 0.035, 0,  1, 0, 0, 0,  "l_hip_yaw" },
            { -1, 0, 0,  -0.024, 0, -0.0285,  1, 0, 0, 0,  "l_hip_roll" },
            { 0, 1, 0,  0.0241, 0.019, 0,  1, 0, 0, 0,  "l_hip_pitch" },
            { 0, 1, 0,  0, 0, -0.11015,  1, 0, 0, 0,  "l_knee" },
            { 0, -1, 0,  0, 0, -0.11,  1, 0, 0, 0,  "l_ank_pitch" },
            { 1, 0, 0,  -0.0241, -0.019, 0,  1, 0, 0, 0,  "l_ank_roll" }
    };
    static const J right[6] = {
            { 0, 0, -1,  0, -0.035, 0,  1, 0, 0, 0,  "r_hip_yaw" },
            { -1, 0, 0,  -0.024, 0, -0.0285,  1, 0, 0, 0,  "r_hip_roll" },
            { 0, -1, 0,  0.0241, -0.019, 0,  1, 0, 0, 0,  "r_hip_pitch" },
            { 0, -1, 0,  0, 0, -0.11015,  1, 0, 0, 0,  "r_knee" },
            { 0, 1, 0,  0, 0, -0.11,  1, 0, 0, 0,  "r_ank_pitch" },
            { 1, 0, 0,  -0.0241, 0.019, 0,  1, 0, 0, 0,  "r_ank_roll" }
    };

    for (int leg = 0; leg < 2; ++leg) {
        LegSpec& L = s.leg[leg];
        L.dof = 6;
        L.side = leg == 0 ? +1 : -1;
        // Sole = mean of the two foot boxes' BOTTOM-FACE centres. The half-thickness
        // subtraction is 4 mm here, not 9 mm as on Apollo, and skipping it is just
        // as silent. Generated, because the sole is applied at the very END of the
        // chain and its rounding lands straight on the output position.
        // left: 0.024, 0.01275, -0.0305
        // right: 0.024, -0.01275, -0.0305
        L.sole = leg == 0 ? Eigen::Vector3d(0.024, 0.01275, -0.0305)
                          : Eigen::Vector3d(0.024, -0.01275, -0.0305);

        const J* J6 = (leg == 0) ? left : right;
        for (int i = 0; i < 6; ++i) {
            L.axis[i]   = Eigen::Vector3d(J6[i].ax, J6[i].ay, J6[i].az);
            // Same (x,y,z,w)-constructor / wxyz-positional-read cancellation as
            // make_apollo and make_g1; do not "tidy" this into named accessors.
            L.origin[i] = Eigen::Vector3d(J6[i].ox, J6[i].oy, J6[i].oz);
            L.quat[i]   = Eigen::Vector4d(J6[i].qw, J6[i].qx, J6[i].qy, J6[i].qz);
        }
        // The chain is rooted at body_link, which IS the base frame for a
        // floating-base humanoid. The +/-0.035 m hip offset lives in origin[0].y.
        L.hip_base.setZero();
    }
    return s;
}

RobotSpec robot_spec(const char* name) {
    const std::string n = name;
    if (n == "go2")    return make_go2();
    if (n == "g1")     return make_g1();
    if (n == "apollo") return make_apollo();
    if (n == "op3")    return make_op3();
    throw std::runtime_error("unknown robot " + n);
}

const char* const* leg_names(const RobotSpec& spec) {
    static const char* go2_names[4]    = {"FL", "FR", "RL", "RR"};
    static const char* biped_names[2] = {"left", "right"};
    // Both bipeds use left/right, and both have 2 legs. Asking the robot rather
    // than testing for "g1" means the next biped does not need editing here --
    // which is the whole reason leg_names() exists.
    return spec.n_legs == 2 ? biped_names : go2_names;
}

const LegSpec& leg_by_name(const RobotSpec& spec, const char* name) {
    const std::string n = name;
    const char* const* names = leg_names(spec);
    for (int i = 0; i < spec.n_legs; ++i)
        if (n == names[i]) return spec.leg[i];
    throw std::runtime_error("unknown leg " + n + " for " + spec.name);
}

} // namespace otolith