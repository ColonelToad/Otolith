"""Per-robot leg descriptors and forward kinematics, generalised past Go2.

WHY THIS FILE LOOKS THE WAY IT DOES
===================================
`fusion/src/leg_kin.cpp` is the filter's leg model: a hardcoded planar 2R for
Go2, validated against MuJoCo to <2 mm. That is right for a quadruped whose leg
is genuinely planar, and wrong for a biped:

    G1's CoM must shift 0.1165 m over the stance foot. Over a 0.719 m leg that
    is a 9.2 deg tilt, and a planar FK ignoring it misplaces the foot by
    115 mm -- 0.4x the entire sigma_leg budget of 0.3 m/s.

So the descriptor carries a *chain*, not two lengths. The chain data is read
from the model rather than typed out, for the reason the Go2 constants came from
the URDF in the first place: a transcribed axis or origin is wrong in a way that
still produces plausible numbers.

THE SOLE POINT IS THE IDEA WORTH KEEPING
========================================
Go2's foot is one sphere and `r_base` is its centre. G1's foot is FOUR r=5 mm
spheres at the corners of a flat 170 x 60 mm sole -- heel at x=-0.050, toe at
x=+0.120, all four coplanar at z=-0.030. There is no single geom to read a
position from, so the descriptor carries a *sole reference point*: the mean of
the four sphere centres, (0.035, 0, -0.030) in the ankle_roll body frame.

Explicitly NOT the centroid of whichever spheres happen to be touching. That
would move as contact breaks and re-forms, injecting noise into exactly the
quantity the filter is measuring. This is the v0.6 foot-pad lesson applied --
docs/V06_FOOT_PAD.md -- where a sphere foot turned out to be a kinematic proxy
whose placement was quietly load-bearing. The sole point must be a property of
the foot, not of the current contact set.

Verified by: test_g1_fk_matches_mujoco (<2 mm), test_sole_point_is_coplanar,
and the C++ differential in P1.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

# Chain order = the tree topology, NOT alphabetical and not the order that reads
# naturally. pelvis -> hip_pitch_link -> hip_roll_link -> hip_yaw_link -> ...
# which is genuinely hip_pitch first even though hip_roll is the more familiar
# first joint on a quadruped. Getting this wrong is invisible at q=0 for most
# robots and wrong everywhere once the joints move.
G1_LEGS = ("left", "right")
G1_JOINTS = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
G1_ROOT = "pelvis"

# Naming scheme for the patched G1 scene. Go2's foot spheres are already named
# FL/FR/RL/RR; G1 ships with NO named geoms at all, so the scene patch adds
# `<geom name="left_sole0" .../>` etc. See mech/spec/build_g1_scene.py.
SOLE_GEOM_TEMPLATE = "{leg}_sole{i}"

# --- Apollo (Apptronik) -----------------------------------------------------
# Same 6-DoF leg shape as G1, so the seam is unchanged -- but nothing else about
# the two models lines up, which is exactly what makes Apollo worth adding.
#   G1    chain hip_pitch -> hip_roll -> hip_yaw -> knee -> ankle_pitch -> ankle_roll
#   Apollo chain hip_ie -> hip_aa -> hip_fe -> knee_fe -> ankle_ie -> ankle_pd
# G1 leads with a pitch hip; Apollo leads with a yaw (ie) hip and rolls (aa) second.
# G1's chain roots at `pelvis`; Apollo's at `base_link`. G1's feet are four spheres
# each, Apollo's is one 200 x 85 x 18 mm box. And Apollo's bodies are named with
# `l_`/`r_` prefixes while its legs are addressed as left/right, so the side
# prefix has to be mapped rather than interpolated.
APOLLO_LEGS = ("left", "right")
APOLLO_SIDE_PREFIX = {"left": "l", "right": "r"}
APOLLO_JOINTS = ("hip_ie", "hip_aa", "hip_fe", "knee_fe", "ankle_ie", "ankle_pd")
# Joint -> child body suffix. Explicit because the pattern breaks at the end:
# every joint is named after its child body EXCEPT ankle_pd, whose child is
# `l_foot_link`. Interpolating the name works for five of six and then raises
# KeyError on the sixth, which is why this is a table.
APOLLO_JOINT_BODY = {
    "hip_ie": "hip_ie_link", "hip_aa": "hip_aa_link", "hip_fe": "hip_fe_link",
    "knee_fe": "knee_fe_link", "ankle_ie": "ankle_ie_link", "ankle_pd": "foot_link",
}
APOLLO_ROOT = "base_link"
# One sole geom per foot. Apollo needs NO scene patch: its scene.xml already
# declares explicit <pair> entries for both soles against the floor, and an
# explicit pair is checked regardless of contype/conaffinity. The floor itself is
# contype=0, and every one of the 79 geoms is too, which is what made this look
# like there was no contact at all. There is: 4 contacts at the stand pose.
# An earlier draft patched contype=1 onto the soles and broke it -- see
# docs/V05_HUMANOID.md P6.
APOLLO_SOLE_GEOM = {"left": "collision_l_sole", "right": "collision_r_sole"}
APOLLO_FOOT_BODY = {"left": "l_foot_link", "right": "r_foot_link"}

# --- OP3 (Robotis) ----------------------------------------------------------
# The third robot, and the one that shares the LEAST with the other two. G1 and
# Apollo are both 6-DoF humanoids with `left`/`right` legs and l_/r_ bodies; OP3 is
# a 3.15 kg miniature, but the chain is the same shape and the footprint of every
# other difference is larger:
#   G1      hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
#   Apollo  hip_ie (yaw), hip_aa (roll), hip_fe (pitch), knee_fe, ankle_ie, ankle_pd
#   OP3     hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll
# OP3 is closer to G1's ordering than Apollo's (yaw first vs pitch first), and its
# ankle is `ank_` not `ankle_`. Chain root is `body_link`, not `pelvis` or
# `base_link`.
#
# Two properties make OP3 the odd one out in a way that matters:
#   * ZERO limited joints. Every jnt_range is [0,0]. T4 is therefore vacuous, and
#     the null-space clamp that caught G1 winding ankle_roll to 102 rad has
#     nothing to clamp against -- so it will HIDE that class of bug, not fix it.
#   * NO keyframes (nkey 0) and an all-zero default pose whose feet sit 20.9 mm
#     above the floor. The home pose has to be solved outright.
# It also has the narrowest stance of the three: hips at +/-35 mm, so 70 mm of hip
# spacing, against G1's 129 mm and Apollo's 220 mm. The lateral CoM shift that
# breaks the planar 2R model scales with that distance, so OP3 is the most
# aggressive test of the planar assumption of the three.
OP3_LEGS = ("left", "right")
OP3_SIDE_PREFIX = {"left": "l", "right": "r"}
OP3_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll")
# No entry needed for ank_roll: the pattern holds for all six.
OP3_JOINT_BODY = {j: f"{j}_link" for j in OP3_JOINTS}
OP3_ROOT = "body_link"
OP3_FOOT_BODY = {"left": "l_ank_roll_link", "right": "r_ank_roll_link"}
OP3_FOOT_GEOM_TEMPLATE = "{leg}_foot{i}"


@dataclass(frozen=True)
class LegChain:
    """One leg's serial chain, read from the model."""
    name: str
    joints: tuple[str, ...]        # chain order, which is the tree topology
    bodies: tuple[str, ...]        # root body, then one per joint
    axis: np.ndarray               # (DOF,3) joint axis in the child body frame
    origin: np.ndarray             # (DOF,3) child origin in the parent frame
    quat: np.ndarray               # (DOF,4) FIXED child frame rotation, wxyz
    root: str

    @property
    def dof(self) -> int:
        return len(self.joints)


@dataclass(frozen=True)
class LegModel:
    """Everything the filter needs to know about a robot's legs.

    `contact_geoms` maps a leg to the geom names that make up its foot. G1 needs
    four; Go2 needs one. The filter sums over them, which is exactly why contact
    identification is data in the descriptor and not a naming convention baked
    into the filter.
    """
    name: str
    legs: tuple[str, ...]
    dof_per_leg: int
    chains: tuple[LegChain, ...]
    contact_geoms: dict[str, tuple[str, ...]]
    sole: dict[str, np.ndarray]    # leg -> (3,) offset in the foot body frame

    @property
    def n_legs(self) -> int:
        return len(self.legs)

    @property
    def total_dof(self) -> int:
        return self.n_legs * self.dof_per_leg

    def chain(self, leg: str) -> LegChain:
        return self.chains[self.legs.index(leg)]


def _rodrigues(axis: np.ndarray, theta: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        raise ValueError("joint axis is degenerate")
    a = a / n
    K = np.array([[0.0, -a[2], a[1]],
                  [a[2], 0.0, -a[0]],
                  [-a[1], a[0], 0.0]])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def _quat_to_mat(wxyz) -> np.ndarray:
    w, x, y, z = wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def chain_fk(chain: LegChain, q, root_pos, root_quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
    """World position of the last link's origin for joint angles `q`.

    Product of translate(origin) * rotate(axis, theta) down the chain, which is
    exact for hinge joints: a hinge's own frame origin is fixed relative to its
    parent, so only the rotation about the axis is configuration dependent.

    MuJoCo is deliberately not consulted here. The point of this function is to
    be an independent implementation to check MuJoCo against, in the same spirit
    as leg_kin.cpp being validated rather than trusted.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (chain.dof,):
        raise ValueError(f"{chain.name}: expected {chain.dof} joint angles, got {q.shape}")
    R = _quat_to_mat(root_quat_wxyz)
    p = np.asarray(root_pos, dtype=float).copy()
    for i in range(chain.dof):
        p = p + R @ chain.origin[i]
        # The FIXED frame rotation, then the joint rotation. Both act about the
        # child origin, so neither moves `p` -- but the fixed one rotates every
        # LATER offset, and dropping it is not a small error: G1's hip_roll_link
        # is tilted 10 deg and its knee_link -10 deg, which is exactly why the x
        # offsets refuse to telescope (53 mm of error at q=0 if omitted).
        R = R @ _quat_to_mat(chain.quat[i])
        R = R @ _rodrigues(chain.axis[i], float(q[i]))
    return p


def sole_world(chain: LegChain, q, root_pos, root_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
               sole_offset=None) -> np.ndarray:
    """World position of the foot's sole reference point.

    `sole_offset` is required rather than defaulted: an omitted sole point would
    silently return the ankle origin, which is a different physical quantity and
    the exact class of bug this file exists to prevent.
    """
    if sole_offset is None:
        raise ValueError("sole_offset is required; the ankle origin is not the sole")
    R = _quat_to_mat(root_quat_wxyz)
    p = np.asarray(root_pos, dtype=float).copy()
    for i in range(chain.dof):
        p = p + R @ chain.origin[i]
        R = R @ _quat_to_mat(chain.quat[i])
        R = R @ _rodrigues(chain.axis[i], float(q[i]))
    return p + R @ np.asarray(sole_offset, dtype=float)


# ---------------------------------------------------------------------------
# G1
# ---------------------------------------------------------------------------
def g1_contact_geoms(model: mujoco.MjModel, leg: str) -> tuple[str, ...]:
    """Geom names for one foot, in the patched scene.

    Selection is by BODY plus a contact-enabling test rather than by magic
    index. An earlier draft used `g - 14`, which breaks the moment the model is
    regenerated -- and would pick the wrong geoms silently rather than loudly.
    """
    body = model.body(f"{leg}_ankle_roll_link").id
    gs = [g for g in range(model.ngeom)
          if model.geom_bodyid[g] == body
          and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
          and model.geom_contype[g] != 0]
    if not gs:
        raise ValueError(f"{leg}: no contact spheres on ankle_roll_link")
    names = []
    for g in gs:
        named = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        if named is None:
            raise ValueError(
                f"{leg}: contact geom {g} is unnamed. Run "
                "mech/spec/build_g1_scene.py to produce the patched scene -- the "
                "descriptor reads names so the filter has a stable handle.")
        names.append(named)
    return tuple(names)


def g1_sole_offset(model: mujoco.MjModel, leg: str) -> np.ndarray:
    """Sole reference point in the ankle_roll body frame, in METRES.

    Mean of the foot's contact-sphere CENTRES. Averaging fixed points in the
    body frame is not the same as averaging whichever spheres are touching: this
    does not move during stance, which is the property leg odometry needs.
    """
    body = model.body(f"{leg}_ankle_roll_link").id
    pts = [np.array(model.geom_pos[g], dtype=float)
           for g in range(model.ngeom)
           if model.geom_bodyid[g] == body
           and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
           and model.geom_contype[g] != 0]
    if not pts:
        raise ValueError(f"{leg}: no contact spheres on ankle_roll_link")
    return np.mean(pts, axis=0)


def apollo_body(leg: str, joint: str) -> str:
    """`left` + `hip_ie` -> `l_hip_ie_link`. Apollo mixes side-prefixed body names
    with unprefixed leg names, so this mapping has to exist somewhere explicit."""
    return f"{APOLLO_SIDE_PREFIX[leg]}_{APOLLO_JOINT_BODY[joint]}"


def apollo_contact_geoms(model: mujoco.MjModel, leg: str) -> tuple[str, ...]:
    """The single sole box for one foot.

    Selection is by geom NAME, and existence is asserted, because the vendor
    file's `l_foot_fl/fr/bl/br` are mesh assets, not geoms: asking MuJoCo for a
    geom by one of those names returns -1, and indexing geom -1 then silently
    reads the last geom in the model, which sits on the *opposite* foot. That made
    both feet look coincident until the names were checked against the XML.

    Deliberately does NOT require contype != 0. Apollo's soles are contype=0 and
    still make contact, because scene.xml pairs them against the floor explicitly
    and MuJoCo checks a declared pair regardless of the geom's contact mask. An
    earlier version asserted contype != 0 and so rejected the correct model; the
    fix was to patch contype, which made it worse (the floor is contype=0 too, so
    the soles fell straight through). Assert existence, not the mask.
    """
    name = APOLLO_SOLE_GEOM[leg]
    gid = model.geom(name).id
    if gid is None or gid < 0:
        raise ValueError(f"{leg}: no geom named {name} in this model")
    return (name,)


def apollo_sole_offset(model: mujoco.MjModel, leg: str) -> np.ndarray:
    """Sole reference point in the FOOT body frame, in METRES.

    The BOTTOM-FACE CENTRE of the sole box, not its centre: for leg odometry the
    meaningful point is the one that sits on the floor when the foot is flat, and
    the box centre sits half its thickness (9 mm) above that. Fixed in the body
    frame, so it does not move during stance -- the property the G1 sphere mean
    gives, and the one leg odometry actually needs.
    """
    gid = model.geom(APOLLO_SOLE_GEOM[leg]).id
    foot = APOLLO_FOOT_BODY[leg]
    if model.geom_bodyid[gid] != model.body(foot).id:
        raise ValueError(
            f"{leg}: geom {APOLLO_SOLE_GEOM[leg]} is on body "
            f"{model.geom_bodyid[gid]}, expected {foot}")
    p = np.array(model.geom_pos[gid], dtype=float).copy()
    p[2] -= model.geom_size[gid][2]   # box half-thickness
    return p


def op3_contact_geoms(model: mujoco.MjModel, leg: str) -> tuple[str, ...]:
    """OP3's two foot boxes, selected by the ankle-roll body plus a contact test.

    Same construction as G1: by BODY plus a contact-enabling test, never by a
    magic index. OP3 makes the reason obvious -- the whole body is collidable
    capsules, so an index-based selection would happily pick a thigh.
    """
    body = model.body(OP3_FOOT_BODY[leg]).id
    gs = [g for g in range(model.ngeom)
          if model.geom_bodyid[g] == body
          and model.geom_contype[g] != 0
          and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX]
    if len(gs) != 2:
        raise ValueError(
            f"{leg}: expected 2 contact boxes on {OP3_FOOT_BODY[leg]}, found {len(gs)}")
    names = []
    for g in gs:
        named = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        if named is None:
            raise ValueError(
                f"{leg}: contact geom {g} is unnamed. Run "
                "mech/spec/build_op3_scene.py -- the descriptor reads names so the "
                "filter has a stable handle.")
        names.append(named)
    return tuple(names)


def op3_sole_offset(model: mujoco.MjModel, leg: str) -> np.ndarray:
    """Sole reference point in the ankle-roll body frame, in METRES.

    Mean of the two boxes' BOTTOM-FACE centres. Both corrections matter and both
    are silent if skipped: without the half-thickness subtraction the point sits
    4 mm above the floor, and averaging box centres instead of bottom faces is
    what made Apollo's FK read 9 mm wrong until it was caught.

    The two boxes are nearly coincident (0.5 mm apart laterally), so the mean is
    stable -- there is no ill-conditioned choice to make here.
    """
    body = model.body(OP3_FOOT_BODY[leg]).id
    pts = []
    for g in range(model.ngeom):
        if (model.geom_bodyid[g] == body and model.geom_contype[g] != 0
                and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX):
            p = np.array(model.geom_pos[g], dtype=float).copy()
            p[2] -= model.geom_size[g][2]      # box half-thickness
            pts.append(p)
    if not pts:
        raise ValueError(f"{leg}: no contact boxes on {OP3_FOOT_BODY[leg]}")
    return np.mean(pts, axis=0)


def load_op3(model: mujoco.MjModel, joints=OP3_JOINTS, legs=OP3_LEGS) -> LegModel:
    """Build the OP3 descriptor from the model, not from transcribed constants."""
    chains, contacts, soles = [], {}, {}
    for leg in legs:
        p = OP3_SIDE_PREFIX[leg]
        bodies = (OP3_ROOT,) + tuple(f"{p}_{OP3_JOINT_BODY[j]}" for j in joints)
        axis, origin, quat, jnames = [], [], [], []
        for jname, child in zip(joints, bodies[1:]):
            jid = model.joint(f"{p}_{jname}").id
            cid = model.body(child).id
            jnames.append(f"{p}_{jname}")
            axis.append(np.array(model.jnt_axis[jid], dtype=float))
            origin.append(np.array(model.body_pos[cid], dtype=float))
            quat.append(np.array(model.body_quat[cid], dtype=float))
        chains.append(LegChain(leg, tuple(jnames), tuple(bodies),
                               np.array(axis), np.array(origin),
                               np.array(quat), OP3_ROOT))
        contacts[leg] = op3_contact_geoms(model, leg)
        soles[leg] = op3_sole_offset(model, leg)
    return LegModel("op3", tuple(legs), len(joints), tuple(chains),
                    contacts, soles)


def load_apollo(model: mujoco.MjModel, joints=APOLLO_JOINTS,
                legs=APOLLO_LEGS) -> LegModel:
    """Build the Apollo descriptor from the model, not from transcribed constants."""
    chains, contacts, soles = [], {}, {}
    for leg in legs:
        bodies = (APOLLO_ROOT,) + tuple(apollo_body(leg, j) for j in joints)
        axis, origin, quat, jnames = [], [], [], []
        for jname, child in zip(joints, bodies[1:]):
            jid = model.joint(f"{APOLLO_SIDE_PREFIX[leg]}_{jname}").id
            cid = model.body(child).id
            jnames.append(f"{APOLLO_SIDE_PREFIX[leg]}_{jname}")
            # A hinge axis is expressed in the child body frame, which is the frame
            # the rotation is applied in -- so it is the axis this FK needs directly.
            axis.append(np.array(model.jnt_axis[jid], dtype=float))
            origin.append(np.array(model.body_pos[cid], dtype=float))
            quat.append(np.array(model.body_quat[cid], dtype=float))
        chains.append(LegChain(leg, tuple(jnames), tuple(bodies),
                               np.array(axis), np.array(origin),
                               np.array(quat), APOLLO_ROOT))
        contacts[leg] = apollo_contact_geoms(model, leg)
        soles[leg] = apollo_sole_offset(model, leg)
    return LegModel("apollo", tuple(legs), len(joints), tuple(chains),
                    contacts, soles)


def load_g1(model: mujoco.MjModel, joints=G1_JOINTS, legs=G1_LEGS) -> LegModel:
    """Build the G1 descriptor from the model, not from transcribed constants."""
    chains, contacts, soles = [], {}, {}
    for leg in legs:
        bodies = (G1_ROOT,) + tuple(f"{leg}_{j}_link" for j in joints)
        axis, origin, quat, jnames = [], [], [], []
        for jname, child in zip(joints, bodies[1:]):
            jid = model.joint(f"{leg}_{jname}_joint").id
            cid = model.body(child).id
            jnames.append(f"{leg}_{jname}_joint")
            # For a hinge, jnt_axis is expressed in the child body frame, which
            # is the frame the joint rotation is applied in -- so it is also the
            # axis this FK needs, with no extra transform.
            axis.append(np.array(model.jnt_axis[jid], dtype=float))
            origin.append(np.array(model.body_pos[cid], dtype=float))
            quat.append(np.array(model.body_quat[cid], dtype=float))
        chains.append(LegChain(leg, tuple(jnames), tuple(bodies),
                               np.array(axis), np.array(origin),
                               np.array(quat), G1_ROOT))
        contacts[leg] = g1_contact_geoms(model, leg)
        soles[leg] = g1_sole_offset(model, leg)
    return LegModel("g1", tuple(legs), len(joints), tuple(chains), contacts, soles)


def chain_angles(model: mujoco.MjModel, data: mujoco.MjData,
                 chain: LegChain) -> np.ndarray:
    """Joint angles in chain order, read from the sim state by FULL joint name.

    Preferred over `joint_angles`, which reconstructs names as `{leg}_{j}_joint`
    and so only works for G1. Apollo's joints are `l_hip_ie` with an `l_`/`r_`
    prefix and no `_joint` suffix, so the name has to come from the chain -- which
    already holds it -- rather than be rebuilt from the leg name.
    """
    out = np.empty(len(chain.joints))
    for i, jn in enumerate(chain.joints):
        jid = model.joint(jn).id
        out[i] = float(data.qpos[model.jnt_qposadr[jid]])
    return out


def joint_angles(model: mujoco.MjModel, data: mujoco.MjData, leg: str,
                 joints=G1_JOINTS) -> np.ndarray:
    """Leg joint angles in DESCRIPTOR order, read from the sim state."""
    out = np.empty(len(joints))
    for i, j in enumerate(joints):
        jid = model.joint(f"{leg}_{j}_joint").id
        out[i] = float(data.qpos[model.jnt_qposadr[jid]])
    return out