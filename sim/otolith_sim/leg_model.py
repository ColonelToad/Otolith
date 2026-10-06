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


def joint_angles(model: mujoco.MjModel, data: mujoco.MjData, leg: str,
                 joints=G1_JOINTS) -> np.ndarray:
    """Leg joint angles in DESCRIPTOR order, read from the sim state."""
    out = np.empty(len(joints))
    for i, j in enumerate(joints):
        jid = model.joint(f"{leg}_{j}_joint").id
        out[i] = float(data.qpos[model.jnt_qposadr[jid]])
    return out