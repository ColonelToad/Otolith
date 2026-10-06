"""Phase P0 gates: the G1 leg descriptor and its forward kinematics.

Three things are load-bearing here, and each one has already been wrong once
during development, so each is gated rather than trusted:

1. **The chain order is the tree topology.** G1's left leg runs
   pelvis -> hip_pitch -> hip_roll -> hip_yaw -> knee -> ankle_pitch ->
   ankle_roll. `hip_pitch` first is counter-intuitive on a robot whose
   quadruped sibling puts abduction first, and reordering it is invisible at
   q=0 for most models.

2. **The fixed `body_quat` is part of the chain.** G1's `hip_roll_link` is
   tilted 10 deg and its `knee_link` -10 deg. Omitting those rotations gave a
   53 mm error at q=0 -- the x offsets stop telescoping -- and would have looked
   like a wrong number rather than a missing term.

3. **The sole reference point is the mean of the foot's contact spheres**, a
   fixed property of the foot, NOT the centroid of whichever spheres are
   currently touching. That distinction is the v0.6 foot-pad lesson applied;
   see sim/otolith_sim/leg_model.py.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
G1_SCENE = ROOT / ".work" / "g1scene" / "scene.xml"


@pytest.fixture(scope="module")
def g1():
    mujoco = pytest.importorskip("mujoco")
    sys_path = ROOT / "sim"
    import sys
    if str(sys_path) not in sys.path:
        sys.path.insert(0, str(sys_path))
    if not G1_SCENE.exists():
        from pathlib import Path as _P
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "build_g1_scene", ROOT / "mech" / "spec" / "build_g1_scene.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.build(dest=str(G1_SCENE.parent))
    model = mujoco.MjModel.from_xml_path(str(G1_SCENE))
    from otolith_sim.leg_model import load_g1
    return model, mujoco, load_g1(model)


def test_chain_order_follows_the_tree_not_a_convention(g1):
    """The descriptor's joint order must be the model's parent chain.

    Cheapest possible guard against a silent reorder: walk the parent map and
    compare. G1 puts hip_pitch before hip_roll, which is the opposite of what a
    quadruped-derived mental model predicts, so it is worth asserting rather than
    commenting.
    """
    model, mujoco, _ = g1
    from otolith_sim.leg_model import G1_JOINTS
    for leg in ("left", "right"):
        expected = []
        parent = model.body("pelvis").id
        seen = set()
        while True:
            child = next((i for i in range(model.nbody)
                          if model.body_parentid[i] == parent
                          and i not in seen
                          and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "")
                          .startswith(leg + "_")), None)
            if child is None:
                break
            seen.add(child)
            parent = child
            expected.append(mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_JOINT, model.body_jntadr[child]))
        walked = [j.split(f"{leg}_", 1)[1].removesuffix("_joint") for j in expected]
        assert walked == list(G1_JOINTS), (
            f"{leg}: descriptor joint order {list(G1_JOINTS)} does not match "
            f"the kinematic chain {walked}")


def test_fk_matches_mujoco(g1):
    """Worst-case FK error over random legal poses, for both the foot and sole.

    The gate is 2 mm to mirror the existing Go2 `leg_kin` validation. The
    measured error is ~0, because this FK is the same arithmetic MuJoCo performs
    rather than a hand-derived planar formula, so the tolerance is headroom for
    floating-point ordering rather than a modelling allowance.
    """
    model, mujoco, lm = g1
    from otolith_sim.leg_model import chain_fk, sole_world, joint_angles
    d = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    pelvis = model.body("pelvis").id
    worst_fk = worst_sole = 0.0
    for _ in range(2000):
        q = np.zeros(model.nq)
        for j in range(1, model.njnt):
            if model.jnt_limited[j]:
                lo, hi = model.jnt_range[j]
                q[model.jnt_qposadr[j]] = rng.uniform(lo, hi)
            else:
                q[model.jnt_qposadr[j]] = rng.uniform(-0.6, 0.6)
        d.qpos[:] = q
        mujoco.mj_forward(model, d)
        for leg in lm.legs:
            ch = lm.chain(leg)
            ja = joint_angles(model, d, leg)
            mine = chain_fk(ch, ja, d.xpos[pelvis], d.xquat[pelvis])
            theirs = d.body(model.body(f"{leg}_ankle_roll_link").id).xpos
            worst_fk = max(worst_fk, float(np.linalg.norm(mine - theirs)))
            ms = sole_world(ch, ja, d.xpos[pelvis], d.xquat[pelvis], lm.sole[leg])
            ts = np.mean([d.geom_xpos[model.geom(n).id]
                          for n in lm.contact_geoms[leg]], axis=0)
            worst_sole = max(worst_sole, float(np.linalg.norm(ms - ts)))
    assert worst_fk < 2e-3, f"ankle FK error {worst_fk * 1000:.4f} mm exceeds 2 mm"
    assert worst_sole < 2e-3, f"sole FK error {worst_sole * 1000:.4f} mm exceeds 2 mm"


def test_sole_point_is_the_mean_of_contact_spheres_and_is_coplanar(g1):
    """The sole point must be a property of the foot, not of the contact set.

    Also asserts the four spheres are coplanar, because if they were not the
    "mean" would be an arbitrary choice rather than a sole reference.
    """
    model, mujoco, lm = g1
    for leg in lm.legs:
        assert len(lm.contact_geoms[leg]) == 4, (
            f"{leg}: expected 4 contact spheres, got {len(lm.contact_geoms[leg])}. "
            "The sole reference point is defined as their mean, so a different "
            "count changes it silently.")
        pts = np.array([model.geom_pos[model.geom(n).id]
                        for n in lm.contact_geoms[leg]])
        assert np.ptp(pts[:, 2]) < 1e-6, (
            f"{leg}: contact spheres are not coplanar (z spread "
            f"{np.ptp(pts[:, 2]) * 1000:.3f} mm); 'sole point' would be arbitrary")
        assert np.allclose(lm.sole[leg], pts.mean(axis=0), atol=1e-9)


def test_sole_point_is_fixed_in_the_foot_frame(g1):
    """Moving the joints must not move the sole point relative to the foot.

    The offset is fixed in the FOOT BODY frame, so in WORLD coordinates it
    necessarily rotates with the foot. The invariant is therefore its LENGTH --
    the sole point stays a fixed distance from the ankle origin through every
    pose -- not its world direction. Checking the direction would be checking
    that the robot never moves.

    The failure mode this guards is someone redefining the sole point as the
    centroid of contacting spheres. That changes its length as contact breaks
    and re-forms, and would inject noise into exactly the measurement the filter
    is making.
    """
    model, mujoco, lm = g1
    from otolith_sim.leg_model import chain_fk, sole_world, joint_angles
    d = mujoco.MjData(model)
    pelvis = model.body("pelvis").id
    seen = []
    for k in range(50):
        q = np.zeros(model.nq)
        for j in range(1, model.njnt):
            if model.jnt_limited[j]:
                lo, hi = model.jnt_range[j]
                q[model.jnt_qposadr[j]] = lo + (hi - lo) * (k / 49.0)
            else:
                q[model.jnt_qposadr[j]] = 0.3 * np.sin(k)
        d.qpos[:] = q
        mujoco.mj_forward(model, d)
        ch = lm.chain("left")
        ja = joint_angles(model, d, "left")
        ank = chain_fk(ch, ja, d.xpos[pelvis], d.xquat[pelvis])
        sole = sole_world(ch, ja, d.xpos[pelvis], d.xquat[pelvis], lm.sole["left"])
        seen.append(np.linalg.norm(sole - ank))
    spread = float(np.ptp(seen))
    expected = float(np.linalg.norm(lm.sole["left"]))
    assert spread < 1e-9, (
        f"sole-to-ankle distance varied by {spread * 1000:.6f} mm across poses; "
        "a contact-dependent sole point would move here")
    assert abs(float(np.mean(seen)) - expected) < 1e-9, (
        f"sole-to-ankle distance {np.mean(seen):.6f} != |offset| {expected:.6f}")


def test_descriptor_shape(g1):
    """The numbers P1's C++ seam will be built against."""
    model, mujoco, lm = g1
    assert lm.name == "g1"
    assert lm.legs == ("left", "right")
    assert lm.n_legs == 2
    assert lm.dof_per_leg == 6
    assert lm.total_dof == 12, (
        f"G1 has {lm.total_dof} leg DoF, same as Go2's 12. Coincidence worth "
        "noting: the WIDTH of the interface need not change, only its meaning.")
    assert abs(float(model.body_mass[1:].sum()) - 33.34) < 0.01, (
        "G1 mass drifted; the scene patch must not change inertias")


def test_joint_ranges_are_within_sin_cos_wide(g1):
    """T4, the analytic half: the CORDIC folding already covers G1.

    `sin_cos_wide` folds to |theta| <= 3*pi before the CORDIC's +/-1.7433 rad
    convergence, which is the fix for the Go2 knee reaching ~2.8 rad. For G1 the
    worst 2R-equivalent sum is knee + hip_pitch = 165 + 145 = 310 deg = 5.41 rad.
    This asserts that against the fold's limit rather than trusting the
    arithmetic, because the limits are model data that can be regenerated.
    """
    model, mujoco, _ = g1
    FOLD_LIMIT = 3 * np.pi
    worst = 0.0
    for jid in range(model.njnt):
        if not model.jnt_limited[jid]:
            continue
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if not nm or "_knee_joint" not in nm:
            continue
        hip = model.joint(nm.replace("_knee_joint", "_hip_pitch_joint")).id
        worst = max(worst,
                    abs(float(model.jnt_range[hip][0]))
                    + abs(float(model.jnt_range[jid][1])))
    assert worst > 0, "no G1 knee found; the scene changed"
    assert worst < FOLD_LIMIT, (
        f"worst 2R-equivalent joint sum is {worst:.3f} rad against sin_cos_wide's "
        f"{FOLD_LIMIT:.3f} rad limit; G1 would need a different folding")


def test_patch_names_exactly_eight_contact_geoms(g1):
    """The scene patch must name both feet and change nothing else."""
    model, mujoco, _ = g1
    named = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(model.ngeom)]
    sole = sorted(n for n in named if n and n.endswith(tuple(f"sole{i}" for i in range(4))))
    assert len(sole) == 8, f"expected 8 named sole geoms, got {len(sole)}: {sole}"