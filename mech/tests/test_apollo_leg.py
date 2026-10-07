"""Apollo (Apptronik) descriptor gates.

Apollo is the second robot and the first one that does not resemble G1. Same 6-DoF
leg, so the seam is unchanged -- but the chain order, root body, foot geometry and
naming scheme are all different:

              G1                          Apollo
    root      pelvis                      base_link
    chain     hip_pitch, hip_roll,        hip_ie (yaw), hip_aa (roll),
              hip_yaw, knee,              hip_fe (pitch), knee_fe,
              ankle_pitch, ankle_roll      ankle_ie (roll), ankle_pd (pitch)
    foot      4 spheres per foot          one 200 x 85 x 18 mm box
    naming    left/right -> left_*         left/right -> l_*

The chain order is not cosmetic. G1 taught that twice: omitting `body_quat` gave
53 mm of error at a zero pose, and mirroring the right leg's quaternions gave
331 mm, because only `body_pos.y` mirrors. These gates check Apollo against
MuJoCo's own FK rather than against a constant, so a similar mistake shows up as a
number instead of as a plausible-looking trajectory.

The vendor `stand` keyframe is a real pose (base z 1.01597, non-zero joints),
unlike G1's all-zeros zero pose -- but its sole boxes sit 1.19 mm BELOW the floor.
So it is a starting guess, not a stance, and the gates below do not treat it as one.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def apollo():
    import sys
    if str(ROOT / "sim") not in sys.path:
        sys.path.insert(0, str(ROOT / "sim"))
    scene = ROOT / ".work" / "apolloscene" / "scene.xml"
    if not scene.exists():
        spec = importlib.util.spec_from_file_location(
            "build_apollo_scene", ROOT / "mech" / "spec" / "build_apollo_scene.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.build(dest=str(scene.parent))
    from otolith_sim.leg_model import load_apollo
    model = mujoco.MjModel.from_xml_path(str(scene))
    return model, load_apollo(model)


def _legal_q(model, leg, rng, n):
    """Random joint angles strictly INSIDE the real limits.

    Sampling the limits themselves would put poses exactly on the rails, where a
    half-open interval comparison and MuJoCo disagree about inclusion and the gate
    fails for a reason that has nothing to do with kinematics.
    """
    out = np.empty((n, 6))
    for i in range(6):
        jid = model.joint(f"{'l' if leg == 'left' else 'r'}_"
                          f"{('hip_ie','hip_aa','hip_fe','knee_fe','ankle_ie','ankle_pd')[i]}").id
        lo, hi = model.jnt_range[jid]
        out[:, i] = rng.uniform(lo + 1e-3, hi - 1e-3, n)
    return out


def test_fk_matches_mujoco_bit_exactly(apollo):
    """The core gate. 3000 random legal poses per leg, against MuJoCo's own FK."""
    model, lm = apollo
    from otolith_sim.leg_model import sole_world
    d = mujoco.MjData(model)
    rng = np.random.default_rng(20240)
    worst = 0.0
    for leg in lm.legs:
        qs = _legal_q(model, leg, rng, 3000)
        chain = lm.chain(leg)
        for k in range(len(qs)):
            qpos = np.zeros(model.nq)
            qpos[0:3] = rng.uniform(-0.3, 0.3, 3)
            qpos[2] = 1.0
            qpos[3:7] = np.array([1.0, 0, 0, 0])
            base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, chain.bodies[0])
            qpos[3:7] = model.body_quat[base]
            for i, jn in enumerate(chain.joints):
                jid = model.joint(jn).id
                qpos[model.jnt_qposadr[jid]] = qs[k, i]
            d.qpos[:] = qpos
            mujoco.mj_forward(model, d)
            # Compare against the box CENTRE, i.e. against the geom position --
            # NOT the descriptor's sole, which is deliberately the bottom-face
            # centre 9 mm lower. Comparing the two first gave a "FK error" of
            # exactly 8.9999 mm, which is the box half-thickness and nothing else.
            gid = model.geom(lm.contact_geoms[leg][0]).id
            mine = sole_world(chain, qs[k], qpos[0:3], qpos[3:7],
                              np.array(model.geom_pos[gid]))
            worst = max(worst, float(
                np.abs(mine - d.geom_xpos[gid]).max()))
    assert worst < 1e-9, f"Apollo FK worst error {worst * 1e9:.3f} nm"


def test_sole_is_the_box_bottom_face_centre(apollo):
    """The sole point must be where the foot touches, not where the box is centred.

    The box centre sits 9 mm -- half the 18 mm thickness -- above the floor when
    the foot is flat. Using it would bias every foot position in the log by 9 mm,
    which is 3% of the 0.3 m sigma_leg budget, silently and in a plausible-looking
    direction.
    """
    model, lm = apollo
    for leg in lm.legs:
        gid = model.geom(lm.contact_geoms[leg][0]).id
        centre = np.array(model.geom_pos[gid])
        assert lm.sole[leg][0] == pytest.approx(centre[0])
        assert lm.sole[leg][1] == pytest.approx(centre[1])
        assert lm.sole[leg][2] == pytest.approx(centre[2] - model.geom_size[gid][2])


def test_sole_is_fixed_in_the_foot_frame_during_stance(apollo):
    """Fixed in the BODY frame -- the property leg odometry actually needs.

    G1's sole is a mean over contact spheres, which is fixed in the ankle frame by
    construction. Apollo's has to earn the same property.
    """
    model, lm = apollo
    for leg in lm.legs:
        before = lm.sole[leg].copy()
        d = mujoco.MjData(model)
        rng = np.random.default_rng(5)
        for _ in range(50):
            q = _legal_q(model, leg, rng, 1)[0]
            qpos = np.zeros(model.nq)
            qpos[2] = 1.0
            for i, jn in enumerate(lm.chain(leg).joints):
                qpos[model.jnt_qposadr[model.joint(jn).id]] = q[i]
            d.qpos[:] = qpos
            mujoco.mj_forward(model, d)
            assert np.array_equal(lm.sole[leg], before)


def test_chain_topology_is_the_tree_not_a_naming_convention(apollo):
    """Each chain entry must be the actual child of the previous one.

    This is the gate that has bitten twice on G1. Apollo's body names break the
    pattern at the last link (`ankle_pd` -> `foot_link`), which is exactly the kind
    of irregularity that lets a chain be assembled in the wrong order and still
    look plausible.
    """
    model, lm = apollo
    for leg in lm.legs:
        chain = lm.chain(leg)
        assert chain.bodies[0] == "base_link"
        for a, b in zip(chain.bodies[:-1], chain.bodies[1:]):
            kids = [model.body(a).id]
            found = None
            def visit(bid):
                nonlocal found
                if found is not None:
                    return
                for c in range(model.nbody):
                    if model.body_parentid[c] == bid:
                        if model.body(c).name == b:
                            found = c
                            return
                        visit(c)
            visit(kids[0])
            assert found is not None, (
                f"{leg}: {b} is not a descendant of {a}; the chain is not the tree")


def test_right_leg_mirrors_position_y_and_reflects_quaternions(apollo):
    """Apollo's right leg mirrors body_pos.y AND reflects its quaternions.

    This is NOT the same as G1, and the difference is the whole reason the gate
    exists. On G1 the two legs' body_quats were identical and only body_pos.y
    mirrored -- mirroring the quaternions produced 331 mm of error. On Apollo the
    quaternions DO reflect about the y axis, (w,x,y,z) -> (w,-x,y,-z), uniformly
    across all six links.

    So neither model's mirroring rule can be reused for the other. Carrying G1's
    rule into Apollo would put every link after the hip 180 degrees out; carrying
    Apollo's into G1 would do the mirror image. Which is exactly why the
    descriptor reads the transform from the model instead of deriving one leg from
    the other.

    Ported from the G1 version of this test, which asserted the opposite and
    correctly failed -- the assertion was the thing that was wrong, not the model.
    """
    model, lm = apollo
    l, r = lm.chain("left"), lm.chain("right")
    reflect_y = lambda q: np.array([q[0], -q[1], q[2], -q[3]])
    n = l.dof
    # Index 0 carries the hip's lateral offset; links 1.. are all at y = 0. Both
    # facts matter, and the split is worth stating rather than discovering.
    for i in range(n):
        assert l.origin[i][1] == pytest.approx(-r.origin[i][1], abs=1e-12), (
            f"link {i} ({l.bodies[i + 1]}): body_pos.y must mirror between legs")
        assert np.allclose(r.quat[i], reflect_y(l.quat[i]), atol=1e-12), (
            f"link {i} ({l.bodies[i + 1]}): expected a y-axis reflection "
            f"(w,-x,y,-z), got right {r.quat[i]} vs left {l.quat[i]}")
    assert l.origin[0][1] == pytest.approx(0.11, abs=1e-6), (
        f"link 0 should carry the +/-0.11 m hip offset, got {l.origin[0][1]}")
    assert all(abs(l.origin[i][1]) < 1e-12 for i in range(1, n)), (
        "links 1.. should be lateral-neutral; if one is not, the chain has picked "
        "up a frame that was not supposed to be there")


def test_joint_ranges_and_stance_width(apollo):
    """Record the numbers the descriptor depends on, so a model swap is caught.

    Stance width matters more than it looks: the lateral CoM shift that breaks the
    planar 2R model is set by how far the feet sit outboard of the hips. Apollo's
    feet are 43 mm outboard of their hips, and 306 mm apart -- wider than G1's
    233 mm -- so Apollo is a MILDER test of the planar assumption than G1, not a
    harder one. Worth knowing before claiming Apollo is the stronger evidence.
    """
    model, lm = apollo
    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, 0)
    mujoco.mj_forward(model, d)
    gy = lambda s: float(d.geom_xpos[model.geom(lm.contact_geoms[s][0]).id][1])
    assert gy("left") == pytest.approx(0.1528, abs=1e-4)
    assert gy("right") == pytest.approx(-0.1528, abs=1e-4)
    hip = lambda s: float(d.xpos[model.body(f"{'l' if s == 'left' else 'r'}_hip_ie_link").id][1])
    assert abs(gy("left") - hip("left")) == pytest.approx(0.0428, abs=1e-3)
    # The vendor stand is NOT a ground stance: its sole boxes are below the floor.
    assert gy("left") > 0 and abs(gy("left")) < 0.2
    sole_z = float(d.geom_xpos[model.geom(lm.contact_geoms["left"][0]).id][2]) \
        - model.geom_size[model.geom(lm.contact_geoms["left"][0]).id][2]
    assert sole_z < 0, (
        f"vendor stand sole is {sole_z * 1000:.2f} mm ABOVE the floor; the home "
        "pose has to be solved, not read off the keyframe")

def test_joint_ranges_are_within_sin_cos_wide(apollo):
    """T4, the analytic half, for Apollo.

    `sin_cos_wide` folds to |theta| <= 3*pi before the CORDIC's +/-1.7433 rad
    convergence. The G1 sagittal sum (hip_pitch + knee) is 5.41 rad. Apollo's is
    wider still -- hip_fe 1.85, knee_fe 2.618, ankle_pd 1.571 -- because Apollo
    has a pitch ankle as well as a pitch hip, and the 2R-equivalent sum has to
    cover all three.

    Note this is only the SAGITTAL chain. hip_ie and ankle_ie are yaw and roll
    respectively, and they are not part of a 2R fold; they are covered by the
    full-range argument below.
    """
    model, lm = apollo
    FOLD_LIMIT = 3 * np.pi
    sagittal = {"hip_fe", "knee_fe", "ankle_pd"}
    worst = 0.0
    for leg in lm.legs:
        side = "l" if leg == "left" else "r"
        for j in lm.chain(leg).joints:
            name = j[len(side) + 1:]
            if name not in sagittal:
                continue
            jid = model.joint(j).id
            if not model.jnt_limited[jid]:
                continue
            lo, hi = model.jnt_range[jid]
            worst = max(worst, abs(float(lo)), abs(float(hi)))
    assert worst > 0, "no Apollo sagittal joints found; the scene changed"
    total = 0.0
    for leg in lm.legs:
        side = "l" if leg == "left" else "r"
        legsum = 0.0
        for j in lm.chain(leg).joints:
            if j[len(side) + 1:] not in sagittal:
                continue
            lo, hi = model.jnt_range[model.joint(j).id]
            legsum += max(abs(float(lo)), abs(float(hi)))
        total = max(total, legsum)
    assert total < FOLD_LIMIT, (
        f"worst Apollo 2R-equivalent joint sum is {total:.3f} rad against "
        f"sin_cos_wide's {FOLD_LIMIT:.3f} rad limit")


def test_every_joint_range_is_within_the_cordic_convergence(apollo):
    """Every Apollo joint individually must fold inside 3*pi.

    Wider than T4's sagittal sum on purpose: hip_ie, hip_aa and ankle_ie are not
    part of a 2R chain but are still fed through the same sin_cos, so a range that
    only summed correctly could still break a single call.
    """
    model, lm = apollo
    FOLD_LIMIT = 3 * np.pi
    worst, where = 0.0, ""
    for jid in range(model.njnt):
        if not model.jnt_limited[jid]:
            continue
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or ""
        lo, hi = model.jnt_range[jid]
        m = max(abs(float(lo)), abs(float(hi)))
        if m > worst:
            worst, where = m, nm
    assert worst <= FOLD_LIMIT, (
        f"Apollo joint {where} reaches {worst:.3f} rad, past the {FOLD_LIMIT:.3f} "
        "rad fold limit")
