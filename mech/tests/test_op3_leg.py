"""OP3 (Robotis) descriptor gates -- the third robot, and the least like the others.

G1 and Apollo are both 6-DoF humanoids of similar scale. OP3 is a 3.15 kg, 510 mm
miniature, and every difference from the other two widens rather than narrows:

              G1                       Apollo                    OP3
    root     pelvis                   base_link                 body_link
    chain    hip_pitch, hip_roll,     hip_ie(yaw), hip_aa,      hip_yaw, hip_roll,
             hip_yaw, knee,           hip_fe, knee_fe,          hip_pitch, knee,
             ankle_pitch, ankle_roll  ankle_ie, ankle_pd        ank_pitch, ank_roll
    foot     4 spheres                1 box (200x85x18)         2 boxes (127x56, 114x78)
    legs     `left`/`right`, l_/r_    `left`/`right`, l_/r_     `left`/`right`, l_/r_
    joint limits  real                 real                      **NONE** (nkey 0 too)
    mirror rule  quats EQUAL           quats REFLECT about y     quats EQUAL
    hip spacing 129 mm                220 mm                    70 mm

Two of those are traps rather than trivia:

**Zero joint limits.** Every jnt_range is [0,0] and nothing is limited, so T4 is
vacuous for OP3 and the null-space clamp that caught G1 winding `ankle_roll` to
102 rad against a 0.262 rad limit has nothing to clamp against. OP3 will HIDE that
class of bug rather than surface it, which is a reason to keep G1's puppet gates
running rather than a reason to feel covered.

**No keyframes at all** (nkey 0), and the all-zero default pose floats the feet
20.9 mm above the floor. There is nothing to seed from, so the home pose has to be
solved outright -- the same situation as G1's `stand`, with no `scene_mjx.xml` to
fall back on either.

The third mirror rule is also worth stating plainly: G1 and OP3 share one (quats
equal, only body_pos.y mirrors) and Apollo has the other. That is not a
coincidence -- a quaternion about y is invariant under the sagittal reflection, so
any model whose link frames are pure y-rotations behaves like G1 no matter how
different everything else is. Deriving one leg from the other still needs to know
which rule applies, which is why all three are gated.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def op3():
    import sys
    if str(ROOT / "sim") not in sys.path:
        sys.path.insert(0, str(ROOT / "sim"))
    scene = ROOT / ".work" / "op3scene" / "scene.xml"
    if not scene.exists():
        spec = importlib.util.spec_from_file_location(
            "build_op3_scene", ROOT / "mech" / "spec" / "build_op3_scene.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.build(dest=str(scene.parent))
    from otolith_sim.leg_model import load_op3
    model = mujoco.MjModel.from_xml_path(str(scene))
    return model, load_op3(model)


def _poses(model, lm, rng, n):
    """Uniform random joint angles over the FULL range.

    OP3 has no joint limits, so there is no "legal" sub-range to sample and
    sampling the whole +/-pi space is the honest choice -- it also makes this the
    broadest FK test of the three robots.
    """
    chain = lm.chain("left")
    return rng.uniform(-np.pi, np.pi, (n, len(chain.joints)))


def test_fk_matches_mujoco_bit_exactly(op3):
    """Core gate: 3000 random poses per leg, against MuJoCo's own FK."""
    model, lm = op3
    from otolith_sim.leg_model import sole_world
    d = mujoco.MjData(model)
    rng = np.random.default_rng(31415)
    worst = 0.0
    for leg in lm.legs:
        chain = lm.chain(leg)
        foot = model.body(chain.bodies[-1]).id
        root = model.body(chain.bodies[0]).id
        for q in _poses(model, lm, rng, 3000):
            qpos = np.zeros(model.nq)
            qpos[0:3] = rng.uniform(-0.2, 0.2, 3)
            qpos[2] = 0.3
            qpos[3:7] = model.body_quat[root]
            for i, jn in enumerate(chain.joints):
                qpos[model.jnt_qposadr[model.joint(jn).id]] = q[i]
            d.qpos[:] = qpos
            mujoco.mj_forward(model, d)
            mine = sole_world(chain, q, qpos[0:3], qpos[3:7], lm.sole[leg])
            R = d.xmat[foot].reshape(3, 3)
            theirs = d.xpos[foot] + R @ lm.sole[leg]
            worst = max(worst, float(np.abs(mine - theirs).max()))
    assert worst < 1e-9, f"OP3 FK worst error {worst * 1e9:.3f} nm"


def test_sole_is_mean_of_box_bottom_faces(op3):
    """Two boxes per foot, so the sole is a mean -- of BOTTOM faces.

    Both halves are silent if wrong. Skipping the half-thickness subtraction puts
    the point 4 mm above the floor, and averaging box CENTRES is exactly what made
    Apollo's FK read 9 mm wrong until the nm-scale gate caught it.
    """
    model, lm = op3
    for leg in lm.legs:
        bottoms = []
        for name in lm.contact_geoms[leg]:
            gid = model.geom(name).id
            p = np.array(model.geom_pos[gid])
            bottoms.append([p[0], p[1], p[2] - model.geom_size[gid][2]])
        assert lm.sole[leg] == pytest.approx(np.mean(bottoms, axis=0), abs=1e-12)


def test_chain_topology_is_the_tree(op3):
    """Each chain entry must be the real child of the previous one.

    G1 bit twice on this: omitting body_quat gave 53 mm at a zero pose, and
    mirroring the right leg's quaternions gave 331 mm. Both produced plausible
    numbers rather than errors.
    """
    model, lm = op3
    for leg in lm.legs:
        chain = lm.chain(leg)
        assert chain.bodies[0] == "body_link"
        parent_of = {}
        for c in range(model.nbody):
            parent_of[model.body(c).name] = model.body(model.body_parentid[c]).name
        for a, b in zip(chain.bodies[:-1], chain.bodies[1:]):
            assert parent_of.get(b) == a, (
                f"{leg}: {b}'s parent is {parent_of.get(b)}, not {a} -- the chain is "
                "not the tree, and FK will still return a plausible number")


def test_mirror_rule_matches_g1_not_apollo(op3):
    """OP3 shares G1's rule: quaternions EQUAL across legs, only body_pos.y mirrors.

    Apollo's is the other rule -- quaternions reflected about y -- and applying
    either rule to the wrong robot silently misplaces every link past the hip.
    That this happens to agree with G1 is not luck: a rotation about y is
    invariant under the sagittal reflection, so any model whose link frames are
    pure y-rotations satisfies both rules at once. OP3 is, Apollo is not.
    """
    model, lm = op3
    l, r = lm.chain("left"), lm.chain("right")
    for i in range(l.dof):
        assert l.origin[i][1] == pytest.approx(-r.origin[i][1], abs=1e-12), (
            f"link {i} ({l.bodies[i + 1]}): body_pos.y must mirror")
        assert np.allclose(l.quat[i], r.quat[i], atol=1e-12), (
            f"link {i}: OP3 quaternions must be IDENTICAL across legs, not "
            f"reflected -- got {l.quat[i]} vs {r.quat[i]}")
        # Both rules holding simultaneously is what makes this check meaningful.
        assert np.allclose(l.quat[i][1], 0.0, atol=1e-12), (
            f"link {i}: expected an x=0 (y-axis-only) quaternion; got {l.quat[i]}")


def test_zero_joint_limits_is_recorded_not_assumed(op3):
    """OP3 has NO joint limits. Recorded here because T4 is therefore vacuous.

    This is a hazard, not a clean bill of health. `sin_cos_wide` folds to
    |theta| <= 3*pi; with every jnt_range at [0,0] there is nothing for that check
    to look at, so it cannot fail for OP3 even if a joint could rotate past the
    fold. The null-space clamp is likewise a no-op here.

    So OP3 CANNOT validate the joint-range machinery. G1's puppet gates have to
    keep running for that, and this test exists to make the gap explicit rather
    than let three passing robots imply three-fold coverage.
    """
    model, lm = op3
    limited = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
               for j in range(model.njnt) if model.jnt_limited[j]]
    assert limited == [], f"OP3 now has limited joints: {limited}"
    assert all(tuple(model.jnt_range[model.joint(j).id]) == (0.0, 0.0)
               for leg in lm.legs for j in lm.chain(leg).joints), (
        "a joint range is non-zero; T4 would need to say something after all")
    assert model.nkey == 0, (
        f"OP3 now has {model.nkey} keyframes, so the 'no stance to seed from' "
        "assumption in the puppet needs revisiting")


def test_zero_pose_floats_and_stance_is_the_narrowest(op3):
    """Records the two numbers the puppet and the planar argument depend on.

    The all-zero default pose leaves the foot boxes 20.9 mm above the floor, so it
    is not a stance and cannot be used as one.

    The stance is the narrowest of the three robots by a wide margin: hips at
    +/-35 mm, feet 13 mm outboard. The lateral CoM shift that breaks the planar 2R
    model scales with hip spacing, so at 70 mm OP3 is the MOST aggressive test of
    the planar assumption -- G1 is 129 mm and Apollo 220 mm. That ordering is the
    reason OP3 is worth adding after two humanoids that both under-test it.
    """
    model, lm = op3
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    lowest = min(float(d.geom_xpos[model.geom(n).id][2] - model.geom_size[model.geom(n).id][2])
                 for n in lm.contact_geoms["left"])
    assert lowest > 0.005, (
        f"zero-pose sole is {lowest * 1000:.2f} mm above the floor; expected ~20.9 "
        "mm, so the model changed")
    hip_y = float(d.xpos[model.body("l_hip_yaw_link").id][1])
    foot_y = float(np.mean([d.geom_xpos[model.geom(n).id][1]
                            for n in lm.contact_geoms["left"]]))
    assert hip_y == pytest.approx(0.035, abs=1e-6)
    assert foot_y == pytest.approx(0.04775, abs=1e-5)
    assert 2 * hip_y < 0.1, "OP3 hip spacing should be ~70 mm, the narrowest of the three"


def test_patch_names_exactly_four_foot_geoms(op3):
    """The scene patch must name 4 geoms and touch nothing else.

    A first attempt at the patcher split the text on `<geom` and looked for
    `class="foot"` in the following chunk, which also matched the geom inside the
    `<default class="foot">` block and named five. So the count is asserted, not
    just the presence of names.
    """
    model, lm = op3
    assert model.ngeom == 50, (
        f"OP3 has {model.ngeom} geoms; the patched scene should still have 50, since "
        "the patch only adds name attributes")
    for leg in lm.legs:
        assert len(lm.contact_geoms[leg]) == 2
        for n in lm.contact_geoms[leg]:
            assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) >= 0
    # The right foot must be labelled right, not merely numbered.
    for leg in ("left", "right"):
        assert all(n.startswith(leg) for n in lm.contact_geoms[leg])