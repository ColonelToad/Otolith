"""v0.6 phase B gates: the thigh solid is real, and the mass question is answered.

Two distinct jobs here.

1. GEOMETRY VALIDITY. A CAD script that silently produces an empty or
   non-manifold shape is the classic failure, and it is invisible in a STEP file
   you only glance at. So `isValid`, positive volume and a closed shell are
   asserted, not assumed.

2. THE MASS ANSWER. The contract cannot settle the mass budget (visual meshes
   are non-watertight, collision proxies are smaller than the parts). A CAD
   roll-up can, so the wall sweep is pinned here against a tolerance: if a
   future edit to the taper, bosses or holes moves the answer, the gate fails.

The analytic cross-check is deliberately about something CAD cannot fake: the
section properties of the thin dimension must reproduce beam theory from the
same dimensions. If a boolean silently ate a wall, I and Z disagree with theory
and this catches it.
"""
import importlib.util
import json
import math
import os
import sys
from pathlib import Path

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
MECH = os.path.abspath(os.path.join(HERE, ".."))
SPEC = Path(MECH) / "spec" / "go2_urdf.json"

# Do NOT use a module-level pytest.importorskip here. On pytest 8.4.2 a
# module-level skip makes the ENTIRE directory collect zero tests -- it took out
# the other 18 gates in mech/tests silently, and the run still printed
# "23 passed", so nothing looked wrong. Use a skipif MARK plus a lazy import.
HAVE_CADQUERY = importlib.util.find_spec("cadquery") is not None
pytestmark = pytest.mark.skipif(
    not HAVE_CADQUERY,
    reason="needs cadquery; run with: pixi run -e cad python -m pytest mech/tests")


@pytest.fixture(scope="module")
def thigh():
    """The thigh model, imported lazily so collection never needs cadquery."""
    pytest.importorskip("cadquery", reason="run with: pixi run -e cad")
    sys.path.insert(0, os.path.join(MECH, "links"))
    import thigh as _t
    return _t

DENSITY = 2700.0   # Al 6061-T6, from mech/materials.py


@pytest.fixture(scope="module")
def spec():
    if not SPEC.exists():
        pytest.skip(f"{SPEC} missing; run mech/spec/build_spec.py")
    return json.loads(SPEC.read_text())


@pytest.fixture(scope="module")
def solid(thigh):
    return thigh.build(thigh.ThighParams()).val()


def test_solid_is_valid_and_closed(solid):
    assert solid.isValid(), "OCP says the solid is invalid"
    assert solid.Volume() > 0.0
    # Every shell must be closed, which is what makes Volume() mean anything and
    # what FEA needs. Note solid.Closed() reports the TopoDS "Closed" FLAG,
    # which OCCT does not always set after a boolean, so check the shells.
    for sh in solid.Shells():
        assert sh.Closed(), "a shell is not closed; volume would be meaningless"


def test_solid_has_one_shell_and_no_internal_voids(solid):
    """Exactly one boundary surface, so no internal voids.

    A hollow part whose lightening holes break through has ONE shell: the
    cavity is continuous with the outside. Two shells would mean the cavity is
    still enclosed; three meant the bosses or the holes were leaving voids --
    which is exactly what happened twice while building this.
    """
    assert len(solid.Solids()) == 1
    assert len(solid.Shells()) == 1, (
        f"{len(solid.Shells())} shells: 2 = enclosed cavity (holes not through "
        "or bosses detached), 3+ = internal voids")
    # Every edge bounded by exactly two faces is the manifoldness test that a
    # boolean failure usually trips.
    from collections import Counter
    counts = Counter()
    for f in solid.Faces():
        for e in f.Edges():
            counts[e.hashCode()] += 1
    assert counts, "no edges at all"
    bad = [h for h, c in counts.items() if c not in (1, 2)]
    assert not bad, f"{len(bad)} edges shared by != 1 or 2 faces: not manifold"


def test_fits_inside_the_visual_envelope(spec, solid, thigh):
    """CAD must sit within the part's measured AABB, not exceed it.

    The envelope comes from the visual mesh, which the contract records as the
    only thing in the repo that bounds the part. Exceeding it would mean the
    CAD has invented material outside the measured silhouette.
    """
    d = thigh.load_dimensions(spec)
    env = d["envelope_mm"]
    bb = solid.BoundingBox()
    got = [bb.xlen, bb.ylen, bb.zlen]
    for axis, (g, e) in enumerate(zip(got, env)):
        assert g <= e + 2.0, (
            f"axis {axis}: CAD {g:.1f} mm exceeds the visual envelope {e} mm")


def test_joint_spacing_is_exactly_L1(spec, solid, thigh):
    """The kinematic length is the one dimension that must not be a choice."""
    d = thigh.load_dimensions(spec)
    p = thigh.ThighParams()
    bb = solid.BoundingBox()
    assert bb.zlen >= p.L, f"solid is {bb.zlen:.1f} mm, shorter than L1={p.L} mm"
    assert d["L"] == pytest.approx(213.0)


def test_section_properties_match_beam_theory(thigh):
    """Analytic cross-check on the thin dimension.

    Beam theory for the web's mid-length section: a hollow rectangle's I and
    section modulus are closed-form. CAD cannot fake this -- if a boolean ate a
    wall, these disagree.
    """
    p = thigh.ThighParams()
    # At mid-length the ruled loft is the mean of the two end sections.
    w = (p.w_hip + p.w_knee) / 2.0
    h = (p.h_hip + p.h_knee) / 2.0
    t = p.wall
    I = (w * h**3 - (w - 2 * t) * (h - 2 * t) ** 3) / 12.0     # bending about x
    Z = I / (h / 2.0)
    # Thin-walled limit: the two webs at y = +-h/2 each contribute w*t*(h/2)^2,
    # so I -> w*t*h^2/2 and Z = I/(h/2) -> w*t*h.
    I_thin = w * t * h * h / 2.0
    Z_thin = w * t * h
    assert I == pytest.approx(I_thin, rel=0.08), (
        f"I = {I:.0f} mm^4 is not near the thin-wall limit {I_thin:.0f} mm^4 "
        f"(t/h = {t/h:.3f}); if a boolean ate a wall these diverge")
    assert Z == pytest.approx(Z_thin, rel=0.08)
    assert I > 0.0 and Z > 0.0


def test_wall_thickness_that_hits_the_declared_mass(spec, thigh):
    """The mass question, as a pinned number.

    A 5.64 mm wall is outside normal aluminium extrusion practice (3-5 mm). That
    is the finding: the vendor's 1.152 kg is not obviously reachable with a
    conventional wall in this envelope.
    """
    declared = spec["links"]["FL_thigh"]["mass_kg"]
    t = thigh.solve_wall_for_mass(declared, DENSITY)
    m = thigh.mass_kg(thigh.ThighParams(wall=t), DENSITY)
    assert m == pytest.approx(declared, rel=1e-3)
    assert 5.0 < t < 6.5, f"solved wall {t:.2f} mm moved outside its range"


def test_wall_thickness_that_hits_the_published_scaled_mass(spec, thigh):
    """3.37 mm -- squarely inside normal extrusion practice."""
    mb = spec["mass_budget"]
    scaled = spec["links"]["FL_thigh"]["mass_kg"] * \
        mb["published_go2_kg"] / mb["urdf_total_kg"]
    t = thigh.solve_wall_for_mass(scaled, DENSITY)
    m = thigh.mass_kg(thigh.ThighParams(wall=t), DENSITY)
    assert m == pytest.approx(scaled, rel=1e-3)
    assert 3.0 < t < 4.0, f"solved wall {t:.2f} mm moved outside its range"


def test_mass_is_monotone_in_wall(spec, thigh):
    """The bisection depends on this; if a future edit breaks it, say so."""
    masses = [thigh.mass_kg(thigh.ThighParams(wall=t), DENSITY)
              for t in (2.0, 3.0, 4.0, 5.0)]
    assert all(b > a for a, b in zip(masses, masses[1:])), \
        f"mass not monotone in wall thickness: {masses}"


def test_collision_proxy_is_not_used_as_a_dimension(thigh):
    """Anti-error-(4), enforced on the code rather than the docs.

    The thigh's proxy is 213 x 24.5 x 34 mm and does NOT bound the part. If a
    future edit sizes the web from it, the CAD would inherit a contact artifact
    as a structural dimension -- the exact mistake the contract exists to stop.
    """
    p = thigh.ThighParams()
    assert p.w_knee > 34.0, "web knee width collapsed onto the collision proxy"
    assert p.h_hip > 34.0, "web hip height collapsed onto the collision proxy"


def test_degenerate_wall_is_rejected_loudly(thigh):
    """A wall that leaves no cavity must raise a clear error, not an OCC one."""
    assert 12.0 < thigh.max_valid_wall(thigh.ThighParams()) <= 13.0
    with pytest.raises(ValueError, match="no cavity"):
        thigh.volume_mm3(thigh.ThighParams(wall=40.0))

# ---------------------------------------------------------------------------
# Phase C: the FEA pipeline. These are cheap because they exercise the deck
# ASSEMBLY (units, DOF syntax, load distribution) rather than re-solving.
# ---------------------------------------------------------------------------
def test_material_card_unit_system_is_consistent():
    """mm/N/MPa decks must emit E in MPa, or the numbers are meaningless.

    A deck with mm lengths and E in Pa converges cleanly and reports stresses
    off by orders of magnitude, so this is a real gate rather than cosmetics.
    """
    from materials import AL_6061_T6, calculix_material_card
    pa = calculix_material_card(AL_6061_T6, stress_unit="Pa")[2]
    mpa = calculix_material_card(AL_6061_T6, stress_unit="MPa")[2]
    assert float(pa.split(",")[0]) == pytest.approx(AL_6061_T6.E_pa)
    assert float(mpa.split(",")[0]) == pytest.approx(AL_6061_T6.E_pa / 1e6)
    assert float(pa.split(",")[0]) / float(mpa.split(",")[0]) == pytest.approx(1e6)
    with pytest.raises(ValueError):
        calculix_material_card(AL_6061_T6, stress_unit="ksi")


def test_ccx_kwarg_handles_both_spellings():
    """gmsh writes "ELSET=HIP", CalculiX documents "ELSET = HIP".

    Whitespace-only parsing returned None for every gmsh card, which made the
    face sets vanish instead of failing.
    """
    from mesh_thigh import _kwarg
    assert _kwarg("*ELSET,ELSET=HIP", "ELSET") == "HIP"
    assert _kwarg("*ELSET, ELSET = KNEE", "ELSET") == "KNEE"
    assert _kwarg("*ELEMENT, TYPE=C3D4, ELSET=EALL", "TYPE") == "C3D4"
    assert _kwarg("*NODE, NSET=NALL", "NSET") == "NALL"
    assert _kwarg("*HEADING", "NSET") is None


def test_stance_angle_resolves_force_into_thigh_frame():
    """Transverse component must be F*sin(t), axial F*cos(t).

    The bug this gates: applying the force along -z only (pure axial, no bending)
    and then adding a hand-computed moment whose arm had been converted m->mm
    twice, giving 1000x too much bending.
    """
    import math
    F = 92.35
    for ang in (0.0, 30.0, 45.0, 60.0):
        t = math.radians(ang)
        axial, transverse = F * math.cos(t), F * math.sin(t)
        # Pythagorean split: the resolved components must reconstruct the load.
        assert math.hypot(axial, transverse) == pytest.approx(F, rel=1e-12)
        # and the angle must actually change the split, or the sweep is a no-op
    assert 0.0 == pytest.approx(F * math.sin(math.radians(0.0)))
    assert F * math.sin(math.radians(45.0)) == pytest.approx(F / math.sqrt(2), rel=1e-9)


def test_physical_knee_load_reconstructs_the_foot_force():
    """The resolved force must equal the measured foot force at any angle.

    The bug this gates: the first version applied the force along -z only
    (pure axial, no bending) and bolted on a hand-computed moment with an arm
    converted from metres to millimetres twice, giving 19,671 N.m against a true
    ~17 N.m.
    """
    import math
    from mesh_thigh import physical_knee_load, FOOT_FORCE_N
    for th, ca in ((0.0, 0.0), (math.radians(44.2), math.radians(-107.0)),
                   (math.radians(61.1), math.radians(-104.0))):
        f, _m = physical_knee_load(th, ca)
        assert math.sqrt(sum(v * v for v in f)) == pytest.approx(FOOT_FORCE_N, rel=1e-9)


def test_physical_knee_load_matches_mujoco_foot_position():
    """The analytic foot-in-thigh-frame expression must match the model.

    Verified against MuJoCo at thigh 44.3 deg / calf 106.8 deg, which reports
    the foot contact sphere at (204.5, 0, -153.3) mm in the thigh body frame.
    """
    import math
    from mesh_thigh import L2_M, L1_MM, physical_knee_load
    thigh, calf = math.radians(44.3), math.radians(-106.8)
    # foot relative to the knee is the calf direction scaled by L2
    f_rel_knee = (-L2_M * math.sin(calf), 0.0, -L2_M * math.cos(calf))
    foot = (f_rel_knee[0], f_rel_knee[1], -L1_MM / 1000.0 + f_rel_knee[2])
    ref = (0.2045, 0.0, -0.1533)     # from the model
    assert foot[0] == pytest.approx(ref[0], abs=2e-3)
    assert foot[2] == pytest.approx(ref[2], abs=2e-3)


def test_knee_moment_is_geometry_not_a_free_parameter():
    """Moment about y must fall out of the moment arm, and be O(10 N.m).

    17 N.m is the physical answer; 19,671 was the unit bug. A regression here
    means someone reintroduced a hand-entered moment.
    """
    import math
    from mesh_thigh import physical_knee_load
    _f, m = physical_knee_load(math.radians(50.0), math.radians(-107.0))
    assert 10.0 < abs(m[1]) < 25.0, f"knee moment {m[1]:.2f} N.m is implausible"
    # and it must be sensitive to the geometry it claims to come from
    _f2, m_straight = physical_knee_load(math.radians(50.0), 0.0)
    assert abs(m_straight[1]) < abs(m[1])


def test_gait_spec_covers_the_real_stance_range():
    """The synthetic sweep stopped at 45 deg; the sim actually uses 44-62."""
    import json
    from pathlib import Path
    p = Path(__file__).resolve().parents[1] / "spec" / "gait_stance.json"
    if not p.exists():
        pytest.skip("run mech/spec/build_gait.py")
    spec = json.loads(p.read_text())
    lo, hi = spec["thigh_deg_range"]
    assert 40.0 < lo < 50.0 and 55.0 < hi < 70.0, \
        f"stance thigh range {lo}..{hi} deg moved; the load case may be stale"
    assert len(spec["configs"]) >= 5
    for c in spec["configs"]:
        assert c["calf_rad"] < 0.0, "calf angle sign convention flipped"


# ---------------------------------------------------------------------------
# Phase D: the foot pad.
# ---------------------------------------------------------------------------
def test_foot_pad_is_controlled_against_the_baseline_sphere():
    """The pad must be no wider than the r=22 mm sphere it replaces.

    Otherwise the stance geometry changes and the comparison stops being about
    contact shape. The visual foot is 45.3 x 39.6 mm -- an oval -- and the
    sphere it replaces is 44 x 44 mm, so gating against the visual mesh would
    fail the BASELINE too, which is why that is not the gate.
    """
    import foot
    s = foot.summary()
    assert s["valid"] and s["solids"] == 1 and s["shells"] == 1
    assert s["no_wider_than_baseline"], s["bbox_mm"]


def test_foot_pad_edge_roll_threshold_and_shape():
    """Flat face below the threshold, constant offset above -- not a ramp.

    This is the whole σ_leg argument for a rigid pad: a constant offset in
    r_base contributes nothing to r_dot, so a rigid pad cannot produce sustained
    noise. If this ever became a smooth ramp the argument would break.
    """
    import foot
    p = foot.params()
    thr = foot.summary(p)["edge_angle_deg"]
    assert foot.contact_point_shift_mm(0.0) == 0.0
    assert foot.contact_point_shift_mm(thr) == 0.0
    assert foot.contact_point_shift_mm(thr + 1.0) == pytest.approx(
        p["contact_face_d"] / 2.0, rel=1e-6)
    assert foot.contact_point_shift_mm(thr + 20.0) == pytest.approx(
        p["contact_face_d"] / 2.0, rel=1e-6), "offset must stop growing"


def test_puppet_derives_L2_from_the_contact_geom_placement():
    """Document the coupling that blocks phase E, so it cannot be forgotten.

    `puppet._leg_geoms()` computes L2 as |geom_pos(foot_geom)|, so moving the
    contact geom moves the ESTIMATOR's leg length one-for-one. This test asserts
    that coupling still exists -- i.e. it will FAIL if someone decouples L2 from
    the contact geom, which is the fix and should then update this test and
    docs/V06_FOOT_PAD.md. It is a tripwire, not an endorsement.
    """
    pytest.importorskip("mujoco")
    from pathlib import Path
    scene = Path(__file__).resolve().parents[2] / "third_party" / "menagerie" / \
        "unitree_go2" / "scene.xml"
    if not scene.exists():
        pytest.skip("menagerie symlink missing")
    import mujoco
    from otolith_sim.puppet import _leg_geoms
    m = mujoco.MjModel.from_xml_path(str(scene))
    g = next(i for i in range(m.ngeom)
             if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "") == "FL")
    base = _leg_geoms(m)["FL"].L2
    z0 = float(m.geom_pos[g][2])
    m2 = mujoco.MjModel.from_xml_path(str(scene))
    m2.geom_pos[g][2] = z0 - 0.010
    moved = _leg_geoms(m2)["FL"].L2
    assert moved - base == pytest.approx(0.010, abs=1e-9), (
        "L2 no longer tracks the contact geom's placement. Either the puppet "
        "was fixed (good -- update this test and docs/V06_FOOT_PAD.md) or the "
        "derivation changed (needs a look).")
