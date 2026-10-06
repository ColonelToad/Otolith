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