"""v0.6 geometry contract: the shared constants must match leg_kin.cpp exactly.

These are the gates for the four transcription/eyeballing errors this repo made
in three days, all of which were confident and wrong:

  1. `L2` flagged as a bad constant when it is `hypot(0.002, 0.213)`.
  2. "the thigh has no collision geom" -- true of the query, false of the model.
  3. MuJoCo `geom size` read as full extents when it is half extents.
  4. Collision proxies treated as volume bounds; they are contact proxies that
     are SMALLER than the parts they approximate.

The contract's answer to all four is the same: derive both sides, then assert
they agree. `mech/spec/go2_urdf.json` derives from the pinned vendor URDF;
these tests derive the values `fusion/src/leg_kin.cpp` actually uses, by parsing
its source. Two independent derivations, one equality.
"""
import json
import math
import os
import re
import sys
from pathlib import Path

import pytest

HERE = Path(os.path.dirname(os.path.abspath(__file__)))
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))

SPEC = HERE.parent / "spec" / "go2_urdf.json"
LEG_KIN = REPO / "fusion" / "src" / "leg_kin.cpp"
URDF = REPO / "third_party" / "vendor" / "go2_description.urdf"


@pytest.fixture(scope="module")
def spec():
    if not SPEC.exists():
        pytest.skip(f"{SPEC} missing; run mech/spec/build_spec.py")
    return json.loads(SPEC.read_text())


@pytest.fixture(scope="module")
def leg_kin_literals():
    """The numbers leg_kin.cpp actually uses, parsed from its source.

    Parsed rather than hardcoded so the test fails if someone edits the C++
    without regenerating the spec, which is the drift that matters.
    """
    if not LEG_KIN.exists():
        pytest.skip(f"{LEG_KIN} missing")
    src = LEG_KIN.read_text()
    out = {}
    for name in ("a_offset", "L1", "L2"):
        m = re.search(rf"lg\.{name}\s*=\s*([0-9.eE+-]+)\s*;", src)
        if not m:
            pytest.fail(f"could not find lg.{name} literal in {LEG_KIN}")
        out[name] = float(m.group(1))
    m = re.search(r'hip_base\s*=\s*Eigen::Vector3d\(\s*([-\d.eE+]+)\s*,\s*'
                  r'([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\)', src)
    if not m:
        pytest.fail(f"could not find hip_base literal in {LEG_KIN}")
    out["hip_base"] = [float(m.group(i)) for i in (1, 2, 3)]
    return out


def test_shared_constants_match_leg_kin(spec, leg_kin_literals):
    """The whole point of the contract. Derived from the URDF, compared to C++."""
    sh = spec["shared_with_leg_kin"]
    for name in ("L1", "L2", "a_offset"):
        assert sh[name]["value_m"] == leg_kin_literals[name], (
            f"{name}: URDF derives {sh[name]['value_m']!r} but leg_kin.cpp uses "
            f"{leg_kin_literals[name]!r}. One of them moved.")
    assert sh["hip_base"]["value_m"] == leg_kin_literals["hip_base"]


def test_L2_is_kinematic_not_the_contact_sphere(spec, leg_kin_literals):
    """L2 must come from the kinematic chain, NOT from the contact sphere.

    This test previously asserted the OPPOSITE -- that L2 must be
    hypot(0.213, 0.002) and must NOT be 0.213 -- to lock in a decision that has
    since been reversed, and it would have failed the moment the coupling was
    removed. Both positions were defensible from the contract alone; the thing
    that settled it was that leg_kin *read* the value off a collision geom, so
    editing contact geometry silently moved leg length (docs/V06_FOOT_PAD.md).

    The contract now records L2 = |calf_joint.origin.z| and says why in the
    meaning field, so the reasoning survives even if the number changes again.
    """
    L2 = leg_kin_literals["L2"]
    assert L2 == 0.213
    assert L2 != math.hypot(0.213, 0.002), (
        "L2 is the foot contact sphere distance again; it must not be")
    meaning = spec["shared_with_leg_kin"]["L2"]["meaning"].upper()
    assert "KINEMATIC" in meaning
    assert "CONTACT SPHERE" in meaning, (
        "the meaning field must still name the quantity it is NOT, or the "
        "reversal becomes unexplained")


def test_units_block_names_every_convention(spec):
    """All four unit conventions that have bitten, named in one place."""
    u = spec["units"]
    assert "FULL extents" in u["urdf"]
    assert "HALF extents" in u["mujoco_mjcf"]
    assert "MILLIMETRES" in u["cadquery_ocp"]
    assert u["leg_kin_cpp"] == "metres"


def test_no_collision_proxy_claims_to_bound_its_part(spec):
    """Anti-error-(4). Every proxy measured smaller than its part must say so.

    If a future upstream revision tightens a collision box enough to actually
    bound its part, this fails and the flag becomes meaningful instead of
    decorative.
    """
    checked = 0
    for name, link in spec["links"].items():
        for c in link["collision"]:
            if c.get("bounds_the_part") is None:
                continue
            checked += 1
            if not c["bounds_the_part"]:
                assert c["visual_extent_mm"] is not None
                assert any(ce * 1000 < ve
                           for ce, ve in zip(c["full_extents_m"],
                                             c["visual_extent_mm"])), (
                    f"{name}: proxy marked not-bounding but no axis is smaller")
    assert checked >= 12, f"only {checked} proxies checked; expected the 4 legs + trunk"


def test_collision_extents_are_full_not_half(spec):
    """The thigh's collision box must be 0.213 long, not 0.1065.

    Error (3) read MuJoCo half extents as full and halved every volume.
    """
    thigh = spec["links"]["FL_thigh"]["collision"][0]
    assert thigh["primitive"] == "box"
    assert thigh["full_extents_m"] == pytest.approx([0.213, 0.0245, 0.034])
    assert thigh["volume_m3"] == pytest.approx(0.213 * 0.0245 * 0.034, rel=1e-6)


def test_mass_gap_is_recorded_and_not_silently_resolved(spec):
    mb = spec["mass_budget"]
    assert mb["urdf_total_kg"] == pytest.approx(16.087, abs=1e-3)
    assert mb["urdf_over_published"] == pytest.approx(0.297, abs=5e-3)
    assert mb["resolvable_from_geometry"] is False, (
        "if this ever becomes true, the mass budget needs re-deriving and the "
        "load-case scale factor with it")
    assert "INHERITED" in mb["menagerie_note"]


def test_vendor_asymmetry_is_recorded(spec):
    """The vendor URDF is left/right asymmetric on calf collision radius.

    Recorded so that a symmetric CAD is a stated choice rather than an
    unexamined assumption that happens to match FL.
    """
    r = spec["asymmetry"]["calf_collision_radius_m"]
    assert r["FL"] == pytest.approx(0.012)
    assert r["FR"] == pytest.approx(0.013)
    assert r["FL"] != r["FR"]


def test_provenance_is_pinned_and_licensed(spec):
    p = spec["provenance"]
    assert p["licence"] == "BSD-3-Clause"
    assert re.fullmatch(r"[0-9a-f]{40}", p["commit"]), "vendor commit not pinned"
    assert re.fullmatch(r"[0-9a-f]{64}", p["urdf_sha256"])


@pytest.mark.skipif(not URDF.exists(),
                    reason="vendor URDF not fetched; commit regenerates")
def test_committed_spec_matches_the_pinned_urdf(spec):
    """Guards against hand-editing the contract instead of deriving it."""
    import hashlib
    digest = hashlib.sha256(URDF.read_bytes()).hexdigest()
    assert digest == spec["provenance"]["urdf_sha256"], (
        "the URDF on disk is not the pinned revision; the spec must be "
        "regenerated, not edited")