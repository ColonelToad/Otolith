"""v0.6 material table and the volume budget it implies.

Closes v0.6 prerequisite (3): nothing recorded a material, so the FEA harness
validated a beam at E = 200 GPa (steel) while the robot it represents is a
machined-aluminium quadruped. Forces are not stresses without a material.

These tests deliberately do NOT certify anything. The property values are
handbook figures for the named temper, not vendor datasheets, and the volume
check is weak by construction -- the collision geoms are primitives. What they
do is stop the numbers silently drifting and make the known uncertainties
explicit rather than implicit.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from materials import (  # noqa: E402
    ABS, AL_6061_T6, AL_7075_T6, BY_NAME, DEFAULT_LINK_MATERIAL, STEEL_1018,
    calculix_material_card, mass_budget_summary, volume_budget,
)

# Values are m[0] (modulus), m[1] (Poisson), in Pa.
PA = 1e9


def test_elast_card_has_exactly_two_values():
    """CalculiX *ELASTIC takes E and nu, in that order, and nothing else."""
    for m in BY_NAME.values():
        E, nu = m.linear_elastic()
        assert E > 0.0
        assert 0.0 < nu < 0.5, f"{m.name}: nu={nu} is outside 0..0.5"


def test_calculix_card_is_parseable_shape():
    card = calculix_material_card(AL_6061_T6)
    assert card[0].startswith("*MATERIAL, NAME=")
    assert card[1].strip() == "*ELASTIC"
    assert card[2].startswith(f"{AL_6061_T6.E_pa:.6e}")
    assert card[2].endswith(f", {AL_6061_T6.nu}")
    # No trailing comma, no extra column -- a third value silently becomes an
    # expansion coefficient and would be accepted without complaint.
    assert card[2].count(",") == 1


def test_default_link_material_is_aluminium():
    """The robot is aluminium. The steel default existed only because the
    cantilever harness predated any material record."""
    assert DEFAULT_LINK_MATERIAL is AL_6061_T6
    assert AL_6061_T6.E_pa / PA == pytest.approx(68.9)
    assert AL_6061_T6.density_kg_m3 == pytest.approx(2700)


def test_alloy_ordering_is_sane():
    """7075 buys yield over 6061 at slightly higher density -- if this inverts,
    someone has edited the table into nonsense."""
    assert AL_7075_T6.yield_pa > AL_6061_T6.yield_pa
    assert AL_7075_T6.density_kg_m3 > AL_6061_T6.density_kg_m3
    assert AL_7075_T6.E_pa > AL_6061_T6.E_pa
    # And both are far softer and lighter than steel, far stiffer than plastic.
    assert STEEL_1018.E_pa > AL_7075_T6.E_pa > AL_6061_T6.E_pa > ABS.E_pa
    assert STEEL_1018.density_kg_m3 > AL_7075_T6.density_kg_m3 > AL_6061_T6.density_kg_m3 > ABS.density_kg_m3


def test_volume_for_mass_inverts_density():
    for m in BY_NAME.values():
        assert m.volume_for_mass(m.density_kg_m3) == pytest.approx(1.0)


def test_tensile_allowable_applies_safety_factor():
    assert AL_6061_T6.tensile_allowable(2.0) == pytest.approx(138e6)
    assert AL_6061_T6.tensile_allowable(2.0) < AL_6061_T6.yield_pa


def test_volume_budget_is_plausibly_aluminium():
    """The weak cross-check: are the MJCF's declared masses consistent with
    aluminium at the collision-primitive volumes?

    This is reported as WEAK on purpose. The collision geoms are cylinders and
    boxes standing in for machined parts, so agreement to ~25% is the most that
    can be expected, and agreement means only "not obviously wrong" -- never
    "validated". The thigh is absent from this check because its collision geom
    is a mesh, and a primitive volume cannot bound it.
    """
    rows = {r["part"]: r for r in volume_budget()}
    checked = [r for r in rows.values() if r["implied_density_kg_m3"] is not None]
    assert checked, "no link had a primitive collision volume to check"
    for r in checked:
        rho = r["implied_density_kg_m3"]
        assert 500.0 < rho < 6000.0, (
            f"{r['part']}: implied density {rho:.0f} kg/m3 is not near any "
            "structural material -- the declared mass and the collision volume "
            "disagree by more than 2x")
    # Rows that cannot be checked must say so rather than defaulting to a number.
    assert rows["thigh"]["implied_density_kg_m3"] is None
    assert rows["thigh"]["collision_volume_m3"] is None


def test_mass_budget_records_the_published_gap():
    """The model is heavier than the published Go2. That must stay visible,
    because it scales every load case in docs/V06_LOAD_CASES.md."""
    s = mass_budget_summary()
    assert s["model_total_mass_kg"] == pytest.approx(15.2064)
    assert s["model_above_published_fraction"] > 0.2, (
        "if the model mass changed, re-derive the load-case scale factor and "
        "update docs/V06_LOAD_CASES.md with it")
    assert s["load_case_scale_if_published"] == pytest.approx(1.2263, rel=1e-3)
    assert "UNVERIFIED" in s["verdict"]


def test_yield_is_not_consumed_by_any_analysis_yet():
    """Guard the honesty of the module docstring: if a plasticity run ever
    consumes yield_pa, this should be revisited rather than left stale."""
    card = "\n".join(calculix_material_card(AL_6061_T6))
    assert "yield" not in card.lower()
    assert "plastic" not in card.lower()