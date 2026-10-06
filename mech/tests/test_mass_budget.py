"""The mass budget: the one-sided bound, and the decision recorded with it.

The model totals 15.2064 kg against a published ~12.4 kg (+22.6%), which scales
every load case. `docs/V06_MASS_BUDGET.md` establishes that:

  * convex-hull volume is the right instrument, because a hull is bounded even
    when the mesh is not watertight -- the previous divergence-theorem attempt
    reported 6267 cm3 for a link whose hull is 1837 cm3;
  * a hull is >= the true volume, so implied density is a LOWER bound on the
    real material and the test can only detect OVER-mass;
  * no link is over-mass, so the +22.6%-over-published question is invisible to
    it, and settling that needs per-link solid geometry (the parked CAD phase);
  * the conclusion v0.6 rests on survives either way, because 22.6% is small
    against a ~150x stress margin.
"""
from __future__ import annotations

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
MENAGERIE = ROOT / "third_party" / "menagerie" / "unitree_go2" / "scene.xml"

MATERIAL_DENSITY = {"al6061": 2700.0, "al7075": 2810.0, "steel1018": 7850.0}


def _hull_volume(model, data, body_id):
    """Convex-hull volume (m^3) of a body's VISUAL meshes, in the body frame.

    Vertices must be transformed into the body frame: each mesh is authored in
    its own local frame, and stacking them raw misplaces 2-5 of them per body.
    """
    import mujoco
    import numpy as np
    import trimesh
    gids = [g for g in range(model.ngeom)
            if model.geom_bodyid[g] == body_id
            and model.geom_dataid[g] >= 0
            and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    visual = [g for g in gids if model.geom_group[g] == 2]
    pts = []
    for g in (visual or gids):
        mid = model.geom_dataid[g]
        idx = model.mesh_vertadr[mid] + np.arange(model.mesh_vertnum[mid])
        R = data.geom_xmat[g].reshape(3, 3)
        x = data.geom_xpos[g]
        for v in idx:
            pts.append(R @ (model.mesh_vert[v] - model.mesh_pos[mid]) + x)
    uniq = np.unique(np.round(np.array(pts), 9), axis=0)
    return float(trimesh.convex.convex_hull(uniq).volume)


@pytest.fixture(scope="module")
def links():
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("trimesh")
    if not MENAGERIE.exists():
        pytest.skip(f"{MENAGERIE} missing")
    model = mujoco.MjModel.from_xml_path(str(MENAGERIE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    out = {}
    for b in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if name:
            out[name] = (float(model.body_mass[b]), _hull_volume(model, data, b))
    return out


def test_no_link_is_over_mass_for_its_own_hull(links):
    """Implied density must stay below the softest plausible material.

    This is the only direction a convex hull can test: hull >= true volume, so
    implied density <= true density. Nothing here is heavier than solid steel of
    its own envelope, so every part is consistent with being hollow or shelled.

    It does NOT constrain the +22.6%-over-published question, which is the
    under-mass direction. See docs/V06_MASS_BUDGET.md.
    """
    worst = 0.0
    for name, (mass, vol) in links.items():
        assert vol > 0, f"{name} has zero hull volume"
        worst = max(worst, mass / vol)
    assert worst < MATERIAL_DENSITY["steel1018"], (
        f"implied density {worst:.0f} kg/m3 exceeds steel -- at least one link "
        "is over-mass for its own envelope, which a hull volume can detect")


def test_fill_fractions_are_plausible_for_shelled_parts(links):
    """Thigh and trunk should read as thin-walled, calf as mostly air.

    Not a tight tolerance: a hull is a weak proxy for a curved link, and the
    point is only to catch a change that would mean the meshes or masses moved.
    """
    for name, lo, hi in (("base", 5.0, 30.0), ("FL_thigh", 20.0, 55.0),
                         ("FL_hip", 25.0, 60.0), ("FL_calf", 1.0, 15.0)):
        mass, vol = links[name]
        fill = mass / (MATERIAL_DENSITY["al6061"] * vol) * 100.0
        assert lo <= fill <= hi, (
            f"{name} fills {fill:.1f}% of a solid Al 6061 envelope, outside "
            f"{lo}-{hi}%. Either the meshes or the declared masses moved; "
            "docs/V06_MASS_BUDGET.md needs re-measuring")


def test_model_mass_and_the_published_multiplier_are_recorded(links):
    """Pin the two numbers the multiplier is computed from.

    Every load case in docs/V06_LOAD_CASES.md scales with this, so if the model
    mass moves the docs have to move with it. 15.2064 kg / 12.4 kg = 1.2263.
    """
    total = sum(m for m, _ in links.values())
    assert abs(total - 15.2064) < 1e-3, (
        f"model mass is now {total:.4f} kg, not 15.2064; the 1.2263x multiplier "
        "in docs/V06_LOAD_CASES.md and docs/V06_MASS_BUDGET.md is stale")
    published = 12.4
    assert abs(total / published - 1.2263) < 5e-4, total / published
