"""v0.6 material properties, and the volume budget they imply.

Prerequisite (3) of `docs/V06_CAD_FEASIBILITY.md`: nothing in the repo recorded a
material, so the FEA toolchain validated a beam at E = 200 GPa (steel) while the
robot it is meant to represent is a machined-aluminium quadruped. Forces are not
stresses without a material, and this is what makes the load cases in
`docs/V06_LOAD_CASES.md` mean anything.

PROVENANCE, STATED PLAINLY
--------------------------
These are standard handbook values for the named temper and temper designation,
not measurements of any real part, and not vendor datasheets. They are the right
starting point for a first FEA pass and the wrong thing to certify a design
with. Before any allow/ultimate decision, replace each with the supplier's
datasheet for the actual extrusion or billet you have bought. The table is
version-controlled so that swap is a diff, not a redesign.

Sources: standard mechanical-engineering material property tables (ASM
Handbook vol. 2; MatWeb datasheets) for the alloy/temper pairs as listed.
Al 6061-T6 is the default because it is the common robotics extrusion; 7075-T6
is carried because it buys yield for the same density at higher cost.

CalculiX consumes only what `linear_elastic()` returns: E and nu. The yield and
ultimate columns are carried for hand checks and for a future plasticity run --
they are not yet consumed by any analysis in this repo, and pretending
otherwise would be the FEA equivalent of fabricating contact forces.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class Material:
    name: str
    E_pa: float           # Young's modulus
    nu: float             # Poisson ratio
    density_kg_m3: float
    yield_pa: float       # NOT consumed by any analysis yet
    ultimate_pa: float    # NOT consumed by any analysis yet
    note: str = ""

    def linear_elastic(self):
        """Exactly the two values a CalculiX *ELASTIC card needs."""
        return self.E_pa, self.nu

    def volume_for_mass(self, mass_kg):
        """Solid volume (m^3) that `mass_kg` of this material occupies."""
        return mass_kg / self.density_kg_m3

    def tensile_allowable(self, safety_factor):
        """Allowable stress (Pa) at a given safety factor against yield."""
        return self.yield_pa / safety_factor


AL_6061_T6 = Material(
    "Al 6061-T6", 68.9e9, 0.33, 2700.0, 276e6, 310e6,
    "default: the common robotics extrusion, cheap, welds, moderate yield")
AL_7075_T6 = Material(
    "Al 7075-T6", 71.7e9, 0.33, 2810.0, 503e6, 572e6,
    "higher yield for ~4% more density and worse fatigue/weldability")
STEEL_1018 = Material(
    "Steel 1018", 200e9, 0.29, 7850.0, 370e6, 440e6,
    "what mech/validation/cantilever.py uses; a cantilever stand-in, not the robot")
ABS = Material(
    "ABS", 2.0e9, 0.35, 1050.0, 40e6, 46e6,
    "covers and cable guides only -- orders of magnitude too soft for a link")

BY_NAME = {m.name: m for m in (AL_6061_T6, AL_7075_T6, STEEL_1018, ABS)}
DEFAULT_LINK_MATERIAL = AL_6061_T6


def calculix_material_card(mat: Material, name="MATL"):
    """The exact lines a CalculiX *MATERIAL / *ELASTIC block needs."""
    E, nu = mat.linear_elastic()
    return [f"*MATERIAL, NAME={name}",
            f"*ELASTIC",
            f"{E:.6e}, {nu}"]


# ---------------------------------------------------------------------------
# Volume budget: does the declared mass fit the declared geometry?
# ---------------------------------------------------------------------------
# The mass in the MJCF came from somewhere we cannot see, and it is the
# multiplier on every load case: forces scale with mass. So it needs a sanity
# check, and the only check available without a vendor drawing is whether the
# declared mass is consistent with the collision volume at a plausible density.
#
# This is a WEAK check on purpose and is reported as such. The collision geoms
# are primitives (cylinder/box/capsule) standing in for machined parts, and some
# have degenerate sizes, so agreement to ~25% is the best that can be expected.
# Disagreement by more than ~2x would be a real finding; agreement means only
# "not obviously wrong", NOT "validated".

# Declared per-body masses from third_party/menagerie/unitree_go2/go2.xml
# inertial blocks, and the analytic volume of that body's collision primitives.
# Volumes in m^3. Trunk excluded: its geoms include a floor-plane SDF whose
# volume is undefined.
LEG_GEOMETRY = {
    # body:      (mass_kg, collision_volume_m3, note)
    "hip":  (0.6780, 265.9e-6, "cylinder r=46mm h=40mm only -- the actuator housing"),
    "thigh": (1.1520, None,  "MESH collision geom; mesh volume needs CAD, not primitives"),
    "calf": (0.2414, 79.0e-6, "cylinders + a 106.5x12.25x17mm slab, the crude one"),
}
LEG_MASS_KG = {k: v[0] for k, v in LEG_GEOMETRY.items()}
MODEL_TOTAL_MASS_KG = 15.2064
PUBLISHED_GO2_MASS_KG = 12.4


def volume_budget(mat: Material = DEFAULT_LINK_MATERIAL):
    """Per-link: required solid volume vs collision-primitive volume."""
    rows = []
    for part, (mass, vol, note) in LEG_GEOMETRY.items():
        need = mat.volume_for_mass(mass)
        row = {"part": part, "mass_kg": mass,
               "required_volume_m3": need, "collision_volume_m3": vol,
               "note": note}
        if vol:
            row["implied_density_kg_m3"] = mass / vol
            row["volume_ratio_required_over_collision"] = need / vol
        else:
            row["implied_density_kg_m3"] = None
            row["volume_ratio_required_over_collision"] = None
        rows.append(row)
    return rows


def mass_budget_summary(mat: Material = DEFAULT_LINK_MATERIAL):
    rows = volume_budget(mat)
    ratio = MODEL_TOTAL_MASS_KG / PUBLISHED_GO2_MASS_KG
    return {
        "material": mat.name,
        "material_density_kg_m3": mat.density_kg_m3,
        "model_total_mass_kg": MODEL_TOTAL_MASS_KG,
        "published_go2_mass_kg": PUBLISHED_GO2_MASS_KG,
        "model_above_published_fraction": ratio - 1.0,
        # Forces scale linearly with mass, so this is the factor by which every
        # load case would be wrong if the robot is really the published mass.
        "load_case_scale_if_published": ratio,
        "per_link": rows,
        "verdict": ("model is heavier than the published figure; masses are "
                    "internally plausible for aluminium but UNVERIFIED against a "
                    "vendor spec, and they scale every load case"),
    }


def as_dict(m):
    return asdict(m)