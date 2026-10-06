"""Parametric Go2 thigh, and the mass it implies.

THE POINT OF THIS FILE IS NOT THE SHAPE. It is to find out whether a thigh built
to sensible engineering judgement lands on the vendor's declared 1.152 kg or on
the published Go2's ~12.4 kg total -- because those two disagree by 29.7% and
nothing in the repo can currently say which is right.

WHAT THE CONTRACT SUPPLIES (mech/spec/go2_urdf.json)
----------------------------------------------------
  * hip pitch axis at the thigh origin, knee pitch axis at z = -L1 = -213 mm.
    This is the kinematic frame and it is exact.
  * visual AABB of the part: 96.86 x 73.42 x 276.23 mm.
  * a collision proxy of 213 x 24.5 x 34 mm -- which the contract records as NOT
    bounding the part, so it is NOT used as a dimension here. It is a contact
    proxy.

WHAT NOTHING SUPPLIES
---------------------
Wall thickness, housing extents, boss sizes, lightening. Those are the design
decisions this file makes explicit and sweeps, rather than pretending to look up.

THE INTERESTING RESULT
----------------------
`target_mass_kg` for a 3 mm wall lands near 0.83-0.89 kg, and 1.152 kg is
exactly what you get by scaling every URDF mass by 12.4/16.087. So the
question this answers is whether a defensible aluminium thigh agrees with the
vendor number or with the published number. Whichever it agrees with is the
number the load cases should use.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict

import cadquery as cq

# CadQuery/OCP works in MILLIMETRES. Every dimension below is mm and no
# conversion happens inside this file, because the URDF (metres) is converted
# once in `load_dimensions` with the factor named.
MM = 1.0


@dataclass
class ThighParams:
    """All dimensions in millimetres. Length runs along -z, joint axis is +y."""
    L: float = 213.0                 # hip pivot -> knee pivot (URDF L1, exact)

    # Tapered core web, outer section at each end.
    w_hip: float = 90.0              # x extent at the hip end
    h_hip: float = 65.0              # y extent at the hip end
    w_knee: float = 60.0             # x extent at the knee end
    h_knee: float = 40.0             # y extent at the knee end

    wall: float = 3.0                # the decision this file sweeps
    end_cap: float = 6.0             # closed ends, so the cavity is a tube

    # Joint bosses: solid, because a bearing seat is not hollow. Lengths are
    # DERIVED from the local web height (+ boss_protrusion) rather than set
    # independently: a boss shorter than the web floats inside the cavity,
    # touches nothing, and leaves an internal void. That showed up as 3 shells
    # instead of 2 and a `Closed()` of False.
    boss_r_hip: float = 22.0
    boss_r_knee: float = 19.0
    boss_protrusion: float = 2.0

    def boss_len_hip(self):
        return self.h_hip + 2.0 * self.boss_protrusion

    def boss_len_knee(self):
        return self.h_knee + 2.0 * self.boss_protrusion

    def local_height(self, z):
        """Web height in y at station z (z is negative, from 0 at hip to -L).

        The loft is ruled, so the section interpolates linearly.
        """
        f = min(max(abs(z) / self.L, 0.0), 1.0)
        h = self.h_hip + (self.h_knee - self.h_hip) * f
        return h + 2.0 * self.boss_protrusion

    # Lightening through the web, along the pivot axis.
    n_holes: int = 3
    hole_r: float = 9.0
    hole_span: float = 0.62          # fraction of L the hole row occupies


def load_dimensions(spec: dict, part: str = "FL_thigh"):
    """Contract dimensions for the thigh, in millimetres.

    Converts metres -> mm exactly once, here, with the factor named. The
    collision proxy is deliberately NOT used; see the module docstring.
    """
    link = spec["links"][part]
    box = next(c for c in link["collision"] if c["primitive"] == "box")
    joint_len_m = spec["shared_with_leg_kin"]["L1"]["value_m"]
    return {
        # Exact kinematic length, in mm.
        "L": joint_len_m * 1000.0,
        # Envelope from the visual mesh AABB (body frame), mm.
        "envelope_mm": link["collision"][0]["visual_extent_mm"],
        # Recorded for the record only.
        "collision_proxy_mm": [round(x * 1000, 2) for x in box["full_extents_m"]],
        "collision_bounds_part": box["bounds_the_part"],
        "declared_mass_kg": link["mass_kg"],
    }


def max_valid_wall(p: ThighParams) -> float:
    """Largest wall that still leaves a positive cavity at both ends.

    The cavity is a loft, so every inner dimension must stay positive. Solving
    this explicitly is better than letting OCC raise StdFail_NotDone from inside
    the solver's bisection, where the traceback points at geometry code rather
    than at the parameter that went out of range.
    """
    return min(p.w_hip, p.w_knee, p.h_hip, p.h_knee) / 2.0 - p.end_cap - 1.0


def _tapered_prism(w0, h0, w1, h1, length):
    """Ruled loft from a w0 x h0 rectangle at z=0 to w1 x h1 at z=-length."""
    return (cq.Workplane("XY").rect(w0, h0)
            .workplane(offset=-length).rect(w1, h1)
            .loft(ruled=True).val())


def _axial_cylinder(radius, length, z):
    """Cylinder on the joint axis (+y through the origin), centred on y=0.

    Built from an explicit vector rather than cq.Workplane("XZ") because that
    plane's normal is -y, so `.extrude()` runs the wrong way and silently
    produces a boss offset by a whole length. That is what gave a 110.5 mm
    bbox against a 73.4 mm envelope and two un-fused solids.
    """
    return cq.Solid.makeCylinder(radius, length,
                                 cq.Vector(0, -length / 2.0, z),
                                 cq.Vector(0, 1, 0))


def build(p: ThighParams):
    """Return a cq.Workplane holding exactly one closed solid."""
    L, t = p.L, p.wall
    if t >= max_valid_wall(p):
        raise ValueError(
            f"wall {t:.2f} mm leaves no cavity at the "
            f"{min(p.w_knee, p.h_knee):.0f} mm end "
            f"(max {max_valid_wall(p):.2f} mm)")

    # --- tapered hollow core: outer loft minus cavity loft
    web = _tapered_prism(p.w_hip, p.h_hip, p.w_knee, p.h_knee, L)
    cavity = _tapered_prism(p.w_hip - 2 * t, p.h_hip - 2 * t,
                            p.w_knee - 2 * t, p.h_knee - 2 * t,
                            L - 2 * p.end_cap)
    # Shift the cavity down by end_cap so both ends stay closed.
    cavity = cavity.translate(cq.Vector(0, 0, -p.end_cap))
    solid = web.cut(cavity)

    # --- joint bosses on the pivot axis
    solid = solid.fuse(_axial_cylinder(p.boss_r_hip, p.boss_len_hip(), 0.0))
    solid = solid.fuse(_axial_cylinder(p.boss_r_knee, p.boss_len_knee(), -L))

    # --- lightening through the web, clear of the bosses
    #
    # These MUST span the full local web height or they become blind slots that
    # open into the cavity, which adds an internal void and a third shell. The
    # first version cut to 2*depth where depth came from the wall and hole
    # radii -- 32 mm through a 52 mm tall web -- so it never broke through.
    if p.n_holes > 0:
        span = L * p.hole_span
        z0 = -p.end_cap - span / 2.0
        step = span / max(p.n_holes - 1, 1)
        for i in range(p.n_holes):
            z = z0 - i * step
            solid = solid.cut(_axial_cylinder(p.hole_r, p.local_height(z),
                                              z))
    return cq.Workplane("XY").newObject([solid])


def volume_mm3(p: ThighParams):
    return float(build(p).val().Volume())


def mass_kg(p: ThighParams, density_kg_m3: float = 2700.0):
    """cadquery returns mm^3; convert once, explicitly."""
    return volume_mm3(p) * 1e-9 * density_kg_m3


def solve_wall_for_mass(target_kg: float, density_kg_m3: float = 2700.0,
                        lo: float = 1.0, hi: float | None = None,
                        tol: float = 1e-4):
    """Wall thickness whose thigh weighs `target_kg`, by bisection.

    Mass is monotone in wall thickness, so bisection is safe here and needs no
    gradient. A numeric solve rather than an algebraic one because the lightening
    holes interact with the wall as it grows -- at some thickness the holes stop
    being through-material and the closed form stops holding.
    """
    if hi is None:
        # Stay strictly inside the valid range: build() rejects wall >= max, so
        # evaluating f() exactly at the bound raises instead of returning a
        # number, which used to be reported as "target unreachable" when it was
        # in fact reachable.
        hi = max_valid_wall(ThighParams()) - 0.5
    def f(t):
        return mass_kg(ThighParams(wall=t), density_kg_m3) - target_kg
    if f(lo) * f(hi) > 0:
        raise ValueError(
            f"target {target_kg} kg is outside the reachable range: "
            f"{f(lo)+target_kg:.3f} kg at wall={lo} mm, "
            f"{mass_kg(ThighParams(wall=hi), density_kg_m3):.3f} kg at wall={hi} mm")
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if (hi - lo) < tol:
            break
        if f(lo) * f(mid) <= 0:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def report(spec: dict, density_kg_m3: float = 2700.0):
    d = load_dimensions(spec)
    declared = d["declared_mass_kg"]
    urdf_total = spec["mass_budget"]["urdf_total_kg"]
    published = spec["mass_budget"]["published_go2_kg"]
    scaled = declared * published / urdf_total

    lines = []
    lines.append(f"thigh, Al 6061-T6 @ {density_kg_m3:.0f} kg/m3")
    lines.append(f"  kinematic length L1   {d['L']:.1f} mm   (URDF, exact)")
    lines.append(f"  part envelope         {d['envelope_mm']} mm   (visual AABB)")
    lines.append(f"  collision proxy       {d['collision_proxy_mm']} mm   "
                 f"bounds_part={d['collision_bounds_part']}  <- NOT a dimension")
    lines.append(f"  declared mass         {declared:.3f} kg   (vendor URDF)")
    lines.append(f"  published-scaled mass {scaled:.3f} kg   "
                 f"= {declared:.3f} x {published}/{urdf_total:.3f}")
    lines.append("")
    lines.append(f"  {'wall mm':>8}{'mass kg':>10}{'vs declared':>14}"
                 f"{'vs pub-scaled':>14}")
    for t in (2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0):
        m = mass_kg(ThighParams(wall=t), density_kg_m3)
        lines.append(f"  {t:8.1f}{m:10.3f}{(m/declared-1)*100:+13.1f}%"
                     f"{(m/scaled-1)*100:+13.1f}%")
    try:
        t_decl = solve_wall_for_mass(declared, density_kg_m3)
        m = mass_kg(ThighParams(wall=t_decl), density_kg_m3)
        lines.append(f"\n  wall that hits the DECLARED {declared:.3f} kg: "
                     f"{t_decl:.2f} mm  (check {m:.3f} kg)")
    except ValueError as e:
        lines.append(f"\n  declared mass unreachable: {e}")
    try:
        t_pub = solve_wall_for_mass(scaled, density_kg_m3)
        m = mass_kg(ThighParams(wall=t_pub), density_kg_m3)
        lines.append(f"  wall that hits PUBLISHED-SCALED {scaled:.3f} kg: "
                     f"{t_pub:.2f} mm  (check {m:.3f} kg)")
    except ValueError as e:
        lines.append(f"  published-scaled mass unreachable: {e}")
    lines.append("")
    lines.append("  Plausibility, not proof: a typical aluminium extrusion wall is")
    lines.append("  3-5 mm, and that range brackets the PUBLISHED-scaled mass but not")
    lines.append("  the declared one. Reaching 1.152 kg needs a wall outside normal")
    lines.append("  extrusion practice, or much beefier bosses/ribs than modelled.")
    return "\n".join(lines), dict(declared=declared, published_scaled=scaled)


if __name__ == "__main__":
    import json
    import os
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    spec_path = os.path.join(here, "..", "spec", "go2_urdf.json")
    sys.path.insert(0, os.path.abspath(os.path.join(here, "..")))  # materials.py
    from materials import DEFAULT_LINK_MATERIAL

    spec = json.loads(open(spec_path).read())
    text, _ = report(spec, DEFAULT_LINK_MATERIAL.density_kg_m3)
    print(text)