"""Parametric Go2 foot pad, and why it is not a sphere.

THE MECHANISM THIS IS HERE TO TEST
----------------------------------
The sim's foot collision is a single r = 22 mm sphere. A sphere has a useful
property that is easy to overlook: the lowest point is always directly below its
centre, so in the FOOT'S OWN FRAME the contact point is fixed at (0, 0, -r) no
matter how the foot tilts. Nothing migrates.

A real foot pad is not spherical. It is a flat contact face with a filleted
edge, and that changes the mechanism: under a tilt of phi the patch pivots
about the edge, so the effective contact point moves roughly

    shift ~ r_edge * sin(phi)

relative to the pad centre in the foot frame. That shift is fed straight into
the estimator's `r_base`, and therefore into `rdot` — the quantity sigma_leg is
actually a noise floor for. So this is the leading structural candidate for
sigma_leg = 0.3 m/s, and it is testable.

A flat face also gives a finite contact patch instead of a mathematical point,
so the peak pressure is bounded and the friction limit mu*N has something to
act on. A point contact has unbounded pressure, which is a property of the
model, not of the robot.

GEOMETRY SOURCE
---------------
The visual mesh is 45.3 x 39.6 x 38.8 mm, and the existing sphere is r = 22 mm
(i.e. 44 mm across). The pad is sized to the SAME 44 mm width so the stance
geometry and therefore `L2` are unchanged between the baseline and the pad —
otherwise this would not be a controlled comparison.

UNITS
-----
Millimetres, as everywhere in mech/links. Note the baseline sphere's centre sits
at (-0.002, 0, 0) in the foot frame (the MJCF collision default's x offset);
`pad_z_ref` reproduces that so the two contact points start co-located.
"""
from __future__ import annotations

import math
import os
import sys

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
MECH = os.path.abspath(os.path.join(HERE, ".."))

# Visual envelope, from the Menagerie foot mesh (mm).
FOOT_ENVELOPE_MM = (45.3, 39.6, 38.8)


def params():
    """All pad dimensions in mm. Returned as a dict so it is easy to print."""
    return {
        # Overall width kept at 44 mm to match the r=22 sphere, so L2 and the
        # stance geometry are unchanged and the comparison is controlled.
        "width": 44.0,
        "contact_face_d": 32.0,     # flat face; +2*edge_radius gives the 44 mm width
        "edge_radius": 6.0,         # the parameter the tilt mechanism scales with
        "height": 22.0,
        "top_radius": 15.0,
        "boss_r": 8.0,              # mounts into the calf
        "boss_h": 18.0,
        # Baseline sphere centre in the foot frame, for co-location.
        "baseline_sphere_r": 22.0,
        "pad_z_ref": -2.0,
    }


def build(p=None):
    """Revolve a profile about z. Returns a cq.Workplane holding one solid."""
    p = p or params()
    R = p["width"] / 2.0
    rf = p["contact_face_d"] / 2.0          # flat contact face radius
    re_ = p["edge_radius"]
    rt = p["top_radius"]
    z0 = p["pad_z_ref"]
    h = p["height"]

    # Half-profile in the (r, z) plane, from the contact face up to the top.
    # Fillet centre sits at (rf, z0 + re_); the face is flat out to rf.
    prof = [
        (0.0,            z0),
        (rf,             z0),
        (rf + re_,       z0 + re_ * (1.0 - math.sin(math.pi / 4))),
        (rf + re_,       z0 + re_ + (h - rt - re_) * 0.35),
        (rt,             z0 + h - rt * 0.2),
        (rt * 0.85,      z0 + h),
        (0.0,            z0 + h),
    ]
    wp = (cq.Workplane("XZ")
          .polyline(prof).close()
          .revolve(360, (0, 0, 0), (0, 1, 0)))
    # Boss on top, so the part has something to mount.
    boss = (cq.Workplane("XY", origin=(0, 0, z0 + h - 1.0))
            .circle(p["boss_r"]).extrude(p["boss_h"]))
    return wp.union(boss)


def contact_point_shift_mm(tilt_deg, p=None):
    """Contact-point offset vs the sphere baseline, for a flat pad.

    NOT a smooth ramp, and getting this wrong is the whole point. A flat face
    plus a filleted edge has two states:

      * tilt below atan(r_edge / r_face): the whole face is down and the contact
        centroid is the face centre -- offset 0, same as a sphere.
      * tilt above it: the face touches along one EDGE LINE, and the contact
        sits at radius r_face from the pad axis -- a CONSTANT offset, not a
        continuously migrating point.

    So the pad contributes a step at the edge angle and then nothing further.
    It does not generate sustained time-varying noise in r_dot, because a
    constant offset in r_base drops out of its derivative. Only the step
    produces a transient, once, at touchdown.

    Measured foot tilt in the sim's own gait is 47.8-63.4 deg through stance,
    always above the threshold -- so in this gait the pad is permanently
    edge-down and the offset is constant. Which is why phase D measures the
    COMPLIANT case next: a rigid pad can only contribute a constant, and
    sigma_leg needs something that varies.
    """
    p = p or params()
    rf = p["contact_face_d"] / 2.0
    edge_angle = math.degrees(math.atan2(p["edge_radius"], rf))
    if abs(tilt_deg) <= edge_angle:
        return 0.0
    return math.copysign(rf, tilt_deg)


def summary(p=None):
    p = p or params()
    s = build(p).val()
    bb = s.BoundingBox()
    return {
        "valid": s.isValid(),
        "solids": len(s.Solids()),
        "shells": len(s.Shells()),
        "volume_mm3": s.Volume(),
        "bbox_mm": (round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)),
        "envelope_mm": FOOT_ENVELOPE_MM,
        # Gate against the BASELINE sphere, not the visual mesh. The visual
        # foot is 45.3 x 39.6 mm -- an oval -- while the r=22 sphere it
        # replaces is 44 x 44 mm, so the baseline already overhangs the visual
        # envelope in y. Comparing to the visual mesh would fail the sphere too,
        # and what matters is that the pad does not change the stance geometry.
        "baseline_diameter_mm": p["baseline_sphere_r"] * 2.0,
        "no_wider_than_baseline": max(bb.xlen, bb.ylen) <= p["baseline_sphere_r"] * 2.0 + 1e-6,
        "edge_angle_deg": math.degrees(math.atan2(p["edge_radius"],
                                                   p["contact_face_d"] / 2.0)),
    }


def contact_face_below_centroid_mm(p=None):
    """How far the contact face sits below the pad's own centroid, in mm.

    Load-bearing in a way that is not obvious. For a mesh geom MuJoCo
    re-centres the mesh on its centroid, and `puppet._leg_geoms()` derives
    leg_kin's L2 as `|geom_pos(foot)|`. So L2 is set by wherever the contact
    face ends up relative to the authored geom position.

    Earlier I tried to FORCE this to exactly 22 mm (the sphere's radius) so the
    pad's contact point would coincide with the sphere's and L2 would stay
    0.21300938946440834. It is reachable -- bisection gives boss_h = 56.5 mm
    -- but that produces a 77.5 mm tall post, well outside the 38.8 mm visual
    foot, so it is not a Go2 foot.

    Better: place the pad so its contact face lands where the sphere's contact
    point was, and REPORT the resulting L2 instead of hiding it. A changed L2
    is itself a finding -- it means the collision geom is load-bearing for the
    estimator -- rather than something to engineer away.
    """
    p = p or params()
    s = build(p).val()
    c = s.Center()
    face_z = s.BoundingBox().zmin
    # Already millimetres. An earlier version multiplied by 1000 here and
    # reported a 10271 mm offset for a 10.27 mm pad -- the same double
    # conversion that produced a 19671 N.m knee moment in the thigh load case.
    return float(c.z - face_z)


def geom_pos_for_same_contact_point(p=None, baseline_sphere_r=0.022,
                                   baseline_z=-0.213):
    """Geom position that puts the pad's contact face where the sphere's was.

    The sphere's contact point sits `baseline_sphere_r` below its geom origin.
    The pad's contact face sits `contact_face_below_centroid_mm()` below its
    centroid, and MuJoCo puts the centroid at the geom origin. So:

        geom_z = (baseline_z - baseline_sphere_r) + face_offset

    Returns (x, y, z) for the MJCF.
    """
    p = p or params()
    off = contact_face_below_centroid_mm(p) * 0.001     # mm -> m
    return (-0.002, 0.0, (baseline_z - baseline_sphere_r) + off)


def implied_L2(p=None, baseline_sphere_r=0.022):
    """L2 the puppet will derive from the pad's geom position, in metres.

    Compare against the baseline 0.21300938946440834. If this moves, every
    estimator result that depends on leg length moves with it.
    """
    g = geom_pos_for_same_contact_point(p, baseline_sphere_r)
    return math.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)


def solve_boss_height(target_mm=22.0, lo=2.0, hi=60.0, tol=1e-4):
    """Boss height putting the contact face `target_mm` below the centroid.

    Kept because it is the knob that would hold L2 fixed if a future phase
    wanted a strictly length-neutral comparison. Bisection because the offset is
    monotone in boss height and has no closed form once the revolve and union
    are involved.
    """
    def f(bh):
        p = params()
        p["boss_h"] = bh
        return contact_face_below_centroid_mm(p) - target_mm
    if f(lo) * f(hi) > 0:
        raise ValueError(f"target {target_mm} mm unreachable: "
                         f"{f(lo):+.2f} at boss_h={lo}, {f(hi):+.2f} at {hi}")
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if f(lo) * f(mid) <= 0:
            hi = mid
        else:
            lo = mid
        if hi - lo < tol:
            break
    return 0.5 * (lo + hi)


def export_for_mujoco(path, p=None):
    """Write the pad STL for MuJoCo.

    NO unit conversion here on purpose. MuJoCo applies no scale to mesh
    coordinates, so the metres/mm factor is declared in the MJCF as
    `<mesh scale="0.001">` instead -- one place, visible in the scene file,
    rather than hidden in a post-processing step. (Doing it the other way round
    is how the pad first came out as a 29 METRE foot: cadquery writes millimetres,
    MuJoCo read them as metres, and `geom_pos` came back 8.058 m instead of
    -0.213, which broke the puppet's IK outright.)
    """
    p = p or params()
    cq.exporters.export(build(p), path, tolerance=0.02, angularTolerance=0.05)
    return path


if __name__ == "__main__":
    import json
    print(json.dumps(summary(), indent=2, default=float))
    print("\ncontact-point migration vs foot tilt (flat face + filleted edge):")
    print(f"{'tilt deg':>10}{'shift mm':>11}")
    for t in (0, 5, 10, 15, 20, 30):
        print(f"{t:10d}{contact_point_shift_mm(t):11.3f}")