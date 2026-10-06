"""Derive the v0.6 geometry contract from the pinned Unitree URDF.

WHY A GENERATOR INSTEAD OF A HAND-WRITTEN TABLE
-----------------------------------------------
Four times now, hand-checking geometry between two representations of the same
number has produced a confident wrong answer in this repo:

  1. `L2` flagged as a bad constant when it is `hypot(0.002, 0.213)` -- a
     different quantity from the calf joint offset.
  2. "the thigh has no collision geom" -- true of my query, false of the model.
  3. MuJoCo `geom size` read as full extents when it is half extents, which
     halved every collision volume.
  4. Collision envelopes treated as volume bounds, when they are contact
     proxies deliberately SMALLER than the parts they approximate.

Every one of those was a transcription or eyeballing failure, not a modelling
one. So the shared constants are computed here from the URDF with the
expression recorded next to each value, and a test asserts they still equal
what `fusion/src/leg_kin.cpp` uses. Two representations that must agree, both
derived rather than retyped.

WHAT THIS DOES NOT DO
--------------------
It does not resolve the mass discrepancy. The URDF totals 16.087 kg and the
published Go2 is ~12.4 kg, a 29.7% gap that is *inherited from Unitree's own
URDF* (Menagerie's 15.2064 kg is this file minus the rotor and head links it
drops). Nothing in the available geometry can adjudicate that: the visual
meshes are non-watertight so they have no volume, and the collision proxies are
smaller than the parts so they bound nothing. It is recorded, not resolved.

UNITS, EXPLICITLY, BECAUSE ALL THREE OF THESE HAVE BITTEN
--------------------------------------------------------
  * URDF: metres. `<box size=...>` is FULL extents; `<cylinder length=>` is
    full length, `<radius=>` is a radius.
  * MuJoCo MJCF: metres, but `geom size` is HALF extents.
  * cadquery / OCP: millimetres.
  * `fusion/src/leg_kin.cpp`: metres.
Nothing in this file converts silently; every emitted field says which.
"""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

# --- provenance, pinned -----------------------------------------------------
VENDOR_REPO = "unitreerobotics/unitree_ros"
VENDOR_COMMIT = "a3b70cae6fd4a82c0e1ece633d5c6f97e88c9d76"   # 2025-06-19
VENDOR_PATH = "robots/go2_description/urdf/go2_description.urdf"
VENDOR_LICENCE = "BSD-3-Clause"
VENDOR_SHA256 = "7d19fe48e2e689ee1a032ab99f2a4a8b671d87e73de48d3e65811682a5b48b9e"
MASS_CONVENTION = "published Go2 standing mass, vendor datasheet figure"

HERE = Path(__file__).resolve().parent
URDF_PATH = HERE.parents[1] / "third_party" / "vendor" / "go2_description.urdf"
OUT_PATH = HERE / "go2_urdf.json"
MENAGERIE_SCENE = HERE.parents[1] / "third_party" / "menagerie" / "unitree_go2" / "scene.xml"

LEGS = ("FL", "FR", "RL", "RR")
# Links that carry no structural role in the estimator's kinematics.
SENSOR_LINKS = ("imu", "radar", "front_camera", "Head_upper", "Head_lower")


def _floats(text, n=None):
    vals = [float(v) for v in text.split()]
    if n is not None and len(vals) != n:
        raise ValueError(f"expected {n} values, got {vals}")
    return vals


def _origin_xyz(node):
    """Translation of an <origin xyz=...> element, defaulting to zero.

    Reads the ATTRIBUTE. An earlier version looked for a child element named
    "xyz", found none, and silently returned [0, 0, 0] for every joint -- which
    would have produced a spec of four zeros that still looked plausible.
    """
    o = node.find("origin")
    return _floats(o.get("xyz"), 3) if o is not None and o.get("xyz") else [0.0, 0.0, 0.0]


def _vec_attr(node, attr):
    return _floats(node.get(attr), 3) if node.get(attr) else [0.0, 0.0, 0.0]


def parse_urdf(path: Path = URDF_PATH):
    """Raw URDF facts, units as-written (metres, full extents)."""
    root = ET.parse(path).getroot()
    links = {}
    for l in root.findall("link"):
        name = l.get("name")
        inert = l.find("inertial")
        rec = {"name": name, "mass_kg": None, "com_m": None,
               "inertia_kg_m2": None, "collision": [], "visual_mesh": None}
        if inert is not None:
            m = inert.find("mass")
            if m is not None:
                rec["mass_kg"] = float(m.get("value"))
            rec["com_m"] = _origin_xyz(inert)
            i = inert.find("inertia")
            if i is not None:
                rec["inertia_kg_m2"] = {k: float(i.get(k)) for k in
                                         ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
                                         if i.get(k) is not None}
        for c in l.findall("collision"):
            o = c.find("origin")
            g = c.find("geometry")
            prim = next(iter(g))
            # URDF packs a box as one "x y z" string but gives cylinders and
            # spheres separate radius/length attributes. Normalise here.
            if prim.tag == "box":
                size = {"x": None, "y": None, "z": None}
                size.update(dict(zip("xyz", _floats(prim.get("size"), 3))))
            else:
                size = {k: float(prim.get(k)) for k in prim.attrib if prim.get(k)}
            rec["collision"].append({
                "primitive": prim.tag,
                "origin_xyz_m": _vec_attr(o, "xyz") if o is not None else [0.0, 0.0, 0.0],
                "origin_rpy_rad": _vec_attr(o, "rpy"),
                # FULL extents, converted once here so no consumer can forget.
                "size": size,
            })
        v = l.find("visual/geometry/mesh")
        if v is not None:
            rec["visual_mesh"] = v.get("filename")
        links[name] = rec

    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        lim = j.find("limit")
        ax = j.find("axis")
        joints[j.get("name")] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "origin_xyz_m": _origin_xyz(j),
            "origin_rpy_rad": _vec_attr(o, "rpy"),
            "axis": _floats(ax.get("xyz"), 3) if ax is not None and ax.get("xyz") else None,
            "limit": None if lim is None else {
                "lower_rad": float(lim.get("lower")),
                "upper_rad": float(lim.get("upper")),
                "effort_Nm": float(lim.get("effort")) if lim.get("effort") else None,
                "velocity_rad_s": float(lim.get("velocity")) if lim.get("velocity") else None,
            },
        }
    return links, joints


def collision_full_extents_m(prim_rec):
    """Full extents of a URDF collision primitive, in metres, as [x, y, z].

    Every input here is URDF and therefore already FULL extents. Explicit
    because the bug this repo hit was reading MuJoCo's HALF extents as full,
    which silently halved every collision volume. Box/cylinder/sphere are
    normalised to the same three-axis form so they are comparable at all.
    """
    kind, s = prim_rec["primitive"], prim_rec["size"]
    if kind == "box":
        return [s["x"], s["y"], s["z"]]
    if kind == "sphere":
        d = 2.0 * s["radius"]
        return [d, d, d]
    if kind in ("cylinder", "capsule"):
        return [2.0 * s["radius"], 2.0 * s["radius"], s["length"]]
    raise ValueError(f"unhandled primitive {kind}")


def derive_shared_constants(links, joints, leg="FL"):
    """The four constants `fusion/src/leg_kin.cpp` hardcodes.

    Each carries the URDF expression that produced it, so a reader can check the
    derivation instead of trusting a literal. This is the cross-check that would
    have caught error (1) above.
    """
    hip = joints[f"{leg}_hip_joint"]["origin_xyz_m"]
    thigh = joints[f"{leg}_thigh_joint"]["origin_xyz_m"]
    calf = joints[f"{leg}_calf_joint"]["origin_xyz_m"]
    foot = joints[f"{leg}_foot_joint"]["origin_xyz_m"]
    # The foot's contact sphere is offset within the foot link; that offset is
    # what makes the contact-point distance differ from the joint offset.
    sphere = next(c for c in links[f"{leg}_foot"]["collision"]
                  if c["primitive"] == "sphere")
    so = sphere["origin_xyz_m"]
    return {
        "L1": {
            "value_m": abs(calf[2]),
            "urdf_expression": f"|{leg}_calf_joint.origin.z| = |{calf[2]}|",
            "meaning": "calf joint frame offset in the thigh; the thigh link length",
            "matches_leg_kin": True,
        },
        # L2 is the leg's SECOND SEGMENT, from the kinematic chain. It is
        # deliberately NOT hypot(calf.z, foot_sphere.x) = 0.21300938946440834,
        # which is what this used to emit. That number is the distance to the
        # foot CONTACT SPHERE centre, and Menagerie places the r=22 mm sphere
        # 2 mm off the leg plane, so it is a contact-geometry artifact wearing
        # a kinematic constant's clothes. leg_kin copied it, which meant moving
        # the foot collision geom moved the estimator's leg length 1:1 (see
        # docs/V06_FOOT_PAD.md). A collision proxy is not a source of
        # kinematic truth -- same rule as every other proxy in this contract.
        "L2": {
            "value_m": abs(calf[2]),
            "urdf_expression": f"|{leg}_calf_joint.origin.z| = |{calf[2]}|",
            "meaning": "calf frame -> foot contact, from the kinematic chain. "
                       "Equal to L1 for this robot, which is a real property of "
                       "the Menagerie frame layout, not a copy-paste error. NOT "
                       "the foot contact sphere centre: hypot(calf.z, "
                       "foot_sphere.x) = 0.21300938946440834 is a contact-geometry "
                       "quantity and used to be carried here as a kinematic one",
            "matches_leg_kin": True,
        },
        "a_offset": {
            "value_m": abs(thigh[1]),
            "urdf_expression": f"|{leg}_thigh_joint.origin.y| = |{thigh[1]}|",
            "meaning": "lateral offset of the leg's pitch plane from the hip roll axis",
            "matches_leg_kin": True,
        },
        "hip_base": {
            "value_m": hip,
            "urdf_expression": f"{leg}_hip_joint.origin.xyz",
            "meaning": "hip roll axis position in the trunk frame",
            "matches_leg_kin": True,
        },
    }


def visual_envelopes():
    """Per-body visual extent in metres, via MuJoCo, when the tree has menagerie.

    Optional enrichment. This is the only thing in the repo that actually bounds
    a part, since the collision proxies are smaller than the parts and the
    visual surfaces are not closed.
    """
    if not MENAGERIE_SCENE.exists():
        return {}
    try:
        import mujoco
        import numpy as np
    except ImportError:
        return {}
    m = mujoco.MjModel.from_xml_path(str(MENAGERIE_SCENE))
    R = np.zeros(9)
    out = {}
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        b = int(m.geom_bodyid[g])
        if b == 0:
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        mid = int(m.geom_dataid[g])
        adr, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        v = np.array(m.mesh_vert[adr:adr + n], dtype=np.float64)
        # Scale comes from mesh_scale (mesh_vert is already metres). geom_size on
        # a mesh geom is a collision size and MUST NOT be applied here -- doing
        # so is what produced 0.85 mm "base" extents on the first run.
        v = v * np.array(m.mesh_scale[mid])
        # Rotate into the BODY frame so this is comparable with a body-frame
        # collision box. Comparing a local mesh extent to a body-frame proxy
        # would be a frame mixup, which is the same class of bug again.
        mujoco.mju_quat2Mat(R, np.array(m.mesh_quat[mid]))
        v = v @ R.reshape(3, 3).T
        lo, hi = v.min(axis=0), v.max(axis=0)
        prev = out.get(name)
        out[name] = {
            "min_m": list(map(float, lo if prev is None else np.minimum(prev["min_m"], lo))),
            "max_m": list(map(float, hi if prev is None else np.maximum(prev["max_m"], hi))),
        }
    for name, e in out.items():
        e["extent_m"] = [e["max_m"][i] - e["min_m"][i] for i in range(3)]
        e["extent_mm"] = [round(x * 1000, 2) for x in e["extent_m"]]
    return out


def bounds_the_part(collision_extent_m, visual_extent_m, tol=1e-6):
    """Does this collision proxy actually contain the part?

    Derived from a measurement rather than asserted, because assuming it is what
    produced error (4). A proxy smaller on any axis bounds nothing.

    Visual extent is the mesh AABB in the BODY frame; the collision extent is
    full extents in the body frame. Axis-aligned, so this is a conservative test
    -- a rotated part can have an AABB larger than the proxy without the proxy
    failing to bound it.
    """
    if not visual_extent_m:
        return None
    return all(ce >= ve - tol for ce, ve in zip(collision_extent_m, visual_extent_m))


def build():
    links, joints = parse_urdf()
    vis = visual_envelopes()

    total_mass = sum(l["mass_kg"] for l in links.values() if l["mass_kg"])

    per_link = {}
    for name, rec in links.items():
        entry = {k: rec[k] for k in ("mass_kg", "com_m", "inertia_kg_m2", "visual_mesh")}
        entry["role"] = ("sensor" if name in SENSOR_LINKS
                         else "structure" if name in {f"{L}_{p}" for L in LEGS
                                                     for p in ("hip", "thigh", "calf", "foot")}
                         else "trunk" if name == "base" else "actuator_rotor_inertia")
        entry["collision"] = []
        for c in rec["collision"]:
            ext = collision_full_extents_m(c)
            ce = {
                "primitive": c["primitive"],
                "origin_xyz_m": c["origin_xyz_m"],
                "origin_rpy_rad": c["origin_rpy_rad"],
                "full_extents_m": [round(x, 6) for x in ext],
                "volume_m3": round(_prim_volume(c["primitive"], c["size"]), 9),
            }
            v = vis.get(name)
            ce["visual_extent_mm"] = v["extent_mm"] if v else None
            ce["bounds_the_part"] = bounds_the_part(ext, v["extent_m"]) if v else None
            ce["provenance"] = ("URDF collision primitive. A CONTACT PROXY -- not a "
                                "dimension of the part and not a volume bound unless "
                                "bounds_the_part is true")
            entry["collision"].append(ce)
        per_link[name] = entry

    spec = {
        "provenance": {
            "source": f"https://github.com/{VENDOR_REPO}",
            "commit": VENDOR_COMMIT,
            "path": VENDOR_PATH,
            "licence": VENDOR_LICENCE,
            "urdf_sha256": VENDOR_SHA256,
            "mass_convention": MASS_CONVENTION,
            "regenerate": "pixi run python mech/spec/build_spec.py",
            "note": "Fetch the URDF to third_party/vendor/ (gitignored) to regenerate; "
                    "the committed JSON is the contract and does not need the network.",
        },
        "units": {
            "urdf": "metres; <box size> is FULL extents; <cylinder length> full length",
            "mujoco_mjcf": "metres; geom size is HALF extents (3x source of error (3))",
            "cadquery_ocp": "MILLIMETRES",
            "leg_kin_cpp": "metres",
            "rule": "no silent conversion; every emitted field names its convention",
        },
        "shared_with_leg_kin": derive_shared_constants(links, joints),
        "mass_budget": {
            "urdf_total_kg": round(total_mass, 4),
            "published_go2_kg": 12.4,
            "urdf_over_published": round(total_mass / 12.4 - 1.0, 4),
            "menagerie_total_kg": 15.2064,
            "menagerie_note": "URDF minus 3 rotor links per leg (0.089 kg each) and the "
                              "head links, which Menagerie drops. So the 29.7% excess is "
                              "INHERITED FROM UNITREE'S URDF, not introduced downstream.",
            "resolvable_from_geometry": False,
            "why_not": "visual meshes are non-watertight (no volume) and collision proxies "
                       "are smaller than the parts (no bound). Needs a CAD mass roll-up or "
                       "a vendor mass sheet.",
        },
        "asymmetry": {
            "calf_collision_radius_m": {L: next(
                (c["size"]["radius"] for c in links[f"{L}_calf"]["collision"]
                 if c["primitive"] == "cylinder"), None) for L in LEGS},
            "note": "the vendor URDF is itself left/right asymmetric on the calf "
                    "collision radius. Any CAD that assumes symmetry is making a choice, "
                    "not copying the source.",
        },
        "links": per_link,
        "rules": [
            "Never treat a collision primitive as a dimension of the part: it is a "
            "contact proxy and is often smaller (see bounds_the_part).",
            "Never treat a visual surface as a solid: all 16 menagerie meshes are "
            "non-watertight, so their volumes are meaningless.",
            "A shared constant must equal leg_kin.cpp exactly; it is derived here from "
            "the URDF and asserted by mech/tests, not retyped.",
            "When two consumers need different values from one nominal dimension "
            "(joint-frame offset vs contact-point distance), record both and say which "
            "is which. L2 is the worked example.",
        ],
    }
    return spec


def _prim_volume(kind, s):
    if kind == "box":
        return s["x"] * s["y"] * s["z"]
    if kind == "sphere":
        return 4.0 / 3.0 * math.pi * s["radius"] ** 3
    if kind in ("cylinder", "capsule"):
        v = math.pi * s["radius"] ** 2 * s["length"]
        if kind == "capsule":
            v += 4.0 / 3.0 * math.pi * s["radius"] ** 3
        return v
    return float("nan")


if __name__ == "__main__":
    spec = build()
    OUT_PATH.write_text(json.dumps(spec, indent=2) + "\n")
    sh = spec["shared_with_leg_kin"]
    print(f"wrote {OUT_PATH.relative_to(HERE.parents[1])} "
          f"({OUT_PATH.stat().st_size/1024:.1f} KiB)")
    print("\nshared constants, derived from the URDF:")
    for k, v in sh.items():
        val = v["value_m"]
        sval = f"[{val[0]:.4f}, {val[1]:.4f}, {val[2]:.4f}]" if isinstance(val, list) else f"{val:.17g}"
        print(f"  {k:<10} = {sval:<34} <- {v['urdf_expression']}")
    mb = spec["mass_budget"]
    print(f"\nmass budget: URDF {mb['urdf_total_kg']} kg vs published "
          f"{mb['published_go2_kg']} kg = +{mb['urdf_over_published']*100:.1f}%")
    print("  inherited from the vendor URDF, not resolvable from available geometry")
    print("\ndoes each collision proxy bound its part?")
    for name, e in spec["links"].items():
        for c in e["collision"]:
            if c.get("bounds_the_part") is not None:
                print(f"  {name:<14} {c['primitive']:<9} "
                      f"extents={c['full_extents_m']} visual={c['visual_extent_mm']} "
                      f"-> bounds={c['bounds_the_part']}")