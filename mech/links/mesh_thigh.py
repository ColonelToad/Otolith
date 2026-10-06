"""Mesh the thigh with gmsh and emit a CalculiX deck.

THE FIRST UNPROVEN LINK IN THE REPO
-----------------------------------
Everything mechanical before this was model-only. This is where a STEP solid
becomes a solved stress field, and it is the step most likely to be wrong in a
way that produces plausible numbers.

WHAT IS DERIVED RATHER THAN ASSUMED
-----------------------------------
The boundary-condition faces are found by *geometry*, not by hardcoded face
indices. The hip boss is the cylindrical surface centred on the pivot axis at
z = 0; the knee boss is the one at z = -L. gmsh numbers faces however it likes,
and the earlier CAD work in this repo already produced one confident wrong
answer from trusting an index-like thing (`geom size` read as full extents). So
the selector below matches on centre and radius and asserts that it found
exactly one face of each kind. If it finds zero or two, it fails.

MESH DENSITY IS A MEASURED RULE, NOT A HABIT
--------------------------------------------
The cantilever sweep in mech/validation/sweep.py found that bending stress
converges with the number of elements across the SECTION DEPTH and not at all
with refinement along the span. So the target here is elements through the
structural section, and the span is refined only enough to avoid a coarse
aspect ratio on the taper.

WHY NO SHELLS
-------------
Solid elements, because they need no wall-thickness assumption beyond what the
CAD already has, and because the cantilever gate that validated this solver is a
solid-element validation. Shells would be cheaper and are the natural next step
once a stress number is trusted.

BOUNDARY CONDITIONS, AND WHAT THEY ASSUME
-----------------------------------------
The load cases (docs/V06_LOAD_CASES.md) give a per-foot VERTICAL force and no
joint reactions -- the reaction distribution is still unmeasured. So:
  * hip boss cylindrical surface: fully fixed, standing in for the load path
    into the body;
  * knee boss: the foot force and the moment it carries about the knee pivot.
The moment arm is L2, the calf length, because the foot force acts L2 beyond
the knee. That arm is a geometry fact, not a fitted one.
"""
from __future__ import annotations

import math
import os
import sys

import cadquery as cq
import gmsh

HERE = os.path.dirname(os.path.abspath(__file__))
MECH = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(MECH, "links"))
sys.path.insert(0, MECH)

import thigh as thigh_model  # noqa: E402
from materials import DEFAULT_LINK_MATERIAL, calculix_material_card  # noqa: E402

# From docs/V06_LOAD_CASES.md: per-foot peak vertical GRF.
FOOT_FORCE_N = 92.35
L2_M = 0.21300938946440834   # leg_kin's L2 == calf length, per the contract
L2_MM = L2_M * 1000.0
L1_MM = 213.0                    # thigh length, hip pivot to knee pivot

# gmsh writes 12-char fixed-width floats, and CalculiX will not accept more than
# 16 entries on a line.
FLOAT_FMT = "{:.6e}"


def _chunk(items, n=16):
    return [", ".join(items[i:i + n]) for i in range(0, len(items), n)]


def find_boss_faces(params):
    """Locate the hip and knee boss CYLINDRICAL surfaces by type and extent.

    Matching on centre-of-mass z looks obvious and does not work: the boss is
    booleaned against the web, so OCC trims it and its centre of mass lands at
    z = +11.8 (hip) and z = -223.8 (knee) rather than on the pivot. Matching on
    a quarter-diagonal radius is worse -- the boss END CAPS are planes of
    44 x 0 x 44 and 38 x 0 x 38, which score exactly 22.0 and 19.0 on that
    heuristic, so the first version selected two flat discs and would have
    constrained nothing.

    So: filter on the surface TYPE being a cylinder, then require both
    cross-section extents to be the boss diameter AND the y extent to be the
    boss length. The lightening holes (r=9, y-extent ~4) cannot pass.
    """
    def cylinders_with(diameter, length, tol=1.5):
        out = []
        for dim, tag in gmsh.model.getEntities(2):
            if "Cylinder" not in gmsh.model.getType(dim, tag):
                continue
            bb = gmsh.model.getBoundingBox(dim, tag)
            dx, dy, dz = bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]
            if (abs(dx - diameter) < tol and abs(dz - diameter) < tol
                    and abs(dy - length) < tol):
                out.append((int(tag), dx, dy, dz))
        return out

    hip = cylinders_with(2 * params.boss_r_hip, params.boss_len_hip())
    knee = cylinders_with(2 * params.boss_r_knee, params.boss_len_knee())
    for label, found in (("hip", hip), ("knee", knee)):
        if len(found) != 1:
            raise RuntimeError(
                f"expected exactly 1 {label} boss cylinder "
                f"(d={2*(params.boss_r_hip if label=='hip' else params.boss_r_knee)}, "
                f"len={params.boss_len_hip() if label=='hip' else params.boss_len_knee()}), "
                f"matched {len(found)}: {found}")
    # They must be well separated, or the two selectors collided on one feature.
    hip_bb = gmsh.model.getBoundingBox(2, hip[0][0])
    knee_bb = gmsh.model.getBoundingBox(2, knee[0][0])
    gap = abs((hip_bb[2] + knee_bb[2]) / 2.0)
    if gap < params.L * 0.5:
        raise RuntimeError(f"hip and knee bosses are only {gap:.1f} mm apart; "
                           "the selector matched the wrong feature")
    return [hip[0][0]], [knee[0][0]]


def mesh(params: thigh_model.ThighParams, target_depth_elems: int = 8,
         span_elems: int = 60, out_dir: str = None):
    """Build, mesh, and write a CalculiX deck. Returns a summary dict."""
    out_dir = out_dir or os.path.join(MECH, "out")
    os.makedirs(out_dir, exist_ok=True)
    wp = thigh_model.build(params)
    solid = wp.val()
    if not solid.isValid():
        raise RuntimeError("refusing to mesh an invalid solid")
    step = os.path.join(out_dir, "thigh.step")
    cq.exporters.export(wp, step)

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("thigh")
        gmsh.model.occ.importShapes(step)
        gmsh.model.occ.synchronize()

        hip_tags, knee_tags = find_boss_faces(params)

        # Nominal section depth at mid-span, for the measured density rule.
        h_mid = params.local_height(-params.L / 2.0) - 2 * params.boss_protrusion
        target = h_mid / target_depth_elems
        # Keep the span from producing a wild aspect ratio on the taper.
        target = min(target, params.L / span_elems)
        gmsh.option.setNumber("Mesh.MeshSizeMax", target)
        gmsh.option.setNumber("Mesh.MeshSizeMin", target / 8.0)
        gmsh.option.setNumber("Mesh.Algorithm", 6)          # Frontal-Delaunay
        gmsh.option.setNumber("Mesh.Algorithm3D", 10)       # HXT, fast & robust
        gmsh.option.setNumber("Mesh.ElementOrder", 1)
        # Without SaveAll, gmsh's Abaqus writer emits ONLY elements that belong to
        # a physical group. My physical groups are 2D faces, so every one of the
        # ~23k tetrahedra was silently dropped and the deck contained surface
        # triangles only -- a "mesh" with no volume to solve in.
        gmsh.option.setNumber("Mesh.SaveAll", 1)
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.option.setNumber("Geometry.OCCFixSmallEdges", 1)
        gmsh.option.setNumber("Geometry.Tolerance", 1e-6)

        gmsh.model.addPhysicalGroup(2, hip_tags, name="HIP")
        gmsh.model.addPhysicalGroup(2, knee_tags, name="KNEE")
        gmsh.model.mesh.generate(3)
        gmsh.model.mesh.optimize("Netgen")

        nodes = gmsh.model.mesh.getNodes()[0]
        etypes, _, enodes = gmsh.model.mesh.getElements(3)
        etype_counts = {gmsh.model.mesh.getElementProperties(int(t))[0]: int(len(e))
                        for t, e in zip(etypes, enodes)}
        # Extension decides the format: gmsh keys off the LAST component, so
        # "thigh.inp.mesh" silently produced MSH with no * cards at all.
        gmsh.write(os.path.join(out_dir, "thigh.inp"))
    finally:
        gmsh.finalize()

    summary = {
        "step": step,
        "mesh_inp": os.path.join(out_dir, "thigh.inp"),
        "hip_faces": [int(t) for t in hip_tags],
        "knee_faces": [int(t) for t in knee_tags],
        "nodes": len(nodes),
        "volume_mm3": float(solid.Volume()),
        "target_mesh_mm": target,
        "element_types": etype_counts,
    }
    return summary


# --- CalculiX deck -----------------------------------------------------------
def _kwarg(upper_line, name):
    """Pull NAME=value out of an Abaqus card; None if absent.

    Tolerates both "ELSET=HIP" (gmsh's spelling, no space) and "ELSET = HIP"
    (CalculiX's documentation). Splitting on whitespace alone returned None for
    every gmsh card, which made the face sets vanish rather than fail loudly.
    """
    for part in upper_line.split(","):
        part = part.strip()
        if "=" in part:
            k, _, v = part.partition("=")
            if k.strip().upper() == name.upper():
                return v.strip()
        else:
            bits = part.split()
            if len(bits) >= 2 and bits[0] == name.upper():
                return bits[1]
    return None


def read_gmsh_inp(path):
    """Parse gmsh's Abaqus/CalculiX .inp.

    Returns (nodes, vol, surfs, nsets, elsets).

    Classified by the TYPE on the *ELEMENT card, not by node count: CPS4 (a
    surface quad) and C3D4 (a volume tet) both carry 4 nodes, so counting
    nodes silently merges the surface shell into the volume mesh. That matters
    because the boundary-condition faces arrive as an *ELSET of SURFACE elements
    which then has to be mapped to nodes.
    """
    nodes, vol, surfs, nsets, elsets = {}, {}, {}, {}, {}
    mode = None
    cur_elset = cur_type = None
    with open(path) as fh:
        for raw in fh:
            s = raw.strip()
            if not s:
                continue
            if s.startswith("*"):
                up = s.upper()
                if up.startswith("*NODE"):
                    mode = "node"
                elif up.startswith("*ELEMENT"):
                    mode = "elem"
                    cur_elset = _kwarg(up, "ELSET")
                    cur_type = (_kwarg(up, "TYPE") or "").upper()
                elif up.startswith("*NSET"):
                    mode = ("set", nsets, _kwarg(up, "NSET"))
                elif up.startswith("*ELSET"):
                    mode = ("set", elsets, _kwarg(up, "ELSET"))
                else:
                    mode = None
                continue
            if mode == "node":
                p = [x.strip() for x in s.split(",") if x.strip()]
                nodes[int(p[0])] = tuple(float(v) for v in p[1:4])
            elif mode == "elem":
                p = [int(x) for x in s.replace(",", " ").split()]
                eid, conn = p[0], p[1:]
                if cur_type.startswith("C3D") or cur_type.startswith("F3D"):
                    vol[eid] = conn
                else:
                    surfs[eid] = conn
            elif isinstance(mode, tuple) and mode[0] == "set":
                _, store, key = mode
                for tok in s.replace(",", " ").split():
                    store.setdefault(key, []).append(int(tok))
    return nodes, vol, surfs, nsets, elsets


def nodes_of(*element_dicts, ids):
    """Union the nodes of the given element ids across several element dicts."""
    out = set()
    for e in ids:
        for d in element_dicts:
            if e in d:
                out.update(d[e])
                break
    return sorted(out)


def build_ccx_deck(gmsh_inp, out_path, material=DEFAULT_LINK_MATERIAL,
                   foot_force_n=FOOT_FORCE_N, stance_deg: float = 0.0,
                   hip_dof=(1, 3)):
    """Assemble a CalculiX deck from gmsh's mesh.

    Boundary conditions are stated here rather than imported, because gmsh does
    not emit the CalculiX *SURFACE keyword and the element edges belonging to a
    physical curve (that is what calculix/gmsh2ccx exists to work around). Doing
    it from the node sets is equivalent for a face-selected constraint and has
    one less moving part.
    """
    nodes, elements, surfs, nsets, elsets = read_gmsh_inp(gmsh_inp)

    def face_nodes(name):
        # Prefer an NSET if gmsh wrote one; otherwise the ELSET of elements on
        # that face, mapped to their nodes.
        if name in nsets:
            return sorted(nsets[name])
        if name in elsets:
            return nodes_of(elements, surfs, ids=elsets[name])
        return []

    hip_nodes = face_nodes("HIP")
    knee_nodes = face_nodes("KNEE")
    if not hip_nodes or not knee_nodes:
        raise RuntimeError(
            f"missing face sets: HIP={len(hip_nodes)} KNEE={len(knee_nodes)}; "
            f"nsets={sorted(nsets)} elsets={sorted(elsets)}")

    L = []
    L += [f"** thigh FEA; {len(nodes)} nodes, {len(elements)} elements",
          f"** foot force {foot_force_n} N (measured peak, docs/V06_LOAD_CASES.md)",
          "** units: mm, N, MPa (consistent system; CalculiX is unitless)",
          f"** material {material.name}  E={material.E_pa/1e6:.6g} MPa, "
          f"nu={material.nu}"]
    L.append("*HEADING")
    L.append("Go2 thigh, static strength at measured peak foot force")

    # Nodes: mm throughout, which is CalculiX's working unit when Pa is used for
    # E. Keeping the deck in mm and Pa means stress comes out in Pa directly.
    L.append("*NODE, NSET=NALL")
    for n in sorted(nodes):
        x, y, z = nodes[n]
        L.append(f"{n}, {x:.6f}, {y:.6f}, {z:.6f}")

    # Tetrahedra: gmsh's default volume mesh. Coarser and stiffer than hexes
    # for the same node count, so the stress here is an UPPER bound on the true
    # peak until it is confirmed converged by refinement.
    L.append("*ELEMENT, TYPE=C3D4, ELSET=EALL")
    for e in sorted(elements):
        L.append(f"{e}, " + ", ".join(str(n) for n in elements[e]))

    L.append("*NSET, NSET=HIP")
    L += _chunk([str(n) for n in hip_nodes])
    L.append("*NSET, NSET=KNEE")
    L += _chunk([str(n) for n in knee_nodes])

    # UNIT SYSTEM: mm, N, MPa. See calculix_material_card's docstring -- mixing
    # mm lengths with E in Pa converges cleanly and reports nonsense.
    L += list(calculix_material_card(material, name="MATL", stress_unit="MPa"))
    L.append("*SOLID SECTION, ELSET=EALL, MATERIAL=MATL")

    L.append("*STEP")
    L.append("*STATIC")
    # Load case. The measured quantity is a VERTICAL foot force; a thigh is
    # never vertical in stance, so the force has to be resolved into the thigh's
    # own frame. At stance angle t from vertical:
    #     axial       F*cos(t)   (compression down the link)
    #     transverse  F*sin(t)   (this is what bends the hip, because the hip is
    #                              fixed and the force acts L1 away from it)
    # The bending moment is therefore NOT applied by hand -- the fixed-hip
    # boundary condition generates it. An earlier version applied the foot force
    # along -z and then bolted on a moment with a doubly-converted arm, which
    # loaded the part axially only and then over-bent it by 1000x.
    th = math.radians(stance_deg)
    f_axial = foot_force_n * math.cos(th)
    f_trans = foot_force_n * math.sin(th)
    L.append(f"** stance {stance_deg:.1f} deg: axial {f_axial:.2f} N, "
             f"transverse {f_trans:.2f} N")
    # *BOUNDARY takes nset, FIRSTdof, LASTdof [, value]. Writing "HIP, 1,2,3"
    # is read as DOF 1..2 with value 3, so the constraint silently did nothing
    # and the body translated by 1e10 mm while the solve still "converged".
    L.append("*BOUNDARY")
    L.append(f"HIP, {hip_dof[0]}, {hip_dof[1]}")   # load path into the body
    # DISTRIBUTE the load over the knee face. Concentrating 92 N on a single node
    # creates a point singularity: the peak landed on that node at 2.18 MPa and
    # barely moved with stance angle, because it was measuring the singularity
    # rather than the part. A real joint reaction acts over the bearing area, so
    # spread it uniformly across the face nodes.
    L.append(f"** load spread over {len(knee_nodes)} knee face nodes")
    for nid in knee_nodes:
        if abs(f_trans) > 0:
            L.append("*CLOAD")
            L.append(f"{nid}, 1, {-f_trans / len(knee_nodes):.6e}")
        L.append("*CLOAD")
        L.append(f"{nid}, 3, {-f_axial / len(knee_nodes):.6e}")
    L.append("*NODE FILE OUTPUT")
    L.append("U")
    L.append("RF")          # reactions, so equilibrium can be checked
    L.append("*EL FILE OUTPUT")
    L.append("S")
    L.append("*NODE PRINT, NSET=HIP")
    L.append("RF")
    L.append("*END STEP")

    with open(out_path, "w") as fh:
        fh.write("\n".join(L) + "\n")
    return {"nodes": len(nodes), "elements": len(elements),
            "hip_nodes": len(hip_nodes), "knee_nodes": len(knee_nodes),
            "hip_node_ids": hip_nodes,
            "stance_deg": stance_deg,
            "axial_N": f_axial, "transverse_N": f_trans, "path": out_path}


if __name__ == "__main__":
    import json
    p = thigh_model.ThighParams()
    ms = mesh(p, out_dir=os.path.join(MECH, "out"))
    deck = build_ccx_deck(ms["mesh_inp"], os.path.join(MECH, "out", "thigh_ccx.inp"))
    print(json.dumps({**ms, **deck}, indent=2, default=str))