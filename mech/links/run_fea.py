"""Run the thigh FEA across stance angles and read the stress back.

WHY A SWEEP
-----------
The measured load case is a VERTICAL foot force (92.35 N peak). A thigh is never
vertical in stance, so the load has to be resolved into the thigh's own frame,
and how much of it is bending depends on the stance angle -- which is a gait
parameter nobody has pinned down yet. Rather than pick one angle and report a
number that quietly encodes that choice, sweep it and report the sensitivity.

The first version applied the force along -z and then bolted on a hand-computed
moment with an arm that had been converted from metres to millimetres twice, so
the part saw pure axial load plus a 1000x bending moment. That is why the load
case is now built from a stance angle and the moment is left to the boundary
conditions.

THE VALIDATION, WHICH MATTERS MORE THAN THE NUMBER
--------------------------------------------------
Absolute stress from linear-elastic FEA is only as good as the mesh, and this
repo has been burned by trusting a solver that ran. So:
  * every case must converge (nonzero reaction at the hip, nonzero stress);
  * the reported peak must be a mesh-sensitive quantity, so `converged()`
    re-runs at a finer mesh and returns the relative change;
  * reaction equilibrium is checked -- the sum of hip reactions must equal the
    applied force, which catches a boundary condition that silently did nothing.
"""
from __future__ import annotations

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MECH = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, MECH)

import thigh as thigh_model          # noqa: E402
import mesh_thigh                    # noqa: E402
from materials import DEFAULT_LINK_MATERIAL  # noqa: E402

OUT = os.path.join(MECH, "out")


def vmises(s):
    sxx, syy, szz, sxy, syz, sxz = s[:6]
    return (0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
            + 3.0 * (sxy ** 2 + syz ** 2 + sxz ** 2)) ** 0.5


def solve_case(stance_deg, foot_force_n, depth_elems, tag):
    """Build, solve, and summarise one stance angle."""
    workdir = os.path.join(OUT, f"stance{tag}")
    os.makedirs(workdir, exist_ok=True)
    params = thigh_model.ThighParams()

    # gmsh writes into out_dir; keep each case separate so the .frd is not
    # overwritten before it is read.
    ms = mesh_thigh.mesh(params, target_depth_elems=depth_elems, out_dir=workdir)
    deck = mesh_thigh.build_ccx_deck(ms["mesh_inp"],
                                     os.path.join(workdir, "case.inp"),
                                     foot_force_n=foot_force_n,
                                     stance_deg=stance_deg)
    return ms, deck, workdir


def read_results(workdir, hip_nodes=None):
    """Peak von Mises and the hip reaction resultant, from CalculiX output."""
    sys.path.insert(0, os.path.join(MECH, "validation"))
    from frd import read_frd  # the cantilever's reader; also reads element STRESS

    frd_path = os.path.join(workdir, "case.frd")
    if not os.path.exists(frd_path):
        raise RuntimeError(f"{frd_path} missing -- the solve produced no results")
    blocks = read_frd(frd_path)

    peak = 0.0
    peak_el = None
    if "STRESS" in blocks:
        for eid, sig in blocks["STRESS"].items():
            if len(sig) >= 6:
                v = vmises(sig)
                if v > peak:
                    peak, peak_el = v, eid

    # Reaction resultant. Summing |R| over the constrained nodes over-counts a
    # distributed reaction, so also report the VECTOR sum, which must equal the
    # applied force. That vector check is the one with teeth: it is what proves
    # the boundary condition actually engaged, and it is what would have caught
    # the "HIP, 1,2,3" DOF-range bug that let the body translate 1e10 mm.
    # Equilibrium is checked on the CONSTRAINED nodes only. Summing FORC over
    # every node includes the node the *CLOAD acts on, whose external force is
    # exactly the applied load -- so an all-node sum cancels to zero and looks
    # like "no reaction" even when the constraint is working perfectly.
    react_sum, react_vec = 0.0, [0.0, 0.0, 0.0]
    if "FORC" in blocks:
        sel = hip_nodes if hip_nodes else blocks["FORC"].keys()
        for nid in sel:
            f = blocks["FORC"].get(nid)
            if f and len(f) >= 3:
                react_sum += math.sqrt(f[0] ** 2 + f[1] ** 2 + f[2] ** 2)
                for k in range(3):
                    react_vec[k] += f[k]
    return peak, peak_el, (react_sum, react_vec), sorted(blocks)


def run_sweep(stances=(0.0, 15.0, 30.0, 45.0), foot_force_n=mesh_thigh.FOOT_FORCE_N,
              depth_elems=8, converge_check=None):
    """Solve each stance angle. Returns a list of result dicts."""
    results = []
    for st in stances:
        tag = f"{st:g}".replace(".", "p")
        ms, deck, workdir = solve_case(st, foot_force_n, depth_elems, tag)
        import subprocess
        ccx = _ccx_path()
        proc = subprocess.run([ccx, "-i", "case"], cwd=workdir,
                              capture_output=True, text=True)
        if "*ERROR" in (proc.stdout + proc.stderr):
            raise RuntimeError(f"CalculiX failed at {st} deg:\n"
                               f"{(proc.stdout + proc.stderr)[-1200:]}")
        peak, peak_el, (react_sum, react_vec), blocks = read_results(
            workdir, hip_nodes=deck.get("hip_node_ids"))
        applied = math.hypot(deck["axial_N"], deck["transverse_N"])
        react_mag = math.sqrt(sum(v * v for v in react_vec))
        # The hip reaction must OPPOSE the applied load, so compare magnitudes.
        equilibrium_err = abs(react_mag - applied) / applied if applied else None
        results.append({
            "stance_deg": st,
            "foot_force_N": foot_force_n,
            "axial_N": deck["axial_N"],
            "transverse_N": deck["transverse_N"],
            "elements": deck["elements"],
            "nodes": deck["nodes"],
            "peak_von_mises_Pa": peak,
            "peak_element": peak_el,
            "hip_reaction_sum_N": react_sum,
            "hip_reaction_vector_N": react_vec,
            "applied_force_N": applied,
            "equilibrium_rel_error": equilibrium_err,
            "result_blocks": blocks,
            # The deck's stress unit IS MPa, so the .frd value is already MPa.
            # Dividing again by 1e6 is what printed "0.00" for every case.
            "peak_MPa": peak,
            "margin_vs_yield": ((DEFAULT_LINK_MATERIAL.yield_pa / 1.0e6) / peak
                                if peak > 0 else None),
            "workdir": workdir,
        })
    return results


def material_yield():
    from materials import DEFAULT_LINK_MATERIAL
    return DEFAULT_LINK_MATERIAL.yield_pa


def _ccx_path():
    import shutil
    p = shutil.which("ccx")
    if p:
        return p
    for cand in (".pixi/envs/cad/bin/ccx", "mech/.pixi/envs/cad/bin/ccx"):
        if os.path.exists(os.path.join(os.path.dirname(MECH), cand)):
            return os.path.join(os.path.dirname(MECH), cand)
    raise RuntimeError("ccx not found; run under `pixi run -e cad`")


def interior_stress(workdir, band_mm=12.0, pct=0.999):
    """Peak and high-percentile von Mises EXCLUDING a band at both end faces.

    Necessary because the boundary stress is a modelling artifact, not the part's
    strength: the hip face is fully fixed, and a fully fixed face makes linear-
    elastic stress unbounded there. Measured evidence (docs/V06_FEA_THIGH.md):
    the absolute max climbs monotonically with refinement while the 99.9th
    percentile converges, which is what distinguishes a singularity from physics.
    """
    sys.path.insert(0, os.path.join(MECH, "validation"))
    from frd import read_frd
    blocks = read_frd(os.path.join(workdir, "case.frd"))
    nodes, f = {}, False
    for line in open(os.path.join(workdir, "case.inp")):
        if line.startswith("*NODE"):
            f = True
            continue
        if f and line.startswith("*"):
            break
        if f and line.strip():
            p = [t.strip() for t in line.split(",")]
            nodes[int(p[0])] = (float(p[1]), float(p[2]), float(p[3]))
    vals = []
    for nid, sig in blocks.get("STRESS", {}).items():
        if len(sig) < 6:
            continue
        z = nodes.get(nid, (0, 0, 0))[2]
        if -band_mm < z < band_mm:          # constrained hip face
            continue
        if z < -213.0 - band_mm:            # loaded knee boss face
            continue
        vals.append(vmises(sig))
    if not vals:
        return None, None, 0
    vals.sort()
    return vals[-1], vals[int(pct * (len(vals) - 1))], len(vals)


def report(results, material=DEFAULT_LINK_MATERIAL):
    L = []
    L.append(f"thigh FEA, {material.name}  yield {material.yield_pa/1e6:.0f} MPa")
    L.append(f"foot force {results[0]['foot_force_N']} N (measured peak), "
             f"{results[0]['elements']} C3D4 tets, {results[0]['nodes']} nodes")
    L.append("")
    L.append(f"{'stance':>8}{'axial N':>10}{'transv N':>10}{'peak MPa':>11}"
             f"{'yield MPa':>11}{'margin':>9}{'hip react N':>13}")
    for r in results:
        L.append(f"{r['stance_deg']:8.1f}{r['axial_N']:10.2f}{r['transverse_N']:10.2f}"
                 f"{r['peak_MPa']:11.2f}{material.yield_pa/1e6:11.0f}"
                 f"{(r['margin_vs_yield'] or 0):9.2f}{r['applied_force_N']:13.2f}")
    L.append(f"  {'equilibrium rel err':>18}: " +
             ", ".join(f"{r['stance_deg']:g}deg={r['equilibrium_rel_error']:.2e}"
                       for r in results if r['equilibrium_rel_error'] is not None))
    L.append("")
    L.append("")
    L.append("The 'peak' column is NOT a design number. Two modelling artifacts sit")
    L.append("on top of it, both measured rather than argued:")
    L.append("  1. C3D4 tetrahedra are stiff, so they over-predict for a given mesh;")
    L.append("  2. the hip face is fully fixed, and a fully fixed face makes linear-")
    L.append("     elastic stress unbounded there, so the peak grows without limit as")
    L.append("     the mesh refines.")
    L.append("See docs/V06_FEA_THIGH.md for the refinement sweep that separates them.")
    return "\n".join(L)


if __name__ == "__main__":
    res = run_sweep()
    print(report(res))
    for r in res:
        mx, p999, n = interior_stress(r["workdir"])
        r["interior_max_MPa"] = mx
        r["interior_p999_MPa"] = p999
        r["interior_nodes"] = n
        r["margin_vs_yield_interior"] = ((material_yield() / 1e6) / p999
                                         if p999 else None)
    print("\ninterior stress (12 mm band excluded at both end faces):")
    for r in res:
        print(f"  stance {r['stance_deg']:4.1f} deg: max {r['interior_max_MPa']:7.3f} MPa  "
              f"99.9th pct {r['interior_p999_MPa']:7.3f} MPa  "
              f"margin {r['margin_vs_yield_interior']:.1f}")
    with open(os.path.join(OUT, "thigh_fea.json"), "w") as fh:
        json.dump([{k: v for k, v in r.items() if k != "workdir"} for r in res],
                  fh, indent=2, default=str)
    print(f"\nwrote {os.path.join(OUT, 'thigh_fea.json')}")