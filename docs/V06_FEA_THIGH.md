# v0.6 phase C: thigh FEA — the first solved stress field in the repo

2026-10-05. This is where a STEP solid becomes a solved stress field. It is the
first mechanical analysis in this project and it needed **six** corrections
before the numbers meant anything. Each is recorded, because five of them
produced plausible output rather than an error — the failure mode this repo has
now hit repeatedly.

`mech/links/mesh_thigh.py` (mesh + deck) · `mech/links/run_fea.py` (sweep +
results) · harness in `mech/validation/frd.py`

## Result

Al 6061-T6, yield 276 MPa, foot force 92.35 N (the measured peak from
`docs/V06_LOAD_CASES.md`), C3D4 tetrahedra, 48 503 elements / 14 499 nodes.

| stance | axial N | transverse N | interior max MPa | 99.9th pct MPa | margin vs yield |
|---|---|---|---|---|---|
| 0° | 92.35 | 0.00 | 0.409 | 0.350 | 790 |
| 15° | 89.20 | 23.90 | 0.909 | 0.840 | 329 |
| 30° | 79.98 | 46.17 | 1.511 | 1.394 | 198 |
| 45° | 65.30 | 65.30 | 2.011 | 1.858 | 149 |

**The thigh is not the weak link.** It runs at roughly 1/150th of yield at the
worst stance angle considered, and the stress rises monotonically with stance
angle exactly as bending predicts — which is the first sign the load case is
physically sensible rather than accidentally plausible.

Equilibrium check: the vector sum of hip reactions equals the applied force to a
relative error of **3e-8 to 9e-7** at every stance angle.

## Why a stance sweep

The measured quantity is a **vertical** foot force. A thigh is never vertical in
stance, so the force has to be resolved into the thigh's own frame, and how much
of it bends depends on the stance angle — which is a gait parameter nobody has
pinned down. Rather than pick one angle and hide the choice inside a number, the
angle is swept and the sensitivity is reported.

The first version applied the force along −z and bolted on a hand-computed
moment with an arm converted from metres to millimetres **twice**. That loaded
the part axially and then over-bent it by 1000×.

## Six things that had to be fixed first

Five of these produced a plausible number instead of an error.

1. **`*BOUNDARY nset, 1,2,3` is not valid syntax.** It is read as DOF 1–2 with
   value 3, so the constraint silently did nothing and the body translated by
   **1.3 × 10¹⁰ mm** while the solve still reported "Job finished". The correct
   form is `nset, firstdof, lastdof`. This is why `run_fea.py` now carries an
   equilibrium check — it fails loudly on this class of bug.
2. **Unit system.** The deck mixed mm lengths with E in Pa. CalculiX is
   unitless, so it must be internally consistent; the inconsistent deck reported
   a plausible displacement and stresses off by orders of magnitude. The decks
   are now **mm / N / MPa** throughout, enforced by `calculix_material_card`'s
   `stress_unit` argument.
3. **Point load on a single node.** Concentrating 92 N on one node put the peak
   *on that node* at 2.18 MPa, and the answer barely moved with stance angle —
   because it was measuring the singularity, not the part. The load is now spread
   uniformly over the 371 knee-face nodes, as a real joint reaction acts over a
   bearing area. Peak dropped to 0.41 MPa at stance 0 and began responding
   correctly to stance.
4. **`geom size` is half-extents, but gmsh's Abaqus writer emits full ones.**
   The mesh itself was correct; it was my boss-face selector that was wrong. It
   matched on a quarter-diagonal radius, which the boss **end caps** (44 × 0 × 44
   planes) score at exactly 22.0 — so the constraint would have been applied to
   two flat discs.
5. **Matching on centre-of-mass does not work for a boss.** The boss is
   booleaned against the web, so OCC trims it and its centroid lands at
   z = +11.8 (hip) rather than on the pivot. The selector now filters on surface
   **type** plus both cross-section extents plus the y-extent equal to the boss
   length.
6. **`CPS4` and `C3D4` both have 4 nodes.** Classifying gmsh's elements by node
   count merged the surface shell into the volume mesh. Classified by the card's
   TYPE instead — which matters because the boundary-condition faces arrive as an
   `*ELSET` of *surface* elements that then has to be mapped to nodes.

Plus two gmsh behaviours worth recording: the output format is chosen by the
**last** file extension, so `thigh.inp.mesh` silently produced MSH with no `*`
cards at all; and without `Mesh.SaveAll = 1` its Abaqus writer emits only elements
belonging to a physical group, so all 23k tetrahedra were dropped because the
only physical groups were the 2D face sets.

## Why "peak" is not the design number

The absolute maximum does **not converge**. Refining both size constraints:

| target mesh | elements | peak MPa | change | 99.9th pct MPa | change |
|---|---|---|---|---|---|
| 7.10 mm | 12 566 | 1.552 | — | 1.452 | — |
| 3.55 mm | 48 503 | 2.011 | +29.6% | 1.858 | +28.0% |
| 1.94 mm | 191 804 | 2.842 | +41.3% | 2.122 | +14.2% |
| 1.25 mm | 656 310 | 3.879 | +36.5% | 2.281 | +7.5% |
| 0.89 mm | 1 715 128 | 4.901 | +26.3% | **2.337** | **+2.4%** |

The max climbs monotonically; the 99.9th percentile flattens. That divergence is
the signature of a **singularity**, not of physics. Two are present:

- **The hip face is fully fixed.** A completely fixed face makes linear-elastic
  stress unbounded there — the peak sits at z = 0.00 mm, i.e. on the constrained
  boundary, at every mesh size.
- **C3D4 tetrahedra are stiff**, so they over-predict for a given node count.

So the honest reporting is: the converged *characteristic* interior stress is
**~1.9–2.3 MPa** (the 99.9th percentile, converged to 2.4% on the last refinement)
against a 276 MPa yield. The absolute peak is mesh-dependent and must not be
quoted.

To get a real design number you would need, in order of value:
1. **a fillet** at the boss/web junction and any other sharp re-entrant edge —
   which is also what a real machined part has, and which this CAD omits;
2. a **less artificial boundary condition** than a fully fixed face (a pinned
   boss with a remote coupling, or a proper joint representation);
3. **hexahedral or quadratic tets** instead of linear C3D4;
4. **plasticity**, so the singularity is cut off by yielding rather than running
   away. `mech/materials.py` records yield for exactly this and nothing consumes
   it yet.

## What this does and does not settle

**Does:** the mechanical phase has a working, validated toolchain — STEP →
gmsh → CalculiX → stress, with equilibrium verified. The thigh is comfortably
over-strength at the measured foot force. σ_leg's premise (a rigid 2R link) is
not contradicted by the link's own strength.

**Does not:** joint reactions are still not measured — the load case resolves a
*vertical* force into the thigh frame by stance angle, which is a model, not a
measurement. Shear and torsion are absent. The foot pad is still the r = 0.022
sphere (phase D). And the boundary singularity means this says nothing about
local stress at the bearing seats, which is where a real joint would fail first.