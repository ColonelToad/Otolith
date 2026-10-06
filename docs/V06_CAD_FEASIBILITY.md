# v0.6 feasibility: CAD + FEA, probed rather than assumed

Written 2026-10-05, before committing a phase to this. Every claim below was
produced by *running* the tool, not by checking that a package exists. The
three traps in particular are ones that would have survived a package-availability
survey and then cost days.

The question this answers: CAD + PCB were identified as bedrock for a possible
future project, so is there room here, and is anything blocking?

## Verdict

| half | status |
|---|---|
| **PCB** | **not ready, and not close** — nothing in this repo specifies a board |
| **CAD + FEA** | **ready to start**, behind 3 prerequisites below |

## What the probe established

**CAD (cadquery 2.8.0, headless, conda-forge):** builds solids and exports STEP
+ STL with no GUI. A scripted parametric link model is straightforward.

**FEA (CalculiX 2.23, headless, conda-forge):** solves, and was validated against
a closed-form cantilever — see `validation/` in this directory.

Tip deflection, 400x20x20 mm cantilever, 50 N tip load, 640 C3D8R elements:

```
tip deflection   FEA  -396.41 um   Euler-Bernoulli   400.00 um   err 0.90%
```

0.90% is the expected residual for reduced integration plus the Timoshenko shear
correction (~1.5% at L/h = 20), so the two partially cancel. The point is that
the solver is trustworthy to ~1%, which is the only thing that makes an FEA
result worth having.

Bending stress converges too, once the mesh is refined **through the thickness**:

```
   nx  ny  nz  elems  defl err%   stress ratio min/mean/max
   20   2   2      80       2.90   0.521 / 0.543 / 0.589
   40   4   4     640       0.90   0.767 / 0.786 / 0.821
   40   4   8    1280       2.67   0.878 / 0.900 / 0.941
   40   4  16    2560       3.21   0.936 / 0.960 / 1.004
   80   4  16    5120       0.74   0.948 / 0.959 / 0.981
   80   8  16   10240       0.78   0.946 / 0.957 / 0.979
  160   8  16   20480       0.02   0.946 / 0.952 / 0.962
```

Four things that are worth more than the numbers themselves, because three of
them contradicted what I expected:

- **nz (through the thickness) is the only refinement that matters for bending**:
  2 -> 16 elements across the section takes the stress ratio 0.54 -> 0.96.
- **nx (along the span) is already converged**: 40 -> 160 moves the ratio 0.960
  -> 0.952, nowhere. Don't spend elements lengthwise on a bending problem.
- **The floor is ~0.95, not 0.77.** I wrote earlier in this file that reduced
  integration reads 20-25% low and would need extrapolation or quadratic
  elements to fix. That was wrong — the deficit was an under-meshed thickness,
  not reduced integration's character. What remains is a genuine ~5% low offset
  and it is *constant* across the span, so it is correctable by a scalar.
- **Tip deflection is not a convergence indicator here.** It wanders
  0.02-3.21% non-monotonically (C3D8R hourglassing) while stress converges
  cleanly. Gate on stress; treat a smooth deflection as luck, not convergence.

Net rule for v0.6: **~16 elements through the thickness of any thin member,
and ignore the span.** `mech/validation/sweep.py` reproduces the table.

`mech/validation/frd.py` gates on the 0.95 offset being *constant* rather than
on its magnitude — a constant offset is correctable, a ratio that wanders across
stations means the mesh is wrong and nothing downstream of it is trustworthy.

## Four traps, all silent

1. **cadquery is millimetres; `leg_kin.cpp` is metres.** `.box(0.213, …)` gives
   a *valid* 0.213 mm part — 1000x too small, no error, no warning, correct
   topology. Confirmed: `.box(0.213)` -> `Volume() = 0.000307 mm³` vs
   `.box(213.0)` -> `0.306720 mm³`. Since the whole point is to drive CAD from
   `leg_kin.cpp`, **the geometry contract must carry units explicitly.** This is
   the same failure shape as the `sin_cos_wide` bug: silent, plausible, wrong by
   a factor.
2. **CalculiX `*BOUNDARY` takes an NSET, not an inline node list.** The Abaqus
   shorthand silently misparses (`unknown DOF: 206`). Declare `*NSET, NSET=FIXED`
   first.
3. **Clamping only the corners of a face turns a cantilever into a propped pin.**
   This one is the reason the closed-form check earned its keep: the solve was
   green, the stress field looked plausible, and the tip deflection came out
   **3.6x too large** (1429 um vs 400 um). Constrain the whole face.
4. **Linear hexes shear-lock in bending.** Use `C3D8R` (reduced integration) or
   `C3D8I`, not plain `C3D8`. Separately: generating a `C3D20` mesh from corner
   nodes only is degenerate — the mid-edge nodes collapse onto corners and
   CalculiX reports `nonpositive jacobian`.

Also worth knowing: `.frd` output packs values as contiguous 12-char fields with
no separator (a negative number is exactly 12 chars, which is why splitting on
whitespace corrupts them), and both nodal and element blocks are introduced by
` -1` — only the enclosing ` -4` name distinguishes them. `frd.py` here handles
both.

## Prerequisites before any CAD work

1. ~~**Resolve the `L2` provenance.**~~ **DONE 2026-10-05 — no bug, and my
   flag was wrong.** `L1 = 0.213` is the *calf joint frame* offset;
   `L2 = 0.21300938946440834` is the distance to the *foot contact sphere
   centre*. The MJCF's collision default adds `pos="-0.002 0 -0.213"`, so
   `L2 = hypot(0.002, 0.213)` — 9.4 um further out, exactly. The 17 digits are
   just `hypot()` on a rounded decimal; nothing was fitted. MuJoCo computes the
   bit-identical double from `norm(geom_pos[foot_geom])`, and both the C++ and
   Rust twins match it with relative difference 0.0. Correct as-is.

   The finding that *does* matter for CAD: **`L2` is a distance to a collision
   proxy sphere, not a vendor link dimension.** Leg odometry wants the contact
   point, which is what this is — the right constant for the estimator. But CAD
   must take link lengths from the visual meshes or a vendor drawing, or the
   2 mm collision offset gets baked into the link.
2. ~~**Log contact forces.**~~ **DONE 2026-10-05** — see
   `docs/V06_LOAD_CASES.md`. Note the OTLG was deliberately *not* extended:
   `LogRow` is 288 B and ADR-0005/0006 pin the transport bake-off to that size,
   so the forces go to a sidecar (`mech/out/loads.json`) rather than into the
   estimator's input contract. Forcing MuJoCo's contact solver to produce them
   failed three ways (kinematic puppet, forward-only constraint solver,
   `<motor>` actuators that ignore `ctrl`); the shipped method is exact CoM
   momentum balance instead, gated on conservation at 0.9987 W.
3. **Record material properties.** Density, Young's modulus, yield. Nothing in
   the repo. The MJCF's per-link masses (6.921 / 0.678 / 1.152 / 0.241352 kg,
   full inertia tensors) are a usable cross-check on any CAD — but see the
   correction below, because they do **not** agree with the published mass.

Then the phase's first real deliverable: the sim's calf collision is a
`0.1065 x 0.01225 x 0.017` m **box** — a 12 x 17 mm slab standing in for a
structural extrusion. Nothing mechanical in the sim is currently trustworthy.

## Correction: the mass budget is NOT already validated

An earlier draft of this file claimed the MJCF masses "already sum to ~12.4 kg
against the Go2's published ~12 kg — so the mass budget is the one piece of the
mechanical spec that is *already* validated." **That was wrong.** Recomputed from
the model: `body_mass[1:].sum()` = **15.2064 kg** (trunk 6.921 + 4 × 2.071),
which is ~27% above the published ~12 kg for a real Go2.

So the mass budget is *also* unvalidated, and it matters more than the other
prerequisites combined: it is the multiplier on every load case in
`docs/V06_LOAD_CASES.md`. If the robot is really 12 kg, every force there is
~25% too high. Recorded rather than quietly corrected because a feasibility
note that claims a spec is validated when it is not is worse than one that
admits the gap.

## Still unproven

- gmsh -> CalculiX meshing of an imported STL/STEP solid, with nodes mapped to
  the estimator's frame.
- Any material. FEA without a real alloy and yield is a shape exercise.
- Whether CAD output should flow *into* the MJCF, into `leg_kin.cpp`, or both.
  Two consumers with different units (MuJoCo is metres) is exactly where the
  mm/m trap above bites.

## Why the PCB half stays parked

There is no BOM, no MCU, no IMU part, no power budget, no rail and no interface
spec anywhere in this repo. That is not immaturity — it is an undecided product
question, and nothing in the stack justifies a custom board today: the estimator
runs at 500 Hz in C++/Rust and the RTL port is a study rather than a product. The
one framing that would make a board interesting is "carrier for the RTL core",
which would make the deferred option-B work a prerequisite instead of optional.