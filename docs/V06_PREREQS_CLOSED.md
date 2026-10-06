# v0.6 prerequisites: all three resolved

Written 2026-10-05. This closes all three prerequisites from
`docs/V06_CAD_FEASIBILITY.md`. Two of them changed what the project believes;
one of those two is a correction of something I had claimed earlier, which is
recorded rather than quietly overwritten.

| # | prerequisite | outcome |
|---|---|---|
| 1 | resolve the `L2` provenance | **no bug in the value — but read from the wrong file.** Provenance was right, conclusion was wrong; corrected below. |
| 2 | log contact forces | **done**, by an exact method after MuJoCo's failed three ways → `docs/V06_LOAD_CASES.md` |
| 3 | record material properties | **done** → `mech/materials.py`, wired into the FEA harness and validated |

A fourth item surfaced while doing (3) and is *not* closed: the mass budget.
It is now quantified rather than ignored, and it is the largest remaining
uncertainty on every force in the repo.

## 1. `L2` — the flag was wrong

`fusion/src/leg_kin.cpp` uses `L2 = 0.21300938946440834` where the MJCF's
calf joint shows `0.213`, and I recorded that as an unexplained 9.4 µm gap and
speculated it was an IK fit.

**Both halves of that were wrong.**

```
L1 = |calf joint frame in thigh|         = 0.213                <- the XML's "0 0 -0.213"
L2 = |foot contact sphere centre in calf| = 0.21300938946440834 <- a DIFFERENT quantity
```

The MJCF's collision default applies `pos="-0.002 0 -0.213"`, so
`L2 = hypot(0.002, 0.213)` — exactly 9.4 µm further out, which is what the
constant is. The 17 significant digits are just `hypot()` on a rounded
decimal. Verified: MuJoCo computes the **bit-identical** double from
`norm(geom_pos[foot_geom])`, and the C++ and Rust twins both match it with
relative difference exactly `0.0`. Nothing was fitted; nothing is wrong. The
constant is unchanged, and a test now gates it.

The finding that *does* carry into CAD: **`L2` is a distance to a collision
proxy sphere, not a vendor link dimension.** Leg odometry wants the contact
point, which is what this is, so it is the correct constant for the estimator.
But CAD must take link lengths from the visual meshes or a vendor drawing, or
the 2 mm collision offset would be baked into the link. Two consumers, two
different right answers — which is exactly why "it disagrees with the XML"
looked like a bug and wasn't.

## 2. Contact forces

Full detail in `docs/V06_LOAD_CASES.md`. Summary: MuJoCo's contact solver is
unusable here (kinematic puppet → no forces exist; forward-only constraint
solver → free fall still reports 160 N; `<motor>` actuators ignore `ctrl`), and
the mean-GRF-must-equal-weight check rejects the dynamic rollout at 0.88 W. The
shipped method is exact CoM momentum balance, gated on conservation at
**0.998661 W**, with peak 1.238 W matching the analytic bob prediction to 1%.

## 3. Materials

`mech/materials.py` carries Al 6061-T6 (default), Al 7075-T6, Steel 1018 and ABS
with `E`, `nu`, density, yield and ultimate. Provenance is stated in the module
docstring and it matters: these are **handbook values for the named temper, not
vendor datasheets**. Right for a first FEA pass, wrong for certifying anything.
Swapping in a supplier datasheet is a diff, which is why the table is in git.

Wired into `mech/validation/cantilever.py` via `--material`, which was the
point — the harness had validated a **steel** beam (E = 200 GPa) for a robot
that is machined aluminium. The closed form is linear in 1/E, so re-running it
per material is a real check of the material path:

| material | E (GPa) | FEA tip deflection | Euler-Bernoulli | error |
|---|---|---|---|---|
| Al 6061-T6 | 68.9 | 1151.24 µm | 1161.10 µm | **0.85%** |
| Al 7075-T6 | 71.7 | 1106.28 µm | 1115.76 µm | **0.85%** |
| Steel 1018 | 200 | 397.17 µm | 400.00 µm | **0.71%** |

`yield_pa` and `ultimate_pa` are carried but **consumed by no analysis yet** —
CalculiX `*ELASTIC` takes only E and ν. A test asserts the emitted card cannot
silently grow a third value, because CalculiX would read it as an expansion
coefficient and accept it without complaint.

### The volume budget, and how weak it is

With a material, the declared masses become checkable: `mass / rho` gives the
solid volume each link should occupy, which can be compared against the volume
of its collision primitive.

| part | mass kg | required cm³ (Al) | collision cm³ | implied density | required/collision |
|---|---|---|---|---|---|
| hip | 0.678 | 251.1 | 265.9 | **2550** | 0.94 |
| calf | 0.241 | 89.4 | 79.0 | **3056** | 1.13 |
| thigh | 1.152 | 426.7 | *needs CAD* | — | — |

Both checkable links imply **2550 and 3056 kg/m³ against 2700 for aluminium** —
agreement within 6–13%. So the declared masses are not arbitrary; they are
consistent with machined aluminium parts.

This is a **weak check and is labelled as one in the code and the test.** The
collision geoms are cylinders and boxes standing in for machined parts, so ~25%
agreement is the best available, and agreement means *not obviously wrong*,
never *validated*. The thigh is absent because its collision geom is a **mesh**,
and a primitive volume cannot bound it — which is the clearest single argument
for replacing the collision geometry with real CAD.

## 4. Not closed: the mass budget

The model totals **15.2064 kg** against a published Go2 figure of ~12.4 kg:
**+22.6%**. That is a real, quantified gap and it scales everything:

> every load case in `docs/V06_LOAD_CASES.md` would be **1.2263× too high** if
> the robot is really the published mass.

Both facts from §3 are true simultaneously and they are different questions:

- the masses are *internally plausible* for aluminium (2550–3056 kg/m³);
- the model is *22.6% heavier* than the published spec.

So the plausible reading is that this is a heavier variant, or that the
published figure counts something the model does not. I could not settle it
here, and I want to be explicit that I tried and failed: I attempted to
cross-check each link's declared mass against its **visual mesh** volume
(extracted from MuJoCo's own `mesh_vert`/`mesh_face` arrays) and the attempt
was confounded by at least one non-watertight mesh — mesh 4 reports 6267 cm³,
which is a divergence-theorem artefact, not a part. Without per-link vendor
geometry there is no clean way to close this.

**Update (2026-10-05): resolved, and the resolution is that it does not
matter.** See `docs/V06_MASS_BUDGET.md`. Convex-hull volume replaces the failed
divergence-theorem cross-check and is bounded even for non-watertight meshes; it
shows no link is over-mass for its own envelope, but being a lower bound on
density it cannot see the +22.6%-over-published direction, which is the one in
question. So it does not settle the number.

It settles the *consequence*: the thigh's ~150× stress margin survives ±22.6%
(which moves it to 120–154×), and the σ_leg conclusions are mass-independent
because they are measured ratios. Pin the model's 15.2064 kg — self-consistent
with the runs that produced the recorded numbers, and the conservative direction
for consuming margin — record the published figure and its 1.2263× multiplier,
and defer to the CAD phase. A vendor per-link mass table would settle it outright
and is cheaper than that CAD work.

## Also corrected

- The **mass budget**: see the update above and `docs/V06_MASS_BUDGET.md`.

The feasibility note had claimed the mass budget was "already validated"
because the masses "sum to ~12.4 kg against the published ~12 kg". They sum to
15.2064 kg. The note now carries a correction section rather than the original
claim.

Separately, a claim I made in passing while looking at collision geometry — that
"the thigh has no collision geom at all" — was **wrong**, caught by re-checking:
the thigh *does* have one, a mesh. My filter had selected only box and cylinder
primitives. The calf is the genuinely crude one (cylinders plus a
106.5 × 12.25 × 17 mm slab).

## Gates

`pixi run python -m pytest sim/tests eval/tests mech/tests -q` → **32 passed**
(was 22). New: 9 material/volume-budget tests, plus the `L2` regression gate in
`sim/tests/test_grf.py`. `ctest` unchanged at 24/24 — this phase touches no
fusion code beyond a comment.

`mech/tests` is a new pytest root; `CLAUDE.md` and ADR-0004 updated.

## Still open for FEA to actually run

- Per-link **vendor geometry** (or CAD built to a chosen spec) — blocks both the
  thigh volume check and any real stress number.
- **Shear and joint reactions** — the load cases are world-+z only.
- **Plasticity** — yield is recorded but unused, so there is no allow/ultimate
  verdict available yet, only elastic stress.
- gmsh → CalculiX meshing of imported geometry is still unproven.
---

## Correction (2026-10-05): the flag was wrong, the conclusion was too

This section recorded the `L2` provenance as **"no bug — nothing is wrong, the
constant is unchanged."** The provenance analysis below is correct and still
stands: `L2 = hypot(0.002, 0.213)` is exactly the distance to the foot contact
sphere, the 17 digits are just `hypot()` on rounded decimals, and nothing was
fitted.

The *conclusion* was wrong, because it missed where the number came from.

`leg_kin` did not hold `0.21300938946440834` as a constant. It computed it,
at runtime, as

```cpp
L2 = |geom_pos[foot_geom]|      // read live off the collision proxy
```

and the C++ and Rust twins then hardcoded whatever that produced. So the fact
that the value was "correct" was incidental — the mechanism was reading a
collision proxy's *placement* and calling it a link length. Measured: moving
only that geom's position moves `L2` one-for-one, to within 1e-9 over ±10 mm,
while `L1` stays put because it comes from the body chain.

Which means:

- "Two consumers, two different right answers" was the wrong framing. There is
  one consumer, and it was reading the wrong file. A collision proxy is not a
  source of kinematic truth — the same rule this whole contract applies
  everywhere else, and the reason error (4) above exists.
- The test added at the time gated `L2 == 0.21300938946440834` **and asserted
  `L2 != 0.213`**, explicitly forbidding the fix. It would have failed the
  moment the coupling was removed. A gate that locks in a mechanism you have
  just decided is wrong is worse than no gate, because it looks like coverage.

`L2` is now `0.213`, taken from the kinematic chain, across Python, C++, Rust,
the geometry contract, and the FEA. Full detail and the measurements in
`docs/V06_FOOT_PAD.md`.

The general lesson, which is the fourth instance of it in this phase: a
provenance question ("where does this number come from?") is not the same as a
correctness question ("should anything read this number here?"). Answering the
first one well is what made the second one visible at all.
