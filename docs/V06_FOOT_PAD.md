# v0.6 phase D: the foot pad, and a coupling that blocks phase E

2026-10-05. The foot was promoted from "follow-on" to "the main event" once the
thigh FEA ruled out structural causes for σ_leg. It did not, quite — it found
something else, and it found it by failing.

`mech/links/foot.py` · pad scene in `.work/padscene/` (gitignored)

## The prediction being tested

The foot's collision is a single **r = 22 mm sphere**. A sphere has a property
that is easy to miss: its lowest point is always directly below its centre, so
in the **foot's own frame** the contact point is fixed at `(0, 0, −r)` no matter
how the foot tilts. Nothing migrates, ever.

A real foot pad is a **flat contact face with a filleted edge**, and that changes
the mechanism:

* below `atan(r_edge / r_face)` the whole face is down — contact centroid at the
  face centre, offset 0, same as the sphere;
* above it, the face touches along **one edge line** — contact at radius
  `r_face` from the pad axis, a **constant** offset, not a migrating point.

So a rigid pad contributes a *step* at the edge angle and then nothing further.
A constant offset in `r_base` drops out of its derivative, so **a rigid pad
cannot generate sustained σ_leg noise** — only a one-time transient at
touchdown.

Measured foot tilt in the sim's own gait: **47.8°–63.4° through stance**, against
an edge-roll threshold of 20.6°. The pad is therefore permanently edge-down, so
in this gait the offset is constant for the whole stance.

That is a falsifiable prediction, and it points at the next thing to measure: a
rigid pad gives a constant, so whatever produces σ_leg has to **vary**. Which
leaves contact *compliance* — and the sim already models that separately
(`solimp="0.015 1 0.022"`, `friction="0.8 0.02 0.01"`, `condim="6"`), so foot
slip was never unmodelled either.

## The pad

`mech/links/foot.py`, a revolved profile sized to the **same 44 mm width** as
the sphere it replaces, so the stance geometry is a controlled comparison:

| | |
|---|---|
| width | 44.0 mm (matches the r=22 sphere) |
| height | 39.0 mm (visual foot envelope is 38.8 mm) |
| contact face | ⌀32 mm flat |
| edge radius | 6.0 mm → 20.6° edge-roll threshold |
| valid / solids / shells | True / 1 / 1 |

## Two export bugs, both silent

1. **MuJoCo applies no scale to mesh coordinates.** A 29 mm pad exported
   straight from cadquery became a **29 metre** pad, and `geom_pos` came back as
   8.058 m. The scale belongs in the scene as `<mesh scale="0.001">`, in one
   visible place, not in a post-processing step.
2. **A unit error in my own helper**: `contact_face_below_centroid_mm()`
   multiplied by 1000 and reported a 10 271 mm offset for a 10.27 mm pad. Same
   double-conversion that produced a 19 671 N·m knee moment in the thigh load
   case. Third time.

## What actually blocks phase E

Installing the pad broke the puppet immediately:

```
ValueError: FL: unreachable foot target [0.2319 0.0465 0.0]
```

Because `puppet._leg_geoms()` derives the estimator's leg length as

```python
L2 = float(np.linalg.norm(model.geom_pos[foot_geom_id]))
```

— the **placement of the contact geom**. Moving only that geom, changing nothing
else, moves `L2` one-for-one:

| foot geom pos z (m) | L1 (m) | L2 (m) | ΔL2 |
|---|---|---|---|
| −0.2130 (baseline) | 0.2130 | 0.213009389 | — |
| −0.2180 | 0.2130 | 0.218009174 | +5 000 µm |
| −0.2230 | 0.2130 | 0.223008968 | +10 000 µm |
| −0.2030 | 0.2130 | 0.203009852 | −10 000 µm |

`L1` is unaffected because it comes from the body chain. `L2` tracks the
collision proxy's *placement* exactly.

**This is the finding that matters for the plan.** The contact proxy and the
kinematic link length are the same number, which is why they must never be
changed together — and it means:

- A strictly shape-only comparison is **impossible** without decoupling: placing
  the pad so its contact face coincides with the sphere's contact point puts the
  geom at `z = −0.223024`, implying **L2 = 0.223033 m**, a 10 mm change to the
  leg length.
- **Phase E as originally planned is actively dangerous.** "Export the CAD back
  as collision geoms" would silently change `L2`, and therefore every estimator
  result in the repo, with no error anywhere. That is the same failure class as
  everything else in this phase: a plausible-looking change that quietly moves
  something load-bearing.

Note this is *not* the `L2` question from the geometry contract, which is
resolved and correct. That was "is `L2 = 0.21300938946440834` the right number?"
(yes — `hypot(0.002, 0.213)`). This is a different question: **what should the
puppet read `L2` from?** It currently reads a contact proxy's placement, when
it should read the kinematic chain — the foot joint offset, which the URDF gives
directly and which is independent of how the contact is modelled.

## What this says the artifact must do better

This is the question the framing was aiming at, and the answer is concrete:

1. **The mechanical model and the contact model must be separate files.** One
   currently supplies the other's link length. A CAD export cannot be trusted
   until that is undone.
2. **The contact model needs its own spec**, separate from the kinematic
   constants: pad width, contact face, edge radius, friction, and stiffness —
   with `contact_point_shift_mm()` documenting that a rigid pad gives a constant
   offset, not a migrating contact.
3. **σ_leg needs a different explanation than the contact shape.** A rigid pad
   gives a constant; the sim already has μ=0.8 and compliant contact; and link
   flex is ≤9 µm against ~500 µm of foot travel. The candidates left are
   encoder noise (already in-sim), joint bearing compliance, and terrain.

## The measurement, and a prediction it contradicts

With the decoupling in place the comparison is finally controlled. Contact
geometry is placed so the pad's lowest point meets the sphere's to **0.0 µm**,
the pad is 44 mm wide against the r=22 mm sphere's 44 mm diameter, and `L2` is
0.213 in both scenes. Only the contact *shape* differs.

Foot position residual vs the commanded foothold, during stance, 3 s at 500 Hz:

| | sphere | pad |
|---|---|---|
| residual magnitude | 2.000 mm | 10.2 mm |
| magnitude swing | **0.000 mm** | **0.047 mm** |
| x direction swing | 0.52 mm | **17.70 mm** |
| z direction swing | 0.36 mm | **13.62 mm** |
| foot tilt | 47.0°–64.2° | **68.5°–111.4°** |
| stance samples past 90° tilt | 0.00% | **50.00%** |

**The magnitude half of my prediction was right.** A rigid pad's contact
distance is constant to 0.047 mm out of 10.2 mm, and it fits
`|res| = 10.240 − 0.0006·tilt` with only 0.017 mm unexplained — so above the
edge-roll threshold the offset genuinely stops growing, exactly as the geometry
said.

**The conclusion I drew from it was wrong.** I wrote that a constant offset
"drops out of `r_dot`, so a rigid pad cannot produce sustained σ_leg." That
assumes the offset is constant *in the world frame*. It is constant in
*magnitude* but its **direction rotates with the foot's roll**: 17.7 mm of
x-swing while the contact distance barely moves. Differentiate that and you get
real apparent foot velocity — roughly 17.7 mm over a ~0.3 s stance is ~60 mm/s,
against σ_leg = 0.3 m/s. So a rigid pad **is** a viable explanation for σ_leg,
and the shape argument that seemed to rule it out was resting on the wrong
invariant.

The mechanism: the pad contacts along an **edge**, and which point of that edge
touches depends on the foot's roll direction, which swings through 43° of tilt
across the stance. The sphere has no such degree of freedom — it contacts along
a normal, always.

### The 111° result is its own finding

Half of all stance-foot samples sit past 90° of tilt. That is not a robot
stance; it is the model standing on the edge of its foot. The Menagerie gait
was tuned around the sphere proxy and has never been checked against the real
foot shape, so **the vendor's foot visual mesh cannot be dropped in as
collision geometry** — it changes the stance qualitatively, not quantitatively.

This is the artifact finding, and it is a stronger version of the rule the rest
of v0.6 keeps re-learning: a collision proxy is not a contact model, and a
visual mesh is not a validated contact model either.

## Closed: the decoupling

The fix is one line of intent. `L2` now comes from the kinematic chain
(`|FL_calf_joint.origin.z|` = 0.213) rather than from the foot collision geom:

- `sim/otolith_sim/puppet.py` — `_leg_geoms()`
- `fusion/src/leg_kin.cpp` — `leg_geom()`
- `rust/otolith-fusion/src/leg.rs` — `leg_geom()`
- `mech/spec/build_spec.py` → `mech/spec/go2_urdf.json` (regenerated)
- `mech/links/mesh_thigh.py` — `L2_M`

Verified invariant: moving the foot contact geom by ±10 m now leaves `L2` at
0.213 exactly, where before it tracked 1:1.

### Cost of the decoupling

M5 gate re-measured on the same 10 s log, 500 Hz:

| | pos RMSE | vel RMSE | att RMSE | saturations |
|---|---|---|---|---|
| float | 0.3334 m | 0.0946 m/s | 24.21 deg | — |
| **M5 fully fixed** | **0.3072 m** | **0.0934 m/s** | **22.18 deg** | **0** |

Previously 0.3074 / 0.0934 / 22.19. Position moved 0.2 mm in 307 mm — 0.07% —
and velocity and attitude are unchanged at the digits reported. The float row
is bit-identical to the original run, which is the part worth having: the
decoupling perturbed nothing the filter notices while removing the coupling
that made phase E unsafe.

### One thing the decoupling cost

The Rust differential (`fusion.rs differential_vs_cpp_golden`) failed, correctly,
at its 1e-9 tolerance — 27 hardcoded f64 literals derived from a now-changed
`L2`. Its doc comment said the goldens came from "`/tmp` golden.cpp", and that
file was gone, so regenerating meant hand-editing every literal.

That is fixed rather than worked around: `fusion/src/gen_golden.cpp` is now a
checked-in target (`./fusion/build/gen_golden`) that reprints the goldens from
the C++ filter at 17 digits. It reproduced `updates = 78` and agreed with the
Rust port to ~1e-16 on the way out, which is how I knew it was faithful rather
than merely producing numbers that made the test pass. It has two bugs of its
own from the first draft — calling `update_legs` once per leg instead of once
per step, and computing `trace` from a zero-size Eigen block — both of which
would have produced plausible-but-wrong goldens.

### Still not done

The controlled pad comparison itself. The sphere-vs-pad run is now unblocked,
but it has not been executed; `.work/padscene/` holds the prepared scene. Worth
measuring when it runs: whether a rigid pad changes anything *time-varying* at
all, which the analysis says it should not.
