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

## Next

Not "install the pad anyway." The prerequisite is a one-line change in
`puppet._leg_geoms()` to take `L2` from the kinematic chain rather than the
contact geom — after which the pad comparison is controlled, and phase E becomes
safe. That change moves `L2` by 9.4 µm (the contact-sphere offset), so it is
small for the estimator but it touches every result in the repo, which is why it
is called out here rather than made unilaterally.