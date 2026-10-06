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

## The measurement

Controlled comparison: contact point matched to **0.00 µm**, width matched to
44 mm, `L2` = 0.213 in both scenes, and the calf-body attitude identical
(47.04°–64.19° in both, since `qpos` is prescribed). Only contact shape differs.

Foot position residual vs the commanded foothold, during stance, 3 s at 500 Hz:

| | sphere | pad |
|---|---|---|
| residual magnitude | 2.000 mm | 10.222 mm |
| magnitude swing | **0.000 mm** | **0.000 mm** |
| x direction swing | 0.52 mm | 2.50 mm |
| z direction swing | 0.36 mm | 2.11 mm |

### The answer: the foot pad does not explain σ_leg

The prediction was right. Both shapes hold their contact distance exactly —
0.000 mm swing over the whole stance — so a rigid pad contributes a constant,
which drops out of `r_dot`.

The pad's direction does swing, 2.50 mm against the sphere's 0.52 mm, so it is
not nothing. But 2.50 mm across a ~0.3 s stance is ~8 mm/s of apparent `r_dot`
against σ_leg = 0.3 m/s — **36× too small**. The foot is not the source.

What the pad does produce is a **bias**: 10.2 mm of constant offset against the
sphere's 2.0 mm. A constant is harmless to the filter's rate of change and
noticeable in absolute position, which is worth knowing but is not σ_leg.

So the candidates left for σ_leg are the ones that actually *vary*: encoder
noise (already modelled in-sim), joint-bearing compliance, and terrain.

## Two corrections, both mine, and both instructive

This section previously reported a 17.7 mm direction swing, 50% of stance
samples past 90° tilt, and concluded that the pad **does** explain σ_leg —
explicitly overruling my own prediction. Both numbers were bugs. Neither was a
property of the foot.

**1. The scene only placed one of the four feet correctly.** I set `pos` on `FL`
by patching that one line, and the other three inherited the collision class's
`-0.213` — so three feet sat 20 mm higher than the fourth. Every measurement
taken before that was fixed was measuring a robot standing on one foot and
three stumps. `build_pad_scene()` now writes all four from one solved value.

**2. "Tilt" was the mesh's mounting rotation, not the foot's attitude.** I
read `geom_xmat` on the foot geom. For a mesh geom that matrix includes the
rotation MuJoCo applies when framing the mesh, so it reported 68.5°–111.4° and
50% of samples past horizontal — a model apparently standing on the edge of its
foot. Measuring the **calf body** frame instead gives 47.04°–64.19° in both
scenes, identical, which is the only possible answer because `qpos` is
prescribed and identical (`max|Δq| = 0.0`).

Along the way I also chased the mount backwards: `mesh_vert` showed the pad's
39 mm axis on `x` and I concluded the STL was being loaded sideways. That was
the asset frame again. The effective geometry (`geom_xmat @ (mesh_vert -
mesh_pos)`) has the flat 32×32 face pointing −z and extent 44×44×39 — correct
as built, with no geom `quat` needed.

The pattern is worth stating, because it is the same one three times in this
phase: I reached a surprising conclusion, it disagreed with my prediction, and
the disagreement turned out to be a measurement bug rather than new physics. The
tell each time was the same — **a surprising result that only I could see,
because I had built the thing being measured.**

### GRF is structurally blind to contact geometry

The obvious second measurement — compare ground reaction forces between the two
scenes — returns **bit-identical numbers**, every delta exactly 0.00000 N:
mean 148.15345 N, peak 184.69676 N (1.23812 × weight), per-foot peak 92.3484 /
92.3290 / 92.3290 / 92.3484 N, duty 54.984%, in both scenes.

That is not a bug and not a null result, it is how the method is built.
`com_height_series()` runs `mj_kinematics` + `mj_comPos` on the puppet's
prescribed `qpos` and never calls `mj_forward`, so it never resolves a contact.
GRF is a function of the *prescribed kinematics* alone.

Two consequences, pointing opposite ways:

- **Good for FEA.** The load cases driving the thigh analysis are kinematic and
  geometry-independent, so a future CAD export cannot perturb them. That is worth
  more than it looks: it is the property the `L2` coupling lacked.
- **Useless for validating contact.** GRF cannot discriminate a sphere from a
  pad, because it never looks. Only the foot residual discriminates them, which
  is why the residual measurement above is the whole of the empirical result and
  the GRF comparison contributes nothing to it.

So `record_grf` is the right instrument for the FEA load case and the wrong
instrument for this question, and no amount of runtime would have changed that.

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
