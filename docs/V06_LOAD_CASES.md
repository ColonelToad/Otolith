# v0.6 load cases: contact forces for the leg FEA

Written 2026-10-05. This closes prerequisite (2) of `docs/V06_CAD_FEASIBILITY.md`:
the OTLG log recorded contacts as binary flags, so FEA had no load cases. It now
does — from an exact method, after the obvious one failed three separate ways.

`sim/otolith_sim/grf.py` (the method) · `mech/emit_loads.py` (the emitter) ·
`sim/tests/test_grf.py` (the gates)

## Headline

For the 15.2064 kg model at a 0.7 s trot cycle, duty 0.55:

| quantity | value |
|---|---|
| weight | 149.175 N |
| mean total GRF | **0.998661 W** (conservation gate: 1.0) |
| peak total GRF | **184.70 N = 1.238 W** |
| rms total GRF | 151.27 N |
| measured duty factor | 0.5444 (configured 0.55) |
| per-foot peak normal force | FL 92.35, FR 92.33, RL 92.33, RR 92.35 N |
| per-stance vertical impulse | 24.7–25.7 N·s mean, 21.8–26.1 N·s spread |

Per-stance impulse is the fatigue input, not the peak — a life estimate needs
the distribution, and the spread (sd 0.6 to 2.0 N·s, differing between the two
diagonal pairs) is exactly what a peak-only case would throw away.

## Why not MuJoCo's contact solver

This was tried first because it is the obvious source. It does not work here,
for three independent reasons — all measured:

1. **There are no forces to read.** The puppet is kinematic by construction:
   `puppet.py` prescribes `qpos` from a foothold schedule plus leg IK, sets
   `qvel = 0`, and calls `mj_forward`. So it is a prescribed path, not a
   simulation, and `data.cfrc_ext` is identically zero.

2. **MuJoCo's constraint solver does not invert.** With `mjENBL_FWDINV` set and
   a prescribed `qacc` of **+g** (free fall), the static home pose still
   reported **160.68 N** instead of 0. `FWDINV` changes how `qacc` is derived
   from the solved forces; it does not solve for a different contact set. Also
   tried the explicit `mj_fwdPosition → mj_velocity → mj_inverse →
   mj_fwdConstraint → mj_invConstraint` sequence: still 160.68 N at free fall.

3. **The model has no usable actuators.** The Menagerie Go2 declares `<motor>`
   actuators with `biastype = mjBIAS_NONE`, `gainprm[0] = 1`, `biasprm = 0`.
   Actuator force is therefore `gainprm[0] * length` = **the joint angle in
   radians**, and `ctrl` is ignored entirely. A dynamic rollout of the
   prescribed trajectory gives mean GRF 0.66 W with the base sinking ~10 mm/s.
   Forcing a correct affine-bias position servo
   (`biastype = mjBIAS_AFFINE`, `biasprm = [0, -kp, -kv]`, with `forcerange`
   from the model's own torque limits) fixed the height drift to −0.21 mm and
   lifted mean GRF to **0.88 W** — but the result was **bit-identical across a
   100× gain sweep** (kp 20 → 2000), so the commanded torque still is not
   reaching the plant.

### The check that settles it

A rigid body at constant height moving at constant speed has zero net vertical
acceleration, so its ground reaction must equal its weight:

```
sum(F_z) == mass * g
```

The rollout gives 0.88 W. It is not a valid load case, and handing 0.88 W of
pretend load to an FEA would be fabricating data. Same shape as M4's ABC wall:
measured, reported, not worked around.

## What works instead, and is exact

For prescribed kinematics, Newton's second law needs no plant model at all:

```
F_z_total(t) = mass * (g + a_com_z(t))
```

`a_com_z` comes from FK on the prescribed joint angles, the CoM from the link
masses, and two derivatives. No actuator model, no contact solver, no penalty
stiffness, no penetration depth to tune. And because every link's mass enters
the CoM, the swing legs' inertial contribution is captured automatically —
which matters here, because **the legs are 8.29 kg of the 15.21 kg total
(54%)**. A crude `m * (g + a_base)` would discard over half the inertia.

### Smoothing is mandatory, and why

The swing arc is `step_height * sin(pi * s)`, which is **not tangent to the
ground at touchdown**: the foot arrives with a finite vertical velocity that is
instantly clamped to zero when stance starts. The prescribed CoM height is
therefore only C0, with a velocity kink at each footfall, and its true second
derivative there contains a delta. Differentiating twice amplifies that as
1/h². Sampling off the dt grid at `t ± h` with `h = dt/10` made it far worse —
**1808 N, 12.1 W** — because those samples land at arbitrary phases.

The fix is a local quadratic least-squares fit (Savitzky-Golay) over the dt-grid
series, taking the fit's second derivative. Window sweep, 18 to 162 steps:

| half-window (steps) | window (s) | mean GRF / W | peak GRF / W |
|---|---|---|---|
| 9 | 0.036 | 0.999344 | 1.244 |
| 17 | 0.068 | 0.999221 | 1.239 |
| 25 | 0.100 | 0.999164 | 1.231 |
| 41 | 0.164 | 0.999239 | 1.208 |
| 81 | 0.324 | 0.999849 | 1.131 |

Two independent checks agree that this is right:

- **Conservation**: mean GRF/W stays in 0.9992–0.9998 across a 9× range of
  window sizes. It is not sensitive to the arbitrary window.
- **Amplitude**: the CoM bobs ±7.8 mm at 2.86 Hz, so `a_peak = A(2πf)²` predicts
  a peak of **1.26 W**. The smoothed answer is **1.238 W**. An unsmoothed
  central difference gives 2.11 W — that is the kinks, not the physics.

Both are gates in `sim/tests/test_grf.py`, along with no-negative-force,
no-load-on-swing-feet, and the diagonal-pair symmetry.

## The one assumption, stated

The per-foot split is **equal across stance feet**, justified because this is a
diagonal trot so FL+RR and FR+RL are mirror pairs. It shows up in the data as
FL and RR peaks agreeing to 0.01 N (92.35 / 92.35). It is exact for these pairs
and is *not* a general result — a gait with asymmetric loading, a turning gait,
or a lateral load would need the distribution solved from the stance geometry
instead. Recorded in the emitted JSON's `provenance.assumption`.

## Why this is a sidecar, not a `LogRow` field

`LogRow` is 288 B, and ADR-0005 pins the transport bake-off to that size (as
does `fusion/bench/common.hpp` via `static_assert`, ADR-0006's Rust `BenchMsg`,
and `rust/otolith-transport`'s hardcoded `[u8; 272]`). Contact force is a
mechanical-analysis channel that **no estimator reads**. Growing the estimator's
input contract to carry it would invalidate two Accepted ADRs' measurements for
no consumer. So `mech/emit_loads.py` writes `mech/out/loads.json` instead.

Worth noting what extending `LogRow` *would* have cost, since it was the other
option: a version bump plus edits to `logger.py` (fmt, pack, unpack, the
import-time `calcsize` asserts), `log.hpp` (struct + `static_assert`),
`log.cpp`, `test_log.cpp`'s eleven `offsetof` assertions, `tb_predict.cpp`
(hand-rolled, hardcoded `288`/`32`/eleven field lengths, **no `static_assert` —
it would have failed silently inside the DUT**), Rust `log.rs` (including a
`debug_assert_eq!(o, ROW_BYTES)`), Rust `transport/lib.rs`, `ring.rs`,
`iox2.rs`, `fusion/bench/common.hpp`, `iox2/src/main.rs`, and six Python
fixtures. The `tb_predict.cpp` case is the argument against touching it.

## Still open

- **Shear and joint reactions.** This is world +z only. A link FEA also wants
  the tangential (friction) force and the per-joint reaction distribution.
  Friction is derivable the same way from the horizontal momentum balance, but
  the per-joint split needs the stance-geometry solve that the symmetry
  assumption currently bypasses.
- **Dynamic FEA run.** The series is emitted so a transient run can be driven by
  the real signal; only static/quasi-static use has been exercised.
- **The material.** No alloy, no yield. Forces without a material are not a
  stress.
- **Geometry.** Still prerequisite (1) — the 9.4 µm `L2` provenance gap in
  `fusion/src/leg_kin.cpp` — and the calf is still the MJCF's
  `0.1065 × 0.01225 × 0.017` m collision box, so the *contact patch* these
  forces act on is a slab, not a foot.