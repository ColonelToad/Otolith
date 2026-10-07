# v0.5: humanoid generalization

Status: **P0, P1, P3, P4 complete; P2 wired but G1 attitude diverges; P6 COMPLETE** (descriptor, C++ spec, puppet, stationarity gate, sigma_leg). P5 skipped by decision. Apollo puppet + C++ spec not started.

The phase's purpose is falsification, not a port. Its thesis was that per-robot
constants are where the wrong assumptions live, on the evidence that `sin_cos_wide`
was a real per-robot bug. Three targets were identified; results below.

## What decided the scope

G1 is 33.3 kg, 2 legs, 6 DoF per leg, 12 leg DoF (the same *width* as Go2).

**The planar 2R leg model cannot survive a biped.** A biped must shift its CoM
over the stance foot: 0.1165 m of lateral travel over a 0.719 m leg is a 9.2 deg
tilt, and a planar FK ignoring it misplaces the foot by **115 mm, 0.4x the entire
sigma_leg budget of 0.3 m/s**. That is the whole reason this phase exists.

## P0 — descriptor and FK (done, `80b767f`)

`sim/otolith_sim/leg_model.py`, `mech/spec/build_g1_scene.py`,
`mech/tests/test_g1_leg.py` (7 gates).

- G1 ships with **no named geoms at all**; the scene patch names the eight
  contact geoms (four per foot).
- The foot is **four r=5 mm spheres**, coplanar at z=-0.030, a flat 170 x 60 mm
  sole. (Reported as capsules at first — wrong; `MJ_GEOM_CAPSULE` is 3 and type
  2 is a sphere.)
- The sole reference point is the **mean of the four sphere centres**,
  (0.035, 0, -0.030): a property of the foot, not of the current contact set.
  Explicitly not the centroid of whichever spheres touch -- that moves as
  contact breaks and re-forms, which is the v0.6 foot-pad lesson applied.
- FK is **bit-exact** against MuJoCo over 3000 random legal poses.

Two bugs the gates caught: omitting the fixed `body_quat` (G1's `hip_roll_link`
is tilted 10 deg, `knee_link` -10 deg; omitting them gave 53 mm at q=0), and
assuming the chain order was a convention when it is the tree topology -- G1 runs
`pelvis -> hip_pitch -> hip_roll`, so hip_pitch is **first**.

**T4 settled analytically:** G1's worst 2R-equivalent joint sum is 310 deg =
5.41 rad against `sin_cos_wide`'s 3*pi, so the Go2 CORDIC fix already covers it.
Gated, not assumed.

## P1 — the C++ seam (done, `50454c4`)

`robot_spec(name)` returns a per-robot descriptor; `foot_pos_base` is the seam,
with `Planar2R` and `Chain` behind one interface.

Two models rather than one path, on measured grounds: Go2's planar form carries
**exactly 2.000 mm** of position error at every pose -- the foot sphere's 2 mm
lateral offset in the calf frame, which the planar form cannot represent because
it places the foot directly below the calf joint. Constant magnitude, direction
rotating with the leg, so it cancels in the finite difference the filter forms:
**r_dot error 0.0018 m/s, 0.01x sigma_leg**. The chain would buy nothing for
Go2 and cost the float/fixed bit-parity that `test_fixed.cpp` asserts.

**Gate: Go2 bit-identical.** float 0.3334 / 0.0946 / 24.2088, fully fixed
0.3072 / 0.0934 / 22.1798 -- both exactly the recorded M5 numbers. 25/25 ctest.

The header had claimed "validated against MuJoCo FK to <2mm (see test)" where
the test asserted a z-range and a second one discarded its own cases with
`(void)cases`. Replaced with real comparisons against a MuJoCo-generated fixture
(800 Go2 + 400 G1 samples). The Go2 gate is two-sided (<2.5mm and >1.9mm): the
error is a known structural offset, so a loose one-sided gate would pass a real
regression.

### A false alarm worth recording

I twice concluded the filter's leg model was broken -- first "652 mm", then
"r_dot off by 0.23 m/s, 0.8x sigma_leg". Both were mine: I compared **base-frame
FK output against world-frame MuJoCo**. Corrected values are 2.000 mm and
0.0018 m/s. Go2 was never broken.

## P3 — G1 puppet (done, `see git log`)

`sim/otolith_sim/g1_puppet.py`, `mech/tests/test_g1_puppet.py` (5 gates).

Quasi-static gait (stand, lateral weight shift, squat). The weight shift *is* the
9.2 deg lateral leg tilt that breaks the planar model, so a locomotion gait would
not test it harder, and writing a kinematic biped walk is mostly gait authoring
rather than filter testing.

**THE GATE: stance feet exactly stationary.**

| | before | after |
|---|---|---|
| worst consecutive stance step | 173 um | **0.33 um** |
| apparent foot velocity vs sigma_leg 0.3 | 0.087 m/s (29%) | **0.00017 m/s (0.05%)** |
| IK residual (mean / max) | 0.00012 / 0.00036 mm | 0.00002 / 0.00073 mm |
| joint-limit violations | 0 | 0 |

### The creep bug: two correct numbers hiding a wrong pose

The 173 um creep presented as a *stationarity* failure with an IK residual of
1e-7 m and zero limit violations -- both of which passed. Two things were wrong:

1. **`_rpy_to_quat` computed `cos(angle)/2`, not `cos(angle/2)`**, so it returned
   a **non-unit** quaternion (norm 0.125 for a 0.02 rad pitch). MuJoCo applied
   that malformed base attitude while the IK target was built with the correct
   rotation matrix, so the sole advanced with the base at exactly the base's own
   rate, with a constant ~19 mm x offset. Invisible at zero attitude, which is the
   only case any earlier check exercised. Now gated directly against MuJoCo's
   `xmat` at four non-zero attitudes.
2. **The null-space posture term wound joints without bound** -- `ankle_roll`
   reached 102 rad against a 0.262 rad limit, with the foot still exactly on
   target. A converged solve is not a valid pose. Now clamped to the real limits.

The gates are deliberately redundant as a result: residual AND limit legality AND
world-frame stationarity, because each of the two bugs hid behind the other two.

### Seeded from the vendor stance, not a guess

`scene_mjx.xml` ships two real stance keyframes; `g1.xml`'s only `stand` is the
model zero pose -- a straight leg reaching 0.8021 m, which is why an early
`base_height` of 0.79 left 9 mm of margin and the IK stalled against clipped
rails. `_load_vendor_stance()` reads `knees_bent` (knee 38.33 deg, base z 0.7550)
from scene_mjx.xml and uses it as the posture prior.

Contact geometry still comes from the patched `g1.xml`, deliberately: all eight
foot geoms in `scene_mjx.xml` carry `contype=0` / `conaffinity=0`, because MJX
uses explicit contact pairs the stock scene does not enable.

## P4 — G1 sigma_leg (done: the shipped constant is wrong for a leg this long)

`fusion/src/sigma_study.cpp --robot g1`, `mech/tests/test_g1_puppet.py` (2 gates).

| sigma(r_dot), stance, m/s | all axes | notes |
|---|---|---|
| recorded G1 log, dt = 2 ms | **1.048** | 3.5x the shipped 0.3 |
| recorded G1 log, dt = 1 ms | **2.042** | **1.95x the 2 ms value** |
| noiseless encoders, dt = 2 ms | **0.184** | genuine body motion |
| shipped `sigma_leg` | 0.3 | covers 0.184 with 1.6x margin |

**sigma_leg scales as 1/dt. It cannot be a constant.**

Leg odometry differentiates foot position. Encoder angle noise of `sigma_q` over a
lever arm `L` becomes velocity noise of `sigma_q * sqrt(2) * L / dt`. With
`sigma_q = 0.002 rad` and `L = 0.80 m` that predicts **1.131 m/s** at 500 Hz
against **1.064 m/s** measured — an implied effective lever of 0.752 m, 94% of
nominal, which is what random joint errors partly cancelling along a 6-DoF chain
should give.

The ratio gate is the sharp part: halving `dt` doubles sigma(r_dot) to within
0.1% (1.9988). A genuine white *position* noise term would also give 1/dt, which
is exactly the problem — the filter's `sigma_leg` sits in the *velocity* domain
and so is correct at precisely one sample rate and wrong at every other.

### Why this was not caught on Go2

Go2 measures 0.25–0.41 m/s against a shipped 0.3, which looks like agreement. It
is coincidence between two terms: a 3-DoF leg with a ~0.3 m lever has a much
smaller quantization term than a 6-DoF 0.80 m leg, and Go2's real-motion term is
correspondingly larger. G1 separates them -- 0.184 of motion, 1.03 of
quantization.

### This also explains v0.6's performance sweep

v0.6 measured stance sigma(r_dot) = 0.25–0.41, shipped 0.3, and then found the
*performance* optimum near `sigma_leg = 10`. That looked like a systematic bias
with no provenance. It has one: inflating `sigma_leg` toward 10 was partially
compensating for a mis-modelled, rate-dependent quantization term roughly 3x
larger than the constant allowed for. The sweep was not noise, it was the
odometry's true noise, mis-fitted.

### Recommendation

Keep `sigma_leg = 0.3` for Go2 — it is validated bit-identically and the golden
tests depend on it. For G1, the honest value is **~1.05**, or better: make it
leg-length dependent (`sigma_q*sqrt(2)*L/dt`) rather than constant, which is what
the mechanism actually says. The structural fix is to stop differentiating raw
encoder noise — filter the joint angles before differencing, or model `r_dot` noise
as the sum of a white term and a rate-dependent term.

## P2 — G1 end-to-end (wired; the G1 attitude diverges, cause not yet proven)

`fusion/src/fuse_log.cpp`: `--robot <go2|g1>`, `--sigma-leg-from-robot`.

**Go2 is bit-identical.** `fuse_log go2.otlg out.est` and
`fuse_log go2.otlg out.est --robot go2 --sigma-leg-from-robot` produce
byte-identical output (md5 `ff21f689…`). The descriptor is only pushed into the
filter when `n_legs != 4 || dof_per_leg != 3`, so the Go2 path is untouched by
construction rather than by a passing test.

G1 runs end-to-end: 3000 rows, `robot g1: 2 legs x 6 dof, sigma_leg_vel = 1.05`.

### It does not converge. The ablation localizes it exactly.

| run | att RMSE roll/pitch/yaw (deg) | pos RMSE x/y/z (m) |
|---|---|---|
| G1, contact updates on | **60.0** / 2.0 / 66.2 | 0.059 / 0.132 / 0.016 |
| G1, `--no-leg-update` | **0.5** / 0.6 / 0.4 | 0.160 / 0.498 / 0.432 |
| Go2, contact updates on | 2.2 / 1.5 / 15.0 | 0.495 / 0.157 / 0.473 |

Dead reckoning holds attitude to **0.5 deg**, so the gyro/accel path and the
initialisation are sound. The contact update is what destroys it — and it
simultaneously *improves* position by 3x in z. So the update is not adding noise;
it is applying a correction whose sign or frame is wrong for this robot.

Roll is the axis that matters: the 9.2 deg lateral leg tilt is a roll excitation,
and roll is what diverges (final −177 deg, i.e. flipped) while pitch stays at
2.0 deg. Go2's roll is 2.2 deg.

### What is NOT established

I checked the obvious suspect and it is innocent: the update already uses the true
sole position via `foot_pos_base(robot_, lg, qleg)`, not a fixed nominal, so this
is not the "constant foot position" assumption failing. The residual is
`r_dot + omega x r_base`, which is the correct world-stationarity constraint for a
stance foot. Remaining candidates, in the order I would test them:

1. **Single-contact attitude observability.** G1 has ONE foot down 90% of the
   time (double support is only 10%). Go2's trot keeps two down far more often.
   A single contact constrains 3 DOF but leaves the attitude about the contact
   poorly determined, and the resulting gain can oscillate.
2. Frame convention on `v` (body vs world) that Go2's symmetric stance hides.
3. Whether `LegSpec.side` is still consulted on the `Chain` path, where `hip_base`
   already carries the lateral offset — a double-counted or dropped lateral term
   would be exactly a roll error proportional to lateral sole excursion.

I am not claiming a root cause. The measurement above is the result; (3) is where
I would look first.

Also stale: `eval/evaluate.py` still prints hard-coded Go2 prose ("kinematic trot",
"`sigma_leg=0.3 m/s` per foot") for every input including this G1 run.

## P6 — Apollo (Apptronik), second robot: descriptor layer done

`sim/otolith_sim/leg_model.py` (`load_apollo`), `mech/tests/test_apollo_leg.py`
(9 gates). **No scene patch -- and the first draft of one was wrong twice.**

80.898 kg, 6 DoF/leg, so the same 12-DOF log shape and the same seam as G1. Nothing
else about the two models lines up, which is the point of adding it:

| | G1 | Apollo |
|---|---|---|
| root | `pelvis` | `base_link` |
| chain | `hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll` | `hip_ie (yaw), hip_aa (roll), hip_fe (pitch), knee_fe, ankle_ie (roll), ankle_pd (pitch)` |
| foot | 4 spheres | one 200x85x18 mm box |
| body names | `left_*` | `l_*` / `r_*` |
| vendor stance | all-zeros zero pose | real pose, base z 1.01597 — but 1.19 mm through the floor |
| collidable geoms | 8 (after patching) | **0 — and it does not matter** |

### Apollo's right leg mirrors its quaternions. G1's does not.

The gate I ported from G1 asserted that only `body_pos.y` mirrors and the
quaternions are identical — and it correctly failed. Apollo reflects every link's
quaternion about y, `(w,x,y,z) -> (w,-x,y,-z)`, uniformly across all six.

Neither rule transfers. Carrying G1's into Apollo puts every link past the hip
180 degrees out; carrying Apollo's into G1 does the mirror image. So the
descriptor reads transforms from the model and never derives one leg from the
other. The assertion was the thing that was wrong, not the model.

### Two traps in this model, both silent

**Every one of the 79 geoms has `contype=0 conaffinity=0`**, including the two
obvious sole boxes. There is nothing to select by until contact is enabled, so
`build_apollo_scene.py` turns it on. Same situation as G1's `scene_mjx.xml`, same
reason: those scenes are driven with an explicit contact pair list.

**`l_foot_fl/fr/bl/br` are mesh assets, not geoms.** Asking MuJoCo for a geom by
one of those names returns **-1**, and indexing `geom_bodyid[-1]` then silently
reads the *last* geom, which sits on `r_foot_link` — so both feet appeared at
y = -0.1528 and looked coincident. The real geoms are `collision_l_sole` and
`collision_r_sole`. `apollo_contact_geoms` selects by name *and* asserts the geom
exists and is contact-enabled, so this cannot recur silently.

### Sole point: box bottom face, not box centre

The box centre sits 9 mm — half the 18 mm thickness — above the floor when the
foot is flat. Using it would bias every foot position in the log by 9 mm, 3% of
the σ_leg budget, in a plausible-looking direction. The FK gate caught this
immediately by reporting a worst error of exactly 8.9999 mm, which is the
half-thickness and nothing else.

### T4

Worst sagittal 2R-equivalent sum is hip_fe 1.85 + knee_fe 2.618 + ankle_pd 1.571
= 6.04 rad, against the 3π = 9.425 rad fold limit. Wider than G1's 5.41 rad
because Apollo has a pitch ankle as well as a pitch hip. Every joint range is
also checked individually, since hip_ie/hip_aa/ankle_ie are not in a 2R chain but
still go through the same `sin_cos`.

### Not done, and worth flagging

Stance width is 305.6 mm against G1's 233 mm, with the feet 42.8 mm outboard of
their hips. The lateral CoM shift that breaks the planar 2R model scales with
that outboard distance, so **Apollo is a milder test of the planar assumption than
G1, not a harder one** — worth knowing before treating it as the stronger
evidence. No puppet or C++ `robot_spec("apollo")` yet.

## Not done

- **P2** G1 descriptor wired end-to-end (unblocked now the puppet produces a log)
- **P4** sigma_leg re-measured for G1 (`sigma_study` is Go2-only today)
- **P5** falsification report -- T1 planar leg, T2 equal-split GRF, T3 sigma_leg,
  T5 4-contact all still unscored
- **P6** Apptronik Apollo
