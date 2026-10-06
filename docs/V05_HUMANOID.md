# v0.5: humanoid generalization

Status: **P0, P1, P3 and P4 complete.** P2, P5, P6 not started.

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

## Not done

- **P2** G1 descriptor wired end-to-end (unblocked now the puppet produces a log)
- **P4** sigma_leg re-measured for G1 (`sigma_study` is Go2-only today)
- **P5** falsification report -- T1 planar leg, T2 equal-split GRF, T3 sigma_leg,
  T5 4-contact all still unscored
- **P6** Apptronik Apollo
