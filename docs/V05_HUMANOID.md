# v0.5: humanoid generalization

Status: **P0 and P1 complete, P3 in progress.** P2, P4, P5, P6 not started.

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

## P3 — G1 puppet (in progress)

`sim/otolith_sim/g1_puppet.py`. Quasi-static gait (stand, lateral weight shift,
squat): the weight shift *is* the 9.2 deg tilt that breaks the planar model, so a
locomotion gait would not test it harder, and writing a kinematic biped walk is
mostly gait authoring rather than filter testing.

Solved so far, each found by measuring:

| bug | symptom | cause |
|---|---|---|
| null-space projector | 1190 mm residual | `I - J^T(JJ^T+l^2 I)^-1 J` is a projector only at l=0; rebuilt from an SVD |
| no limit clamping | `ankle_roll` = 102 rad (limit ±15 deg) | a true projector lets the posture term wind joints without bound while the foot stays exact |
| foothold offset sign | right foot 122 mm behind its hip | floored on `t/T - 0.5` but placed on `k - 0.5`; unreachable, showed as 38 mm residual |
| `base_height` 0.79 | marginal | reachable span is 0.8021 m; sampling 40k legal poses set it to 0.70 |
| warm-start | 105-247 mm residual | knee range starts at -0.087 rad, so a warm start stalls against a clipped rail; seeded from the posture prior instead |

Current state: IK residual **0.000111 mm mean**, **0 joint-limit violations**.

### Open: the stance feet still creep

Worst consecutive stance step is **173 um**, i.e. **0.087 m/s of apparent foot
velocity against sigma_leg = 0.3 m/s** -- 29% of the noise budget from foot creep
alone, which would contaminate every RMSE downstream.

The cause is NOT the solver: the base-frame residual is 1e-7 m, and reading the
joint angles back out of the compiled model reproduces `p_base` exactly. Yet the
world sole advances with the base at exactly the base's own rate, with a constant
~19 mm x offset. So the target-to-world composition is wrong somewhere in
`sample()`, and it has not been located. Unresolved.

## Not done

- **P2** G1 descriptor wired end-to-end (blocked on P3 producing a log)
- **P4** sigma_leg re-measured for G1 (`sigma_study` is Go2-only today)
- **P5** falsification report -- T1 planar leg, T2 equal-split GRF, T3 sigma_leg,
  T5 4-contact all still unscored
- **P6** Apptronik Apollo
