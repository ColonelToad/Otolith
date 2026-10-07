# v0.5: humanoid generalization

Status: **P0, P1, P3, P4, P6, P7 complete. P2 root-caused** (attitude Jacobian, left unfixed by design — see P2). P5 skipped by decision.

Four robots through one seam: Go2 (3 DoF planar, validated), G1, Apollo, OP3 (all 6 DoF chains).

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

## P3 — G1 puppet (done, `d5a3dc0`)

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

## P2 — G1 end-to-end (wired; attitude diverges — ROOT CAUSE FOUND, deliberately unfixed)

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

### Root cause: FOUND, by finite-differencing the measurement Jacobian

The measurement model itself is correct. `h = v + R(w x r + r_dot)` is exactly the
world velocity of the stance foot, and forcing `h = 0` is the right constraint.
Finite-differencing it confirms two of the four Jacobian blocks:

| block | quantity | finite-difference truth | code | verdict |
|---|---|---|---|---|
| velocity (3) | `dh/dv` | `I` | `I` | correct |
| gyro bias (9) | `dh/dbg = -dh/dw` | `+R skew(r_base)` | `+R skew(r_base)` | correct |
| **attitude (0)** | `dh/dtheta` | **`+R skew(r_base)`** | **`-R skew(w x r)`** | **WRONG** |

`fusion/src/fusion.cpp:96`:

```cpp
Eigen::Matrix3d wr = skew(omega_cross_r);
...
H.block<3,3>(k*3, 0) = -R * wr;      // <- attitude block
```

A body-frame attitude perturbation `dtheta` moves a world-frame point by
`R (dtheta x r_base)`, so `dh/dtheta = R skew(r_base)`. The code instead
differentiates with respect to something else entirely — `w x r` rather than `r`
— and negates it. Wrong quantity *and* wrong sign, in the one block that controls
attitude, which is why **roll** is the axis that diverges while pitch holds at
2.0 deg.

The magnitude matters too: `|w x r| ~ 0.016` against `|r| ~ 0.8`, so the attitude
Jacobian is roughly **50x too small**. That is not G1-specific — Go2 runs the same
code — but a 50x-too-small attitude Jacobian means contact updates barely correct
attitude at all, which is exactly the behaviour already noted in
`eval/M3_REPORT.md` and in the eval notes ("dead reckoning wins attitude"). Go2's
attitude errors are small (2.2 deg) because its attitude is dominated by gyro
integration, which is correct when nothing is pushing it; G1's larger `r` and the
9.2 deg lateral tilt are enough to make the wrong block actively harmful rather
than merely inert.

**NOT FIXED HERE, DELIBERATELY.** This block is shared with Go2, whose results are
bit-identical and whose golden tests depend on it. Changing it would move every
recorded Go2 number in the repo, which is a decision about the project's results,
not a bug fix to slip in alongside P7. The finding is recorded so the change can be
made deliberately, with the golden baseline regenerated and the Go2 regression
re-run as its own piece of work.

### What was ruled out along the way

1. **The data path is exonerated.** Per-leg innovations measured from the G1 log
   (Python FK, which is bit-exact against MuJoCo) are symmetric between legs:
   left mean `[0.049, -0.023, -0.017]` / std `[0.997, 0.867, 0.197]`, right
   `[0.053, 0.020, 0.017]` / std `[1.047, 0.895, 0.212]`. A one-leg indexing or
   contact-slot error would show as an asymmetry; there is none. The ~1 m/s RMS is
   the known encoder-quantization term, exactly as P4 predicts.
2. **Single-contact observability is not the cause.** G1 does have one foot down
   90% of the time, but that would produce a weak update, not a sign-flipped one.
3. **`LegSpec.side` is not double-counted.** On the `Chain` path `foot_pos_base`
   uses `hip_base` and the link `origin`s; `side` is the planar Go2 field. The
   measurement model is verified against MuJoCo, so `r_base` is right.
4. **Frame convention on `v` is correct** — confirmed by `dh/dv = I`.

Also stale: `eval/evaluate.py` still prints hard-coded Go2 prose ("kinematic trot",
"`sigma_leg=0.3 m/s` per foot") for every input including this G1 run.

## P6 — Apollo (Apptronik), second robot: COMPLETE

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

## P7 — OP3 (Robotis), third robot: COMPLETE

`mech/spec/build_op3_scene.py`, `leg_model.load_op3`, `op3_puppet.py`,
`build_biped_spec.py op3`, `mech/tests/test_op3_leg.py` (7 gates),
`mech/tests/test_op3_puppet.py` (7 gates).

3.147 kg, 510 mm, 20 DoF. A miniature, and the least like the other two bipeds:

| | G1 | Apollo | OP3 |
|---|---|---|---|
| root | `pelvis` | `base_link` | `body_link` |
| chain | hip_pitch… | hip_ie… | hip_yaw… |
| foot | 4 spheres | 1 box | 2 boxes |
| joint limits | real | real | **NONE** |
| keyframes | none usable | `stand` | **none at all** |
| mirror rule | quats EQUAL | quats REFLECTED | quats EQUAL + **axes negated** |
| hip spacing | 129 mm | 220 mm | 70 mm |
| FK error vs MuJoCo | 27 pm | 0.80 nm | **0.0018 nm** (over full ±π) |

### Three bugs, two of them in the SHARED base

**1. A straight leg is a singularity — third robot, third disguise.** OP3 has
`nkey 0`, so the only stance available is the authored one: all joints zero, feet
flat, base at 0.27915 after lowering by the measured 20.9 mm float. But that is
full extension, and the IK answered a small target change with a **211 mm residual
and a joint thrown to ±π**. Standing at 0.95 of full extension puts the knee at
0.672 rad. This is the same trap as G1's `stand` keyframe — straight leg, a little
margin, an IK that stalled at 38 mm — and it has now appeared in three robots.

**2. `_home_q` was solving in mixed frames.** It passed
`root_pos = (0,0,base_height)` with a target of `(0, hip_y, −base_height)`, asking
the leg to reach `2×base_height` below its own root: 1.51 m for a 0.80 m leg.

This lived in the shared base class and had been there for two robots. It was
invisible because the residual is *discarded* and `q_home` is only read for its
non-leg entries — every leg joint is overwritten per sample. G1 and Apollo were
accidentally immune because neither derived anything from it. OP3 was not, because
its posture seed comes from exactly that solve. The residual is now **checked**
rather than discarded, so the next robot cannot inherit it.

**3. `joint_limits` mistook "unconstrained" for "at zero".** An unlimited hinge
reports `jnt_range [0,0]`, and handing that back verbatim made the IK clamp all
twelve of OP3's joints to zero — a **41 mm residual** that read as a convergence
failure rather than a missing bounds check. `jnt_limited` is the only thing that
distinguishes the two.

OP3's own limit gate must check `jnt_limited` too: against `jnt_range`, every pose
is a "violation".

### σ_leg: the falsification case, and it holds

| robot | lever | σ_q | motion | quant | total | vs shipped 0.3 |
|---|---|---|---|---|---|---|
| Go2 | ~0.30 | — | ~0.18 | ~0.25 | 0.30 | 1.0× |
| **OP3** | 0.28 | **0.000443** | 0.138 | 0.073 | **0.156** | **1.9× conservative** |
| OP3 | 0.28 | 0.002 | 0.138 | 0.337 | 0.364 | 0.8× |
| G1 | 0.80 | 0.002 | 0.184 | 1.032 | 1.048 | 3.5× optimistic |
| Apollo | ~0.90 | 0.002 | 0.184 | 1.386 | 1.398 | 4.7× optimistic |

OP3's 0.000443 rad is its real actuator: DYNAMIXEL XM430, 4096 counts/rev =
0.001534 rad/tick, quantisation σ = tick/√12.

OP3 was the robot that could have **disproved** the lever-arm argument — short lever
*and* 4.5× finer encoder, both pushing σ_leg down, so a wrong mechanism would have
been easiest to hide. Instead it is the one robot where the shipped 0.3 turns out
conservative.

The ordering is monotone in **(σ_q × lever)** and is *not* monotone in mass: OP3 is
3.15 kg and Apollo 80.9 kg, yet their σ_leg differs 9×. Mass was the wrong variable —
that is the v0.6 error, now settled.

Prediction from the nominal leg: `σ_q√2·L/dt` in quadrature with the measured motion
term gives 0.163 against 0.156 measured (4.5% over) at the real encoder, and 0.418
against 0.364 at σ_q=0.002. Both over-predict because the effective lever is ~0.24 m
against a 0.279 m nominal (86%) — same direction as G1 (94%) and Apollo (109%).

### A trap in the noiseless-log recipe

The first OP3 "noiseless" log measured 0.42 m/s and looked like a large real-motion
term. It was not noiseless: `record_biped_log` does
`from otolith_sim.sensors import EncoderNoise` **inside the function body**, so
patching `logger.EncoderNoise` is shadowed by that local binding. Patching
`sensors.EncoderNoise` works, and that is what G1's and Apollo's clean runs had
done.

The tell was that the logged `qj` differed from a live puppet by 3e-3 rad —
suspiciously close to σ_q = 0.002 — while two puppet runs agreed to 0.0e+00.

### The coverage gap, stated plainly

OP3 has no joint limits, so it validates **none** of the joint-range machinery:
`sin_cos_wide`'s fold is vacuous and the null-space clamp is a no-op. Three robots
passing must not be read as three-fold coverage — G1's puppet gates have to keep
running. Recorded in `test_op3_leg.py` rather than left implied.

The narrowest stance of the three (70 mm of hip spacing, against G1's 129 and
Apollo's 220) also makes OP3 the **most** aggressive test of the planar assumption,
which is the opposite of Apollo. The three bipeds are complementary rather than
redundant.

## Not done

Rewritten after the phase-6 close-out; the previous version still listed P2, P4
and P6 as incomplete and described P5 as unscored, all of which contradicted the
status line at the top of this file.

- **The attitude Jacobian fix itself** — root-caused in P2 above, and now applied
  with the finite-difference gate alongside it. See P2 for the measured before and
  after. What remains open is the *consequence*: Go2's recorded results are
  historical, and any doc quoting them needs rewording.
- **σ_leg is still four constants standing in for a formula.** The 1/dt
  dependence is now explicit, but `sigma_leg = sigma_q * sqrt(2) * k * L / dt`
  needs a fitted `k` per robot (G1 0.94, Apollo 1.09, OP3 0.86) because the
  effective lever is not derivable from nominal length. The structural fix —
  filtering joint angles before differencing rather than differentiating raw
  encoder noise — is still not done.
- **Chain model has no fixed-point path.** `rust/otolith-fusion/src/leg.rs` is
  Go2-planar only, so "float and fixed are bit-identical" remains a Go2-only
  claim. Deferred deliberately: a 6-DoF chain needs per-link axis/origin/
  quaternion triples in fixed point, and CORDIC `sin_cos_wide` folding constrains
  how those are represented. Doing it before something needs the embedded path
  would be work chosen by symmetry rather than by requirement.
  `rust/otolith-fusion/tests/scope.rs` pins the boundary so it cannot drift
  silently.
- **P5** skipped by decision. T1 (planar leg), T2 (equal-split GRF), T3
  (sigma_leg), T5 (4-contact) remain unscored.
- **Apollo's series-elastic compliance** is a real and unmodelled leg-odometry
  error term: joint angle is not actuator position on that hardware. Out of scope
  for the kinematic path, and v0.6's ruling that joint-bearing compliance was not
  the Go2 sigma_leg source does not transfer to a series-elastic leg.
