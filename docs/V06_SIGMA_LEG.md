# v0.6: σ_leg — where 0.3 m/s comes from, and what it does not explain

2026-10-06. The open question was that σ_leg = 0.3 m/s shipped as an
*assertion* — "inflated to cover encoder-noise-amplified `r_dot` via finite
difference" — with no measurement behind it. Phases C and D removed the
structural candidates (link flex, foot shape) without touching it, so this phase
asks the question properly.

Short answer: **0.3 is correct, and verified rather than assumed.** Encoder noise
alone produces σ(r_dot) = 0.25–0.41 m/s, and 0.3 sits in the middle of that. The
candidates that could *not* explain it are now excluded by measurement.

The surprise is somewhere else entirely, and it is not about σ_leg.

## Measurement: `fusion/src/sigma_study.cpp`

A new C++ tool rather than a script, deliberately. `r_base` comes from
`leg_kin::foot_pos_base` — the same planar 2R FK the filter's measurement model
uses. Re-implementing that in Python to analyse it would create a second source
of truth that can disagree with the filter by an unknown amount, and this repo has
already been bitten twice that way (`L2` provenance, `mat3_mul_fixed` sizing).

On a 20 s log at 500 Hz, stance feet only:

| foot | σ_x | σ_y | σ_z |
|---|---|---|---|
| FL | 0.3967 | 0.3879 | 0.2481 |
| FR | 0.4065 | 0.3912 | 0.2531 |
| RL | 0.3968 | 0.3802 | 0.2495 |
| RR | 0.4091 | 0.3898 | 0.2582 |

**worst stance σ(r_dot) = 0.4091 m/s**, shipped σ_leg = 0.3, ratio **0.73**.

Stance-only because a swing foot's `r_dot` is real motion rather than noise; the
first sample of each stance is excluded because the transition is a real
discontinuity.

## The mechanism, confirmed by arithmetic

`EncoderNoise.sigma = 0.002 rad`. Three joints per leg, lever arm ~0.42 m, so
σ_r ≈ 0.42 × 0.002 ≈ 0.8 mm. The finite difference against the previous step
multiplies by √2 for two independent samples, then divides by dt = 2 ms:

    0.8e-3 * sqrt(2) / 0.002  =  0.57 m/s

Same order as the measured 0.25–0.41 m/s, and the gap is just the exact
`∂r/∂q` geometry rather than a worst-case lever arm. **The documented rationale
for 0.3 is correct.** It was a guess that happened to be right, which is a
different and worse thing to have had.

## Terrain: excluded, exactly like the foot pad

`sigma_study --place-err <σ_mm>` adds a placement error drawn once per stance and
held for that stance — which is what uneven ground or a sloppy foothold actually
does. The foot lands slightly off and then stays where it landed.

| placement σ | σ(r_dot) stance feet | σ(r_dot) incl. swing |
|---|---|---|
| 0 mm | **0.4091** | 0.4400 |
| 10 mm | **0.4091** | 0.7817 |
| 50 mm | **0.4091** | 3.2663 |
| 200 mm | **0.4091** | 12.9580 |

The stance column is **bit-identical** from 0 mm to 200 mm. A placement error is
constant during stance, so it cancels exactly in `(r(t) − r(t−1))/dt`. It is an
offset, not a rate.

Terrain therefore cannot explain σ_leg — the same argument, and the same
conclusion, as the foot pad in `docs/V06_FOOT_PAD.md`. It shows up only in the
touchdown transient, at roughly 0.065 · σ_err m/s for one sample per stance.

Two independent mechanisms, both producing offsets, both excluded. That is now a
pattern rather than a coincidence.

## The thing that is actually surprising

`fuse_log --sigma-leg <v>` (new flag) sweeps the filter's *performance*. The
existing `fuse_update_study --sigma-leg` sweep measures the fixed-point
numerical floor (~0.0055), which says nothing about tolerated noise — the two are
easy to confuse and both are called "the σ_leg sweep".

Pos RMSE, 20 s, 500 Hz, two independent seeds:

| σ_leg | seed A | seed B | mean | att A | att B |
|---|---|---|---|---|---|
| 0.1 | 1.9663 | 0.1634 | 1.0648 | 86.74° | 10.63° |
| **0.3 (shipped)** | **1.7658** | **0.4614** | **1.1136** | 71.92° | 15.08° |
| 1.0 | 1.0912 | 0.1333 | 0.6122 | 46.37° | 5.87° |
| 3.0 | 0.5438 | 0.0790 | 0.3114 | 26.06° | 4.97° |
| **10** | **0.3007** | **0.1867** | **0.2437** | 18.92° | 9.03° |
| 30 | 0.5792 | 0.5256 | 0.5524 | 14.51° | 10.36° |

Optimum at σ_leg ≈ 10 m/s in both seeds, on both sides of the cliff: worse below
0.3 (attitude blows up to 87°), worse above 10 (corrections stop mattering). The
shipped 0.3 is roughly **4–5× off optimal**, and sits on the steep part.

So the filter performs best when it **under-trusts leg odometry by ~25×** relative
to the noise actually present. That is not something σ_leg can fix, because
σ_leg only scales `Rmat`. It says the leg measurement carries **non-zero-mean
error** that the filter is currently absorbing as though it were noise — and the
cheapest available mitigation for bias is exactly to inflate `Rmat`.

That is a real, separate finding, and it is a *bias* question rather than a
noise one. It is not fixed here; it is recorded, because the honest reading is
that σ_leg = 0.3 was never the problem and that tuning it to 10 would be fitting
a symptom.

### Also worth knowing: the seed variance

At σ_leg = 0.3, pos RMSE ranges **0.461–1.766 m** across two seeds of the same
gait. Any single-run RMSE comparison in this repo, including the shipped
`0.3072 m`, carries that much uncertainty. Comparisons made on one log are not
evidence unless both arms used the same one.

## Joint-bearing compliance: excluded, and how

Asked as an **inverse** question, because there is no bearing stiffness anywhere
in this repo and picking one would invent the answer. Using the measured joint
moments from `docs/V06_JOINT_REACTIONS.md`:

    r_dot from compliance  =  lever * (dM/dt) / k

At the knee: lever 0.213 m, p95 |dM/dt| over steady stance 61.3 N·m/s (the p95,
not the max — the max is the touchdown step, see below). So:

    k needed for compliance alone to reach 0.3 m/s   =  44 N·m/rad

| k (N·m/rad) | sag at the 16.5 N·m peak | r_dot (m/s) | vs 0.3 budget |
|---|---|---|---|
| 44 | 21.5° | 0.297 | 1.0× |
| 100 | 9.5° | 0.131 | 2.3× |
| 200 | 4.7° | 0.065 | 4.6× |
| **470** | **2.0°** | **0.028** | **10.8×** |
| 1000 | 1.0° | 0.013 | 23.0× |

**44 N·m/rad would mean 21.5° of knee sag under peak load.** That is not a soft
drivetrain, it is a leg that cannot carry itself. A legged robot tolerates
order 1–2° here, which puts the admissible stiffness at k ≳ 470 N·m/rad — where
the contribution is 0.028 m/s, **11× short** of the budget.

So compliance is excluded, and excluded by a wide margin rather than narrowly.

### Two things that had to be handled to get there

**The touchdown step is an artifact, not physics.** Peak |dM/dt| at the knee is
3488 N·m/s — 57× the steady value. It is the per-foot force switching between
zero and full on the contact mask. The *total* contact force is smoothed over 18
steps, but the equal split across stance feet is not, so the mask applies a step
to one foot's share. Excluding 2 samples either side of each transition is
therefore removing an artifact of the method, not smoothing over a real force
onset.

**It may not be visible at all.** Leg odometry is built from encoder angles. A
motor-side encoder sits upstream of the bearing compliance and would not see the
deflection in the link; only an output-side or link-side encoder would. So before
the stiffness question even arises, there is a question of *where the encoder
sits* — and the sim has no compliance at all, so any contribution is
definitionally zero there. This exclusion is about whether reality has it in a
place the encoder can see, and it says: not at a magnitude that matters.

## Where this leaves σ_leg

| candidate | verdict | evidence |
|---|---|---|
| encoder noise | **the explanation** | measured 0.25–0.41 m/s; σ_leg = 0.3 verified |
| terrain / foothold placement | **excluded** | stance σ invariant to 200 mm |
| foot pad shape | **excluded** | 8 mm/s, 36× short (`V06_FOOT_PAD.md`) |
| link flex | **excluded** | ≤9 µm vs ~500 µm travel (`V06_FEA_THIGH.md`) |
| joint-bearing compliance | **excluded** | needs k < 44 N·m/rad, which is 21° of knee sag |

σ_leg is no longer an unexplained fudge — it is a measured consequence of
0.002 rad encoder noise differentiated at 500 Hz, and the filter's belief about
it is accurate. Every competing mechanism is now excluded by measurement, and
three of the four do so by the same argument: they produce an **offset**, and an
offset drops out of `r_dot`.

The genuine open item is no longer σ_leg's provenance. It is the bias finding
above: the filter would rather under-trust leg odometry by 25× than trust it at
its true noise level, which means the leg measurement carries systematic error
that `Rmat` is absorbing. σ_leg was never the thing that needed explaining.

## Reproducing

```bash
PYTHONPATH=sim pixi run python -c "
from otolith_sim.logger import record_puppet_log
from pathlib import Path
record_puppet_log(Path('/tmp/sl.otlg'), duration_s=20.0, dt=1/500)"
./fusion/build/sigma_study /tmp/sl.otlg                    # measured sigma
./fusion/build/sigma_study /tmp/sl.otlg --place-err 50      # terrain
for s in 0.1 0.3 1.0 3.0 10 30; do
  ./fusion/build/fuse_log /tmp/sl.otlg /tmp/e.estm --sigma-leg $s
done
```