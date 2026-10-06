# v0.6: joint reactions, measured instead of modelled

2026-10-06. Phase C flagged this as the largest gap in the FEA: the load case
resolved a *vertical* force into the thigh frame by stance angle, which is a
model rather than a measurement, and shear and torsion were absent entirely.

The obvious approach was to read MuJoCo's constraint forces, since the puppet
zeroes `qvel` and calls `mj_forward` — a static pose, so the forces look like a
clean measurement. **They are not.**

## Why `qfrc_constraint` does not work here

Checked directly, total contact force at t = 0.35 s:

    from mj_forward        219.09 N
    from CoM momentum      166.07 N
    relative residual       36.0%

A 36% imbalance settles it. The puppet drives positions directly and never
integrates dynamics, so its legs are not *holding the body up* — they are being
*placed*. The prescribed motion is not dynamically consistent, so the constraint
solver's forces are reactions to nothing. Reading `qfrc_constraint` would have
produced plausible numbers with no physical referent, which is the specific
failure this repo keeps paying for.

The force input is therefore the CoM momentum balance — the one contact-force
method here with a verified conservation check (0.998661 W,
`docs/V06_LOAD_CASES.md`). It answers "what force does the prescribed motion
demand", and statics distributes that into the joints.

New module: `sim/otolith_sim/joint_reactions.py`.

## Method

Per step: total contact force `m·(a_com − g)` from the same 9-to-162-step
smoother as the GRF series, split equally across stance feet (exact for the
diagonal pairs FL+RR and FR+RL). Then, for each joint, statics on the distal
chain — the joint supplies exactly the negative of gravity plus contact force
on the bodies below it, and the torque about its own axis follows from the same
sum taken about the joint origin.

### Two identities that verify it

A swinging leg hangs entirely from its hip, so its hip reaction must equal the
leg's own weight and nothing else:

    FL hip vertical reaction during swing   20.320 N
    leg self-weight (0.678 + 1.152 + 0.2414 kg)   20.320 N

Exact. In stance the ground lifts the leg while its own weight pulls it down, so
the hip pushes **down** on it by the difference:

    measured hip reaction    -47.342 N
    leg weight - ground force  -47.342 N   (20.320 - 67.662)

Also exact. Both are checked in `sim/tests/test_grf.py`.

#### A sign error that both identities would have hidden

The first version of this module had the contact force sign inverted
(`m(a − g)` instead of `m(a + g)`). The swing check *passed anyway*, because
with no contact force a flipped sign has nothing to act on — and the swing case
was the one I reached for first. In stance the two errors cancelled in the force
sum, leaving every number ~6% high with inverted component signs: hip peak 114 N
instead of 72 N, knee 107 N instead of 79 N.

It was caught only by comparing against `grf.fz_feet`, an independently computed
foot force. That is the second time in this project a check on the *easy* case
passed while the interesting one was wrong, and both times the fix was to
compare against something computed a different way.

## Results

4 s at 500 Hz, stance feet only. **Naming:** in the Menagerie chain the body
called `calf` is what the knee moves, so `FL_calf_joint` is the *knee* and
`FL_thigh_joint` is the hip pitch. The indices look wrong until you check the
joint axis, and the two differ by an order of magnitude in moment.

| joint | role | peak \|F\| (N) | mean \|F\| (N) | peak \|M_axis\| (N·m) |
|---|---|---|---|---|
| FL_hip | abduction | 72.03 | 47.36 | 0.834 |
| FL_thigh | hip pitch | 78.68 | 54.01 | 1.315 |
| **FL_calf** | **knee** | **89.99** | **65.31** | **16.514** |

Right legs agree to three figures (FR 72.01 / 78.67 / 89.97, RL 72.01 / 78.67 /
89.97, RR 72.03 / 78.68 / 89.99), which is itself a check since the equal-split
assumption holds exactly only for the mirror pairs.

### The knee moment reproduces phase C independently

Phase C2 derived knee moments of **13.4–17.5 N·m** from geometry and stance
angle. This path never looks at stance angle and peaks at **15.7–16.6 N·m** —
inside phase C's band. Two independent routes to the same peak, which is the
strongest evidence in this phase that the load case is right.

(Phase C quoted a *range across gait configurations*; the instantaneous range
here runs 5.4–16.6 N·m because it includes light-load stance instants. Only the
peaks are comparable, and those agree.)

### Shear and torsion, the part phase C could not see

At the knee:

| component | peak | mean |
|---|---|---|
| axial (along leg) | 78.68 N | — |
| shear, horizontal | **4.67 N** | — |
| bending about knee axis | 16.51 N·m | — |

**Peak shear is 5.9% of peak axial.** So phase C's vertical-only load case was a
reasonable approximation, and it now has a number behind it rather than an
assumption. Torsion is not separately resolved — a hinge carries no moment about
its own transverse axes by construction, so what a real bearing sees is
dominated by the 16.5 N·m bending term.

## What this does not settle

- **Load distribution between stance feet.** The CoM wrench cannot determine it;
  the equal split is exact only for the mirror pairs. A gait with asymmetric
  loading needs the distribution solved per stance pair.
- **Dynamic reaction loads.** The puppet has `qvel = 0` at every sample, so
  these are quasi-static. Inertial reaction from link acceleration is absent, and
  for a trot with 2.86 Hz body bob it is not obviously negligible.
- **The singularity is still there.** `docs/V06_FEA_THIGH.md` reports interior
  stress because the fully-fixed hip face makes peak stress unbounded. This
  measurement does not touch that, and closing it needs the CAD phase.

## Reproducing

```bash
PYTHONPATH=sim pixi run python -c "
import mujoco
from otolith_sim.joint_reactions import record_joint_reactions, summarise
m = mujoco.MjModel.from_xml_path('third_party/menagerie/unitree_go2/scene.xml')
s = record_joint_reactions(m, duration_s=4.0, dt=1/500)
for k, v in summarise(s).items(): print(k, v)"
```