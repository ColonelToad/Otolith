# v0.6: the mass budget — what can be measured, and a recommendation

2026-10-06. The model totals **15.2064 kg** against a published Go2 figure of
**~12.4 kg**, a gap of **+22.6%** that scales every load case in
`docs/V06_LOAD_CASES.md` by 1.2263×. `docs/V06_PREREQS_CLOSED.md` recorded this
as a decision nobody could compute their way out of. This attempts the
computation, and reaches a different answer than expected.

## The cross-check that was worth doing

The earlier attempt measured each link's **visual-mesh volume** by the
divergence theorem and got a nonsense result — mesh 4 reported 6267 cm³, a
non-watertight artifact. Replaced with **convex-hull volume** (`trimesh`), which
cannot fail that way: a hull is defined by its extreme points, so holes,
inverted normals and duplicated vertices are irrelevant.

Vertices are transformed into the body frame first (`geom_xmat @ (vert − mesh_pos)
+ geom_xpos`), because each of the 2–5 meshes per body is authored in its own
local frame and stacking them raw puts them in the wrong places.

| body | mass (kg) | hull (cm³) | implied ρ (kg/m³) | fill % of solid Al |
|---|---|---|---|---|
| base | 6.9210 | 18 794.9 | 368 | 13.6 |
| FL_hip | 0.6780 | 608.7 | 1 114 | 41.3 |
| FL_thigh | 1.1520 | 1 191.6 | 967 | 35.8 |
| FL_calf | 0.2414 | 1 836.8 | 131 | 4.9 |

Materials for reference: Al 6061-T6 **2700**, Al 7075-T6 **2810**, steel 1018
**7850** kg/m³.

## What this actually tells us, and what it cannot

A hull is ≥ the true volume, so **implied density ≤ true material density**.
That makes the test one-directional: it can only detect a link that is *too
heavy* for its own envelope, never one that is too light. Every implied density
here (131–1114) sits below aluminium, so **no link is over-mass for its hull** —
including under steel, which has a lot more headroom than any of them need.

So every part is consistent with being a hollow or shelled structure, and the
thigh at 35.8% of solid aluminium is a plausible figure for a thin-walled leg.
The calf's 4.9% is not alarming either: it is a long curved link whose hull is
mostly air, and a hull is simply a poor proxy for it.

**The +22.6%-over-published question is the one direction this cannot see.**
Detecting over-mass properly needs the true solid volume — per-link vendor
geometry, or CAD. Both are the phase being parked.

## Recommendation: the decision is not load-bearing, so stop treating it as one

The reason this has stayed open is that it reads as load-bearing — every stress
number depends on it. It does not:

| conclusion | depends on mass? | survives ±22.6%? |
|---|---|---|
| thigh interior stress 1.9–2.3 MPa, ~150× margin | yes, linearly | **yes** — margin 120×–154× |
| link flex ≤9 µm | yes | yes |
| σ_leg = 0.3 m/s verified (0.25–0.41 measured) | **no** — it is measured noise | n/a |
| foot pad excluded, terrain excluded, compliance excluded | **no** — all are ratio arguments | n/a |
| GRF / joint reactions | yes, linearly | no threshold claimed from them |

So the one conclusion v0.6 rests on — the thigh is over-strength by more than
two orders of magnitude — holds at either mass. The mass question cannot change
it, because 22.6% is small against 150×.

**Recommendation:** pin the mass at the model's **15.2064 kg**, record the
published figure and the 1.2263× multiplier alongside it, and defer resolution
to the CAD phase where per-link solid volume becomes available.

Reasons for that specific choice rather than the alternative:

- It is the value the sim actually uses, so recorded numbers stay
  self-consistent with the runs that produced them.
- It is the **conservative** direction for structural work — higher mass means
  higher load means a larger claimed margin consumed. Pinning 12.4 kg would
  understate every load by 22.6% until someone proves the model wrong.
- It does not require believing the model is right. It requires only not
  spending margin that has not been earned.

What would change this recommendation: a vendor per-link mass table. That is a
lookup, not a calculation, and it would settle the question outright. It is
also, notably, cheaper than the CAD work that would otherwise be needed — and it
is the reason this should not be reopened without one.

## The lesson from the failed first attempt

The original check used mesh volume and reported 6267 cm³ for a link whose hull
is 1837 cm³. A number 3.4× larger than the convex hull of the same point set is
impossible for any closed surface, which makes it a *detectable* error — but only
if you check the output against a bound. It sat in a table for a phase before
anyone noticed.

A convex hull is not a better volume estimate. It is a volume estimate with a
**guaranteed bound**, which is a different and more useful property when the
input is known to be broken.