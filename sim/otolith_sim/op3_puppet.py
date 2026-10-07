"""OP3's puppet: the shared 6-DoF biped gait, with a home pose solved from nothing.

OP3 is the awkward case for seeding, and in the opposite way to G1:

  * G1 has no usable stance in its own model but a `knees_bent` keyframe in
    scene_mjx.xml to borrow.
  * Apollo has a real `stand` keyframe that penetrates 1.19 mm.
  * OP3 has **no keyframes at all** (nkey 0) and an all-zero default pose whose
    foot boxes float 20.9 mm above the floor. There is nothing to borrow and
    nothing to nudge -- the home pose has to be found.

`_home_q` in the base class already does the finding: it solves for the joint
angles that put both feet flat at a given base height. The only OP3-specific
decision is what base height to ask for.

WHY THE BASE HEIGHT IS SOLVED BY BISECTION RATHER THAN ASSUMED. OP3 has NO joint
limits, so there is no rail for the solver to stall against and no range to check
the answer against -- the failure mode that bit G1 (a straight leg with 9 mm of
margin, clipping at 38 mm residual) is not available here. That removes the usual
guard and replaces it with a different one: with no limits, a wrong base height
simply gives a wrong-but-converging pose. So the height is found by bisection on
the actual geometric constraint -- both soles at z = 0 -- rather than taken from a
number that happens to look plausible.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from otolith_sim.biped_puppet import BipedGaitConfig, BipedPuppet
from otolith_sim.leg_model import load_apollo  # noqa: F401  (documented sibling)
from otolith_sim.leg_model import load_op3

# The leg's reach sets the bracket. hip_yaw sits at z = 0.30 in the default pose
# and each leg segment is ~0.11 m, so a standing base is near 0.24-0.28 m.
# Fraction of full extension to stand at. NOT 1.0.
#
# The bisection below finds the height at which the feet are flat AND every joint is
# at zero -- OP3's authored pose, a perfectly straight leg. That is a KINEMATIC
# SINGULARITY: the Jacobian loses a rank, the IK has no authority in the direction
# that matters, and it answers a small target change with a 211 mm residual and a
# joint thrown to +/-pi. Standing slightly short forces the knee to flex and gives
# the solver something to work with.
#
# This is the same trap as G1's `stand` keyframe -- a straight leg, 9 mm of margin,
# and an IK that stalled at 38 mm residual -- arriving a third time in a third
# robot. 0.95 keeps the knee at a working angle without dropping the base so far
# that the squat has no travel left.
_STAND_FRACTION = 0.95


@dataclass
class OP3GaitConfig(BipedGaitConfig):
    """OP3 has no vendor stance; base_height is solved by OP3Puppet._vendor_stance."""
    base_height: float = 0.25        # placeholder, overwritten by the solve
    # Vocabulary: OP3's ankle is `ank_` not `ankle_`, and its hip pitch is
    # `hip_pitch` -- which happens to match G1's matcher. Asserted in the gates
    # rather than assumed, since a silent miss leaves the posture prior unset and
    # still converges to a plausible-looking pose.
    knee_match: tuple = ("knee",)
    hip_pitch_match: tuple = ("hip_pitch",)
    # OP3 is a third of G1's size, so the same gait amplitudes are wildly out of
    # scale: an 80 mm sway on a 70 mm stance walks the base clean off the feet.
    # Everything below is scaled to the robot rather than shared.
    sway_amplitude: float = 0.030
    squat_amplitude: float = 0.010
    step_height: float = 0.010
    stride: float = 0.05
    cycle_s: float = 1.0


class OP3Puppet(BipedPuppet):
    """Prescribes a quasi-static OP3 gait. Ground truth is the prescription."""

    def __init__(self, model, data=None, cfg: OP3GaitConfig | None = None):
        super().__init__(model, load_op3(model), cfg or OP3GaitConfig(), data)

    # ------------------------------------------------------------------
    # Stance seeding
    # ------------------------------------------------------------------
    def _vendor_stance(self):
        """Find a stance that is not in the model, from geometry alone.

        OP3 has no keyframes and an all-zero default pose whose foot boxes float
        20.9 mm above the floor. The float is therefore measurable directly: run one
        forward pass at the default qpos, measure the sole height, and lower the
        base by exactly that. No IK involved.

        A first attempt BISECTED the base height on the sole constraint instead. That
        is worse than useless once the joint limits are fixed, because `_sole_z_at`
        is then the residual of an arbitrary IK solution rather than a monotone
        function: it has several roots, and the search converged to 0.15 m with the
        legs folded and the shanks self-colliding (63 contacts). Bisecting on a
        function whose solver is part of the integrand cannot be trusted.

        Then the geometric stance -- a perfectly straight leg, joints all zero -- is
        a KINEMATIC SINGULARITY, so the base is shortened by _STAND_FRACTION to put
        the knee at a working angle. See _STAND_FRACTION.
        """
        d = self.data
        d.qpos[:] = 0.0
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, d)
        float_z = float(np.mean([
            d.geom_xpos[self.model.geom(n).id][2]
            - self.model.geom_size[self.model.geom(n).id][2]
            for leg in self.lm.legs for n in self.lm.contact_geoms[leg]]))
        base_z_default = float(d.qpos[2])
        straight = base_z_default - float_z
        self.cfg.base_height = straight * _STAND_FRACTION

        self.q_home = self._home_q()
        self._vendor_q = {leg: np.array(self.q_home[self._leg_qadr(leg)], dtype=float)
                          for leg in self.lm.legs}
        # Mirror the solved pose onto the cfg fields too, so the numbers a reader
        # inspects match the posture actually used. `_nominal_q` consults _vendor_q
        # first, so this is bookkeeping -- but a cfg claiming knee_nominal 0.6 while
        # the prior is 0 is the kind of discrepancy that later reads as a bug.
        joints = self.lm.chain(self.lm.legs[0]).joints
        first = self._vendor_q[self.lm.legs[0]]
        for i, jn in enumerate(joints):
            short = jn.lstrip("lr_")
            if any(k in short for k in self.cfg.knee_match):
                self.cfg.knee_nominal = float(first[i])
            elif any(k in short for k in self.cfg.hip_pitch_match):
                self.cfg.hip_pitch_nominal = float(first[i])

    def _leg_qadr(self, leg: str):
        """qpos addresses for one leg's joints, in chain order."""
        return [self.model.jnt_qposadr[self.model.joint(j).id]
                for j in self.lm.chain(leg).joints]
