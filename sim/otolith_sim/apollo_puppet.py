"""Apollo's puppet: the shared 6-DoF biped gait plus Apollo's descriptor.

Apollo is the easy case for seeding, which is worth saying because G1 is not:
its `stand` keyframe is a REAL pose -- base z 1.01597, hip_fe -0.477, knee_fe
+1.033, ankle_pd -0.580 -- not the model zero pose that G1's `stand` is. It still
cannot be used as-is, because its sole boxes sit 1.19 mm BELOW the floor at that
pose. So the vendor angles seed the posture, and the height gets solved onto the
ground instead.

The height solve matters more than it looks. Apollo is a 1.73 m robot on 0.79 m
legs, and `_home_q` searches for the base height that puts both feet flat. Too
high and the solver stalls against clipped joint rails, which is precisely how
G1's early runs died; too low and the knees splay past their 2.618 rad ceiling.
"""
from __future__ import annotations

from dataclasses import dataclass

from otolith_sim.biped_puppet import BipedGaitConfig, BipedPuppet
from otolith_sim.leg_model import load_apollo

MENAGERIE = "third_party/menagerie/apptronik_apollo"


@dataclass
class ApolloGaitConfig(BipedGaitConfig):
    """Apollo seeds from its own `stand` keyframe; base_height is then solved."""
    base_height: float = 1.01597      # vendor stand qpos[2]; 1.19 mm too high
    # Apollo's vocabulary is not G1's. `hip_pitch` matches NOTHING on Apollo, and
    # a silently unmatched hip leaves its posture prior unset -- which still looks
    # like a plausible pose, just a converged-from-nowhere one.
    knee_match: tuple = ("knee_fe",)
    hip_pitch_match: tuple = ("hip_fe",)


class ApolloPuppet(BipedPuppet):
    """Prescribes a quasi-static Apollo gait. Ground truth is the prescription."""

    def __init__(self, model, data=None, cfg: ApolloGaitConfig | None = None):
        super().__init__(model, load_apollo(model), cfg or ApolloGaitConfig(), data)

    def _vendor_stance(self, key="stand"):
        """Seed posture from Apollo's own `stand` keyframe.

        Unlike G1, which has to reach into a second file (scene_mjx.xml) because
        g1.xml's only keyframe is the zero pose, Apollo carries a usable one. It
        is a real stance -- knee 59 deg, hip -27 deg -- but its sole boxes are
        1.19 mm through the floor, so the BASE HEIGHT is discarded and re-solved
        by `_home_q` while the joint posture is kept.
        """
        import mujoco as _mj
        if self.model.nkey == 0:
            return
        kid = next((i for i in range(self.model.nkey)
                    if _mj.mj_id2name(self.model, _mj.mjtObj.mjOBJ_KEY, i) == key), None)
        if kid is None:
            return
        d = _mj.MjData(self.model)
        _mj.mj_resetDataKeyframe(self.model, d, kid)
        _mj.mj_forward(self.model, d)
        self._vendor_q = {}
        for leg in self.lm.legs:
            self._vendor_q[leg] = [
                float(d.qpos[self.model.jnt_qposadr[self.model.joint(j).id]])
                for j in self.lm.chain(leg).joints]
        joints = self.lm.chain(self.lm.legs[0]).joints
        first = self._vendor_q[self.lm.legs[0]]
        for i, jn in enumerate(joints):
            short = jn.lstrip("lr_")
            if any(k in short for k in self.cfg.knee_match):
                self.cfg.knee_nominal = first[i]
            elif any(k in short for k in self.cfg.hip_pitch_match):
                self.cfg.hip_pitch_nominal = first[i]
        # Height is NOT taken from the keyframe -- see the docstring.
        self.cfg.base_height = float(d.qpos[2])
