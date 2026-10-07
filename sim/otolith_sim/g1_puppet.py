"""G1's puppet: the shared 6-DoF biped gait plus G1's descriptor and stance seed.

The gait, the DLS IK and every stationarity guarantee live in
`sim/otolith_sim/biped_puppet.py`. This module supplies only what is specific to
G1: its leg descriptor, its joint-name vocabulary, and the vendor stance pose.

G1's seed is awkward and is kept here rather than pushed into the base class,
because it is not a property of bipeds: `g1.xml`'s only keyframe is `stand`, and
that is the model ZERO pose -- a straight leg reaching 0.8021 m, with its feet
1.86 mm BELOW the floor. The usable stance lives in `scene_mjx.xml` as
`knees_bent`. Apollo's situation is the opposite, and simpler: its `stand`
keyframe is a real pose (base z 1.01597) that penetrates by 1.19 mm. So the base
class takes a `_vendor_stance()` hook and each robot answers it for itself.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from otolith_sim.biped_puppet import (  # noqa: F401  (re-exported)
    BipedGaitConfig,
    BipedPuppet,
    BipedSample,
    _leg_jacobian,
    _quat_to_mat,
    _rpy_to_quat,
    joint_limits,
    solve_leg_ik,
)
from otolith_sim.leg_model import load_g1


@dataclass
class G1GaitConfig(BipedGaitConfig):
    """G1's seed: knees_bent from scene_mjx.xml, applied by G1Puppet._vendor_stance.

    NOT the `stand` keyframe of g1.xml. That is the model zero pose, a straight
    leg, reaching 0.8021 m and leaving 9 mm of margin for any forward or lateral
    offset -- one foothold offset from unreachable, which the DLS solver then
    papers over with a residual rather than an error. Sampling 40k legal poses
    gives the envelope; the sole spans z = -0.8021..+0.5805.
    """
    base_height: float = 0.7550        # knees_bent qpos[2]
    knee_nominal_hint: float = 0.669   # knees_bent knee angle, radians
    knee_match: tuple = ("knee",)
    hip_pitch_match: tuple = ("hip_pitch",)


class G1Puppet(BipedPuppet):
    """Prescribes a quasi-static G1 gait. Ground truth is the prescription."""

    def __init__(self, model, data=None, cfg: G1GaitConfig | None = None):
        super().__init__(model, load_g1(model), cfg or G1GaitConfig(), data)

    def _vendor_stance(self, mjx="third_party/menagerie/unitree_g1/scene_mjx.xml",
                       key="knees_bent"):
        """Seed posture and base height from the vendor `knees_bent` keyframe.

        Read from scene_mjx.xml rather than solved, because that file ships two
        real stance poses (home: knee 17.19 deg, knees_bent: knee 38.33 deg) where
        g1.xml's only `stand` is the all-zeros straight-leg zero pose. The joint
        angles transfer directly -- both files describe the same robot with the
        same joint and body naming.

        The geoms are NOT taken from scene_mjx: all eight foot geoms there carry
        contype=0 / conaffinity=0, because MJX uses explicit contact pairs that
        the stock scene does not enable. So contact stays the four collidable
        spheres that build_g1_scene.py names in g1.xml.
        """
        from pathlib import Path
        path = Path(mjx)
        if not path.exists():
            return
        import mujoco as _mj
        src = _mj.MjModel.from_xml_path(str(path))
        kid = next((i for i in range(src.nkey)
                    if _mj.mj_id2name(src, _mj.mjtObj.mjOBJ_KEY, i) == key), None)
        if kid is None:
            return
        d = _mj.MjData(src)
        _mj.mj_resetDataKeyframe(src, d, kid)
        _mj.mj_forward(src, d)
        self.cfg.base_height = float(d.qpos[2])
        self._vendor_q = {}
        for leg in self.lm.legs:
            self._vendor_q[leg] = np.array(
                [float(d.qpos[self.model.jnt_qposadr[self.model.joint(j).id]])
                 for j in self.lm.chain(leg).joints])
        knee = next(i for i, j in enumerate(self.lm.chain(self.lm.legs[0]).joints)
                    if "knee" in j)
        self.cfg.knee_nominal = float(self._vendor_q[self.lm.legs[0]][knee])
        hp = next((i for i, j in enumerate(self.lm.chain(self.lm.legs[0]).joints)
                   if "hip_pitch" in j), None)
        if hp is not None:
            self.cfg.hip_pitch_nominal = float(self._vendor_q[self.lm.legs[0]][hp])


import numpy as np  # noqa: E402  (used by _vendor_stance above)