"""Adapters presenting every robot through one interface for `sim_node`.

`sim_node.py` was written against `Go2Puppet` and reads four Go2-specific things:
a fixed `FOOT_ORDER`, joint names built as `{leg}_{hip,thigh,calf}_joint`, joint
angles read through `puppet.legs[leg].qpos_adr`, and a `sample(model, data, t, dt)`
signature that takes the MuJoCo model and data.

The biped puppets have a different shape on all four: chain-ordered joints named
from the descriptor (`l_hip_ie`, ...), angles read by name, and `sample(t, dt)`
which owns its own MjData. Rather than teach `sim_node` about both, each robot gets
an adapter exposing one interface.

That indirection is the point. v0.5 spent a lot of effort proving that a biped's
chain order is the tree topology and that mirroring rules differ per robot; a sim
node that reconstructed joint names by string formatting would quietly reintroduce
exactly those assumptions. Here the names come from the descriptor, which is
already gated against MuJoCo.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from otolith_sim.biped_puppet import BipedPuppet
from otolith_sim.leg_model import load_apollo, load_g1, load_op3
from otolith_sim.puppet import Go2Puppet, GaitConfig

@dataclass
class RobotAdapter:
    """One interface over Go2Puppet and BipedPuppet."""

    name: str
    model: mujoco.MjModel
    scene: str
    n_legs: int
    joint_names: tuple[str, ...]

    def sample(self, data, t: float, dt: float):
        raise NotImplementedError


class Go2Adapter(RobotAdapter):
    def __init__(self, model, scene):
        self.puppet = Go2Puppet(model, GaitConfig())
        super().__init__("go2", model, scene, 4,
                         tuple(f"{leg}_{p}_joint"
                               for leg in ("FL", "FR", "RL", "RR")
                               for p in ("hip", "thigh", "calf")))

    def sample(self, data, t, dt):
        return self.puppet.sample(self.model, data, t, dt)

    def joint_q(self, sample):
        # Go2 exposes addresses directly; keep using them rather than by name.
        return np.array([sample.qpos[adr]
                         for leg in ("FL", "FR", "RL", "RR")
                         for adr in self.puppet.legs[leg].qpos_adr])

    def contacts(self, sample):
        return np.asarray(sample.contacts, dtype=float)


class BipedAdapter(RobotAdapter):
    """A 6-DoF biped: G1, Apollo or OP3.

    Contacts are published 4-wide with slots 2 and 3 zeroed, because
    `fusion_node` reads `msg->data[i]` for i < 4 unconditionally -- a 2-element
    message would be an out-of-bounds read. A biped only uses slots 0 and 1, so the
    padding is inert. Same convention as `logger.record_biped_log`.
    """

    def __init__(self, model, scene, robot: str, replay: str | None = None):
        self.robot = robot
        loaders = {"g1": load_g1, "apollo": load_apollo, "op3": load_op3}
        if robot not in loaders:
            raise ValueError(f"unknown biped {robot}; known: {sorted(loaders)}")
        self.lm = loaders[robot](model)
        cfg = {"g1": "G1GaitConfig", "apollo": "ApolloGaitConfig",
               "op3": "OP3GaitConfig"}[robot]
        mod = {"g1": "g1_puppet", "apollo": "apollo_puppet",
               "op3": "op3_puppet"}[robot]
        import importlib
        puppet_cls = getattr(importlib.import_module(f"otolith_sim.{mod}"), "G1Puppet"
                             if robot == "g1" else
                             "ApolloPuppet" if robot == "apollo" else "OP3Puppet")
        config_cls = getattr(importlib.import_module(f"otolith_sim.{mod}"), cfg)
        if replay:
            # Streamed instead of solved. See bake_replay.py for why: the filter is
            # not the bottleneck (1009 us of a 2000 us budget), the numpy DLS IK is
            # (21.95 ms/sample = 46 Hz ceiling).
            from otolith_sim.bake_replay import ReplayTraj
            self.puppet = ReplayTraj(replay)
            if self.puppet.qpos.shape[1] != model.nq:
                raise ValueError(
                    f"replay has nq={self.puppet.qpos.shape[1]} but the scene has "
                    f"nq={model.nq} -- the trajectory was baked from a different model")
        else:
            self.puppet = puppet_cls(model, cfg=config_cls())
        names = tuple(j for leg in self.lm.legs
                      for j in self.lm.chain(leg).joints)
        super().__init__(robot, model, scene, self.lm.n_legs, names)

    def sample(self, data, t, dt):
        s = self.puppet.sample(t, dt)
        # The puppet keeps its own MjData; publish on the caller's so that callers
        # reading `data` see the same state they just sampled.
        data.qpos[:] = s.qpos
        data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, data)
        return s

    def joint_q(self, sample):
        return np.array([sample.qpos[self.model.jnt_qposadr[self.model.joint(j).id]]
                         for j in self.joint_names])

    def contacts(self, sample):
        c = np.zeros(4)
        c[:2] = np.asarray(sample.contacts, dtype=float)
        return c

    @property
    def replaying(self) -> bool:
        return hasattr(self.puppet, "qpos")


def make_adapter(robot: str, scene: str | None = None,
                 replay: str | None = None) -> RobotAdapter:
    """Build an adapter for `go2`, `g1`, `apollo` or `op3`.

    Scene defaults come from the menagerie symlink. OP3 uses the PATCHED scene
    because the vendor file has no named geoms and `load_op3` reads contact-geom
    names; see mech/spec/build_op3_scene.py.
    """
    if robot == "go2":
        scene = scene or "third_party/menagerie/unitree_go2/scene.xml"
        return Go2Adapter(mujoco.MjModel.from_xml_path(scene), scene)
    # go2 needs no replay: its closed-form 2R IK is fast enough to solve live.
    default = {
        "g1": ".work/g1scene/scene.xml",
        "apollo": "third_party/menagerie/apptronik_apollo/scene.xml",
        "op3": ".work/op3scene/scene.xml",
    }[robot]
    scene = scene or default
    return BipedAdapter(mujoco.MjModel.from_xml_path(scene), scene, robot, replay)


ROBOT_CHOICES = ("go2", "g1", "apollo", "op3")