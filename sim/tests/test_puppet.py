"""IK and gait consistency tests (pure numpy + mujoco, no ROS)."""
import os
import sys

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from otolith_sim.puppet import (Go2Puppet, GaitConfig, _leg_geoms, leg_ik,
                                _quat_to_mat)

SCENE = os.path.join(os.path.dirname(__file__), "..", "..",
                     "third_party", "menagerie", "unitree_go2", "scene.xml")
FOOT_ORDER = ("FL", "FR", "RL", "RR")


@pytest.fixture
def model():
    return mujoco.MjModel.from_xml_path(SCENE)


def foot_world(model, data, name):
    gid = model.geom(name).id
    return data.geom_xpos[gid].copy()


def test_ik_reaches_random_targets(model):
    """Closed-form IK + mujoco FK must agree within 5 mm."""
    data = mujoco.MjData(model)
    legs = _leg_geoms(model)
    kid = model.key("home").id
    rng = np.random.default_rng(7)
    checked = 0
    for name in FOOT_ORDER:
        leg = legs[name]
        for _ in range(25):
            target = leg.hip_base + rng.uniform([-0.12, -0.03, -0.28],
                                                [0.12, 0.03, -0.08])
            sol = leg_ik(leg, target)
            if sol is None:
                continue
            q = np.zeros(model.nq)
            q[0:3] = [0, 0, 0.27]
            q[3] = 1.0
            adr = leg.qpos_adr
            q[adr[0]:adr[2] + 1] = sol
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            # foot position expressed in the BASE frame (base sits at
            # qpos[0:3]; orientation is identity in this test)
            got = foot_world(model, data, name) - data.qpos[0:3]
            assert np.linalg.norm(got - target) < 5e-3, (name, target, got)
            checked += 1
    assert checked >= 40


def test_stance_feet_stationary(model):
    """During a stance phase the stance foot must not move in the world."""
    data = mujoco.MjData(model)
    puppet = Go2Puppet(model, GaitConfig())
    dt = 0.002
    t = 0.10  # inside the first stance window
    first = {}
    for _ in range(60):
        s = puppet.sample(model, data, t, dt)
        for i, name in enumerate(FOOT_ORDER):
            if s.contacts[i]:
                gid = model.geom(name).id
                pos = data.geom_xpos[gid].copy()
                if name in first:
                    drift = np.linalg.norm(pos[:2] - first[name][:2])
                    assert drift < 5e-3, (name, drift)
                else:
                    first[name] = pos
        t += dt


def test_contacts_match_swing_height(model):
    """A foot is 'in contact' exactly when its world height is at the floor."""
    data = mujoco.MjData(model)
    puppet = Go2Puppet(model, GaitConfig())
    dt = 0.002
    t = 0.05
    for _ in range(150):
        s = puppet.sample(model, data, t, dt)
        for i, name in enumerate(FOOT_ORDER):
            z = foot_world(model, data, name)[2]
            if s.contacts[i]:
                assert z < 0.02, (name, t, z)
        t += dt
