"""Phase P3 gates: the G1 puppet produces a log the eval can trust.

THE GATE THAT MATTERS
=====================
Stance feet must be EXACTLY stationary. Everything downstream inherits it: if a
planted foot creeps, leg odometry reports motion the filter attributes to the
robot, and every RMSE is contaminated by the gait rather than the estimator.

This gate has already earned its keep. The first implementation crept 173 um per
step -- 0.087 m/s of apparent foot velocity against sigma_leg = 0.3 m/s, i.e. 29%
of the noise budget -- while reporting an IK residual of 1e-7 m and **zero**
joint-limit violations. Two wrong things hid behind two correct numbers:

  * `_rpy_to_quat` computed `cos(angle)/2` instead of `cos(angle/2)`, so it
    returned a non-unit quaternion (norm 0.125 for a 0.02 rad pitch). MuJoCo
    applied that malformed attitude while the IK target used the correct R, so the
    sole advanced with the base at exactly the base's rate.
  * the null-space posture term wound ankle_roll to 102 rad against a 0.262 rad
    limit, with the foot still exactly on target.

So the checks are deliberately redundant: residual AND limit legality AND
world-frame stationarity. A converged solve is not a valid pose, and a valid
pose that is not measured in the world frame has told you nothing.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def g1():
    import sys
    if str(ROOT / "sim") not in sys.path:
        sys.path.insert(0, str(ROOT / "sim"))
    scene = ROOT / ".work" / "g1scene" / "scene.xml"
    if not scene.exists():
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "build_g1_scene", ROOT / "mech" / "spec" / "build_g1_scene.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.build(dest=str(scene.parent))
    model = mujoco.MjModel.from_xml_path(str(scene))
    from otolith_sim.g1_puppet import G1Puppet, G1GaitConfig
    return model, G1Puppet(model, cfg=G1GaitConfig())


def test_stance_feet_are_stationary(g1):
    """The gate. Worst consecutive world-frame step per stance foot.

    Threshold 5 um, against sigma_leg = 0.3 m/s at dt = 2 ms, i.e. 0.6 mm of
    permitted creep per step. The implementation achieves ~0.3 um, so this has
    about 15x headroom -- enough to not be flaky, far below anything that could
    contaminate a 0.3 m/s noise budget.
    """
    model, pu = g1
    d = pu.data
    worst = {leg: 0.0 for leg in pu.lm.legs}
    last = {}
    n = int(round(4.0 / (1 / 500.0)))
    for i in range(n):
        s = pu.sample(i / 500.0, 1 / 500.0)
        mujoco.mj_forward(model, d)
        for li, leg in enumerate(pu.lm.legs):
            pos = np.mean([d.geom_xpos[model.geom(nm).id]
                           for nm in pu.lm.contact_geoms[leg]], axis=0)
            if s.contacts[li]:
                if leg in last:
                    worst[leg] = max(worst[leg],
                                     float(np.linalg.norm(pos - last[leg])))
                last[leg] = pos
            else:
                last.pop(leg, None)
    for leg, w in worst.items():
        assert w < 5e-6, (
            f"{leg}: stance foot moved {w * 1e6:.3f} um between consecutive "
            f"samples = {w / 0.002:.5f} m/s of apparent foot velocity against "
            "sigma_leg = 0.3 m/s. Leg odometry would be reporting gait creep.")


def test_ik_converges_and_respects_joint_limits(g1):
    model, pu = g1
    d = pu.data
    res, viol = [], 0
    for i in range(int(round(4.0 / (1 / 500.0)))):
        pu.sample(i / 500.0, 1 / 500.0)
        mujoco.mj_forward(model, d)
        for leg in pu.lm.legs:
            res.append(pu._resid[leg])
            for j in pu.lm.chain(leg).joints:
                a = float(d.qpos[model.jnt_qposadr[model.joint(j).id]])
                r = model.jnt_range[model.joint(j).id]
                if a < r[0] - 1e-9 or a > r[1] + 1e-9:
                    viol += 1
    assert np.max(res) < 1e-5, f"worst IK residual {np.max(res) * 1000:.5f} mm"
    assert viol == 0, (
        f"{viol} joint-limit violations. The null-space posture term can wind "
        "joints without moving the foot, so a converged solve is not by itself a "
        "valid pose -- this failed once at ankle_roll = 102 rad against 0.262.")


def test_rpy_to_quat_is_unit_and_matches_mujoco(g1):
    """Regression on the bug that produced 173 um of stance creep.

    `_rpy_to_quat` used `cos(angle) / 2` rather than `cos(angle / 2)`, which is
    not a unit quaternion. MuJoCo then applied a malformed base attitude while the
    IK target was computed with the correct rotation matrix, so the sole drifted
    with the base. Invisible at zero attitude, which is the only case the earlier
    checks exercised.
    """
    model, _ = g1
    from otolith_sim.g1_puppet import _rpy_to_quat, _quat_to_mat
    d = mujoco.MjData(model)
    pel = model.body("pelvis").id
    for roll, pitch, yaw in ((0.0, 0.02, 0.0), (0.03, -0.05, 0.01),
                             (0.1, 0.2, -0.15), (-0.2, 0.35, 0.4)):
        q = _rpy_to_quat(roll, pitch, yaw)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-12, (
            f"rpy ({roll}, {pitch}, {yaw}) gave |q| = {np.linalg.norm(q)}")
        d.qpos[3:7] = q
        mujoco.mj_forward(model, d)
        assert np.abs(_quat_to_mat(q) - d.xmat[pel].reshape(3, 3)).max() < 1e-12


def test_gait_produces_both_legs_and_moves_the_base(g1):
    """Sanity: the gait is actually a gait.

    Guards against a puppet that "passes" stationarity by never planting a foot,
    which is the failure mode of every gate above.
    """
    model, pu = g1
    both, x0 = 0, None
    for i in range(int(round(2.5 / (1 / 500.0)))):
        s = pu.sample(i / 500.0, 1 / 500.0)
        both += int(s.contacts[0] and s.contacts[1])
        if x0 is None:
            x0 = s.base_pos[0]
    assert both > 0, "both feet were never simultaneously in stance"
    assert pu.sample(2.0, 1 / 500.0).base_pos[0] > x0 + 0.05, "the base never advanced"


def test_vendor_stance_seed_is_used(g1):
    """Posture comes from scene_mjx's `knees_bent`, not a guess.

    g1.xml's only keyframe is `stand`, which is the model zero pose -- a straight
    leg reaching 0.8021 m, which is why an early base_height of 0.79 left 9 mm of
    margin and the IK stalled against clipped rails.
    """
    model, pu = g1
    assert pu.cfg.base_height == pytest.approx(0.7550, abs=1e-4)
    assert np.degrees(pu.cfg.knee_nominal) == pytest.approx(38.33, abs=0.05)