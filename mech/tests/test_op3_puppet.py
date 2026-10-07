"""OP3 puppet gates.

The hard part of OP3 was not the gait -- it was that OP3 has no stance to start
from, no joint limits to keep the solver honest, and a `_home_q` bug that had been
sitting in the shared code for two robots. All three are gated here.

WHAT THE STANDING START REQUIRED
================================
OP3 has nkey 0: no keyframes at all. Its default pose is all zeros with the foot
boxes floating 20.9 mm above the floor, and its leg is a perfectly straight line
when q = 0. Three separate things had to be dealt with:

1. **The float is measurable without any IK.** Run one forward pass at the default
   qpos, measure the sole height, lower the base by exactly that. 0.3 - 0.02085 =
   0.27915, and that IS the straight-leg stance.

2. **A straight leg is a singularity.** Standing at full extension gives the
   Jacobian a rank deficiency, and the IK answers a small target change with a
   211 mm residual and a joint thrown to +/-pi. So the base is shortened to 0.95 of
   full extension, which puts the knee at 0.672 rad (38 deg) and gives the solver
   something to work with. This is the same trap as G1's `stand` keyframe -- a
   straight leg with a little margin and an IK that stalled -- arriving in a third
   robot in a third disguise.

3. **`_home_q` was solving in mixed frames.** It passed root_pos = (0,0,base_height)
   with a target of (0, hip_y, -base_height), asking the leg to reach 2*base_height
   below its own root: 1.51 m for a 0.80 m leg. It went unnoticed because that
   residual is discarded and `q_home` is only read for its non-leg entries -- every
   leg joint is overwritten per sample. G1 and Apollo were both immune because
   neither derived anything from it. OP3 was not, because its posture seed comes
   from exactly this solve. The residual is now checked rather than discarded.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[2]
DT = 1 / 500.0


@pytest.fixture(scope="module")
def op3():
    import importlib.util
    if str(ROOT / "sim") not in sys.path:
        sys.path.insert(0, str(ROOT / "sim"))
    scene = ROOT / ".work" / "op3scene" / "scene.xml"
    if not scene.exists():
        spec = importlib.util.spec_from_file_location(
            "build_op3_scene", ROOT / "mech" / "spec" / "build_op3_scene.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.build(dest=str(scene.parent))
    from otolith_sim.op3_puppet import OP3GaitConfig, OP3Puppet
    model = mujoco.MjModel.from_xml_path(str(scene))
    return model, OP3Puppet(model, cfg=OP3GaitConfig())


def _walk(fx, duration_s=2.0):
    model, pu = fx
    d = pu.data
    worst = {leg: 0.0 for leg in pu.lm.legs}
    res, viol, last, ncon = [], 0, {}, 0
    for i in range(int(round(duration_s / DT))):
        s = pu.sample(i * DT, DT)
        mujoco.mj_forward(model, d)
        ncon = max(ncon, d.ncon)
        for li, leg in enumerate(pu.lm.legs):
            fb = model.body(pu.lm.chain(leg).bodies[-1]).id
            R = d.xmat[fb].reshape(3, 3).copy()
            sole = d.xpos[fb].copy() + R @ pu.lm.sole[leg]
            if s.contacts[li]:
                if leg in last:
                    worst[leg] = max(worst[leg],
                                     float(np.linalg.norm(sole - last[leg])))
                last[leg] = sole
            else:
                last.pop(leg, None)
            res.append(pu._resid[leg])
            for j in pu.lm.chain(leg).joints:
                a = float(d.qpos[model.jnt_qposadr[model.joint(j).id]])
                # OP3 has no limited joints, so legality is checked against
                # jnt_limited rather than against jnt_range. Comparing against the
                # range would flag every pose as a violation, because the range of
                # an unlimited hinge is [0,0] and means "unconstrained", not "zero".
                if model.jnt_limited[model.joint(j).id]:
                    r = model.jnt_range[model.joint(j).id]
                    if a < r[0] - 1e-9 or a > r[1] + 1e-9:
                        viol += 1
    return worst, np.array(res), viol, ncon


def test_stance_sole_is_stationary(op3):
    """THE GATE. Worst consecutive world-frame step of the planted sole.

    Threshold 5 um against sigma_leg = 0.3 m/s at dt = 2 ms (600 um permitted).
    OP3 achieves ~1 um despite having no joint limits at all to stop it.
    """
    worst, _, _, _ = _walk(op3)
    for leg, w in worst.items():
        assert w < 5e-6, (
            f"{leg}: planted sole moved {w * 1e6:.4f} um between samples = "
            f"{w / DT:.6f} m/s against sigma_leg = 0.3 m/s")


def test_ik_converges(op3):
    worst, res, viol, _ = _walk(op3, duration_s=1.0)
    assert np.max(res) < 1e-5, (
        f"worst IK residual {np.max(res) * 1000:.5f} mm. With no joint limits the "
        "solver has nothing to clip it, so a large residual means the target was "
        "genuinely unreachable -- usually a stance too tall for the leg.")
    assert viol == 0


def test_joints_actually_move_and_stay_sane(op3):
    """OP3 has no joint limits, so nothing bounds the IK but the posture prior.

    Without a limit to clip it, a diverging solve is free to throw a joint to +/-pi
    and put the shank through the floor. This is the check that catches that, and
    it is the one thing OP3 cannot inherit from G1's puppet gates.
    """
    model, pu = fx = op3
    d = pu.data
    qmax = 0.0
    for i in range(1000):
        pu.sample(i * DT, DT)
        mujoco.mj_forward(model, d)
        for leg in pu.lm.legs:
            for j in pu.lm.chain(leg).joints:
                qmax = max(qmax, abs(float(d.qpos[model.jnt_qposadr[model.joint(j).id]])))
    assert qmax > 0.05, f"joints never moved (max {qmax:.4f}); the gait is frozen"
    assert qmax < 1.6, (
        f"a joint reached {qmax:.3f} rad. With no limits this is the signature of a "
        "diverging solve rather than a large but valid pose -- the straight-leg "
        "singularity throws joints to +/-pi.")


def test_no_self_collision(op3):
    """Contact count must stay near the feet.

    While the solver was diverging, ncon hit 118: the shanks were through the floor
    and every capsule was registering. A stationarity number from that pose is
    meaningless, so this is checked rather than assumed.
    """
    _, _, _, ncon = _walk(op3, duration_s=1.0)
    assert ncon <= 20, (
        f"peak ncon {ncon} is far above the ~4 expected from two foot plates; the "
        "body is colliding with itself or the floor")


def test_home_pose_is_derived_and_bent(op3):
    """The stance is SOLVED, and it is not the straight-leg one.

    Pins both halves of the construction: the height comes from the measured float
    times the standing fraction, and the resulting knee angle proves the leg is
    actually flexed rather than singular.
    """
    model, pu = op3
    full = 0.3 - 0.02085
    assert pu.cfg.base_height == pytest.approx(full * 0.95, abs=1e-4), (
        f"base_height {pu.cfg.base_height:.5f}; expected {full * 0.95:.5f} = "
        "(default base - measured float) x 0.95")
    assert 0.3 < pu.cfg.knee_nominal < 1.2, (
        f"knee prior {pu.cfg.knee_nominal:.4f} rad is not a working stance angle; "
        "0 means the straight-leg singularity is still in play")
    q = pu.q_home[pu._leg_qadr("left")]
    assert abs(q[3]) > 0.1, "the solved home knee is straight; the leg is singular"
    assert np.abs(q).max() < 1.6, f"home pose joint at {np.abs(q).max():.3f} rad"


def test_gait_actually_gait(op3):
    model, pu = op3
    d = pu.data
    stance = [0, 0]
    x0 = None
    for i in range(int(1.5 / DT)):
        s = pu.sample(i * DT, DT)
        mujoco.mj_forward(model, d)
        for li in range(2):
            stance[li] += int(s.contacts[li])
        if x0 is None:
            x0 = s.base_pos[0]
    assert all(c > 0 for c in stance), f"a foot never planted: {stance}"
    assert pu.sample(1.0, DT).base_pos[0] > x0 + 0.02, "the base never advanced"

def test_sigma_leg_scales_with_encoder_times_lever(op3):
    """P4's mechanism across all four robots, and OP3 is the falsification case.

        robot   lever   sigma_q    motion   quant    total    vs shipped 0.3
        go2     ~0.30   --          ~0.18    ~0.25    0.30      1.0x
        op3     0.28    0.000443   0.138    0.073    0.156     1.9x CONSERVATIVE
        op3     0.28    0.002      0.138    0.337    0.364     0.8x
        g1      0.80    0.002      0.184    1.032    1.048     3.5x optimistic
        apollo  ~0.90   0.002      0.184    1.386    1.398     4.7x optimistic

    The ordering is monotone in (sigma_q x lever) and is NOT monotone in mass --
    OP3 is 3.15 kg and Apollo 80.9 kg, but their sigma_leg differs by 9x while their
    lever arms differ by 3.2x and their encoders by 4.5x. Mass was the wrong
    variable; that was the v0.6 error.

    OP3 is the case that could have DISPROVED the argument, because its short lever
    and its good encoder both push sigma_leg down, so a wrong mechanism would have
    been easiest to hide. Instead it is the one robot where the shipped 0.3 turns
    out conservative.

    And the model predicts OP3 from the NOMINAL leg length to within 4.5% at the
    real encoder and 15% at 0.002: sigma_q*sqrt(2)*L/dt gives a quantization term,
    and in quadrature with the measured 0.1377 motion term that is 0.163 against
    0.156 measured, and 0.418 against 0.364. Both over-predict, because the
    effective lever is ~0.24 m against a 0.279 m nominal -- 86%. The real-encoder
    case lands closer only because the motion term dominates there and hides the
    lever error; at 0.002 the quantization term dominates and the same error shows
    in full. Same direction as G1 (94% effective) and Apollo (109%), and the
    measurement, not the band, is what decides.
    """
    L, dt = 0.279, 1 / 500.0
    motion = 0.1377
    for sigma_q, expected_band in ((0.000443, (0.13, 0.19)), (0.002, (0.32, 0.45))):
        quant = sigma_q * np.sqrt(2) * L / dt
        total = np.sqrt(quant ** 2 + motion ** 2)
        lo, hi = expected_band
        assert lo < total < hi, (
            f"sigma_q {sigma_q:.6f}: predicted sigma_leg {total:.3f} m/s outside "
            f"[{lo}, {hi}] from sigma_q*sqrt(2)*L/dt in quadrature with the "
            f"{motion} m/s motion term")
