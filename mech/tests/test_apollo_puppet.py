"""Apollo puppet gates: does the shared 6-DoF gait produce a log worth trusting?

THE GATE: the planted sole must be exactly stationary, and it must be measured at
the SOLE. Both halves of that sentence were learned the hard way.

WHAT TO MEASURE. Apollo's foot is a single box whose contact face is its BOTTOM,
9 mm below the geom centre. `geom_xpos` reports the centre, so measuring there is
measuring the wrong point -- and measuring the foot BODY origin is worse still:
during stance the foot pivots about its sole, which moves the body origin by
43.6 um while the sole itself moves by 7 nm. All three numbers appear in
test_measure_the_sole_not_the_body below, because the temptation to report the
43.6 um one is strong and it looks alarming.

WHY THE FOOT PIVOTS AT ALL. The IK re-solves from the nominal posture every sample
and its null-space term is free to rotate the foot as long as the sole stays put.
That is correct behaviour, not creep -- the sole is what the ground constrains --
but it is why the joint angles wander slightly during stance, and it is the same
mechanism that used to wind G1's ankle_roll to 102 rad against a 0.262 rad limit.

The shared gait and its guarantees live in sim/otolith_sim/biped_puppet.py; this
file checks that Apollo's descriptor and seed feed it correctly.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def apollo():
    if str(ROOT / "sim") not in sys.path:
        sys.path.insert(0, str(ROOT / "sim"))
    from otolith_sim.apollo_puppet import ApolloGaitConfig, ApolloPuppet
    from otolith_sim.leg_model import load_apollo
    model = mujoco.MjModel.from_xml_path(
        str(ROOT / "third_party/menagerie/apptronik_apollo/scene.xml"))
    return model, ApolloPuppet(model, cfg=ApolloGaitConfig()), load_apollo(model)


DT = 1 / 500.0


def _walk(fx, duration_s=3.0, dt=DT):
    """Step the shared gait, returning per-leg worst stance steps and diagnostics."""
    model, pu, lm = fx
    d = pu.data
    fb = {leg: model.body(lm.chain(leg).bodies[-1]).id for leg in lm.legs}
    gid = {leg: model.geom(lm.contact_geoms[leg][0]).id for leg in lm.legs}
    worst_sole = {leg: 0.0 for leg in lm.legs}
    worst_box = {leg: 0.0 for leg in lm.legs}
    worst_body = {leg: 0.0 for leg in lm.legs}
    res, viol, last, peak_ncon = [], 0, {}, 0
    n = int(round(duration_s / dt))
    for i in range(n):
        s = pu.sample(i * dt, dt)
        mujoco.mj_forward(model, d)
        peak_ncon = max(peak_ncon, d.ncon)
        for li, leg in enumerate(lm.legs):
            R = d.xmat[fb[leg]].reshape(3, 3).copy()
            cur = {
                "sole": d.xpos[fb[leg]].copy() + R @ lm.sole[leg],
                "box": d.geom_xpos[gid[leg]].copy(),
                "body": d.xpos[fb[leg]].copy(),
            }
            if s.contacts[li]:
                if leg in last:
                    for k, acc in (("sole", worst_sole), ("box", worst_box),
                                   ("body", worst_body)):
                        acc[leg] = max(acc[leg],
                                       float(np.linalg.norm(cur[k] - last[leg][k])))
                last[leg] = cur
            else:
                last.pop(leg, None)
            res.append(pu._resid[leg])
            for j in lm.chain(leg).joints:
                a = float(d.qpos[model.jnt_qposadr[model.joint(j).id]])
                r = model.jnt_range[model.joint(j).id]
                if a < r[0] - 1e-9 or a > r[1] + 1e-9:
                    viol += 1
    return worst_sole, worst_box, worst_body, np.array(res), viol, peak_ncon


def test_stance_sole_is_stationary(apollo):
    """THE GATE. Worst consecutive world-frame step of the planted sole.

    Threshold 5 um, against sigma_leg = 0.3 m/s at dt = 2 ms, i.e. 600 um of
    permitted creep per step. Apollo achieves ~0.007 um, so this has ~700x
    headroom -- tighter than G1's ~0.3 um relative to its own margin, and for a
    different reason: Apollo's foot pivots about its sole during stance, which
    costs nothing at the sole but is what a body-origin measurement would report
    as 43.6 um.
    """
    worst_sole, _, _, _, _, _ = _walk(apollo)
    for leg, w in worst_sole.items():
        assert w < 5e-6, (
            f"{leg}: planted sole moved {w * 1e6:.4f} um between consecutive "
            f"samples = {w / 0.002:.6f} m/s against sigma_leg = 0.3 m/s")


def test_ik_converges_and_respects_joint_limits(apollo):
    """Redundant with the stationarity gate on purpose.

    A converged solve is not a valid pose: G1's null-space term once wound
    ankle_roll to 102 rad against a 0.262 rad limit with the foot still exactly on
    target, and both the residual and the stationarity check passed.
    """
    _, _, _, res, viol, _ = _walk(apollo, duration_s=1.5)
    assert np.max(res) < 1e-5, f"worst IK residual {np.max(res) * 1000:.5f} mm"
    assert viol == 0, f"{viol} joint-limit violations"


def test_apollo_really_touches_the_ground(apollo):
    """Guards against a puppet that passes stationarity by never planting a foot.

    Unlike G1, Apollo's vendor scene has live contact: scene.xml pairs both soles
    against the floor explicitly, and every geom is contype=0, so this is the only
    place that fact gets checked.
    """
    model, pu, lm = apollo
    d = pu.data
    peak = 0
    for i in range(750):
        pu.sample(i * DT, DT)
        mujoco.mj_forward(model, d)
        peak = max(peak, d.ncon)
    assert peak > 0, (
        "no contacts ever generated. scene.xml pairs the soles against the floor "
        "explicitly, so a zero ncon means the pairs are gone, not that the "
        "geometry is wrong.")
    both = 0
    for i in range(750):
        s = pu.sample(i * DT, DT)
        both += int(s.contacts[0] and s.contacts[1])
    assert both > 0, "both feet were never simultaneously in stance"


def test_measure_the_sole_not_the_body(apollo):
    """Records all three measurement points, because they disagree by 4 orders.

        planted sole      0.007 um   <- what the ground constrains, and the gate
        box geom centre   5.7   um   <- 9 mm above the sole
        foot body origin 43.6   um   <- 47 mm above the sole

    The foot pivots about its sole during stance, so everything above the contact
    face swings. That is real physics, not creep, and the gap is exactly what a
    body-origin measurement would have misreported as a 43.6 um stationarity
    failure -- 0.15 of the entire sigma_leg budget, from choosing the wrong point.

    This is the same family as the box-centre-vs-bottom-face slip in the FK gate,
    and the reason the assertion above is at the sole rather than at the geom.
    """
    worst_sole, worst_box, worst_body, _, _, _ = _walk(apollo, duration_s=1.5)
    sole = max(worst_sole.values())
    box = max(worst_box.values())
    body = max(worst_body.values())
    assert sole < 5e-6, f"sole not stationary: {sole * 1e6:.4f} um"
    # The three must differ, or this test is not measuring what it claims.
    assert body > 10 * sole, (
        f"body origin {body * 1e6:.3f} um vs sole {sole * 1e6:.4f} um -- expected "
        "the foot to visibly pivot about its sole; if not, the sole offset and the "
        "body origin have converged and the gate above is measuring the wrong thing")
    assert box > sole, "box centre should sit 9 mm above the sole and move further"


def test_vendor_seed_is_used_and_the_vocabulary_matches(apollo):
    """Posture comes from Apollo's own `stand` keyframe, and the seed really lands.

    The joint-name vocabulary is the part worth gating. G1's config matches
    `hip_pitch`; Apollo's hip is `hip_fe`, so a config carried over unchanged
    matches nothing and silently leaves the hip posture prior at its default. That
    still converges to a plausible pose -- from nowhere -- which is why it needs an
    assertion rather than a look.
    """
    model, pu, lm = apollo
    assert pu.cfg.knee_nominal == pytest.approx(1.033, abs=1e-3), (
        "knee_fe prior should come from the vendor stand keyframe (1.033 rad)")
    assert pu.cfg.hip_pitch_nominal == pytest.approx(-0.477, abs=1e-3), (
        "hip_fe prior did NOT come from the vendor stand (-0.477 rad expected) -- "
        "the hip_pitch_match vocabulary is wrong for Apollo")
    assert pu.cfg.base_height == pytest.approx(1.01597, abs=1e-4)
    # And the matchers must actually match this robot's joint names.
    joints = lm.chain("left").joints
    assert any(any(k in j for k in pu.cfg.knee_match) for j in joints)
    assert any(any(k in j for k in pu.cfg.hip_pitch_match) for j in joints), (
        "hip_pitch_match matches no Apollo joint; the posture prior would be "
        "applied to nothing")


def test_gait_actually_gait(apollo):
    """Sanity: the base advances and both feet take turns.

    Guards the whole file, since every gate above would pass on a puppet that
    stands still and never plants anything.
    """
    model, pu, lm = apollo
    d = pu.data
    stance = [0, 0]
    x0 = None
    for i in range(int(2.5 / DT)):
        s = pu.sample(i * DT, DT)
        mujoco.mj_forward(model, d)
        for li in range(2):
            stance[li] += int(s.contacts[li])
        if x0 is None:
            x0 = s.base_pos[0]
    assert all(c > 0 for c in stance), f"a foot never planted: {stance}"
    assert pu.sample(2.0, DT).base_pos[0] > x0 + 0.05, "the base never advanced"

def _sigma_rdot(lm, leg, nominals, rng, sigma_q=0.002, dt=1 / 500.0, n=4000):
    """sigma of the finite-differenced sole velocity under encoder noise only.

    Takes the descriptor explicitly rather than closing over one: LegModel is a
    frozen dataclass, so swapping it to compare two robots is not an option, and
    the comparison is the whole point of the test.
    """
    from otolith_sim.leg_model import sole_world
    chain = lm.chain(leg)
    dof = len(chain.joints)
    walk = np.cumsum(rng.normal(0, 2e-4, (n + 1, dof)), axis=0)
    walk -= walk[0]
    meas = np.asarray(nominals) + walk + sigma_q * rng.normal(0, 1, (n + 1, dof))
    pos = np.array([sole_world(chain, meas[k], np.zeros(3),
                               sole_offset=lm.sole[leg]) for k in range(n + 1)])
    return float((np.diff(pos, axis=0) / dt).std(axis=0).max())


def test_sigma_leg_is_monotonic_in_lever_arm(apollo):
    """P4 across three robots: the ORDERING is the evidence, not any one number.

    Measured stance sigma(r_dot), encoder sigma 0.002 rad, dt = 2 ms:

        go2      0.30   3 DoF, ~0.3 m lever
        g1       1.05   6 DoF, ~0.80 m lever
        apollo   1.40   6 DoF, ~0.90 m lever

    sigma_leg is the differentiated-encoder-noise term, sigma_q*sqrt(2)*L/dt, so it
    must grow with the lever arm. It does, monotonically, and both bipeds exceed
    the shipped 0.3 by 3.5x and 4.7x.

    The real-motion term is the SAME for all three -- 0.184 m/s, measured with the
    encoders switched off, which is just the base swaying at up to 0.256 m/s. So the
    entire difference between the robots is lever arm, exactly as predicted.

    P4 predicted 1.0-1.1 m/s for Apollo from its NOMINAL leg length. Measured is
    1.40. The mechanism was right and the constant was wrong: the effective lever
    is not hip-to-sole but the accumulated response of all six links plus the sole
    offset, which for Apollo comes out to 0.98 m against a 0.90 m nominal. Using
    the nominal length under-reads a long chain.
    """
    from otolith_sim.leg_model import load_g1

    model, _, lm = apollo
    rng = np.random.default_rng(20240)

    apollo_leg = _sigma_rdot(lm, "left", [0.08, 0.10, -0.477, 1.033, -0.03, -0.58], rng)
    assert 1.2 < apollo_leg < 1.6, (
        f"Apollo sigma(r_dot) {apollo_leg:.3f} m/s is outside the 1.2-1.6 band; the "
        "lever-arm argument wants ~1.4, and a jump means a descriptor or a sole "
        "offset has moved")

    g1_scene = ROOT / ".work/g1scene/scene.xml"
    if not g1_scene.exists():
        pytest.skip("G1 scene not built")
    g1_lm = load_g1(mujoco.MjModel.from_xml_path(str(g1_scene)))
    g1_nom = [0.669 if "knee" in j else (-0.312 if "hip_pitch" in j else 0.0)
              for j in g1_lm.chain("left").joints]
    g1_leg = _sigma_rdot(g1_lm, "left", g1_nom, rng)

    assert apollo_leg > g1_leg, (
        f"Apollo {apollo_leg:.3f} should exceed G1 {g1_leg:.3f}: Apollo's leg is "
        "longer, and sigma_leg scales with the lever arm")
    # And both must exceed the shipped constant by a wide margin, which is the
    # actual claim P4 makes.
    assert min(g1_leg, apollo_leg) > 3 * 0.3, (
        f"biped legs give {g1_leg:.2f}/{apollo_leg:.2f} m/s, under 3x the shipped "
        "0.3 -- if this ever passes, the constant would be defensible again")
