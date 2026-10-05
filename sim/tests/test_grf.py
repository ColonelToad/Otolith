"""v0.6 ground-reaction load cases: conservation and symmetry gates.

The whole point of computing GRF from CoM momentum balance instead of MuJoCo's
contact solver is that it has a conservation law to be checked against. These
tests ARE that check -- if they pass, the forces are trustworthy; if the
smoothening window or the mass assembly regresses, they fail loudly rather
than feeding FEA a plausible-looking number.

See sim/otolith_sim/grf.py for the measured reasons the MuJoCo route was
rejected, and docs/V06_LOAD_CASES.md for the numbers.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, "sim")

MENAGERIE = Path("third_party/menagerie/unitree_go2/scene.xml")
DURATION_S = 3.5          # 5 gait cycles at cycle_s=0.7


@pytest.fixture(scope="module")
def series():
    mujoco = pytest.importorskip("mujoco")
    if not MENAGERIE.exists():
        pytest.skip(f"{MENAGERIE} missing (see CLAUDE.md for the menagerie symlink)")
    from otolith_sim.grf import record_grf
    model = mujoco.MjModel.from_xml_path(str(MENAGERIE))
    return record_grf(model, DURATION_S)


def test_mean_grf_equals_weight(series):
    """A periodic gait at constant height must integrate to exactly the weight.

    This is the check that rejected MuJoCo's contact solver, which gave
    0.88 W. Anything that breaks the mass assembly, the CoM, or the edge
    handling shows up here as a number off 1.0.
    """
    from otolith_sim.grf import load_summary
    ratio = load_summary(series)["mean_fz_over_weight"]
    assert ratio == pytest.approx(1.0, abs=5e-3), (
        f"mean GRF is {ratio:.6f} W; a periodic gait must integrate to 1.0 W. "
        "Check the mass assembly (legs included) and the smoothing edges.")


def test_grf_never_negative(series):
    """No foot can pull on the floor. Negative Fz means a sign or edge error."""
    w = slice(series.smooth_steps, -series.smooth_steps or None)
    assert series.fz_total[w].min() > 0.0, "ground reaction went negative"


def test_swing_feet_carry_nothing(series):
    assert np.all(series.fz_feet[~series.stance] == 0.0), \
        "a swing foot was assigned load"


def test_diagonal_pairs_are_symmetric(series):
    """The per-foot split assumes FL+RR and FR+RL share equally.

    If this fails, the symmetry assumption the split rests on no longer holds
    for this gait and the split needs to be solved from stance geometry
    instead. The pairs are (FL, RR) = (0, 3) and (FR, RL) = (1, 2).
    """
    from otolith_sim.grf import load_summary
    peaks = load_summary(series)["per_foot_peak_N"]
    assert peaks[0] == pytest.approx(peaks[3], rel=2e-3), \
        f"FL/RR peaks differ: {peaks[0]:.3f} vs {peaks[3]:.3f}"
    assert peaks[1] == pytest.approx(peaks[2], rel=2e-3), \
        f"FR/RL peaks differ: {peaks[1]:.3f} vs {peaks[2]:.3f}"


def test_peak_amplitude_matches_the_bob(series):
    """Independent amplitude check: peak GRF from the bob's own kinematics.

    The CoM bobs +/- ~7.8 mm at 2.86 Hz, so a_com_peak = A(2*pi*f)^2 and
    peak GRF/W = 1 + a_com_peak/g. An unsmoothed second difference instead
    reports ~2.11 W because it is differentiating the footfall kinks; this
    pins the smoothed answer to the analytic one.
    """
    from otolith_sim.grf import load_summary
    su = load_summary(series)
    amplitude_m = 0.0078
    freq_hz = 2.0 / 0.7                      # bob is 2 cycles per gait cycle
    a_peak = amplitude_m * (2 * np.pi * freq_hz) ** 2
    expected = 1.0 + a_peak / 9.81
    assert su["peak_fz_over_weight"] == pytest.approx(expected, rel=0.05), (
        f"peak {su['peak_fz_over_weight']:.3f} W vs {expected:.3f} W predicted "
        "from the bob amplitude -- smoothing window may be wrong")


def test_smooth_window_is_not_load_bearing(series):
    """The answer must not depend on the arbitrary smoothing window.

    Sweeping 9..81 steps moves the mean by <1e-3 W (docs/V06_LOAD_CASES.md).
    If a change makes this sensitive, the window is hiding something.
    """
    mujoco = pytest.importorskip("mujoco")
    from otolith_sim.grf import record_grf, load_summary
    model = mujoco.MjModel.from_xml_path(str(MENAGERIE))
    means = [load_summary(record_grf(model, DURATION_S, smooth_steps=w))["mean_fz_over_weight"]
             for w in (9, 25, 81)]
    assert max(means) - min(means) < 5e-3, \
        f"mean GRF varies with the smoothing window: {means}"


def test_stance_impulses_are_positive_and_finite(series):
    """Impulse per stance is the fatigue input; it must be a real number."""
    from otolith_sim.grf import load_summary
    for foot, runs in enumerate(load_summary(series)["per_foot_stance_impulse_Ns"]):
        assert runs, f"foot {foot} never entered stance"
        for v in runs:
            assert np.isfinite(v) and v > 0.0