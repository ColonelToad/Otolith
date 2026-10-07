"""The eval report must not mislabel its own subject.

`evaluate.py` used to hard-code "kinematic trot" and `σ_leg=0.3 m/s` into the
report for every input. That was already wrong for one robot and is now wrong for
three, so it is gated: the report identifies its robot from the log itself and says
"unknown" rather than guessing.

Two discriminators, and the order matters. Base height alone cannot work: Go2's
base sits at 0.270 m and OP3's at 0.260 m -- 1 cm apart -- so height alone labels a
quadruped as a biped, which is exactly what the first two attempts did. Contact-slot
count IS decisive (a trot puts 4 feet down, every biped puts 2), and only then is
height used, to separate the three bipeds at 0.260 / 0.745 / 1.006 m.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def ev():
    spec = importlib.util.spec_from_file_location(
        "evaluate", ROOT / "eval" / "evaluate.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_every_robot_has_facts(ev):
    """No robot may be missing from the facts table.

    A robot present in `robot_spec` but absent here would silently print "unknown
    gait", which is at least honest -- but it is a hole, so the set is pinned.
    """
    assert set(ev.ROBOT_FACTS) == {"go2", "g1", "apollo", "op3"}
    assert set(ev.ROBOT_BASE_Z) == {"g1", "apollo", "op3"}
    # sigma_leg must be non-zero and plausible for every robot.
    for name, (_gait, sig) in ev.ROBOT_FACTS.items():
        assert 0.05 < sig < 3.0, f"{name}: sigma_leg {sig} looks wrong"


def test_contact_count_is_the_discriminator(ev):
    """Go2 and OP3 have near-identical base heights and must not be confused."""
    assert ev.ROBOT_LEGS["go2"] == 4
    assert all(ev.ROBOT_LEGS[r] == 2 for r in ("g1", "apollo", "op3"))
    # The 1 cm collision that makes height-alone unsafe.
    assert abs(ev.ROBOT_BASE_Z["op3"] - 0.270) < 0.02, (
        "Go2's base height is not recorded here; if it moves closer to or further "
        "from OP3's, revisit whether contact count alone is sufficient")


def test_unknown_log_is_reported_as_unknown(ev, tmp_path):
    """A log it cannot identify must produce 'unknown', not a confident guess."""
    bad = tmp_path / "bad.otlg"
    bad.write_bytes(b"not a log")
    assert ev._robot_facts(str(bad)) is None


def test_report_has_no_hardcoded_go2_prose(ev):
    """The report body must interpolate the robot, not literalise Go2's facts.

    Checked as source text because the failure mode is invisible in the numbers --
    a G1 run that says "kinematic trot" and "sigma_leg=0.3 m/s" produces entirely
    plausible RMSE values.
    """
    src = (ROOT / "eval" / "evaluate.py").read_text()
    # The title and notes are built from `robot`, `gait` and `sig`.
    assert "`σ_leg=0.3 m/s` per foot" not in src, (
        "the report still hard-codes sigma_leg=0.3")
    assert "RMSE (trot," not in src, "the report title still hard-codes the gait"
    assert "# Otolith eval — RMSE (trot" not in src
    # And it must interpolate rather than embed a literal subject.
    assert "subject = robot.upper()" in src
    assert "{hz} Hz" in src, "the rate must come from the log's dt, not a constant"
