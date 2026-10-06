"""Make the sim package importable from the CAD environment.

`mech/tests` runs under two environments: the default one (where `sim` is on
PYTHONPATH via the pixi task) and the opt-in `cad` one, which is
`no-default-feature` and therefore has no `otolith_sim` at all.

The phase-D gates need both -- cadquery to build the foot pad, and the puppet to
measure what it does to the stance -- so the path is set here rather than left to
whichever invocation happened to work.
"""
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parents[2] / "sim"
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))
