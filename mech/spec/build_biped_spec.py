"""Generate a biped block of `RobotSpec` for fusion/src/leg_kin.cpp.

G1's C++ spec is hand-transcribed literals, and the comment above `make_g1` claims
it came from a generator (`build_g1_spec.py`) that does not exist. That comment is
wrong about its own provenance, and hand transcription is exactly what it warns
against -- a mis-typed axis produces plausible numbers rather than an error.

Apollo is the case where that matters most, because **its mirroring rule is not
G1's**: G1's legs mirror only `body_pos.y` and share `body_quat`, while Apollo's
reflect every quaternion about y as (w,x,y,z) -> (w,-x,y,-z). Deriving one leg from
the other with the wrong rule silently misplaces every link past the hip. So this
reads BOTH legs out of the model and emits each one, rather than emitting the left
and negating y.

Handles both chain bipeds: `apollo` and `op3`. G1's literals are still
hand-transcribed and its generator still does not exist; that is noted below
rather than fixed here, because rewriting make_g1 is not a change to make in the
same breath as adding a third robot.

Run:  pixi run python mech/spec/build_biped_spec.py [apollo|op3]
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[2]
MENAGERIE = ROOT / "third_party" / "menagerie"

import sys as _sys
_sys.path.insert(0, str(ROOT / "sim"))
from otolith_sim import leg_model as _lm
_LADDER = {"apollo": _lm.load_apollo, "op3": _lm.load_op3}

ROBOTS = {
    "apollo": dict(
        scene="apptronik_apollo/scene.xml",
        joints=_lm.APOLLO_JOINTS,
        body=lambda j: _lm.APOLLO_JOINT_BODY[j],
        joint=lambda p, j: f"{p}_{j}",
        sides={"left": "l", "right": "r"},
    ),
    "op3": dict(
        scene="robotis_op3/scene.xml",
        joints=_lm.OP3_JOINTS,
        body=lambda j: _lm.OP3_JOINT_BODY[j],
        joint=lambda p, j: f"{p}_{j}",
        sides={"left": "l", "right": "r"},
    ),
}
# OP3's scene is the PATCHED one: it has no named geoms in the vendor file, but the
# chain transform data does not depend on the patch. Reading the vendor path would
# work too; the patched path is used so the generator and the descriptor read the
# same file the filter does.
SCENE_OVERRIDE = {"op3": ROOT / ".work/op3scene/scene.xml"}


def fmt(v: float) -> str:
    """Compact but exact-enough literal. 17 significant digits is overkill and
    noisy; %.9g round-trips a double to well under a nanometre."""
    if v == 0.0:
        return "0"
    return f"{v:.9g}"


def emit(model: mujoco.MjModel, spec: dict) -> str:
    out: list[str] = []
    for leg in ("left", "right"):
        p = spec["sides"][leg]
        rows = []
        for j in spec["joints"]:
            jid = model.joint(spec["joint"](p, j)).id
            bid = model.body(f"{p}_{spec['body'](j)}").id
            ax = model.jnt_axis[jid]
            o = model.body_pos[bid]
            q = model.body_quat[bid]
            rows.append((spec["joint"](p, j), ax, o, q))
        out.append(f"        // {leg}")
        for name, ax, o, q in rows:
            out.append(
                f"            {{ {fmt(ax[0])}, {fmt(ax[1])}, {fmt(ax[2])},"
                f"  {fmt(o[0])}, {fmt(o[1])}, {fmt(o[2])},"
                f"  {fmt(q[0])}, {fmt(q[1])}, {fmt(q[2])}, {fmt(q[3])},"
                f"  \"{name}\" }},")
    return "\n".join(out)


def emit_sole(model: mujoco.MjModel, spec: dict) -> str:
    """The C++ `L.sole` line for both legs.

    Emitted from the model rather than typed, because the sole offset is applied
    at the very END of the chain: its rounding error lands directly on the output
    position with nothing to average it against. Typed to 5 decimals it cost
    5.6 um of FK error on its own -- small enough to pass a 2 mm gate and large
    enough to be the entire error budget of an nm-scale test.
    """
    lm = _LADDER[spec["name"]](model)
    out = []
    for leg in ("left", "right"):
        v = lm.sole[leg]
        out.append(f"        // {leg}: {fmt(v[0])}, {fmt(v[1])}, {fmt(v[2])}")
    return "\n".join(out)


def scene_for(name: str) -> Path:
    if name in SCENE_OVERRIDE and SCENE_OVERRIDE[name].exists():
        return SCENE_OVERRIDE[name]
    return MENAGERIE / ROBOTS[name]["scene"]


def main() -> int:
    which = sys.argv[1] if len(sys.argv) > 1 else "apollo"
    if which not in ROBOTS:
        print(f"unknown robot {which}; known: {sorted(ROBOTS)}", file=sys.stderr)
        return 2
    path = scene_for(which)
    if not path.exists():
        print(f"missing {path}", file=sys.stderr)
        return 2
    spec = dict(ROBOTS[which], name=which)
    model = mujoco.MjModel.from_xml_path(str(path))
    print(emit(model, spec))
    print("// soles")
    print(emit_sole(model, spec))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())