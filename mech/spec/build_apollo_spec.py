"""Generate the Apollo block of `RobotSpec` for fusion/src/leg_kin.cpp.

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

Run:  pixi run python mech/spec/build_apollo_spec.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[2]
MENAGERIE = ROOT / "third_party" / "menagerie" / "apptronik_apollo"
MODEL = MENAGERIE / "scene.xml"

JOINTS = ("hip_ie", "hip_aa", "hip_fe", "knee_fe", "ankle_ie", "ankle_pd")
SIDE = {"left": "l", "right": "r"}


def fmt(v: float) -> str:
    """Compact but exact-enough literal. 17 significant digits is overkill and
    noisy; %.9g round-trips a double to well under a nanometre."""
    if v == 0.0:
        return "0"
    return f"{v:.9g}"


def emit(model: mujoco.MjModel) -> str:
    out: list[str] = []
    for leg in ("left", "right"):
        p = SIDE[leg]
        rows = []
        for j in JOINTS:
            jid = model.joint(f"{p}_{j}").id
            bid = model.body(f"{p}_{j if j != 'ankle_pd' else 'foot'}_link").id
            ax = model.jnt_axis[jid]
            o = model.body_pos[bid]
            q = model.body_quat[bid]
            rows.append((f"{p}_{j}", ax, o, q))
        out.append(f"        // {leg}")
        for name, ax, o, q in rows:
            out.append(
                f"            {{ {fmt(ax[0])}, {fmt(ax[1])}, {fmt(ax[2])},"
                f"  {fmt(o[0])}, {fmt(o[1])}, {fmt(o[2])},"
                f"  {fmt(q[0])}, {fmt(q[1])}, {fmt(q[2])}, {fmt(q[3])},"
                f"  \"{name}\" }},")
    return "\n".join(out)


def emit_sole(model: mujoco.MjModel) -> str:
    """The C++ `L.sole` line for both legs.

    Emitted from the model rather than typed, because the sole offset is applied
    at the very END of the chain: its rounding error lands directly on the output
    position with nothing to average it against. Typed to 5 decimals it cost
    5.6 um of FK error on its own -- small enough to pass a 2 mm gate and large
    enough to be the entire error budget of an nm-scale test.
    """
    sys.path.insert(0, str(ROOT / "sim"))
    from otolith_sim.leg_model import load_apollo
    lm = load_apollo(model)
    out = []
    for leg in ("left", "right"):
        v = lm.sole[leg]
        out.append(f"        // {leg}: {fmt(v[0])}, {fmt(v[1])}, {fmt(v[2])}")
    return "\n".join(out)


def main() -> int:
    if not MODEL.exists():
        print(f"missing {MODEL}", file=sys.stderr)
        return 2
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    print(emit(model))
    print("// soles")
    print(emit_sole(model))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())