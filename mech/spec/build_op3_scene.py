"""Name OP3's foot contact geoms so a descriptor can refer to them.

OP3 ships with **zero named geoms** -- all 54 are anonymous -- but unlike Apollo
it does mark its feet: four of them carry `class="foot"`, which is the same marker
the G1 scene uses and for the same reason. So this patcher is a near-twin of
`build_g1_scene.py`, and the detection is by class rather than by geometry: an
earlier G1 draft selected foot geoms by `g - 14`, which breaks the moment the
model is regenerated and picks the wrong geoms silently rather than loudly.

Four foot geoms, two boxes per foot on `l/r_ank_roll_link`:
    size 0.0635 x 0.028 x 0.004  (127 x 56 x 8 mm)
    size 0.057  x 0.039 x 0.004  (114 x 78 x 8 mm)

They are nearly coincident -- 0.5 mm apart laterally -- so the sole point is the
mean of their bottom-face centres, the same construction as G1's mean over contact
spheres. See sim/otolith_sim/leg_model.py.

Contact already works: the whole body is collidable (capsules) and the foot boxes
carry contype=1, so this patch NAMES geoms rather than enabling anything. That is
the opposite of Apollo, where contact had to be found already working in the vendor
scene's <pair> list, and the opposite again of G1's scene_mjx.xml.

Parsing is line-based with an explicit body stack. A first attempt split the text
on `<geom` and looked for `class="foot"` in the following chunk, which also matched
the geom INSIDE the `<default class="foot">` block and named five geoms instead of
four -- a plausible-looking patch of the wrong size.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

MENAGERIE = Path(__file__).resolve().parents[2] / "third_party" / "menagerie" / "robotis_op3"

FOOT_BODIES = {"l_ank_roll_link": "left", "r_ank_roll_link": "right"}
EXPECTED = 4

_BODY_OPEN = re.compile(r'<body\b[^>]*\bname="([^"]+)"')
_BODY_CLOSE = re.compile(r'</body>')
_GEOM = re.compile(r'^\s*<geom\b(.*?)/?>')


def name_foot_geoms(xml: str) -> tuple[str, int]:
    """Add `name="left_foot0"` etc. to foot geoms. Returns (xml, count).

    Only geoms directly inside an ankle-roll body are renamed, and only ones not
    already named. Tracking the enclosing body is what keeps the per-side index
    correct; a running counter would label the right foot 2 and 3 only by luck of
    document order.
    """
    lines = xml.splitlines(keepends=True)
    stack: list[str] = []
    counts = {"left": 0, "right": 0}
    n = 0
    for i, line in enumerate(lines):
        m = _GEOM.match(line)
        inside = stack[-1] if stack else None
        if m and inside in FOOT_BODIES and 'class="foot"' in line and "name=" not in line:
            side = FOOT_BODIES[inside]
            idx = counts[side]
            counts[side] += 1
            line = line.replace("<geom", f'<geom name="{side}_foot{idx}"', 1)
            lines[i] = line
            n += 1
        for b in _BODY_OPEN.findall(line):
            stack.append(b)
        if _BODY_CLOSE.search(line) and stack:
            stack.pop()
    return "".join(lines), n


def build(dest=".work/op3scene", menagerie=MENAGERIE) -> Path:
    """Write the patched scene plus an asset symlink farm. Returns the scene path."""
    dest = Path(dest)
    (dest / "assets").mkdir(parents=True, exist_ok=True)
    for f in (menagerie / "assets").glob("*"):
        link = dest / "assets" / f.name
        if not (link.is_symlink() or link.exists()):
            link.symlink_to(f.resolve())

    xml = (menagerie / "op3.xml").read_text()
    xml, n = name_foot_geoms(xml)
    if n != EXPECTED:
        raise ValueError(f"expected {EXPECTED} foot geoms, named {n}")
    xml = re.sub(r'meshdir="[^"]*"',
                 f'meshdir="{(dest / "assets").resolve()}"', xml, count=1)
    scene = dest / "op3_scene.xml"
    scene.write_text(xml)
    # scene.xml is the conventional entry point every other model here uses.
    (dest / "scene.xml").write_text(xml)
    return scene


if __name__ == "__main__":
    p = build(sys.argv[1] if len(sys.argv) > 1 else ".work/op3scene")
    print(f"wrote {p}")