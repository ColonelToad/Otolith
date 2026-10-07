"""Enable foot contact on the Apollo scene.

Apptronik's Apollo model ships with **zero collidable geoms**: all 79 of them have
`contype="0" conaffinity="0"`, including the two sole boxes that are obviously
meant to be the feet. So there is nothing to select by -- contact has to be turned
on before a descriptor can refer to it.

This is the same situation as G1's `scene_mjx.xml`, where all eight foot geoms are
also disabled, and for the same reason: those scenes are meant to be driven with
an explicit contact pair list. It is not a defect in either model.

The vendor `stand` keyframe is a real pose -- base z 1.01597 with non-zero joint
angles, unlike G1's all-zeros zero pose -- but its sole boxes sit 1.19 mm *below*
the floor, so it is a millimetre of penetration rather than a stance to stand in.
The puppet solves its own ground-contact home pose, the way G1's does; nothing
here tries to make the keyframe exact.

Foot sole geoms: `collision_l_sole` / `collision_r_sole`, one box per foot
(half-size 0.1 x 0.0425 x 0.009, so 200 x 85 x 18 mm), on `l_foot_link` /
`r_foot_link`. Note the geom names do NOT match the mesh asset names -- the vendor
file's `l_foot_fl/fr/bl/br` are mesh files, not geoms, and asking MuJoCo for a
geom by those names returns -1. Silently indexing geom -1 then reads the *last*
geom in the model, which happens to sit on `r_foot_link` and made both feet look
coincident. Worth knowing before anyone else wastes an hour on it.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

MENAGERIE = Path(__file__).resolve().parents[2] / "third_party" / "menagerie" / "apptronik_apollo"

SOLE_GEOMS = ("collision_l_sole", "collision_r_sole")

# Contact pairs for the enabled soles: foot against the floor. Apollo's floor is
# plane 0 in the scene, so contype=conaffinity=1 with the floor's default mask is
# sufficient and needs no explicit <pair>.
_CONTACT = 'contype="1" conaffinity="1"'


def enable_sole_contact(xml: str) -> tuple[str, int]:
    """Turn on contact for the two sole geoms. Returns (xml, count)."""
    n = 0
    for name in SOLE_GEOMS:
        pat = re.compile(r'(<geom\b[^>]*\bname="%s"[^>]*?)(/?>)' % re.escape(name))

        def _sub(m: "re.Match[str]") -> str:
            nonlocal n
            tag = m.group(1)
            tag = re.sub(r'\s*contype="[^"]*"', "", tag)
            tag = re.sub(r'\s*conaffinity="[^"]*"', "", tag)
            n += 1
            return f"{tag} {_CONTACT}{m.group(2)}"

        xml, k = pat.subn(_sub, xml)
        if k != 1:
            raise ValueError(f"expected exactly one geom named {name}, patched {k}")
    return xml, n


def build(dest=".work/apolloscene", menagerie=MENAGERIE) -> Path:
    """Write the patched scene plus an asset symlink farm. Returns the scene path."""
    dest = Path(dest)
    (dest / "assets").mkdir(parents=True, exist_ok=True)
    for f in (menagerie / "assets").glob("*"):
        link = dest / "assets" / f.name
        if not (link.is_symlink() or link.exists()):
            link.symlink_to(f.resolve())

    xml = (menagerie / "apptronik_apollo.xml").read_text()
    xml, n = enable_sole_contact(xml)
    if n != 2:
        raise ValueError(f"expected 2 sole geoms enabled, got {n}")
    xml = re.sub(r'meshdir="[^"]*"',
                 f'meshdir="{(dest / "assets").resolve()}"', xml, count=1)
    scene = dest / "apollo_scene.xml"
    scene.write_text(xml)
    # scene.xml is the conventional entry point every other model here uses.
    (dest / "scene.xml").write_text(xml)
    return scene


if __name__ == "__main__":
    p = build(sys.argv[1] if len(sys.argv) > 1 else ".work/apolloscene")
    print(f"wrote {p}")