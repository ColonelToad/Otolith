"""Produce a G1 scene with its foot contact geoms NAMED.

WHY THIS HAS TO EXIST
=====================
G1 ships with **no named geoms at all** -- every geom is anonymous. Go2 names its
four foot spheres FL/FR/RL/RR, and the filter resolves contacts by name. So
without a patch, the G1 path would have to select contacts by index, and indices
are exactly the thing that silently moves when a model is regenerated.

Selecting by body plus a contact-enabling test works, but then the *name* is
still needed as the stable handle the descriptor and the log format carry. So
the scene gets patched once, deterministically, and the name becomes the
contract.

The alternative -- selecting by body and carrying no name -- was considered and
rejected: it makes the filter's contact handling robot-specific, which is the
thing this phase is trying to remove.

WHAT IT PATCHES
===============
The four contact spheres per ankle (group 3, contype != 0, contype sphere) get
`name="left_sole0..3"` / `right_sole0..3`. Nothing else is touched: visual
meshes, groups, inertias and masses are left exactly as the vendor shipped
them, because a hand-edited contact model is how the Go2 foot-sphere offset
became load-bearing in the first place (docs/V06_FOOT_PAD.md).

Writes to `.work/` (gitignored). The patched scene is derived, not authored.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

MENAGERIE = Path("third_party/menagerie/unitree_g1")
LEGS = ("left", "right")


def _is_contact_sphere(line: str) -> bool:
    """True for a foot contact geom, identified by `class="foot"`.

    Selected by CLASS, not by primitive. The G1 scene does not spell the
    primitives out -- the four contact spheres per ankle are
    `<geom class="foot" pos="..."/>`, with the sphere, radius, friction and
    condim inherited from a nested default. An earlier version looked for
    `<sphere` and found none, which is the general hazard of doing model
    surgery on XML text: a selector that matches nothing is indistinguishable
    from a model that changed.

    The class is the stable handle here precisely because the defaults are
    inherited -- an explicit `<sphere ...>` is what you would have to keep
    transcribing.
    """
    return "<geom" in line and 'class="foot"' in line


def name_foot_geoms(xml: str) -> tuple[str, int]:
    """Return the XML with the four contact spheres per ankle named."""
    total = 0
    for leg in LEGS:
        start = xml.find(f'<body name="{leg}_ankle_roll_link"')
        if start < 0:
            raise ValueError(f"no <body name=\"{leg}_ankle_roll_link\"> in the scene")
        end = xml.find("</body>", start)
        if end < 0:
            raise ValueError(f"unterminated <body> for {leg}_ankle_roll_link")
        block = xml[start:end]

        lines = block.split("\n")
        idx = [i for i, ln in enumerate(lines) if _is_contact_sphere(ln)]
        if len(idx) != 4:
            raise ValueError(
                f"{leg}_ankle_roll_link: expected 4 contact spheres, found "
                f"{len(idx)}. Refusing to guess -- a different count means the "
                "sole reference point would be wrong, and silently so.")

        for n, i in enumerate(idx):
            ln = lines[i]
            if 'name="' in ln:
                raise ValueError(f"geom already named: {ln.strip()}")
            patched = ln.replace("<geom", f'<geom name="{leg}_sole{n}"', 1)
            # Assert the substitution HAPPENED. An earlier version counted the
            # matches and then substituted a pattern that was not in them, so it
            # reported 8 geoms named while writing the file untouched -- and the
            # scene compiled cleanly, so the only symptom was an unnamed geom
            # three steps later. Counting is not evidence of editing.
            if patched == ln:
                raise ValueError(
                    f"failed to inject a name into {ln.strip()!r}: no <geom tag "
                    "found. The selector matched this line but the edit did not.")
            lines[i] = patched
            total += 1
        xml = xml[:start] + "\n".join(lines) + xml[end:]
    return xml, total


def build(dest=".work/g1scene", menagerie=MENAGERIE) -> Path:
    """Write the patched scene plus an asset symlink farm. Returns the scene path."""
    dest = Path(dest)
    (dest / "assets").mkdir(parents=True, exist_ok=True)
    for f in (menagerie / "assets").glob("*"):
        link = dest / "assets" / f.name
        if not (link.is_symlink() or link.exists()):
            link.symlink_to(f.resolve())

    xml = (menagerie / "g1.xml").read_text()
    xml, n = name_foot_geoms(xml)
    if n != 8:
        raise ValueError(f"expected 8 contact spheres across both feet, named {n}")
    # the meshdir must resolve from the new location
    xml = re.sub(r'meshdir="[^"]*"', f'meshdir="{(dest / "assets").resolve()}"', xml, count=1)
    scene = dest / "g1_scene.xml"
    scene.write_text(xml)
    (dest / "scene.xml").write_text(
        xml.replace('<mujoco model="g1">', '<mujoco model="g1_scene">', 1))
    return scene


if __name__ == "__main__":
    p = build(sys.argv[1] if len(sys.argv) > 1 else ".work/g1scene")
    print(f"wrote {p}")