#!/usr/bin/env python3
"""One-shot: OBJ (Menagerie, Chinese mtllib, Foxglove-unfriendly) -> binary STL.

Foxglove does not read .mtl and lists OBJ as its weakest format ("no material
support, no compression, additional processing overhead") while STL is "well
supported". Geometry is preserved exactly (triangle soup in, triangle soup out).

Usage: pixi run python scripts/convert_meshes.py
Writes ros2/otolith_description/meshes/*.stl next to the .obj sources.
"""
from pathlib import Path

import trimesh

SRC = Path(__file__).parent.parent / "ros2/otolith_description/meshes"

def main():
    objs = sorted(SRC.glob("*.obj"))
    assert objs, f"no .obj in {SRC}"
    for src in objs:
        # process=False keeps raw triangles; ignore missing .mtl (materials unused)
        mesh = trimesh.load(str(src), force="mesh", process=False)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
        dst = src.with_suffix(".stl")
        mesh.export(str(dst))  # binary STL by default
        n_faces = len(mesh.faces)
        print(f"{src.name} -> {dst.name}  faces={n_faces}  "
              f"{src.stat().st_size/1e6:.1f}MB -> {dst.stat().st_size/1e6:.1f}MB")

if __name__ == "__main__":
    main()
