# Foxglove layouts

Two layouts live here. Both wire the same topics/frames; they differ in **3D panel presentation** and **scope**.

## `go2_demo.json` — primary demo layout (load this)

Hand-authored full multi-panel layout (3D + velocity plot + contacts plot). This is what `README` / `go2_demo.sh` tell you to load.

- Follow: **`follow-pose`** on frame `base` (robot stays centered; path trails behind)
- Camera: close-in (distance ≈ 1.8, phi 35) framing the mesh
- TF display: **on** (layer visible; yellow connectors are `world→base` / `world→base_gt`, not Path topics)
- Grid: grey **`#333333`**
- Path topics: `gradient` green→violet / orange→violet, `lineWidth` **0.1** (merged from the export)
- Panels: 3D + Plot (vel X GT vs est) + Plot (contacts)
- `meshUpAxis: z_up` (restart Studio after changing), URDF Source=Topic `/robot_description`
- Markers: leave hidden for video (`publish_covariance:=true` only for analysis); never set a topic color override on `/otolith/markers`

## `go2_demo_export.json` — Studio 3D-panel export (styling reference)

Exported from a working Studio session (3D panel config only — **no Plot panels**, not a full `configById` layout). Kept for the path-styling values that were merged into `go2_demo.json`, and for the alternate 3D presentation below.

Differences vs `go2_demo.json` (intentional, not bugs):

| Setting | `go2_demo.json` (primary) | `go2_demo_export.json` |
|--------|---------------------------|-------------------------|
| Follow mode | `follow-pose` (`base`) | **`follow-heading`** |
| Camera | distance ≈ 1.8, phi 35, close on mesh | **distance ≈ 6.15, phi 60, targetOffset** (wider shot) |
| TF / transforms display | layer **visible** | **`scene.transforms.visible: false`** (TF connectors hidden; Path layers unaffected) |
| Grid color | `#333333` | **`#248eff`** (blue) |
| Scope | full layout (3D + 2 plots) | 3D panel only |
| Path style | gradient + `lineWidth: 0.1` | same (source of the merge) |

`meshUpAxis: z_up` and URDF `/robot_description` agree in both.

**Do not** wholesale-replace the primary layout with the export — you would lose the Plot panels and switch follow/camera/TF/grid. If you re-export from Studio later, export the **full layout** (File → Export layout) and diff against `go2_demo.json` before replacing.
