# Foxglove layouts

Three layouts live here.

## `humanoid_demo.json` — humanoid split-screen demo (load this for the biped demo)

Start the stack first: `pixi run bash ./scripts/humanoid_demo.sh g1` (or `apollo`,
`op3`).

```
┌──────────────────┬──────────────────────────┐
│                  │  SOFTWARE · step time    │  live,  /otolith/perf
│   3D + paths     ├──────────────────────────┤
│                  │  SOFTWARE · duty & rate  │  live,  /otolith/perf
│                  ├──────────────────────────┤
│                  │  SILICON · update cost   │  STATIC, /otolith/perf_hw
│                  ├──────────────────────────┤
│                  │  SILICON · measured PPA  │  STATIC, /otolith/perf_hw
└──────────────────┴──────────────────────────┘
```

The two halves answer different questions and are **not** equally live:

- **Top right is live.** `/otolith/perf` is measured from the running C++ filter:
  predict/update CPU time, deadline, duty cycle, achieved rate, jitter p99.
- **Bottom right is a cost model.** `/otolith/perf_hw` publishes PPA we already
  measured (ADR-0007) plus a derived cycle estimate from the MAC table in
  `hdl/M5_UPDATE_STUDY.md`. Its field 16 is `0` and stays `0` until the Verilator
  co-simulation lands; do not present that panel as a measurement of the running
  filter's silicon cost.

Worth noticing when both are on screen: a **biped update is the cheaper silicon
case**. The 15×15×15 Joseph product is width-independent and is 54% of the cost, so
a 2-foot biped runs 12,500 MAC against a trot's 22,390 — ~45% of the 2 ms budget
versus ~80%. The software panel disagrees (bipeds measured ~31% duty, well under
the trot), which is the interesting bit: the two implementations are not
comparable at equal accuracy.

3D panel setup is the same as the Go2 layout: add a URDF layer with
`Source=Topic /robot_description`, frame `base`, and set Scene `meshUpAxis` to
`z_up` (restart Studio after changing). Note `otolith_description` currently ships
Go2 meshes only, so the biped demo shows paths and frames but not a biped mesh until
a G1 URDF lands — see "Not done" in `docs/V05_HUMANOID.md`.

## `go2_demo.json` — primary Go2 demo layout

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
