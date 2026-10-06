"""Derive the stance joint configurations the FEA needs, from the sim itself.

Generated in the DEFAULT pixi environment (it needs mujoco) and committed, so
the `cad` environment can consume it without the two environments having to
share a dependency. Same pattern as the geometry spec: derived once, pinned,
committed, asserted.

Why this exists: the first thigh FEA swept a hand-chosen "stance angle" from
0 to 45 deg. The sim's actual trot puts the thigh at 44-62 deg, so the synthetic
sweep stopped short of every real case AND invented a load split that geometry
determines. These numbers replace both.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

import mujoco
import numpy as np

sys.path.insert(0, os.path.join(REPO, "sim"))
from otolith_sim.puppet import Go2Puppet  # noqa: E402

OUT = os.path.join(HERE, "gait_stance.json")
SCENE = os.path.join(REPO, "third_party", "menagerie", "unitree_go2", "scene.xml")


def build(n=9):
    if not os.path.exists(SCENE):
        raise SystemExit(f"{SCENE} missing -- see CLAUDE.md for the menagerie symlink")
    model = mujoco.MjModel.from_xml_path(SCENE)
    puppet = Go2Puppet(model)
    dt = model.opt.timestep
    data = mujoco.MjData(model)

    rows = []
    for k in range(int(3.0 / dt)):
        s = puppet.sample(model, data, k * dt, dt)
        if s.contacts[0]:                      # FL stance
            rows.append((float(s.qpos[8]), float(s.qpos[9])))
    if not rows:
        raise SystemExit("no FL stance samples found")

    step = max(len(rows) // n, 1)
    picked = rows[::step][:n]
    return {
        "provenance": {
            "source": "otolith_sim.puppet Go2Puppet, FL stance samples over 3 s",
            "model": "third_party/menagerie/unitree_go2/scene.xml",
            "dt": dt,
            "note": "thigh and calf joint angles, radians, MuJoCo convention "
                    "(calf negative = folded). Feed to "
                    "mesh_thigh.physical_knee_load.",
            "regenerate": "pixi run python mech/spec/build_gait.py",
        },
        "cycle_s": puppet.cfg.cycle_s,
        "duty": puppet.cfg.duty,
        "thigh_deg_range": [float(np.degrees(min(r[0] for r in rows))),
                            float(np.degrees(max(r[0] for r in rows)))],
        "calf_deg_range": [float(np.degrees(min(r[1] for r in rows))),
                           float(np.degrees(max(r[1] for r in rows)))],
        "stance_samples": n,
        "configs": [{"thigh_rad": th, "calf_rad": ca,
                     "thigh_deg": float(np.degrees(th)),
                     "calf_deg": float(np.degrees(ca))} for th, ca in picked],
    }


if __name__ == "__main__":
    spec = build()
    with open(OUT, "w") as fh:
        json.dump(spec, fh, indent=2)
        fh.write("\n")
    print(f"wrote {OUT}")
    print(f"  thigh {spec['thigh_deg_range'][0]:.1f}..{spec['thigh_deg_range'][1]:.1f} deg, "
          f"calf {spec['calf_deg_range'][0]:.1f}..{spec['calf_deg_range'][1]:.1f} deg")
    for c in spec["configs"]:
        print(f"   thigh {c['thigh_deg']:6.2f}  calf {c['calf_deg']:7.2f}")
