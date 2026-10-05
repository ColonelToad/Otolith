"""Mesh-convergence study: does refining fix CalculiX's low stress reading?

What decides how the real phase meshes bending problems. The first guess -- that
C3D8R's 1-point integration has an irreducible low bias, so v0.6 will need
stress extrapolation or quadratic elements -- was wrong. Measurement:

   nx  ny  nz  elems  defl err%   stress ratio min/mean/max
   20   2   2      80       2.90   0.521 / 0.543 / 0.589
   40   4   4     640       0.90   0.767 / 0.786 / 0.821
   40   4   8    1280       2.67   0.878 / 0.900 / 0.941
   40   4  16    2560       3.21   0.936 / 0.960 / 1.004
   80   4  16    5120       0.74   0.948 / 0.959 / 0.981
   80   8  16   10240       0.78   0.946 / 0.957 / 0.979
  160   8  16   20480       0.02   0.946 / 0.952 / 0.962

Conclusions, all of which cost a solve to learn:

  - **nz (through the thickness) is the only refinement that matters.** 2 -> 16
    elements across the section takes the stress ratio 0.54 -> 0.96.
  - **nx (along the span) is already converged.** 40 -> 160 moves the ratio
    0.960 -> 0.952, i.e. nowhere. Do not spend elements lengthwise on bending.
  - **The floor is ~0.95, not 0.77.** The 20-25% deficit was an under-meshed
    thickness, not reduced integration's character. What remains is a genuine
    ~5% low offset, and it is constant across the span, so it is correctable.
  - **Tip deflection is NOT a convergence indicator here.** It wanders
    0.02-3.21% non-monotonically (C3D8R hourglassing), while stress converges
    cleanly. Gate on stress, not deflection, and treat a smooth deflection as a
    lucky run rather than a converged one.

Rule for v0.6: ~16 elements through the thickness of any thin member, and ignore
the span.
"""
import os
import subprocess
import sys

# Run inside an env that has calculix + cadquery; not wired into the project
# pixi env because CalculiX is not a project dependency (see the feasibility
# note). Either add both to pixi.toml, or point CCX at any ccx on PATH.
CCX = os.environ.get("CCX", "ccx")
PY = sys.executable
L, w, h, E, F = 0.4, 0.02, 0.02, 200e9, 50.0
I = w * h**3 / 12.0

import os
TEMPLATE = open(os.path.join(os.path.dirname(__file__) or ".", "cantilever.py")).read()


def run(nx, ny, nz):
    src = TEMPLATE.replace("nx, ny, nz = 40, 4, 4", f"nx, ny, nz = {nx}, {ny}, {nz}")
    open("sweep_inp.py", "w").write(src)
    subprocess.run([PY, "sweep_inp.py"], capture_output=True)
    if os.path.exists("beam.frd"):
        os.remove("beam.frd")
    subprocess.run([CCX, "-i", "beam"], capture_output=True)
    if not os.path.exists("beam.frd"):
        return None
    out = subprocess.run([PY, "frd.py", "beam.frd", str(nx)],
                         capture_output=True, text=True).stdout
    if "tip deflection" not in out:
        return None
    derr = float(out.split("err ")[1].split("%")[0])
    ratios = []
    for line in out.splitlines():
        p = line.split()
        # station rows are "<el> <x> <fea> <theory> <ratio>"; drop the tip row,
        # where beam theory is ~0 and the ratio is meaningless.
        if len(p) == 5 and p[0].isdigit() and int(p[0]) != nx - 1:
            ratios.append(float(p[4]))
    return derr, ratios


print(f"{'nx':>4}{'ny':>4}{'nz':>4}{'elems':>8}{'defl err%':>11}   "
      f"stress ratio min/mean/max")
for nx, ny, nz in [(20, 2, 2), (40, 4, 4), (40, 4, 8), (40, 4, 16),
                   (80, 4, 16), (80, 8, 16), (160, 8, 16)]:
    r = run(nx, ny, nz)
    if r is None:
        print(f"{nx:4d}{ny:4d}{nz:4d}{nx*ny*nz:8d}   solve failed")
        continue
    derr, ratios = r
    print(f"{nx:4d}{ny:4d}{nz:4d}{nx*ny*nz:8d}{derr:11.2f}   "
          f"{min(ratios):.3f} / {sum(ratios)/len(ratios):.3f} / {max(ratios):.3f}")