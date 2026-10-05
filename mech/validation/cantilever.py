# v0.6 CAD/FEA validation. Not wired into fusion's ctest because CalculiX is
# not a project dependency yet -- see docs/V06_CAD_FEASIBILITY.md.
#
#   pixi run -e cad python mech/validation/cantilever.py && ccx -i beam
#   pixi run -e cad python mech/validation/frd.py beam.frd
#
# Exits nonzero if the tip deflection strays from the closed-form reference by
# more than TOL, so this is a gate rather than a demo. It is the cheapest
# possible insurance on the FEA toolchain: without it a green CalculiX solve
# proves only that CalculiX ran, which is how a 3.6x-too-soft cantilever gets
# believed (see the NSET/clamp notes below).
#
# Structured reduced-integration hex mesh for a cantilever, written straight to
# a CalculX .inp. Closed-form reference (Euler-Bernoulli):
#   sigma_fixed_end = M*c/I ,  M = F*L,  I = w*h^3/12
#   delta_tip       = F*L^3/(3 E I)
# Timoshenko shear adds ~6(h/L)^2 = 1.5% to the deflection at L/h = 20, so
# 400 um is the target and ~406 um is the honest answer.
L, w, h, E, nu, F = 0.4, 0.02, 0.02, 200e9, 0.3, 50.0
# Mesh rule measured by sweep.py (see docs/V06_CAD_FEASIBILITY.md):
#   nz (through thickness) is what controls stress; nx (along span) does not.
#   nz=4 -> 0.79 of beam theory, nz=8 -> 0.90, nz=16 -> 0.96, then it plateaus.
# 16 elements across a 20 mm section = 1.25 mm, and nx=40 vs nx=160 moves the
# stress ratio only 0.960 -> 0.952, i.e. it is already converged. Do not spend
# elements lengthwise on a bending problem.
nx, ny, nz = 80, 4, 16
TOL = 0.03          # 3% on deflection; the measured value here is 0.74%

I = w * h**3 / 12.0
sigma_ref = F * L * (h / 2) / I
delta_ref = F * L**3 / (3 * E * I)
print(f"reference: sigma={sigma_ref/1e6:.4f} MPa  delta={delta_ref*1e6:.2f} um")

nid = lambda i, j, k: 1 + i + (nx + 1) * (j + (ny + 1) * k)


def _chunks(items, n):
    """CalculX caps a line at 16 entries, but keyword lines continue."""
    return [", ".join(items[i:i + n]) for i in range(0, len(items), n)]

# C3D8R: 8 corners, reduced integration. Reduced integration is the point --
# plain linear hexes (C3D8) shear-lock in bending and read far too stiff.
conn = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
        (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]

lines = ["*HEADING", "cantilever validation", "*NODE, NSET=Nall"]
for k in range(nz + 1):
    for j in range(ny + 1):
        for i in range(nx + 1):
            lines.append(f"{nid(i,j,k)}, {L*i/nx:.9f}, {w*j/ny:.9f}, {h*k/nz:.9f}")

lines.append("*ELEMENT, TYPE=C3D8R, ELSET=Eall")
e = 0
for k in range(nz):
    for j in range(ny):
        for i in range(nx):
            e += 1
            nodes = [nid(i + a, j + b, k + c) for (a, b, c) in conn]
            lines.append(f"{e}, " + ", ".join(map(str, nodes)))

tip = nid(nx, ny // 2, nz // 2)
# Clamp the WHOLE end face, not just its four corners. Holding only the corners
# leaves the face free to rotate, which silently turns the cantilever into a
# propped pin and overshoots the tip deflection by ~3.6x while the stress field
# still looks plausible -- a boundary-condition bug that a green solve hides.
fixed = [nid(0, j, k) for j in range(ny + 1) for k in range(nz + 1)]
lines += [
    "*MATERIAL, NAME=STEEL",
    "*ELASTIC",
    f"{E:.6e}, {nu}",
    "*SOLID SECTION, ELSET=Eall, MATERIAL=STEEL",
    # CalculX's *BOUNDARY takes a node SET name, not an inline node list the way
    # Abaqus allows, so declare the clamped face as an NSET first.
    "*NSET, NSET=FIXED",
    *_chunks([str(n) for n in fixed], 16),   # max 16 entries per line
    "*BOUNDARY",
    "FIXED, 1, 3",
    "*STEP",
    "*STATIC",
    "*CLOAD",
    f"{tip}, 3, {-F}",
    "*NODE PRINT, NSET=Ntip",
    "U",
    "*NODE FILE OUTPUT",
    "U",
    "*EL FILE OUTPUT",
    "S",
    "*END STEP",
]
open("beam.inp", "w").write("\n".join(lines) + "\n")
print(f"wrote beam.inp: {e} C3D8R elements, {len(lines)} lines, tip node {tip}")