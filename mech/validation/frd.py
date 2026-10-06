"""Minimal CalculX .frd reader.

Both nodal and element result blocks are introduced by " -4 <NAME>" and their
records are " -1 <id><values>", so the enclosing block name is the only thing
that tells them apart. Values are contiguous 12-char exponential fields with
no separator -- a negative number packs as exactly 12 chars, which is why
splitting on whitespace silently corrupts them.
"""
import sys

W = 12


def _fields(chunk):
    return [float(chunk[i:i + W].strip().replace("D", "E"))
            for i in range(0, len(chunk) - W + 1, W) if chunk[i:i + W].strip()]


def read_frd(path):
    blocks, name = {}, None
    with open(path) as fh:
        for line in fh:
            if line.startswith(" -4"):
                name = line[5:13].strip()
                blocks.setdefault(name, {})
            elif line.startswith(" -1") and name is not None:
                ident = int(line[3:13])
                blocks[name].setdefault(ident, []).extend(_fields(line[13:]))
    return blocks


def vmises(s):
    sxx, syy, szz, sxy, syz, sxz = s[:6]
    return (0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
            + 3.0 * (sxy ** 2 + syz ** 2 + sxz ** 2)) ** 0.5


if __name__ == "__main__":
    L, w, h, F = 0.4, 0.02, 0.02, 50.0  # metres
    # Closed form only needs E from the material; defaults to Al 6061-T6 to
    # match cantilever.py's default. Any mismatch is caught by the tolerance.
    E = float(sys.argv[3]) if len(sys.argv) > 3 else 68.9e9
    I = w * h**3 / 12.0
    b = read_frd(sys.argv[1] if len(sys.argv) > 1 else "beam.frd")
    # The element count along the span must be passed in: hardcoding it puts every
    # comparison station at the wrong x, which reads as physics that went wrong.
    NX = int(sys.argv[2]) if len(sys.argv) > 2 else 80

    uz = {k: v[2] for k, v in b["DISP"].items()}
    tip = min(uz.values())
    ref = F * L**3 / (3 * E * I)
    err = abs(abs(tip) - ref) / ref
    print(f"tip deflection   FEA {tip*1e6:8.2f} um   "
          f"Euler-Bernoulli {ref*1e6:8.2f} um   err {err*100:5.2f}%")

    # Stress at the outer fibre is only meaningful away from the clamp, where the
    # fixed face is singular, so compare stations rather than taking a max. The
    # last station is dropped: beam theory goes to zero at the tip, so the ratio
    # there is numerical noise rather than a measurement.
    sig = b["STRESS"]
    print("\nstation   x[mm]   FEA |Sxx|[MPa]   beam theory[MPa]   ratio")
    ratios = []
    for el in [max(2, NX // 8), NX // 4, NX // 2, 3 * NX // 4]:
        sxx = max(abs(sig[e][0]) for e in sig if e in range(el, el + 1))
        x = L * el / NX
        theory = F * (L - x) * (h / 2) / I / 1e6
        ratio = sxx / 1e6 / theory
        ratios.append((ratio, el))
        print(f"  {el:3d}    {x*1000:6.1f}      {sxx/1e6:8.2f}        "
              f"{theory:8.2f}      {ratio:5.3f}")

    # Gate on deflection, and on the stress SHAPE rather than its magnitude: a
    # constant offset is reduced integration's known floor and is correctable,
    # whereas a ratio that wanders across stations means the mesh is wrong and
    # nothing downstream of it can be trusted.
    TOL_D, TOL_SHAPE = 0.03, 0.07
    worst = max((abs(r - 1.0), e, r) for r, e in ratios)
    print(f"\ndeflection err {err*100:.2f}% (tol {TOL_D*100:.0f}%)  "
          f"stress ratio {min(r for r,_ in ratios):.3f}"
          f"-{max(r for r,_ in ratios):.3f} "
          f"(worst deviation {worst[0]*100:.1f}% at element {worst[1]}, "
          f"tol {TOL_SHAPE*100:.0f}%)")
    ok = err < TOL_D and worst[0] < TOL_SHAPE
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)