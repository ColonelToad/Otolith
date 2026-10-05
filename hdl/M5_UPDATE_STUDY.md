# M5-C — Fixed-point update: numerical study

Date: 2026-09-25 · ADR-0007 · driver `fusion/src/fuse_update_study.cpp` ·
reference `fusion/fixed/fixed_update.hpp`

## Question

M4 closed the RTL loop on a kernel, not the core, and left the update path
entirely unmeasured. Before committing RTL to it (option B) or building a
kernel for it (option A), three things had to be answered with numbers:

1. **Does the update's linear algebra survive fixed point at all?** (C2)
2. **What are the update path's real ranges?** ADR-0007 M1 explicitly
   deferred these ("update-path ranges deferred to Cholesky"). (C3)
3. **What would the complete fixed-point MEKF cost in silicon?** (C5)

## Method

`FusionEKF` gained an `UpdateTrace` tap that records the *same* `H`, `y`,
`Rmat`, `S`, `K`, `dx` the filter computed — no re-derivation, no second code
path. The driver then runs the pure-float filter over a 10 s log and, at every
real update step, runs `update_linear` on the **same pre-update `P`** with the
same `H`/`y`/`Rmat` converted to fixed point.

Isolation matters: the float `P` trajectory drives the inputs, so this measures
the update's linear algebra *alone*. Predict-path quantization was measured
separately in M1 (0.1023 m vs 0.1064 m float) and is not double-counted.

## Factorization: LDLᵀ, not Cholesky

`S = H P Hᵀ + Rmat` is SPD by construction — `Rmat = σ_leg²·I` with
`σ_leg = 0.3`, so `Rmat = 0.09·I`. That dominates `S`'s diagonal and is why the
unpivoted, division-only LDLᵀ is safe here:

- no square roots (`D` is diagonal, not `√D`) — removes 9 `invsqrt6` instances
- no divisions in either triangular solve (`L` is unit-diagonal) — removes
  2·rows²/2 divisions

What remains is `rows` divisions in the factor and `rows` per solve. A Cholesky
route would need 9 sqrt + ~162 solve divisions for the same answer.

## Results (4999 real updates, 10 s log, dt = 2 ms)

| width | updates | share |
|---|---|---|
| rows = 6 (2 stance feet) | 4501 | 90.0% |
| rows = 12 (4 stance feet) | 498 | 10.0% |

**Accuracy vs the float filter's own arithmetic**

| DivMode | dx max abs err | vs signal | P' max abs err | P' rel Frobenius | non-PD | saturations |
|---|---|---|---|---|---|---|
| (a) Q16.48 reciprocal | 9.83e-08 | 8.2e-07 rel | 1.67e-08 | 3.18e-07 | 0/4999 | 0 |
| (b) Q8.24 `div_scaled` | 2.47e-07 | 2.1e-06 rel | 1.58e-08 | 3.30e-07 | 0/4999 | 0 |

Reference `max|dx| = 0.119603`. Per-block errors are uniform (dtheta 9.8e-08,
dv 9.0e-08, dp 9.0e-08, dbg 9.2e-08, dba 9.0e-08) — no state block is
pathological.

**Envelopes — the M1 deferral, closed**

| quantity | measured | note |
|---|---|---|
| max abs S | 0.200053 | `Rmat`'s 0.09 plus `HPHᵀ` |
| max abs L | 0.542742 | unit-diagonal L, so `\|L\|<1` holds |
| max abs D | 0.194047 | |
| **min abs D** | **0.0910808** | LDL pivot floor — never small |
| max abs K | 0.20994 | matches float trace exactly |
| max abs dx | 0.119604 | |
| max abs AP | 0.0987482 | the Joseph intermediate `A·P` |
| cond(S) | 30.5 | |

The pivot floor is the number that mattered. `σ_leg = 0.3` makes `Rmat` a
genuine 0.09·I floor, so `D[i]` never approaches zero and the division's
denormalizing shift stays in a narrow band. That is *why* fixed point is
comfortable here, and it is a tuning-knob consequence, not luck: shrinking
`σ_leg` shrinks `D` and moves the division toward its rail.

**Correction to the earlier record:** ADR-0007 notes `cond(S)` 1.026–5.705 and
"m=3:2239 m=4:261". Both are wrong for this configuration. Measured `cond(S)`
is **30.5**, and the stance-width mix is **m=2:4501 / m=4:498** (a clean
diagonal trot). `cond(S)` still leaves ample margin at `D ≥ 0.091`, so no
conclusion changes — but the numbers on record were not measured from this
log and should not have been stated as if they were.

## C2: which division precision?

The question was whether Q16.48 dividends (`div_scaled` is Q8.24-only) force a
64-bit reciprocal. Measured: **they don't have to.**

- (a) full Q16.48 reciprocal (`clz64` + 8 N-R iterations in Q16.48): dx error
  9.83e-08
- (b) narrow both sides to Q8.24, reuse the tested `div_scaled`, widen back:
  dx error 2.47e-07 — **2.5× worse, still ~2e-06 relative**

Both are format-limited, not iteration-limited: with `m ∈ [0.5,1)` the
Newton error squares from `e₀ ≤ 0.5`, so 8 iterations reach `2^-256` and the
Q8.24/Q16.48 rounding floor binds first.

**Decision for phase A: (a)** — 2.5× accuracy for 4× the multiplier width is a
bad trade in an FPGA/ASIC where the multiplier dominates area, *except* that
(b)'s cost is a 32×32 rather than 64×64 reciprocal. Phase A should take (a)
if the divider is reused for anything wide, (b) if the kernel's only job is
`num/D[i]`. Recorded as a live choice, not a settled one.

## C5: cost, and what the full MEKF actually is

MAC counts are analytic (fixed by the algorithm shape); area scales them by
M4's measured sky130 density (3.40 mm² / 6750 MAC).

| op | rows=6 | rows=9 | rows=12 |
|---|---|---|---|
| `S = HPHᵀ` | 1665 | 2700 | 3870 |
| LDLᵀ factor | 35 | 120 | 286 |
| `S⁻¹` (unit-vector solves) | 180 | 648 | 1584 |
| `K = T·S⁻¹` | 540 | 1215 | 2160 |
| `dx = Ky` | 90 | 135 | 180 |
| `A = I − KH` | 1350 | 2025 | 2700 |
| `APAᵀ` | 6750 | 6750 | 6750 |
| `KRKᵀ` | 1890 | 3240 | 4860 |
| **total** | **12500** | **16833** | **22390** |

At the observed mix: **6.79 mm²** for the update, **10.19 mm²** for the
complete fixed-point MEKF. (Linear MAC scaling is an upper bound — it ignores
shared state and control overhead. The width-independent `APAᵀ` is 54% of the
update at rows=6, which is the honest headline: **the update is dominated by
the Joseph form's 15×15×15 product, not by the factorization.**)

This settles the option-B question. Porting the full update to RTL is ~10 mm²
of sky130 — a real chip, not an afternoon flow, and 3× the full predict core
that M4 already found unplaceable in a 1600×1600 die. **Phase A (a kernel) is
the right scope; option B is not.**

## What the differential caught

Five bugs, all in `fixed_update.hpp`, all found by comparing against the float
trace — none by inspection:

1. **Mixed fractional-bit units.** The LDLᵀ accumulator mixed a 48-fractional-bit
   diagonal entry with 96-fractional-bit `L·D` products. Every pivot past the
   first railed: 107/107 non-PD.
2. **`recip48` discarded its rescale.** It computed `r << s`, range-checked the
   result, and never stored it — so every divisor below 1.0 returned `1/m`
   instead of `1/x`.
3. **`recip48` built `m` at 96 fractional bits**, making `m·r` a 144-bit product
   that `narrow96to48` under-shifted.
4. **`K` solved with `T`'s rows**, which computes `S⁻¹PHᵀ` — the inverse on the
   *left*. `K = T·S⁻¹` needs `S⁻¹`'s *columns*. Caught by `max|K| = 0.056`
   against the float trace's 0.210.
5. **`(KRKᵀ)[i][j]` indexed `K[k][j]` with stride `N_ST`** on a `rows`-stride
   array, then `(k,j)` instead of `(j,k)`: ~25% of the term, which the Joseph
   form then amplified into a 13% `P'` error.

The general lesson, now enforced by a header comment: with three formats in
one datapath, **every accumulator's fractional width must be tracked
explicitly**, and there is a named narrowing per width pair
(`narrow()` 48→24, `narrow48()` 72→48, `narrow96to48()` 96→48,
`narrow72to24()` 72→24).

A 3×3 smoke matrix masked bugs 4 and 5 behind structural zeros (`L[2][0]=0`,
pivots that were powers of two). The dense-9×9 and end-to-end tests now in
`fusion/tests/test_fixed.cpp` exist specifically to close that, and were
mutation-checked: reintroducing each bug class fails the suite.

## Gates

`ctest` 21 (was 17) — four new cases: `recip48` exactness, `div48` both modes,
LDLᵀ reconstruction + `S⁻¹` residual on a dense SPD, and `update_linear`
end-to-end vs float. `pytest` 15, `cargo` 9+9.

## Phase A: the `ldl_kernel` (closed)

`hdl/rtl/ldl_kernel.sv`, WIDTH=6. **Bit-parity 9002/9002** against
`ldl_factor` + `ldl_solve` on the real captured covariances. PPA and the
SKY130 ABC wall are in `hdl/README.md`'s M5-A section and
`hdl/openlane/ldl/config.yaml`.

## Phase B: fully fixed-point filter (closed)

`fusion/fixed/fixed_meas.hpp` builds `H` and `y` in fixed point — Go2 leg
geometry in Q8.24, the planar 2R FK, `skew`, `R*[omega x r]`, `R*[r]`, and
`R*(omega x r + rdot)` with `rdot = (r_base - r_prev)/dt` — so predict,
measure and correct are all fixed and the error is attributable end to end.

Same 10 s log, same harness (`evaluate.py`, ESTM-v2):

| | pos RMSE | vel RMSE | att RMSE | saturations |
|---|---|---|---|---|
| float | 0.3334 m | 0.0946 m/s | 24.21 deg | — |
| M1 hybrid (fixed predict, float update) | 0.3083 m | 0.0936 m/s | 22.22 deg | — |
| **M5 fully fixed** | **0.3074 m** | **0.0934 m/s** | **22.19 deg** | **0** |

**The fixed update costs essentially nothing over the float update.** That is
the closure M5-C could not deliver while it injected `H`/`y` from float.

Three bugs, all caught by measurement rather than review:

- **The CORDIC's range is smaller than the Go2's knee.** `thigh + calf`
  reaches ~2.8 rad against +/-1.7433 rad convergence, so `cordic_sincos`
  saturated to the rail and returned garbage (44 saturations, 0.14 m of FK
  error). `sin_cos_wide` folds into [-pi/2, pi/2] first — fixed-iteration,
  exact for |theta| <= 3*pi.
- **`prev_qj` was overwritten before `r_prev` was read from it**, silently
  zeroing every `rdot`.
- **`mat3_mul_fixed(R, sum, tmp)` with a 3-element `sum`.** That helper
  indexes `B[0..8]`; arrays decay to pointers, so nothing caught it. It
  presented as `h[0] = 4.18` against a true -0.014 and a wholesale
  divergence (pos RMSE 119 m). Added a dedicated `mat3_vec_fixed`.

Element-wise from identical states, `H` matches float to **3.9e-07** and `y`
to **~4e-04**.

## sigma_leg sweep (closed): where min|D| starts to cost

`fuse_update_study --sigma-leg <v>`, 4999 updates each:

| sigma_leg | min abs D | dx max abs err | saturations |
|---|---|---|---|
| 0.001 | 1.92e-06 | 41.5 (signal is 12.95) | 194,098 |
| 0.003 | 1.48e-05 | 17.8 (signal 11.72) | 194,060 |
| 0.01 | 1.22e-04 | 5.9e-06 | 0 |
| 0.03 | 9.89e-04 | 7.1e-07 | 0 |
| 0.1 | 1.03e-02 | 1.6e-07 | 0 |
| **0.3 (shipped)** | **9.11e-02** | **9.8e-08** | **0** |
| 1.0 | 1.00e+00 | 9.0e-08 | 0 |

`min|D|` tracks `sigma^2` exactly, since `Rmat = sigma^2 I` dominates S's
diagonal. The cliff sits between 1.5e-05 and 1.2e-04, which is precisely
where `1/D` leaves the Q16.48 rail (`1/D <= 32768` needs `D >= 3.05e-05`).

**So the fixed-point update requires sigma_leg >= ~0.0055; at the shipped 0.3
there is ~1000x margin.** Note what fails below the cliff: the *reciprocal*
saturates, never the factorization — non-PD stays 0/4999 at every sigma
tested, including 0.001. A tighter noise assumption would need a wider
reciprocal (Q16.64 or a two-word representation), not a better factorization.

## Option B: declined on measured cost (not unfinished work)

The update path outside `ldl_kernel` is model-only. Three pieces:

- **The measurement model** (`fusion/fixed/fixed_meas.hpp`): leg FK,
  `cordic_sincos`, `rdot`, `H`, `y`. `cordic_sincos` is 20 shift-add iterations
  with no multiplier, so it ports cheaply — but it converges only to
  +/-1.7433 rad and the Go2 knee reaches ~2.8 rad, so the RTL port needs the
  same `sin_cos_wide` quadrant folding the model now has.
- **`apply_dx_fixed`**: `K*d` at 15xN, then the log-odds quaternion add. No
  multiplier needed (the 2^48 shift is the log-odds scale), but it is a state
  write on the hot path.
- **The 15x15 Joseph form**: `P' = P - K*S*K' + K*Rmat*K'`. **54% of the update
  cost at rows=6** per the table in C5 — the single largest term.

**This is option B, and C5 already declined it**: ~10.19 mm² of sky130 for the
complete fixed MEKF, 3x a predict core that M4 found unplaceable in a
1600x1600 die. The honest summary is that the partition was measured and the
cost was measured, and the cost said no. Listing this as "still open" would
imply unfinished work rather than a decision made on numbers.

Two things follow that are *not* optional:

- **The 10.19 mm² projection excludes all three pieces above**, so it is a
  lower bound, not an estimate.
- **The kernel is 0.3% of the update's arithmetic** (35 of 12,500 MACs at
  rows=6). It was worth building because cancellation and dynamic range are
  hard to get right, not because it is expensive — a correctness retirement,
  now complete at 9002/9002. Do not read "91 us, 22x inside budget" as
  evidence that the update is cheap to port; `APA^T` is a dense 15x15x15
  product, which is CPU or DSP-array work, and the natural question is whether
  the update belongs on the CPU at all, with `ldl_kernel` as the FPGA island
  and the v0.2 C-ring seam already built and measured to join them.
