#pragma once
// Q8.24 fixed-point foundation for the v0.4 model (ADR-0007).
//
// Storage: int32, range ±128, LSB 2^-24 ≈ 6e-8. Accumulators: int64
// (sums of Q8.24xQ8.24 products stay Q16.48 in 64 bits; narrowed once).
// Rounding: half away from zero on narrowing. Overflow: saturate.
// Model-only: lives beside the float path, never inside it.
//
// Every operation here is exactly specifiable in RTL: the CORDIC and
// Newton-Raphson routines use FIXED iteration counts, so hdl/rtl copies
// the algorithm, not just the results.

#include <cmath>
#include <cstdint>
#include <limits>

namespace otolith {
namespace fixed {

using q24 = int32_t;   // Q8.24 storage (states, Phi, inputs)
using acc = int64_t;   // 64-bit accumulator (Q16.48 product sums)
using q48 = int64_t;   // Q16.48 storage (covariance P — see below)
using acc128 = __int128; // 128-bit accumulator (mixed Q8.24xQ16.48 sums)
//
// Why P is 64-bit: P spans ~1e-9 (correlations, gain-structural) to ~2
// (predict-only growth/transients) = 11 decades. No 32-bit format covers
// both (Q8.24 LSB 6e-8 destroys correlations -> Joseph gains go wrong ->
// measured hybrid divergence 3.87 m; x256 block-scale rails at 0.5).
// Q16.48 (LSB 3.6e-15, rail 32768) covers everything with margin.
// Overflow-freedom of 128-bit sums: |Phi|<=~1.3 (int32), |P|<=2^49 actual
// (far below int64 capacity); products <= 2^80, x15 terms <= 2^84 <<
// 2^127. 43 bits of margin — no saturation logic in accumulators, only
// at the narrowing to int64 (genuine rail, counted).

inline constexpr int FRAC = 24;
inline constexpr q24 ONE = q24(1) << FRAC;          // 1.0
inline constexpr acc HALF = acc(1) << (FRAC - 1);   // half ULP (ULP of storage)
inline constexpr q24 QMAX = std::numeric_limits<q24>::max();
inline constexpr q24 QMIN = std::numeric_limits<q24>::min();

// Saturation counter: the model counts every saturating event so the
// error bound can attribute rail hits (F/a near ±128, see ADR-0007).
inline int& sat_count() {
    static int n = 0;
    return n;
}
inline void sat_reset() { sat_count() = 0; }

inline q24 saturate(acc x) {
    if (x > acc(QMAX)) { ++sat_count(); return QMAX; }
    if (x < acc(QMIN)) { ++sat_count(); return QMIN; }
    return q24(x);
}

inline q24 from_double(double x) {
    // round-half-away, then saturate
    double s = x * double(acc(1) << FRAC);
    acc r = (s >= 0) ? acc(std::floor(s + 0.5)) : -acc(std::floor(-s + 0.5));
    return saturate(r);
}

inline double to_double(q24 x) { return double(x) / double(acc(1) << FRAC); }

// Narrow a Q16.48 accumulator to Q8.24 with round-half-away + saturation.
inline q24 narrow(acc x) {
    acc half = (x >= 0) ? HALF : -HALF;
    return saturate((x + half) >> FRAC);
}

// Narrow a Q24.72 128-bit accumulator to Q16.48 storage (P pipeline)
// with round-half-away; saturates (counted) only past int64 — unreachable
// per the overflow-freedom proof above, kept for honesty.
inline q48 narrow48(acc128 x) {
    acc128 half = (x >= 0) ? (acc128(1) << 23) : -(acc128(1) << 23);
    acc128 v = (x + half) >> 24;
    if (v > acc128(std::numeric_limits<q48>::max())) {
        ++sat_count();
        return std::numeric_limits<q48>::max();
    }
    if (v < acc128(std::numeric_limits<q48>::min())) {
        ++sat_count();
        return std::numeric_limits<q48>::min();
    }
    return q48(v);
}

inline q24 add(q24 a, q24 b) { return saturate(acc(a) + acc(b)); }
inline q24 sub(q24 a, q24 b) { return saturate(acc(a) - acc(b)); }
inline q24 mul(q24 a, q24 b) {
    // product is Q16.48 in 64 bits; narrow once.
    return narrow(acc(a) * acc(b));
}
inline q24 neg(q24 a) {
    if (a == QMIN) { ++sat_count(); return QMAX; } // saturate -MIN, counted
    return q24(-a);
}

// Saturating 64-bit accumulator add. Products of two int32s always fit
// (2^62 < 2^63); only sums can overflow — on rail-only inputs. This is
// industry-standard DSP behavior (cf. ARM VQADD), exactly specifiable,
// and keeps every accumulation free of signed-overflow UB by
// construction. Overflow saturates and counts.
inline acc acc_add(acc a, acc b) {
    acc r = acc(uint64_t(a) + uint64_t(b)); // wrap is well-defined unsigned
    if ((a >= 0 && b >= 0 && r < 0) || (a < 0 && b < 0 && r >= 0)) {
        ++sat_count();
        return (a >= 0) ? std::numeric_limits<acc>::max()
                        : std::numeric_limits<acc>::min();
    }
    return r;
}

// Scaled division num/den in Q8.24 — the only division in the model.
// Plain N-R reciprocal overflows Q8.24 at both ends (small den: 1/den
// exceeds the rail; den > 2: diverges from seed 1.0), so the divisor is
// prescaled into [0.5, 1) via leading-zero count (a priority encoder in
// RTL), N-R runs 8 iterations there (error squares per step: e0 <= 0.5
// -> e8 ~ 2^-256, exact for our width), and the quotient is rescaled.
// Contract: caller ensures num<<s fits (true for axis normalization,
// the only user: |num| ~ den). den == 0 saturates to rail (counted).
// hdl/rtl copies the shift amounts and iteration count exactly.
inline int clz32(uint32_t x) {
    // Synthesizable priority encoder; __builtin_clz(0) is UB so guard.
    if (x == 0) return 32;
    return __builtin_clz(x);
}

inline q24 div_scaled(q24 num, q24 den) {
    if (den == 0) {
        ++sat_count();
        return (num >= 0) ? QMAX : QMIN;
    }
    // Work positive, reapply sign at the end (N-R seed +1 needs m > 0).
    const bool neg_out = (num < 0) != (den < 0);
    const uint32_t anum = (num == QMIN)
        ? uint32_t(QMAX) // ||num|| unrepresentable: 1-LSB under-read, no count
        : uint32_t(num < 0 ? -num : num);
    const uint32_t aden = (den == QMIN)
        ? uint32_t(QMAX) + 1u
        : uint32_t(den < 0 ? -den : den);
    const int s = clz32(aden) - 8; // m = aden*2^s in [0.5, 1)
    // m via exact shift (den side provably in range — no saturation).
    // NOTE: shifting the DIVISOR loses only relative-LSB precision
    // (~7e-8); the numerator is never pre-truncated (see below).
    const q24 m = q24((s >= 0) ? (acc(aden) << s) : (acc(aden) >> (-s)));
    // N-R reciprocal of m, seed 1.0, 8 fixed iterations (e0 <= 0.5).
    const q24 TWO = ONE * 2;
    q24 r = ONE;
    for (int i = 0; i < 8; ++i) {
        q24 xy = narrow(acc(m) * acc(r));
        r = narrow(acc(r) * acc(sub(TWO, xy)));
    }
    // quotient = (|num|*r)*2^s: full-precision numerator first, rescale
    // last with round-half-away. s>=0 path saturates (genuine rail only);
    // s<0 path shrinks (cannot overflow).
    const q24 an = (anum > uint32_t(QMAX)) ? QMAX : q24(anum);
    const q24 t = narrow(acc(an) * acc(r));
    q24 mag = (s >= 0) ? saturate(acc(t) << s)
                       : q24((acc(t) + (acc(1) << (-s - 1))) >> (-s));
    if (neg_out) return neg(mag);
    return mag;
}

// CORDIC sin/cos, rotation mode, 20 iterations, Q8.24 radians in.
// Convergence range |theta| <= 1.7433 rad; inputs outside saturate to
// the rail (counted). Model inputs (exp_quat half-angles, |h| <= 0.05
// in-envelope) never approach the rail.
// Table: atan(2^-i) in Q8.24 — hardcoded integers, no libm dependence,
// so the model is bit-exact across platforms and hdl/rtl copies values.
inline constexpr q24 CORDIC_ATAN[20] = {
    13176795, 7778716, 4110060, 2086331, 1047214, 524117, 262123,
    131069, 65536, 32768, 16384, 8192, 4096, 2048, 1024, 512,
    256, 128, 64, 32,
};
// CORDIC gain K20 = prod cos(atan(2^-i)) = 0.6072529350092496 -> 10188014
inline constexpr q24 CORDIC_K = 10188014;
inline constexpr q24 CORDIC_LIM = 29246861; // 1.7433 rad in Q8.24 (rail)

struct SinCos {
    q24 s;
    q24 c;
};

inline SinCos cordic_sincos(q24 theta) {
    if (theta > CORDIC_LIM) { ++sat_count(); theta = CORDIC_LIM; }
    if (theta < -CORDIC_LIM) { ++sat_count(); theta = -CORDIC_LIM; }
    // Arithmetic right shift on negatives is floor (C++20) — matches RTL
    // ashr. No saturation inside the loop: |x|,|y| stay < 1.21 (bounded
    // by construction: rotation preserves norm ~1/K20*K20... start norm
    // K20 < 1, rotations preserve it; shifts only shrink magnitudes).
    q24 x = CORDIC_K, y = 0, z = theta;
    for (int i = 0; i < 20; ++i) {
        q24 xd = x >> i, yd = y >> i; // arithmetic shift, exact
        if (z >= 0) {
            x = x - yd;
            y = y + xd;
            z = z - CORDIC_ATAN[i];
        } else {
            x = x + yd;
            y = y - xd;
            z = z + CORDIC_ATAN[i];
        }
    }
    return {y, x};
}
// Fixed-iteration Newton-Raphson inverse sqrt, seed y0 = 1.0.
// Converges for x near 1 (unit quats); fixed count, same contract.
// y <- y*((3 - x*y*y)/2), halving done in Q8.24 with round-half-away
// before the final narrow (three narrowings per iteration, all explicit).
inline q24 invsqrt_nr(q24 x, int iters = 6) {
    const q24 THREE = ONE * 3; // exact
    auto halve_away = [](q24 v) {
        return q24((acc(v) + (v >= 0 ? acc(1) : acc(-1))) >> 1);
    };
    q24 y = ONE;
    for (int i = 0; i < iters; ++i) {
        q24 t1 = narrow(acc(x) * acc(y));      // x*y
        q24 t2 = narrow(acc(t1) * acc(y));     // x*y*y
        q24 half = halve_away(sub(THREE, t2)); // (3-x*y*y)/2
        y = narrow(acc(y) * acc(half));
    }
    return y;
}

} // namespace fixed
} // namespace otolith
