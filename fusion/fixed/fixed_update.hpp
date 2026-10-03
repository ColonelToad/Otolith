#pragma once
// Fixed-point UPDATE path — bit-model of fusion.cpp:106-128 (ADR-0007 M5-C).
//
// Mirrors the float update's linear algebra in Q16.48 with 128-bit
// accumulators: S = H P H' + Rmat, factor, K = P H' S^-1, dx = K y, and
// the Joseph form (I-KH)P(I-KH)' + K Rmat K'. H and y are INJECTED as
// fixed-point values (the caller owns the FK/CORDIC measurement model —
// see scope note below), exactly as the float path receives them.
//
// FACTORIZATION: unpivoted LDL', not Cholesky. S = H P H' + Rmat is SPD
// by construction — Rmat = sigma_leg_vel^2 * I with sigma_leg_vel = 0.3,
// so Rmat = 0.09*I, which also DOMINATES S's diagonal (measured Sdiag
// 0.091..0.20 vs S maxabs 0.20). LDL' therefore needs:
//   - no square roots at all (D is diagonal, not sqrt(D)),
//   - no divisions in either triangular solve (L is unit-diagonal),
// leaving exactly rows divisions (one per D[i]) in the factor and rows
// in each solve. A Cholesky route would need 9 sqrt + 162 solve
// divisions for the same result. Unpivoted is safe here because S is
// SPD by construction and cond(S) measured 1.026..5.705.
//
// Scope note: this header deliberately stops at the linear algebra.
// Building H and y needs a fixed-point leg FK (sin/cos via
// cordic_sincos, which has NO rtl mirror yet) and a dt division; that is
// a separate concern tracked after the C2/C3/C5 measurements, which do
// not depend on it.
//
// Division precision is switchable because that IS the open question
// (ADR-0007 M5): div_scaled (fixed.hpp) is Q8.24-only, but LDL' pivots
// are Q16.48. See DivMode.

#include <cstdint>
#include <limits>

#include "fixed/fixed.hpp"
#include "fixed/fixed_predict.hpp"

namespace otolith {
namespace fixed {

constexpr int N_ST = 15;  // MEKF state dimension
constexpr int MAX_ROWS = 12; // 3 * 4 stance legs

// ---------------------------------------------------------------------------
// FRACTIONAL-UNIT CONVENTION (the whole point of this header's care).
//
// The update mixes three formats, so every accumulator's fractional width
// must be tracked or the arithmetic silently changes scale. With
// FRAC24 = 24 (q24) and FRAC48 = 48 (q48):
//
//   q24 x q24  -> 48 frac  -> narrow()         (shift 24, back to q24)
//   q24 x q48  -> 72 frac  -> narrow48()       (shift 24, back to q48)
//   q48 x q48  -> 96 frac  -> narrow96to48()   (shift 48, back to q48)
//   q48 x q24  -> 72 frac  -> narrow72to24()   (shift 48, back to q24)
//
// narrow48() (fixed_predict.hpp) is the 72->48 case. Using it on a 96-frac
// product divides by 2^24 too few and drove every LDL pivot to the rail in
// the first study run — hence this explicit second narrowing. Likewise
// narrow()'s 48->24 shift must NOT be reused for a 72-frac accumulator
// (dx = K y and A = I - K H): that left both 2^24 too large and railed.
// RTL mirror: the same shift amount.
inline constexpr int FRAC24 = 24;
inline constexpr int FRAC48 = 48;

// acc128 at 72 fractional bits -> Q8.24, round-half-away + saturation.
// RTL mirror: s_narrow (shift 48 on the K*H dot products).
inline q24 narrow72to24(acc128 x) {
    const acc128 half =
        (x >= 0) ? (acc128(1) << (FRAC48 - 1)) : -(acc128(1) << (FRAC48 - 1));
    const acc128 v = (x + half) >> FRAC48;
    if (v > acc128(std::numeric_limits<q24>::max())) {
        ++sat_count();
        return std::numeric_limits<q24>::max();
    }
    if (v < acc128(std::numeric_limits<q24>::min())) {
        ++sat_count();
        return std::numeric_limits<q24>::min();
    }
    return q24(v);
}

inline q48 narrow96to48(acc128 x) {
    const acc128 half =
        (x >= 0) ? (acc128(1) << (FRAC48 - 1)) : -(acc128(1) << (FRAC48 - 1));
    const acc128 v = (x + half) >> FRAC48;
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

// ---------------------------------------------------------------------------
// Division precision modes (the C2 experiment).
// ---------------------------------------------------------------------------
enum class DivMode {
    Q48, // (a) full Q16.48 reciprocal: 64-bit clz + 8 N-R iters in Q16.48
    Q24, // (b) Q16.48 operands narrowed to Q8.24, div_scaled, widened back
};

inline int clz64(uint64_t x) {
    if (x == 0) return 64;
    return __builtin_clzll(x);
}

// Q16.48 saturating subtract (widening; exact). RTL mirror: s_sub64.
inline q48 psub(q48 a, q48 b) {
    const acc128 d = acc128(a) - acc128(b);
    if (d > acc128(std::numeric_limits<q48>::max())) {
        ++sat_count();
        return std::numeric_limits<q48>::max();
    }
    if (d < acc128(std::numeric_limits<q48>::min())) {
        ++sat_count();
        return std::numeric_limits<q48>::min();
    }
    return q48(d);
}

// 1/x in Q16.48, fixed 8 N-R iterations, seed 1.0 on a divisor normalized
// into [0.5, 1). Error squares per step (e0 <= 0.5 -> e8 ~ 2^-256), so the
// 8-iteration count is FORMAT-limited (~6e-8/2^48), not iteration-limited —
// same conclusion div_scaled reaches for Q8.24.
// x == 0 returns 0; callers guard and count the saturation themselves.
inline q48 recip48(q48 x) {
    if (x == 0) return 0;
    const bool neg_out = x < 0;
    // |x| as unsigned: two's complement handles INT64_MIN without UB.
    const uint64_t ax = neg_out ? (~uint64_t(x) + 1ull) : uint64_t(x);
    // m = |x| * 2^s lands in [0.5, 1), i.e. integer [2^47, 2^48) — so the
    // normalized MSB must sit at bit 47: p + s = 47 with p = 63 - clz.
    const int s = clz64(ax) - 16;
    // m is a Q16.48 NUMBER (48 fractional bits), not a 96-bit accumulator:
    // s places ax's MSB at bit 47 so m < 2^48 and cannot overflow. Keeping
    // it 48-frac is what makes m*r a 96-frac product that narrow96to48
    // accepts — building m at 96 frac pushed the product to 144 frac and
    // left every N-R iterate 2^48 too large.
    const q48 m = q48((s >= 0) ? (acc128(ax) << s) : (acc128(ax) >> (-s)));
    const q48 TWO = q48(1) << 49; // 2.0 in Q16.48
    const q48 ONE48 = q48(1) << 48;
    q48 r = ONE48;
    for (int i = 0; i < 8; ++i) {
        const q48 mr = narrow96to48(m * acc128(r)); // m*r, 96 frac
        r = narrow96to48(acc128(r) * acc128(psub(TWO, mr)));
    }
    // 1/|x| = 2^s / m, so the rescale must be APPLIED. (An earlier version
    // computed the shifted value, range-checked it, and never stored it —
    // every divisor below 1.0 silently returned 1/m instead of 1/x, which
    // wrecked L and left the reconstruction of S off by ~47%.)
    if (s >= 0) {
        const acc128 q = acc128(r) << s;
        if (q > acc128(std::numeric_limits<q48>::max())) {
            ++sat_count();
            r = std::numeric_limits<q48>::max();
        } else {
            r = q48(q);
        }
    } else {
        r = q48(r >> (-s)); // shrinks, cannot overflow
    }
    if (!neg_out) return r;
    if (r == std::numeric_limits<q48>::min()) {
        ++sat_count();
        return std::numeric_limits<q48>::max();
    }
    return q48(-r);
}

// Q16.48 / Q16.48 under the selected precision mode.
inline q48 div48(q48 num, q48 den, DivMode mode) {
    if (den == 0) {
        ++sat_count();
        return num >= 0 ? std::numeric_limits<q48>::max()
                        : std::numeric_limits<q48>::min();
    }
    if (mode == DivMode::Q24) {
        // Narrow both sides to Q8.24, use the tested Q8.24 primitive, widen.
        const q24 n24 = narrow(num), d24 = narrow(den);
        const q24 q24r = div_scaled(n24, d24);
        return q48(acc128(q24r) << 24); // value * 2^48, exact widening
    }
    return narrow96to48(acc128(num) * acc128(recip48(den)));
}

// ---------------------------------------------------------------------------
// LDL' factorization + solve (Q16.48).
// ---------------------------------------------------------------------------
struct LdlFactor {
    int n = 0;
    q48 L[MAX_ROWS * MAX_ROWS] = {0}; // strictly-lower, row-major, unit diag
    q48 D[MAX_ROWS] = {0};
    bool pd = false; // false if any pivot D[i] <= 0 (not positive-definite)
};

// In-place LDL' of symmetric S (row-major n*n, upper ignored).
// Golub & Van Loan Alg. 4.2.4 form: D[j] = S[j][j] - sum L[j][k]^2 D[k],
// L[i][j] = (S[i][j] - sum L[i][k] D[k] L[j][k]) / D[j].
// One narrowing per term (L*D -> Q16.48), one acc128 accumulation per
// entry: every product stays <= 96 fractional bits, well inside the
// 127-bit accumulator. Triple-width L*L*D would NOT fit — see ADR-0007.
inline bool ldl_factor(const q48* S, int n, LdlFactor& f, DivMode mode) {
    q48 A[MAX_ROWS * MAX_ROWS];
    for (int i = 0; i < n * n; ++i) A[i] = S[i];
    f.n = n;
    f.pd = true;
    for (int j = 0; j < n; ++j) {
        // Working unit is 96 fractional bits so the Q16.48 diagonal entry and
        // the L*D products (both q48 x q48 -> 96 frac) are commensurate; each
        // entry is narrowed back to Q16.48 exactly once. Mixing the units
        // drove every pivot to the rail — caught by the first study run
        // reporting 107/107 non-PD factorizations.
        acc128 dj = acc128(A[j * n + j]) << FRAC48;
        for (int k = 0; k < j; ++k) {
            const q48 ljk = A[j * n + k];
            const q48 ld = narrow96to48(acc128(ljk) * acc128(f.D[k]));
            dj -= acc128(ljk) * acc128(ld);
        }
        f.D[j] = narrow96to48(dj);
        if (!(f.D[j] > 0)) { // non-PD: stop, leave pd=false for the caller
            f.pd = false;
            return false;
        }
        for (int i = j + 1; i < n; ++i) {
            acc128 s = acc128(A[i * n + j]) << FRAC48;
            for (int k = 0; k < j; ++k) {
                // L[i][k] * D[k] * L[j][k] — the inner factor is L[j][k], the
                // row BEING FACTORED. Using L[i][k] for both gives
                // L[i][k]^2 D[k] instead, which is right only when k==j==0 or
                // when L[i][k]==0; it left L[3][1] at -0.284 against Eigen's
                // -0.00083. The 3x3 smoke matrix masked it via L[2][0]==0.
                const q48 ld = narrow96to48(acc128(A[i * n + k]) * acc128(f.D[k]));
                s -= acc128(A[j * n + k]) * acc128(ld);
            }
            A[i * n + j] = div48(narrow96to48(s), f.D[j], mode);
        }
    }
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < i; ++j) f.L[i * n + j] = A[i * n + j];
    return true;
}

// Solve S^-1 b in place from the factor (S itself is not needed).
// Forward L z = b (unit diagonal, no division), scale by 1/D[i], back L' x.
// Same 96-frac working unit as the factorization; b[i] is Q16.48.
inline void ldl_solve(const LdlFactor& f, q48* b) {
    const int n = f.n;
    for (int i = 0; i < n; ++i) { // forward substitution
        acc128 s = acc128(b[i]) << FRAC48;
        for (int k = 0; k < i; ++k) s -= acc128(f.L[i * n + k]) * acc128(b[k]);
        b[i] = narrow96to48(s);
    }
    for (int i = 0; i < n; ++i) b[i] = div48(b[i], f.D[i], DivMode::Q48);
    for (int i = n - 1; i >= 0; --i) { // back substitution
        acc128 s = acc128(b[i]) << FRAC48;
        for (int k = i + 1; k < n; ++k) s -= acc128(f.L[k * n + i]) * acc128(b[k]);
        b[i] = narrow96to48(s);
    }
}

// ---------------------------------------------------------------------------
// The update itself.
// ---------------------------------------------------------------------------
struct UpdateOut {
    int rows = 0;
    bool pd = false;
    q24 dx[N_ST] = {0}; // Q8.24 state correction (mirrors dx.segment blocks)
};

// Optional capture of the update's intermediate gain/shape matrices. Used by
// the study driver to compare against the float trace element-wise (max
// magnitudes alone hid a transposed-factorization bug).
struct UpdateDump {
    q48 K[N_ST * MAX_ROWS] = {0};      // gain, 15 x rows
    q24 A[N_ST * N_ST] = {0};          // I - K H, 15 x 15 (Q8.24)
    q48 S[MAX_ROWS * MAX_ROWS] = {0};  // innovation covariance, rows x rows
    LdlFactor f;                       // LDL' factor
};

// Observed envelopes + saturation attribution for one update. Filled only
// when requested; the study driver aggregates it over the log and the
// standing ctest asserts its bounds (ADR-0007 M5-C).
struct UpdateRanges {
    double max_abs_S = 0.0;
    double max_abs_L = 0.0;
    double max_abs_D = 0.0;
    double max_abs_K = 0.0;
    double max_abs_dx = 0.0;
    double min_D = 0.0;   // smallest LDL pivot seen (the stall risk)
    double max_abs_AP = 0.0; // |A P| — the Joseph intermediate
    int sat_before = 0;
    int sat_delta = 0;
};

inline void note_range(double& slot, double v) {
    const double a = v < 0 ? -v : v;
    if (a > slot) slot = a;
}

// One MEKF update. H: rows x 15 Q8.24 (row-major), y: rows Q8.24,
// Rmat: rows x rows Q16.48 diagonal-block (sigma_leg_vel^2 * I).
// Mutates s.P (Joseph form) and returns dx for the caller to apply to the
// nominal state. Mirrors fusion.cpp:106-128.
inline UpdateOut update_linear(FixedState& s, const q24* H, const q24* y,
                               const q48* Rmat, int rows, DivMode mode,
                               UpdateRanges* rg = nullptr,
                               UpdateDump* dump = nullptr) {
    UpdateOut out;
    out.rows = rows;
    if (rg) {
        *rg = UpdateRanges{};
        rg->sat_before = sat_count();
    }

    // --- S = H P H' + Rmat (lower triangle computed, mirrored: guarantees
    // exact symmetry for the factorization instead of relying on the
    // arithmetic cancelling, and halves the work — what hardware should do)
    q48 T[N_ST * MAX_ROWS]; // T = P H'  (15 x rows)
    for (int a = 0; a < N_ST; ++a)
        for (int j = 0; j < rows; ++j) {
            acc128 sum = 0;
            for (int b = 0; b < N_ST; ++b)
                sum += acc128(s.P[a * N_ST + b]) * acc128(H[j * N_ST + b]);
            T[a * rows + j] = narrow48(sum);
        }
    q48 S[MAX_ROWS * MAX_ROWS] = {0};
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j <= i; ++j) {
            acc128 sum = 0;
            for (int a = 0; a < N_ST; ++a)
                sum += acc128(H[i * N_ST + a]) * acc128(T[a * rows + j]);
            q48 v = padd(narrow48(sum), Rmat[i * rows + j]);
            S[i * rows + j] = v;
            S[j * rows + i] = v;
        }
    if (rg)
        for (int i = 0; i < rows * rows; ++i) note_range(rg->max_abs_S, p_to_double(S[i]));

    // --- factor
    LdlFactor f;
    if (!ldl_factor(S, rows, f, mode)) {
        out.pd = false;
        return out; // caller must skip the correction (state untouched)
    }
    out.pd = true;
    if (dump) {
        for (int i = 0; i < rows * rows; ++i) dump->S[i] = S[i];
        dump->f = f;
    }
    if (rg) {
        for (int i = 0; i < f.n * f.n; ++i) note_range(rg->max_abs_L, p_to_double(f.L[i]));
        for (int i = 0; i < f.n; ++i) {
            note_range(rg->max_abs_D, p_to_double(f.D[i]));
            const double d = p_to_double(f.D[i]);
            if (d < rg->min_D || rg->min_D == 0.0) rg->min_D = d;
        }
    }

    // --- S^-1 materialized once (rows unit-vector solves), then K = T S^-1.
// (TA)[:,i] = T * A[:,i] and A[:,i] = A e_i, so column i of S^-1 comes from
// solving S x = e_i. The tempting shortcut — solving with row i of T because
// S is symmetric — is NOT valid: (TA)[:,i] = sum_a T[a][i]A[a][i] uses T's
// COLUMN i, while the shortcut computes sum_a A[i][a]T[i][a] which is T's
// ROW i. The study caught it: max|K| 0.056 against the float trace's 0.210.
    q48 Si[MAX_ROWS * MAX_ROWS] = {0};
    for (int c = 0; c < rows; ++c) {
        q48 e[MAX_ROWS] = {0};
        e[c] = p_from_double(1.0);
        ldl_solve(f, e);
        for (int j = 0; j < rows; ++j) Si[j * rows + c] = e[j];
    }
    q48 K[N_ST * MAX_ROWS];
    for (int i = 0; i < N_ST; ++i)
        for (int j = 0; j < rows; ++j) {
            acc128 sum = 0; // q48 x q48 -> 96 frac
            for (int k = 0; k < rows; ++k)
                sum += acc128(T[i * rows + k]) * acc128(Si[k * rows + j]);
            K[i * rows + j] = narrow96to48(sum);
        }

    // --- dx = K y  (Q8.24 out)
    for (int i = 0; i < N_ST; ++i) {
        acc128 sum = 0;
        for (int j = 0; j < rows; ++j) sum += acc128(K[i * rows + j]) * acc128(y[j]);
        out.dx[i] = narrow72to24(sum); // q48 x q24 -> 72 frac
    }
    if (rg)
        for (int i = 0; i < N_ST; ++i) note_range(rg->max_abs_dx, to_double(out.dx[i]));

    // --- Joseph: A = I - K H  (Q8.24; |A| <= 1 + |K||H| ~ 1.2)
    q24 A[N_ST * N_ST];
    for (int i = 0; i < N_ST; ++i)
        for (int j = 0; j < N_ST; ++j) {
            acc128 sum = 0;
            for (int k = 0; k < rows; ++k)
                sum += acc128(K[i * rows + k]) * acc128(H[k * N_ST + j]);
            A[i * N_ST + j] = narrow72to24(-sum); // I - KH, delta added below
        }
    for (int i = 0; i < N_ST; ++i) A[i * N_ST + i] = add(A[i * N_ST + i], ONE);
    if (dump)
        for (int i = 0; i < N_ST * N_ST; ++i) dump->A[i] = A[i];

    // --- P' = A P A' + K Rmat K'   (all Q16.48)
    q48 AP[N_ST * N_ST];
    for (int i = 0; i < N_ST; ++i)
        for (int j = 0; j < N_ST; ++j) {
            acc128 sum = 0;
            for (int k = 0; k < N_ST; ++k)
                sum += acc128(A[i * N_ST + k]) * acc128(s.P[k * N_ST + j]);
            AP[i * N_ST + j] = narrow48(sum);
        }
    if (rg)
        for (int i = 0; i < N_ST * N_ST; ++i) note_range(rg->max_abs_AP, p_to_double(AP[i]));
    q48 KR[N_ST * MAX_ROWS]; // K Rmat (q48 x q48 -> 96 frac)
    for (int i = 0; i < N_ST; ++i)
        for (int j = 0; j < rows; ++j) {
            acc128 sum = 0;
            for (int k = 0; k < rows; ++k)
                sum += acc128(K[i * rows + k]) * acc128(Rmat[k * rows + j]);
            KR[i * rows + j] = narrow96to48(sum);
        }

    // P' = A P A' + K Rmat K'. Both terms narrowed to Q16.48 BEFORE being
    // added (they carry different fractional widths: 72 and 96), then the
    // off-diagonal pair is averaged with rounded halving — the same
    // symmetrize idiom as the predict path, and it must average (i,j) with
    // (j,i), not merely halve a single entry.
    q48 Pn[N_ST * N_ST];
    for (int i = 0; i < N_ST; ++i)
        for (int j = 0; j < N_ST; ++j) {
            acc128 sum = 0; // A P A' : q48 x q24 -> 72 frac
            for (int k = 0; k < N_ST; ++k)
                sum += acc128(AP[i * N_ST + k]) * acc128(A[j * N_ST + k]);
            acc128 kr = 0; // (K Rmat) K' : q48 x q48 -> 96 frac
            for (int k = 0; k < rows; ++k)
                // (K Rmat K')[i][j] = sum_k KR[i][k] * K[j][k]  — the second factor is
                // K[j][k] (K TRANSPOSED), not K[k][j]. Two separate index
                // mistakes lived here: stride N_ST on a rows-stride array,
                // and then (k,j) instead of (j,k). Together they returned
                // ~25% of the term. K is 15 x rows, so the correct
                // subscript is K[j * rows + k].
                kr += acc128(KR[i * rows + k]) * acc128(K[j * rows + k]);
            Pn[i * N_ST + j] = padd(narrow48(sum), narrow96to48(kr));
        }
    for (int i = 0; i < N_ST; ++i)
        for (int j = 0; j < i; ++j) {
            const acc128 halves = acc128(Pn[i * N_ST + j]) + acc128(Pn[j * N_ST + i]);
            const acc128 h = (halves >= 0) ? acc128(1) : acc128(-1);
            acc128 sym = (halves + h) >> 1;
            if (sym > acc128(std::numeric_limits<q48>::max())) {
                ++sat_count();
                sym = acc128(std::numeric_limits<q48>::max());
            }
            if (sym < acc128(std::numeric_limits<q48>::min())) {
                ++sat_count();
                sym = acc128(std::numeric_limits<q48>::min());
            }
            Pn[i * N_ST + j] = Pn[j * N_ST + i] = q48(sym);
        }
    for (int i = 0; i < N_ST * N_ST; ++i) s.P[i] = Pn[i];
    if (dump)
        for (int i = 0; i < N_ST * rows; ++i) dump->K[i] = K[i];
    if (rg) {
        for (int i = 0; i < N_ST * rows; ++i) note_range(rg->max_abs_K, p_to_double(K[i]));
        rg->sat_delta = sat_count() - rg->sat_before;
    }
    return out;
}

// Apply dx to the nominal state (mirrors fusion.cpp:116-120). Uses the
// SAME first-order exp + N-R normalize as the predict path
// (fixed_predict.hpp), so there is no second quaternion convention.
inline void apply_dx_fixed(FixedState& s, const q24 dx[N_ST]) {
    const q24 TWO = ONE * 2;
    q24 ev[3] = {mul(dx[0], ONE), mul(dx[1], ONE), mul(dx[2], ONE)};
    for (int i = 0; i < 3; ++i) { // h = dtheta/2, first order
        const acc t = acc(ev[i]);
        ev[i] = q24((t + (ev[i] >= 0 ? acc(1) : acc(-1))) >> 1);
    }
    acc en2 = acc(ONE) * acc(ONE);
    for (int i = 0; i < 3; ++i) en2 = acc_add(en2, acc(ev[i]) * acc(ev[i]));
    const q24 einv = invsqrt_nr(narrow(en2));
    const q24 eq[4] = {mul(ONE, einv), mul(ev[0], einv), mul(ev[1], einv),
                       mul(ev[2], einv)};
    const q24 q0 = s.q[0], q1 = s.q[1], q2 = s.q[2], q3 = s.q[3];
    const q24 e0 = eq[0], e1 = eq[1], e2 = eq[2], e3 = eq[3];
    q24 nq[4];
    nq[0] = sub(sub(sub(mul(q0, e0), mul(q1, e1)), mul(q2, e2)), mul(q3, e3));
    nq[1] = sub(add(add(mul(q0, e1), mul(q1, e0)), mul(q2, e3)), mul(q3, e2));
    nq[2] = sub(add(add(mul(q0, e2), mul(q2, e0)), mul(q3, e1)), mul(q1, e3));
    nq[3] = sub(add(add(mul(q0, e3), mul(q3, e0)), mul(q1, e2)), mul(q2, e1));
    acc qn2 = 0;
    for (int i = 0; i < 4; ++i) qn2 = acc_add(qn2, acc(nq[i]) * acc(nq[i]));
    const q24 inv = invsqrt_nr(narrow(qn2));
    for (int i = 0; i < 4; ++i) s.q[i] = mul(nq[i], inv);
    for (int i = 0; i < 3; ++i) {
        s.v[i] = add(s.v[i], dx[3 + i]);
        s.p[i] = add(s.p[i], dx[6 + i]);
        s.bg[i] = add(s.bg[i], dx[9 + i]);
        s.ba[i] = add(s.ba[i], dx[12 + i]);
    }
    (void)TWO;
}

} // namespace fixed
} // namespace otolith