// Bit-parity: Verilated fixed_pkg primitives vs the C++ model (ADR-0007 M5-A).
// Deliberately separate from the ldl_kernel bench: these four functions carry
// all the fractional-unit and normalization risk, so they get checked alone
// before any sequencing is layered on top.

#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <vector>

#include <cmath>
#include "Vfixed_pkg_wrap.h"
#include "fixed/fixed.hpp"
#include "fixed/fixed_update.hpp"

using otolith::fixed::acc128;
using otolith::fixed::clz64;
using otolith::fixed::narrow96to48;
using otolith::fixed::p_from_double;
using otolith::fixed::p_to_double;
using otolith::fixed::q48;
using otolith::fixed::recip48;
using otolith::fixed::sat_count;
using otolith::fixed::sat_reset;

static Vfixed_pkg_wrap* top = nullptr;

int main() {
    top = new Vfixed_pkg_wrap;
    top->eval();

    std::vector<uint64_t> vecs;
    // Values from the measured envelope: D in [0.091, 0.194], plus the
    // boundaries and signs that stress normalization.
    for (double d : {0.0910808, 0.09, 0.1, 0.19, 0.19262, 0.194047, 0.2, 0.25,
                     0.5, 1.0, 2.0, 3.7, 11.0, 1e-9, 1e-15})
        vecs.push_back((uint64_t)p_from_double(d));
    // Bit patterns: ±1, ±max, ±min, powers of two, and a deterministic sweep.
    const uint64_t kEdge[] = {0ull, 1ull, 2ull, (1ull << 47), (1ull << 48),
                              (1ull << 49), 0x7fffffffffffffffull,
                              0x8000000000000000ull, 0xffffffffffffffffull};
    for (uint64_t v : kEdge) {
        vecs.push_back(v);
        vecs.push_back(~v + 1ull); // magnitude mirror
    }
    uint64_t st = 0x123456789abcdefull;
    for (int i = 0; i < 400; ++i) {
        st = st * 6364136223846793005ull + 1442695040888963407ull;
        vecs.push_back(st);
    }

    size_t fails = 0;
    // --- clz64 ---
    for (uint64_t v : vecs) {
        top->clz_x = v;
        top->eval();
        const int want = clz64(v);
        if ((int)top->clz_out != want) {
            std::printf("FAIL clz64(%llu): rtl %d want %d\n",
                        (unsigned long long)v, (int)top->clz_out, want);
            if (++fails > 40) break;
        }
    }

    // --- recip48 (value + sat flag) ---
    for (uint64_t v : vecs) {
        sat_reset();
        const q48 got = recip48((q48)v);
        const bool want_sat = sat_count() > 0;
        top->recip_x = v;
        top->eval();
        if ((uint64_t)top->recip_out != (uint64_t)got ||
            (top->recip_sat != 0) != want_sat) {
            std::printf("FAIL recip48(%llu): rtl %llu sat %d | want %llu sat %d\n",
                        (unsigned long long)v, (unsigned long long)top->recip_out,
                        (int)top->recip_sat, (unsigned long long)got,
                        (int)want_sat);
            if (++fails > 40) break;
        }
    }

    // --- narrow96to48 over products of the above (96 fractional bits) ---
    for (size_t i = 0; i + 1 < vecs.size(); i += 2) {
        const acc128 prod = acc128((q48)vecs[i]) * acc128((q48)vecs[i + 1]);
        sat_reset();
        const q48 got = narrow96to48(prod);
        const bool want_sat = sat_count() > 0;
        // RTL computes the same thing via s_mulq48.
        top->mul_a = vecs[i];
        top->mul_b = vecs[i + 1];
        top->eval();
        if ((uint64_t)top->mul_out != (uint64_t)got ||
            (top->mul_sat != 0) != want_sat) {
            std::printf("FAIL mulq48(%llu,%llu): rtl %llu sat %d | want %llu sat %d\n",
                        (unsigned long long)vecs[i], (unsigned long long)vecs[i + 1],
                        (unsigned long long)top->mul_out, (int)top->mul_sat,
                        (unsigned long long)got, (int)want_sat);
            if (++fails > 40) break;
        }
    }

    // --- reciprocal accuracy on the real envelope, THROUGH THE RTL ---
    // An earlier version of this block called the C++ recip48 directly, so it
    // never exercised the RTL at all and passed while the DUT returned 0.
    double worst = 0.0;
    for (double d : {0.0910808, 0.09, 0.19, 0.19262, 0.194047, 0.5, 1.0, 3.7}) {
        top->recip_x = (uint64_t)p_from_double(d);
        top->eval();
        const double got = p_to_double((q48)top->recip_out);
        worst = std::fmax(worst, std::fabs(got - 1.0 / d) * d);
    }
    std::printf("RTL recip48 worst relative error on envelope: %.3g\n", worst);
    if (!(worst < 1e-13)) {
        std::printf("FAIL recip48 accuracy\n");
        ++fails;
    }

    if (fails == 0)
        std::printf("done: %zu vectors x {clz64, recip48, mulq48}, fails=0: PASS\n",
                    vecs.size());
    else
        std::printf("done: fails=%zu: FAIL\n", fails);
    delete top;
    return fails == 0 ? 0 : 1;
}