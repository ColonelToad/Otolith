// Bit-parity testbench: Verilated mul48_kernel vs the C++ fixed-point
// model (ADR-0007 M4). Drives edge + random (a,b) vectors through the
// 1-cycle pipeline; compares {p, sat} bits against
// narrow48(acc128(a) * acc128(b)) from fusion/fixed/fixed.hpp.
// Any mismatch prints the first divergence and fails.

#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "Vmul48_kernel.h"
#include "fixed/fixed.hpp"

using otolith::fixed::acc128;
using otolith::fixed::narrow48;
using otolith::fixed::q24;
using otolith::fixed::q48;
using otolith::fixed::sat_count;
using otolith::fixed::sat_reset;

static void tick(Vmul48_kernel* top) {
    top->clk = 0;
    top->eval();
    top->clk = 1;
    top->eval();
}

int main() {
    Vmul48_kernel* top = new Vmul48_kernel;

    // Reset: 3 dead cycles.
    top->clk = 0;
    top->rst_n = 0;
    top->valid = 0;
    top->a = 0;
    top->b = 0;
    top->eval();
    for (int i = 0; i < 3; ++i) tick(top);
    top->rst_n = 1;

    // Vector set: saturation/rounding edges + small-value cross product
    // (dense round-boundary coverage) + seeded randoms.
    std::vector<std::pair<int32_t, int64_t>> vecs;
    const int32_t A[] = {0, 1, -1, 2, -2, 3, -3, 4096, -4096,
                         8388608, -8388608, 8388609, 16777216,
                         INT32_MAX, INT32_MIN};
    const int64_t B[] = {0, 1, -1, 2, -2, 3, -3, 16777216LL, -16777216LL,
                         16777215LL, 16777217LL, 140737488355328LL,
                         -140737488355328LL, INT64_MAX, INT64_MIN};
    for (int32_t a : A)
        for (int64_t b : B) vecs.emplace_back(a, b);
    // Interior edges that must NOT saturate but exercise full width.
    vecs.emplace_back(16777216, 281474976710656LL);   // 1.0 * 1.0
    vecs.emplace_back(-16777216, 281474976710656LL);  // -1.0 * 1.0
    vecs.emplace_back(16777216, -281474976710656LL);  // 1.0 * -1.0
    std::mt19937_64 rng(0xC10Cu);
    for (int i = 0; i < 2000; ++i)
        vecs.emplace_back(static_cast<int32_t>(rng()),
                          static_cast<int64_t>(rng()));

    // Reference results (model side, bit patterns).
    std::vector<std::pair<uint64_t, bool>> ref;
    ref.reserve(vecs.size());
    for (auto [a, b] : vecs) {
        sat_reset();
        q48 r = narrow48(acc128(static_cast<int64_t>(a)) *
                         acc128(static_cast<int64_t>(b)));
        ref.emplace_back(static_cast<uint64_t>(r), sat_count() > 0);
    }

    // Drive pipeline: output at tick k+1 holds vector k.
    size_t fails = 0;
    top->valid = 1;
    for (size_t i = 0; i < vecs.size(); ++i) {
        top->a = static_cast<uint32_t>(vecs[i].first);
        top->b = static_cast<uint64_t>(vecs[i].second);
        tick(top);
        if (i == 0) continue; // pipeline priming; check ready below
        size_t k = i - 1;
        uint64_t got_p = top->p;
        bool got_sat = top->sat != 0;
        if (!top->ready) {
            std::printf("FAIL vec %zu: ready low\n", k);
            if (++fails > 5) break;
        }
        if (got_p != ref[k].first || got_sat != ref[k].second) {
            std::printf("FAIL vec %zu: a=%d b=%lld\n  dut p=%llu sat=%d\n  ref p=%llu sat=%d\n",
                        k, vecs[k].first, (long long)vecs[k].second,
                        (unsigned long long)got_p, (int)got_sat,
                        (unsigned long long)ref[k].first,
                        (int)ref[k].second);
            if (++fails > 5) break;
        }
    }
    // Drain last vector.
    top->a = 0;
    top->b = 0;
    tick(top);
    {
        size_t k = vecs.size() - 1;
        if (top->p != ref[k].first || (top->sat != 0) != ref[k].second) {
            std::printf("FAIL drain vec %zu\n", k);
            ++fails;
        }
    }

    if (fails == 0)
        std::printf("done: %zu vectors, fails=0: PASS\n", vecs.size());
    else
        std::printf("done: %zu vectors, fails=%zu: FAIL\n", vecs.size(),
                    fails);
    delete top;
    return fails == 0 ? 0 : 1;
}
