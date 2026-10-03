// Bit-parity: Verilated ldl_kernel vs the C++ model (ADR-0007 M5-A).
//
// Drives REAL innovation covariances captured from the 10 s trot log — not
// synthetic matrices — so the kernel is verified on the distribution it will
// actually see. The C++ side calls fixed_update.hpp's ldl_factor() and
// ldl_solve() directly; the DUT factorizes and solves the same S and b.
// Compares L, D, x bit-for-bit plus the pd/sat flags.
//
// usage: Vldl_kernel <s_capture.otlg-like text file>
// The capture file is plain text, one matrix per line group, produced by
// fuse_update_study --dump-s. See hdl/tb/Makefile.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "Vldl_kernel.h"
#include "fixed/fixed.hpp"
#include "fixed/fixed_update.hpp"

using namespace otolith::fixed;

static constexpr int W = 6;
static constexpr int WT = W * (W + 1) / 2;
static constexpr int WL = W * (W - 1) / 2;
static constexpr int NW_IN = WT + W;
static constexpr int NW_OUT = WL + W + W;

static int lt_idx(int i, int j) { return i * (i + 1) / 2 + j; }
static int sl_idx(int i, int j) { return i * (i - 1) / 2 + j; }

static Vldl_kernel* top = nullptr;
static long cycles = 0;
static long max_cycles = 0;

static void tick() {
    top->clk = 0;
    top->eval();
    top->clk = 1;
    top->eval();
    ++cycles;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: Vldl_kernel <capture.txt> [max_cases]\n");
        return 2;
    }
    const long max_cases = (argc > 2) ? std::atol(argv[2]) : -1;
    std::FILE* f = std::fopen(argv[1], "r");
    if (!f) { std::perror("open"); return 2; }

    top = new Vldl_kernel;
    top->rst_n = 0; top->start = 0; top->in_valid = 0; top->in_data = 0;
    top->out_ready = 0; top->eval();
    for (int i = 0; i < 4; ++i) tick();
    top->rst_n = 1;

    long ncase = 0, fails = 0;
    std::string tok;
    while (true) {
        // read WT S words + W b words, whitespace separated, 16-hex digits
        std::vector<uint64_t> words;
        bool eof = false;
        for (int i = 0; i < NW_IN; ++i) {
            unsigned long long v;
            if (std::fscanf(f, "%llx", &v) != 1) { eof = true; break; }
            words.push_back((uint64_t)v);
        }
        if (eof) break;
        ++ncase;
        if (max_cases > 0 && ncase > max_cases) break;

        // ---- C++ reference ----
        q48 S[W * W] = {0}, b[W];
        // unpack lower triangle from the flat word list (row-major, i>=j)
        {
            int k = 0;
            for (int i = 0; i < W; ++i)
                for (int j = 0; j <= i; ++j) S[i * W + j] = (q48)words[k++];
        }
        for (int i = 0; i < W; ++i) b[i] = (q48)words[WT + i];

        FixedState st;
        for (int i = 0; i < 225; ++i) st.P[i] = 0;
        LdlFactor fct;
        sat_reset();
        const bool pd = ldl_factor(S, W, fct, DivMode::Q48);
        q48 x[12];
        for (int i = 0; i < W; ++i) x[i] = b[i];
        if (pd) ldl_solve(fct, x);
        const int ref_sat = sat_count();

        // ---- DUT ----
        const long c0 = cycles;
        // start is sampled in S_IDLE and moves the kernel into S_LOAD, which
        // is what consumes in_valid. Asserting start AFTER the words left it
        // parked in S_LOAD waiting for data that had already been sent.
        top->in_valid = 0; top->start = 0; top->out_ready = 0;
        for (int guard = 0; guard < 100000 && !top->in_ready; ++guard) tick();
        if (!top->in_ready) { std::printf("FAIL case %ld: never idle\n", ncase); ++fails; break; }
        top->start = 1;
        tick();
        top->start = 0;
        for (int i = 0; i < NW_IN; ++i) {
            top->in_data = words[i];
            top->in_valid = 1;
            tick();
        }
        top->in_valid = 0;
        // wait for done (bounded: a stuck FSM must report, not hang)
        for (int guard = 0; guard < 50000 && !top->done; ++guard) tick();
        if (!top->done) { std::printf("FAIL case %ld: no done\n", ncase); ++fails; break; }
        { const long d = cycles - c0; if (d > max_cycles) max_cycles = d; }
        // Dump NW_OUT words. out_data is combinational off ld_i, which
        // advances on the out_ready cycle, so the word must be SAMPLED BEFORE
        // pulsing out_ready -- sampling after the tick reads the next word.
        // (The earlier version also looped unbounded.)
        std::vector<uint64_t> out;
        for (int i = 0; i < NW_OUT; ++i) {
            int guard = 0;
            while (!top->out_valid && guard < 1000) { tick(); ++guard; }
            if (!top->out_valid) {
                std::printf("FAIL case %ld: out_valid stuck at word %d\n", ncase, i);
                ++fails; break;
            }
            out.push_back(top->out_data);
            top->out_ready = 1;
            tick();
            top->out_ready = 0;
        }

        // ---- compare ----
        auto bad = [&](const char* what, int i, int j, uint64_t got, int64_t want) {
            std::printf("FAIL case %ld %s[%d][%d]: rtl %llx want %lld\n", ncase, what, i,
                        j, (unsigned long long)got, (long long)want);
        };
        if ((bool)top->pd != pd) { std::printf("FAIL case %ld pd: rtl %d want %d\n",
                                               ncase, (int)top->pd, (int)pd); ++fails; }
        if ((int)top->sat != (ref_sat > 0)) {
            std::printf("FAIL case %ld sat: rtl %d want %d\n", ncase, (int)top->sat,
                        ref_sat > 0); ++fails;
        }
        // NOTE: LdlFactor::L is stored ROW-MAJOR with stride W (i*W + j), not
        // with the kernel's packed strict-lower sl_idx(). Comparing the DUT's
        // packed order against sl_idx on the C++ side read the diagonal and
        // wrong rows, which looked like a wholesale RTL failure.
        int k = 0;
        for (int i = 1; i < W; ++i)
            for (int j = 0; j < i; ++j, ++k)
                if (out[k] != (uint64_t)fct.L[i * W + j]) {
                    bad("L", i, j, out[k], (int64_t)fct.L[i * W + j]); ++fails;
                }
        for (int i = 0; i < W; ++i, ++k)
            if (out[k] != (uint64_t)fct.D[i]) { bad("D", i, i, out[k], (int64_t)fct.D[i]); ++fails; }
        for (int i = 0; i < W; ++i, ++k)
            if (out[k] != (uint64_t)x[i]) { bad("x", i, i, out[k], (int64_t)x[i]); ++fails; }

        if (fails > 8) { std::printf("too many failures, stopping\n"); break; }
    }
    std::fclose(f);

    if (fails == 0)
        std::printf("done: %ld matrices (W=%d), fails=0: PASS "
                    "(max %ld cycles/case, 1 multiply per cycle)\n",
                    ncase, W, max_cycles);
    else
        std::printf("done: %ld matrices, fails=%ld: FAIL\n", ncase, fails);
    delete top;
    return fails == 0 ? 0 : 1;
}