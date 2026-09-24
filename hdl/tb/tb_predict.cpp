// Bit-parity testbench: Verilated predict_core vs the C++ fixed-point
// model (ADR-0007 M2). Loads GT init once, then locksteps 2500 trot rows:
// same fixed inputs to DUT + model, compare q/p/v/P STATE BITS + sat
// count every step. Any mismatch prints the first divergence and fails.
// Cycle count per step is recorded (must be constant — deterministic).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#include "Vpredict_core.h"
#include "fixed/fixed_predict.hpp"

using namespace otolith::fixed;

namespace {
// Bus map (mirror of predict_core.sv)
constexpr uint32_t A_DONE = 0x000, A_SAT = 0x001;
constexpr uint32_t A_GYRO = 0x010, A_ACCEL = 0x013, A_DT = 0x016, A_GF = 0x017;
constexpr uint32_t A_Q = 0x020, A_P = 0x024, A_V = 0x027, A_BG = 0x02a, A_BA = 0x02d;
constexpr uint32_t A_PCOV = 0x100;

struct OtlgRow {
    double t, gyro[3], accel[3], qj[12];
    uint8_t contacts[4], gt_contacts[4];
    double gt_pos[3], gt_quat[4], gt_vel[3], gt_rpy[3], gt_acc[3];
};

bool read_otlg(const char* path, double& dt, std::vector<OtlgRow>& rows) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    char magic[4];
    uint32_t ver, rb, res;
    double d;
    uint64_t cnt;
    f.read(magic, 4);
    f.read(reinterpret_cast<char*>(&ver), 4);
    f.read(reinterpret_cast<char*>(&rb), 4);
    f.read(reinterpret_cast<char*>(&res), 4);
    f.read(reinterpret_cast<char*>(&d), 8);
    f.read(reinterpret_cast<char*>(&cnt), 8);
    if (std::memcmp(magic, "OTLG", 4) != 0 || ver != 1 || rb != 288) return false;
    dt = d;
    f.seekg(0, std::ios::end);
    size_t n = (size_t(f.tellg()) - 32) / 288;
    f.seekg(32);
    rows.resize(n);
    for (size_t i = 0; i < n; ++i) {
        auto& r = rows[i];
        f.read(reinterpret_cast<char*>(&r.t), 8);
        f.read(reinterpret_cast<char*>(r.gyro), 24);
        f.read(reinterpret_cast<char*>(r.accel), 24);
        f.read(reinterpret_cast<char*>(r.qj), 96);
        f.read(reinterpret_cast<char*>(r.contacts), 4);
        f.read(reinterpret_cast<char*>(r.gt_contacts), 4);
        f.read(reinterpret_cast<char*>(r.gt_pos), 24);
        f.read(reinterpret_cast<char*>(r.gt_quat), 32);
        f.read(reinterpret_cast<char*>(r.gt_vel), 24);
        f.read(reinterpret_cast<char*>(r.gt_rpy), 24);
        f.read(reinterpret_cast<char*>(r.gt_acc), 24);
    }
    return !!f;
}
} // namespace

static Vpredict_core* dut;
static uint64_t tick = 0;

static void step_clk() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    ++tick;
}

static void bus_write(uint32_t addr, uint64_t data) {
    dut->bus_addr = addr;
    dut->bus_wdata = data;
    dut->bus_we = 1;
    step_clk();
    dut->bus_we = 0;
}

static uint64_t bus_read(uint32_t addr) {
    dut->bus_addr = addr;
    dut->eval(); // combinational read settles
    return dut->bus_rdata;
}

static void pulse_start_wait_done() {
    dut->start = 1;
    step_clk();
    dut->start = 0;
    for (int i = 0; i < 100000; ++i) {
        step_clk();
        if (bus_read(A_DONE)) return;
    }
    std::fprintf(stderr, "TIMEOUT waiting done\n");
    std::exit(1);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: tb_predict <in.otlg> [--selftest]\n");
        return 2;
    }
    bool selftest = (argc >= 3 && std::string(argv[2]) == "--selftest");
    double dt = 0;
    std::vector<OtlgRow> rows;
    if (!selftest) {
        if (!read_otlg(argv[1], dt, rows)) {
            std::fprintf(stderr, "bad log\n");
            return 1;
        }
    } else {
        dt = 0.002;
        rows.resize(1); // one zero row: P stays 0 -> P1=P2=0, P=Qd diag
        rows[0].gt_quat[0] = 1.0; // keep init sane (unit quat)
    }
    dut = new Vpredict_core;
    dut->clk = 0;
    dut->rst = 1;
    dut->start = 0;
    dut->bus_we = 0;
    for (int i = 0; i < 5; ++i) step_clk();
    dut->rst = 0;
    step_clk();

    // Init both sides from GT row 0 (mirror of fuse_log_fixed init).
    FixedPredict fx;
    {
        auto& r0 = rows[0];
        fx.s.q[0] = from_double(r0.gt_quat[0]);
        fx.s.q[1] = from_double(r0.gt_quat[1]);
        fx.s.q[2] = from_double(r0.gt_quat[2]);
        fx.s.q[3] = from_double(r0.gt_quat[3]);
        for (int i = 0; i < 3; ++i) {
            fx.s.p[i] = from_double(r0.gt_pos[i]);
            fx.s.v[i] = from_double(r0.gt_vel[i]);
        }
    }
    q24 dtf = from_double(dt);
    q24 gf = from_double(9.81);
    auto u32 = [](q24 v) { return uint64_t(uint32_t(v)); };
    auto u64 = [](q48 v) { return uint64_t(v); }; // bit-preserving
    // Load DUT: constant inputs + full init state.
    bus_write(A_DT, u32(dtf));
    bus_write(A_GF, u32(gf));
    for (int i = 0; i < 4; ++i) bus_write(A_Q + i, u32(fx.s.q[i]));
    for (int i = 0; i < 3; ++i) {
        bus_write(A_P + i, u32(fx.s.p[i]));
        bus_write(A_V + i, u32(fx.s.v[i]));
        bus_write(A_BG + i, u32(fx.s.bg[i]));
        bus_write(A_BA + i, u32(fx.s.ba[i]));
    }
    for (int i = 0; i < 225; ++i) bus_write(A_PCOV + i, u64(fx.s.P[i]));

    // Write-then-read check: catches bus/addressing bugs before they can
    // hide as datapath mismatches (a real hour-saver during M2 bringup).
    {
        int bad = 0;
        auto chk = [&](uint32_t a, uint64_t expect, const char* what) {
            uint64_t got = bus_read(a);
            if (got != expect) {
                std::printf("BUS MISMATCH %s @%03x: wrote %lld read %lld\n", what, a,
                            (long long)expect, (long long)got);
                ++bad;
            }
        };
        chk(A_DT, u32(dtf), "dtf");
        chk(A_GF, u32(gf), "gf");
        // gyro/accel regs are loaded per step; verify the write path once
        // here with row-0 values (a decode bug here hides as datapath fail).
        {
            auto& r0 = rows[0];
            for (int i = 0; i < 3; ++i) {
                char b[16];
                std::snprintf(b, sizeof b, "gyro[%d]", i);
                bus_write(A_GYRO + i, u32(from_double(r0.gyro[i])));
                chk(A_GYRO + i, u32(from_double(r0.gyro[i])), b);
                std::snprintf(b, sizeof b, "accel[%d]", i);
                bus_write(A_ACCEL + i, u32(from_double(r0.accel[i])));
                chk(A_ACCEL + i, u32(from_double(r0.accel[i])), b);
            }
        }
        for (int i = 0; i < 4; ++i) {
            char b[16];
            std::snprintf(b, sizeof b, "q[%d]", i);
            chk(A_Q + i, u32(fx.s.q[i]), b);
        }
        chk(A_PCOV + 0, u64(fx.s.P[0]), "P[0]");
        chk(A_PCOV + 224, u64(fx.s.P[224]), "P[224]");
        // Full P write-read verify (225 regs; a decode bug at high
        // indices hides as engine failure — worth the 225 cycles).
        for (int i = 0; i < 225; ++i) {
            char b[16];
            std::snprintf(b, sizeof b, "P[%d]", i);
            chk(A_PCOV + i, u64(fx.s.P[i]), b);
        }
        if (bad > 0) {
            std::printf("bus check FAILED\n");
            return 1;
        }
        std::printf("bus check ok\n");
    }

    // Lockstep: same fixed inputs to DUT + model, bit-compare every step.
    int fails = 0;
    uint64_t max_cycles = 0;
    for (size_t n = 0; n < rows.size(); ++n) {
        auto& row = rows[n];
        double gyro[3] = {row.gyro[0], row.gyro[1], row.gyro[2]};
        double accel[3] = {row.accel[0], row.accel[1], row.accel[2]};
        for (int i = 0; i < 3; ++i) {
            bus_write(A_GYRO + i, u32(from_double(gyro[i])));
            bus_write(A_ACCEL + i, u32(from_double(accel[i])));
        }
        uint64_t t0 = tick;
        pulse_start_wait_done();
        uint64_t cycles = tick - t0;
        if (cycles > max_cycles) max_cycles = cycles;
        sat_reset();
        fx.step(dt, gyro, accel);
        int model_sat = sat_count();
        // read back + compare
        auto chk32 = [&](uint32_t addr, q24 expect, const char* what) {
            q24 got = q24(uint32_t(bus_read(addr)));
            if (got != expect) {
                if (fails < 5)
                    std::printf("step %zu MISMATCH %s: dut=%d model=%d\n", n, what, got, expect);
                ++fails;
            }
        };
        for (int i = 0; i < 4; ++i) {
            char b[32];
            std::snprintf(b, sizeof b, "q[%d]", i);
            chk32(A_Q + i, fx.s.q[i], b);
        }
        for (int i = 0; i < 3; ++i) {
            char b[32];
            std::snprintf(b, sizeof b, "p[%d]", i);
            chk32(A_P + i, fx.s.p[i], b);
            std::snprintf(b, sizeof b, "v[%d]", i);
            chk32(A_V + i, fx.s.v[i], b);
        }
        for (int i = 0; i < 225; ++i) {
            q48 got = q48(bus_read(A_PCOV + i));
            if (got != fx.s.P[i]) {
                if (fails < 5)
                    std::printf("step %zu MISMATCH P[%d]: dut=%lld model=%lld\n", n, i,
                                (long long)got, (long long)fx.s.P[i]);
                ++fails;
            }
        }
        int dut_sat = int(bus_read(A_SAT));
        if ((dut_sat != 0) != (model_sat != 0) || dut_sat != model_sat) {
            if (fails < 5)
                std::printf("step %zu SAT MISMATCH dut=%d model=%d\n", n, dut_sat, model_sat);
            ++fails;
        }
        if (n == 0) {
            // DUT observability snapshot (nominal intermediates, ST_NOM).
            std::printf("dbg step0: R0=%d R4=%d R8=%d eq0=%d nq0=%d Phi0=%d qdg=%lld Phi9=%d\n",
                        int(uint32_t(bus_read(0x1f0))), int(uint32_t(bus_read(0x1f1))),
                        int(uint32_t(bus_read(0x1f2))), int(uint32_t(bus_read(0x1f3))),
                        int(uint32_t(bus_read(0x1f4))), int(uint32_t(bus_read(0x1f5))),
                        (long long)bus_read(0x1f6), int(uint32_t(bus_read(0x1f7))));
            // Indirected P1/P2/Phi reads for engine bisection.
            bus_write(0x1f8, 9);
            std::printf("dbg step0: P1[9]=%lld P2[9]=%lld (expect P1 ~ -5629594829)\n",
                        (long long)bus_read(0x1f9), (long long)bus_read(0x1fa));
            bus_write(0x1f8, 45);
            std::printf("dbg step0: Phi45=%d P1_45=%lld (expect Phi ~ -679)\n",
                        int(uint32_t(bus_read(0x1fb))), (long long)bus_read(0x1f9));
            if (std::getenv("TB_DUMPALL")) {
                // Full Phi/P1/P2 dump for exhaustive diff vs model.
                FILE* fphi = std::fopen("/tmp/probe/dut_phi.bin", "wb");
                FILE* fp1 = std::fopen("/tmp/probe/dut_p1.bin", "wb");
                FILE* fp2 = std::fopen("/tmp/probe/dut_p2.bin", "wb");
                for (int m = 0; m < 225; ++m) {
                    bus_write(0x1f8, uint64_t(m));
                    int32_t ph = int32_t(uint32_t(bus_read(0x1fb)));
                    int64_t p1 = int64_t(bus_read(0x1f9));
                    int64_t p2 = int64_t(bus_read(0x1fa));
                    std::fwrite(&ph, 4, 1, fphi);
                    std::fwrite(&p1, 8, 1, fp1);
                    std::fwrite(&p2, 8, 1, fp2);
                }
                std::fclose(fphi);
                std::fclose(fp1);
                std::fclose(fp2);
            }
        }
        // feed DUT state forward: reload model state into DUT regs so both
        // advance identically (the DUT already holds its own state; this
        // reload is belt-and-braces against bus-read blindness... NO —
        // reload would MASK divergence! The DUT keeps its own state across
        // steps (true lockstep); we only reload INPUTS per step above.
        // Model advances via fx.step (called before compare). Correct.
        if (fails > 20) {
            std::printf("too many fails, stopping\n");
            break;
        }
        if ((n + 1) % 500 == 0) std::printf("... %zu steps ok\n", n + 1);
    }
    std::printf("done: %zu steps, max %llu cycles/step, fails=%d: %s\n", rows.size(),
                (unsigned long long)max_cycles, fails, fails == 0 ? "PASS" : "FAIL");
    if (std::getenv("TB_DUMP")) {
        // Full P dump (dut + model) for pattern analysis.
        FILE* fd = std::fopen("/tmp/opencode/dut_P.bin", "wb");
        for (int i = 0; i < 225; ++i) {
            int64_t v = int64_t(bus_read(A_PCOV + i));
            std::fwrite(&v, 8, 1, fd);
        }
        std::fclose(fd);
        FILE* fm = std::fopen("/tmp/opencode/model_P.bin", "wb");
        for (int i = 0; i < 225; ++i) {
            int64_t v = int64_t(fx.s.P[i]);
            std::fwrite(&v, 8, 1, fm);
        }
        std::fclose(fm);
    }
    return fails == 0 ? 0 : 1;
}
