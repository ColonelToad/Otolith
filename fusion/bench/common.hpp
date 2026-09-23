#pragma once
// Shared protocol for the v0.2 transport bake-off (ADR-0005).
//
// Every contender moves the same payload across a process boundary:
// a 288 B message size-locked to the LogRow contract (log.hpp:42), with
// the send stamp (CLOCK_MONOTONIC ns) and sequence number overlaid on the
// first 16 bytes. The consumer records (seq, latency) pairs; stats and
// drop accounting happen offline in run_bench.py so no contender pays
// for analysis in its hot loop.
//
// CLI (all bench mains): --role pub|sub --rate HZ --n N --out PATH [--ready PATH]
//   pub sends N messages paced at HZ, then writes PATH.pubinfo (sent/dropped).
//   sub touches PATH.ready once its transport is up, then records until a
//   2 s stall (or 15 s with zero messages) and writes PATH (.bin pairs).

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

#include "otolith/log.hpp"

namespace bench {

struct Args {
    std::string role;   // "pub" | "sub"
    double rate_hz = 500.0;
    int n = 2500;
    std::string out;    // latency file (sub) / basename for sidecar (pub)
    std::string ready;  // sub touches this when ready (empty = skip)
};

inline bool parse_args(int argc, char** argv, Args& a) {
    for (int i = 1; i + 1 < argc; i += 2) {
        std::string k = argv[i], v = argv[i + 1];
        if (k == "--role") a.role = v;
        else if (k == "--rate") a.rate_hz = std::stod(v);
        else if (k == "--n") a.n = std::stoi(v);
        else if (k == "--out") a.out = v;
        else if (k == "--ready") a.ready = v;
        else { std::fprintf(stderr, "unknown arg %s\n", k.c_str()); return false; }
    }
    if ((a.role != "pub" && a.role != "sub") || a.rate_hz <= 0 || a.n <= 0 || a.out.empty()) {
        std::fprintf(stderr, "usage: %s --role pub|sub --rate HZ --n N --out PATH [--ready PATH]\n", argv[0]);
        return false;
    }
    return true;
}

// 288 B wire message. Size-locked to the log contract; content beyond the
// header is irrelevant to a transport bench, only the byte count matters.
struct BenchMsg {
    uint64_t tx_ns;  // send stamp, CLOCK_MONOTONIC
    uint64_t seq;    // 0..N-1
    uint8_t pad[otolith::kRowBytes - 16];
};
static_assert(sizeof(BenchMsg) == otolith::kRowBytes, "bench payload matches LogRow size");

inline uint64_t now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ull + uint64_t(ts.tv_nsec);
}

// Sleep until an absolute deadline (ns, CLOCK_MONOTONIC). No catch-up:
// if we wake late the slot is simply missed by the schedule.
inline void sleep_until_ns(uint64_t deadline_ns) {
    timespec ts{};
    ts.tv_sec = time_t(deadline_ns / 1000000000ull);
    ts.tv_nsec = long(deadline_ns % 1000000000ull);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

struct Sample {
    uint64_t seq;
    uint64_t lat_ns;
};

inline bool write_samples(const std::string& path, const std::vector<Sample>& v) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    bool ok = v.empty() || std::fwrite(v.data(), sizeof(Sample), v.size(), f) == v.size();
    std::fclose(f);
    return ok;
}

inline bool write_sidecar(const std::string& path, int sent, int dropped) {
    FILE* f = std::fopen((path + ".pubinfo").c_str(), "w");
    if (!f) return false;
    std::fprintf(f, "sent=%d dropped=%d\n", sent, dropped);
    std::fclose(f);
    return true;
}

inline bool touch(const std::string& path) {
    if (path.empty()) return true;
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;
    std::fclose(f);
    return true;
}

#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
inline void spin_pause() { _mm_pause(); }
#else
#include <sched.h>
inline void spin_pause() { sched_yield(); }
#endif

}  // namespace bench
