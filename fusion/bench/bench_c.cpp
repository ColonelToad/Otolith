// Contender C bench main: typed SPSC ring over POSIX SHM (ADR-0005).
// See common.hpp for CLI + protocol.

#include <vector>

#include "common.hpp"
#include "ring.hpp"

namespace {
// Stall policy shared by all bench subs: exit 2 s after the last message
// (or 15 s with zero messages so a dead transport fails fast).
bool stalled(uint64_t now, uint64_t first_rx, uint64_t last_rx, int received) {
    if (received == 0) return now - first_rx > 15ull * 1000000000ull;
    return now - last_rx > 2ull * 1000000000ull;
}
}  // namespace

int main(int argc, char** argv) {
    bench::Args a;
    if (!bench::parse_args(argc, argv, a)) return 2;
    const uint64_t period_ns = uint64_t(1000000000.0 / a.rate_hz);

    if (a.role == "pub") {
        // Sub creates the segment (and signals ready after mapping); the
        // pub only opens it, so no message is sent before the peer exists.
        bench::RingSeg* seg = bench::ShmRing::open_existing();
        if (!seg) return 1;
        bench::RingProducer p(seg);
        const uint64_t t0 = bench::now_ns();
        int sent = 0, dropped = 0;
        for (int i = 0; i < a.n; ++i) {
            bench::BenchMsg m{};
            m.tx_ns = bench::now_ns();
            m.seq = uint64_t(i);
            if (p.send(m)) ++sent; else ++dropped;
            bench::sleep_until_ns(t0 + uint64_t(i + 1) * period_ns);
        }
        bench::ShmRing::unmap(seg);  // no unlink: peer may still be reading
        if (!bench::write_sidecar(a.out, sent, dropped)) return 1;
        return 0;
    }

    // Sub creates the segment so it exists before the pub sends anything.
    bench::RingSeg* seg = bench::ShmRing::create();
    if (!seg) return 1;
    if (!bench::touch(a.ready)) return 1;
    bench::RingConsumer c(seg);
    std::vector<bench::Sample> v;
    v.reserve(std::size_t(a.n));
    const uint64_t t_start = bench::now_ns();
    uint64_t last_rx = t_start;
    while (true) {
        bench::BenchMsg m{};
        if (c.try_recv(m)) {
            const uint64_t rx = bench::now_ns();
            v.push_back({m.seq, rx - m.tx_ns});
            last_rx = rx;
        } else {
            const uint64_t now = bench::now_ns();
            if (stalled(now, t_start, last_rx, int(v.size()))) break;
            bench::spin_pause();
        }
    }
    bench::ShmRing::unmap(seg);
    if (!bench::write_samples(a.out, v)) return 1;
    return 0;
}
