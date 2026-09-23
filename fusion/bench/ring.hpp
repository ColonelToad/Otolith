#pragma once
// Contender C: typed-contract SPSC ring in POSIX shared memory (ADR-0005).
//
// Disruptor-style per-slot sequences, single producer + single consumer,
// release/acquire pairing, no syscalls in steady state. The consumer spins
// (pause); the producer never blocks — a full ring counts a drop and moves
// on, keeping the pacing schedule sacred. Payload is BenchMsg (288 B).
//
// Layout in shm segment: CAP slots of { atomic seq, BenchMsg }.
// Slot i is claimable for message n iff seq == n; published by storing
// seq = n+1 (release); freed by the consumer storing seq = n+CAP.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "common.hpp"

namespace bench {

inline constexpr std::size_t kRingCap = 1024;

struct RingSlot {
    std::atomic<uint64_t> seq{0};
    BenchMsg msg{};
};

struct RingSeg {
    RingSlot slots[kRingCap];
};

class ShmRing {
public:
    static constexpr const char* kName = "/otolith_bench_c";
    static constexpr std::size_t kSize = sizeof(RingSeg);

    // Producer side: unlink stale, create, size, map, init sequences.
    static RingSeg* create() {
        shm_unlink(kName);
        int fd = shm_open(kName, O_CREAT | O_RDWR, 0666);
        if (fd < 0) { std::perror("shm_open create"); return nullptr; }
        if (ftruncate(fd, off_t(kSize)) != 0) { std::perror("ftruncate"); ::close(fd); return nullptr; }
        RingSeg* seg = map(fd);
        ::close(fd);
        if (!seg) return nullptr;
        for (std::size_t i = 0; i < kRingCap; ++i)
            seg->slots[i].seq.store(uint64_t(i), std::memory_order_relaxed);
        if (!seg->slots[0].seq.is_lock_free()) {
            std::fprintf(stderr, "slot seq not lock-free\n");
            munmap(seg, kSize);
            return nullptr;
        }
        return seg;
    }

    // Consumer side: wait for a fully-sized segment, then map.
    static RingSeg* open_existing() {
        for (int tries = 0; tries < 1000; ++tries) {  // ~10 s
            int fd = shm_open(kName, O_RDWR, 0666);
            if (fd >= 0) {
                struct stat st{};
                if (fstat(fd, &st) == 0 && st.st_size == off_t(kSize)) {
                    RingSeg* seg = map(fd);
                    ::close(fd);
                    return seg;
                }
                ::close(fd);
            }
            usleep(10000);
        }
        std::fprintf(stderr, "timeout waiting for %s\n", kName);
        return nullptr;
    }

    static void unmap(RingSeg* seg) { munmap(seg, kSize); }
    static void unlink() { shm_unlink(kName); }

private:
    static RingSeg* map(int fd) {
        void* p = mmap(nullptr, kSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (p == MAP_FAILED) { std::perror("mmap"); return nullptr; }
        return static_cast<RingSeg*>(p);
    }
};

class RingProducer {
public:
    explicit RingProducer(RingSeg* seg) : seg_(seg) {}
    // Returns false (drop) if the consumer has not freed the slot.
    bool send(const BenchMsg& m) {
        RingSlot& s = seg_->slots[next_ % kRingCap];
        if (s.seq.load(std::memory_order_acquire) != next_) return false;
        s.msg = m;
        s.seq.store(next_ + 1, std::memory_order_release);
        ++next_;
        return true;
    }
private:
    RingSeg* seg_;
    uint64_t next_ = 0;
};

class RingConsumer {
public:
    explicit RingConsumer(RingSeg* seg) : seg_(seg) {}
    bool try_recv(BenchMsg& out) {
        RingSlot& s = seg_->slots[next_ % kRingCap];
        if (s.seq.load(std::memory_order_acquire) != next_ + 1) return false;
        out = s.msg;
        s.seq.store(next_ + kRingCap, std::memory_order_release);
        ++next_;
        return true;
    }
    // Spins until the next message is published.
    BenchMsg recv() {
        BenchMsg m{};
        while (!try_recv(m)) spin_pause();
        return m;
    }
private:
    RingSeg* seg_;
    uint64_t next_ = 0;
};

}  // namespace bench
