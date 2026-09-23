#pragma once
// Contender D: POSIX SHM baseline — single-slot mailbox + semaphores (ADR-0005).
//
// Deliberately minimal: one 288 B slot in a shm_open segment, handoff via
// two named semaphores (empty/full). The consumer blocks in sem_wait
// (kernel futex path); the producer trywaits so pacing stays sacred —
// contention counts a drop. This measures the syscall-handoff cost that
// contender C's spinning ring avoids. Copies per message: 1, same as C.

#include <cstdint>
#include <cstdio>
#include <string>
#include <cerrno>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "common.hpp"

namespace bench {

struct MboxSeg {
    BenchMsg buf{};
};

class ShmMailbox {
public:
    static constexpr const char* kShmName = "/otolith_bench_d";
    static constexpr const char* kSemEmpty = "/otolith_bdt_e";
    static constexpr const char* kSemFull = "/otolith_bdt_f";
    static constexpr std::size_t kSize = sizeof(MboxSeg);

    static void cleanup() {
        shm_unlink(kShmName);
        sem_unlink(kSemEmpty);
        sem_unlink(kSemFull);
    }

    // Both sides call open(): O_CREAT is idempotent, initvals apply at
    // creation only. Producer must call create_size() first is avoided by
    // having the producer explicitly size the segment before the consumer
    // maps it (consumer polls for full size).
    static MboxSeg* create_sized() {
        int fd = shm_open(kShmName, O_CREAT | O_RDWR, 0666);
        if (fd < 0) { std::perror("shm_open create"); return nullptr; }
        if (ftruncate(fd, off_t(kSize)) != 0) { std::perror("ftruncate"); ::close(fd); return nullptr; }
        MboxSeg* seg = map(fd);
        ::close(fd);
        return seg;
    }

    static MboxSeg* open_existing() {
        for (int tries = 0; tries < 1000; ++tries) {  // ~10 s
            int fd = shm_open(kShmName, O_RDWR, 0666);
            if (fd >= 0) {
                struct stat st{};
                if (fstat(fd, &st) == 0 && st.st_size == off_t(kSize)) {
                    MboxSeg* seg = map(fd);
                    ::close(fd);
                    return seg;
                }
                ::close(fd);
            }
            usleep(10000);
        }
        std::fprintf(stderr, "timeout waiting for %s\n", kShmName);
        return nullptr;
    }

    // O_CREAT without O_EXCL: exactly one side creates, the other's
    // initvals are ignored. empty starts 1 (slot free), full starts 0.
    static sem_t* open_empty() {
        sem_t* s = sem_open(kSemEmpty, O_CREAT, 0666, 1);
        if (s == SEM_FAILED) std::perror("sem_open empty");
        return s == SEM_FAILED ? nullptr : s;
    }
    static sem_t* open_full() {
        sem_t* s = sem_open(kSemFull, O_CREAT, 0666, 0);
        if (s == SEM_FAILED) std::perror("sem_open full");
        return s == SEM_FAILED ? nullptr : s;
    }

    static void unmap(MboxSeg* seg) { munmap(seg, kSize); }

private:
    static MboxSeg* map(int fd) {
        void* p = mmap(nullptr, kSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (p == MAP_FAILED) { std::perror("mmap"); return nullptr; }
        return static_cast<MboxSeg*>(p);
    }
};

class MboxProducer {
public:
    MboxProducer(MboxSeg* seg, sem_t* empty, sem_t* full)
        : seg_(seg), empty_(empty), full_(full) {}
    // trywait: never blocks; contention counts a drop.
    bool send(const BenchMsg& m) {
        if (sem_trywait(empty_) != 0) return false;
        seg_->buf = m;
        // Release ordering vs the mutex-free read: sem_post is the barrier.
        sem_post(full_);
        return true;
    }
private:
    MboxSeg* seg_;
    sem_t* empty_;
    sem_t* full_;
};

class MboxConsumer {
public:
    MboxConsumer(MboxSeg* seg, sem_t* empty, sem_t* full)
        : seg_(seg), empty_(empty), full_(full) {}
    bool try_recv(BenchMsg& out) {
        if (sem_trywait(full_) != 0) return false;
        out = seg_->buf;
        sem_post(empty_);
        return true;
    }
    BenchMsg recv() {
        sem_wait(full_);
        BenchMsg m = seg_->buf;
        sem_post(empty_);
        return m;
    }
private:
    MboxSeg* seg_;
    sem_t* empty_;
    sem_t* full_;
};

}  // namespace bench
