//! SPSC sequence ring — faithful port of `fusion/bench/ring.hpp`.
//!
//! Disruptor-style per-slot sequences: slot `i` is claimable for message
//! `n` iff `seq == n`, published by storing `n+1` (release), freed by the
//! consumer storing `n+CAP`. Single producer + single consumer, release /
//! acquire pairing, no syscalls steady-state. The producer never blocks —
//! a full ring reports a drop, keeping pacing sacred.
//!
//! Everything here is **safe** Rust and Miri-gated. Cross-process sharing
//! is sound by construction: `seq` is an atomic (alias-safe); payload
//! bytes live in `UnsafeCell` and are touched only via raw copies gated
//! by the handshake (claimed ⇒ producer-only writer; published ⇒
//! consumer reads once, then frees). The cursors are process-local, never
//! shared — exactly like the C++ `next_` members.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU64, Ordering};

use super::{shm::Mapping, BenchMsg, Transport};

pub const CAP: usize = 1024;
pub const SHM_NAME: &str = "/otolith_bench_e";
pub const SEG_BYTES: usize = CAP * std::mem::size_of::<Slot>();

/// One ring slot. `repr(C)` matches `RingSlot` in `ring.hpp`
/// (`{ atomic seq; BenchMsg msg; }`) so a Rust pub could serve a C++ sub
/// and vice versa — same bytes, same protocol.
#[repr(C)]
pub struct Slot {
    pub seq: AtomicU64,
    pub msg: UnsafeCell<BenchMsg>,
}

// Slot is !Sync (UnsafeCell): single-threaded use per process, enforced by
// the type system. No `unsafe impl Sync` — the protocol needs no threads.

/// Process-local ring endpoint. `send` side is used by the pub role,
/// `try_recv` by the sub role; each keeps its own cursor.
pub struct ShmRing {
    map: Mapping,
    next_pub: u64,
    next_sub: u64,
}

impl ShmRing {
    /// Sub side: create the segment so the mapping exists before any pub
    /// sends (the orchestrator starts sub first and gates pub on ready).
    pub fn create() -> std::io::Result<Self> {
        Ok(Self {
            map: Mapping::create(SHM_NAME)?,
            next_pub: 0,
            next_sub: 0,
        })
    }

    /// Pub side: attach to the sub-created segment.
    pub fn open_existing() -> std::io::Result<Self> {
        Ok(Self {
            map: Mapping::open_existing(SHM_NAME)?,
            next_pub: 0,
            next_sub: 0,
        })
    }

    fn slot(&mut self, n: u64) -> &mut Slot {
        // SAFETY (shm I2): unique in-process owner, single-threaded; the
        // returned borrow is scoped to &mut self and never escapes with a
        // peer's borrow alive.
        &mut self.map.slots()[(n % CAP as u64) as usize]
    }
}

impl Transport for ShmRing {
    fn send(&mut self, msg: &BenchMsg) -> std::io::Result<bool> {
        let n = self.next_pub;
        let s = self.slot(n);
        if s.seq.load(Ordering::Acquire) != n {
            return Ok(false); // full: drop, pacing sacred
        }
        // SAFETY: slot claimed (seq == n) — the consumer cannot hold
        // it (it only reads published slots) and no in-process alias exists.
        unsafe { std::ptr::write(s.msg.get(), *msg) };
        s.seq.store(n + 1, Ordering::Release);
        self.next_pub = n + 1;
        Ok(true)
    }

    fn try_recv(&mut self, out: &mut BenchMsg) -> std::io::Result<bool> {
        let n = self.next_sub;
        let s = self.slot(n);
        if s.seq.load(Ordering::Acquire) != n + 1 {
            return Ok(false);
        }
        // SAFETY: slot published — the producer has released it and will
        // not touch it until we free it below. Single copy, slot -> out.
        unsafe { std::ptr::copy_nonoverlapping(s.msg.get(), out as *mut BenchMsg, 1) };
        s.seq.store(n + CAP as u64, Ordering::Release);
        self.next_sub = n + 1;
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vec_slots() -> Vec<Slot> {
        (0..CAP as u64)
            .map(|i| Slot {
                seq: AtomicU64::new(i),
                msg: UnsafeCell::new(BenchMsg::new(0, 0)),
            })
            .collect()
    }

    /// Drive the protocol over process-local (Vec-backed) slots — the exact
    /// state machine Miri checks. No SHM involved.
    fn drive(n: usize) -> (Vec<u64>, usize) {
        let mut slots = vec_slots();
        let mut pub_n = 0u64;
        let mut sub_n = 0u64;
        let mut received = Vec::new();
        let mut drops = 0usize;
        let at = |n: u64| (n % CAP as u64) as usize;
        for i in 0..n as u64 {
            let s = &mut slots[at(pub_n)];
            if s.seq.load(Ordering::Acquire) == pub_n {
                unsafe { std::ptr::write(s.msg.get(), BenchMsg::new(i, i)) };
                s.seq.store(pub_n + 1, Ordering::Release);
                pub_n += 1;
            } else {
                drops += 1;
            }
            loop {
                let s = &mut slots[at(sub_n)];
                if s.seq.load(Ordering::Acquire) != sub_n + 1 {
                    break;
                }
                let m = unsafe { std::ptr::read(s.msg.get()) };
                s.seq.store(sub_n + CAP as u64, Ordering::Release);
                sub_n += 1;
                received.push(m.seq);
            }
        }
        // drain
        loop {
            let s = &mut slots[at(sub_n)];
            if s.seq.load(Ordering::Acquire) != sub_n + 1 {
                break;
            }
            let m = unsafe { std::ptr::read(s.msg.get()) };
            s.seq.store(sub_n + CAP as u64, Ordering::Release);
            sub_n += 1;
            received.push(m.seq);
        }
        (received, drops)
    }

    #[test]
    fn layout_matches_cpp() {
        assert_eq!(std::mem::size_of::<Slot>(), 8 + 288);
        assert_eq!(std::mem::offset_of!(Slot, msg), 8);
        assert_eq!(SEG_BYTES, CAP * 296);
    }

    #[test]
    fn ordered_no_drop_small() {
        let (rx, drops) = drive(500);
        assert_eq!(drops, 0);
        assert_eq!(rx, (0..500).collect::<Vec<_>>());
    }

    #[test]
    fn wraparound_over_capacity() {
        // 3x capacity forces slot reuse across laps.
        let (rx, drops) = drive(3 * CAP);
        assert_eq!(drops, 0);
        assert_eq!(rx, (0..3 * CAP as u64).collect::<Vec<_>>());
    }

    #[test]
    fn slow_consumer_drops_but_never_reorders() {
        // Never drain mid-run: only CAP can be in flight; the rest drops.
        let mut slots = vec_slots();
        let mut pub_n = 0u64;
        let mut drops = 0;
        for i in 0..(2 * CAP) as u64 {
            let s = &mut slots[(pub_n % CAP as u64) as usize];
            if s.seq.load(Ordering::Acquire) == pub_n {
                unsafe { std::ptr::write(s.msg.get(), BenchMsg::new(i, i)) };
                s.seq.store(pub_n + 1, Ordering::Release);
                pub_n += 1;
            } else {
                drops += 1;
            }
        }
        assert!(drops > 0, "a never-drained ring must drop");
        // Drain now: received seqs are a strictly-increasing subsequence of
        // sent seqs (overwritten-while-unread messages are lost, never
        // reordered or duplicated).
        let mut prev = None;
        let mut sub_n = 0u64;
        let mut got = 0u64;
        loop {
            let s = &mut slots[(sub_n % CAP as u64) as usize];
            if s.seq.load(Ordering::Acquire) != sub_n + 1 {
                break;
            }
            let m = unsafe { std::ptr::read(s.msg.get()) };
            assert!(m.seq < pub_n, "ghost message beyond sent range");
            if let Some(p) = prev {
                assert!(m.seq > p, "reorder or dup: {m:?} after {p}");
            }
            prev = Some(m.seq);
            s.seq.store(sub_n + CAP as u64, Ordering::Release);
            sub_n += 1;
            got += 1;
        }
        assert!(got > 0 && got <= pub_n, "got={got} pub_n={pub_n}");
    }
}
