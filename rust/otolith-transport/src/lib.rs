//! Transport trait + backends for the v0.3 Rust port (ADR-0006).
//!
//! The wire message is [`BenchMsg`]: 288 B size-locked to the LogRow
//! contract (`fusion/include/otolith/log.hpp:42`), tx stamp + seq in the
//! first 16 bytes — the same bytes as `fusion/bench/common.hpp`, both
//! languages. Backends: hand-rolled SHM ring (`ring`, M1) and iceoryx2
//! (`iox2`, M2).

/// 288 B wire message. Matches `BenchMsg` in `fusion/bench/common.hpp`.
#[repr(C)]
pub struct BenchMsg {
    pub tx_ns: u64,
    pub seq: u64,
    pub pad: [u8; 272],
}

impl BenchMsg {
    pub const BYTES: usize = 288;

    pub fn new(tx_ns: u64, seq: u64) -> Self {
        Self {
            tx_ns,
            seq,
            pad: [0; 272],
        }
    }

    pub fn as_bytes(&self) -> &[u8; Self::BYTES] {
        // SAFETY: repr(C), all-integer fields, no padding (8+8+272 = 288).
        unsafe { &*(self as *const Self as *const [u8; Self::BYTES]) }
    }

    pub fn from_bytes(b: &[u8; Self::BYTES]) -> Self {
        let mut tx = [0u8; 8];
        let mut seq = [0u8; 8];
        tx.copy_from_slice(&b[0..8]);
        seq.copy_from_slice(&b[8..16]);
        Self {
            tx_ns: u64::from_le_bytes(tx),
            seq: u64::from_le_bytes(seq),
            pad: [0; 272],
        }
    }
}

/// Transport endpoint contract. `send` never blocks (a full ring counts a
/// drop, keeping pacing sacred); `try_recv` is non-blocking — subscribers
/// spin, same discipline as the C++ bench.
pub trait Transport {
    /// Queue one message. `Ok(true)` = accepted, `Ok(false)` = dropped full.
    fn send(&mut self, msg: &BenchMsg) -> std::io::Result<bool>;
    /// Dequeue one message if present.
    fn try_recv(&mut self) -> std::io::Result<Option<BenchMsg>>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bench_msg_is_288_bytes() {
        assert_eq!(std::mem::size_of::<BenchMsg>(), 288);
    }

    #[test]
    fn bench_msg_roundtrip() {
        let m = BenchMsg::new(123456789, 42);
        let r = BenchMsg::from_bytes(m.as_bytes());
        assert_eq!(r.tx_ns, 123456789);
        assert_eq!(r.seq, 42);
    }
}
