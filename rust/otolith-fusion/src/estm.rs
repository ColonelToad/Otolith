//! ESTM estimate writer (v2) — byte-compat with
//! `fusion/include/otolith/estimate_log.hpp` and `evaluate.py:read_est`.
//!
//! Header 32 B: `ESTM`, version 2, row_bytes 1936, reserved, dt,
//! row_count. Row 1936 B LE: base 136 B (`t, p[3], quat[4] wxyz, v[3],
//! bg[3], ba[3]`) + 225 covariance doubles, row-major 15x15.

use std::fs;
use std::io;

pub const MAGIC: &[u8; 4] = b"ESTM";
pub const VERSION: u32 = 2;
pub const ROW_BYTES: usize = 1936;

pub struct EstRowV2 {
    pub t: f64,
    pub p: [f64; 3],
    pub quat: [f64; 4], // wxyz
    pub v: [f64; 3],
    pub bg: [f64; 3],
    pub ba: [f64; 3],
    pub p_cov_row_major: [f64; 225],
}

fn push_u32(v: &mut Vec<u8>, x: u32) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn push_f64(v: &mut Vec<u8>, x: f64) {
    v.extend_from_slice(&x.to_le_bytes());
}

pub fn write_estimate_v2(path: &str, dt: f64, rows: &[EstRowV2]) -> io::Result<()> {
    let mut out = Vec::with_capacity(32 + rows.len() * ROW_BYTES);
    out.extend_from_slice(MAGIC);
    push_u32(&mut out, VERSION);
    push_u32(&mut out, ROW_BYTES as u32);
    push_u32(&mut out, 0);
    push_f64(&mut out, dt);
    out.extend_from_slice(&(rows.len() as u64).to_le_bytes());
    for r in rows {
        push_f64(&mut out, r.t);
        for &x in &r.p {
            push_f64(&mut out, x);
        }
        for &x in &r.quat {
            push_f64(&mut out, x);
        }
        for &x in &r.v {
            push_f64(&mut out, x);
        }
        for &x in &r.bg {
            push_f64(&mut out, x);
        }
        for &x in &r.ba {
            push_f64(&mut out, x);
        }
        for &x in &r.p_cov_row_major {
            push_f64(&mut out, x);
        }
    }
    debug_assert_eq!(out.len(), 32 + rows.len() * ROW_BYTES);
    fs::write(path, out)
}
