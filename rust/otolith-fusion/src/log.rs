//! OTLG log reader — byte-compat with `sim/otolith_sim/logger.py` and
//! `fusion/include/otolith/log.hpp`.
//!
//! Header 32 B: `OTLG`, version, row_bytes, reserved, dt, row_count.
//! Row 288 B LE: `t, gyro[3], accel[3], qj[12], contacts[4]B,
//! gt_contacts[4]B, gt_pos[3], gt_quat[4] (wxyz), gt_vel[3],
//! gt_rpy_rate[3], gt_accel[3]`.

use std::fs;
use std::io;

pub const MAGIC: &[u8; 4] = b"OTLG";
pub const VERSION: u32 = 1;
pub const ROW_BYTES: usize = 288;
pub const HEADER_SIZE: usize = 32;

#[derive(Clone, Debug)]
pub struct LogRowData {
    pub t: f64,
    pub gyro: [f64; 3],
    pub accel: [f64; 3],
    pub qj: [f64; 12],
    pub contacts: [u8; 4],
    pub gt_contacts: [u8; 4],
    pub gt_pos: [f64; 3],
    pub gt_quat: [f64; 4],
    pub gt_vel: [f64; 3],
    pub gt_rpy_rate: [f64; 3],
    pub gt_accel: [f64; 3],
}

pub struct LogFile {
    pub dt: f64,
    pub rows: Vec<LogRowData>,
}

fn u32le(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
fn f64le(b: &[u8], o: usize) -> f64 {
    f64::from_le_bytes([
        b[o],
        b[o + 1],
        b[o + 2],
        b[o + 3],
        b[o + 4],
        b[o + 5],
        b[o + 6],
        b[o + 7],
    ])
}
fn arr3(b: &[u8], o: &mut usize) -> [f64; 3] {
    let mut a = [0.0; 3];
    for v in a.iter_mut() {
        *v = f64le(b, *o);
        *o += 8;
    }
    a
}
fn arr4d(b: &[u8], o: &mut usize) -> [f64; 4] {
    let mut a = [0.0; 4];
    for v in a.iter_mut() {
        *v = f64le(b, *o);
        *o += 8;
    }
    a
}

pub fn read_log(path: &str) -> io::Result<LogFile> {
    let data = fs::read(path)?;
    let fail = |m: &str| io::Error::new(io::ErrorKind::InvalidData, m);
    if data.len() < HEADER_SIZE {
        return Err(fail("too short"));
    }
    if &data[0..4] != MAGIC {
        return Err(fail("bad otlg magic"));
    }
    if u32le(&data, 4) != VERSION {
        return Err(fail("otlg version"));
    }
    if u32le(&data, 8) as usize != ROW_BYTES {
        return Err(fail("otlg row_bytes"));
    }
    let dt = f64le(&data, 16);
    let payload = &data[HEADER_SIZE..];
    if payload.len() % ROW_BYTES != 0 {
        return Err(fail("truncated otlg row"));
    }
    let mut rows = Vec::with_capacity(payload.len() / ROW_BYTES);
    for chunk in payload.as_chunks::<ROW_BYTES>().0 {
        let mut o = 0;
        let t = f64le(chunk, o);
        o += 8;
        let gyro = arr3(chunk, &mut o);
        let accel = arr3(chunk, &mut o);
        let mut qj = [0.0; 12];
        for v in qj.iter_mut() {
            *v = f64le(chunk, o);
            o += 8;
        }
        let contacts: [u8; 4] = chunk[o..o + 4].try_into().unwrap();
        o += 4;
        let gt_contacts: [u8; 4] = chunk[o..o + 4].try_into().unwrap();
        o += 4;
        let gt_pos = arr3(chunk, &mut o);
        let gt_quat = arr4d(chunk, &mut o);
        let gt_vel = arr3(chunk, &mut o);
        let gt_rpy_rate = arr3(chunk, &mut o);
        let gt_accel = arr3(chunk, &mut o);
        debug_assert_eq!(o, ROW_BYTES);
        rows.push(LogRowData {
            t,
            gyro,
            accel,
            qj,
            contacts,
            gt_contacts,
            gt_pos,
            gt_quat,
            gt_vel,
            gt_rpy_rate,
            gt_accel,
        });
    }
    Ok(LogFile { dt, rows })
}
