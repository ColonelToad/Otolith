//! Contender E bench main: Rust SHM ring (ADR-0006, M1).
//!
//! Same CLI + protocol as the C++ mains (`fusion/bench/common.hpp`):
//! `--role pub|sub --rate HZ --n N --out PATH [--ready PATH]`; paced
//! `CLOCK_MONOTONIC` sends; sub writes (seq, latency) u64-LE pairs and
//! exits on 2 s stall (15 s grace with zero messages); pub writes
//! `<out>.pubinfo` with sent/dropped counts.

use std::fs;
use std::hint;
use std::process::ExitCode;
use std::time::Duration;

use otolith_transport::{ring::ShmRing, BenchMsg, Transport};

const STALL: Duration = Duration::from_secs(2);
const GRACE: Duration = Duration::from_secs(15);

fn now_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: valid zeroed timespec, CLOCK_MONOTONIC always succeeds.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

fn sleep_until(deadline_ns: u64) {
    let ts = libc::timespec {
        tv_sec: (deadline_ns / 1_000_000_000) as libc::time_t,
        tv_nsec: (deadline_ns % 1_000_000_000) as libc::c_long,
    };
    // SAFETY: valid timespec, null remaining, absolute monotonic clock.
    unsafe {
        libc::clock_nanosleep(
            libc::CLOCK_MONOTONIC,
            libc::TIMER_ABSTIME,
            &ts,
            std::ptr::null_mut(),
        )
    };
}

struct Args {
    role: String,
    rate_hz: f64,
    n: u64,
    out: String,
    ready: String,
}

fn parse_args() -> Option<Args> {
    let mut a = Args {
        role: String::new(),
        rate_hz: 500.0,
        n: 2500,
        out: String::new(),
        ready: String::new(),
    };
    let raw: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i + 1 < raw.len() {
        match raw[i].as_str() {
            "--role" => a.role = raw[i + 1].clone(),
            "--rate" => a.rate_hz = raw[i + 1].parse().ok()?,
            "--n" => a.n = raw[i + 1].parse().ok()?,
            "--out" => a.out = raw[i + 1].clone(),
            "--ready" => a.ready = raw[i + 1].clone(),
            _ => return None,
        }
        i += 2;
    }
    if (a.role != "pub" && a.role != "sub") || a.rate_hz <= 0.0 || a.n == 0 || a.out.is_empty() {
        return None;
    }
    Some(a)
}

fn run_pub(a: &Args) -> ExitCode {
    let mut ring = match ShmRing::open_existing() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("open_existing: {e}");
            return ExitCode::FAILURE;
        }
    };
    let period_ns = (1_000_000_000.0 / a.rate_hz) as u64;
    let t0 = now_ns();
    let (mut sent, mut dropped) = (0u64, 0u64);
    for i in 0..a.n {
        let m = BenchMsg::new(now_ns(), i);
        match ring.send(&m) {
            Ok(true) => sent += 1,
            Ok(false) => dropped += 1,
            Err(e) => {
                eprintln!("send: {e}");
                return ExitCode::FAILURE;
            }
        }
        sleep_until(t0 + (i + 1) * period_ns);
    }
    if fs::write(
        format!("{}.pubinfo", a.out),
        format!("sent={sent} dropped={dropped}\n"),
    )
    .is_err()
    {
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

fn run_sub(a: &Args) -> ExitCode {
    let mut ring = match ShmRing::create() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("create: {e}");
            return ExitCode::FAILURE;
        }
    };
    if !a.ready.is_empty() && fs::write(&a.ready, b"").is_err() {
        return ExitCode::FAILURE;
    }
    let mut samples: Vec<u8> = Vec::with_capacity((a.n as usize) * 16);
    let t_start = now_ns();
    let mut last_rx = t_start;
    let mut received = 0u64;
    let mut m = BenchMsg::new(0, 0);
    loop {
        match ring.try_recv(&mut m) {
            Ok(true) => {
                let rx = now_ns();
                samples.extend_from_slice(&m.seq.to_le_bytes());
                samples.extend_from_slice(&(rx - m.tx_ns).to_le_bytes());
                last_rx = rx;
                received += 1;
            }
            Ok(false) => {
                let now = now_ns();
                let stalled = if received == 0 {
                    now - t_start > GRACE.as_nanos() as u64
                } else {
                    now - last_rx > STALL.as_nanos() as u64
                };
                if stalled {
                    break;
                }
                hint::spin_loop();
            }
            Err(e) => {
                eprintln!("try_recv: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    if fs::write(&a.out, samples).is_err() {
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

fn main() -> ExitCode {
    let Some(a) = parse_args() else {
        eprintln!("usage: bench_e --role pub|sub --rate HZ --n N --out PATH [--ready PATH]");
        return ExitCode::from(2);
    };
    if a.role == "pub" {
        run_pub(&a)
    } else {
        run_sub(&a)
    }
}
