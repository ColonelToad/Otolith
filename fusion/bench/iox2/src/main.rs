// Contender B bench main: iceoryx2 publish-subscribe, [u8; 288] payload
// (ADR-0005, option 1: cargo-fetched iceoryx2 0.10 against the rustup
// toolchain; this crate is NOT pixi-managed).
//
// Same protocol as the C++ contenders (see ../common.hpp): CLI
// --role pub|sub --rate HZ --n N --out PATH [--ready PATH]; tx stamp +
// seq in the first 16 bytes (CLOCK_MONOTONIC ns, LE); sub writes (seq,
// latency) u64 pairs; pub writes <out>.pubinfo. Subscriber spins on
// receive() like contender C spins on its ring; buffer 1024 matches C's
// ring capacity so the comparison isolates handoff mechanism, not depth.

use std::env;
use std::fs;
use std::hint;
use std::process::ExitCode;

use iceoryx2::prelude::*;

const PAYLOAD_BYTES: usize = 288;
const SERVICE: &str = "otolith/bench";
const BUFFER: usize = 1024;
const STALL_NS: u64 = 2_000_000_000;
const GRACE_NS: u64 = 15_000_000_000;

fn now_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

fn sleep_until(deadline_ns: u64) {
    let ts = libc::timespec {
        tv_sec: (deadline_ns / 1_000_000_000) as libc::time_t,
        tv_nsec: (deadline_ns % 1_000_000_000) as libc::c_long,
    };
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
    let raw: Vec<String> = env::args().skip(1).collect();
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

fn usage() -> ExitCode {
    eprintln!("usage: bench_iox2 --role pub|sub --rate HZ --n N --out PATH [--ready PATH]");
    ExitCode::from(2)
}

fn run_pub(a: &Args) -> ExitCode {
    let node = match NodeBuilder::new().create::<ipc::Service>() {
        Ok(n) => n,
        Err(e) => {
            eprintln!("node: {e:?}");
            return ExitCode::FAILURE;
        }
    };
    let service = match node
        .service_builder(&SERVICE.try_into().unwrap())
        .publish_subscribe::<[u8; PAYLOAD_BYTES]>()
        .subscriber_max_buffer_size(BUFFER)
        .open_or_create()
    {
        Ok(s) => s,
        Err(e) => {
            eprintln!("service: {e:?}");
            return ExitCode::FAILURE;
        }
    };
    let publisher = match service.publisher_builder().create() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("publisher: {e:?}");
            return ExitCode::FAILURE;
        }
    };
    let period_ns = (1_000_000_000.0 / a.rate_hz) as u64;
    let t0 = now_ns();
    let (mut sent, mut dropped) = (0u64, 0u64);
    for i in 0..a.n {
        let mut msg = [0u8; PAYLOAD_BYTES];
        msg[0..8].copy_from_slice(&now_ns().to_le_bytes());
        msg[8..16].copy_from_slice(&i.to_le_bytes());
        match publisher.send_copy(msg) {
            Ok(_) => sent += 1,
            Err(_) => dropped += 1,
        }
        sleep_until(t0 + (i + 1) * period_ns);
    }
    if fs::write(format!("{}.pubinfo", a.out), format!("sent={sent} dropped={dropped}\n")).is_err() {
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

fn run_sub(a: &Args) -> ExitCode {
    let node = match NodeBuilder::new().create::<ipc::Service>() {
        Ok(n) => n,
        Err(e) => {
            eprintln!("node: {e:?}");
            return ExitCode::FAILURE;
        }
    };
    let service = match node
        .service_builder(&SERVICE.try_into().unwrap())
        .publish_subscribe::<[u8; PAYLOAD_BYTES]>()
        .subscriber_max_buffer_size(BUFFER)
        .open_or_create()
    {
        Ok(s) => s,
        Err(e) => {
            eprintln!("service: {e:?}");
            return ExitCode::FAILURE;
        }
    };
    let subscriber = match service.subscriber_builder().buffer_size(BUFFER).create() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("subscriber: {e:?}");
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
    loop {
        match subscriber.receive() {
            Ok(Some(sample)) => {
                let rx = now_ns();
                let p = sample.payload();
                let tx = u64::from_le_bytes(p[0..8].try_into().unwrap());
                let seq = u64::from_le_bytes(p[8..16].try_into().unwrap());
                samples.extend_from_slice(&seq.to_le_bytes());
                samples.extend_from_slice(&(rx - tx).to_le_bytes());
                last_rx = rx;
                received += 1;
            }
            Ok(None) => {
                let now = now_ns();
                let stalled = if received == 0 {
                    now - t_start > GRACE_NS
                } else {
                    now - last_rx > STALL_NS
                };
                if stalled {
                    break;
                }
                hint::spin_loop();
            }
            Err(e) => {
                eprintln!("receive: {e:?}");
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
    let Some(a) = parse_args() else { return usage() };
    if a.role == "pub" {
        run_pub(&a)
    } else {
        run_sub(&a)
    }
}
