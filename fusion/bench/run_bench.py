#!/usr/bin/env python3
"""v0.2 transport bake-off orchestrator (ADR-0005).

Spawns cross-process producer/consumer pairs per contender, collects the
(seq, latency) files the bench mains write, and prints a markdown table.
No analysis runs in any contender's hot loop — everything here is offline.

Usage:
  pixi run python fusion/bench/run_bench.py [--quick] [--contenders c,d]
      [--rates 500,5000] [--n N] [--outdir DIR]

Results go to eval/out/bench-<timestamp>/ (gitignored): raw .bin latency
files, .pubinfo sidecars, summary.json + SUMMARY.md.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BENCH = ROOT / "fusion" / "bench"
BUILD = ROOT / "fusion" / "build"
IOX2 = BENCH / "iox2"
IOX2_BIN = IOX2 / "target" / "release" / "bench-iox2"
RUST_WS = ROOT / "rust"
RUST_E_BIN = RUST_WS / "target" / "release" / "bench_e"
RUST_F_BIN = RUST_WS / "target" / "release" / "bench_f"

DEFAULT_RATES = (500, 5000)  # sim rate + 10x stress probe (ADR-0005)

CONTENDERS = {
    # name: (pub argv, sub argv, cleanup callable)
    "c": ([str(BUILD / "bench_c")], [str(BUILD / "bench_c")], "shm_c"),
    "d": ([str(BUILD / "bench_d")], [str(BUILD / "bench_d")], "shm_d"),
    "a": ([str(BUILD / "bench_a_pub")], [str(BUILD / "bench_a_sub")], None),
    "b": ([str(IOX2_BIN)], [str(IOX2_BIN)], "iox2"),
    "e": ([str(RUST_E_BIN)], [str(RUST_E_BIN)], "shm_e"),
    "f": ([str(RUST_F_BIN)], [str(RUST_F_BIN)], "iox2"),
}


def cleanup_shm_c():
    for p in ("/dev/shm/otolith_bench_c",):
        try:
            os.unlink(p)
        except FileNotFoundError:
            pass


def cleanup_shm_d():
    for p in ("/dev/shm/otolith_bench_d",
              "/dev/shm/sem.otolith_bdt_e",
              "/dev/shm/sem.otolith_bdt_f"):
        try:
            os.unlink(p)
        except FileNotFoundError:
            pass


def cleanup_iox2():
    # Stale service registry carries the old static config (buffer sizes);
    # open_or_create would reject a mismatched config. Bench fixture owns it.
    shutil.rmtree("/tmp/iceoryx2", ignore_errors=True)


def cleanup_shm_e():
    try:
        os.unlink("/dev/shm/otolith_bench_e")
    except FileNotFoundError:
        pass


CLEANUPS = {"shm_c": cleanup_shm_c, "shm_d": cleanup_shm_d, "iox2": cleanup_iox2,
            "shm_e": cleanup_shm_e}


def wait_file(path: Path, timeout: float) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if path.exists():
            return True
        time.sleep(0.05)
    return False


def kill_tree(proc: subprocess.Popen):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass


def start_router(env: dict):
    """Contender A needs rmw_zenohd: zenoh peers in this env do not
    discover router-less (multicast scouting dead), so the router is part
    of the as-deployed fixture. Traffic hairpins pub->router->sub; the
    bench README records this. Returns the Popen or None."""
    ros2 = shutil.which("ros2")
    if ros2 is None:
        print("ros2 not on PATH: contender a cannot discover", file=sys.stderr)
        return None
    r = subprocess.Popen(
        [ros2, "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        env=env, start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)  # let the router bind; pub's discovery gate covers the rest
    return r


def run_pair(name: str, pub_argv, sub_argv, cleanup, rate: int, n: int,
             outdir: Path, env: dict) -> dict:
    """Run one producer/consumer pair. Returns a result dict (never raises)."""
    res = {"contender": name, "rate_hz": rate, "n": n, "ok": False}
    lat = outdir / f"{name}_{rate}.bin"
    ready = outdir / f"{name}_{rate}.ready"
    for p in (lat, ready, Path(str(lat) + ".pubinfo")):
        try:
            p.unlink()
        except FileNotFoundError:
            pass
    if cleanup:
        CLEANUPS[cleanup]()

    common = ["--rate", str(rate), "--n", str(n)]
    try:
        sub = subprocess.Popen(
            [*sub_argv, "--role", "sub", *common,
             "--out", str(lat), "--ready", str(ready)],
            env=env, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as e:
        res["error"] = f"sub spawn: {e}"
        return res
    if not wait_file(ready, 15.0):
        kill_tree(sub)
        res["error"] = "sub never ready (15 s)"
        return res
    try:
        pub = subprocess.Popen(
            [*pub_argv, "--role", "pub", *common, "--out", str(lat)],
            env=env, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as e:
        kill_tree(sub)
        res["error"] = f"pub spawn: {e}"
        return res

    budget = n / rate + 20.0
    try:
        pub.wait(timeout=budget)
    except subprocess.TimeoutExpired:
        kill_tree(pub)
        res["error"] = "pub timeout"
    try:
        sub.wait(timeout=budget)
    except subprocess.TimeoutExpired:
        kill_tree(sub)
        res["error"] = "sub timeout"
    if "error" in res:
        return res

    res.update(analyze(lat, n))
    res["ok"] = True
    return res


def analyze(lat_path: Path, n: int) -> dict:
    d = np.fromfile(str(lat_path),
                    dtype=[("seq", "<u8"), ("lat", "<u8")])
    received = len(d)
    sent = n
    prod_drops = 0
    sidecar = Path(str(lat_path) + ".pubinfo")
    if sidecar.exists():
        kv = dict(kv.split("=") for kv in sidecar.read_text().split())
        sent = int(kv.get("sent", n))
        prod_drops = int(kv.get("dropped", 0))
    transport_drops = max(0, sent - received)
    out = {
        "received": received,
        "producer_drops": prod_drops,
        "transport_drops": transport_drops,
        "drop_pct": 100.0 * (n - received) / n,
    }
    if received:
        lat_us = d["lat"].astype(float) / 1000.0
        out.update({
            "mean_us": float(lat_us.mean()),
            "p50_us": float(np.percentile(lat_us, 50)),
            "p99_us": float(np.percentile(lat_us, 99)),
            "max_us": float(lat_us.max()),
        })
        edges = [1, 10, 100, 1000, 10000]
        hist, _ = np.histogram(lat_us, bins=[0, *edges, np.inf])
        out["hist_us"] = {f"<{e}": int(c) for e, c in zip([*edges, "inf"], hist)}
    return out


def fmt_row(r: dict) -> str:
    if not r["ok"]:
        return f"| {r['contender']} | {r['rate_hz']} | ERROR: {r.get('error')} |"
    return (f"| {r['contender']} | {r['rate_hz']} | {r['received']}/{r['n']} | "
            f"{r['drop_pct']:.2f}% ({r['producer_drops']}+{r['transport_drops']}) | "
            f"{r['mean_us']:.1f} | {r['p50_us']:.1f} | {r['p99_us']:.1f} | {r['max_us']:.1f} |")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true",
                    help="smoke: 200 msgs @500 Hz, contenders c+d only")
    ap.add_argument("--contenders", default="c,d,a,b,e,f")
    ap.add_argument("--rates", default="500,5000")
    ap.add_argument("--n", type=int, default=0,
                    help="messages per run (default: 5 s worth at each rate)")
    ap.add_argument("--outdir", default="")
    args = ap.parse_args()

    names = [c.strip() for c in args.contenders.split(",") if c.strip()]
    for c in names:
        if c not in CONTENDERS:
            print(f"unknown contender {c}", file=sys.stderr)
            return 2
    if args.quick:
        names = [c for c in names if c in ("c", "d")]
        rates = (500,)
        n_default = 200
    else:
        rates = tuple(int(r) for r in args.rates.split(","))
        n_default = 0

    if "b" in names and IOX2_BIN.exists() is False:
        print("building iceoryx2 bench (cargo fetch, option-1 per ADR-0005)...")
        r = subprocess.run(["cargo", "build", "--release"], cwd=str(IOX2))
        if r.returncode != 0 or not IOX2_BIN.exists():
            print("cargo build failed; dropping contender b", file=sys.stderr)
            names = [c for c in names if c != "b"]

    if "e" in names and RUST_E_BIN.exists() is False:
        print("building rust workspace (pixi cargo)...")
        r = subprocess.run(
            ["cargo", "build", "--release", "--manifest-path", str(RUST_WS / "Cargo.toml")])
        if r.returncode != 0 or not RUST_E_BIN.exists():
            print("cargo build failed; dropping contender e", file=sys.stderr)
            names = [c for c in names if c != "e"]

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path(args.outdir) if args.outdir else ROOT / "eval" / "out" / f"bench-{stamp}"
    outdir.mkdir(parents=True, exist_ok=True)

    env = dict(os.environ)
    env["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    # NOTE: default ROS domain (as deployed). Stop go2_demo.sh before
    # benching to avoid crosstalk; see bench README.

    router = start_router(env) if "a" in names else None
    results = []
    try:
        for name in names:
            pub_argv, sub_argv, cleanup = CONTENDERS[name]
            for rate in rates:
                n = args.n or n_default or rate * 5
                print(f"[{name} @ {rate} Hz x{n}] ...", flush=True)
                r = run_pair(name, pub_argv, sub_argv, cleanup, rate, n, outdir, env)
                print("   ", fmt_row(r), flush=True)
                results.append(r)
    finally:
        if router is not None:
            kill_tree(router)

    header = ("| contender | rate (Hz) | recv/sent | drops (prod+transport) | "
              "mean (us) | p50 (us) | p99 (us) | max (us) |")
    lines = [header, "|" + "---|" * 8,
             *[fmt_row(r) for r in results]]
    table = "\n".join(lines) + "\n"
    print("\n" + table)
    (outdir / "SUMMARY.md").write_text(
        f"# Transport bake-off {stamp}\n\n{table}\n")
    (outdir / "summary.json").write_text(json.dumps(results, indent=1))
    if any(not r["ok"] for r in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
