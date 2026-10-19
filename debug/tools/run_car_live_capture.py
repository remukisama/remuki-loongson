#!/usr/bin/env python3
"""Run car mode with live UDP analysis and bounded safety guards.

The controller intentionally stops emitting ``steer`` UDP packets while the
IMU visual-only fallback branch is active.  SSH stdout is therefore captured
line-buffered as a second signal so fallback is reported rather than mistaken
for a dead controller.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import select
import socket
import statistics
import struct
import subprocess
import threading
import time
from typing import Dict, List, Optional


ROOT = pathlib.Path(
    "/home/zoumaguanbei/SmartCar_Project/"
    "Loongson_2k300_301_Library-master/Loongson_2K300_301_LIB"
)
BOARD = "root@192.168.145.100"
SSH_BASE = [
    "ssh",
    "-o", "BatchMode=yes",
    "-o", "StrictHostKeyChecking=no",
    "-o", "ConnectTimeout=3",
    "-o", "ServerAliveInterval=1",
    "-o", "ServerAliveCountMax=2",
    BOARD,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("duration", nargs="?", type=int, default=15)
    parser.add_argument(
        "--board",
        default=BOARD,
        help="board SSH target (default: root@192.168.145.100)",
    )
    parser.add_argument("--no-images", action="store_true")
    parser.add_argument("--wheel-guard", type=float, default=1.20)
    return parser.parse_args()


def parse_steer(line: str) -> Optional[Dict[str, float]]:
    if not line.startswith("steer,"):
        return None
    values: Dict[str, float] = {}
    try:
        for item in line.strip().split(",")[1:]:
            key, value = item.split("=", 1)
            values[key] = float(value)
    except (TypeError, ValueError):
        return None
    return values


def percentile(values: List[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def metric(values: List[float]) -> str:
    if not values:
        return "n=0"
    mae = statistics.fmean(abs(v) for v in values)
    rms = math.sqrt(statistics.fmean(v * v for v in values))
    p95 = percentile([abs(v) for v in values], 0.95)
    return f"n={len(values)} mae={mae:.3f} rms={rms:.3f} p95={p95:.3f}"


def remote_stop() -> None:
    subprocess.run(
        SSH_BASE + ["pkill", "-TERM", "-x", "main"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=4,
        check=False,
    )


def main() -> int:
    global BOARD
    args = parse_args()
    if args.duration < 5 or args.duration > 120:
        raise SystemExit("duration must be in [5, 120] seconds")
    if args.wheel_guard < 0.1 or args.wheel_guard > 5.0:
        raise SystemExit("wheel guard must be in [0.1, 5.0] m/s")
    BOARD = args.board if "@" in args.board else f"root@{args.board}"
    SSH_BASE[-1] = BOARD

    ts = time.strftime("%Y%m%d_%H%M%S")
    debug = ROOT / "debug"
    text_log = debug / f"car_live_text_{ts}.log"
    stdout_log = debug / f"car_live_stdout_{ts}.log"
    image_log = debug / f"car_live_image_{ts}.log"
    meta_log = debug / f"car_live_meta_{ts}.txt"
    image_dir = debug / f"car_live_udp_images_{ts}"
    if not args.no_images:
        image_dir.mkdir(parents=True, exist_ok=True)

    stop = threading.Event()
    fallback_active = threading.Event()
    guard_reason: List[str] = []
    samples: List[Dict[str, float]] = []
    samples_lock = threading.Lock()
    state = {
        "text_packets": 0,
        "images": 0,
        "image_bad": 0,
        "first_packet": 0.0,
        "last_packet": 0.0,
        "full_loss_run": 0,
        "cross_full_loss_started": 0.0,
        "ring_full_loss_started": 0.0,
        "ring_full_loss_stage": 0,
        "ring_full_loss_last_progress": 0.0,
        "slow_dt_run": 0,
        "fallback_events": 0,
        "fallback_recoveries": 0,
    }

    def request_guard_stop(reason: str) -> None:
        if guard_reason:
            return
        guard_reason.append(reason)
        print(f"[LIVE][GUARD] {reason}; requesting graceful stop", flush=True)
        remote_stop()

    def recv_text() -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 8080))
        sock.setblocking(False)
        with text_log.open("w", encoding="utf-8") as out:
            while not stop.is_set():
                ready, _, _ = select.select([sock], [], [], 0.1)
                if not ready:
                    continue
                data, addr = sock.recvfrom(65535)
                now = time.monotonic()
                decoded = data.decode("utf-8", errors="replace")
                for line in decoded.splitlines():
                    sample = parse_steer(line)
                    if sample is None:
                        out.write(line + "\n")
                        continue
                    if not state["first_packet"]:
                        state["first_packet"] = now
                    sample["host_t"] = now - state["first_packet"]
                    out.write(f"{line},host_t={sample['host_t']:.6f}\n")
                    with samples_lock:
                        samples.append(sample)
                    state["text_packets"] += 1
                    state["last_packet"] = now

                    if fallback_active.is_set():
                        fallback_active.clear()
                        state["fallback_recoveries"] += 1
                        print("[LIVE][IMU] steer UDP resumed; gyro path recovered", flush=True)

                    off = sample.get("off", 0.0)
                    road_type = int(sample.get("rt", 0.0))
                    ring_side = int(sample.get("ring", 0.0))
                    ring_stage = int(sample.get("rs", 0.0))
                    target_gyro = sample.get("tgt", 0.0)
                    real_gyro = sample.get("gz", 0.0)
                    dt = sample.get("dt", 0.0)
                    enc_l = sample.get("encL", 0.0)
                    enc_r = sample.get("encR", 0.0)
                    if off >= 45 and ring_side and ring_stage:
                        state["full_loss_run"] = 0
                        state["cross_full_loss_started"] = 0.0
                        if (
                            not state["ring_full_loss_started"]
                            or state["ring_full_loss_stage"] != ring_stage
                        ):
                            state["ring_full_loss_started"] = now
                            state["ring_full_loss_stage"] = ring_stage
                            state["ring_full_loss_last_progress"] = now
                        ring_turn_tracking = (
                            abs(target_gyro) >= 20.0
                            and target_gyro * real_gyro > 0.0
                            and abs(real_gyro) >= max(8.0, 0.25 * abs(target_gyro))
                        )
                        if ring_turn_tracking:
                            state["ring_full_loss_last_progress"] = now
                    elif off >= 45 and road_type in (3, 7):
                        state["full_loss_run"] = 0
                        state["ring_full_loss_started"] = 0.0
                        state["ring_full_loss_stage"] = 0
                        state["ring_full_loss_last_progress"] = 0.0
                        if not state["cross_full_loss_started"]:
                            state["cross_full_loss_started"] = now
                    else:
                        state["cross_full_loss_started"] = 0.0
                        state["ring_full_loss_started"] = 0.0
                        state["ring_full_loss_stage"] = 0
                        state["ring_full_loss_last_progress"] = 0.0
                        state["full_loss_run"] = state["full_loss_run"] + 1 if off >= 45 else 0
                    state["slow_dt_run"] = state["slow_dt_run"] + 1 if dt >= 0.08 else 0

                    n = state["text_packets"]
                    if n == 1 or n % 10 == 0 or abs(sample.get("raw", 0.0)) >= 12:
                        print(
                            "[LIVE] "
                            f"n={n} raw={sample.get('raw', math.nan):+.1f} "
                            f"tgt={sample.get('tgt', math.nan):+.1f} "
                            f"gz={sample.get('gz', math.nan):+.1f} "
                            f"goal={sample.get('goal', math.nan):.2f} "
                            f"enc={enc_l:.2f}/{enc_r:.2f} "
                            f"off/ll/rl={off:.0f}/{sample.get('ll', 0):.0f}/{sample.get('rl', 0):.0f} "
                            f"ring={ring_side}/{ring_stage}",
                            flush=True,
                        )

                    if max(abs(enc_l), abs(enc_r)) > args.wheel_guard:
                        request_guard_stop(f"wheel speed outlier {enc_l:.2f}/{enc_r:.2f} m/s")
                    elif state["full_loss_run"] >= 8:
                        request_guard_stop(f"persistent full line loss off={off:.0f}")
                    elif (
                        state["cross_full_loss_started"]
                        and now - state["cross_full_loss_started"] >= 1.2
                    ):
                        request_guard_stop(
                            f"cross full line loss persisted for 1.2s off={off:.0f}"
                        )
                    elif state["ring_full_loss_started"]:
                        ring_loss_age = now - state["ring_full_loss_started"]
                        ring_progress_age = now - state["ring_full_loss_last_progress"]
                        if ring_loss_age >= 3.0:
                            request_guard_stop(
                                "ring full line loss stayed in one stage for "
                                f"3.0s stage={ring_stage} off={off:.0f}"
                            )
                        elif ring_loss_age >= 1.2 and ring_progress_age >= 0.6:
                            request_guard_stop(
                                "ring turn stopped tracking for 0.6s "
                                f"stage={ring_stage} off={off:.0f} "
                                f"tgt/gz={target_gyro:.1f}/{real_gyro:.1f}"
                            )
                    elif state["slow_dt_run"] >= 3:
                        request_guard_stop(f"control dt stalled at {dt:.3f}s")
                out.flush()
        sock.close()

    def recv_image() -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 8081))
        sock.setblocking(False)
        expected: Optional[int] = None
        buf = bytearray()
        started = 0.0
        with image_log.open("w", encoding="utf-8") as log:
            while not stop.is_set():
                ready, _, _ = select.select([sock], [], [], 0.1)
                if not ready:
                    if expected is not None and time.monotonic() - started > 1.0:
                        state["image_bad"] += 1
                        log.write(f"timeout got={len(buf)} expected={expected}\n")
                        expected = None
                        buf.clear()
                    continue
                data, addr = sock.recvfrom(65535)
                if expected is None:
                    if len(data) != 4:
                        state["image_bad"] += 1
                        continue
                    length = struct.unpack("<I", data)[0]
                    if not 100 <= length <= 300000:
                        state["image_bad"] += 1
                        continue
                    expected = length
                    started = time.monotonic()
                    buf.clear()
                    log.write(f"start len={length} from={addr}\n")
                    continue
                buf.extend(data)
                if len(buf) >= expected:
                    jpg = bytes(buf[:expected])
                    index = state["images"]
                    path = image_dir / f"udp_img_{index:03d}.jpg"
                    ok = jpg.startswith(b"\xff\xd8") and jpg.endswith(b"\xff\xd9")
                    path.write_bytes(jpg)
                    state["images"] += 1
                    if not ok:
                        state["image_bad"] += 1
                    image_now = time.monotonic()
                    image_host_t = (
                        image_now - state["first_packet"]
                        if state["first_packet"]
                        else -1.0
                    )
                    log.write(
                        f"done idx={index} host_t={image_host_t:.6f} "
                        f"bytes={len(jpg)} ok={int(ok)}\n"
                    )
                    log.flush()
                    expected = None
                    buf.clear()
        sock.close()

    threads = [threading.Thread(target=recv_text, daemon=True)]
    if not args.no_images:
        threads.append(threading.Thread(target=recv_image, daemon=True))
    for thread in threads:
        thread.start()
    time.sleep(0.4)

    remote_stop()
    time.sleep(0.3)
    board_hard_limit = args.duration + 12
    remote_cmd = [
        "timeout", "-s", "TERM", "-k", "3s", f"{board_hard_limit}s",
        "env", "LQ_RACE_CAPTURE=0", "stdbuf", "-oL", "-eL",
        "./main", "--mode=car",
    ]
    print(
        f"[LIVE] starting car run: {args.duration}s active telemetry "
        f"({board_hard_limit}s board hard limit) on {BOARD}",
        flush=True,
    )
    started_at = time.monotonic()
    proc = subprocess.Popen(
        SSH_BASE + remote_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    last_stdout = started_at
    active_duration_complete = False
    with stdout_log.open("w", encoding="utf-8") as out:
        assert proc.stdout is not None
        while proc.poll() is None:
            ready, _, _ = select.select([proc.stdout], [], [], 0.1)
            if ready:
                line = proc.stdout.readline()
                if line:
                    last_stdout = time.monotonic()
                    out.write(line)
                    out.flush()
                    stripped = line.strip()
                    if "[IMU_GUARD]" in stripped:
                        if not fallback_active.is_set():
                            state["fallback_events"] += 1
                        fallback_active.set()
                        print(f"[LIVE][IMU] {stripped}; keeping visual-speed fallback", flush=True)
                    elif "[MPU6050_GUARD]" in stripped or "[MPU6050_WORKER]" in stripped:
                        print(f"[LIVE][IMU] {stripped}", flush=True)
                    elif any(token in stripped for token in (
                        "[MAIN][FATAL]", "[MOTION_GUARD]", "[VISION_GUARD] fatal",
                        "[CONTROL_GUARD]", "[ENCODER_GUARD]",
                    )):
                        print(f"[LIVE][BOARD] {stripped}", flush=True)

            now = time.monotonic()
            first_packet = state["first_packet"]
            last_packet = state["last_packet"]
            active_duration = (
                last_packet - first_packet
                if first_packet and last_packet >= first_packet
                else 0.0
            )
            if not active_duration_complete and active_duration >= args.duration:
                active_duration_complete = True
                print(
                    f"[LIVE] reached {args.duration}s active telemetry; stopping",
                    flush=True,
                )
                remote_stop()
            if (
                first_packet
                and not fallback_active.is_set()
                and now - last_packet > 2.0
                and now - last_stdout > 2.0
            ):
                request_guard_stop("both UDP telemetry and board stdout silent for 2s")

            if guard_reason and now - started_at > 1.0:
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    remote_stop()
                break

        remaining = proc.stdout.read()
        if remaining:
            out.write(remaining)

    try:
        ssh_exit = proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        remote_stop()
        proc.kill()
        ssh_exit = proc.wait(timeout=2)

    elapsed = time.monotonic() - started_at
    time.sleep(0.8)
    stop.set()
    for thread in threads:
        thread.join(timeout=1.0)

    post = subprocess.run(
        SSH_BASE + ["pgrep", "-a", "main"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=4,
        check=False,
    )

    with samples_lock:
        snapshot = list(samples)
    raw = [s.get("raw", 0.0) for s in snapshot]
    speed_err = [
        (s.get("encL", 0.0) + s.get("encR", 0.0)) * 0.5 - s.get("goal", 0.0)
        for s in snapshot
    ]
    gyro_err = [s.get("ge", 0.0) for s in snapshot if abs(s.get("tgt", 0.0)) >= 5.0]
    se_l = [s.get("seL", 0.0) for s in snapshot]
    se_r = [s.get("seR", 0.0) for s in snapshot]
    first_packet_delay = (
        state["first_packet"] - started_at if state["first_packet"] else math.nan
    )
    active_telemetry_duration = (
        state["last_packet"] - state["first_packet"]
        if state["first_packet"] and state["last_packet"] >= state["first_packet"]
        else 0.0
    )

    summary = {
        "ts": ts,
        "run_mode": "car",
        "board": BOARD,
        "duration_requested_s": args.duration,
        "duration_actual_s": f"{elapsed:.3f}",
        "ssh_exit": ssh_exit,
        "guard_reason": guard_reason[0] if guard_reason else "",
        "post_pgrep": post.stdout.strip(),
        "text_packets": state["text_packets"],
        "udp_images": state["images"],
        "udp_image_bad": state["image_bad"],
        "fallback_events": state["fallback_events"],
        "fallback_recoveries": state["fallback_recoveries"],
        "active_duration_complete": int(active_duration_complete),
        "first_packet_delay_s": (
            f"{first_packet_delay:.3f}" if math.isfinite(first_packet_delay) else "nan"
        ),
        "active_telemetry_duration_s": f"{active_telemetry_duration:.3f}",
        "raw_error": metric(raw),
        "avg_speed_error": metric(speed_err),
        "gyro_error_active": metric(gyro_err),
        "speed_error_left": metric(se_l),
        "speed_error_right": metric(se_r),
        "text_log": str(text_log),
        "stdout_log": str(stdout_log),
        "image_dir": str(image_dir) if not args.no_images else "",
    }
    with meta_log.open("w", encoding="utf-8") as out:
        for key, value in summary.items():
            out.write(f"{key}={value}\n")

    print("[LIVE] completed; board main process is stopped", flush=True)
    print(f"META={meta_log}")
    for key, value in summary.items():
        print(f"{key}={value}")
    if post.stdout.strip():
        return 2
    return 0 if active_duration_complete and not guard_reason else 1


if __name__ == "__main__":
    raise SystemExit(main())
