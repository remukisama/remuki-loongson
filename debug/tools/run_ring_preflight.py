#!/usr/bin/env python3
"""Capture a bounded, no-motor roundabout preflight from the board."""

from __future__ import annotations

import argparse
import pathlib
import select
import socket
import struct
import subprocess
import threading
import time


ROOT = pathlib.Path(__file__).resolve().parents[1]
BOARD = "root@192.168.145.100"
SSH = [
    "ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no",
    "-o", "ConnectTimeout=3", BOARD,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("duration", nargs="?", type=int, default=5)
    parser.add_argument("--board", default=BOARD)
    return parser.parse_args()


def remote_stop(ssh: list[str]) -> None:
    subprocess.run(
        ssh + ["pkill", "-TERM", "-x", "main"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=4,
        check=False,
    )


def receive_images(stop: threading.Event, output: pathlib.Path, state: dict[str, int]) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 8081))
    sock.setblocking(False)
    expected: int | None = None
    data_buf = bytearray()
    started = 0.0
    with (output / "udp_images.log").open("w", encoding="utf-8") as log:
        while not stop.is_set():
            ready, _, _ = select.select([sock], [], [], 0.1)
            if not ready:
                if expected is not None and time.monotonic() - started > 1.0:
                    state["bad"] += 1
                    log.write(f"timeout got={len(data_buf)} expected={expected}\n")
                    expected = None
                    data_buf.clear()
                continue
            packet, address = sock.recvfrom(65535)
            if expected is None:
                if len(packet) != 4:
                    state["bad"] += 1
                    continue
                length = struct.unpack("<I", packet)[0]
                if not 100 <= length <= 300000:
                    state["bad"] += 1
                    continue
                expected = length
                data_buf.clear()
                started = time.monotonic()
                log.write(f"start len={length} from={address}\n")
                continue
            data_buf.extend(packet)
            if len(data_buf) < expected:
                continue
            jpg = bytes(data_buf[:expected])
            valid = jpg.startswith(b"\xff\xd8") and jpg.endswith(b"\xff\xd9")
            path = output / f"udp_img_{state['images']:04d}.jpg"
            path.write_bytes(jpg)
            log.write(f"done idx={state['images']} bytes={len(jpg)} ok={int(valid)}\n")
            log.flush()
            state["images"] += 1
            if not valid:
                state["bad"] += 1
            expected = None
            data_buf.clear()
    sock.close()


def main() -> int:
    args = parse_args()
    if not 3 <= args.duration <= 15:
        raise SystemExit("duration must be in [3, 15] seconds")
    board = args.board if "@" in args.board else f"root@{args.board}"
    ssh = SSH[:-1] + [board]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    output = ROOT / "debug" / f"ring_preflight_{stamp}"
    output.mkdir(parents=True)

    stop = threading.Event()
    state = {"images": 0, "bad": 0}
    receiver = threading.Thread(target=receive_images, args=(stop, output, state), daemon=True)
    receiver.start()
    time.sleep(0.3)

    remote_stop(ssh)
    remote = [
        "timeout", "-s", "TERM", "-k", "2s", f"{args.duration}s",
        "env", "LQ_ELEMENT_OBSERVE_PROCESS=1", "stdbuf", "-oL", "-eL",
        "./main", "--mode=element-observe", "--display=udp",
    ]
    print(f"[RING_PREFLIGHT] no-motor capture {args.duration}s on {board}", flush=True)
    result = subprocess.run(
        ssh + remote,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=args.duration + 10,
        check=False,
    )
    (output / "stdout.log").write_text(result.stdout, encoding="utf-8")
    stop.set()
    receiver.join(timeout=2)
    remote_stop(ssh)
    post = subprocess.run(
        ssh + ["pgrep", "-af", "main"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        timeout=4,
        check=False,
    ).stdout.strip()

    observations = [line for line in result.stdout.splitlines() if line.startswith("[OBS]")]
    transitions = [line for line in result.stdout.splitlines() if line.startswith("[RING_")]
    print(f"[RING_PREFLIGHT] output={output}")
    print(f"[RING_PREFLIGHT] observations={len(observations)} images={state['images']} bad={state['bad']}")
    for line in transitions:
        print(line)
    if observations:
        print(f"[RING_PREFLIGHT] last={observations[-1]}")
    if post:
        print(f"[RING_PREFLIGHT][FAIL] residual main: {post}")
        return 2
    if not observations or not state["images"] or state["bad"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
