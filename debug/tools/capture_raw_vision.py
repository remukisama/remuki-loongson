#!/usr/bin/env python3
"""Capture static source-camera frames without enabling the motors.

This utility deliberately runs ``main --mode=vision-capture`` rather than
``--mode=car``.  It is intended for documenting the entrance, reversal point,
and exit of a feature before a track test.  The board-side diagnostic saves
the color source frame (with its ROI/status annotation) under
``/tmp/vision_capture``; this script retrieves those JPEGs over SSH.
"""

from __future__ import annotations

import argparse
import io
import json
import os
import pathlib
import subprocess
import tarfile
import time
from typing import Iterable


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_BOARD = os.environ.get("SMARTCAR_BOARD", "root@192.168.145.100")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture static color camera frames; never starts car mode."
    )
    parser.add_argument("--label", default="feature", help="human-readable location label")
    parser.add_argument("--frames", type=int, default=12, help="12 to 500 requested frames")
    parser.add_argument("--timeout", type=int, default=12, help="4 to 30 seconds")
    parser.add_argument("--board", default=DEFAULT_BOARD)
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=ROOT / "debug" / "raw_vision",
        help="local directory for retrieved JPEGs",
    )
    return parser.parse_args()


def ssh_base(board: str) -> list[str]:
    return [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "ConnectTimeout=3",
        "-o",
        "ServerAliveInterval=1",
        "-o",
        "ServerAliveCountMax=2",
        board,
    ]


def run_text(command: list[str], timeout: int) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )


def safe_members(archive: tarfile.TarFile) -> Iterable[tarfile.TarInfo]:
    for member in archive.getmembers():
        path = pathlib.PurePosixPath(member.name)
        if (
            not member.isfile()
            or path.is_absolute()
            or ".." in path.parts
            or path.name != member.name
        ):
            raise RuntimeError(f"unexpected archive member: {member.name!r}")
        yield member


def main() -> int:
    args = parse_args()
    if not 12 <= args.frames <= 500:
        raise SystemExit("--frames must be in [12, 500]")
    if not 4 <= args.timeout <= 30:
        raise SystemExit("--timeout must be in [4, 30] seconds")
    base = ssh_base(args.board)

    # Do not interrupt a running track test just to take a diagnostic image.
    active = run_text(base + ["pgrep -a main || true"], timeout=6)
    if active.returncode != 0:
        raise SystemExit(f"cannot inspect board process: {active.stdout.strip()}")
    if active.stdout.strip():
        raise SystemExit(
            "refusing raw capture because board main is already running:\n"
            + active.stdout.strip()
        )

    remote_capture = (
        f"timeout -s TERM -k 2s {args.timeout}s "
        f"env LQ_VISION_CAPTURE_FRAMES={args.frames} ./main --mode=vision-capture"
    )
    print(f"[RAW] static capture label={args.label!r}; motors remain disabled", flush=True)
    result = run_text(base + [remote_capture], timeout=args.timeout + 12)
    print(result.stdout, end="")
    if result.returncode != 0:
        raise SystemExit(f"vision-capture failed with ssh exit {result.returncode}")

    # The diagnostic keeps exactly twelve output slots even in stress mode.
    names = [f"frame_{index:02d}_raw.jpg" for index in range(12)]
    archive_cmd = "tar -C /tmp/vision_capture -cf - -- " + " ".join(names)
    archive = subprocess.run(
        base + [archive_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=12,
        check=False,
    )
    if archive.returncode != 0:
        raise SystemExit(
            "could not retrieve source frames: "
            + archive.stderr.decode("utf-8", errors="replace").strip()
        )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    destination = args.output_dir / f"{timestamp}_{args.label}"
    destination.mkdir(parents=True, exist_ok=False)
    with tarfile.open(fileobj=io.BytesIO(archive.stdout), mode="r:") as bundle:
        for member in safe_members(bundle):
            source = bundle.extractfile(member)
            if source is None:
                raise RuntimeError(f"cannot read archive member: {member.name}")
            (destination / member.name).write_bytes(source.read())

    meta = {
        "label": args.label,
        "board": args.board,
        "requested_frames": args.frames,
        "retrieved_frames": len(names),
        "mode": "vision-capture (no motor drive)",
        "directory": str(destination),
    }
    (destination / "capture.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"[RAW] saved {len(names)} source frames to {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
