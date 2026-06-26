#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Forward SmartCar tuning console output to a UDP tuning receiver.

Run this on the board as a stdout pipe.  It does not consume stdin, therefore
T05 interactive parameter input remains available in the original terminal.

Example:
  stdbuf -oL -eL ./main --mode=tune 2>&1 | \
    python3 tools/tuning/udp_tuning_forwarder.py --host 192.168.1.20 --port 24405
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
import uuid

PROTOCOL = "SC_TUNE/1"
MAX_TEXT_BYTES = 1200


def packet(session: str, seq: int, line: str) -> bytes:
    raw = line.rstrip("\r\n").encode("utf-8", errors="replace")
    if len(raw) > MAX_TEXT_BYTES:
        raw = raw[: MAX_TEXT_BYTES - 18] + b" ...[truncated]"
    header = f"{PROTOCOL}|{session}|{seq}|".encode("ascii")
    return header + raw


def main() -> int:
    parser = argparse.ArgumentParser(description="Forward T05/T11/T06 terminal logs to the UDP tuning receiver.")
    parser.add_argument("--host", required=True, help="Upper-computer IPv4 address, for example 192.168.1.20")
    parser.add_argument("--port", type=int, default=24405)
    parser.add_argument("--session", default=None, help="Optional session ID; default is generated automatically.")
    parser.add_argument("--quiet", action="store_true", help="Do not mirror received terminal lines back to stdout.")
    args = parser.parse_args()

    session = args.session or f"{int(time.time())}-{uuid.uuid4().hex[:8]}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    print(f"[UDP_TUNE_FORWARDER] session={session} target={args.host}:{args.port}", file=sys.stderr, flush=True)

    try:
        for line in sys.stdin:
            seq += 1
            try:
                sock.sendto(packet(session, seq, line), (args.host, args.port))
            except OSError as exc:
                print(f"[UDP_TUNE_FORWARDER] send failed: {exc}", file=sys.stderr, flush=True)
            if not args.quiet:
                sys.stdout.write(line)
                sys.stdout.flush()
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            sock.sendto(packet(session, seq + 1, "[UDP_TUNE_FORWARDER] EOF"), (args.host, args.port))
        except OSError:
            pass
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
