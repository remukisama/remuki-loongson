#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Upper-computer UDP receiver for SmartCar tuning messages."""

from __future__ import annotations

import argparse
import socketserver


class TuningReceiver(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        data, _sock = self.request
        text = data.decode("utf-8", errors="replace").rstrip()
        print(text, flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive SmartCar tuning UDP messages.")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=24405)
    args = parser.parse_args()
    with socketserver.ThreadingUDPServer((args.bind, args.port), TuningReceiver) as server:
        print(f"[UDP_TUNE_RECEIVER] listen={args.bind}:{args.port}", flush=True)
        server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
