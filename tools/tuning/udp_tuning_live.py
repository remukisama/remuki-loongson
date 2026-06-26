#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run the SmartCar live UDP tuning receiver on the upper computer."""

from __future__ import annotations

import argparse
import socketserver
from pathlib import Path

from live_tuning_logic import LiveTuner


class TuningServer(socketserver.ThreadingUDPServer):
    allow_reuse_address = True


def decode(data, sender):
    text = data.decode("utf-8", errors="replace").rstrip("\r\n")
    fields = text.split("|", 3)
    if len(fields) == 4 and fields[0] == "SC_TUNE/1":
        return fields[1], fields[3]
    return f"raw-{sender[0]}-{sender[1]}", text


def main():
    parser = argparse.ArgumentParser(description="Receive and analyze SmartCar tuning logs over UDP.")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=24405)
    parser.add_argument("--log-dir", type=Path, default=Path("udp_tuning_logs"))
    parser.add_argument("--curve-config", type=Path, default=Path(__file__).resolve().parents[2] / "user_app/include/competition_tuning_config.hpp")
    args = parser.parse_args()
    tuner = LiveTuner(args.log_dir, args.curve_config)

    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            data, _sock = self.request
            session, line = decode(data, self.client_address)
            for message in tuner.consume(session, line):
                print(message, flush=True)

    with TuningServer((args.bind, args.port), Handler) as server:
        print(f"[UDP_TUNE_LIVE] listen={args.bind}:{args.port}; logs={args.log_dir}", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
