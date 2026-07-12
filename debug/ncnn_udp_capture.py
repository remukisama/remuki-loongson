#!/usr/bin/env python3
import argparse
import pathlib
import select
import signal
import socket
import struct
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="debug/ncnn_vehicle_live")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--max-len", type=int, default=400000)
    args = parser.parse_args()

    img_dir = pathlib.Path(args.dir)
    img_dir.mkdir(parents=True, exist_ok=True)
    log_path = img_dir.with_suffix(".log")
    meta_path = img_dir.with_suffix(".meta.txt")

    stop = False

    def on_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, on_stop)
    signal.signal(signal.SIGINT, on_stop)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.setblocking(False)

    expected = None
    buf = bytearray()
    idx = 0
    bad = 0
    started = 0.0

    with open(log_path, "w", encoding="utf-8") as log:
        log.write(f"listen port={args.port} dir={img_dir}\n")
        log.flush()
        while not stop:
            ready, _, _ = select.select([sock], [], [], 0.1)
            now_m = time.monotonic()
            if not ready:
                if expected is not None and now_m - started > 1.0:
                    log.write(f"timeout idx={idx} got={len(buf)} expected={expected}\n")
                    log.flush()
                    bad += 1
                    expected = None
                    buf.clear()
                continue

            data, addr = sock.recvfrom(65535)
            if expected is None:
                if len(data) == 4:
                    length = struct.unpack("<I", data)[0]
                    if 100 <= length <= args.max_len:
                        expected = length
                        started = now_m
                        buf.clear()
                        log.write(f"start idx={idx} len={length} from={addr}\n")
                    else:
                        log.write(f"bad_len value={length} raw={data.hex()}\n")
                        bad += 1
                else:
                    log.write(f"orphan len={len(data)} head={data[:8].hex()}\n")
                    bad += 1
                log.flush()
                continue

            buf.extend(data)
            if len(buf) >= expected:
                jpg = bytes(buf[:expected])
                ok = jpg.startswith(b"\xff\xd8") and jpg.endswith(b"\xff\xd9")
                path = img_dir / f"udp_img_{idx:04d}.jpg"
                path.write_bytes(jpg)
                log.write(f"done idx={idx} got={len(jpg)} ok={1 if ok else 0} path={path}\n")
                log.flush()
                idx += 1
                expected = None
                buf.clear()

    sock.close()
    with open(meta_path, "w", encoding="utf-8") as meta:
        meta.write(f"udp_images={idx}\n")
        meta.write(f"udp_bad={bad}\n")
        meta.write(f"image_dir={img_dir.resolve()}\n")
        meta.write(f"image_log={log_path.resolve()}\n")
    print(f"stopped images={idx} bad={bad}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
