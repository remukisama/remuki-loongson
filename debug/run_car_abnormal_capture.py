#!/usr/bin/env python3
import pathlib
import select
import socket
import struct
import subprocess
import sys
import threading
import time


ROOT = pathlib.Path("/home/zoumaguanbei/SmartCar_Project/Loongson_2k300_301_Library-master/Loongson_2K300_301_LIB")
BOARD = "root@192.168.141.100"


def main() -> int:
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    ts = time.strftime("%Y%m%d_%H%M%S")
    debug = ROOT / "debug"
    img_dir = debug / f"car_abnormal_udp_images_{ts}"
    img_dir.mkdir(parents=True, exist_ok=True)

    text_log = debug / f"car_abnormal_text_{ts}.log"
    image_log = debug / f"car_abnormal_image_{ts}.log"
    stdout_log = debug / f"car_abnormal_stdout_{ts}.log"
    meta_log = debug / f"car_abnormal_meta_{ts}.txt"
    capture_dir = debug / f"vision_race_capture_{ts}"
    capture_dir.mkdir(parents=True, exist_ok=True)

    stop = threading.Event()
    counts = {"text": 0, "img": 0, "img_bad": 0}

    def recv_text() -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 8080))
        sock.setblocking(False)
        with open(text_log, "wb") as out:
            while not stop.is_set():
                ready, _, _ = select.select([sock], [], [], 0.1)
                if not ready:
                    continue
                data, _addr = sock.recvfrom(65535)
                counts["text"] += 1
                out.write(data)
                if not data.endswith(b"\n"):
                    out.write(b"\n")
                out.flush()
        sock.close()

    def recv_image() -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 8081))
        sock.setblocking(False)

        expected = None
        buf = bytearray()
        idx = 0
        started_at = 0.0
        with open(image_log, "w", encoding="utf-8") as log:
            while not stop.is_set():
                ready, _, _ = select.select([sock], [], [], 0.1)
                if not ready:
                    if expected is not None and time.monotonic() - started_at > 1.0:
                        log.write(f"timeout idx={idx} got={len(buf)} expected={expected}\n")
                        log.flush()
                        counts["img_bad"] += 1
                        expected = None
                        buf.clear()
                    continue

                data, addr = sock.recvfrom(65535)
                now = time.time()
                if expected is None:
                    if len(data) == 4:
                        length = struct.unpack("<I", data)[0]
                        if 100 <= length <= 300000:
                            expected = length
                            started_at = time.monotonic()
                            buf.clear()
                            log.write(f"start idx={idx} t={now:.3f} len={length} from={addr}\n")
                        else:
                            log.write(f"bad_len value={length} raw={data.hex()}\n")
                            counts["img_bad"] += 1
                    else:
                        log.write(f"orphan_packet len={len(data)} head={data[:8].hex()}\n")
                        counts["img_bad"] += 1
                    log.flush()
                    continue

                buf.extend(data)
                if len(buf) >= expected:
                    jpg = bytes(buf[:expected])
                    path = img_dir / f"udp_img_{idx:03d}.jpg"
                    ok = jpg.startswith(b"\xff\xd8") and jpg.endswith(b"\xff\xd9")
                    path.write_bytes(jpg)
                    log.write(f"done idx={idx} t={now:.3f} got={len(jpg)} ok={1 if ok else 0} path={path}\n")
                    log.flush()
                    counts["img"] += 1
                    idx += 1
                    expected = None
                    buf.clear()
        sock.close()

    threads = [
        threading.Thread(target=recv_text, daemon=True),
        threading.Thread(target=recv_image, daemon=True),
    ]
    for thread in threads:
        thread.start()

    time.sleep(0.5)
    subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", BOARD, "pkill", "-x", "main"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", BOARD, "rm", "-rf", "/tmp/vision_race_capture"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    cmd = [
        "ssh", "-o", "StrictHostKeyChecking=no", BOARD,
        "timeout", "-k", "1s", f"{duration}s", "env", "LQ_RACE_CAPTURE=0", "./main", "--mode=car",
    ]
    start = time.time()
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    elapsed = time.time() - start
    stdout_log.write_bytes(proc.stdout)

    time.sleep(1.5)
    stop.set()
    for thread in threads:
        thread.join(timeout=1.0)

    post = subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", BOARD, "pgrep", "main"],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    subprocess.run(["scp", "-q", "-r", f"{BOARD}:/tmp/vision_race_capture/.", str(capture_dir)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    meta = {
        "ts": ts,
        "ssh_exit": proc.returncode,
        "duration_s": f"{elapsed:.3f}",
        "text_packets": counts["text"],
        "udp_images": counts["img"],
        "udp_image_bad": counts["img_bad"],
        "stdout_log": str(stdout_log),
        "text_log": str(text_log),
        "image_log": str(image_log),
        "image_dir": str(img_dir),
        "race_capture_dir": str(capture_dir),
        "post_pgrep": post.stdout.decode(errors="replace").strip(),
    }
    with open(meta_log, "w", encoding="utf-8") as out:
        for key, value in meta.items():
            out.write(f"{key}={value}\n")

    print(f"META={meta_log}")
    for key, value in meta.items():
        print(f"{key}={value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
