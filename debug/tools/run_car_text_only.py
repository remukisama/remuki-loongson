#!/usr/bin/env python3
import pathlib
import select
import socket
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
    text_log = debug / f"car_text_only_{ts}.log"
    stdout_log = debug / f"car_text_only_stdout_{ts}.log"
    meta_log = debug / f"car_text_only_meta_{ts}.txt"

    stop = threading.Event()
    count = 0

    def recv_text() -> None:
        nonlocal count
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
                count += 1
                out.write(data)
                if not data.endswith(b"\n"):
                    out.write(b"\n")
                out.flush()
        sock.close()

    thread = threading.Thread(target=recv_text, daemon=True)
    thread.start()
    time.sleep(0.5)

    subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", BOARD, "killall", "-9", "main"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    cmd = [
        "ssh", "-o", "StrictHostKeyChecking=no", BOARD,
        "timeout", "-k", "1s", f"{duration}s", "env", "LQ_RACE_CAPTURE=1", "./main", "--mode=car",
    ]
    start = time.time()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    timed_out = False
    try:
        stdout, _ = proc.communicate(timeout=duration + 4)
    except subprocess.TimeoutExpired:
        timed_out = True
        proc.kill()
        stdout, _ = proc.communicate(timeout=2)
        subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", BOARD, "killall", "-9", "main"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    elapsed = time.time() - start
    stdout_log.write_bytes(stdout or b"")
    time.sleep(0.5)
    stop.set()
    thread.join(timeout=1.0)

    post = subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", BOARD, "pgrep", "main"],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    meta = {
        "ts": ts,
        "ssh_exit": proc.returncode,
        "timed_out": int(timed_out),
        "duration_s": f"{elapsed:.3f}",
        "text_packets": count,
        "text_log": str(text_log),
        "stdout_log": str(stdout_log),
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
