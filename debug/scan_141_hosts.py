#!/usr/bin/env python3
import concurrent.futures
import socket
import subprocess


def ping_host(ip: str) -> str | None:
    proc = subprocess.run(
        ["ping", "-c", "1", "-W", "1", ip],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if proc.returncode == 0:
        return ip
    return None


def ssh_open(ip: str) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.25)
    try:
        return sock.connect_ex((ip, 22)) == 0
    finally:
        sock.close()


def main() -> int:
    ips = [f"192.168.141.{i}" for i in range(1, 255)]
    live: list[str] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=96) as pool:
        for result in pool.map(ping_host, ips):
            if result:
                live.append(result)

    print("LIVE_PING", " ".join(live) if live else "(none)")
    ssh_hosts = [ip for ip in live if ssh_open(ip)]
    print("SSH_OPEN", " ".join(ssh_hosts) if ssh_hosts else "(none)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
