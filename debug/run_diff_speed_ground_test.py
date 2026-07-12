#!/usr/bin/env python3
import pathlib
import re
import statistics
import subprocess
import sys
import time


ROOT = pathlib.Path("/home/zoumaguanbei/SmartCar_Project/Loongson_2k300_301_Library-master/Loongson_2K300_301_LIB")
BOARD = "root@192.168.141.100"
SSH_BASE = ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=3", BOARD]


DIFF_RE = re.compile(
    r"p(?P<phase>\d+):(?P<name>\S+) "
    r"gL=(?P<gL>-?\d+\.\d+) gR=(?P<gR>-?\d+\.\d+) "
    r"eL=(?P<eL>-?\d+\.\d+) eR=(?P<eR>-?\d+\.\d+) "
    r"errL=(?P<errL>[+-]?\d+\.\d+) errR=(?P<errR>[+-]?\d+\.\d+) "
    r"pwmL=(?P<pwmL>-?\d+) pwmR=(?P<pwmR>-?\d+) "
    r"gz=(?P<gz>[+-]?\d+\.\d+) "
    r"tgtDiff=(?P<tgtDiff>[+-]?\d+\.\d+) actDiff=(?P<actDiff>[+-]?\d+\.\d+)"
)


def ssh_ok() -> bool:
    proc = subprocess.run(
        SSH_BASE + ["pwd"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return proc.returncode == 0


def wait_for_board(timeout_s: int) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ssh_ok():
            return True
        time.sleep(2.0)
    return False


def parse_rows(text: str) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    for line in text.splitlines():
        match = DIFF_RE.search(line)
        if not match:
            continue
        row: dict[str, float | int | str] = {}
        for key, value in match.groupdict().items():
            if key in {"phase", "pwmL", "pwmR"}:
                row[key] = int(value)
            elif key == "name":
                row[key] = value
            else:
                row[key] = float(value)
        rows.append(row)
    return rows


def summarize(rows: list[dict[str, float | int | str]]) -> str:
    out: list[str] = []
    by_name: dict[str, list[dict[str, float | int | str]]] = {}
    for row in rows:
        by_name.setdefault(str(row["name"]), []).append(row)

    for name, phase_rows in by_name.items():
        active = [r for r in phase_rows if abs(float(r["tgtDiff"])) > 0.05]
        sample = active or phase_rows
        if not sample:
            continue
        tgt = statistics.mean(float(r["tgtDiff"]) for r in sample)
        act = statistics.mean(float(r["actDiff"]) for r in sample)
        e_l = statistics.mean(float(r["eL"]) for r in sample)
        e_r = statistics.mean(float(r["eR"]) for r in sample)
        pwm_l = statistics.mean(float(r["pwmL"]) for r in sample)
        pwm_r = statistics.mean(float(r["pwmR"]) for r in sample)
        ratio = act / tgt if abs(tgt) > 1e-6 else 0.0
        same = sum(1 for r in sample if float(r["tgtDiff"]) * float(r["actDiff"]) > 0)
        same_rate = same / len(sample)
        out.append(
            f"{name}: n={len(sample)} tgtDiff={tgt:+.3f} actDiff={act:+.3f} "
            f"ratio={ratio:+.2f} same={same_rate:.0%} "
            f"eL={e_l:.3f} eR={e_r:.3f} pwmL={pwm_l:.0f} pwmR={pwm_r:.0f}"
        )
    return "\n".join(out)


def main() -> int:
    wait_s = int(sys.argv[1]) if len(sys.argv) > 1 else 90
    ts = time.strftime("%Y%m%d_%H%M%S")
    debug = ROOT / "debug"
    stdout_log = debug / f"diff_speed_ground_stdout_{ts}.log"
    meta_log = debug / f"diff_speed_ground_meta_{ts}.txt"
    summary_log = debug / f"diff_speed_ground_summary_{ts}.txt"

    if not wait_for_board(wait_s):
        meta_log.write_text(
            f"ts={ts}\nstatus=board_offline\nwait_s={wait_s}\nboard={BOARD}\n",
            encoding="utf-8",
        )
        print(f"META={meta_log}")
        print("status=board_offline")
        return 2

    subprocess.run(SSH_BASE + ["pkill", "-x", "main"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    proc = subprocess.run(
        SSH_BASE + ["timeout", "14s", "./main", "--mode=diff-speed-test"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    stdout_log.write_bytes(proc.stdout)

    post = subprocess.run(
        SSH_BASE + ["pgrep", "main"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    rows = parse_rows(proc.stdout.decode(errors="replace"))
    summary = summarize(rows)
    summary_log.write_text(summary + "\n", encoding="utf-8")
    meta_log.write_text(
        "\n".join(
            [
                f"ts={ts}",
                "status=done",
                f"ssh_exit={proc.returncode}",
                f"rows={len(rows)}",
                f"stdout_log={stdout_log}",
                f"summary_log={summary_log}",
                f"post_pgrep={post.stdout.decode(errors='replace').strip()}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"META={meta_log}")
    print(meta_log.read_text(encoding="utf-8"), end="")
    print(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
