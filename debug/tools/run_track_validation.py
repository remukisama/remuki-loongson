#!/usr/bin/env python3
"""Run or replay one bounded, named track-feature validation case.

Each physical feature is tested from its own fixed start point.  This avoids
guessing the order of the small S, short straight, U-turns, cross, and the two
roundabouts from vision telemetry alone.  ``--run`` delegates all motion and
safety protection to ``run_car_live_capture.py``; without it, this script is
an offline analyzer and never contacts the board.

Temperature is intentionally optional input rather than a fabricated sensor:
without an external temperature CSV the report is marked ``REVIEW`` instead
of claiming that braking thermal safety passed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import re
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any, Iterable, Optional


ROOT = pathlib.Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "debug" / "run_car_live_capture.py"
DEFAULT_OUTPUT = ROOT / "debug" / "track_validation"


@dataclass(frozen=True)
class TestCase:
    key: str
    title: str
    duration_s: int
    max_duration_s: int
    expected: str


CASES: dict[str, TestCase] = {
    "small_s": TestCase(
        "small_s", "完整小 S 弯", 8, 10, "至少两次稳定转向反向"
    ),
    "straight_lt5m": TestCase(
        "straight_lt5m", "小于 5m 的短直道", 6, 8, "转向目标以直行为主"
    ),
    "u_turn_a": TestCase("u_turn_a", "U 型弯 A", 10, 12, "持续单方向转向"),
    "u_turn_b": TestCase("u_turn_b", "U 型弯 B", 10, 12, "持续单方向转向"),
    "cross": TestCase("cross", "十字路口", 8, 10, "按通用中线通过，不产生 Cross/Cross_ture 状态"),
    "ring_small": TestCase(
        "ring_small", "小半径环岛", 12, 15, "人工确认的小半径持续曲线"
    ),
    "ring_large": TestCase(
        "ring_large", "大半径环岛", 12, 15, "人工确认的大半径持续曲线"
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=sorted(CASES))
    parser.add_argument("--list-cases", action="store_true")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--run", action="store_true", help="explicitly start one bounded test")
    source.add_argument("--text-log", type=pathlib.Path, help="replay a car_live_text_*.log")
    parser.add_argument("--meta-log", type=pathlib.Path, help="optional matching car_live_meta_*.txt")
    parser.add_argument("--duration", type=int, help="override case duration within its safety cap")
    parser.add_argument("--wheel-guard", type=float, default=1.20)
    parser.add_argument("--no-images", action="store_true")
    parser.add_argument("--label", default="", help="run label shown in generated reports")
    parser.add_argument("--note", action="append", default=[], help="operator observation; repeatable")
    parser.add_argument(
        "--thermal-csv",
        type=pathlib.Path,
        help="optional external log: t_s,motor_l_c,motor_r_c,driver_l_c,driver_r_c,battery_c",
    )
    parser.add_argument("--output-dir", type=pathlib.Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--dry-run", action="store_true", help="validate and print the capture command only")
    return parser.parse_args()


def parse_steer(line: str) -> Optional[dict[str, float]]:
    if not line.startswith("steer,"):
        return None
    values: dict[str, float] = {}
    try:
        for item in line.strip().split(",")[1:]:
            key, value = item.split("=", 1)
            values[key] = float(value)
    except (TypeError, ValueError):
        return None
    return values


def read_samples(path: pathlib.Path) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parsed = parse_steer(line)
        if parsed is not None:
            samples.append(parsed)
    if not samples:
        raise ValueError(f"no steer telemetry in {path}")
    return samples


def read_meta(path: Optional[pathlib.Path]) -> dict[str, str]:
    if path is None:
        return {}
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def percentile(values: Iterable[float], q: float) -> Optional[float]:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * q
    low, high = math.floor(position), math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - position) + ordered[high] * (position - low)


def mean_abs(values: Iterable[float]) -> Optional[float]:
    prepared = [abs(float(value)) for value in values]
    return statistics.fmean(prepared) if prepared else None


def ratio(numerator: int, denominator: int) -> Optional[float]:
    return numerator / denominator if denominator else None


def compact_number(value: Optional[float], digits: int = 3) -> Optional[float]:
    return round(value, digits) if value is not None else None


def stable_turn_blocks(samples: list[dict[str, float]], threshold: float = 10.0) -> list[int]:
    """Debounce target-yaw signs so a one-packet fluctuation is not a turn."""
    candidate: Optional[int] = None
    candidate_count = 0
    stable: Optional[int] = None
    blocks: list[int] = []
    for sample in samples:
        target = sample.get("tgt", 0.0)
        if abs(target) < threshold:
            continue
        sign = 1 if target > 0 else -1
        if sign == candidate:
            candidate_count += 1
        else:
            candidate, candidate_count = sign, 1
        if candidate_count >= 3 and sign != stable:
            stable = sign
            blocks.append(sign)
    return blocks


def sample_duration(samples: list[dict[str, float]], index: int) -> float:
    if index == 0:
        return 0.0
    delta = samples[index].get("host_t", 0.0) - samples[index - 1].get("host_t", 0.0)
    # Protect derived thermal-load proxies from a paused PC receiver.
    return delta if 0.0 < delta <= 0.20 else 0.0


def inner_wheel_metrics(samples: list[dict[str, float]]) -> dict[str, Any]:
    overspeed: list[float] = []
    brake_samples = 0
    brake_time_s = 0.0
    brake_pwm_integral = 0.0
    clamp_samples = 0
    candidates = 0
    by_side = {"left": [], "right": []}

    for index, sample in enumerate(samples):
        if not {"L", "R", "encL", "encR", "pwmL", "pwmR"} <= sample.keys():
            continue
        # A meaningful differential target is required before selecting an inner wheel.
        if abs(sample.get("tgt", 0.0)) < 10.0 or abs(sample["L"] - sample["R"]) < 0.08:
            continue
        left_inner = sample["L"] <= sample["R"]
        target = sample["L"] if left_inner else sample["R"]
        actual = sample["encL"] if left_inner else sample["encR"]
        pwm = sample["pwmL"] if left_inner else sample["pwmR"]
        side = "left" if left_inner else "right"
        candidates += 1
        positive = max(0.0, actual - target)
        overspeed.append(positive)
        by_side[side].append(positive)
        if pwm < 0.0:
            duration = sample_duration(samples, index)
            brake_samples += 1
            brake_time_s += duration
            brake_pwm_integral += abs(pwm) * duration
        if pwm <= -1199.5:
            clamp_samples += 1

    return {
        "samples": candidates,
        "overspeed_mae_mps": compact_number(statistics.fmean(overspeed) if overspeed else None),
        "overspeed_p95_mps": compact_number(percentile(overspeed, 0.95)),
        "brake_negative_pwm_fraction": compact_number(ratio(brake_samples, candidates)),
        "brake_time_s": compact_number(brake_time_s),
        "brake_pwm_time_integral": compact_number(brake_pwm_integral, 1),
        "brake_clamp_fraction": compact_number(ratio(clamp_samples, candidates)),
        "left_overspeed_p95_mps": compact_number(percentile(by_side["left"], 0.95)),
        "right_overspeed_p95_mps": compact_number(percentile(by_side["right"], 0.95)),
    }


def parse_temperature(path: Optional[pathlib.Path]) -> dict[str, Any]:
    if path is None:
        return {"status": "NEEDS_EXTERNAL_SENSOR", "points": 0, "values": {}}
    if not path.is_file():
        raise ValueError(f"thermal CSV does not exist: {path}")
    expected_limits = {
        "motor_l_c": 70.0,
        "motor_r_c": 70.0,
        "driver_l_c": 65.0,
        "driver_r_c": 65.0,
        "battery_c": 50.0,
    }
    values: dict[str, list[float]] = {key: [] for key in expected_limits}
    with path.open(newline="", encoding="utf-8-sig") as source:
        for row in csv.DictReader(source):
            for key in expected_limits:
                try:
                    if row.get(key, "") != "":
                        values[key].append(float(row[key]))
                except (TypeError, ValueError):
                    continue
    summaries: dict[str, dict[str, Optional[float]]] = {}
    violations: list[str] = []
    for key, limit in expected_limits.items():
        series = values[key]
        if not series:
            continue
        peak = max(series)
        summaries[key] = {
            "min_c": compact_number(min(series), 1),
            "max_c": compact_number(peak, 1),
            "delta_c": compact_number(peak - min(series), 1),
            "limit_c": limit,
        }
        if peak >= limit:
            violations.append(f"{key}={peak:.1f}C >= {limit:.1f}C")
    if not summaries:
        return {"status": "NEEDS_EXTERNAL_SENSOR", "points": 0, "values": {}}
    return {
        "status": "FAIL" if violations else "PASS",
        "points": sum(len(series) for series in values.values()),
        "values": summaries,
        "violations": violations,
        "source": str(path),
    }


def analyze(case: TestCase, samples: list[dict[str, float]], meta: dict[str, str], thermal: dict[str, Any]) -> dict[str, Any]:
    raw = [sample.get("raw", 0.0) for sample in samples]
    target = [sample.get("tgt", 0.0) for sample in samples]
    gyro = [sample.get("ge", 0.0) for sample in samples if abs(sample.get("tgt", 0.0)) >= 5.0]
    speed_error = [
        (sample.get("encL", 0.0) + sample.get("encR", 0.0)) * 0.5 - sample.get("goal", 0.0)
        for sample in samples
    ]
    wheel_error = [
        abs(sample.get("seL", 0.0)) for sample in samples
    ] + [abs(sample.get("seR", 0.0)) for sample in samples]
    turning = [sample for sample in samples if abs(sample.get("tgt", 0.0)) >= 10.0]
    turn_fraction = ratio(len(turning), len(samples))
    blocks = stable_turn_blocks(samples)
    cross_samples = sum(int(sample.get("rt", 0.0)) in (3, 7) for sample in samples)
    degraded = sum(sample.get("db", 0.0) >= 3.0 for sample in samples)
    full_loss = sum(sample.get("off", 0.0) >= 45.0 for sample in samples)
    max_wheel = max(
        (max(abs(sample.get("encL", 0.0)), abs(sample.get("encR", 0.0))) for sample in samples),
        default=0.0,
    )
    inner = inner_wheel_metrics(samples)

    gates: list[dict[str, str]] = []

    def gate(name: str, passed: bool, detail: str, severity: str = "review") -> None:
        gates.append({"name": name, "status": "PASS" if passed else severity.upper(), "detail": detail})

    guard_reason = meta.get("guard_reason", "")
    post_pgrep = meta.get("post_pgrep", "")
    fallback_events = int(float(meta.get("fallback_events", "0") or 0))
    image_bad = int(float(meta.get("udp_image_bad", "0") or 0))
    gate("capture_guard", not guard_reason, guard_reason or "no motion guard", "fail")
    gate("board_stopped", not post_pgrep, post_pgrep or "main not present after run", "fail")
    gate("imu_fallback", fallback_events == 0, f"fallback_events={fallback_events}", "fail")
    gate("udp_image_integrity", image_bad == 0, f"bad_images={image_bad}", "fail")
    gate("wheel_limit", max_wheel <= 1.20, f"max_wheel={max_wheel:.3f}m/s", "fail")
    gate("telemetry_volume", len(samples) >= 30, f"samples={len(samples)}")

    if case.key == "small_s":
        gate(
            "s_reversals",
            len(blocks) - 1 >= 2,
            f"stable_target_blocks={blocks}; reversals={max(0, len(blocks) - 1)}",
        )
    elif case.key == "straight_lt5m":
        gate(
            "straight_dominance",
            turn_fraction is not None and turn_fraction <= 0.35,
            f"turn_fraction={turn_fraction!r}",
        )
    elif case.key.startswith("u_turn"):
        dominant = max(blocks.count(-1), blocks.count(1)) / len(blocks) if blocks else 0.0
        gate(
            "u_turn_sustained",
            turn_fraction is not None and turn_fraction >= 0.45 and dominant >= 0.50,
            f"turn_fraction={turn_fraction!r}; target_blocks={blocks}",
        )
    elif case.key == "cross":
        gate(
            "cross_element_disabled",
            cross_samples == 0,
            f"cross_samples={cross_samples}; expected no element classification",
            "fail",
        )
        gate(
            "line_retention",
            full_loss == 0,
            f"full_line_loss_samples={full_loss}",
            "fail",
        )
    else:  # ring identity is supplied by the explicit manual case label.
        gate(
            "ring_curve_coverage",
            turn_fraction is not None and turn_fraction >= 0.40,
            f"turn_fraction={turn_fraction!r}; rt ring detection is intentionally not required",
        )

    gate(
        "wheel_tracking",
        mean_abs(wheel_error) is not None and mean_abs(wheel_error) <= 0.09 and percentile(wheel_error, 0.95) <= 0.25,
        f"wheel_mae={mean_abs(wheel_error)!r}; wheel_p95={percentile(wheel_error, 0.95)!r}",
    )
    if turning:
        gyro_p95 = percentile((abs(value) for value in gyro), 0.95)
        gate(
            "gyro_tracking",
            mean_abs(gyro) is not None and mean_abs(gyro) <= 7.0 and gyro_p95 is not None and gyro_p95 <= 15.0,
            f"gyro_mae={mean_abs(gyro)!r}; gyro_p95={gyro_p95!r}",
        )
        gate(
            "inner_brake_control",
            inner["overspeed_p95_mps"] is not None and inner["overspeed_p95_mps"] <= 0.35,
            f"inner_p95={inner['overspeed_p95_mps']!r}; clamp={inner['brake_clamp_fraction']!r}",
        )

    gate(
        "thermal_measurement",
        thermal["status"] == "PASS",
        thermal["status"],
        "fail" if thermal["status"] == "FAIL" else "review",
    )

    statuses = {item["status"] for item in gates}
    verdict = "FAIL" if "FAIL" in statuses else "PASS" if statuses == {"PASS"} else "REVIEW"
    return {
        "case": asdict(case),
        "verdict": verdict,
        "sample_count": len(samples),
        "metrics": {
            "raw_abs_p95_px": compact_number(percentile((abs(value) for value in raw), 0.95)),
            "turn_fraction": compact_number(turn_fraction),
            "target_turn_blocks": blocks,
            "cross_samples": cross_samples,
            "vision_degraded_fraction": compact_number(ratio(degraded, len(samples))),
            "full_line_loss_samples": full_loss,
            "max_wheel_mps": compact_number(max_wheel),
            "avg_speed_error_mae_mps": compact_number(mean_abs(speed_error)),
            "wheel_error_mae_mps": compact_number(mean_abs(wheel_error)),
            "wheel_error_p95_mps": compact_number(percentile(wheel_error, 0.95)),
            "gyro_error_mae_dps": compact_number(mean_abs(gyro)),
            "gyro_error_p95_dps": compact_number(percentile((abs(value) for value in gyro), 0.95)),
            "inner_brake": inner,
        },
        "thermal": thermal,
        "gates": gates,
    }


def capture_once(args: argparse.Namespace, case: TestCase, duration: int) -> tuple[pathlib.Path, pathlib.Path]:
    command = [
        sys.executable,
        str(CAPTURE),
        str(duration),
        "--wheel-guard",
        str(args.wheel_guard),
    ]
    if args.no_images:
        command.append("--no-images")
    if args.dry_run:
        print("[TRACK][DRY-RUN] " + " ".join(command))
        raise SystemExit(0)
    print(f"[TRACK] case={case.key} duration={duration}s; using existing bounded capture guards", flush=True)
    process = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    print(process.stdout, end="")
    meta_matches = re.findall(r"^META=(.+)$", process.stdout, re.MULTILINE)
    if not meta_matches:
        raise RuntimeError("capture did not report a META path")
    meta = pathlib.Path(meta_matches[-1])
    values = read_meta(meta)
    text = pathlib.Path(values["text_log"])
    if not text.is_file() or not meta.is_file():
        raise RuntimeError("capture meta did not reference usable log files")
    return text, meta


def make_report_markdown(result: dict[str, Any], label: str, notes: list[str], source: pathlib.Path) -> str:
    metrics = result["metrics"]
    inner = metrics["inner_brake"]
    lines = [
        f"# 赛道特征验证：{result['case']['title']}",
        "",
        f"- 标签：`{label}`",
        f"- 判定：**{result['verdict']}**",
        f"- 遥测来源：`{source}`",
        f"- 样本数：{result['sample_count']}",
        "",
        "## 主要指标",
        "",
        f"- 目标转向块：{metrics['target_turn_blocks']}",
        f"- 转向帧占比：{metrics['turn_fraction']}",
        f"- 陀螺误差 MAE/p95：{metrics['gyro_error_mae_dps']} / {metrics['gyro_error_p95_dps']} dps",
        f"- 双轮速度误差 MAE/p95：{metrics['wheel_error_mae_mps']} / {metrics['wheel_error_p95_mps']} m/s",
        f"- 内轮超速 p95：{inner['overspeed_p95_mps']} m/s；制动顶限比例：{inner['brake_clamp_fraction']}",
        f"- 负 PWM 时间：{inner['brake_time_s']} s；负 PWM 时间积分（热负荷代理）：{inner['brake_pwm_time_integral']}",
        f"- 视觉降级占比：{metrics['vision_degraded_fraction']}",
        "",
        "## 门禁",
        "",
    ]
    for item in result["gates"]:
        lines.append(f"- `{item['status']}` {item['name']}: {item['detail']}")
    if notes:
        lines.extend(["", "## 人工观察", ""])
        lines.extend(f"- {note}" for note in notes)
    lines.extend(
        [
            "",
            "温升结论只在提供外部温度 CSV 后有效；没有温度数据时，负 PWM 时间积分仅是热负荷代理。",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    if args.list_cases:
        for case in CASES.values():
            print(f"{case.key:16} {case.duration_s:2d}s  {case.title}：{case.expected}")
        return 0
    if not args.case:
        raise SystemExit("--case is required unless --list-cases is used")
    if not args.run and args.text_log is None:
        raise SystemExit("choose --run for one bounded live test, or --text-log for offline replay")

    case = CASES[args.case]
    duration = args.duration if args.duration is not None else case.duration_s
    if duration < 5 or duration > case.max_duration_s:
        raise SystemExit(f"{case.key} duration must be in [5, {case.max_duration_s}] seconds")
    if args.wheel_guard < 0.1 or args.wheel_guard > 5.0:
        raise SystemExit("wheel guard must be in [0.1, 5.0] m/s")

    if args.run:
        text_log, meta_log = capture_once(args, case, duration)
    else:
        text_log = args.text_log
        meta_log = args.meta_log
        if not text_log.is_file():
            raise SystemExit(f"text log does not exist: {text_log}")

    samples = read_samples(text_log)
    meta = read_meta(meta_log)
    thermal = parse_temperature(args.thermal_csv)
    result = analyze(case, samples, meta, thermal)
    result.update(
        {
            "label": args.label or case.key,
            "notes": args.note,
            "text_log": str(text_log),
            "meta_log": str(meta_log) if meta_log else "",
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
    )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    safe_label = re.sub(r"[^A-Za-z0-9_.-]+", "_", args.label or case.key).strip("_")
    output = args.output_dir / f"{timestamp}_{safe_label}"
    output.mkdir(parents=True, exist_ok=False)
    summary_json = output / "summary.json"
    summary_md = output / "summary.md"
    summary_json.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary_md.write_text(
        make_report_markdown(result, result["label"], args.note, text_log), encoding="utf-8"
    )

    print(f"[TRACK] verdict={result['verdict']}")
    print(f"[TRACK] summary={summary_md}")
    return 2 if result["verdict"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
