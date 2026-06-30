#!/usr/bin/env python3
"""Analyze SmartCar speed_tune UDP logs and suggest conservative PI changes."""

from __future__ import annotations

import argparse
import math
import statistics
from pathlib import Path

NUMERIC_FIELDS = {
    "t_ms", "target", "desired", "encL", "encR", "rawL", "rawR",
    "valid", "badMs", "errL", "errR", "pwmL", "pwmR", "dist",
    "fwd", "ret", "kp", "ki", "maxV",
}


def parse_line(line: str) -> dict[str, object] | None:
    line = line.strip()
    if not line.startswith("speed_tune,"):
        return None
    row: dict[str, object] = {}
    for part in line.split(","):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        if key in NUMERIC_FIELDS:
            try:
                row[key] = float(value)
            except ValueError:
                row[key] = math.nan
        else:
            row[key] = value
    return row if "t_ms" in row else None


def load_rows(path: Path) -> list[dict[str, object]]:
    text = path.read_text(errors="ignore")
    rows = [row for line in text.splitlines() if (row := parse_line(line))]
    rows.sort(key=lambda item: float(item.get("t_ms", 0.0)))
    return rows


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else math.nan


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * pct)))
    return ordered[idx]


def rmse(values: list[float]) -> float:
    return math.sqrt(mean([value * value for value in values])) if values else math.nan


def finite(value: float) -> str:
    return "nan" if math.isnan(value) else f"{value:.3f}"


def suggest(rows: list[dict[str, object]]) -> tuple[str, float, float]:
    kp = float(rows[-1].get("kp", 1800.0)) if rows else 1800.0
    ki = float(rows[-1].get("ki", 35.0)) if rows else 35.0
    forward = [r for r in rows if r.get("phase") == "forward"]
    valid_forward = [r for r in forward if float(r.get("valid", 0.0)) >= 1.0]

    if not forward:
        return "no_forward_data_keep_params", kp, ki
    if len(valid_forward) < max(8, len(forward) * 0.7):
        return "encoder_invalid_do_not_tune_pi", kp, ki

    avg_speeds = [0.5 * (float(r["encL"]) + float(r["encR"])) for r in valid_forward]
    targets = [float(r["target"]) for r in valid_forward]
    errors = [target - speed for target, speed in zip(targets, avg_speeds)]
    peak_target = max(targets)
    peak_speed = max(avg_speeds)
    steady = valid_forward[len(valid_forward) // 2:]
    steady_errors = [
        float(r["target"]) - 0.5 * (float(r["encL"]) + float(r["encR"]))
        for r in steady
    ]
    overshoot = peak_speed - peak_target
    mean_steady_error = mean(steady_errors)
    error_rmse = rmse(errors)
    saturated = [
        r for r in valid_forward
        if abs(float(r.get("pwmL", 0.0))) >= 0.95 * 4200.0
        or abs(float(r.get("pwmR", 0.0))) >= 0.95 * 4200.0
    ]

    next_kp = kp
    next_ki = ki
    reason = "keep_params"
    if overshoot > 0.12 or (error_rmse > 0.22 and mean_steady_error < -0.05):
        next_kp = max(600.0, kp * 0.82)
        next_ki = max(0.0, ki * 0.75)
        reason = "reduce_kp_ki_overshoot"
    elif mean_steady_error > 0.12 and len(saturated) < len(valid_forward) * 0.2:
        next_kp = min(2600.0, kp * 1.10)
        next_ki = min(90.0, ki * 1.15)
        reason = "increase_kp_ki_lag"
    elif len(saturated) >= len(valid_forward) * 0.35:
        next_kp = max(600.0, kp * 0.90)
        next_ki = max(0.0, ki * 0.85)
        reason = "reduce_pi_pwm_saturation"

    return reason, round(next_kp, 1), round(next_ki, 1)


def analyze(path: Path) -> str:
    rows = load_rows(path)
    forward = [r for r in rows if r.get("phase") == "forward"]
    valid_forward = [r for r in forward if float(r.get("valid", 0.0)) >= 1.0]
    reasons = [str(r.get("reason", "")) for r in rows if r.get("phase") in {"done", "cooldown"}]
    reason, next_kp, next_ki = suggest(rows)

    avg_speeds = [0.5 * (float(r["encL"]) + float(r["encR"])) for r in valid_forward]
    targets = [float(r["target"]) for r in valid_forward]
    errors = [target - speed for target, speed in zip(targets, avg_speeds)]
    wheel_mismatch = [
        abs(float(r["encL"]) - float(r["encR"]))
        for r in valid_forward
        if "encL" in r and "encR" in r
    ]
    pwm_abs = [
        max(abs(float(r.get("pwmL", 0.0))), abs(float(r.get("pwmR", 0.0))))
        for r in forward
    ]
    raw_abs = [
        max(abs(float(r.get("rawL", 0.0))), abs(float(r.get("rawR", 0.0))))
        for r in forward
    ]

    lines = [
        f"log={path}",
        f"rows={len(rows)} forward_rows={len(forward)} valid_forward_rows={len(valid_forward)}",
        f"valid_forward_ratio={len(valid_forward) / len(forward):.3f}" if forward else "valid_forward_ratio=nan",
        f"target_peak={finite(max(targets) if targets else math.nan)} speed_peak={finite(max(avg_speeds) if avg_speeds else math.nan)}",
        f"mean_error={finite(mean(errors))} rmse_error={finite(rmse(errors))}",
        f"p95_wheel_mismatch={finite(percentile(wheel_mismatch, 0.95))}",
        f"p95_pwm_abs={finite(percentile(pwm_abs, 0.95))} p95_raw_abs={finite(percentile(raw_abs, 0.95))}",
        f"final_reason={reasons[-1] if reasons else 'unknown'}",
        f"suggest_reason={reason}",
        f"suggest_kSpeedPidP={next_kp:.1f}",
        f"suggest_kSpeedPidI={next_ki:.1f}",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    print(analyze(args.log))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())