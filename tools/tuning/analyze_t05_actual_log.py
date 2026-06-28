#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conservative T05 speed-loop log analyzer and iterative PID tuner.

This tool is designed for the current SmartCar T05 log format:
  >>> [执行] min=... max=... period=... duration=... Kp=... Ki=...
  T05_ACTUAL t_ms,target,encL,encR,errL,errR,pwmL,pwmR

It deliberately proposes one small, reversible candidate per validated run.
It never treats a noisy, saturated or mechanically asymmetric run as evidence
that global speed PID gains should be increased.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import math
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

EXEC_RE = re.compile(
    r"\[执行\].*?min=([\d.]+).*?max=([\d.]+).*?period=([\d.]+)s.*?duration=(\d+)ms.*?Kp=([\d.]+).*?Ki=([\d.]+)"
)
PID_RE = re.compile(r"\[T05\]\s*当前\s*Kp=([\d.]+)/([\d.]+)\s+Ki=([\d.]+)/([\d.]+)")
DATA_RE = re.compile(
    r"^T05_ACTUAL\s+\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*$"
)


@dataclass
class Sample:
    t_ms: float
    target: float
    enc_l: float
    enc_r: float
    err_l: float
    err_r: float
    pwm_l: float
    pwm_r: float


@dataclass
class Run:
    run_id: int
    kp: Optional[float] = None
    ki: Optional[float] = None
    min_speed: Optional[float] = None
    max_speed: Optional[float] = None
    period_s: Optional[float] = None
    duration_ms: Optional[int] = None
    samples: List[Sample] = field(default_factory=list)


@dataclass
class WheelMetrics:
    mae: float
    rmse: float
    p95_abs_error: float
    high_bias: float
    low_bias: float
    max_overshoot: float
    lag_ms: Optional[float]
    saturation_rate: float
    pwm_jitter_per_20ms: float
    error_sign_changes: int


@dataclass
class RunMetrics:
    score: float
    status: str
    reasons: List[str]
    wheel_l: WheelMetrics
    wheel_r: WheelMetrics
    time_gap_p95_ms: float
    duration_ms: float
    left_right_speed_mae: float
    valid: bool


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def percentile(values: Sequence[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    idx = (len(ordered) - 1) * q
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (idx - lo)


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(mean([v * v for v in values])) if values else 0.0


def parse_runs(path: Path) -> List[Run]:
    if not path.exists():
        raise FileNotFoundError(path)
    text = path.read_text(encoding="utf-8", errors="ignore")
    runs: List[Run] = []
    active: Optional[Run] = None
    current_pid: Tuple[Optional[float], Optional[float]] = (None, None)

    def ensure_active() -> Run:
        nonlocal active
        if active is None:
            active = Run(run_id=len(runs) + 1, kp=current_pid[0], ki=current_pid[1])
            runs.append(active)
        return active

    for raw_line in text.splitlines():
        line = raw_line.strip()
        match = EXEC_RE.search(line)
        if match:
            active = Run(
                run_id=len(runs) + 1,
                min_speed=float(match.group(1)),
                max_speed=float(match.group(2)),
                period_s=float(match.group(3)),
                duration_ms=int(match.group(4)),
                kp=float(match.group(5)),
                ki=float(match.group(6)),
            )
            current_pid = (active.kp, active.ki)
            runs.append(active)
            continue

        pid = PID_RE.search(line)
        if pid:
            current_pid = (float(pid.group(1)), float(pid.group(3)))
            if active and not active.samples:
                active.kp, active.ki = current_pid
            continue

        data = DATA_RE.match(line)
        if data:
            run = ensure_active()
            run.samples.append(Sample(*[float(data.group(i)) for i in range(1, 9)]))

    cleaned: List[Run] = []
    for run in runs:
        run.samples.sort(key=lambda s: s.t_ms)
        if run.samples:
            if run.min_speed is None:
                run.min_speed = min(s.target for s in run.samples)
            if run.max_speed is None:
                run.max_speed = max(s.target for s in run.samples)
            cleaned.append(run)
    return cleaned


def interpolate(times: Sequence[float], values: Sequence[float], query: float) -> Optional[float]:
    if not times or query < times[0] or query > times[-1]:
        return None
    idx = bisect.bisect_left(times, query)
    if idx == 0:
        return values[0]
    if idx >= len(times):
        return values[-1]
    t0, t1 = times[idx - 1], times[idx]
    if t1 <= t0:
        return values[idx]
    ratio = (query - t0) / (t1 - t0)
    return values[idx - 1] + (values[idx] - values[idx - 1]) * ratio


def estimate_positive_lag_ms(times: Sequence[float], target: Sequence[float], actual: Sequence[float]) -> Optional[float]:
    """Estimate response delay using actual timestamps, not assumed line spacing."""
    if len(times) < 40 or times[-1] - times[0] < 1200:
        return None
    best_lag: Optional[float] = None
    best_error = float("inf")
    for lag in range(0, 401, 5):
        errors: List[float] = []
        for t, desired in zip(times, target):
            observed = interpolate(times, actual, t + lag)
            if observed is not None:
                errors.append(desired - observed)
        if len(errors) < len(times) * 0.70:
            continue
        candidate = rmse(errors)
        if candidate < best_error:
            best_error = candidate
            best_lag = float(lag)
    return best_lag


def normalized_pwm_jitter(times: Sequence[float], pwm: Sequence[float]) -> float:
    if len(times) < 3:
        return 0.0
    normalized: List[float] = []
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if 1.0 <= dt <= 200.0:
            normalized.append(abs(pwm[i] - pwm[i - 1]) * 20.0 / dt)
    return percentile(normalized, 0.75) if normalized else 0.0


def count_sign_changes(errors: Sequence[float], deadband: float = 0.025) -> int:
    last = 0
    changes = 0
    for error in errors:
        sign = 1 if error > deadband else -1 if error < -deadband else 0
        if sign and last and sign != last:
            changes += 1
        if sign:
            last = sign
    return changes


def wheel_metrics(samples: Sequence[Sample], side: str, max_pwm: float) -> WheelMetrics:
    times = [s.t_ms for s in samples]
    target = [s.target for s in samples]
    actual = [s.enc_l if side == "L" else s.enc_r for s in samples]
    pwm = [s.pwm_l if side == "L" else s.pwm_r for s in samples]
    errors = [t - a for t, a in zip(target, actual)]
    lo, hi = min(target), max(target)
    amplitude = max(hi - lo, 1e-6)
    high = [e for t, e in zip(target, errors) if t >= hi - amplitude * 0.15]
    low = [e for t, e in zip(target, errors) if t <= lo + amplitude * 0.15]
    overshoot = max(0.0, max(actual) - hi)
    sat = mean([1.0 if abs(p) >= max_pwm * 0.95 else 0.0 for p in pwm])
    return WheelMetrics(
        mae=mean([abs(e) for e in errors]),
        rmse=rmse(errors),
        p95_abs_error=percentile([abs(e) for e in errors], 0.95),
        high_bias=mean(high),
        low_bias=mean(low),
        max_overshoot=overshoot,
        lag_ms=estimate_positive_lag_ms(times, target, actual),
        saturation_rate=sat,
        pwm_jitter_per_20ms=normalized_pwm_jitter(times, pwm),
        error_sign_changes=count_sign_changes(errors),
    )


def validate_and_score(run: Run, max_pwm: float) -> RunMetrics:
    samples = run.samples
    wm_l = wheel_metrics(samples, "L", max_pwm)
    wm_r = wheel_metrics(samples, "R", max_pwm)
    times = [s.t_ms for s in samples]
    gaps = [times[i] - times[i - 1] for i in range(1, len(times))]
    duration = times[-1] - times[0] if len(times) > 1 else 0.0
    gap_p95 = percentile(gaps, 0.95) if gaps else 0.0
    speed_delta = mean([abs(s.enc_l - s.enc_r) for s in samples])
    target_range = (run.max_speed or 0.0) - (run.min_speed or 0.0)
    reasons: List[str] = []
    valid = True

    if len(samples) < 100:
        valid = False
        reasons.append(f"有效样本仅 {len(samples)} 条；至少需要 100 条（约 2 秒）才允许给出 PID 候选。")
    if duration < 2500:
        valid = False
        reasons.append(f"有效日志时长 {duration:.0f} ms 过短；至少运行 2500 ms。")
    if target_range < 0.15:
        valid = False
        reasons.append(f"目标速度摆幅仅 {target_range:.3f} m/s；应至少 0.15 m/s，才能判断动态响应。")
    if gap_p95 > 60:
        valid = False
        reasons.append(f"日志时间间隔 P95 为 {gap_p95:.0f} ms，终端输出或调度干扰过大；不要据此改 PID。")

    avg_mae = (wm_l.mae + wm_r.mae) * 0.5
    avg_rmse = (wm_l.rmse + wm_r.rmse) * 0.5
    avg_lag_values = [x for x in (wm_l.lag_ms, wm_r.lag_ms) if x is not None]
    avg_lag = mean(avg_lag_values) if avg_lag_values else 400.0
    avg_sat = (wm_l.saturation_rate + wm_r.saturation_rate) * 0.5
    avg_jitter = (wm_l.pwm_jitter_per_20ms + wm_r.pwm_jitter_per_20ms) * 0.5
    avg_overshoot = (wm_l.max_overshoot + wm_r.max_overshoot) * 0.5

    penalty = (
        min(28.0, avg_mae / 0.12 * 20.0)
        + min(18.0, avg_rmse / 0.16 * 12.0)
        + min(18.0, max(0.0, avg_lag - 60.0) / 180.0 * 18.0)
        + min(18.0, avg_sat * 90.0)
        + min(10.0, speed_delta / 0.12 * 8.0)
        + min(10.0, avg_overshoot / 0.12 * 8.0)
    )
    score = max(0.0, 100.0 - penalty)

    if not valid:
        status = "DATA_INVALID"
    elif avg_sat >= 0.15:
        status = "PWM_SATURATED"
        reasons.append(f"平均 PWM 饱和率 {avg_sat * 100:.1f}%：当前是动力/供电/机械或目标速度上限问题，不允许增大 PID。")
    elif speed_delta > 0.12:
        status = "LEFT_RIGHT_ASYMMETRY"
        reasons.append(f"左右轮平均速度差 {speed_delta:.3f} m/s：先处理轮胎、传动、编码器或左右补偿，不建议用全局 PID 掩盖。")
    elif avg_overshoot > 0.10 or avg_jitter > 850:
        status = "OSCILLATORY"
        reasons.append(f"检测到过冲/输出抖动（过冲 {avg_overshoot:.3f} m/s，PWM P75 抖动 {avg_jitter:.0f}/20ms）。")
    elif avg_lag > 150 or (wm_l.high_bias + wm_r.high_bias) * 0.5 > 0.08:
        status = "SLUGGISH"
        reasons.append(f"动态跟随偏慢（平均估计滞后 {avg_lag:.0f} ms，高速段平均欠速 {(wm_l.high_bias + wm_r.high_bias) * 0.5:.3f} m/s）。")
    elif score >= 85 and avg_mae <= 0.06 and avg_lag <= 100:
        status = "WELL_TUNED"
        reasons.append("误差、滞后和饱和均处于可接受范围，保留当前参数。")
    else:
        status = "ACCEPTABLE"
        reasons.append("速度环可用，但继续以单变量、小步方式优化。")

    return RunMetrics(score, status, reasons, wm_l, wm_r, gap_p95, duration, speed_delta, valid)


def recommend(run: Run, metrics: RunMetrics) -> Tuple[float, float, str]:
    kp = run.kp if run.kp is not None else 1800.0
    ki = run.ki if run.ki is not None else 35.0
    if not metrics.valid:
        return kp, ki, "数据不足：保持当前值，不生成候选。"

    left, right = metrics.wheel_l, metrics.wheel_r
    avg_lag = mean([x for x in (left.lag_ms, right.lag_ms) if x is not None])
    avg_sat = (left.saturation_rate + right.saturation_rate) * 0.5
    avg_high_bias = (left.high_bias + right.high_bias) * 0.5
    avg_overshoot = (left.max_overshoot + right.max_overshoot) * 0.5
    avg_jitter = (left.pwm_jitter_per_20ms + right.pwm_jitter_per_20ms) * 0.5

    if metrics.status in {"PWM_SATURATED", "LEFT_RIGHT_ASYMMETRY"}:
        return kp, ki, "硬件/动力边界未消除：禁止改变全局 PID。"
    if metrics.status == "OSCILLATORY":
        return round(max(500.0, kp * 0.92), 1), round(max(5.0, ki * 0.92), 1), "抑制过冲：P、I 同时小幅下调 8%。"
    if metrics.status == "SLUGGISH":
        if avg_lag > 150 and avg_sat < 0.05:
            return round(min(5000.0, kp * 1.08), 1), ki, "响应滞后且未饱和：仅上调 P 8%，保持 I 不变。"
        if avg_high_bias > 0.08 and avg_sat < 0.05:
            return kp, round(min(150.0, ki * 1.10), 1), "高速段持续欠速且未饱和：仅上调 I 10%。"
    if avg_overshoot > 0.06 or avg_jitter > 650:
        return round(max(500.0, kp * 0.95), 1), ki, "有轻微过冲/抖动：仅下调 P 5%。"
    if metrics.status == "WELL_TUNED":
        return kp, ki, "当前评分已达通过线：保持当前参数。"
    return kp, ki, "未满足明确单变量调整条件：保持当前值并补采两轮可比日志。"


def history_fields() -> List[str]:
    return [
        "created_at", "log_file", "run_id", "min_speed", "max_speed", "period_s", "duration_ms",
        "kp", "ki", "score", "status", "mae_l", "mae_r", "rmse_l", "rmse_r", "lag_l_ms", "lag_r_ms",
        "sat_l", "sat_r", "speed_asymmetry", "recommended_kp", "recommended_ki",
    ]


def append_history(path: Path, log_file: Path, run: Run, metrics: RunMetrics, rec_kp: float, rec_ki: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=history_fields())
        if new_file:
            writer.writeheader()
        writer.writerow({
            "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "log_file": str(log_file), "run_id": run.run_id,
            "min_speed": run.min_speed, "max_speed": run.max_speed, "period_s": run.period_s,
            "duration_ms": run.duration_ms, "kp": run.kp, "ki": run.ki,
            "score": f"{metrics.score:.3f}", "status": metrics.status,
            "mae_l": f"{metrics.wheel_l.mae:.4f}", "mae_r": f"{metrics.wheel_r.mae:.4f}",
            "rmse_l": f"{metrics.wheel_l.rmse:.4f}", "rmse_r": f"{metrics.wheel_r.rmse:.4f}",
            "lag_l_ms": metrics.wheel_l.lag_ms, "lag_r_ms": metrics.wheel_r.lag_ms,
            "sat_l": f"{metrics.wheel_l.saturation_rate:.4f}", "sat_r": f"{metrics.wheel_r.saturation_rate:.4f}",
            "speed_asymmetry": f"{metrics.left_right_speed_mae:.4f}",
            "recommended_kp": rec_kp, "recommended_ki": rec_ki,
        })


def best_history(path: Path, run: Run) -> Optional[Dict[str, str]]:
    if not path.exists():
        return None
    candidates: List[Dict[str, str]] = []
    with path.open("r", newline="", encoding="utf-8") as fp:
        for row in csv.DictReader(fp):
            try:
                same_band = abs(float(row["min_speed"]) - float(run.min_speed or 0.0)) <= 0.05 and abs(float(row["max_speed"]) - float(run.max_speed or 0.0)) <= 0.05
                if same_band and row.get("status") not in {"DATA_INVALID", "PWM_SATURATED", "LEFT_RIGHT_ASYMMETRY"}:
                    candidates.append(row)
            except (TypeError, ValueError):
                continue
    return max(candidates, key=lambda row: float(row["score"])) if candidates else None


def fmt_optional(value: Optional[float], digits: int = 1) -> str:
    return "N/A" if value is None else f"{value:.{digits}f}"


def print_report(log_file: Path, run: Run, metrics: RunMetrics, rec_kp: float, rec_ki: float, reason: str, best: Optional[Dict[str, str]]) -> None:
    left, right = metrics.wheel_l, metrics.wheel_r
    print("\n================ T05 速度环可替换候选报告 ================")
    print(f"日志: {log_file} | 执行段: #{run.run_id} | 样本: {len(run.samples)} | 时长: {metrics.duration_ms:.0f} ms")
    print(f"当前: Kp={run.kp if run.kp is not None else '未知'}  Ki={run.ki if run.ki is not None else '未知'} | 目标: {run.min_speed:.2f}~{run.max_speed:.2f} m/s")
    print(f"状态: {metrics.status} | 综合评分: {metrics.score:.1f}/100 | 日志间隔P95: {metrics.time_gap_p95_ms:.0f} ms")
    print("轮速指标:"
          f"\n  左: MAE={left.mae:.3f} RMSE={left.rmse:.3f} P95={left.p95_abs_error:.3f} lag={fmt_optional(left.lag_ms)}ms sat={left.saturation_rate*100:.1f}%"
          f"\n  右: MAE={right.mae:.3f} RMSE={right.rmse:.3f} P95={right.p95_abs_error:.3f} lag={fmt_optional(right.lag_ms)}ms sat={right.saturation_rate*100:.1f}%")
    print(f"左右轮平均速度差: {metrics.left_right_speed_mae:.3f} m/s")
    for reason_line in metrics.reasons:
        print(f"- {reason_line}")
    print(f"\n本轮策略: {reason}")
    print("[DIRECT_REPLACE_SPEED_PID]")
    print(f"constexpr float kSpeedPidP = {rec_kp:.1f}f;")
    print(f"constexpr float kSpeedPidI = {rec_ki:.1f}f;")
    print("说明: 仅在本报告状态不是 DATA_INVALID/PWM_SATURATED/LEFT_RIGHT_ASYMMETRY 时，才允许将以上两行写入 car_config.hpp。")
    if best:
        print(f"\n历史最优（相同速度区间）: score={float(best['score']):.1f}, Kp={best['kp']}, Ki={best['ki']}, status={best['status']}。")


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze one T05 execution segment and output a conservative, copy-paste PID candidate.")
    parser.add_argument("log_file", type=Path)
    parser.add_argument("--run", type=int, default=-1, help="1-based execution segment; -1 selects the latest segment.")
    parser.add_argument("--max-pwm", type=float, default=8000.0)
    parser.add_argument("--history", type=Path, default=Path("t05_tuning_history.csv"))
    parser.add_argument("--no-history", action="store_true")
    args = parser.parse_args()

    try:
        runs = parse_runs(args.log_file)
    except FileNotFoundError:
        print(f"错误: 找不到日志文件: {args.log_file}", file=sys.stderr)
        return 2
    if not runs:
        print("错误: 没有找到 T05_ACTUAL 数据。", file=sys.stderr)
        return 2
    index = len(runs) - 1 if args.run == -1 else args.run - 1
    if not 0 <= index < len(runs):
        print(f"错误: --run 必须在 1..{len(runs)} 或 -1。", file=sys.stderr)
        return 2

    run = runs[index]
    metrics = validate_and_score(run, args.max_pwm)
    rec_kp, rec_ki, rec_reason = recommend(run, metrics)
    if not args.no_history:
        append_history(args.history, args.log_file, run, metrics, rec_kp, rec_ki)
    best = best_history(args.history, run) if not args.no_history else None
    print_report(args.log_file, run, metrics, rec_kp, rec_ki, rec_reason, best)
    return 0 if metrics.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
