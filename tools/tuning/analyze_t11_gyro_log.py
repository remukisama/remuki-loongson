#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T11 gyro-loop log analyzer for the current SmartCar format.

It supports both parameter snapshot forms:
  pos=1.00/neg=1.00
  pos/neg=1.02/1.08

The program refuses to emit a replacement candidate when the snapshot, IMU,
direction, differential authority, or wheel-speed prerequisites are invalid.
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

SNAP = re.compile(
    r"\[参数快照\]\s*gyro\(P/I/D=([-\d.]+)/([-\d.]+)/([-\d.]+)\s+ff=([-\d.]+)\s+out=([-\d.]+)\s+dOut=([-\d.]+)/([-\d.]+)\s+(?:pos/neg|pos)=([-\d.]+)/(?:neg=)?([-\d.]+)\s+dead=([-\d.]+)\s+lpf=([-\d.]+)\s+maxDiffRatio=([-\d.]+)\)"
)
STEP = re.compile(
    r"\[T11_STEP_SUMMARY\]\s+idx=(\d+)\s+target=([-\d.]+)\s+cmd=([-\d.]+)\s+samples=(\d+)\s+read_fail=(\d+)\s+gf_mean=([-\d.]+)\s+gf_abs=([-\d.]+)\s+real_abs=([-\d.]+)\s+err_abs=([-\d.]+)\s+late_err_abs=([-\d.]+)\s+diff_abs=([-\d.]+)\s+cap_rate=([-\d.]+)\s+reverse_rate=([-\d.]+)\s+speed_err_abs=([-\d.]+)/([-\d.]+)\s+enc_abs=([-\d.]+)/([-\d.]+)"
)
SUMMARY = re.compile(
    r"\[T11_SUMMARY\]\s+samples=(\d+)\s+read_fail=(\d+)\s+fail_rate=([-\d.]+)\s+cap_rate=([-\d.]+)\s+reverse_rate=([-\d.]+)\s+weak_track_steps=(\d+)\s+cap_steps=(\d+)\s+speed_weak_steps=(\d+)\s+reverse_steps=(\d+)\s+base_speed=([-\d.]+)\s+hold_ms=(\d+)"
)
POINT = re.compile(
    r"\[T11\]\s+idx=(\d+)\s+t=([-\d.]+)\s+target=\s*([-\d.]+)\s+cmd=\s*([-\d.]+)\s+real=\s*([-\d.]+)\s+gf=\s*([-\d.]+)\s+err=\s*([-\d.]+)\s+diff0=\s*([-\d.]+)\s+diff=\s*([-\d.]+)\s+cap=(\d+)"
)


def avg(values):
    return sum(values) / len(values) if values else 0.0


def parse(path: Path):
    if not path.exists():
        raise FileNotFoundError(path)
    params = {}
    steps, points, summary = [], [], {}
    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        m = SNAP.search(line)
        if m:
            keys = ("kp", "ki", "kd", "ff", "out", "dout", "dout_reverse", "pos", "neg", "dead", "lpf", "max_ratio")
            params = {key: float(m.group(i + 1)) for i, key in enumerate(keys)}
            continue
        m = STEP.search(line)
        if m:
            steps.append({
                "idx": int(m.group(1)), "target": float(m.group(2)), "cmd": float(m.group(3)),
                "samples": int(m.group(4)), "read_fail": int(m.group(5)),
                "late": float(m.group(10)), "cap": float(m.group(12)), "reverse": float(m.group(13)),
                "se_l": float(m.group(14)), "se_r": float(m.group(15)),
            })
            continue
        m = SUMMARY.search(line)
        if m:
            keys = ("samples", "read_fail", "fail_rate", "cap_rate", "reverse_rate", "weak_track_steps", "cap_steps", "speed_weak_steps", "reverse_steps", "base_speed", "hold_ms")
            summary = {key: float(m.group(i + 1)) for i, key in enumerate(keys)}
            continue
        m = POINT.search(line)
        if m:
            points.append({"idx": int(m.group(1)), "t": float(m.group(2)), "cmd": float(m.group(4)), "gf": float(m.group(6)), "err": float(m.group(7)), "diff0": float(m.group(8)), "diff": float(m.group(9)), "cap": int(m.group(10))})
    return params, steps, points, summary


def dynamics(step, points):
    ps = [p for p in points if p["idx"] == step["idx"]]
    if abs(step["cmd"]) < 5 or not ps:
        return 0.0, None
    command = step["cmd"]
    peak = max(p["gf"] for p in ps) if command > 0 else min(p["gf"] for p in ps)
    overshoot = max(0.0, (peak - command) if command > 0 else (command - peak)) / abs(command) * 100.0
    start = ps[0]["t"]
    rise = next((p["t"] - start for p in ps if abs(p["gf"]) >= abs(command) * 0.9), None)
    return overshoot, None if rise is None else rise * 1000.0


def assess(params, steps, points, summary):
    needed = ("kp", "kd", "ff", "pos", "neg")
    if any(k not in params for k in needed):
        return "CONFIG_UNVERIFIED", ["未匹配到完整 [参数快照]；拒绝回退到旧默认值。"], None, 0.0
    active = [s for s in steps if abs(s["cmd"]) >= 5]
    if len(active) < 2 or not points:
        return "DATA_INVALID", ["有效的 T11 阶跃或时序数据不足。"], None, 0.0
    late = avg([s["late"] for s in active])
    cap = avg([s["cap"] for s in active])
    reverse = avg([s["reverse"] for s in active])
    overs, rise = zip(*[dynamics(s, points) for s in active])
    overshoot, rise_ms = avg(overs), avg([x for x in rise if x is not None]) if any(x is not None for x in rise) else float("inf")
    pos = avg([s["late"] for s in active if s["cmd"] > 0])
    neg = avg([s["late"] for s in active if s["cmd"] < 0])
    score = max(0.0, 100.0 - min(30.0, late * 1.4) - min(25.0, overshoot * 1.2) - min(25.0, cap * 80.0) - min(20.0, reverse * 100.0))
    candidate = {k: params[k] for k in needed}
    if summary.get("fail_rate", 0) > 0.05:
        return "IMU_INPUT_FAULT", ["IMU 读取失败率过高；先查供电、I2C 和调度。"], None, score
    if summary.get("reverse_steps", 0) > 0 or reverse > 0.05:
        return "DIRECTION_MISMATCH", ["目标与实际角速度方向不一致；先检查差速极性或 IMU Z 轴方向。"], None, score
    if summary.get("cap_steps", 0) > 0 or cap > 0.10:
        return "PHYSICAL_DIFF_LIMIT", ["差速输出受到物理/场景限制；先处理差速权限和最低内轮速度。"], None, score
    if summary.get("speed_weak_steps", 0) > 0:
        return "SPEED_LOOP_WEAK", ["轮速环无法实现差速；先回 T05。"], None, score
    if overshoot > 12.0:
        candidate["kp"] = round(max(0.05, candidate["kp"] * 0.92), 3)
        return "OVERSHOOT", [f"平均过冲 {overshoot:.1f}%：仅下调 kGyroPidP 8%。"], candidate, score
    if rise_ms > 220.0 or late > 14.0:
        candidate["kp"] = round(min(1.20, candidate["kp"] * 1.08), 3)
        return "SLUGGISH", [f"响应偏慢（rise={rise_ms:.0f}ms, late_err={late:.1f}dps）：仅上调 kGyroPidP 8%。"], candidate, score
    if abs(pos - neg) > 6.0:
        key = "pos" if pos > neg else "neg"
        candidate[key] = round(min(1.25, candidate[key] * 1.03), 3)
        return "DIRECTION_ASYMMETRY", [f"正负向晚期误差不对称（+={pos:.1f}, -={neg:.1f}dps）：仅提高较弱方向补偿 3%。"], candidate, score
    if overshoot <= 8.0 and late <= 8.0 and rise_ms <= 180.0:
        return "WELL_TUNED", ["阶跃响应、晚期误差和正负方向一致性均达标，保持当前参数。"], candidate, score
    return "ACCEPTABLE", ["当前没有单一明确瓶颈；保持参数并补采重复日志。"], candidate, score


def save_history(path, log, params, status, score, candidate):
    path.parent.mkdir(parents=True, exist_ok=True)
    fresh = not path.exists()
    fields = ("created_at", "log_file", "kp", "kd", "ff", "pos", "neg", "status", "score", "recommended_kp", "recommended_kd", "recommended_ff", "recommended_pos", "recommended_neg")
    with path.open("a", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        if fresh:
            writer.writeheader()
        writer.writerow({"created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"), "log_file": str(log), "status": status, "score": f"{score:.3f}", **{k: params.get(k) for k in ("kp", "kd", "ff", "pos", "neg")}, **{f"recommended_{k}": candidate.get(k) if candidate else None for k in ("kp", "kd", "ff", "pos", "neg")}})


def main():
    parser = argparse.ArgumentParser(description="Analyze T11 and emit a guarded direct-replace gyro candidate.")
    parser.add_argument("log_file", type=Path)
    parser.add_argument("--history", type=Path, default=Path("t11_tuning_history.csv"))
    parser.add_argument("--no-history", action="store_true")
    parser.add_argument("--override-kp", type=float)
    parser.add_argument("--override-kd", type=float)
    parser.add_argument("--override-ff", type=float)
    parser.add_argument("--override-pos-gain", type=float)
    parser.add_argument("--override-neg-gain", type=float)
    args = parser.parse_args()
    try:
        params, steps, points, summary = parse(args.log_file)
    except FileNotFoundError:
        print(f"错误: 找不到日志: {args.log_file}", file=sys.stderr)
        return 2
    overrides = {"kp": args.override_kp, "kd": args.override_kd, "ff": args.override_ff, "pos": args.override_pos_gain, "neg": args.override_neg_gain}
    params.update({k: v for k, v in overrides.items() if v is not None})
    status, reasons, candidate, score = assess(params, steps, points, summary)
    if not args.no_history:
        save_history(args.history, args.log_file, params, status, score, candidate)
    print("\n================ T11 角速度内环可替换候选报告 ================")
    print(f"日志: {args.log_file} | 阶跃汇总: {len(steps)} | 时序点: {len(points)}")
    print(f"当前: P={params.get('kp', float('nan')):.3f} D={params.get('kd', float('nan')):.3f} FF={params.get('ff', float('nan')):.3f} pos/neg={params.get('pos', float('nan')):.3f}/{params.get('neg', float('nan')):.3f}")
    print(f"状态: {status} | 综合评分: {score:.1f}/100")
    for item in reasons:
        print(f"- {item}")
    if candidate is None:
        print("[DIRECT_REPLACE_GYRO_PID] 禁止替换：先处理前置问题。")
        return 1
    print("[DIRECT_REPLACE_GYRO_PID]")
    print(f"constexpr float kGyroPidP = {candidate['kp']:.3f}f;")
    print(f"constexpr float kGyroPidD = {candidate['kd']:.3f}f;")
    print(f"constexpr float kGyroFeedforwardGain = {candidate['ff']:.3f}f;")
    print(f"constexpr float kGyroTargetPositiveGain = {candidate['pos']:.3f}f;")
    print(f"constexpr float kGyroTargetNegativeGain = {candidate['neg']:.3f}f;")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
