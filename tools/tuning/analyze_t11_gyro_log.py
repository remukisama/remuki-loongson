#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T11 angular-rate tuner for current SmartCar logs.

It validates the current parameter snapshot and checks both requested diff and
actual wheel-target diff.  When wheel goals are clamped, no PID candidate is
emitted because changing gyro gains cannot create unavailable turning authority.
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

SNAP = re.compile(r"\[参数快照\]\s*gyro\(P/I/D=([-\d.]+)/([-\d.]+)/([-\d.]+)\s+ff=([-\d.]+).*?pos(?:/neg)?=([-\d.]+)/(?:neg=)?([-\d.]+).*?\)")
STEP = re.compile(r"\[T11_STEP_SUMMARY\].*?target=([-\d.]+).*?cmd=([-\d.]+).*?late_err_abs=([-\d.]+).*?cap_rate=([-\d.]+).*?reverse_rate=([-\d.]+).*?speed_err_abs=([-\d.]+)/([-\d.]+)")
SUMMARY = re.compile(r"\[T11_SUMMARY\].*?fail_rate=([-\d.]+).*?cap_rate=([-\d.]+).*?reverse_rate=([-\d.]+).*?weak_track_steps=(\d+).*?cap_steps=(\d+).*?speed_weak_steps=(\d+).*?reverse_steps=(\d+)")
POINT = re.compile(r"\[T11\].*?idx=(\d+).*?cmd=\s*([-\d.]+).*?gf=\s*([-\d.]+).*?diff=\s*([-\d.]+).*?cap=(\d+).*?goalL/R=\s*([-\d.]+)/\s*([-\d.]+)")


def avg(v):
    return sum(v) / len(v) if v else 0.0


def parse(path):
    if not path.exists():
        raise FileNotFoundError(path)
    params, steps, points, summary = {}, [], [], {}
    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        m = SNAP.search(line)
        if m:
            params = dict(kp=float(m.group(1)), ki=float(m.group(2)), kd=float(m.group(3)), ff=float(m.group(4)), pos=float(m.group(5)), neg=float(m.group(6)))
            continue
        m = STEP.search(line)
        if m:
            steps.append(dict(target=float(m.group(1)), cmd=float(m.group(2)), late=float(m.group(3)), cap=float(m.group(4)), reverse=float(m.group(5)), se_l=float(m.group(6)), se_r=float(m.group(7))))
            continue
        m = SUMMARY.search(line)
        if m:
            summary = dict(fail=float(m.group(1)), cap=float(m.group(2)), reverse=float(m.group(3)), weak=int(m.group(4)), cap_steps=int(m.group(5)), speed_weak=int(m.group(6)), reverse_steps=int(m.group(7)))
            continue
        m = POINT.search(line)
        if m:
            points.append(dict(idx=int(m.group(1)), cmd=float(m.group(2)), gf=float(m.group(3)), diff=float(m.group(4)), cap=int(m.group(5)), goal_l=float(m.group(6)), goal_r=float(m.group(7))))
    return params, steps, points, summary


def decide(params, steps, points, summary):
    need = ("kp", "kd", "ff", "pos", "neg")
    if any(k not in params for k in need):
        return "CONFIG_UNVERIFIED", ["缺少当前 [参数快照]，拒绝使用旧默认值。"], None, 0.0
    active = [s for s in steps if abs(s["cmd"]) >= 5]
    if len(active) < 2 or len(points) < 12:
        return "DATA_INVALID", ["有效 T11 阶跃或时序不足。"], None, 0.0
    late, cap, reverse = avg([s["late"] for s in active]), avg([s["cap"] for s in active]), avg([s["reverse"] for s in active])
    physical = [p for p in points if abs(p["cmd"]) >= 5 and abs(p["diff"]) - abs((p["goal_r"] - p["goal_l"]) * 0.5) > 0.08]
    physical_rate = len(physical) / len(points)
    pos = avg([s["late"] for s in active if s["cmd"] > 0])
    neg = avg([s["late"] for s in active if s["cmd"] < 0])
    score = max(0.0, 100.0 - min(35.0, late * 1.5) - min(25.0, cap * 80.0) - min(20.0, reverse * 100.0) - min(20.0, physical_rate * 100.0))
    candidate = {k: params[k] for k in need}
    if summary.get("fail", 0) > 0.05:
        return "IMU_INPUT_FAULT", ["IMU 读取失败率高；先查 I2C、供电和调度。"], None, score
    if summary.get("reverse_steps", 0) or reverse > 0.05:
        return "DIRECTION_MISMATCH", ["目标与实际角速度方向异常；先查差速极性和 IMU Z 轴。"], None, score
    if physical_rate > 0.10:
        return "PHYSICAL_DIFF_LIMIT", [f"{physical_rate*100:.1f}% 时序点中，请求 diff 与实际 goalL/R diff 不一致；先修最低内轮速度/差速分配。"], None, score
    if summary.get("cap_steps", 0) or cap > 0.10:
        return "PHYSICAL_DIFF_LIMIT", ["差速被场景限幅；先修差速权限，不能靠放大 gyro PID。"], None, score
    if summary.get("speed_weak", 0):
        return "SPEED_LOOP_WEAK", ["轮速环不能实现所需差速；先回 T05。"], None, score
    if late > 14.0:
        candidate["kp"] = round(min(1.20, candidate["kp"] * 1.08), 3)
        return "SLUGGISH", [f"晚期角速度误差 {late:.1f} dps：仅上调 kGyroPidP 8%。"], candidate, score
    if abs(pos - neg) > 6.0:
        key = "pos" if pos > neg else "neg"
        candidate[key] = round(min(1.25, candidate[key] * 1.03), 3)
        return "DIRECTION_ASYMMETRY", [f"正负向误差不对称（+{pos:.1f}/-{neg:.1f} dps）：仅提高较弱方向补偿 3%。"], candidate, score
    if late <= 8.0:
        return "WELL_TUNED", ["角速度晚期误差、方向和差速执行均达标，保持当前值。"], candidate, score
    return "ACCEPTABLE", ["当前无单一明确瓶颈；保持参数并补采重复日志。"], candidate, score


def save(history, log, params, status, score, candidate):
    history.parent.mkdir(parents=True, exist_ok=True)
    fresh = not history.exists()
    fields = ("time", "log", "kp", "kd", "ff", "pos", "neg", "status", "score", "next_kp", "next_kd", "next_ff", "next_pos", "next_neg")
    with history.open("a", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=fields)
        if fresh:
            w.writeheader()
        w.writerow(dict(time=datetime.now(timezone.utc).isoformat(timespec="seconds"), log=str(log), status=status, score=f"{score:.3f}", kp=params.get("kp"), kd=params.get("kd"), ff=params.get("ff"), pos=params.get("pos"), neg=params.get("neg"), next_kp=candidate.get("kp") if candidate else None, next_kd=candidate.get("kd") if candidate else None, next_ff=candidate.get("ff") if candidate else None, next_pos=candidate.get("pos") if candidate else None, next_neg=candidate.get("neg") if candidate else None))


def main():
    ap = argparse.ArgumentParser(description="Analyze T11 and emit a guarded direct-replace candidate.")
    ap.add_argument("log_file", type=Path)
    ap.add_argument("--history", type=Path, default=Path("t11_tuning_history.csv"))
    ap.add_argument("--no-history", action="store_true")
    args = ap.parse_args()
    try:
        params, steps, points, summary = parse(args.log_file)
    except FileNotFoundError:
        print(f"错误: 找不到日志: {args.log_file}", file=sys.stderr)
        return 2
    status, reasons, candidate, score = decide(params, steps, points, summary)
    if not args.no_history:
        save(args.history, args.log_file, params, status, score, candidate)
    print("\n================ T11 角速度内环可替换候选报告 ================")
    print(f"状态: {status} | 评分: {score:.1f}/100 | 阶跃: {len(steps)} | 时序: {len(points)}")
    for reason in reasons:
        print(f"- {reason}")
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
