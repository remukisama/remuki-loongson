#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stateful log-to-advice logic used by the UDP tuning receiver."""
from __future__ import annotations

import re
from pathlib import Path

import analyze_t05_actual_log as t05
import analyze_t06_curve_log as t06
import analyze_t11_gyro_log as t11


def choose_test(line, current):
    if "TEST 05" in line or "T05_ACTUAL" in line or line.startswith("[T05"):
        return "T05"
    if "TEST 11" in line or line.startswith("[T11") or "[参数快照] gyro(" in line:
        return "T11"
    if "TEST 06" in line or line.startswith("[T06"):
        return "T06"
    return current


def done(test, line):
    return (test == "T05" and "[T05_RETURN]" in line) or (test == "T11" and "[T11_SUMMARY]" in line) or (test == "T06" and "[T06_SUMMARY]" in line)


def curve_min(path, fallback=.20):
    try:
        text = path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return fallback
    match = re.search(r"constexpr\s+float\s+kCurveMinWheelSpeedGoal\s*=\s*([-\d.]+)f", text)
    return float(match.group(1)) if match else fallback


class LiveTuner:
    def __init__(self, root, curve_config, t05_interval=100, t11_interval=24, t06_interval=10):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.curve_min = curve_min(Path(curve_config))
        self.intervals = {"T05": t05_interval, "T11": t11_interval, "T06": t06_interval}
        self.sessions = {}

    def consume(self, session, line):
        info = self.sessions.setdefault(session, {"test": None, "paths": {}, "count": {}})
        test = choose_test(line, info["test"])
        if test not in self.intervals:
            return []
        info["test"] = test
        path = info["paths"].setdefault(test, self.root / f"{session}_{test}.log")
        with path.open("a", encoding="utf-8") as fp:
            fp.write(line + "\n")
        active = (test == "T05" and line.startswith("T05_ACTUAL ")) or (test == "T11" and line.startswith("[T11] ")) or (test == "T06" and line.startswith("[T06] "))
        outputs = []
        if active:
            info["count"][test] = info["count"].get(test, 0) + 1
            if info["count"][test] % self.intervals[test] == 0:
                outputs.extend(self.report(test, path, False))
        if done(test, line):
            outputs.extend(self.report(test, path, True))
            outputs.append(f"[UDP_TUNE_RECEIVER] saved={path}")
        return outputs

    def report(self, test, path, final):
        try:
            if test == "T05":
                return self.t05_report(path, final)
            if test == "T11":
                return self.t11_report(path, final)
            return self.t06_report(path, final)
        except Exception as exc:
            return [f"[UDP_TUNE_RECEIVER] {test} analyzer error: {exc}"]

    def t05_report(self, path, final):
        runs = t05.parse_runs(path)
        if not runs:
            return []
        run = runs[-1]
        metrics = t05.validate_and_score(run, 8000.0)
        kp, ki, why = t05.recommend(run, metrics)
        tag = "FINAL" if final else "LIVE"
        rows = [f"[T05_{tag}] {metrics.status}; score={metrics.score:.1f}/100; samples={len(run.samples)}"]
        if metrics.valid and metrics.status not in {"PWM_SATURATED", "LEFT_RIGHT_ASYMMETRY"}:
            rows.append(f"DIRECT: kSpeedPidP={kp:.1f}f; kSpeedPidI={ki:.1f}f. {why}")
        else:
            rows.append("HOLD: 当前数据不允许替换速度 PID。")
        rows.extend(f"- {reason}" for reason in metrics.reasons)
        return rows

    def t11_report(self, path, final):
        params, steps, points, summary = t11.parse(path)
        status, reasons, candidate, score = t11.decide(params, steps, points, summary)
        tag = "FINAL" if final else "LIVE"
        rows = [f"[T11_{tag}] {status}; score={score:.1f}/100; steps={len(steps)}; points={len(points)}"]
        if candidate:
            rows.append("DIRECT: " +
                        f"kGyroPidP={candidate['kp']:.3f}f; kGyroPidD={candidate['kd']:.3f}f; " +
                        f"kGyroFeedforwardGain={candidate['ff']:.3f}f; " +
                        f"kGyroTargetPositiveGain={candidate['pos']:.3f}f; " +
                        f"kGyroTargetNegativeGain={candidate['neg']:.3f}f")
        else:
            rows.append("HOLD: 当前数据不允许替换 gyro PID。")
        rows.extend(f"- {reason}" for reason in reasons)
        return rows

    def t06_report(self, path, final):
        rows = t06.parse(path)
        if len(rows) < 8:
            return []
        turns = [row for row in rows if abs(row.raw) >= 5 or abs(row.filt) >= 5] or rows
        count = len(turns)
        physical = sum(1 for row in turns if row.cap == 0 and abs(row.d0) - abs(row.diff) > .08) / count
        scene = sum(1 for row in turns if row.cap) / count
        vision = sum(1 for row in turns if row.offline > 18 or row.lmiss > 15 or row.rmiss > 15 or abs(row.img_err) > 20) / count
        gyro = sum(1 for row in turns if row.imu and abs(row.gerr) > 20) / count
        wheel = sum(1 for row in turns if max(abs(row.se_l), abs(row.se_r)) > .20) / count
        tag = "FINAL" if final else "LIVE"
        out = [f"[T06_{tag}] physical={physical*100:.1f}% sceneCap={scene*100:.1f}% vision={vision*100:.1f}% gyro={gyro*100:.1f}% wheel={wheel*100:.1f}%"]
        if vision > .20:
            out.append("HOLD: 先修 T03/T04 图像、ROI、前瞻；不要改 PID。")
        elif physical > .15:
            candidate = max(.15, round(self.curve_min - .02, 2))
            out.append(f"DIRECT-GATED: 当前 kCurveMinWheelSpeedGoal={self.curve_min:.2f}f；只有 T05(0.20~0.60m/s)通过且两轮 T06 仍物理限幅时，候选={candidate:.2f}f。")
        elif scene > .20:
            out.append("HOLD: 先检查场景 max_diff 限幅，不增加 PID。")
        elif gyro > .20:
            out.append("HOLD: 角速度跟踪差，回 T11。")
        elif wheel > .20:
            out.append("HOLD: 轮速实现不足，回 T05 低速区间。")
        else:
            out.append("PASS: 暂无主链路硬限幅，保持当前值并结合固定弯道视频评估。")
        return out
