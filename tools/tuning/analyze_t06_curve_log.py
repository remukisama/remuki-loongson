#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T06 curve-closure diagnostic for the current SmartCar T06 log format.

D0 is the gyro-loop differential request before T06 limits; D is the final
request after limits.  Cap=1 is the scene max-diff limit.  Cap=0 with
|D| << |D0| is the hidden inner-wheel physical clamp.
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence

LINE = re.compile(
    r"^\[T06\]\s+Raw:\s*([-\d.]+)\s+Filt:\s*([-\d.]+)\s+dE:\s*([-\d.]+)\s+\|\s+"
    r"PD:\s*([-\d.]+)\s+Tgt:\s*([-\d.]+)\s+Cmd:\s*([-\d.]+)\s+Real:\s*([-\d.]+)\s+GF:\s*([-\d.]+)\s+GErr:\s*([-\d.]+)\s+dT:\s*([-\d.]+)\s+\|\s+"
    r"D0:\s*([-\d.]+)\s+D:\s*([-\d.]+)\s+Cap:(\d+)\s+Max:\s*([-\d.]+)\s+\|\s+"
    r"L:\s*([-\d.]+)\s+R:\s*([-\d.]+)\s+Enc:\s*([-\d.]+)/\s*([-\d.]+)\s+SE:\s*([-\d.]+)/\s*([-\d.]+)\s+\|\s+"
    r"imgErr:([-\d.]+)\s+Road:([^\s]+)\s+OFF:(\d+)\s+L/R:(\d+)/(\d+)\s+dt:\s*([-\d.]+)\s+IMU:(\d+)\s+DB:(\d+)"
)


@dataclass
class Row:
    raw: float
    filt: float
    tgt: float
    gf: float
    gerr: float
    d0: float
    diff: float
    cap: int
    se_l: float
    se_r: float
    img_err: float
    offline: int
    lmiss: int
    rmiss: int
    imu: int


def avg(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def parse(path: Path) -> List[Row]:
    if not path.exists():
        raise FileNotFoundError(path)
    rows = []
    for text in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        m = LINE.match(text.strip())
        if not m:
            continue
        g = m.groups()
        rows.append(Row(
            raw=float(g[0]), filt=float(g[1]), tgt=float(g[4]), gf=float(g[7]), gerr=float(g[8]),
            d0=float(g[10]), diff=float(g[11]), cap=int(g[12]), se_l=float(g[18]), se_r=float(g[19]),
            img_err=float(g[20]), offline=int(g[22]), lmiss=int(g[23]), rmiss=int(g[24]), imu=int(g[26]),
        ))
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Classify T06 curve failures without blindly changing PID.")
    parser.add_argument("log_file", type=Path)
    parser.add_argument("--turn-raw", type=float, default=5.0)
    args = parser.parse_args()
    try:
        rows = parse(args.log_file)
    except FileNotFoundError:
        print(f"错误: 找不到日志: {args.log_file}", file=sys.stderr)
        return 2
    if len(rows) < 10:
        print("错误: 没有解析到足够 T06 行；请提供完整 T06 日志。", file=sys.stderr)
        return 2
    turns = [r for r in rows if abs(r.raw) >= args.turn_raw or abs(r.filt) >= args.turn_raw] or rows
    physical = [r for r in turns if r.cap == 0 and abs(r.d0) - abs(r.diff) > 0.08]
    scene_cap = [r for r in turns if r.cap]
    requested = [r for r in turns if abs(r.d0) >= 0.15]
    weak_outer = [r for r in turns if abs(r.raw) >= args.turn_raw and abs(r.tgt) < 12.0]
    gyro_lag = [r for r in turns if abs(r.gerr) > 20.0 and r.imu]
    wheel_lag = [r for r in turns if max(abs(r.se_l), abs(r.se_r)) > 0.20]
    vision_bad = [r for r in turns if r.offline > 18 or r.lmiss > 15 or r.rmiss > 15 or abs(r.img_err) > 20]
    imu_bad = [r for r in rows if not r.imu]
    ratio = avg([abs(r.diff) / max(abs(r.d0), 1e-6) for r in requested])
    loss = avg([abs(r.d0) - abs(r.diff) for r in physical])
    percent = lambda n, d: f"{(100.0*n/d if d else 0.0):.1f}%"

    print("\n================ T06 弯道闭环诊断报告 ================")
    print(f"日志样本: {len(rows)} | 有效转弯样本: {len(turns)}")
    print(f"外环: Raw均值绝对={avg([abs(r.raw) for r in turns]):.2f}, Tgt均值绝对={avg([abs(r.tgt) for r in turns]):.2f}")
    print(f"角速度: GErr均值绝对={avg([abs(r.gerr) for r in turns]):.2f} dps, IMU异常={percent(len(imu_bad), len(rows))}")
    print(f"差速: D0均值绝对={avg([abs(r.d0) for r in turns]):.3f}, D均值绝对={avg([abs(r.diff) for r in turns]):.3f}, 请求保留率={ratio:.2f}")
    print(f"限幅: 场景Max={percent(len(scene_cap), len(turns))}, 物理内轮限幅推断={percent(len(physical), len(turns))}, 平均损失={loss:.3f}")
    print(f"轮速: SE均值绝对 L/R={avg([abs(r.se_l) for r in turns]):.3f}/{avg([abs(r.se_r) for r in turns]):.3f} m/s")
    print(f"视觉风险: {percent(len(vision_bad), len(turns))}")

    diagnosis = []
    if len(vision_bad) / len(turns) > 0.20:
        diagnosis.append("P0：视觉/ROI/前瞻风险高，先回 T03/T04/T13；不要改转向 PID。")
    if len(physical) / len(turns) > 0.15:
        diagnosis.append("P0：D0 有请求但 D 被二次压小且 Cap=0，主因是最低内轮速度带来的物理差速权限不足。")
    if len(scene_cap) / len(turns) > 0.20:
        diagnosis.append("P1：差速先被场景 max_diff 压住；仅在物理差速权限正常后再评估是否放宽。")
    if len(gyro_lag) / len(turns) > 0.20:
        diagnosis.append("P1：最终目标角速度存在但 GF 跟不上，回 T11；不要直接增加视觉 Kp。")
    if len(wheel_lag) / len(turns) > 0.20:
        diagnosis.append("P1：左右轮目标速度不能实现，回 T05 测 0.20~0.60 m/s 区间。")
    if len(weak_outer) / len(turns) > 0.25:
        diagnosis.append("P2：Raw 已明显偏移而目标角速度偏小；确认差速权限后再调视觉外环/前瞻。")
    if not diagnosis:
        diagnosis.append("未发现单一明显瓶颈；结合失误弯道视频检查入弯前速度是否已经充分下降。")
    print("\n诊断结论:")
    for item in diagnosis:
        print(f"- {item}")
    print("\n每轮只处理最高优先级一项，并使用相同赛道、电量和速度重复 T06。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
