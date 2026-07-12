#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SmartCar 角速度环 SSH stdout 数据分析脚本

从小车通过SSH采集 [T42] 格式的stdout数据，分析角速度环性能并给出调参建议。
用法：
  ssh root@192.168.7.100 './main --mode=gyro-motor-tune --speed=0.15 --kp=0.035 --ff=600' | python3 ssh_gyro_analyzer.py --label=low_speed

或先保存日志再分析：
  ssh root@192.168.7.100 './main --mode=gyro-motor-tune ...' > log.txt
  python3 ssh_gyro_analyzer.py --log=log.txt

数据格式（每帧一行）：
  [T42] t=<time> tgt=<target> raw=<gyro_raw> filt=<gyro_filt> err=<err> diff=<diff> gL=<goal_L> gR=<goal_R> eL=<enc_L> eR=<enc_R> pL=<pwm_L> pR=<pwm_R>
"""

from __future__ import annotations
import argparse
import math
import re
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

class C:
    R = '\033[91m'; G = '\033[92m'; Y = '\033[93m'; B = '\033[94m'
    BOLD = '\033[1m'; END = '\033[0m'

# [T42] t=2.305 tgt=-30.00 raw=-27.40 filt=-27.000 err=-3.000 diff=-0.106 gL=0.306 gR=0.094 eL=0.200 eR=0.260 pL=1106 pR=260
T42_PATTERN = re.compile(
    r'\[T42\]\s+t=([\d.]+)\s+tgt=([\d.\-]+)\s+raw=([\d.\-]+)\s+filt=([\d.\-]+)\s+'
    r'err=([\d.\-]+)\s+diff=([\d.\-]+)\s+gL=([\d.\-]+)\s+gR=([\d.\-]+)\s+'
    r'eL=([\d.\-]+)\s+eR=([\d.\-]+)\s+pL=(\d+)\s+pR=(\d+)'
)

def parse_t42_line(line: str) -> dict | None:
    m = T42_PATTERN.search(line)
    if not m:
        return None
    return {
        't': float(m.group(1)),
        'tgt': float(m.group(2)),
        'raw': float(m.group(3)),
        'filt': float(m.group(4)),
        'err': float(m.group(5)),
        'diff': float(m.group(6)),
        'gL': float(m.group(7)),
        'gR': float(m.group(8)),
        'eL': float(m.group(9)),
        'eR': float(m.group(10)),
        'pL': int(m.group(11)),
        'pR': int(m.group(12)),
    }

def collect_from_stdin(duration: float, label: str) -> list[dict]:
    """从stdin实时采集数据"""
    print(f"{C.BOLD}{C.B}{'='*70}{C.END}")
    print(f"{C.BOLD}  SmartCar 角速度环 SSH stdout 数据采集{C.END}")
    print(f"{'='*70}")
    if label:
        print(f"  测试标签: {label}")
    if duration > 0:
        print(f"  采集时长: {duration:.0f} 秒")
    print(f"  数据源: stdin (SSH pipe)")
    print(f"{'='*70}\n")
    print(f"  等待数据... (确保小车已启动)\n")

    data = []
    start_time = time.time()
    last_display = 0.0
    window = deque(maxlen=200)

    for line in sys.stdin:
        if duration > 0 and (time.time() - start_time) > duration:
            break

        parsed = parse_t42_line(line)
        if parsed is None:
            continue

        data.append(parsed)
        window.append(parsed)

        now = time.time()
        if now - last_display > 0.5 and len(window) > 0:
            last_display = now
            latest = window[-1]
            recent_errs = [w['err'] for w in window]
            recent_gyro = [w['filt'] for w in window]
            rms_err = math.sqrt(sum(e**2 for e in recent_errs) / len(recent_errs))
            max_err = max(abs(e) for e in recent_errs)
            gyro_changes = [abs(recent_gyro[i] - recent_gyro[i-1])
                           for i in range(1, len(recent_gyro))]
            avg_change = sum(gyro_changes) / len(gyro_changes) if gyro_changes else 0

            osc = ""
            if avg_change > 8: osc = f" {C.R}⚠震荡!{C.END}"
            elif avg_change > 4: osc = f" {C.Y}⚠抖动{C.END}"

            ec = C.G
            if abs(latest['err']) > 15: ec = C.R
            elif abs(latest['err']) > 8: ec = C.Y

            print(f"\r  t:{latest['t']:6.1f} tgt:{latest['tgt']:+7.1f} raw:{latest['raw']:+7.1f} "
                  f"filt:{latest['filt']:+7.1f} err:{ec}{latest['err']:+7.1f}{C.END} "
                  f"diff:{latest['diff']:+6.3f} eL:{latest['eL']:5.2f} eR:{latest['eR']:5.2f} "
                  f"pL:{latest['pL']:4d} pR:{latest['pR']:4d} "
                  f"RMS:{rms_err:5.1f} Δ:{avg_change:4.1f}{osc}   ", end='', flush=True)

    print(f"\n\n  采集完成: {len(data)} 帧, 时长 {time.time()-start_time:.1f}s\n")
    return data

def load_from_file(filepath: str) -> list[dict]:
    """从日志文件加载数据"""
    data = []
    with open(filepath, 'r', errors='ignore') as f:
        for line in f:
            parsed = parse_t42_line(line)
            if parsed:
                data.append(parsed)
    print(f"  {C.G}✅ 从 {filepath} 加载了 {len(data)} 帧数据{C.END}\n")
    return data

def analyze_and_recommend(data: list[dict], label: str):
    """分析数据并生成调参建议"""
    if len(data) < 20:
        print(f"  {C.R}数据太少 ({len(data)} 帧), 无法分析{C.END}")
        return

    print(f"\n{C.BOLD}{C.B}{'='*70}{C.END}")
    print(f"{C.BOLD}  数据分析报告{C.END}" + (f"  [{label}]" if label else ""))
    print(f"{C.BOLD}{C.B}{'='*70}{C.END}\n")

    timestamps = [d['t'] for d in data]
    targets = [d['tgt'] for d in data]
    gyro_raw = [d['raw'] for d in data]
    gyro_filt = [d['filt'] for d in data]
    errs = [d['err'] for d in data]
    diffs = [d['diff'] for d in data]
    enc_L = [d['eL'] for d in data]
    enc_R = [d['eR'] for d in data]
    pwm_L = [d['pL'] for d in data]
    pwm_R = [d['pR'] for d in data]
    goal_L = [d['gL'] for d in data]
    goal_R = [d['gR'] for d in data]

    dt_avg = (timestamps[-1] - timestamps[0]) / (len(timestamps) - 1) if len(timestamps) > 1 else 0.005

    # ============ 1. 起步阶段分析 ============
    print(f"{C.BOLD}【1. 起步阶段分析（前2秒）】{C.END}")
    startup_end = 0
    for i, ts in enumerate(timestamps):
        if ts > 2.0:
            startup_end = i
            break

    if startup_end > 10:
        s_enc_L = enc_L[:startup_end]
        s_enc_R = enc_R[:startup_end]
        s_gyro = gyro_filt[:startup_end]
        s_errs = errs[:startup_end]
        s_pwm_L = pwm_L[:startup_end]
        s_pwm_R = pwm_R[:startup_end]

        # 起转时间
        l_start = r_start = None
        for i, (el, er) in enumerate(zip(s_enc_L, s_enc_R)):
            if l_start is None and abs(el) > 0.05:
                l_start = timestamps[i]
            if r_start is None and abs(er) > 0.05:
                r_start = timestamps[i]
            if l_start and r_start:
                break

        if l_start is not None and r_start is not None:
            diff_ms = abs(l_start - r_start) * 1000
            clr = C.G if diff_ms < 100 else (C.Y if diff_ms < 300 else C.R)
            print(f"  左轮起转: {l_start*1000:.0f}ms  右轮起转: {r_start*1000:.0f}ms  "
                  f"时间差: {clr}{diff_ms:.0f}ms{C.END}")
            if diff_ms > 200:
                print(f"  {C.R}⚠ 起转差>200ms → 起步角速度震荡主因！增大 --ff{C.END}")
        else:
            print(f"  {C.Y}⚠ 未检测到起转{C.END}")

        # 起步PWM
        print(f"  起步PWM: L={max(s_pwm_L)} R={max(s_pwm_R)} (前馈后)")

        # 起步角速度波动
        s_range = max(s_gyro) - min(s_gyro)
        s_rms = math.sqrt(sum(g**2 for g in s_gyro) / len(s_gyro))
        s_max_err = max(abs(e) for e in s_errs)
        s_rms_err = math.sqrt(sum(e**2 for e in s_errs) / len(s_errs))
        print(f"  起步角速度: 范围={s_range:.1f}°/s RMS={s_rms:.1f}°/s")
        print(f"  起步误差:   max={s_max_err:.1f}°/s RMS={s_rms_err:.1f}°/s")

        # 起步震荡频率
        if len(s_gyro) > 20:
            zero_crossings = 0
            for i in range(1, len(s_gyro)):
                if (s_gyro[i-1] > 0 and s_gyro[i] < 0) or (s_gyro[i-1] < 0 and s_gyro[i] > 0):
                    zero_crossings += 1
            osc_freq = zero_crossings / 2.0  # 每秒
            print(f"  起步震荡频率: ~{osc_freq:.1f}Hz (零交叉={zero_crossings})")
    else:
        print(f"  (数据不足)")
    print()

    # ============ 2. 稳态阶段分析 ============
    print(f"{C.BOLD}【2. 稳态阶段分析（2秒后）】{C.END}")
    if startup_end < len(data) - 10:
        s_tgts = targets[startup_end:]
        s_gyro = gyro_filt[startup_end:]
        s_errs = errs[startup_end:]
        s_raw = gyro_raw[startup_end:]
        s_pwmL = pwm_L[startup_end:]
        s_pwmR = pwm_R[startup_end:]

        rms_err = math.sqrt(sum(e**2 for e in s_errs) / len(s_errs))
        max_err = max(abs(e) for e in s_errs)
        mean_err = sum(s_errs) / len(s_errs)

        print(f"  RMS误差: {rms_err:.1f}°/s  max误差: {max_err:.1f}°/s  均值: {mean_err:+.1f}°/s")

        # 过冲
        pos_idx = [i for i, t in enumerate(s_tgts) if t > 5]
        neg_idx = [i for i, t in enumerate(s_tgts) if t < -5]
        if pos_idx:
            pos_max = max(s_gyro[i] for i in pos_idx)
            pos_tgt = s_tgts[pos_idx[0]]
            os_pct = (max(0, pos_max - pos_tgt) / abs(pos_tgt) * 100) if pos_tgt else 0
            print(f"  正向(左转)过冲: {max(0,pos_max-pos_tgt):.1f}°/s ({os_pct:.1f}%)")
        if neg_idx:
            neg_min = min(s_gyro[i] for i in neg_idx)
            neg_tgt = s_tgts[neg_idx[0]]
            os_pct = (max(0, neg_tgt - neg_min) / abs(neg_tgt) * 100) if neg_tgt else 0
            print(f"  负向(右转)过冲: {max(0,neg_tgt-neg_min):.1f}°/s ({os_pct:.1f}%)")

        # 震荡检测
        changes = [abs(s_gyro[i] - s_gyro[i-1]) for i in range(1, len(s_gyro))]
        avg_ch = sum(changes) / len(changes) if changes else 0
        max_ch = max(changes) if changes else 0
        print(f"  角速度帧间变化: avg={avg_ch:.2f}°/s max={max_ch:.2f}°/s")

        # PWM饱和检测
        pwm_max = car_max = 8000
        sat_l = sum(1 for p in s_pwmL if p >= pwm_max * 0.95) / len(s_pwmL) * 100
        sat_r = sum(1 for p in s_pwmR if p >= pwm_max * 0.95) / len(s_pwmR) * 100
        print(f"  PWM饱和率: L={sat_l:.1f}% R={sat_r:.1f}% (限幅8000)")

        # 左右不对称
        if pos_idx and neg_idx:
            pos_err = sum(abs(s_errs[i]) for i in pos_idx) / len(pos_idx)
            neg_err = sum(abs(s_errs[i]) for i in neg_idx) / len(neg_idx)
            asym = abs(pos_err - neg_err)
            if asym > 8:
                print(f"  {C.Y}⚠ 左右不对称: 左转err={pos_err:.1f} 右转err={neg_err:.1f} 差={asym:.1f}{C.END}")
    else:
        print(f"  (稳态数据不足)")
    print()

    # ============ 3. 左右轮对称性 ============
    print(f"{C.BOLD}【3. 左右轮速度跟踪】{C.END}")
    enc_diff = [abs(enc_L[i] - enc_R[i]) for i in range(len(enc_L))]
    avg_diff = sum(enc_diff) / len(enc_diff)
    print(f"  左右轮速差: avg={avg_diff:.3f}m/s max={max(enc_diff):.3f}m/s")
    track_err = [(goal_L[i]+goal_R[i])/2 - (enc_L[i]+enc_R[i])/2 for i in range(len(enc_L))]
    print(f"  速度跟踪误差: {sum(track_err)/len(track_err):+.3f}m/s")
    print()

    # ============ 4. PID调参建议 ============
    print(f"{C.BOLD}{C.G}【4. 角速度环PID调参建议】{C.END}\n")

    reasons = []
    status = "WELL_TUNED"

    # 起步问题
    if startup_end > 10:
        s_gyro = gyro_filt[:startup_end]
        s_range = max(s_gyro) - min(s_gyro)
        if s_range > 25:
            status = "STARTUP_OSCILLATION"
            reasons.append(f"起步角速度波动 {s_range:.0f}°/s 过大——左右轮起转不一致+静摩擦。")

    # 稳态问题
    if startup_end < len(data) - 10:
        s_errs = errs[startup_end:]
        s_gyro = gyro_filt[startup_end:]
        rms = math.sqrt(sum(e**2 for e in s_errs) / len(s_errs))
        changes = [abs(s_gyro[i] - s_gyro[i-1]) for i in range(1, len(s_gyro))]
        avg_ch = sum(changes) / len(changes) if changes else 0

        if avg_ch > 6:
            if status != "STARTUP_OSCILLATION":
                status = "OSCILLATION"
            reasons.append(f"稳态帧间变化 {avg_ch:.1f}°/s 过大——高频震荡。")
        elif rms > 12:
            if status == "WELL_TUNED":
                status = "SLUGGISH"
            reasons.append(f"稳态RMS误差 {rms:.1f}°/s 偏大——跟踪不足。")

    if not reasons:
        reasons.append("角速度跟踪良好，参数可保持。")

    status_map = {
        "WELL_TUNED": (C.G, "参数合适 ✅"),
        "STARTUP_OSCILLATION": (C.Y, "起步震荡 ⚠️"),
        "OSCILLATION": (C.R, "稳态震荡 ❌"),
        "SLUGGISH": (C.Y, "响应不足 ⚠️"),
    }
    clr, desc = status_map.get(status, (C.B, status))
    print(f"  状态: {clr}{C.BOLD}{desc}{C.END}\n")
    for r in reasons:
        print(f"  • {r}")
    print()

    if status == "STARTUP_OSCILLATION":
        print(f"  {C.BOLD}建议:{C.END}")
        print(f"  1. 增大前馈: --ff=800 或 --ff=1000")
        print(f"  2. 降Kp: --kp=0.025")
        print(f"  3. 加Kd阻尼: --kd=0.001")
    elif status == "OSCILLATION":
        print(f"  {C.BOLD}建议:{C.END}")
        print(f"  1. 降Kp: --kp=0.025 (70%)")
        print(f"  2. 增Kd: --kd=0.002")
    elif status == "SLUGGISH":
        print(f"  {C.BOLD}建议:{C.END}")
        print(f"  1. 增Kp: --kp=0.050 (140%)")
        print(f"  2. 确认差速限幅 kGyroPidOutputLimit 未饱和")
    print()

def main():
    parser = argparse.ArgumentParser(description='SmartCar 角速度环 SSH stdout 分析')
    parser.add_argument('--log', type=str, default='', help='从日志文件分析(不读stdin)')
    parser.add_argument('--duration', type=float, default=0, help='stdin采集时长(秒)')
    parser.add_argument('--label', type=str, default='', help='测试标签')
    parser.add_argument('--save', type=str, default='', help='保存原始数据到文件')
    args = parser.parse_args()

    if args.log:
        data = load_from_file(args.log)
    else:
        data = collect_from_stdin(args.duration, args.label)

    if args.save:
        with open(args.save, 'w') as f:
            for d in data:
                f.write(f"[T42] t={d['t']:.3f} tgt={d['tgt']:.2f} raw={d['raw']:.2f} "
                        f"filt={d['filt']:.3f} err={d['err']:.3f} diff={d['diff']:.3f} "
                        f"gL={d['gL']:.3f} gR={d['gR']:.3f} eL={d['eL']:.3f} eR={d['eR']:.3f} "
                        f"pL={d['pL']} pR={d['pR']}\n")
        print(f"  {C.G}✅ 保存到 {args.save}{C.END}")

    if len(data) < 10:
        print(f"\n{C.R}错误: 只收到 {len(data)} 帧数据{C.END}")
        sys.exit(1)

    analyze_and_recommend(data, args.label)

if __name__ == '__main__':
    main()
