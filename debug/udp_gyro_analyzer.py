#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SmartCar 角速度环 UDP 实时接收与分析脚本

接收小车发来的 9 通道波形数据，实时显示并保存到 CSV。
测试结束后自动生成分析报告和调参建议。

用法：
  python3 udp_gyro_analyzer.py                    # 默认端口8080
  python3 udp_gyro_analyzer.py --port=8080        # 指定端口
  python3 udp_gyro_analyzer.py --duration=15      # 采集15秒
  python3 udp_gyro_analyzer.py --label=low_speed  # 标注测试标签

UDP 数据格式（VOFA+ JustFloat 兼容）：
  ware:target,gyro_raw,gyro_filt,err,diff,goal_L,goal_R,enc_L,enc_R\n

通道说明：
  ch1: target     - 目标角速度 (°/s)
  ch2: gyro_raw   - 原始角速度 (°/s)
  ch3: gyro_filt  - 滤波后角速度 (°/s)
  ch4: err        - 角速度误差 (°/s)
  ch5: diff       - 差速输出 (m/s)
  ch6: goal_L     - 左轮目标速度 (m/s)
  ch7: goal_R     - 右轮目标速度 (m/s)
  ch8: enc_L      - 左轮编码器速度 (m/s)
  ch9: enc_R      - 右轮编码器速度 (m/s)
"""

from __future__ import annotations
import argparse
import csv
import math
import socket
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

# ANSI颜色
class C:
    R = '\033[91m'
    G = '\033[92m'
    Y = '\033[93m'
    B = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'

CHANNEL_NAMES = ['target', 'gyro_raw', 'gyro_filt', 'err', 'diff',
                 'goal_L', 'goal_R', 'enc_L', 'enc_R']

def parse_ware_line(line: str) -> list[float] | None:
    """解析 'ware:v1,v2,...,v9\\n' 格式的数据行"""
    line = line.strip()
    if not line.startswith('ware:'):
        return None
    data_str = line[5:]
    parts = data_str.split(',')
    if len(parts) < 9:
        return None
    try:
        return [float(p) for p in parts[:9]]
    except ValueError:
        return None

def receive_udp_data(port: int, duration: float, label: str) -> tuple[list, list]:
    """
    接收UDP数据，返回 (timestamps, data_rows)
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    sock.bind(('0.0.0.0', port))
    sock.settimeout(1.0)

    timestamps = []
    data_rows = []
    start_time = time.time()
    last_display = 0.0
    recv_count = 0
    drop_count = 0

    # 滑动窗口用于实时显示
    window = deque(maxlen=200)  # 保留最近200帧

    print(f"{C.BOLD}{C.B}{'='*70}{C.END}")
    print(f"{C.BOLD}  SmartCar 角速度环 UDP 数据采集{C.END}")
    print(f"{'='*70}")
    print(f"  监听端口: {port}")
    print(f"  采集时长: {duration:.0f} 秒" if duration > 0 else "  采集时长: 无限 (Ctrl+C停止)")
    if label:
        print(f"  测试标签: {label}")
    print(f"  数据格式: ware:target,gyro_raw,gyro_filt,err,diff,goal_L,goal_R,enc_L,enc_R")
    print(f"{'='*70}\n")
    print(f"  等待数据... (确保小车已启动并发送UDP数据)\n")

    try:
        while True:
            elapsed = time.time() - start_time
            if duration > 0 and elapsed > duration:
                break

            try:
                raw, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue

            line = raw.decode('ascii', errors='ignore')
            parsed = parse_ware_line(line)
            if parsed is None:
                drop_count += 1
                continue

            now = time.time()
            timestamps.append(now - start_time)
            data_rows.append(parsed)
            window.append(parsed)
            recv_count += 1

            # 实时显示（每0.5秒刷新）
            if now - last_display > 0.5 and len(window) > 0:
                last_display = now
                latest = window[-1]
                # 计算最近窗口的统计
                recent_errs = [w[3] for w in window]  # err通道
                recent_gyro = [w[2] for w in window]  # gyro_filt
                rms_err = math.sqrt(sum(e**2 for e in recent_errs) / len(recent_errs)) if recent_errs else 0
                max_err = max(abs(e) for e in recent_errs) if recent_errs else 0

                # 检测震荡：看gyro_filt的变化率
                gyro_changes = [abs(recent_gyro[i] - recent_gyro[i-1])
                               for i in range(1, len(recent_gyro))]
                avg_change = sum(gyro_changes) / len(gyro_changes) if gyro_changes else 0

                elapsed_str = f"{elapsed:6.1f}s"
                tgt_str = f"{latest[0]:+7.1f}"
                raw_str = f"{latest[1]:+7.1f}"
                filt_str = f"{latest[2]:+7.1f}"
                err_str = f"{latest[3]:+7.1f}"
                diff_str = f"{latest[4]:+6.3f}"
                el_str = f"{latest[7]:5.2f}"
                er_str = f"{latest[8]:5.2f}"

                # 震荡警告
                osc_warn = ""
                if avg_change > 8.0:
                    osc_warn = f" {C.R}⚠ 震荡!{C.END}"
                elif avg_change > 4.0:
                    osc_warn = f" {C.Y}⚠ 抖动{C.END}"

                # 误差颜色
                err_color = C.G
                if abs(latest[3]) > 15:
                    err_color = C.R
                elif abs(latest[3]) > 8:
                    err_color = C.Y

                print(f"\r  {elapsed_str} | tgt:{tgt_str} raw:{raw_str} filt:{filt_str} "
                      f"err:{err_color}{err_str}{C.END} diff:{diff_str} "
                      f"eL:{el_str} eR:{er_str} "
                      f"RMS:{rms_err:5.1f} max:{max_err:5.1f} "
                      f"Δ:{avg_change:4.1f}{osc_warn}   ", end='', flush=True)

    except KeyboardInterrupt:
        print(f"\n\n  {C.Y}[用户中断] 停止采集{C.END}")

    sock.close()
    print(f"\n  采集完成: {recv_count} 帧, 丢弃 {drop_count} 帧, "
          f"时长 {time.time()-start_time:.1f}s\n")
    return timestamps, data_rows

def save_csv(timestamps: list, data_rows: list, filepath: Path):
    """保存数据到CSV"""
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp'] + CHANNEL_NAMES)
        for ts, row in zip(timestamps, data_rows):
            writer.writerow([f'{ts:.4f}'] + [f'{v:.4f}' for v in row])
    print(f"  {C.G}✅ 数据已保存: {filepath}{C.END}")

def analyze_and_recommend(timestamps: list, data_rows: list, label: str):
    """
    分析数据并生成调参建议
    """
    if len(data_rows) < 20:
        print(f"  {C.R}数据太少 ({len(data_rows)} 帧), 无法分析{C.END}")
        return

    print(f"\n{C.BOLD}{C.B}{'='*70}{C.END}")
    print(f"{C.BOLD}  数据分析报告{C.END}" + (f"  [{label}]" if label else ""))
    print(f"{C.BOLD}{C.B}{'='*70}{C.END}\n")

    # 提取各通道
    targets = [r[0] for r in data_rows]
    gyro_raw = [r[1] for r in data_rows]
    gyro_filt = [r[2] for r in data_rows]
    errs = [r[3] for r in data_rows]
    diffs = [r[4] for r in data_rows]
    goal_L = [r[5] for r in data_rows]
    goal_R = [r[6] for r in data_rows]
    enc_L = [r[7] for r in data_rows]
    enc_R = [r[8] for r in data_rows]

    dt_avg = (timestamps[-1] - timestamps[0]) / (len(timestamps) - 1) if len(timestamps) > 1 else 0.005

    # ============ 1. 起步阶段分析（前2秒）============
    print(f"{C.BOLD}【1. 起步阶段分析（前2秒）】{C.END}")
    startup_end = 0
    for i, ts in enumerate(timestamps):
        if ts > 2.0:
            startup_end = i
            break
    if startup_end > 10:
        startup_enc_L = enc_L[:startup_end]
        startup_enc_R = enc_R[:startup_end]
        startup_gyro = gyro_filt[:startup_end]
        startup_errs = errs[:startup_end]

        # 左右轮起转时间差
        l_start = None
        r_start = None
        for i, (el, er) in enumerate(zip(startup_enc_L, startup_enc_R)):
            if l_start is None and abs(el) > 0.05:
                l_start = timestamps[i]
            if r_start is None and abs(er) > 0.05:
                r_start = timestamps[i]
            if l_start is not None and r_start is not None:
                break

        if l_start is not None and r_start is not None:
            start_diff_ms = abs(l_start - r_start) * 1000
            color = C.G if start_diff_ms < 100 else (C.Y if start_diff_ms < 300 else C.R)
            print(f"  左轮起转时间: {l_start*1000:.0f}ms")
            print(f"  右轮起转时间: {r_start*1000:.0f}ms")
            print(f"  起转时间差:   {color}{start_diff_ms:.0f}ms{C.END}")
            if start_diff_ms > 200:
                print(f"  {C.R}⚠ 起转时间差>200ms，会导致起步角速度震荡！{C.END}")
                print(f"     → 需要增大起步前馈PWM (--ff=参数)")
        else:
            print(f"  {C.Y}⚠ 起步阶段未检测到轮速起转{C.END}")

        # 起步阶段角速度震荡
        startup_gyro_range = max(startup_gyro) - min(startup_gyro)
        startup_gyro_std = math.sqrt(sum(g**2 for g in startup_gyro) / len(startup_gyro))
        print(f"  起步角速度范围: {startup_gyro_range:.1f} °/s")
        print(f"  起步角速度RMS:  {startup_gyro_std:.1f} °/s")

        # 起步误差
        startup_max_err = max(abs(e) for e in startup_errs)
        startup_rms_err = math.sqrt(sum(e**2 for e in startup_errs) / len(startup_errs))
        print(f"  起步最大误差:   {startup_max_err:.1f} °/s")
        print(f"  起步RMS误差:    {startup_rms_err:.1f} °/s")
    else:
        print(f"  (数据不足，跳过起步分析)")
    print()

    # ============ 2. 稳态阶段分析（2秒之后）============
    print(f"{C.BOLD}【2. 稳态阶段分析（2秒之后）】{C.END}")
    if startup_end < len(data_rows) - 10:
        steady_targets = targets[startup_end:]
        steady_gyro = gyro_filt[startup_end:]
        steady_errs = errs[startup_end:]
        steady_raw = gyro_raw[startup_end:]

        # 按正负目标分组分析
        pos_idx = [i for i, t in enumerate(steady_targets) if t > 5.0]
        neg_idx = [i for i, t in enumerate(steady_targets) if t < -5.0]

        steady_rms_err = math.sqrt(sum(e**2 for e in steady_errs) / len(steady_errs))
        steady_max_err = max(abs(e) for e in steady_errs)
        steady_mean_err = sum(steady_errs) / len(steady_errs)

        print(f"  稳态RMS误差:    {steady_rms_err:.1f} °/s")
        print(f"  稳态最大误差:   {steady_max_err:.1f} °/s")
        print(f"  稳态平均误差:   {steady_mean_err:+.1f} °/s (正=跟踪不足)")

        # 过冲分析
        if pos_idx:
            pos_gyro_max = max(steady_gyro[i] for i in pos_idx)
            pos_tgt = steady_targets[pos_idx[0]]
            pos_overshoot = max(0, pos_gyro_max - pos_tgt)
            pos_overshoot_pct = (pos_overshoot / abs(pos_tgt) * 100) if pos_tgt != 0 else 0
            print(f"  正向(左转)过冲: {pos_overshoot:.1f} °/s ({pos_overshoot_pct:.1f}%)")
        if neg_idx:
            neg_gyro_min = min(steady_gyro[i] for i in neg_idx)
            neg_tgt = steady_targets[neg_idx[0]]
            neg_overshoot = max(0, neg_tgt - neg_gyro_min)
            neg_overshoot_pct = (neg_overshoot / abs(neg_tgt) * 100) if neg_tgt != 0 else 0
            print(f"  负向(右转)过冲: {neg_overshoot:.1f} °/s ({neg_overshoot_pct:.1f}%)")

        # 震荡检测：角速度变化率
        gyro_changes = [abs(steady_gyro[i] - steady_gyro[i-1])
                       for i in range(1, len(steady_gyro))]
        avg_gyro_change = sum(gyro_changes) / len(gyro_changes) if gyro_changes else 0
        max_gyro_change = max(gyro_changes) if gyro_changes else 0
        print(f"  角速度平均帧间变化: {avg_gyro_change:.2f} °/s")
        print(f"  角速度最大帧间变化: {max_gyro_change:.2f} °/s")

        # 原始vs滤波延迟
        if len(steady_raw) > 10:
            # 简单延迟估计：找原始信号和滤波信号的互相关峰值
            raw_norm = [r - sum(steady_raw)/len(steady_raw) for r in steady_raw]
            filt_norm = [f - sum(steady_gyro)/len(steady_gyro) for f in steady_gyro]
            max_corr = 0
            best_delay = 0
            for delay in range(0, min(20, len(raw_norm)-1)):
                corr = sum(raw_norm[i] * filt_norm[i+delay] for i in range(len(raw_norm)-delay))
                norm = math.sqrt(sum(r**2 for r in raw_norm[:len(raw_norm)-delay]) *
                                sum(f**2 for f in filt_norm[:len(raw_norm)-delay]))
                if norm > 0:
                    corr_norm = corr / norm
                    if abs(corr_norm) > abs(max_corr):
                        max_corr = corr_norm
                        best_delay = delay
            delay_ms = best_delay * dt_avg * 1000
            print(f"  滤波延迟:       {delay_ms:.1f}ms ({best_delay}帧)")
    else:
        print(f"  (稳态数据不足，跳过)")
    print()

    # ============ 3. 左右轮对称性分析 ============
    print(f"{C.BOLD}【3. 左右轮对称性分析】{C.END}")
    enc_diff = [abs(enc_L[i] - enc_R[i]) for i in range(len(enc_L))]
    avg_enc_diff = sum(enc_diff) / len(enc_diff)
    max_enc_diff = max(enc_diff)
    print(f"  左右轮速平均差: {avg_enc_diff:.3f} m/s")
    print(f"  左右轮速最大差: {max_enc_diff:.3f} m/s")

    # 编码器 vs 目标
    goal_avg = [(goal_L[i] + goal_R[i]) / 2 for i in range(len(goal_L))]
    enc_avg = [(enc_L[i] + enc_R[i]) / 2 for i in range(len(enc_L))]
    track_err = [goal_avg[i] - enc_avg[i] for i in range(len(enc_avg))]
    avg_track_err = sum(track_err) / len(track_err)
    print(f"  平均速度跟踪误差: {avg_track_err:+.3f} m/s (正=欠跟踪)")
    print()

    # ============ 4. PID调参建议 ============
    print(f"{C.BOLD}{C.G}【4. 角速度环PID调参建议】{C.END}\n")

    reasons = []
    status = "WELL_TUNED"

    # 判断起步问题
    if startup_end > 10:
        startup_gyro_range = max(gyro_filt[:startup_end]) - min(gyro_filt[:startup_end])
        if startup_gyro_range > 30:
            reasons.append(f"起步阶段角速度波动范围 {startup_gyro_range:.0f}°/s 过大，"
                          f"存在起步震荡。主要原因是左右轮起转时间不一致+静摩擦。")
            status = "STARTUP_OSCILLATION"

    # 判断稳态性能
    if startup_end < len(data_rows) - 10:
        steady_errs = errs[startup_end:]
        steady_rms = math.sqrt(sum(e**2 for e in steady_errs) / len(steady_errs))
        steady_max = max(abs(e) for e in steady_errs)

        # 震荡判断
        steady_gyro = gyro_filt[startup_end:]
        gyro_changes = [abs(steady_gyro[i] - steady_gyro[i-1])
                       for i in range(1, len(steady_gyro))]
        avg_change = sum(gyro_changes) / len(gyro_changes) if gyro_changes else 0

        if avg_change > 6.0:
            status = "OSCILLATION"
            reasons.append(f"稳态阶段角速度帧间变化 {avg_change:.1f}°/s 过大，"
                          f"存在高频震荡。建议降低 Kp 或增大 Kd。")
        elif steady_rms > 15:
            status = "SLUGGISH"
            reasons.append(f"稳态RMS误差 {steady_rms:.1f}°/s 偏大，"
                          f"跟踪不足。建议增大 Kp。")
        elif steady_max > 20:
            status = "OVERSHOOT"
            reasons.append(f"稳态最大误差 {steady_max:.1f}°/s 偏大，"
                          f"可能存在过冲。建议降低 Kp 或增大 Kd。")

    if not reasons:
        reasons.append("角速度跟踪表现良好，当前参数可保持。")
        status = "WELL_TUNED"

    status_colors = {
        "WELL_TUNED": C.G,
        "STARTUP_OSCILLATION": C.Y,
        "OSCILLATION": C.R,
        "SLUGGISH": C.Y,
        "OVERSHOOT": C.Y,
    }
    status_cn = {
        "WELL_TUNED": "参数合适 ✅",
        "STARTUP_OSCILLATION": "起步震荡 ⚠️",
        "OSCILLATION": "稳态震荡 ❌",
        "SLUGGISH": "响应不足 ⚠️",
        "OVERSHOOT": "过冲偏大 ⚠️",
    }

    color = status_colors.get(status, C.B)
    print(f"  状态: {color}{C.BOLD}{status_cn.get(status, status)}{C.END}\n")
    for r in reasons:
        print(f"  • {r}")
    print()

    # 具体建议
    if status == "STARTUP_OSCILLATION":
        print(f"  {C.BOLD}建议操作:{C.END}")
        print(f"  1. 增大起步前馈PWM: --ff=800 或 --ff=1000")
        print(f"  2. 如果前馈已很大仍震荡，降低 Kp: --kp=0.025")
        print(f"  3. 加入少量 Kd 增加阻尼: --kd=0.001")
    elif status == "OSCILLATION":
        print(f"  {C.BOLD}建议操作:{C.END}")
        print(f"  1. 降低 Kp: --kp=0.025 (当前值的70%)")
        print(f"  2. 增大 Kd: --kd=0.002 或 --kd=0.003")
        print(f"  3. 检查低通滤波系数 kGyroLpfAlpha 是否过小")
    elif status == "SLUGGISH":
        print(f"  {C.BOLD}建议操作:{C.END}")
        print(f"  1. 增大 Kp: --kp=0.050 (当前值的140%)")
        print(f"  2. 可加入少量 Ki 积分消除稳态误差")
        print(f"  3. 确认差速输出限幅 kGyroPidOutputLimit 未被限死")
    elif status == "OVERSHOOT":
        print(f"  {C.BOLD}建议操作:{C.END}")
        print(f"  1. 小幅降低 Kp 或增大 Kd")
        print(f"  2. 检查是否差速输出频繁触限")

    print()

def main():
    parser = argparse.ArgumentParser(description='SmartCar 角速度环 UDP 数据采集与分析')
    parser.add_argument('--port', type=int, default=8080, help='UDP监听端口 (默认8080)')
    parser.add_argument('--duration', type=float, default=0,
                       help='采集时长(秒)，0=无限直到Ctrl+C')
    parser.add_argument('--label', type=str, default='', help='测试标签')
    parser.add_argument('--no-save', action='store_true', help='不保存CSV')
    args = parser.parse_args()

    # 接收数据
    timestamps, data_rows = receive_udp_data(args.port, args.duration, args.label)

    if len(data_rows) < 10:
        print(f"\n{C.R}错误: 只收到 {len(data_rows)} 帧数据，无法分析{C.END}")
        print(f"请检查:")
        print(f"  1. 小车是否已启动 gyro-motor-tune 模式")
        print(f"  2. 小车IP是否为 192.168.7.100")
        print(f"  3. PC的IP是否为 192.168.7.22 (小车UDP目标)")
        print(f"  4. 防火墙是否放行UDP {args.port}端口")
        sys.exit(1)

    # 保存CSV
    if not args.no_save:
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        label_str = f"_{args.label}" if args.label else ""
        csv_path = Path(f'gyro_tune_{timestamp_str}{label_str}.csv')
        save_csv(timestamps, data_rows, csv_path)

    # 分析数据
    analyze_and_recommend(timestamps, data_rows, args.label)

if __name__ == '__main__':
    main()
