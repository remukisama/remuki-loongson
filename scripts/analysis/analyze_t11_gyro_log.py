#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SmartCar T11 (Gyro Angular Velocity Loop) Log Analyzer and PID Tuner
Parses T11_STEP_SUMMARY, T11_SUMMARY and T11 time-series logs to recommend optimal gyro PID parameters.
"""

from __future__ import annotations
import argparse
import math
import re
import sys
from pathlib import Path

# Color terminal output utilities
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

def print_banner():
    banner = f"""
{Colors.BOLD}{Colors.BLUE}======================================================================
     SmartCar T11 Gyro Inner Loop Log Analyzer & PID Recommendation
======================================================================{Colors.END}
"""
    print(banner)

def parse_t11_log(log_path: Path) -> tuple[dict[str, any], list[dict[str, any]], list[dict[str, any]], dict[str, any]]:
    """
    Parses T11 log file, extracting:
    1. Initial gyro parameter snapshot from [参数快照]
    2. Step-by-step summary statistics from [T11_STEP_SUMMARY]
    3. Time-series data points from [T11]
    4. Overall run summary from [T11_SUMMARY]
    """
    if not log_path.exists():
        print(f"{Colors.RED}[Error] Log file not found: {log_path}{Colors.END}")
        sys.exit(1)

    try:
        content = log_path.read_text(encoding='utf-8', errors='ignore')
    except Exception as e:
        print(f"{Colors.RED}[Error] Failed to read log file: {e}{Colors.END}")
        sys.exit(1)

    # 1. Parse Parameters Snapshot, e.g.:
    # [参数快照] gyro(P/I/D=0.240/0.0000/0.0180 ff=0.120 out=65.0 dOut=50.0/80.0 pos=1.00/neg=1.00 dead=2.0 lpf=0.30 maxDiffRatio=2.00)
    params = {
        "kp": None, "ki": None, "kd": None, "ff": None,
        "out_limit": None, "diff_limit_pos": None, "diff_limit_neg": None,
        "pos_gain": None, "neg_gain": None, "deadband": None,
        "lpf_alpha": None, "max_diff_ratio": None
    }
    
    snapshot_pattern = re.compile(
        r"\[参数快照\]\s*gyro\(P/I/D=([\d\.]+)/([\d\.]+)/([\d\.]+)\s+ff=([\d\.]+)\s+out=([\d\.]+)\s+dOut=([\d\.]+)/([\d\.]+)\s+pos=([\d\.]+)/neg=([\d\.]+)\s+dead=([\d\.]+)\s+lpf=([\d\.]+)\s+maxDiffRatio=([\d\.]+)\)"
    )
    
    # 2. Parse step summary, e.g.:
    # [T11_STEP_SUMMARY] idx=2 target=35.0 cmd=35.0 samples=180 read_fail=0 gf_mean=30.20 gf_abs=30.20 real_abs=32.10 err_abs=4.80 late_err_abs=2.50 diff_abs=20.40 cap_rate=0.00 reverse_rate=0.00 speed_err_abs=0.12/0.15 enc_abs=0.35/0.45
    step_summary_pattern = re.compile(
        r"\[T11_STEP_SUMMARY\]\s+idx=(\d+)\s+target=([\-\d\.]+)\s+cmd=([\-\d\.]+)\s+samples=(\d+)\s+read_fail=(\d+)\s+gf_mean=([\-\d\.]+)\s+gf_abs=([\d\.]+)\s+real_abs=([\d\.]+)\s+err_abs=([\d\.]+)\s+late_err_abs=([\d\.]+)\s+diff_abs=([\d\.]+)\s+cap_rate=([\d\.]+)\s+reverse_rate=([\d\.]+)\s+speed_err_abs=([\d\.]+)/([\d\.]+)\s+enc_abs=([\d\.]+)/([\d\.]+)"
    )

    # 3. Parse overall summary, e.g.:
    # [T11_SUMMARY] samples=1260 read_fail=0 fail_rate=0.000 cap_rate=0.05 reverse_rate=0.00 weak_track_steps=1 cap_steps=0 speed_weak_steps=0 reverse_steps=0 base_speed=0.5 hold_ms=900
    overall_pattern = re.compile(
        r"\[T11_SUMMARY\]\s+samples=(\d+)\s+read_fail=(\d+)\s+fail_rate=([\d\.]+)\s+cap_rate=([\d\.]+)\s+reverse_rate=([\d\.]+)\s+weak_track_steps=(\d+)\s+cap_steps=(\d+)\s+speed_weak_steps=(\d+)\s+reverse_steps=(\d+)\s+base_speed=([\d\.]+)\s+hold_ms=(\d+)"
    )

    # 4. Parse time-series data point, e.g.:
    # [T11] idx=2 t=0.06 target=  35.0 cmd=  35.0 real=   0.0 gf=   0.0 err=  35.0 diff0=  12.0 diff=  12.0 cap=0 max= 6.0 goalL/R=  0.2/  0.8 encL/R=  0.2/  0.2 se=  0.0/  0.6 out=  0/  0
    time_series_pattern = re.compile(
        r"\[T11\]\s+idx=(\d+)\s+t=([\d\.]+)\s+target=\s*([\-\d\.]+)\s+cmd=\s*([\-\d\.]+)\s+real=\s*([\-\d\.]+)\s+gf=\s*([\-\d\.]+)\s+err=\s*([\-\d\.]+)\s+diff0=\s*([\-\d\.]+)\s+diff=\s*([\-\d\.]+)\s+cap=(\d+)\s+max=\s*([\d\.]+)\s+goalL/R=\s*([\-\d\.]+)/\s*([\-\d\.]+)\s+encL/R=\s*([\-\d\.]+)/\s*([\-\d\.]+)\s+se=\s*([\-\d\.]+)/\s*([\-\d\.]+)\s+out=\s*([\-\d\.]+)/\s*([\-\d\.]+)"
    )

    step_summaries = []
    time_series = []
    overall_summary = {}

    for line in content.splitlines():
        line = line.strip()
        
        # Match snapshot
        snapshot_match = snapshot_pattern.search(line)
        if snapshot_match:
            params["kp"] = float(snapshot_match.group(1))
            params["ki"] = float(snapshot_match.group(2))
            params["kd"] = float(snapshot_match.group(3))
            params["ff"] = float(snapshot_match.group(4))
            params["out_limit"] = float(snapshot_match.group(5))
            params["diff_limit_pos"] = float(snapshot_match.group(6))
            params["diff_limit_neg"] = float(snapshot_match.group(7))
            params["pos_gain"] = float(snapshot_match.group(8))
            params["neg_gain"] = float(snapshot_match.group(9))
            params["deadband"] = float(snapshot_match.group(10))
            params["lpf_alpha"] = float(snapshot_match.group(11))
            params["max_diff_ratio"] = float(snapshot_match.group(12))
            continue

        # Match step summary
        step_match = step_summary_pattern.search(line)
        if step_match:
            step_summaries.append({
                "idx": int(step_match.group(1)),
                "target": float(step_match.group(2)),
                "cmd": float(step_match.group(3)),
                "samples": int(step_match.group(4)),
                "read_fail": int(step_match.group(5)),
                "gf_mean": float(step_match.group(6)),
                "gf_abs": float(step_match.group(7)),
                "real_abs": float(step_match.group(8)),
                "err_abs": float(step_match.group(9)),
                "late_err_abs": float(step_match.group(10)),
                "diff_abs": float(step_match.group(11)),
                "cap_rate": float(step_match.group(12)),
                "reverse_rate": float(step_match.group(13)),
                "se_l": float(step_match.group(14)),
                "se_r": float(step_match.group(15)),
                "enc_l": float(step_match.group(16)),
                "enc_r": float(step_match.group(17)),
            })
            continue

        # Match overall summary
        overall_match = overall_pattern.search(line)
        if overall_match:
            overall_summary = {
                "samples": int(overall_match.group(1)),
                "read_fail": int(overall_match.group(2)),
                "fail_rate": float(overall_match.group(3)),
                "cap_rate": float(overall_match.group(4)),
                "reverse_rate": float(overall_match.group(5)),
                "weak_track_steps": int(overall_match.group(6)),
                "cap_steps": int(overall_match.group(7)),
                "speed_weak_steps": int(overall_match.group(8)),
                "reverse_steps": int(overall_match.group(9)),
                "base_speed": float(overall_match.group(10)),
                "hold_ms": int(overall_match.group(11)),
            }
            continue

        # Match time series
        ts_match = time_series_pattern.search(line)
        if ts_match:
            time_series.append({
                "idx": int(ts_match.group(1)),
                "t": float(ts_match.group(2)),
                "target": float(ts_match.group(3)),
                "cmd": float(ts_match.group(4)),
                "real": float(ts_match.group(5)),
                "gf": float(ts_match.group(6)),
                "err": float(ts_match.group(7)),
                "diff0": float(ts_match.group(8)),
                "diff": float(ts_match.group(9)),
                "cap": int(ts_match.group(10)),
                "max": float(ts_match.group(11)),
                "goal_l": float(ts_match.group(12)),
                "goal_r": float(ts_match.group(13)),
                "enc_l": float(ts_match.group(14)),
                "enc_r": float(ts_match.group(15)),
                "se_l": float(ts_match.group(16)),
                "se_r": float(ts_match.group(17)),
                "out_l": float(ts_match.group(18)),
                "out_r": float(ts_match.group(19)),
            })

    # Sort step summaries by index
    step_summaries.sort(key=lambda s: s["idx"])
    return params, step_summaries, time_series, overall_summary

def analyze_step_dynamics(step_idx: int, cmd: float, ts_points: list[dict[str, any]]) -> dict[str, any]:
    """
    Analyzes overshoot and rise-time dynamics of a single step based on time-series.
    """
    points = [p for p in ts_points if p["idx"] == step_idx]
    if not points or abs(cmd) < 5.0:
        return {"overshoot_pct": 0.0, "overshoot_abs": 0.0, "rise_time_ms": 0.0, "settling_time_ms": 0.0}

    gfs = [p["gf"] for p in points]
    ts = [p["t"] * 1000.0 for p in points]  # convert to ms
    
    # Calculate overshoot
    max_gf = max(gfs)
    min_gf = min(gfs)
    
    if cmd > 0.0:
        overshoot_abs = max(0.0, max_gf - cmd)
    else:
        overshoot_abs = max(0.0, cmd - min_gf)
        
    overshoot_pct = (overshoot_abs / abs(cmd)) * 100

    # Calculate rise time (time to first cross 90% of command from step start)
    rise_time_ms = 0.0
    start_time = ts[0]
    
    threshold = 0.90 * abs(cmd)
    for t, gf in zip(ts, gfs):
        if abs(gf) >= threshold:
            rise_time_ms = t - start_time
            break
            
    return {
        "overshoot_pct": overshoot_pct,
        "overshoot_abs": overshoot_abs,
        "rise_time_ms": rise_time_ms if rise_time_ms > 0 else float('nan'),
    }

def generate_recommendations(
    params: dict[str, any], 
    steps: list[dict[str, any]], 
    overall: dict[str, any],
    ts: list[dict[str, any]]
) -> tuple[float, float, float, list[str], str]:
    """
    Generates recommendations for kGyroPidP, kGyroPidD and kGyroFeedforwardGain.
    Returns: (rec_kp, rec_kd, rec_ff, reasons_list, status_summary)
    """
    curr_kp = params["kp"] if params["kp"] is not None else 0.24
    curr_kd = params["kd"] if params["kd"] is not None else 0.018
    curr_ff = params["ff"] if params["ff"] is not None else 0.12

    next_kp = curr_kp
    next_kd = curr_kd
    next_ff = curr_ff
    reasons = []
    
    # Check overall error rates
    fail_rate = overall.get("fail_rate", 0.0)
    reverse_steps = overall.get("reverse_steps", 0)
    cap_steps = overall.get("cap_steps", 0)
    speed_weak_steps = overall.get("speed_weak_steps", 0)
    weak_track_steps = overall.get("weak_track_steps", 0)
    hold_ms = overall.get("hold_ms", 900)

    # 1. Critical Sensor Fault
    if fail_rate > 0.10:
        reasons.append("IMU 读取失败率过高 (> 10%)。可能是 I2C 连线松动、电磁干扰或传感器供电问题。在硬件修复前，不要调整 PID 参数。")
        return curr_kp, curr_kd, curr_ff, reasons, "HARDWARE_FAULT"

    # 2. Differential Direction Reverse
    if reverse_steps > 0:
        reasons.append("目标角速度与实际角速度方向相反！轮速差速极性颠倒或陀螺仪 Z 轴方向安装错误。请立即检查：左右电机接线是否相反、分配差速公式符号，或陀螺仪 Z 轴安装符号，切忌增加 Kp。")
        return curr_kp, curr_kd, curr_ff, reasons, "DIRECTION_MISMATCH"

    # 3. Speed Loop Lag (T05)
    if speed_weak_steps > 0:
        reasons.append("角速度环虽然给出了差速指令，但左右电机轮速无法跟随目标 (轮速跟踪弱)。内环性能受限于速度环。请优先返回 T05 实地测试调好电机速度环 (增大速度 Kp/Ki 或 PWM 限幅)，不要盲目调大角速度环 Kp。")
        return curr_kp, curr_kd, curr_ff, reasons, "SPEED_LOOP_WEAK"

    # Active step statistics extraction
    active_steps = [s for s in steps if abs(s["cmd"]) > 5.0]
    if not active_steps:
        reasons.append("未检测到有效的角速度阶跃测试数据。请检查测试过程中小车是否正常运行。")
        return curr_kp, curr_kd, curr_ff, reasons, "NO_DATA"

    # Compute average metrics across active steps
    avg_late_err = sum(s["late_err_abs"] for s in active_steps) / len(active_steps)
    avg_cap_rate = sum(s["cap_rate"] for s in active_steps) / len(active_steps)
    
    # Calculate dynamics from time-series
    dynamics = []
    for s in active_steps:
        dyn = analyze_step_dynamics(s["idx"], s["cmd"], ts)
        dynamics.append(dyn)
        
    valid_overshoots = [d["overshoot_pct"] for d in dynamics]
    avg_overshoot_pct = sum(valid_overshoots) / len(valid_overshoots) if valid_overshoots else 0.0
    
    valid_rise_times = [d["rise_time_ms"] for d in dynamics if not math.isnan(d["rise_time_ms"])]
    avg_rise_time = sum(valid_rise_times) / len(valid_rise_times) if valid_rise_times else float('nan')

    # Detect Left/Right asymmetry
    pos_steps = [s for s in active_steps if s["cmd"] > 0.0]
    neg_steps = [s for s in active_steps if s["cmd"] < 0.0]
    
    asymmetry_detected = False
    if pos_steps and neg_steps:
        pos_err = sum(s["late_err_abs"] for s in pos_steps) / len(pos_steps)
        neg_err = sum(s["late_err_abs"] for s in neg_steps) / len(neg_steps)
        if abs(pos_err - neg_err) > 8.0:
            asymmetry_detected = True

    status = "WELL_TUNED"

    # 4. Differential Saturation (Capped)
    if cap_steps > 0 or avg_cap_rate > 0.35:
        next_kp = curr_kp * 0.90  # slightly back off
        reasons.append(f"差速输出经常被硬性限幅压死 (平均限幅率 {avg_cap_rate*100:.1f}%)。说明物理差速能力已到极限，继续增大 Kp 会导致积分饱和并引发剧烈超调。建议适当调大最大差速限制比例 kMaxDiffRatio 或小幅放宽 kGyroPidOutputLimit。")
        status = "OUTPUT_SATURATED"

    # 5. Massive Overshoot or High-frequency Jitter (High Kp / Low D)
    elif avg_overshoot_pct > 15.0:
        # Overshoot is too high, decrease Kp and increase D to add damping
        next_kp = curr_kp * 0.85
        next_kd = curr_kd * 1.30
        reasons.append(f"角速度跟踪存在严重过冲 (平均过冲 {avg_overshoot_pct:.1f}% > 15%)，响应过猛。建议降低比例项 Kp 并增加微分项 Kd 提高阻尼，以抑制甩尾和车身摆动。")
        status = "OVERSHOOT"

    # 6. Sluggish Response / High Steady-state Error (Low Kp / Low FF)
    elif weak_track_steps > 0 or avg_late_err > 12.0:
        # We can increase P and FF because we are NOT saturated
        if avg_late_err > 20.0 or math.isnan(avg_rise_time) or avg_rise_time > 200.0:
            next_kp = curr_kp * 1.20
            next_ff = curr_ff * 1.25
            reasons.append(f"角速度跟踪严重迟缓 (稳态差值 {avg_late_err:.1f} dps，响应延迟偏大)，且物理上限尚有富余。建议大幅增加比例项 Kp 和前馈系数 kGyroFeedforwardGain。")
        else:
            next_kp = curr_kp * 1.10
            next_ff = curr_ff * 1.15
            reasons.append(f"角速度响应偏慢 (稳态差值 {avg_late_err:.1f} dps)，出弯可能拖尾。建议小幅提升 Kp 和前馈系数。")
        status = "SLUGGISH"

    # 7. Well Tuned
    else:
        reasons.append("实际角速度能够极其饱满、平稳地紧跟阶跃目标，且无震荡和严重过冲。内环参数已达优秀状态，建议保持。")
        status = "WELL_TUNED"

    # 8. Asymmetry Warning & Handling
    if asymmetry_detected:
        reasons.append("⚠️ 检测到明显的左右转向不对称！正向(左转)与负向(右转)的跟踪误差差值大于 8 dps。建议小幅增加较弱一侧的方向补偿系数 (kGyroTargetPositiveGain 或 kGyroTargetNegativeGain) 以对齐左右手感。")

    # Clamp to safe boundaries
    next_kp = max(0.05, min(1.2, next_kp))
    next_kd = max(0.0, min(0.1, next_kd))
    next_ff = max(0.0, min(0.8, next_ff))

    return round(next_kp, 3), round(next_kd, 3), round(next_ff, 3), reasons, status

def print_dashboard(params: dict[str, any], steps: list[dict[str, any]], overall: dict[str, any], ts: list[dict[str, any]]):
    """
    Prints a beautifully formatted dashboard comparing each test step.
    """
    # Print configurations
    kp = params["kp"] if params["kp"] is not None else 0.24
    ki = params["ki"] if params["ki"] is not None else 0.0
    kd = params["kd"] if params["kd"] is not None else 0.018
    ff = params["ff"] if params["ff"] is not None else 0.12
    lpf = params["lpf_alpha"] if params["lpf_alpha"] is not None else 0.30
    out_lim = params["out_limit"] if params["out_limit"] is not None else 65.0
    base_spd = overall.get("base_speed", 0.5)
    hold = overall.get("hold_ms", 900)

    print(f"{Colors.BOLD}┌── 运行配置参数快照 ────────────────────────────────────────────────┐{Colors.END}")
    print(f"│  当前参数: Kp = {kp:<7.3f} Ki = {ki:<7.4f} Kd = {kd:<7.4f} FF = {ff:<7.3f}   │")
    print(f"│  系统物理: 差速输出限幅 = {out_lim:<5.1f}  低通滤波滤波 = {lpf:<5.2f}                   │")
    print(f"│  运行设置: 阶跃底速 = {base_spd:<4.2f} m/s  阶跃保持时长 = {hold:<5} ms                 │")
    print(f"└────────────────────────────────────────────────────────────────────┘\n")

    # Table Header
    print(f"{Colors.BOLD}┌── 阶跃响应阶梯测试详细指标 ──────────────────────────────────────────────────────────────────┐{Colors.END}")
    print(f"│ {Colors.UNDERLINE}阶跃阶段 (Step){Colors.END}   │ {Colors.UNDERLINE}目标/指令 (dps){Colors.END} │ {Colors.UNDERLINE}误差(err_abs){Colors.END} │ {Colors.UNDERLINE}过冲(Overshoot){Colors.END} │ {Colors.UNDERLINE}建立时间(Rise){Colors.END} │ {Colors.UNDERLINE}差速限幅率{Colors.END} │ {Colors.UNDERLINE}轮速误差 (L/R){Colors.END} │")
    print(f"├─────────────────┼────────────────┼──────────────┼────────────────┼────────────────┼────────────┼────────────────┤")

    for s in steps:
        target_str = f"{s['target']:+.1f}"
        cmd_str = f"{s['cmd']:+.1f}"
        
        # Calculate dynamics
        dyn = analyze_step_dynamics(s["idx"], s["cmd"], ts)
        overshoot_str = " - "
        rise_str = " - "
        
        if abs(s["cmd"]) > 5.0:
            overshoot_str = f"{dyn['overshoot_abs']:4.1f} ({dyn['overshoot_pct']:4.1f}%)"
            if not math.isnan(dyn['rise_time_ms']):
                rise_str = f"{dyn['rise_time_ms']:4.0f} ms"
            else:
                rise_str = " 无法建立"

        # Speed errors
        speed_err_str = f"{s['se_l']:4.2f} / {s['se_r']:4.2f}"
        cap_rate_str = f"{s['cap_rate']*100:4.1f}%"
        
        # Color codes based on errors
        err_color = Colors.GREEN
        if s["late_err_abs"] > 15.0:
            err_color = Colors.RED
        elif s["late_err_abs"] > 8.0:
            err_color = Colors.YELLOW
            
        cap_color = Colors.GREEN
        if s["cap_rate"] > 0.35:
            cap_color = Colors.RED
        elif s["cap_rate"] > 0.10:
            cap_color = Colors.YELLOW

        # format index name
        name = f"Step {s['idx']} "
        if s["target"] == 0.0:
            name += "(直行)"
        elif s["target"] > 0.0:
            name += "(左转)"
        else:
            name += "(右转)"

        print(f"│ {name:<15} │ {target_str:>5} / {cmd_str:<6} │ {err_color}{s['late_err_abs']:7.1f} dps{Colors.END} │ {overshoot_str:<14} │ {rise_str:<14} │ {cap_color}{cap_rate_str:<10}{Colors.END} │ {speed_err_str:<14} │")

    print(f"└─────────────────┴────────────────┴──────────────┴────────────────┴────────────────┴────────────┴────────────────┘")

def main():
    parser = argparse.ArgumentParser(description="Analyze T11 gyro loop step logs and recommend PID changes.")
    parser.add_argument("log_file", type=Path, help="Path to the T11 log file containing [T11] lines.")
    parser.add_argument("--override-kp", type=float, default=None, help="Override current kGyroPidP.")
    parser.add_argument("--override-kd", type=float, default=None, help="Override current kGyroPidD.")
    parser.add_argument("--override-ff", type=float, default=None, help="Override current kGyroFeedforwardGain.")

    args = parser.parse_args()

    print_banner()

    # Parse log
    params, step_summaries, time_series, overall_summary = parse_t11_log(args.log_file)

    if not step_summaries and not time_series:
        print(f"{Colors.RED}[错误] 未在日志中找到任何有效的 [T11] 或 [T11_STEP_SUMMARY] 数据行！{Colors.END}")
        print("请确保你保存了完整的 T11 测试终端输出，包含形如以下的行：")
        print("  [参数快照] gyro(...)")
        print("  [T11] idx=2 t=0.06 target=  35.0 ...")
        print("  [T11_STEP_SUMMARY] idx=2 target=35.0 ...")
        sys.exit(1)

    print(f"{Colors.GREEN}[成功] 读取到 {len(time_series)} 帧时序数据，共包含 {len(step_summaries)} 个阶跃测试阶段。{Colors.END}\n")

    # Apply overrides
    if args.override_kp is not None:
        params["kp"] = args.override_kp
    if args.override_kd is not None:
        params["kd"] = args.override_kd
    if args.override_ff is not None:
        params["ff"] = args.override_ff

    # Print dashboard
    print_dashboard(params, step_summaries, overall_summary, time_series)
    print()

    # Generate recommendations
    rec_kp, rec_kd, rec_ff, reasons, status = generate_recommendations(params, step_summaries, overall_summary, time_series)

    # Print recommendations card
    status_colors = {
        "WELL_TUNED": Colors.GREEN,
        "OPTIMAL": Colors.GREEN,
        "SLUGGISH": Colors.YELLOW,
        "OVERSHOOT": Colors.RED,
        "NOISY_OSCILLATION": Colors.RED,
        "OUTPUT_SATURATED": Colors.YELLOW,
        "SPEED_LOOP_WEAK": Colors.RED,
        "DIRECTION_MISMATCH": Colors.RED,
        "HARDWARE_FAULT": Colors.RED,
    }
    
    status_cn = {
        "WELL_TUNED": "内环极佳 (Well Tuned)",
        "OPTIMAL": "参数合适 (Optimal)",
        "SLUGGISH": "响应偏慢/不足 (Sluggish)",
        "OVERSHOOT": "角速度过冲/摆头 (Overshoot)",
        "NOISY_OSCILLATION": "高频抖动/震荡 (Noisy Oscillation)",
        "OUTPUT_SATURATED": "差速输出饱和 (Output Saturated)",
        "SPEED_LOOP_WEAK": "外底限制/轮速不跟 (Speed Loop Weak)",
        "DIRECTION_MISMATCH": "差速方向反向 (Direction Mismatch)",
        "HARDWARE_FAULT": "传感器硬件故障 (IMU Hardware Fault)",
    }

    color = status_colors.get(status, Colors.BLUE)
    cn_desc = status_cn.get(status, "未知状态")

    print(f"{Colors.BOLD}┌── AI 内环诊断与角速度 PID 推荐卡片 ──────────────────────────────┐{Colors.END}")
    print(f"│  内环健康状态: {color}{Colors.BOLD}{cn_desc:<46}{Colors.END} │")
    print(f"├──────────────────────────────────────────────────────────────────┤")
    print(f"│  诊断原因分析:                                                   │")
    for r in reasons:
        words = r.split()
        current_line = "│   - "
        for word in words:
            est_len = sum(2 if ord(c) > 127 else 1 for c in current_line + word)
            if est_len > 64:
                print(f"{current_line:<66}│")
                current_line = "│     " + word
            else:
                if current_line == "│   - " or current_line == "│     ":
                    current_line += word
                else:
                    current_line += " " + word
        if current_line.strip() != "│":
            actual_len = sum(2 if ord(c) > 127 else 1 for c in current_line)
            pad = 66 - actual_len
            print(f"{current_line}{' ' * pad}│")
            
    print(f"├──────────────────────────────────────────────────────────────────┤")
    
    curr_kp = params["kp"] if params["kp"] is not None else 0.24
    curr_kd = params["kd"] if params["kd"] is not None else 0.018
    curr_ff = params["ff"] if params["ff"] is not None else 0.12
    
    kp_diff = rec_kp - curr_kp
    kd_diff = rec_kd - curr_kd
    ff_diff = rec_ff - curr_ff
    
    kp_diff_str = f"({kp_diff:+.3f})" if kp_diff != 0 else " (持平)"
    kd_diff_str = f"({kd_diff:+.3f})" if kd_diff != 0 else " (持平)"
    ff_diff_str = f"({ff_diff:+.3f})" if ff_diff != 0 else " (持平)"
    
    print(f"│  {Colors.BOLD}推荐下一轮参数:{Colors.END}                                                 │")
    print(f"│    {Colors.BOLD}{Colors.BLUE}kGyroPidP = {rec_kp:<7.3f}{Colors.END} {kp_diff_str:<12} (当前 P = {curr_kp:.3f})            │")
    print(f"│    {Colors.BOLD}{Colors.BLUE}kGyroPidD = {rec_kd:<7.3f}{Colors.END} {kd_diff_str:<12} (当前 D = {curr_kd:.3f})            │")
    print(f"│    {Colors.BOLD}{Colors.BLUE}kGyroFeedforwardGain = {rec_ff:<7.3f}{Colors.END} {ff_diff_str:<8} (当前 FF = {curr_ff:.3f})      │")
    print(f"└──────────────────────────────────────────────────────────────────┘")
    
    print(f"\n{Colors.BOLD}建议操作步骤:{Colors.END}")
    print(f" 1. 运行板载阶跃测试，在空旷路面上执行 {Colors.BOLD}T11 角速度内环阶跃测试{Colors.END}。")
    print(f" 2. 重新抓取日志进行下一轮迭代，直到评估状态变为 {Colors.GREEN}WELL_TUNED (内环极佳){Colors.END}。")
    print(f" 3. 内环稳定收敛后，将最终值写入 {Colors.BOLD}[car_config.hpp](file:///home/zoumaguanbei/SmartCar_Project/Loongson_2k300_301_Library-master/Loongson_2K300_301_LIB/user_app/include/car_config.hpp){Colors.END} 中对应的 `kGyroPidP`、`kGyroPidD`、`kGyroFeedforwardGain`。")
    print(f" 4. 内环调试通过后，即可直接进入 {Colors.BOLD}T06/T07{Colors.END} 调试视觉巡线外环和提速。")

if __name__ == "__main__":
    main()
