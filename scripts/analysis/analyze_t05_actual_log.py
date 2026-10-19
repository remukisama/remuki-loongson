#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SmartCar T05 (Actual Speed Loop Straight) Log Analyzer and PID Tuner
Analyzes T05_ACTUAL log data to recommend optimal Kp and Ki values.
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
     SmartCar T05 Speed Loop Log Analyzer & PID Recommendation Tool
======================================================================{Colors.END}
"""
    print(banner)

def parse_t05_log(log_path: Path) -> tuple[dict[str, any], list[dict[str, float]]]:
    """
    Parses T05 log file, extracting:
    1. Initial parameter configuration (min, max, Kp, Ki, period, duration)
    2. Time series of T05_ACTUAL frames (t_ms, target, encL, encR, errL, errR, pwmL, pwmR)
    """
    if not log_path.exists():
        print(f"{Colors.RED}[Error] Log file not found: {log_path}{Colors.END}")
        sys.exit(1)

    try:
        content = log_path.read_text(encoding='utf-8', errors='ignore')
    except Exception as e:
        print(f"{Colors.RED}[Error] Failed to read log file: {e}{Colors.END}")
        sys.exit(1)

    metadata = {
        "kp": None,
        "ki": None,
        "min_speed": None,
        "max_speed": None,
        "period_s": None,
        "duration_ms": None,
    }
    
    # Try parsing parameters from the execution start line, e.g.:
    # >>> [执行] min=0.90 max=1.30 amplitude=0.40 period=3.0s duration=4000ms Kp=2000 Ki=40
    exec_pattern = re.compile(
        r"\[执行\].*?min=([\d\.]+).*?max=([\d\.]+).*?period=([\d\.]+)s.*?duration=(\d+)ms.*?Kp=([\d\.]+).*?Ki=([\d\.]+)"
    )
    
    # Also support backup pattern for current status line:
    # >>> [T05] 当前 Kp=2000/2000 Ki=40/40
    current_pid_pattern = re.compile(
        r"\[T05\] 当前 Kp=([\d\.]+)/([\d\.]+)\s+Ki=([\d\.]+)/([\d\.]+)"
    )

    for line in content.splitlines():
        exec_match = exec_pattern.search(line)
        if exec_match:
            metadata["min_speed"] = float(exec_match.group(1))
            metadata["max_speed"] = float(exec_match.group(2))
            metadata["period_s"] = float(exec_match.group(3))
            metadata["duration_ms"] = int(exec_match.group(4))
            metadata["kp"] = float(exec_match.group(5))
            metadata["ki"] = float(exec_match.group(6))
            continue
            
        pid_match = current_pid_pattern.search(line)
        if pid_match and metadata["kp"] is None:
            metadata["kp"] = float(pid_match.group(1))
            metadata["ki"] = float(pid_match.group(3))

    # Parse actual data rows:
    # Format: T05_ACTUAL t_ms,target,encL,encR,errL,errR,pwmL,pwmR
    rows = []
    for line in content.splitlines():
        line = line.strip()
        if line.startswith("T05_ACTUAL "):
            data_str = line[11:]  # strip "T05_ACTUAL "
            parts = data_str.split(",")
            if len(parts) >= 8:
                try:
                    row = {
                        "t_ms": float(parts[0]),
                        "target": float(parts[1]),
                        "encL": float(parts[2]),
                        "encR": float(parts[3]),
                        "errL": float(parts[4]),
                        "errR": float(parts[5]),
                        "pwmL": float(parts[6]),
                        "pwmR": float(parts[7]),
                    }
                    rows.append(row)
                except ValueError:
                    continue  # skip malformed data lines

    # Sort rows by timestamp just in case
    rows.sort(key=lambda r: r["t_ms"])
    return metadata, rows

def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0

def rmse(errors: list[float]) -> float:
    return math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else 0.0

def calculate_phase_lag(target: list[float], actual: list[float], dt_ms: float = 20.0) -> float:
    """
    Finds the time-shift (in ms) of the actual speed that minimizes RMSE with the target speed.
    This represents the phase lag / delay of the system response.
    """
    if len(target) < 10 or len(actual) < 10:
        return 0.0
        
    best_shift = 0
    min_error = float('inf')
    
    # Check shifts from 0 to 15 samples (0ms to 300ms delay at 20ms intervals)
    max_shifts = min(15, len(target) // 3)
    
    for shift in range(max_shifts):
        shifted_actual = actual[shift:]
        trimmed_target = target[:len(target) - shift]
        
        errors = [t - a for t, a in zip(trimmed_target, shifted_actual)]
        err_rmse = rmse(errors)
        
        if err_rmse < min_error:
            min_error = err_rmse
            best_shift = shift
            
    return best_shift * dt_ms

def analyze_wheel_performance(
    target: list[float], 
    actual: list[float], 
    pwm: list[float], 
    t_ms: list[float],
    max_pwm_limit: float = 8000.0
) -> dict[str, any]:
    """
    Analyzes step tracking performance for a single wheel.
    """
    if not target or not actual:
        return {}

    errors = [t - a for t, a in zip(target, actual)]
    abs_errors = [abs(e) for e in errors]
    
    # Calculate target range & amplitude
    min_tgt, max_tgt = min(target), max(target)
    amp = max_tgt - min_tgt
    
    # Calculate overshoot
    max_act = max(actual)
    overshoot_abs = max_act - max_tgt
    overshoot_pct = (overshoot_abs / amp * 100) if amp > 0.1 else 0.0
    if overshoot_abs < 0:
        overshoot_abs = 0.0
        overshoot_pct = 0.0

    # Calculate PWM saturation rate (PWM at >= 95% of limit)
    saturation_count = sum(1 for p in pwm if abs(p) >= max_pwm_limit * 0.95)
    saturation_rate = (saturation_count / len(pwm)) * 100
    
    # Calculate phase lag (responsiveness delay)
    phase_lag_ms = calculate_phase_lag(target, actual, dt_ms=20.0)
    
    # Calculate high-frequency oscillation of PWM (high-frequency jitter)
    pwm_diffs = [abs(pwm[i] - pwm[i-1]) for i in range(1, len(pwm))]
    mean_pwm_jitter = mean(pwm_diffs) if pwm_diffs else 0.0

    # Calculate steady-state tracking error (when target speed is near peak or stable)
    # We define steady-state as regions where target is at >= 85% of peak amplitude
    stable_indices = [i for i, t in enumerate(target) if t >= min_tgt + 0.85 * amp]
    steady_state_error = mean([errors[i] for i in stable_indices]) if stable_indices else mean(errors)

    # Count direction flips (zero crossings of speed error derivative) to find oscillations
    error_diffs = [errors[i] - errors[i-1] for i in range(1, len(errors))]
    flips = 0
    for i in range(1, len(error_diffs)):
        if (error_diffs[i] > 0.01 and error_diffs[i-1] < -0.01) or (error_diffs[i] < -0.01 and error_diffs[i-1] > 0.01):
            flips += 1

    return {
        "rmse": rmse(errors),
        "mae": mean(abs_errors),
        "max_speed": max_act,
        "overshoot_abs": overshoot_abs,
        "overshoot_pct": overshoot_pct,
        "saturation_rate": saturation_rate,
        "phase_lag_ms": phase_lag_ms,
        "pwm_jitter": mean_pwm_jitter,
        "steady_state_error": steady_state_error,
        "oscillation_flips": flips,
    }

def generate_recommendations(
    metadata: dict[str, any], 
    perf_l: dict[str, any], 
    perf_r: dict[str, any]
) -> tuple[float, float, list[str], str]:
    """
    Analyzes performance metrics for both wheels and recommends new Kp, Ki.
    Returns: (recommended_kp, recommended_ki, reasons_list, status_summary)
    """
    # Fallback to defaults if metadata couldn't extract current PID
    curr_kp = metadata["kp"] if metadata["kp"] is not None else 1800.0
    curr_ki = metadata["ki"] if metadata["ki"] is not None else 35.0
    
    reasons = []
    
    # Average metrics of both wheels for decision making
    avg_overshoot_pct = (perf_l["overshoot_pct"] + perf_r["overshoot_pct"]) / 2
    avg_overshoot_abs = (perf_l["overshoot_abs"] + perf_r["overshoot_abs"]) / 2
    avg_lag = (perf_l["phase_lag_ms"] + perf_r["phase_lag_ms"]) / 2
    avg_sat = (perf_l["saturation_rate"] + perf_r["saturation_rate"]) / 2
    avg_jitter = (perf_l["pwm_jitter"] + perf_r["pwm_jitter"]) / 2
    avg_sse = (perf_l["steady_state_error"] + perf_r["steady_state_error"]) / 2
    avg_rmse = (perf_l["rmse"] + perf_r["rmse"]) / 2

    next_kp = curr_kp
    next_ki = curr_ki
    status = "OPTIMAL"

    # Rule 1: Safety/Stuck Check
    # If the target speed is positive but the wheels are barely moving:
    avg_speed = (perf_l["max_speed"] + perf_r["max_speed"]) / 2
    if metadata["max_speed"] and metadata["max_speed"] > 0.5 and avg_speed < 0.15:
        reasons.append("小车轮速几乎为零，电机未正常旋转！请检查物理卡滞、供电电压、电机驱动或编码器接线。")
        return curr_kp, curr_ki, reasons, "HARDWARE_FAULT"

    # Rule 2: Oscillation / Jitter (High Kp)
    # High-frequency PWM oscillations mean Kp is too aggressive
    if avg_jitter > 450.0:
        next_kp = curr_kp * 0.80
        next_ki = curr_ki * 0.90
        reasons.append(f"检测到 PWM 存在高频剧烈震荡 (抖动量 {avg_jitter:.1f} > 450)，表明比例系数 Kp 过高。")
        status = "HIGH_OSCILLATION"
    
    # Rule 3: Significant Overshoot (High Kp / High Ki causing windup)
    elif avg_overshoot_pct > 12.0 or avg_overshoot_abs > 0.12:
        # Decrease Kp and Ki
        if avg_overshoot_pct > 20.0 or avg_overshoot_abs > 0.20:
            next_kp = curr_kp * 0.80
            next_ki = curr_ki * 0.70
            reasons.append(f"速度严重过冲 (左 {perf_l['overshoot_pct']:.1f}% / 右 {perf_r['overshoot_pct']:.1f}%)，建议大幅降低 Kp 和 Ki。")
        else:
            next_kp = curr_kp * 0.88
            next_ki = curr_ki * 0.80
            reasons.append(f"存在中等速度过冲 (左 {perf_l['overshoot_pct']:.1f}% / 右 {perf_r['overshoot_pct']:.1f}%)，建议小幅调低 Kp 和 Ki。")
        status = "OVERSHOOT"

    # Rule 4: Saturation Limit
    # If the PWM is heavily saturated, increasing PID won't help (it only winds up the integrator)
    elif avg_sat > 20.0:
        # Keep PID or slightly reduce Ki to prevent windup
        next_kp = curr_kp
        next_ki = curr_ki * 0.85
        reasons.append(f"电机输出高度饱和 (饱和率 {avg_sat:.1f}% > 20%)，表明电机已达物理动力极限。调大 PID 无济于事，且易导致积分饱和。建议检查电池电量、机械阻力，或降低目标速度。")
        status = "PWM_SATURATED"

    # Rule 5: Sluggish Response / Phase Lag (Low Kp)
    elif avg_lag > 120.0 or (avg_sse > 0.08 and avg_sat < 12.0):
        # We have lag, and we are NOT saturated, so we can increase PID safely
        if avg_lag > 180.0 or avg_sse > 0.15:
            next_kp = curr_kp * 1.18
            next_ki = curr_ki * 1.25
            reasons.append(f"速度跟踪严重滞后 (平均延迟 {avg_lag:.0f}ms，稳态误差 {avg_sse:.3f}m/s)，且动力充足。建议大幅提升 Kp 和 Ki。")
        else:
            next_kp = curr_kp * 1.10
            next_ki = curr_ki * 1.15
            reasons.append(f"速度响应偏慢 (平均延迟 {avg_lag:.0f}ms，稳态误差 {avg_sse:.3f}m/s)。建议小幅增加 Kp 和 Ki。")
        status = "SLUGGISH"

    # Rule 6: High Steady State Error but fast rise (Low Ki)
    elif abs(avg_sse) > 0.05 and avg_sat < 12.0:
        next_ki = curr_ki * 1.20
        reasons.append(f"稳态跟踪误差偏大 ({avg_sse:.3f} m/s)，但响应速度尚可。建议单独调高积分项 Ki 以消除静差。")
        status = "STEADY_STATE_ERROR"

    # Rule 7: Well tuned
    else:
        reasons.append("速度环响应迅速且无明显过冲，波形跟踪极佳！当前参数非常合适，建议保持。")
        status = "WELL_TUNED"

    # Clamp parameters to safe boundaries
    next_kp = max(500.0, min(5000.0, next_kp))
    next_ki = max(5.0, min(150.0, next_ki))

    # Round to reasonable precision
    return round(next_kp, 1), round(next_ki, 1), reasons, status

def print_dashboard(metadata: dict[str, any], perf_l: dict[str, any], perf_r: dict[str, any]):
    """
    Prints a beautiful, informative summary dashboard.
    """
    # 1. Print configuration summary
    kp_str = f"{metadata['kp']:.0f}" if metadata['kp'] is not None else "未检测到 (默认1800)"
    ki_str = f"{metadata['ki']:.0f}" if metadata['ki'] is not None else "未检测到 (默认35)"
    tgt_range = f"{metadata['min_speed']:.2f} ~ {metadata['max_speed']:.2f} m/s" if metadata['min_speed'] is not None else "未知"
    period = f"{metadata['period_s']:.1f} s" if metadata['period_s'] is not None else "未知"
    duration = f"{metadata['duration_ms']} ms" if metadata['duration_ms'] is not None else "未知"

    print(f"{Colors.BOLD}┌── 运行配置信息 ──────────────────────────────────────────────────┐{Colors.END}")
    print(f"│  当前参数: Kp = {kp_str:<12} Ki = {ki_str:<12}                       │")
    print(f"│  目标速度: {tgt_range:<20} 周期 = {period:<10} 时长 = {duration:<10}  │")
    print(f"└──────────────────────────────────────────────────────────────────┘\n")

    # 2. Print performance comparison table
    print(f"{Colors.BOLD}┌── 左右轮性能指标对比 ────────────────────────────────────────────┐{Colors.END}")
    print(f"│ {Colors.UNDERLINE}指标名称{Colors.END}                   │ {Colors.BOLD}{Colors.BLUE}左轮 (Left){Colors.END}        │ {Colors.BOLD}{Colors.GREEN}右轮 (Right){Colors.END}       │ 评估结果     │")
    print(f"├──────────────────────────┼───────────────────┼──────────────────┼──────────────┤")
    
    # MAE
    print(f"│ 平均绝对误差 (MAE)       │ {perf_l['mae']:7.3f} m/s       │ {perf_r['mae']:7.3f} m/s      │ " + 
          (f"{Colors.YELLOW}偏大{Colors.END}" if (perf_l['mae'] + perf_r['mae'])/2 > 0.1 else f"{Colors.GREEN}优秀{Colors.END}") + "         │")
    
    # RMSE
    print(f"│ 均方根误差 (RMSE)        │ {perf_l['rmse']:7.3f} m/s       │ {perf_r['rmse']:7.3f} m/s      │ " +
          (f"{Colors.YELLOW}偏大{Colors.END}" if (perf_l['rmse'] + perf_r['rmse'])/2 > 0.12 else f"{Colors.GREEN}优秀{Colors.END}") + "         │")
    
    # Peak Overshoot
    print(f"│ 最大过冲量 (Overshoot)   │ {perf_l['overshoot_abs']:5.3f} m/s ({perf_l['overshoot_pct']:4.1f}%) │ {perf_r['overshoot_abs']:5.3f} m/s ({perf_r['overshoot_pct']:4.1f}%) │ " +
          (f"{Colors.RED}超标{Colors.END}" if max(perf_l['overshoot_pct'], perf_r['overshoot_pct']) > 12.0 else f"{Colors.GREEN}正常{Colors.END}") + "         │")
    
    # Phase Lag
    print(f"│ 响应延迟 (Phase Lag)     │ {perf_l['phase_lag_ms']:5.0f} ms         │ {perf_r['phase_lag_ms']:5.0f} ms        │ " +
          (f"{Colors.YELLOW}迟缓{Colors.END}" if max(perf_l['phase_lag_ms'], perf_r['phase_lag_ms']) > 120 else f"{Colors.GREEN}灵敏{Colors.END}") + "         │")
    
    # PWM Saturation
    print(f"│ PWM 饱和率 (Saturation)  │ {perf_l['saturation_rate']:5.1f} %          │ {perf_r['saturation_rate']:5.1f} %         │ " +
          (f"{Colors.RED}过载{Colors.END}" if max(perf_l['saturation_rate'], perf_r['saturation_rate']) > 15.0 else f"{Colors.GREEN}充沛{Colors.END}") + "         │")
    
    # PWM Jitter
    print(f"│ PWM 均值变化率 (Jitter)  │ {perf_l['pwm_jitter']:5.1f}           │ {perf_r['pwm_jitter']:5.1f}          │ " +
          (f"{Colors.RED}震荡{Colors.END}" if max(perf_l['pwm_jitter'], perf_r['pwm_jitter']) > 450 else f"{Colors.GREEN}平稳{Colors.END}") + "         │")
    
    print(f"└──────────────────────────┴───────────────────┴──────────────────┴──────────────┘")

    # 3. Print Asymmetry Warn if severe
    speed_diff = abs(mean([perf_l["max_speed"]]) - mean([perf_r["max_speed"]]))
    pwm_sat_diff = abs(perf_l["saturation_rate"] - perf_r["saturation_rate"])
    
    if speed_diff > 0.08 or pwm_sat_diff > 15.0:
        print(f"\n{Colors.YELLOW}{Colors.BOLD}[警告] 检测到左右轮严重的物理机械/电气不对称性！{Colors.END}")
        if speed_diff > 0.08:
            print(f"  * 轮速差异：左右轮最大速度差达 {speed_diff:.3f} m/s。")
        if pwm_sat_diff > 15.0:
            print(f"  * 动力差异：左轮饱和率 {perf_l['saturation_rate']:.1f}%，右轮饱和率 {perf_r['saturation_rate']:.1f}%，一侧电机明显更费力。")
        print(f"  {Colors.YELLOW}建议检查：轮胎磨损、地面轴承摩擦、联轴器是否打滑、齿轮啮合间隙，或左右电机接线接触电阻。{Colors.END}")

def main():
    parser = argparse.ArgumentParser(description="Analyze T05 speed loop logs and recommend PID changes.")
    parser.add_argument("log_file", type=Path, help="Path to the T05 log file containing T05_ACTUAL lines.")
    parser.add_argument("--max-pwm", type=float, default=8000.0, help="Maximum PWM limit configured (default: 8000.0)")
    parser.add_argument("--override-kp", type=float, default=None, help="Override current Kp value if not found in log.")
    parser.add_argument("--override-ki", type=float, default=None, help="Override current Ki value if not found in log.")
    
    args = parser.parse_args()
    
    print_banner()
    
    # Parse log file
    metadata, rows = parse_t05_log(args.log_file)
    
    if not rows:
        print(f"{Colors.RED}[错误] 未在日志中找到任何以 'T05_ACTUAL' 开头的有效数据行！{Colors.END}")
        print("请确保你复制并保存了小车直线运行时的输出，格式形如：")
        print("  T05_ACTUAL 20,0.900,0.210,0.220,0.690,0.680,120,130")
        sys.exit(1)
        
    print(f"{Colors.GREEN}[成功] 读取到 {len(rows)} 帧 T05 速度波形数据。数据覆盖时长: {(rows[-1]['t_ms'] - rows[0]['t_ms']):.0f} ms。{Colors.END}\n")

    # Apply overrides if provided
    if args.override_kp is not None:
        metadata["kp"] = args.override_kp
    if args.override_ki is not None:
        metadata["ki"] = args.override_ki

    # Extract time series for Left and Right
    target = [r["target"] for r in rows]
    enc_l = [r["encL"] for r in rows]
    enc_r = [r["encR"] for r in rows]
    pwm_l = [r["pwmL"] for r in rows]
    pwm_r = [r["pwmR"] for r in rows]
    t_ms = [r["t_ms"] for r in rows]

    # Analyze performance
    perf_l = analyze_wheel_performance(target, enc_l, pwm_l, t_ms, args.max_pwm)
    perf_r = analyze_wheel_performance(target, enc_r, pwm_r, t_ms, args.max_pwm)

    # Print Dashboard
    print_dashboard(metadata, perf_l, perf_r)
    print()

    # Generate Recommendations
    rec_kp, rec_ki, reasons, status = generate_recommendations(metadata, perf_l, perf_r)

    # Print Recommendations Card
    status_colors = {
        "WELL_TUNED": Colors.GREEN,
        "OPTIMAL": Colors.GREEN,
        "SLUGGISH": Colors.YELLOW,
        "STEADY_STATE_ERROR": Colors.YELLOW,
        "OVERSHOOT": Colors.RED,
        "HIGH_OSCILLATION": Colors.RED,
        "PWM_SATURATED": Colors.YELLOW,
        "HARDWARE_FAULT": Colors.RED,
    }
    
    status_cn = {
        "WELL_TUNED": "参数极佳 (Well Tuned)",
        "OPTIMAL": "参数合适 (Optimal)",
        "SLUGGISH": "响应迟缓 (Sluggish)",
        "STEADY_STATE_ERROR": "存在稳态误差 (Steady State Error)",
        "OVERSHOOT": "速度过冲 (Overshoot)",
        "HIGH_OSCILLATION": "高频震荡 (High Oscillation)",
        "PWM_SATURATED": "动力饱和 (PWM Saturated)",
        "HARDWARE_FAULT": "硬件故障/卡滞 (Hardware Fault)",
    }

    color = status_colors.get(status, Colors.BLUE)
    cn_desc = status_cn.get(status, "未知状态")

    print(f"{Colors.BOLD}┌── AI 诊断与 PID 参数推荐卡片 ────────────────────────────────────┐{Colors.END}")
    print(f"│  系统状态评估: {color}{Colors.BOLD}{cn_desc:<50}{Colors.END} │")
    print(f"├──────────────────────────────────────────────────────────────────┤")
    print(f"│  诊断原因分析:                                                   │")
    for r in reasons:
        # Word wrap reasons simple implementation (assuming lines max 62 chars)
        words = r.split()
        current_line = "│   - "
        for word in words:
            # Simple check for Chinese characters since they are multi-byte
            # A rough approximation of character width
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
            # format spacing
            actual_len = sum(2 if ord(c) > 127 else 1 for c in current_line)
            pad = 66 - actual_len
            print(f"{current_line}{' ' * pad}│")
            
    print(f"├──────────────────────────────────────────────────────────────────┤")
    
    curr_kp = metadata["kp"] if metadata["kp"] is not None else 1800.0
    curr_ki = metadata["ki"] if metadata["ki"] is not None else 35.0
    
    kp_diff = rec_kp - curr_kp
    ki_diff = rec_ki - curr_ki
    
    kp_diff_str = f"(+{-kp_diff:.1f})" if kp_diff < 0 else (f"(+{kp_diff:.1f})" if kp_diff > 0 else " (持平)")
    # fix minus sign display
    if kp_diff < 0:
        kp_diff_str = f"({kp_diff:.1f})"
        
    ki_diff_str = f"({ki_diff:.1f})" if ki_diff < 0 else (f"(+{ki_diff:.1f})" if ki_diff > 0 else " (持平)")
    
    # Highlight recommended values
    print(f"│  {Colors.BOLD}推荐下一轮参数:{Colors.END}                                                 │")
    print(f"│    {Colors.BOLD}{Colors.BLUE}Kp = {rec_kp:<10}{Colors.END} {kp_diff_str:<12} (当前 Kp = {curr_kp:.1f})             │")
    print(f"│    {Colors.BOLD}{Colors.BLUE}Ki = {rec_ki:<10}{Colors.END} {ki_diff_str:<12} (当前 Ki = {curr_ki:.1f})             │")
    print(f"└──────────────────────────────────────────────────────────────────┘")
    
    print(f"\n{Colors.BOLD}建议操作步骤:{Colors.END}")
    print(f" 1. 运行板载测试程序并选择 {Colors.BOLD}T05 速度环实地直线测试{Colors.END}。")
    print(f" 2. 在串口终端输入调整参数：将 Kp 改为 {Colors.GREEN}{rec_kp:.0f}{Colors.END}，Ki 改为 {Colors.GREEN}{rec_ki:.0f}{Colors.END}。")
    print(f" 3. 重新抓取日志进行下一轮迭代，直到评估状态为 {Colors.GREEN}WELL_TUNED (参数极佳){Colors.END}。")
    print(f" 4. 参数稳定收敛后，将最终值写入 {Colors.BOLD}[car_config.hpp](file:///home/zoumaguanbei/SmartCar_Project/Loongson_2k300_301_Library-master/Loongson_2K300_301_LIB/user_app/include/car_config.hpp){Colors.END} 中的 `kSpeedPidP` 和 `kSpeedPidI`。")

if __name__ == "__main__":
    main()
