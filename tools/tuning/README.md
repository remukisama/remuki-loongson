# 赛道调参工具

三个脚本均只使用 Python 标准库，可在上位机或开发电脑运行。

```bash
python3 tools/tuning/analyze_t05_actual_log.py logs/<T05日志>.log
python3 tools/tuning/analyze_t11_gyro_log.py logs/<T11日志>.log
python3 tools/tuning/analyze_t06_curve_log.py logs/<T06日志>.log
```

## 使用顺序

1. 固定电池电量、轮胎状态、摄像头高度、赛道、光照和测试速度区间。
2. T05 先测试 `0.20~0.60 m/s`，再测试 `0.45~0.90 m/s`；只在报告允许时复制 `DIRECT_REPLACE_SPEED_PID` 到 `car_config.hpp`。
3. T11 只在 T05 通过后运行；只在报告允许时复制 `DIRECT_REPLACE_GYRO_PID`。
4. T06 使用 `analyze_t06_curve_log.py` 判定故障属于图像、外环、角速度环、物理差速权限还是轮速执行。每一轮只处理最高优先级一项。

## 历史记录

T05/T11 默认在当前目录分别生成 `t05_tuning_history.csv` 和 `t11_tuning_history.csv`。同一速度区间的日志才能横向比较；不要把不同赛道、不同电量或不同速度带的评分混在一起。

## 安全规则

- 输出 `DATA_INVALID`、`PWM_SATURATED`、`LEFT_RIGHT_ASYMMETRY`、`PHYSICAL_DIFF_LIMIT`、`IMU_INPUT_FAULT` 或 `DIRECTION_MISMATCH` 时，不要替换 PID 参数。
- PID 推荐是一次实车迭代的保守候选，不是脱离实测数据的“全局最优值”。
- T06 弯道修复必须先以低速验证：T06 连续通过后才运行 T07 提速。
