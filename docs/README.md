# 文档索引

三层：**说明文档**（怎么写代码）→ **专题指南**（怎么调、怎么构建）→ **原始资料**（芯片与板卡手册）。

## 一、项目说明（仓库根）

| 文件 | 内容 |
| --- | --- |
| `../README.md` | 总入口：结构、快速开始、命令速查 |
| `../PROJECT_STRUCTURE.md` | 架构、安全机制、引脚映射、部署顺序 |
| `../PROJECT_USAGE_GUIDE.md` | 端到端使用流程 |

## 二、专题指南（`guides/`）

| 文件 | 用在什么时候 |
| --- | --- |
| `guides/BUILD_SH_USAGE.md` | `build.sh` 参数、构建流程、报错对照 |
| `guides/PARAMETER_TUNING_METHODS.md` | PID 与各参数怎么整定 |
| `guides/CAR_ALGORITHM_TUNING_GUIDE.md` | 视觉与算法侧调整思路 |
| `guides/SPEED_LOOP_TRACK_TUNING_PLAN.md` | 速度环与赛道专项验证计划 |
| `guides/PROJECT_OPTIMIZATION_SUGGESTIONS.md` | 已知可优化项 |

## 三、版本与重构记录（`notes/`）

| 文件 | 内容 |
| --- | --- |
| `notes/更新日志.md` | 车程序版本变更 |
| `notes/RELEASE_NOTES_v2026.07.14-small-ring-race.md` | 小环岛版本发布说明 |
| `notes/line_reconstruction_refactor_20260606.md` | 循线重构记录 |
| `notes/Version_V2.1.0.txt` | 官方 SDK 版本说明 |
| `notes/matlab_autotune/` | MATLAB 自整定相关材料 |

## 四、原始资料

| 目录 | 内容 |
| --- | --- |
| `datasheets/` | 2K0300 数据手册、用户手册（PDF） |
| `../vendor/loongson_sdk_V2.1.0/docs/` | 官方 SDK 文档：GPIO / PWM / 编码器 / I2C / CANFD / ncnn / TFT 屏 使用手册 |

> 注意：`vendor/` 里是官方原始包，`libraries/` 是裁剪后的实际使用版本，两者内容不完全一致，排查问题时以 `vendor/` 为准、以 `libraries/` 为实际行为。
