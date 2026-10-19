# remuki-loongson

龙芯 **2K0300 / 2K0301**（LoongArch64 + Linux）摄像头循迹智能车工程。

视觉偏差 → 多层 PID → 差速转向，配套独立电机看门狗、NCNN 端侧推理与一套现场调参工具链。

| | |
| --- | --- |
| 目标平台 | Loongson 2K0300 / 2K0301，LoongArch64，Linux 用户态 |
| 语言 / 构建 | C++14 / C，CMake 3.16+，交叉编译 |
| 构建主机 | **仅 Linux x86_64**（见「环境要求」）|
| 主分支 | `main`（保护，只接受 PR）；日常开发在 `dev` |
| 参赛 | 第二十届、二十一届 全国大学生智能汽车竞赛 华北赛区三等奖 |

---

## 1. 三分钟上手

```bash
# ① 环境自检（缺什么它会直接告诉你）
bash scripts/check_env.sh

# ② 编看门狗（先于车程序部署，车程序启动时会检查它是否在线）
cd gpio_proj && ./build.sh manager 192.168.1.100

# ③ 编车程序并跑到板子上
cd ../main && ./build.sh 192.168.1.100 -r
```

板端开机自启（一次性）：

```bash
scp -r board/deploy/autoboot root@192.168.1.100:/root/
ssh root@192.168.1.100 'cd /root/autoboot && ./install.sh && systemctl status gpio-manager --no-pager'
```

---

## 2. 目录结构

顶层只保留「能编的东西」和「给它用的东西」，文档、板级资源、实验数据各自独立成区。

```text
remuki-loongson/
├── main/                    车程序入口：main.cpp、CMakeLists、build.sh（编译+传板+远程运行）
├── user_app/                业务代码：图像、级联 PID、IMU、编码器、NCNN 推理、环岛控制
├── libraries/               官方库裁剪版（app / common / drv），只留本车实际用到的模块
├── example/                 官方例程 + 现场调参 demo（部分编入主程序作运行时辅助）
├── driver/                  内核模块：I2C 传感器、编码器中断、TFT/IPS 屏，含部署脚本
├── gpio_proj/               电机看门狗守护进程 gpio_manager（独立 CMake 工程）
├── board/                   板级资源
│   ├── kernel/              板卡内核 vmlinuz（2026.04.20，修复 I2C 卡顿）+ 刷写说明
│   ├── deploy/autoboot/     开机自启安装包（gpio-manager.service）
│   └── tools/               板端 NCNN 抓图/推理小工具（板端编译）
├── scripts/
│   ├── check_env.sh         环境自检（主机工具链、依赖库、板卡连通性）
│   └── analysis/            离线日志分析（速度环、陀螺仪、实测抓包）
├── docs/                    文档（见 docs/README.md）
├── debug/                   现场联调脚本与采集数据（实验产物，不参与构建）
├── vendor/                  官方 SDK V2.1.0 原始包，只作参考，不进构建
├── tools/                   交叉工具链与依赖库（不入库，见 tools/README.md）
├── README.md                本文件
├── PROJECT_STRUCTURE.md     架构与安全逻辑详解
└── PROJECT_USAGE_GUIDE.md   完整使用流程
```

**只记三条**：要改车的行为 → `user_app/`；要改构建 → `main/`；要动安全兜底 → `gpio_proj/`。

---

## 3. 构建与部署

### 环境要求

- **Linux x86_64 主机**（Ubuntu 20.04+）。Windows 不行：工具链是 LoongArch 的 Linux ELF 程序，本机 WSL 也被安全策略禁用。
- `tools/` 下需自备：龙芯工具链 + `LQ_Dep_libs/{opencv,ncnn,ffmpeg}_install`。缺什么对照 `tools/README.md`，`build.sh` 首次运行会自动解压并生成 `main/toolchain_path.cmake`。
- 主机装 `cmake pkg-config`，能 `ssh/scp` 到板卡。

### 常用命令

```bash
cd main
./build.sh                            # 仅编译
./build.sh --clean                    # 全量重编（删 build）
./build.sh 192.168.1.100              # 编译 + scp 传板
./build.sh 192.168.1.100 -r           # 编译 + 传板 + 运行，默认 --mode=car
APP_ARGS="--mode=tune" ./build.sh 192.168.1.100 -r

cd gpio_proj
./build.sh manager                    # 只编守护进程
./build.sh test                       # 编联调测试程序
./build.sh manager 192.168.1.100 run  # 编译 + 传板 + 运行
```

`main` 支持的运行模式：`car` / `tune` / `encoder-test` / `encoder512` / `vision-debug` / `speed-tune` / `speed-return`。

### 部署顺序（固定）

```
gpio_manager 编译安装 → gpio-manager.service 启动 → 确认在线 → 车程序
```

`--mode=car` 会强制检查看门狗；`--unsafe-dev-mode` 只允许台架诊断，**禁止上赛道**。

---

## 4. 安全机制（改代码前必读）

车控线程每 10 ms 向 `/gpio_proj_shm` 写心跳；`gpio_manager` 每 1 ms 检查，**50 ms 无心跳立即安全停车**：

```text
ATIM CCR1/CCR2 = ARR+1（低有效驱动保持使能）
GPIO21..22 拉低、GTIM CCR2 = 0、GPIO61 拉低
```

电机驱动是**低有效**，紧急停车时禁用 PWM 会让输入悬空，所以实现是「保持使能 + 占空比 0」。**不要把这段改成 disable。**

车程序自身还有 `emergency_stop()`；控制线程会在视觉数据过期、看门狗丢失、编码器反馈异常/反向、持续高 PWM 但轮子不转时主动停车。

细节见 `PROJECT_STRUCTURE.md`。

---

## 5. 文档地图

| 想解决什么 | 去哪 |
| --- | --- |
| 完整文档索引 | `docs/README.md` |
| 架构、引脚映射、部署顺序 | `PROJECT_STRUCTURE.md` |
| 端到端使用流程 | `PROJECT_USAGE_GUIDE.md` |
| `build.sh` 全部参数与排错 | `docs/guides/BUILD_SH_USAGE.md` |
| 参数怎么调、调多少 | `docs/guides/PARAMETER_TUNING_METHODS.md` |
| 算法侧调整思路 | `docs/guides/CAR_ALGORITHM_TUNING_GUIDE.md` |
| 速度环 / 赛道专项 | `docs/guides/SPEED_LOOP_TRACK_TUNING_PLAN.md` |
| 待优化清单 | `docs/guides/PROJECT_OPTIMIZATION_SUGGESTIONS.md` |
| 芯片手册、板卡外设用法 | `docs/datasheets/`、`vendor/loongson_sdk_V2.1.0/docs/` |
| 版本变更 | `docs/notes/更新日志.md` |

---

## 6. 分支约定

```
main      受保护：只接受 PR 合并，禁止直接 push / force push
  └── dev        日常开发与联调
        └── feat/*  fix/*  chore/*     单个改动，完成后回 dev
```

- 提交信息用 `feat/fix/chore/docs/refactor` 前缀；
- 改动涉及 `main/` 或 `gpio_proj/` 的安全逻辑时，必须在 PR 描述里说明验证方式（台架 or 赛道）；
- 本地已启用 `.githooks/pre-push`，直接推 `main` 会被拦截。

远端保护规则设置见 `.github/BRANCHING.md`。

---

## 7. 已知约束

- `libraries/` 是**裁剪版**，缺 `common/src/lq_base_class.cpp`。`gpio_proj` 因此按 `gpio_proj/lib → vendor/loongson_sdk_V2.1.0 → ../libraries` 顺序查找依赖（见 `gpio_proj/CMakeLists.txt` 的 `LQ_LIB` 逻辑）。
- `debug/captures/` 是现场采集数据，会持续变大，已在 `.gitignore` 中排除新采集结果。
- 这份克隆是浅克隆（`--depth 1`），需要完整历史时执行 `git fetch --unshallow`。
