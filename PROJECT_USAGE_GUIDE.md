# 当前项目使用指南

本文档说明当前工程的编译、部署、启动与安全验证流程。项目结构见 `PROJECT_STRUCTURE.md`。

## 1. 运行前的关键结论

1. 正式车模模式必须先运行 `gpio_manager`，再启动主程序。
2. 主程序以约 5 ms 周期运行编码器、差速、速度环和电机输出；控制线程每 10 ms 发送一次共享内存心跳。
3. `gpio_manager` 每 1 ms 检查心跳。心跳超过约 50 ms 未更新时，会直接接管 PWM、方向 GPIO、舵机和蜂鸣器。
4. `main` 只接受明确模式参数。正式运行使用 `--mode=car`；无参数只显示帮助。
5. `--unsafe-dev-mode` 只用于悬空或台架诊断，禁止赛道使用。

## 2. 目录职责

```text
main/        主程序入口、CMake、交叉编译/部署脚本
user_app/    图像、IMU、编码器、差速、PID、电机控制
gpio_proj/   独立 GPIO 看门狗守护进程
board/       板级资源：kernel/(内核) deploy/autoboot/(自启包) tools/(板端小工具)
libraries/   基础库和驱动库（裁剪版）
example/     调参和设备诊断示例
vendor/      官方 SDK V2.1.0 原始包（仅参考）
scripts/     环境自检与离线日志分析
docs/        手册、专题指南、版本记录
tools/       工具链与第三方依赖（不入库）
```

## 3. 编译

### GPIO 看门狗

```bash
cd gpio_proj
./build.sh manager
```

若需要部署到板端：

```bash
./build.sh manager <board-ip>
```

### 小车主程序

```bash
cd main
./build.sh
./build.sh --clean
./build.sh --reconfigure
```

编译、传输、运行：

```bash
./build.sh <board-ip>
./build.sh <board-ip> -r
APP_ARGS="--mode=tune" ./build.sh <board-ip> -r
```

每次更新 `CMakeLists.txt`、新增 `.cpp` 文件或切换工具链后，使用 `./build.sh --reconfigure`；有不确定性时使用 `./build.sh --clean`。

## 4. 板端安装 GPIO 服务

```bash
scp -r autoboot root@<board-ip>:/root/autoboot
ssh root@<board-ip>
cd /root/autoboot
chmod +x install.sh
./install.sh
systemctl status gpio-manager --no-pager
```

当前 `install.sh` 安装并启动的是 `gpio-manager.service`，且会停用旧 `screen.service`。确认状态应为：

```text
Active: active (running)
```

查看日志：

```bash
journalctl -u gpio-manager -f
```

## 5. 正常启动顺序

```text
gpio_manager -> 主程序 --mode=car
```

板端：

```bash
systemctl status gpio-manager --no-pager
cd /root
./main --mode=car
```

主程序在以下条件下会拒绝进入或主动退出正式控制：

- gpio_manager 未就绪；
- MPU6050 初始化或零偏标定失败；
- 摄像头初始化失败；
- 首帧超时或运行中视觉帧超过时限；
- 看门狗活性丢失；
- 编码器无效、方向反向，或高 PWM 持续无编码器运动反馈。

## 6. 自制双路驱动板的停车逻辑

### 主程序本地急停

```text
清空目标速度与 PID 输出
PWM 保持使能
输出停车占空比
方向 GPIO 写 LOW
```

当前驱动板为低有效输入。为了避免 PWM disable 后输入悬空导致误转，急停时不会简单关闭 PWM。

### GPIO 硬件看门狗接管

```text
控制线程心跳停止超过约 50 ms
  -> PWM 通道保持使能
  -> PWM compare 写 ARR + 1，持续输出停车电平
  -> GPIO21..24 LOW
  -> GTIM CCR2 = 0
  -> GPIO61 LOW
```

对当前低有效电机驱动，`CCR=0` 可能是有效驱动电平；手工验证或修改看门狗时，必须使用代码中的 `ARR+1` 停车逻辑，而不要按旧文档写 `CCR=0`。

## 7. 引脚对应关系

| 功能 | 主程序使用 | 看门狗保护 |
| --- | --- | --- |
| 左电机 PWM | `ATIM_PWM0_PIN81` | ATIM CCR1 / GPIO81 |
| 右电机 PWM | `ATIM_PWM1_PIN82` | ATIM CCR2 / GPIO82 |
| 左电机方向 | `PIN_21` | GPIO21 LOW |
| 右电机方向 | `PIN_22` | GPIO22 LOW |

任何改线、换驱动板或改变 PWM 极性后，都必须先架空车轮，重新验证“停止值”和“前进方向”。

## 8. 更新程序

更新 GPIO 看门狗：

```bash
cd gpio_proj
./build.sh manager
scp -O build/gpio_manager root@<board-ip>:/root/autoboot/bin/gpio_manager   # 源文件在 gpio_proj/build/
ssh root@<board-ip> 'systemctl restart gpio-manager && systemctl status gpio-manager --no-pager'
```

更新主程序：

```bash
cd main
./build.sh --reconfigure <board-ip>
```

## 9. 上车前最低检查清单

1. 架空车轮，保留物理断电手段。
2. `systemctl status gpio-manager --no-pager` 为 active。
3. 启动程序，确认没有 `/gpio_proj_shm`、IMU 或相机初始化错误。
4. 低速验证左右轮前进方向和左右编码器符号。
5. 按 Ctrl+C，确认电机停止。
6. 在悬空状态使用 `pkill -TERM -x main`，确认看门狗记录心跳丢失并接管。
7. 用低速短距离验证视觉超时、编码器无反馈与反向反馈保护。

## 10. 常见故障

### 找不到 `/gpio_proj_shm`

```bash
systemctl start gpio-manager
systemctl status gpio-manager --no-pager
```

### 主程序编译成功但车不动

确认启动参数是 `--mode=car`。常见返回码：

| 返回码 | 含义 |
| --- | --- |
| 0 | 正常退出 |
| 2 | mode 参数不支持 |
| 3 | gpio_manager 未就绪 |
| 4 | IMU 初始化失败 |
| 5 | 摄像头初始化失败 |

### 看门狗运行但无法停车

依次确认实际 PWM/方向接线、驱动板的安全电平、`gpio-manager` 日志，以及看门狗接管时 `ARR+1` 是否确实使驱动板停止。
