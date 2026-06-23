# 当前项目使用指南

本文档说明当前工程从编译、部署到上车运行的推荐流程。项目结构说明见
`PROJECT_STRUCTURE.md`。

## 1. 使用前先看这三个结论

1. 自制双路驱动板调试时，必须先启动 `gpio_manager` 看门狗，再启动小车主程序。
2. 小车主程序控制线程每 5ms 运行一次完整闭环（编码器→速度环→电机输出），只有完整周期结束后才向 `/gpio_proj_shm` 写心跳；如果程序崩溃、卡死或被强退，`gpio_manager` 会在约 50ms 后把电机 PWM 和方向 GPIO 写回安全值。
3. 当前 `main/main.cpp` 已无 NCNN 早返回；NCNN 推理代码已注释。正式跑车使用 `--mode=car` 参数。

## 2. 推荐目录认知

```text
.
├── main/        小车主程序入口、CMake、编译部署脚本
├── user_app/    小车控制逻辑、电机控制、图像处理、IMU、心跳发送
├── gpio_proj/   GPIO 安全看门狗守护进程
├── autoboot/    板端开机自启安装包
├── libraries/   龙邱基础库和驱动库
├── driver/      内核驱动相关文件
├── example/     龙邱 demo 和调参示例
├── tools/       龙芯交叉编译工具链和第三方依赖
└── docs/        资料文档
```

当前工程没有把原有车端代码搬进 `car_project/`，而是保留原工程布局：

```text
main/ + user_app/ + libraries/  ~=  loongson-car-system/car_project/
gpio_proj/                     ~=  loongson-car-system/gpio_proj/
autoboot/                      ~=  loongson-car-system/autoboot/
```

## 3. 第一次使用流程

### 3.1 在开发主机编译 GPIO 看门狗

```bash
cd gpio_proj
./build.sh manager
```

成功后生成：

```text
gpio_proj/build/gpio_manager
```

只执行上面的命令时不会自动连接开发板。需要部署到板子时再显式传入 IP：

```bash
cd gpio_proj
./build.sh manager <board-ip>
```

### 3.2 在开发主机编译小车主程序

```bash
cd main
./build.sh
```

成功后生成：

```text
main/build/main
```

如果要编译后传到开发板：

```bash
cd main
./build.sh <board-ip>
```

如果要传输后直接运行：

```bash
cd main
./build.sh <board-ip> -r
```

## 4. 开发板部署 GPIO 安全服务

推荐使用 `autoboot/` 目录部署，它包含已验证的服务模板和预编译程序。

### 4.1 传输 autoboot

```bash
scp -r autoboot root@<board-ip>:/root/autoboot
```

### 4.2 登录开发板安装

```bash
ssh root@<board-ip>
cd /root/autoboot
chmod +x install.sh
./install.sh
```

安装脚本会：

1. 给 `bin/gpio_manager` 和 `bin/screen_app` 添加执行权限。
2. 写入 systemd 服务文件。
3. 启用开机自启。
4. 立即启动 `gpio-manager` 和 `screen` 服务。

### 4.3 验证服务状态

```bash
systemctl status gpio-manager
systemctl status screen
```

重点看 `gpio-manager` 是否为：

```text
Active: active (running)
```

查看实时日志：

```bash
journalctl -u gpio-manager -f
```

## 5. 正常上车启动顺序

上车时按下面顺序执行：

1. 给开发板上电。
2. 确认 `gpio-manager` 已经运行。
3. 再启动小车主程序。

推荐检查命令：

```bash
systemctl status gpio-manager
ps aux | grep gpio_manager
```

然后运行主程序（注意必须带 `--mode=car` 参数）：

```bash
cd /root
./main --mode=car
```

如果你用 `main/build.sh <board-ip> -r` 部署并运行，脚本会自动传入 `--mode=car`。
调参模式需显式指定：`APP_ARGS="--mode=tune" ./build.sh <board-ip> -r`。

## 6. 自制双路驱动板保护逻辑

当前保护分两层。

第一层在小车主程序内：

```text
emergency_stop()
  清空目标速度
  清空 PID 输出
  PWM 保持使能
  占空比写 0
  方向 GPIO 写 LOW
```

这层用于避免自制驱动板在 PWM disable 后输入悬空，被误识别为满转。

第二层在 `gpio_manager`（心跳只在完整控制周期后发送）：

```text
小车主程序每 5ms 控制周期完成后写一次 heartbeat_counter
gpio_manager 每 1ms 检查 heartbeat_counter
超过约 50ms 不变化：
  ATIM CCR1..CCR4 = 0
  GTIM CCR2 = 0
  GPIO21..24 = LOW
  GPIO61 = LOW
```

第二层可以覆盖 Ctrl+C、崩溃、死循环、进程被 kill 等情况，是自制驱动板的关键保底。

## 7. 当前电机相关引脚

| 功能 | 主程序使用 | 看门狗保护 |
| --- | --- | --- |
| 左电机 PWM | `ATIM_PWM0_PIN81` | ATIM CCR1 / GPIO81 |
| 右电机 PWM | `ATIM_PWM1_PIN82` | ATIM CCR2 / GPIO82 |
| 左电机方向 | `PIN_21` | GPIO21 LOW |
| 右电机方向 | `PIN_22` | GPIO22 LOW |

如果你的自制板方向低电平不是安全状态，需要实测后修改 `gpio_proj/config/board_2k301.hpp`
里的安全电平配置。

## 8. 更新 gpio_manager 的方法

如果你修改了 `gpio_proj/` 源码，需要重新编译并替换板端程序。

开发主机：

```bash
cd gpio_proj
./build.sh manager
scp -O build/gpio_manager root@<board-ip>:/root/autoboot/bin/gpio_manager
```

开发板：

```bash
systemctl restart gpio-manager
systemctl status gpio-manager
```

## 9. 更新小车主程序的方法

开发主机：

```bash
cd main
./build.sh <board-ip>
```

开发板：

```bash
cd /root
./main --mode=car
```

如果希望部署后直接运行：

```bash
cd main
./build.sh <board-ip> -r
```

## 10. 上车前检查清单

每次换板、换驱动板或修改电机代码后，建议按这个顺序检查：

1. 架空车轮，禁止落地直接测。
2. `systemctl status gpio-manager` 确认看门狗运行。
3. 启动小车主程序后观察日志，确认没有 `/gpio_proj_shm` 连接失败的持续报错。
4. 小油门测试左右轮方向是否正确。
5. 按 Ctrl+C，确认电机立即停止。
6. 直接杀掉主程序，确认电机也停止：

```bash
pkill -x main
```

7. 查看看门狗日志是否记录心跳丢失：

```bash
journalctl -u gpio-manager -n 50
```

## 11. 常见问题

### 11.1 Ctrl+C 后官方驱动板没事，自制驱动板满转

通常是自制驱动板输入端没有和官方板一样处理悬空、下拉或失能态。主程序退出时如果只
disable PWM，而不是保持 PWM 输出 0，自制板可能把输入误判成有效满占空比。

当前工程的处理方式是：

1. 急停时 PWM 不失能，保持输出 0。
2. 额外启动 `gpio_manager`，程序失联后由独立守护进程强制写安全值。

### 11.2 日志提示找不到 `/gpio_proj_shm`

说明 `gpio_manager` 没有运行，或者小车主程序启动得太早。

处理：

```bash
systemctl start gpio-manager
systemctl status gpio-manager
```

再重新启动小车主程序。

### 11.3 `./build.sh manager` 卡在 SSH

当前 `gpio_proj/build.sh` 已改成没有默认板子 IP。只有显式传入 `<board-ip>` 时才会部署：

```bash
./build.sh manager <board-ip>
```

如果卡在 SSH，检查 IP、网线、开发板地址和 SSH 是否可登录。

### 11.4 主程序编译成功但车不动

先检查是否使用了 `--mode=car` 参数启动。无参数运行只会打印帮助信息。

如果已带参数但仍不进入控制循环，检查返回码：

| 返回码 | 含义 |
| --- | --- |
| 0 | 正常退出 |
| 2 | 不支持的 mode 参数 |
| 3 | gpio_manager 看门狗就绪检测失败 |
| 4 | IMU 初始化失败 |
| 5 | 摄像头初始化失败 |

### 11.5 看门狗运行后仍然不能安全停止

按顺序排查：

1. 检查实际电机 PWM 和方向线是否接在 GPIO81、GPIO82、GPIO21、GPIO22。
2. 检查自制板方向安全电平是 LOW 还是 HIGH。
3. 查看 `journalctl -u gpio-manager -f` 是否有心跳丢失和安全值写入日志。
4. 架空车轮，单独测试 `gpio_proj` 的安全值是否能让驱动板停止。

## 12. 相关文档

- `PROJECT_STRUCTURE.md`：当前项目结构说明
- `gpio_proj/README.md`：GPIO 看门狗子项目说明
- `gpio_proj/开机自启部署指南.md`：单独部署 `gpio_manager` 的详细说明
- `autoboot/README.md`：自启包安装说明
