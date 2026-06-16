# gpio_proj — 引脚管理守护进程

## 简介

gpio_proj 是一个独立的硬件引脚管理守护进程，负责管理龙芯 2K0300/2K0301 开发板上的所有 GPIO、PWM、编码器等硬件引脚。

小车应用工程通过共享内存与本工程通信，不直接操作硬件引脚。

## 架构

```
┌──────────────────────┐     共享内存      ┌──────────────────────┐
│  小车应用工程          │ ◄──────────────► │  gpio_proj           │
│                      │   心跳 + 通道数据   │  (本工程)             │
│  app_motion_ctrl     │                   │                      │
│  bsp_motor           │                   │  pin_manager         │
│  bsp_encoder         │                   │  ├─ 心跳检测          │
│  client_lib.hpp      │                   │  ├─ 硬件读写          │
│  (不接触硬件)          │                   │  └─ 安全恢复          │
└──────────────────────┘                   └──────────┬───────────┘
                                                      │
                                               龙芯 2K0300 硬件
```

## 核心功能

### 1. 硬件引脚管理
统一管理所有输出引脚（PWM、GPIO）和输入引脚（编码器），小车工程通过通道名访问，不关心底层引脚号。

### 2. 心跳机制
- 小车工程每 10ms 发送一次心跳
- 引脚工程检测心跳，50ms 未收到则进入安全模式
- 安全模式：PWM 输出 0，GPIO 输出低电平（电机停止）
- 心跳恢复后立即回到上次的输出状态

### 3. 安全保护
程序崩溃、Ctrl+C、异常退出时，守护进程自动恢复安全默认值，防止电机疯转。

### 4. 板级适配
引脚定义集中在 `config/board_jiujjiupai.hpp`，通过编译时 `-DBOARD=` 宏自动切换。换板子只需新建一个配置文件，在 `config/board_config.hpp` 中添加分发即可。

## 目录结构

```
gpio_proj/
├── CMakeLists.txt              # 构建配置
├── build.sh                    # 编译部署脚本
├── cmake/toolchain.cmake       # 交叉编译工具链
│
├── config/
│   ├── board_config.hpp        # 板级分发器（-DBOARD= 自动切换）
│   └── board_jiujjiupai.hpp    # 久久派 2K0300 引脚定义
│
├── include/
│   ├── channel.hpp             # 通道枚举 + 数据结构
│   └── shm_protocol.hpp        # 共享内存协议定义
│
├── src/
│   ├── main.cpp                # 守护进程入口
│   ├── pin_manager.hpp         # 引脚管理器头文件
│   ├── pin_manager.cpp         # 引脚管理器实现
│   ├── client_lib.hpp          # 客户端库头文件（小车工程 include 这个）
│   └── client_lib.cpp          # 客户端库实现
│
├── lib/
│   ├── common/                 # 龙邱库公共模块（原版复制）
│   └── drv/                    # 龙邱库驱动模块（原版复制）
│
└── test/
    └── test_channels.cpp       # 通道测试程序
```

## 编译

```bash
# 独立编译（在 gpio_proj 目录下）
cd gpio_proj
./build.sh                        # 编译所有（默认久久派）
./build.sh manager                # 只编译守护进程
./build.sh test                   # 只编译测试程序

# 指定板子 IP 编译 + 部署
./build.sh manager 192.168.0.106
./build.sh manager 192.168.0.106 run

# 切换板子
./build.sh --board LOONGSON2K0301 # 切换到 2K0301 核心板
./build.sh JIUJIUPAI              # 切换回久久派
```

编译产物：
- `build/gpio_manager` — 守护进程
- `build/test/test_channels` — 测试程序

## 使用方法

### 在开发板上

```bash
# 1. 启动守护进程（后台运行）
./gpio_manager &

# 2. 运行测试
./test_channels

# 3. 或运行小车应用
./loongson_car
```

### 开机自启（可选）

创建 systemd 服务文件 `/etc/systemd/system/gpio-manager.service`：

```ini
[Unit]
Description=GPIO Manager Daemon
After=network.target

[Service]
Type=simple
ExecStart=/root/gpio_manager
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
```

然后启用：
```bash
systemctl enable gpio-manager
systemctl start gpio-manager
```

## 小车工程接入方式

小车工程通过 `client_lib.hpp` 接入，不需要直接操作硬件：

```cpp
#include "gpio_proj/src/client_lib.hpp"

void app_init() {
    gpio::init();           // 连接引脚工程
}

void app_loop() {           // 每 10ms 调用一次
    gpio::heartbeat();      // 发送心跳

    // 读编码器
    float enc_l = gpio::get_encoder_left();
    float enc_r = gpio::get_encoder_right();

    // 控制电机（正负含方向：正=正转，负=反转，绝对值=占空比）
    gpio::set_motor_left(duty_l);   // e.g. -5000 = 反转 5000/8000 占空比
    gpio::set_motor_right(duty_r);
}

void app_deinit() {
    gpio::deinit();         // 断开连接（自动停车）
}
```

## 通道定义

| 通道 | 方向 | 类型 | 范围 | 说明 |
|------|------|------|------|------|
| `OUT_MOTOR_LEFT` | 输出 | PWM | -8000 ~ 8000 | 左电机（正负含方向，绝对值=占空比） |
| `OUT_MOTOR_RIGHT` | 输出 | PWM | -8000 ~ 8000 | 右电机（正负含方向，绝对值=占空比） |
| `OUT_BUZZER` | 输出 | GPIO | 0/1 | 蜂鸣器 |
| `OUT_SERVO` | 输出 | PWM | — | 舵机（预留） |
| `IN_ENCODER_LEFT` | 输入 | 编码器 | m/s | 左编码器速度 |
| `IN_ENCODER_RIGHT` | 输入 | 编码器 | m/s | 右编码器速度 |

新增通道：在 `include/channel.hpp` 中添加枚举，然后在 `pin_manager.cpp` 中添加硬件操作。

## 心跳机制详解

```
小车应用                         引脚工程
    │                               │
    ├─ 每10ms: counter++        ──→ 检测 counter 变化
    ├─ 写输出值到共享内存       ──→ 转发到硬件
    │                               │
    │                         ┌─────┴─────┐
    │                         │ 50ms 无变化 │ → safe_mode=1
    │                         │ PWM=0      │ → 恢复安全值
    │                         │ GPIO=LOW   │
    │                         └─────┬─────┘
    │                               │
    ├─ 心跳恢复                 ──→ safe_mode=0
    │                               │ 用共享内存中的值恢复输出
```

**关键设计：** 安全模式下不 deinit 硬件，只改变输出值。心跳恢复时可以立即续上，不需要重新初始化。

## 编码器方向校准

如果电机方向与编码器读数方向不一致（比如交换了电机线），需要在 `config/board_jiujjiupai.hpp` 中调整编码器的方向引脚或在小车工程的 BSP 层添加方向系数。

## 依赖

- 龙芯 LoongArch64 交叉编译工具链（GCC 8.3+）
- Linux 内核支持 POSIX 共享内存（`shm_open`）
- CMake 3.16+

## 注意事项

1. 本工程完全独立，不依赖小车工程的任何文件
2. 龙邱库文件为原版复制，方便熟悉原库的开发者接手
3. 换板子只需新建 `config/board_xxx.hpp`，在 `config/board_config.hpp` 中添加分发，编译时加 `-DBOARD=XXX`
4. `client_lib.hpp` 是小车工程唯一需要的接口文件
5. 电机方向由 duty 正负号决定，没有独立的方向通道
6. PWM 量程 8000，不会达到满占空比
# -2K0301-GPIO-
