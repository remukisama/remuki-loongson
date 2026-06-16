/**
 * @file main.cpp
 * @brief GPIO 看门狗守护进程入口
 *
 * 纯看门狗模式：
 *   - 监测小车线程心跳（共享内存）
 *   - 心跳超时时直接写入硬件寄存器安全值
 *   - 不参与正常控制流程，不中转指令
 *
 * 使用方法：
 *   ./gpio_manager              # 前台运行
 *   ./gpio_manager &            # 后台运行
 */

#include "watchdog.hpp"
#include "lq_signal_handle.hpp"
#include <cstdio>

int main()
{
    printf("========================================\n");
    printf("  GPIO Watchdog Daemon\n");
    printf("  Board: LS2K0301 龙丘核心板\n");
    printf("  Mode: Pure watchdog (no relay)\n");
    printf("========================================\n");

    gpio::GpioWatchdog watchdog;

    if (watchdog.init() != 0) {
        printf("[Main] Initialization failed\n");
        return -1;
    }

    watchdog.run();       // 阻塞，直到收到 SIGINT 或 ls_system_running=false
    watchdog.cleanup();

    printf("[Main] Exited\n");
    return 0;
}
