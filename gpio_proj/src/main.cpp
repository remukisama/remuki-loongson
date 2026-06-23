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
#include <atomic>
#include <cstdio>
#include <csignal>

static std::atomic<bool> g_watchdog_exit_requested(false);

static void watchdog_signal_handler(int)
{
    g_watchdog_exit_requested.store(true, std::memory_order_relaxed);
}

int main()
{
    printf("========================================\n");
    printf("  GPIO Watchdog Daemon\n");
    printf("  Board: LS2K0301 龙丘核心板\n");
    printf("  Mode: Pure watchdog (no relay)\n");
    printf("========================================\n");

    struct sigaction sa = {};
    sa.sa_handler = watchdog_signal_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);
    sigaction(SIGQUIT, &sa, nullptr);

    gpio::GpioWatchdog watchdog;

    if (watchdog.init() != 0) {
        printf("[Main] Initialization failed\n");
        return -1;
    }

    watchdog.run(&g_watchdog_exit_requested);       // 阻塞，直到收到退出信号
    watchdog.cleanup();

    printf("[Main] Exited\n");
    return 0;
}
