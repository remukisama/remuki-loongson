#include "watchdog.hpp"
#include <atomic>
#include <csignal>
#include <cstdio>

static std::atomic<bool> g_watchdog_exit_requested(false);

static void watchdog_signal_handler(int)
{
    g_watchdog_exit_requested.store(true, std::memory_order_relaxed);
}

int main()
{
    std::printf("========================================\n");
    std::printf("  GPIO Watchdog Daemon\n");
    std::printf("  Board: LS2K0301\n");
    std::printf("  Mode: Pure watchdog (no relay)\n");
    std::printf("========================================\n");

    struct sigaction sa = {};
    sa.sa_handler = watchdog_signal_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);
    sigaction(SIGQUIT, &sa, nullptr);

    gpio::GpioWatchdog watchdog;
    if (watchdog.init() != 0) {
        std::printf("[Main] Initialization failed\n");
        return -1;
    }

    watchdog.run(&g_watchdog_exit_requested);
    std::printf("[Main] Exited\n");
    return 0;
}
