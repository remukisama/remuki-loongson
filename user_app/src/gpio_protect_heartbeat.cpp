#include "gpio_protect_heartbeat.hpp"

#include "car_config.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace
{
static const char *kShmName = "/gpio_proj_shm";
static const uint32_t kShmVersion = 3;

struct GpioWatchdogShmLayout {
    uint32_t version;
    uint32_t heartbeat_counter;
    uint8_t safe_mode;
    uint8_t heartbeat_alive;
    uint8_t reserved[2];
    uint32_t heartbeat_last_ms;
    uint32_t watchdog_started_ms;
};

static const size_t kShmSize = sizeof(GpioWatchdogShmLayout);

std::atomic<bool> g_heartbeat_enabled{true};
std::atomic<long long> g_last_pulse_ms{0};
std::atomic<long long> g_last_open_attempt_ms{0};
std::atomic<uint32_t> g_heartbeat_counter{0};
int g_shm_fd = -1;
GpioWatchdogShmLayout *g_shm = nullptr;

long long now_ms()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool heartbeat_disabled_by_env()
{
    const char *env_enable = std::getenv("LQ_GPIO_PROTECT_HEARTBEAT_ENABLE");
    return env_enable != nullptr &&
           (std::strcmp(env_enable, "0") == 0 ||
            std::strcmp(env_enable, "false") == 0 ||
            std::strcmp(env_enable, "FALSE") == 0);
}

void close_watchdog_shm()
{
    if (g_shm && g_shm != MAP_FAILED) {
        munmap(g_shm, kShmSize);
        g_shm = nullptr;
    }
    if (g_shm_fd >= 0) {
        close(g_shm_fd);
        g_shm_fd = -1;
    }
}

int open_watchdog_shm()
{
    if (g_shm) return 0;

    long long stamp_ms = now_ms();
    long long last_attempt_ms = g_last_open_attempt_ms.load();
    if (stamp_ms - last_attempt_ms < 1000) return -1;
    if (!g_last_open_attempt_ms.compare_exchange_strong(last_attempt_ms, stamp_ms)) return -1;

    g_shm_fd = shm_open(kShmName, O_RDWR, 0666);
    if (g_shm_fd < 0) {
        std::printf("[GPIO_PROTECT] watchdog shm %s not found; start gpio_manager first\n", kShmName);
        return -1;
    }

    g_shm = static_cast<GpioWatchdogShmLayout *>(
        mmap(nullptr, kShmSize, PROT_READ | PROT_WRITE, MAP_SHARED, g_shm_fd, 0));
    if (g_shm == MAP_FAILED) {
        std::printf("[GPIO_PROTECT] mmap %s failed\n", kShmName);
        g_shm = nullptr;
        close_watchdog_shm();
        return -1;
    }

    if (g_shm->version != kShmVersion) {
        std::printf("[GPIO_PROTECT] shm version mismatch: shm=%u expected=%u\n",
                    g_shm->version, kShmVersion);
        close_watchdog_shm();
        return -1;
    }

    std::printf("[GPIO_PROTECT] connected to gpio_manager shm=%s interval=%dms\n",
                kShmName, car_config::kGpioProtectHeartbeatIntervalMs);
    return 0;
}
}

void gpio_protect_heartbeat_init(void)
{
    if (heartbeat_disabled_by_env()) {
        g_heartbeat_enabled.store(false);
        std::printf("[GPIO_PROTECT] heartbeat disabled by LQ_GPIO_PROTECT_HEARTBEAT_ENABLE\n");
        return;
    }

    g_last_pulse_ms.store(0);
    g_last_open_attempt_ms.store(0);
    g_heartbeat_counter.store(0);
    g_heartbeat_enabled.store(true);
    gpio_protect_heartbeat_pulse();
}

void gpio_protect_heartbeat_pulse(void)
{
    if (!g_heartbeat_enabled.load()) return;
    if (!g_shm && open_watchdog_shm() != 0) return;

    long long stamp_ms = now_ms();
    long long last_ms = g_last_pulse_ms.load();
    if (stamp_ms - last_ms < car_config::kGpioProtectHeartbeatIntervalMs) return;
    if (!g_last_pulse_ms.compare_exchange_strong(last_ms, stamp_ms)) return;

    uint32_t counter = g_heartbeat_counter.fetch_add(1) + 1;
    g_shm->heartbeat_counter = counter;
}

bool gpio_protect_heartbeat_require_ready(int timeout_ms)
{
    if (heartbeat_disabled_by_env()) {
        std::printf("[GPIO_PROTECT][FATAL] heartbeat disabled by env; car mode requires gpio_manager\n");
        return false;
    }

    g_heartbeat_enabled.store(true);
    const long long deadline_ms = now_ms() + timeout_ms;
    uint32_t last_watchdog_ms = 0;
    do {
        if (g_shm || open_watchdog_shm() == 0) {
            const uint32_t before = g_shm->heartbeat_counter;
            g_last_pulse_ms.store(0);
            gpio_protect_heartbeat_pulse();
            const uint32_t after = g_shm->heartbeat_counter;
            const uint32_t first_watchdog_ms = g_shm->heartbeat_last_ms;
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            const uint32_t second_watchdog_ms = g_shm->heartbeat_last_ms;
            last_watchdog_ms = second_watchdog_ms;
            if (after != before &&
                g_shm->watchdog_started_ms != 0 &&
                g_shm->heartbeat_alive == 1 &&
                g_shm->safe_mode == 0 &&
                second_watchdog_ms != 0 &&
                second_watchdog_ms != first_watchdog_ms) {
                std::printf("[GPIO_PROTECT] required watchdog ready: heartbeat %u -> %u watchdog_ms %u -> %u\n",
                            before, after, first_watchdog_ms, second_watchdog_ms);
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (now_ms() < deadline_ms);

    std::printf("[GPIO_PROTECT][FATAL] gpio_manager watchdog not ready within %dms; refuse car mode (last_watchdog_ms=%u)\n",
                timeout_ms, last_watchdog_ms);
    return false;
}

void gpio_protect_heartbeat_deinit(void)
{
    if (g_shm && g_shm != MAP_FAILED) {
        g_shm->heartbeat_counter = 0;
    }
    close_watchdog_shm();
    g_heartbeat_enabled.store(false);
}
