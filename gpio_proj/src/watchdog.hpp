/**
 * @file watchdog.hpp
 * @brief GPIO 看门狗 — 心跳监测 + 异常安全接管
 *
 * 保护范围（心跳超时写安全值）：
 *   - 4路电机 PWM（ATIM CCR1~CCR4 → 100%高电平停车）
 *   - 4路方向 GPIO → LOW
 *   - 1路舵机 PWM（GTIM CCR2 → duty=0）
 *   - 蜂鸣器 GPIO → LOW
 */

#ifndef GPIO_PROJ_WATCHDOG_HPP
#define GPIO_PROJ_WATCHDOG_HPP

#include "shm_protocol.hpp"
#include <atomic>
#include <cstdint>

namespace gpio {

class GpioWatchdog {
public:
    GpioWatchdog();
    ~GpioWatchdog();

    int  init(void);
    void run(const std::atomic<bool> *external_stop = nullptr);
    void stop(void);
    void cleanup(void);

private:
    // 共享内存
    int        shm_fd_;
    ShmLayout *shm_;

    // mmap 的硬件寄存器地址
    volatile uint32_t *atim_base_;      // ATIM PWM (0x16118000)
    volatile uint32_t *gtim_base_;      // GTIM PWM (0x16119000)
    volatile uint8_t  *gpio_base_;      // GPIO 输出寄存器 (0x16104000)

    // 心跳状态
    uint32_t last_heartbeat_counter_;
    uint32_t last_heartbeat_time_ms_;
    bool     safe_mode_active_;

    std::atomic<bool> running_;

    // 内部方法
    int  init_shm(void);
    void cleanup_shm(void);
    int  init_registers(void);
    void write_safe_values(void);
    void check_heartbeat(void);
    uint32_t get_time_ms(void);
};

}  // namespace gpio

#endif /* GPIO_PROJ_WATCHDOG_HPP */
