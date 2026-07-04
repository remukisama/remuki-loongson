/**
 * @file watchdog.cpp
 * @brief GPIO 看门狗实现 — 4路电机 + 舵机 + 蜂鸣器安全接管
 *
 * 直接 mmap 物理寄存器，不创建驱动对象。
 * 心跳超时时一次性写入所有受保护引脚的安全值。
 *
 * 寄存器地址来源：
 *   ATIM: 0x16118000 (lq_reg_atim_pwm.hpp)
 *   GTIM: 0x16119000 (lq_reg_gtim_pwm.hpp)
 *   GPIO: 0x16104000 (lq_reg_gpio.hpp)
 */

#include "watchdog.hpp"
#include "board_config.hpp"
#include "lq_utils.hpp"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

namespace gpio {

// ==================== 物理地址 ====================
static const uintptr_t ATIM_PHYS = 0x16118000;
static const uintptr_t GTIM_PHYS = 0x16119000;
static const uintptr_t GPIO_PHYS = 0x16104000;
static const size_t    REG_PAGE  = 0x1000;

// ==================== ATIM CCR 偏移（CH1~CH4）====================
// ATIM_CH1=CCR1(0x34), CH2=CCR2(0x38), CH3=CCR3(0x3C), CH4=CCR4(0x40)
static const uint32_t ATIM_CCR1 = 0x34;
static const uint32_t ATIM_CCR2 = 0x38;
static const uint32_t ATIM_CCR3 = 0x3C;
static const uint32_t ATIM_CCR4 = 0x40;
static const uint32_t ATIM_CCER = 0x20;
static const uint32_t ATIM_ARR  = 0x2C;

// ==================== GTIM CCR 偏移（CH1~CH4）====================
// GTIM_TIM2_CH2 → CCR2(0x38)
static const uint32_t GTIM_CCR2 = 0x38;

// ==================== GPIO 输出寄存器辅助 ====================
// 输出值寄存器基地址：0x10，每字节控制 8 引脚
// 偏移 = 0x10 + pin/8，位 = pin%8

static inline void gpio_pin_low(volatile uint8_t *base, int pin)
{
    volatile uint8_t *reg = base + 0x10 + (pin / 8);
    *reg = *reg & ~(1 << (pin % 8));
}

static inline void gpio_pin_high(volatile uint8_t *base, int pin)
{
    volatile uint8_t *reg = base + 0x10 + (pin / 8);
    *reg = *reg | (1 << (pin % 8));
}

static inline void ccr_write(volatile uint32_t *base, uint32_t offset, uint32_t val)
{
    if (base && base != MAP_FAILED) {
        volatile uint32_t *reg = (volatile uint32_t *)((uintptr_t)base + offset);
        *reg = val;
    }
}

// ==================== 构造 / 析构 ====================

GpioWatchdog::GpioWatchdog()
    : shm_fd_(-1), shm_(nullptr)
    , atim_base_(nullptr), gtim_base_(nullptr), gpio_base_(nullptr)
    , last_heartbeat_counter_(0), last_heartbeat_time_ms_(0)
    , safe_mode_active_(false), running_(false)
{
}

GpioWatchdog::~GpioWatchdog() { cleanup(); }

// ==================== 初始化 ====================

int GpioWatchdog::init(void)
{
    printf("[Watchdog] Initializing...\n");
    if (init_shm() != 0) return -1;
    if (init_registers() != 0) return -1;

    write_safe_values();

    shm_->version = SHM_VERSION;
    shm_->heartbeat_counter = 0;
    shm_->heartbeat_alive = 0;
    shm_->safe_mode = 0;
    shm_->heartbeat_last_ms = 0;
    shm_->watchdog_started_ms = get_time_ms();

    last_heartbeat_counter_ = 0;
    last_heartbeat_time_ms_ = get_time_ms();
    safe_mode_active_ = false;
    running_ = true;

    printf("[Watchdog] Initialized OK — protecting 4 motors + 1 servo + buzzer\n");
    return 0;
}

// ==================== 主循环 ====================

void GpioWatchdog::run(const std::atomic<bool> *external_stop)
{
    printf("[Watchdog] Main loop started (1ms heartbeat check)\n");
    while (running_ && !(external_stop && external_stop->load(std::memory_order_relaxed))) {
        check_heartbeat();
        usleep(1000);
    }
    running_ = false;
    write_safe_values();
    if (shm_ && shm_ != MAP_FAILED) {
        shm_->safe_mode = 1;
        shm_->heartbeat_alive = 0;
    }
    printf("[Watchdog] Main loop exited\n");
}

void GpioWatchdog::stop(void) { running_ = false; }

void GpioWatchdog::cleanup(void)
{
    write_safe_values();
    if (shm_ && shm_ != MAP_FAILED) {
        shm_->safe_mode = 1;
        shm_->heartbeat_alive = 0;
    }
    cleanup_shm();
    if (atim_base_ && atim_base_ != MAP_FAILED) { munmap((void*)atim_base_, REG_PAGE); atim_base_ = nullptr; }
    if (gtim_base_ && gtim_base_ != MAP_FAILED) { munmap((void*)gtim_base_, REG_PAGE); gtim_base_ = nullptr; }
    if (gpio_base_ && gpio_base_ != MAP_FAILED) { munmap((void*)gpio_base_, REG_PAGE); gpio_base_ = nullptr; }
    printf("[Watchdog] Cleaned up\n");
}

// ==================== 共享内存 ====================

int GpioWatchdog::init_shm(void)
{
    shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) { printf("[Watchdog] ERROR: shm_open failed\n"); return -1; }
    if (ftruncate(shm_fd_, SHM_SIZE) != 0) { printf("[Watchdog] ERROR: ftruncate failed\n"); return -1; }
    shm_ = (ShmLayout *)mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_ == MAP_FAILED) { printf("[Watchdog] ERROR: mmap failed\n"); return -1; }
    memset(shm_, 0, SHM_SIZE);
    printf("[Watchdog] Shared memory created (%s, %u bytes)\n", SHM_NAME, (unsigned)SHM_SIZE);
    return 0;
}

void GpioWatchdog::cleanup_shm(void)
{
    if (shm_ && shm_ != MAP_FAILED) { munmap(shm_, SHM_SIZE); shm_ = nullptr; }
    if (shm_fd_ >= 0) { close(shm_fd_); shm_fd_ = -1; }
    shm_unlink(SHM_NAME);
}

// ==================== 寄存器 mmap ====================

int GpioWatchdog::init_registers(void)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf("[Watchdog] ERROR: open /dev/mem failed; refuse to create fake-safe watchdog\n");
        return -1;
    }

    auto try_mmap = [](uintptr_t phys, int fd) -> void* {
        void *p = mmap(NULL, REG_PAGE, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd >= 0 ? fd : -1, phys);
        return (p != MAP_FAILED) ? p : nullptr;
    };

    // ATIM PWM (0x16118000) — 电机 CH1~CH4
    atim_base_ = (volatile uint32_t *)try_mmap(ATIM_PHYS, fd);
    printf("[Watchdog] ATIM  %s (phys 0x%lx)\n",
           atim_base_ ? "OK" : "FAIL", (unsigned long)ATIM_PHYS);

    // GTIM PWM (0x16119000) — 舵机 CH2
    gtim_base_ = (volatile uint32_t *)try_mmap(GTIM_PHYS, fd);
    printf("[Watchdog] GTIM  %s (phys 0x%lx)\n",
           gtim_base_ ? "OK" : "FAIL", (unsigned long)GTIM_PHYS);

    // GPIO (0x16104000) — 方向引脚 + 蜂鸣器
    gpio_base_ = (volatile uint8_t *)try_mmap(GPIO_PHYS, fd);
    printf("[Watchdog] GPIO  %s (phys 0x%lx)\n",
           gpio_base_ ? "OK" : "FAIL", (unsigned long)GPIO_PHYS);

    close(fd);

    // ATIM 和 GPIO 是电机安全接管的硬要求；缺任一项都必须 fail-closed。
    if (!atim_base_ || !gpio_base_) {
        printf("[Watchdog] ERROR: ATIM/GPIO mmap incomplete; refuse to create fake-safe watchdog\n");
        return -1;
    }
    return 0;
}

// ==================== 安全值写入 ====================

void GpioWatchdog::write_safe_values(void)
{
    // ---- ATIM: 4路电机 IN2 raw duty=SAFE_MOTOR_DUTY ----
    // DRV8701P IN1/IN2: IN1=HIGH and IN2=HIGH is brake/stop.
    uint32_t safe_compare = 0;
    if (atim_base_ && atim_base_ != MAP_FAILED) {
        volatile uint32_t *arr =
            (volatile uint32_t *)((uintptr_t)atim_base_ + ATIM_ARR);
        const uint32_t arr_count = *arr + 1u;
        safe_compare = static_cast<uint32_t>(
            (static_cast<uint64_t>(arr_count) * SAFE_MOTOR_DUTY) / PWM_DUTY_MAX);
    }
    ccr_write(atim_base_, ATIM_CCR1, safe_compare);   // 电机1 (GPIO81)
    ccr_write(atim_base_, ATIM_CCR2, safe_compare);   // 电机2 (GPIO82)
    ccr_write(atim_base_, ATIM_CCR3, safe_compare);   // 电机3 (GPIO83)
    ccr_write(atim_base_, ATIM_CCR4, safe_compare);   // 电机4 (GPIO76)

    // Keep NORMAL polarity and channel enable so the timer actively drives IN2 high.
    if (atim_base_ && atim_base_ != MAP_FAILED) {
        volatile uint32_t *ccer =
            (volatile uint32_t *)((uintptr_t)atim_base_ + ATIM_CCER);
        uint32_t value = *ccer;
        for (int ch = 0; ch < 4; ++ch) {
            value &= ~(1u << (ch * 4 + 1)); // CCxP=0: NORMAL
            value |=  (1u << (ch * 4));     // CCxE=1: 通道保持使能
        }
        *ccer = value;
    }

    // ---- GTIM: 舵机 PWM duty=0 ----
    ccr_write(gtim_base_, GTIM_CCR2, 0);   // 舵机 (GPIO88)

    // ---- GPIO: IN1方向引脚固定 HIGH + 蜂鸣器 LOW ----
    if (gpio_base_ && gpio_base_ != MAP_FAILED) {
        gpio_pin_high(gpio_base_, PIN_MOTOR_DIR_1);  // GPIO21 HIGH
        gpio_pin_high(gpio_base_, PIN_MOTOR_DIR_2);  // GPIO22 HIGH
        gpio_pin_high(gpio_base_, PIN_MOTOR_DIR_3);  // GPIO23 HIGH
        gpio_pin_high(gpio_base_, PIN_MOTOR_DIR_4);  // GPIO24 HIGH
        gpio_pin_low(gpio_base_, PIN_BUZZER);         // GPIO61 LOW
    }
}

// ==================== 心跳检测 ====================

void GpioWatchdog::check_heartbeat(void)
{
    uint32_t now = get_time_ms();
    uint32_t counter = shm_->heartbeat_counter;

    if (counter != last_heartbeat_counter_) {
        last_heartbeat_counter_ = counter;
        last_heartbeat_time_ms_ = now;
        shm_->heartbeat_alive = 1;
        shm_->heartbeat_last_ms = now;
        if (safe_mode_active_) {
            safe_mode_active_ = false;
            shm_->safe_mode = 0;
            printf("[Watchdog] Heartbeat resumed, exiting safe mode\n");
        }
    } else {
        if (!safe_mode_active_ && (now - last_heartbeat_time_ms_ > HEARTBEAT_TIMEOUT_MS)) {
            safe_mode_active_ = true;
            shm_->safe_mode = 1;
            shm_->heartbeat_alive = 0;
            write_safe_values();
            printf("[Watchdog] Heartbeat lost! Safe values written (4 motors + servo + buzzer)\n");
        }
    }
}

// ==================== 工具 ====================

uint32_t GpioWatchdog::get_time_ms(void) { return lq_get_tick_ms(); }

}  // namespace gpio
