// quick_pwm_test.cpp — 最简单的开环 PWM 测试
// 编译：g++ -o quick_pwm_test quick_pwm_test.cpp -I... (用 CMake 更好)
// 直接复用 main 的方式：在 main.cpp 加一个 --mode=pwm-quick-test

#include "lq_all_demo.hpp"
#include "user_app.hpp"
#include "car_config.hpp"
#include <cstdio>
#include <atomic>
#include <chrono>
#include <thread>
#include <sys/select.h>

extern std::atomic<bool> exit_flag;
extern void motor_Duty_L(int duty);
extern void motor_Duty_R(int duty);
extern void motor_Init(void);

static void safe_sleep(int ms) {
    while (ms > 0 && !exit_flag.load()) {
        int chunk = std::min(ms, (int)car_config::kGpioProtectHeartbeatIntervalMs);
        gpio_protect_heartbeat_pulse();
        timeval tv{0, chunk * 1000};
        select(0, nullptr, nullptr, nullptr, &tv);
        ms -= chunk;
    }
}

void lq_pwm_quick_test(void)
{
    const int PWM = 4000;
    const int DURATION_MS = 500000000;
    const int LOG_PERIOD_MS = 50;

    std::printf("\n=== Quick PWM Test: %d PWM for %d ms ===\n\n", PWM, DURATION_MS);

    gpio_protect_heartbeat_init();
    motor_Init();
    encoder_reset_state();

    if (!gpio_protect_heartbeat_require_ready(1500)) {
        motor_Duty_L(0); motor_Duty_R(0);
        gpio_protect_heartbeat_deinit();
        return;
    }

    motor_Duty_L(PWM);
    motor_Duty_R(PWM);

    int elapsed = 0;
    while (elapsed < DURATION_MS && !exit_flag.load()) {
        float sL = encoder_num_L();
        float sR = encoder_num_R();
        std::printf("[T=%4dms] encL=%.3f  encR=%.3f\n", elapsed, sL, sR);
        std::fflush(stdout);
        safe_sleep(LOG_PERIOD_MS);
        elapsed += LOG_PERIOD_MS;
    }

    motor_Duty_L(0);
    motor_Duty_R(0);
    std::printf("\n=== STOP ===\n");

    gpio_protect_heartbeat_deinit();
}
