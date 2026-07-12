#include "lq_all_demo.hpp"
#include "car_config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "gpio_protect_heartbeat.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <csignal>
#include <thread>

// ============================================================
// 【Demo 44】四路通道确认测试（架空专用）
//
// 逐个给左/右电机PWM，同时读左/右编码器
// 确认电机↔编码器↔物理轮的对应关系
// 架空使用，PWM=1500，每段300ms，安全
// ============================================================

static std::atomic<bool> g_exit(false);
static void sig_handler(int) { g_exit.store(true); }

int lq_channel_check_demo(void)
{
    g_exit.store(false);
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGHUP, sig_handler);
    signal(SIGQUIT, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    std::printf("=== Channel Check (off-ground only) ===\n");
    std::printf("Phase1: L motor PWM=2000, R motor PWM=0, 300ms\n");
    std::printf("Phase2: stop (500ms)\n");
    std::printf("Phase3: L motor PWM=0, R motor PWM=2000, 300ms\n");
    std::printf("Phase4: stop\n\n");

    motor_Init();
    Speed_decision();  // 和car模式一样初始化速度环
    if (!gpio_protect_heartbeat_require_ready(1500)) {
        std::printf("[FATAL] gpio_manager not ready\n");
        return 3;
    }

    encoder_reset_state();

    auto start = std::chrono::steady_clock::now();

    // Phase 1: 只给左电机（同时给右电机极小PWM测试是否需要同时激活）
    std::printf("--- Phase 1: L_motor=2000 R_motor=0 ---\n");
    for (int ms = 0; ms < 300 && !g_exit.load(); ms += 20) {
        motor_Duty_L(2000);
        motor_Duty_R(0);
        gpio_protect_heartbeat_pulse();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        float eL = encoder_num_L();
        float eR = encoder_num_R();
        auto t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        std::printf("[CH1] t=%.2f pwmL=2000 pwmR=0 encL=%.3f encR=%.3f\n", t, eL, eR);
    }
    motor_Duty_L(0);
    motor_Duty_R(0);

    // Phase 2: 停车
    std::printf("--- Phase 2: stop ---\n");
    for (int ms = 0; ms < 500 && !g_exit.load(); ms += 100) {
        motor_Duty_L(0);
        motor_Duty_R(0);
        gpio_protect_heartbeat_pulse();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    start = std::chrono::steady_clock::now();

    // Phase 3: 同时给两个电机
    std::printf("--- Phase 3: L_motor=2000 R_motor=2000 ---\n");
    for (int ms = 0; ms < 300 && !g_exit.load(); ms += 20) {
        motor_Duty_L(2000);
        motor_Duty_R(2000);
        gpio_protect_heartbeat_pulse();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        float eL = encoder_num_L();
        float eR = encoder_num_R();
        auto t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        std::printf("[CH3] t=%.2f pwmL=0 pwmR=1500 encL=%.3f encR=%.3f\n", t, eL, eR);
    }
    motor_Duty_L(0);
    motor_Duty_R(0);

    // Phase 4: 停车
    std::printf("--- Phase 4: stop ---\n");
    gpio_protect_heartbeat_pulse();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::printf("\n=== Channel Check Done ===\n");
    std::printf("Check: Phase1 encL>encR? Phase3 encR>encL?\n");
    return 0;
}
