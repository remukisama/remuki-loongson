#include "lq_all_demo.hpp"
#include "car_config.hpp"
#include "my_mpu6050.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "gpio_protect_heartbeat.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <thread>

// ============================================================
// 【Demo 43】差速执行测试 — 绕开视觉和角速度环
//
// 直接设置左右轮目标速度，调用car模式同一套 Speed_PID_L/R + motor_Duty_L/R
// 验证：给左右轮不同目标时，实际编码器是否能拉开差速
//
// 测试矩阵：
//   Phase 1: 直行 L=0.30 R=0.30 (1.0s)
//   Phase 2: 停车 (0.5s)
//   Phase 3: 左转差速 L=0.15 R=0.40 (1.0s) → 应该 eR>eL, gz>0
//   Phase 4: 停车 (0.5s)
//   Phase 5: 右转差速 L=0.40 R=0.15 (1.0s) → 应该 eL>eR, gz<0
//   Phase 6: 停车 (0.5s)
//   Phase 7: 左转差速 L=0.20 R=0.40 (1.0s)
//   Phase 8: 停车 (0.5s)
//   Phase 9: 右转差速 L=0.40 R=0.20 (1.0s)
//   Phase 10: 停车
//
// 输出格式：
//   [DIFF] t=<time> phase=<n> gL=<goal_L> gR=<goal_R> eL=<enc_L> eR=<enc_R> 
//          errL=<err_L> errR=<err_R> pwmL=<pwm_L> pwmR=<pwm_R> gz=<gyro_z>
// ============================================================

static std::atomic<bool> g_exit(false);
static void sig_handler(int) { g_exit.store(true); }

int lq_diff_speed_test_demo(void)
{
    g_exit.store(false);
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGHUP, sig_handler);
    signal(SIGQUIT, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    std::printf("=== Diff Speed Execution Test ===\n");
    std::printf("Tests: straight, left-turn diff, right-turn diff\n");
    std::printf("Uses car-mode Speed_PID_L/R + motor_Duty_L/R\n\n");

    // ---- 初始化硬件 ----
    motor_Init();
    if (!gpio_protect_heartbeat_require_ready(1500)) {
        std::printf("[FATAL] gpio_manager not ready\n");
        return 3;
    }

    MyMpu6050 imu;
    if (!imu.init()) {
        std::printf("[FATAL] MPU6050 init failed\n");
        emergency_stop();
        return 1;
    }

    // 速度环初始化（和car模式完全一致）
    Speed_decision();
    control_set_imu_async_cache_only(true);

    std::printf("Speed PID: P=%.0f I=%.0f D=%.0f max_pwm=%d\n",
                car_config::kSpeedPidP, car_config::kSpeedPidI,
                car_config::kSpeedPidD, car_config::kMotorMaxPwm);
    std::printf("Slew rate=%.0f Integral limit=%.0f\n\n",
                car_config::kSpeedPidSlewRate, car_config::kSpeedPidIntegralLimit);

    // 等IMU稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 测试矩阵
    struct TestPhase {
        const char* name;
        float goal_l;
        float goal_r;
        int duration_ms;
    };

    const TestPhase phases[] = {
        {"straight_0.30",   0.30f, 0.30f, 800},
        {"stop",            0.0f,  0.0f,   400},
        {"left_L0.15_R0.40", 0.15f, 0.40f, 800},
        {"stop",            0.0f,  0.0f,   400},
        {"right_L0.40_R0.15", 0.40f, 0.15f, 800},
        {"stop",            0.0f,  0.0f,   400},
    };

    auto last_time = std::chrono::steady_clock::now();
    auto start_time = last_time;
    int phase_idx = 0;
    int phase_elapsed_ms = 0;

    std::printf("[START] Beginning diff speed test.\n\n");

    while (!g_exit.load(std::memory_order_relaxed) && phase_idx < (int)(sizeof(phases)/sizeof(phases[0]))) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_time;
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        last_time = now;

        phase_elapsed_ms += elapsed_ms;
        if (phase_elapsed_ms >= phases[phase_idx].duration_ms) {
            phase_idx++;
            phase_elapsed_ms = 0;
            if (phase_idx >= (int)(sizeof(phases)/sizeof(phases[0]))) break;
            std::printf("\n--- Phase %d: %s (gL=%.2f gR=%.2f %dms) ---\n",
                        phase_idx, phases[phase_idx].name,
                        phases[phase_idx].goal_l, phases[phase_idx].goal_r,
                        phases[phase_idx].duration_ms);
        }

        const TestPhase& ph = phases[phase_idx];

        // 读编码器
        speed_encoder_L = encoder_num_L();
        speed_encoder_R = encoder_num_R();
        if (!encoder_health_ok()) {
            motor_Duty_L(0);
            motor_Duty_R(0);
            gpio_protect_heartbeat_pulse();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 设置目标速度
        Speed_Goal_l = ph.goal_l;
        Speed_Goal_r = ph.goal_r;

        // 调用car模式的速度环（位置式PID）
        Speed_PID_L();
        Speed_PID_R();

        // 输出到电机
        motor_Duty_R(static_cast<int>(speed_out_r));
        motor_Duty_L(static_cast<int>(speed_out_l));

        // 读陀螺仪
        Mpu6050Data imu_data;
        float gz = 0.0f;
        if (imu.read_processed_data(imu_data)) {
            gz = imu_data.gyro_z;
        }

        // 每50ms打印一次
        static int print_accum_ms = 0;
        print_accum_ms += elapsed_ms;
        if (print_accum_ms >= 50) {
            print_accum_ms = 0;
            auto total_sec = std::chrono::duration<float>(now - start_time).count();
            float err_l = Speed_Goal_l - speed_encoder_L;
            float err_r = Speed_Goal_r - speed_encoder_R;
            float actual_diff = speed_encoder_R - speed_encoder_L;
            float target_diff = ph.goal_r - ph.goal_l;
            std::printf("[DIFF] t=%.2f p%d:%s gL=%.2f gR=%.2f eL=%.2f eR=%.2f "
                        "errL=%+.3f errR=%+.3f pwmL=%d pwmR=%d gz=%+.1f "
                        "tgtDiff=%+.3f actDiff=%+.3f\n",
                        total_sec, phase_idx, ph.name,
                        ph.goal_l, ph.goal_r,
                        speed_encoder_L, speed_encoder_R,
                        err_l, err_r,
                        (int)speed_out_l, (int)speed_out_r,
                        gz,
                        target_diff, actual_diff);
        }

        gpio_protect_heartbeat_pulse();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 停车
    motor_Duty_L(0);
    motor_Duty_R(0);
    emergency_stop();
    std::printf("\nDiff speed test done.\n");
    return 0;
}
