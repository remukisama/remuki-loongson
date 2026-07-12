#include "lq_all_demo.hpp"
#include "car_config.hpp"
#include "my_mpu6050.hpp"
#include "encoder.hpp"
#include "gpio_protect_heartbeat.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <thread>

// ============================================================
// 【Demo 41】角速度环调参 — 无电机纯陀螺仪模式
//
// 用途：标定角速度环的 P/I/D 参数，验证陀螺仪信号调理链
//       （尖峰拒绝→限幅→LPF→死区）的效果。
// 不驱动电机，只读取 MPU6050 角速度，运行 PID 计算，
// 通过 UDP 输出波形供上位机观察。
//
// 阶跃测试：目标角速度在 +target ↔ -target 之间周期切换，
//           观察角速度环的跟踪响应。
//
// UDP 输出格式（VOFA+ JustFloat 兼容）：
//   "ware:target,gyro_raw,gyro_filt,err,diff_out\n"
//
// 用法：./main --mode=gyro-rate-tune
// ============================================================

static std::atomic<bool> g_exit(false);

static void sig_handler(int) { g_exit.store(true); }

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
// 迷你角速度环（复刻 my_cascade_pid.cpp 的信号调理 + PID）
// ============================================================
struct MiniGyroPID {
    float P, I, D;
    float err;
    float last_err;
    float integral;
    float int_max;
    float out_max;
    float output;

    // 信号调理状态
    float gyro_z_filtered;
    float last_gyro_z_measurement;
    float last_output;

    void reset() {
        err = last_err = integral = output = 0.0f;
        gyro_z_filtered = last_gyro_z_measurement = last_output = 0.0f;
    }

    float update(float target_gyro_z, float raw_gyro_z, float dt) {
        if (dt <= 0.0001f || dt > 0.1f) dt = 0.005f;

        // --- 信号调理 ---
        float g = raw_gyro_z;

        // 1. 尖峰拒绝
        if (std::fabs(g) > car_config::kGyroRawSpikeRejectDps) {
            g = last_gyro_z_measurement;
        }
        // 2. 测量限幅
        g = clampf(g, -car_config::kGyroMeasurementLimitDps,
                        car_config::kGyroMeasurementLimitDps);
        // 3. 一阶低通滤波
        gyro_z_filtered = gyro_z_filtered * (1.0f - car_config::kGyroLpfAlpha) +
                          g * car_config::kGyroLpfAlpha;
        // 4. 死区
        if (std::fabs(gyro_z_filtered) < car_config::kGyroDeadBandDps) {
            gyro_z_filtered = 0.0f;
        }

        // --- PID ---
        err = target_gyro_z - gyro_z_filtered;
        integral = clampf(integral + err * dt, -int_max, int_max);
        float derivative = (err - last_err) / dt;
        float out = P * err + I * integral + D * derivative;
        out = clampf(out, -out_max, out_max);

        last_gyro_z_measurement = g;
        last_err = err;
        last_output = out;
        output = out;
        return out;
    }
};

// ============================================================
// 入口函数
// ============================================================
int lq_gyro_rate_pid_tune_demo(void)
{
    g_exit.store(false);
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGHUP,  sig_handler);
    signal(SIGQUIT, sig_handler);

    std::printf("=== Gyro Rate PID Tune (No Motor) ===\n");
    std::printf("UDP target: %s:%d\n", "192.168.7.22", 8080);
    std::printf("Format: ware:target,gyro_raw,gyro_filt,err,out\\n\n");

    // ---- 初始化 UDP ----
    lq_udp_client udp;
    udp.udp_client_init("192.168.7.22", 8080);

    // ---- 初始化 MPU6050 ----
    MyMpu6050 imu;
    if (!imu.init()) {
        std::printf("[FATAL] MPU6050 init failed\n");
        return 1;
    }
    std::printf("MPU6050 OK\n");

    // ---- PID 参数（从 car_config 读取，可在此处覆盖调试）----
    MiniGyroPID pid;
    pid.P = car_config::kGyroPidP;          // 0.045
    pid.I = car_config::kGyroPidI;          // 0.0
    pid.D = car_config::kGyroPidD;          // 0.0008
    pid.int_max = car_config::kGyroPidIntegralLimit;
    pid.out_max = car_config::kGyroPidOutputLimit;  // 0.65
    pid.reset();

    std::printf("Params: P=%.4f I=%.4f D=%.5f out_max=%.2f\n",
                pid.P, pid.I, pid.D, pid.out_max);
    std::printf("Signal: spike=%.0f lpf=%.2f deadband=%.1f\n",
                car_config::kGyroRawSpikeRejectDps,
                car_config::kGyroLpfAlpha,
                car_config::kGyroDeadBandDps);

    // ---- 阶跃测试参数 ----
    constexpr float kStepTarget   = 30.0f;   // 阶跃目标角速度 °/s
    constexpr float kStepHoldSec  = 1.5f;    // 每段保持时间
    constexpr float kControlPeriodSec = 0.005f; // 5ms = 200Hz

    // ---- 主循环 ----
    auto last_time = std::chrono::steady_clock::now();
    bool positive = true;
    auto next_flip_time = last_time + std::chrono::milliseconds(
        static_cast<int>(kStepHoldSec * 1000));

    int print_count = 0;

    while (!g_exit.load(std::memory_order_relaxed)) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = now - last_time;
        float dt = elapsed.count();
        last_time = now;
        if (dt <= 0.0001f || dt > 0.1f) dt = kControlPeriodSec;

        // 阶跃切换
        if (now >= next_flip_time) {
            positive = !positive;
            next_flip_time = now + std::chrono::milliseconds(
                static_cast<int>(kStepHoldSec * 1000));
        }
        const float target_gyro_z = positive ? kStepTarget : -kStepTarget;

        // 读陀螺仪
        Mpu6050Data imu_data;
        if (!imu.read_processed_data(imu_data)) {
            std::printf("[WARN] IMU read failed, skip\n");
            gpio_protect_heartbeat_pulse();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // PID 计算
        float diff_out = pid.update(target_gyro_z, imu_data.gyro_z, dt);

        // UDP 波形输出
        char buf[128];
        int len = std::snprintf(buf, sizeof(buf),
            "ware:%.2f,%.2f,%.2f,%.2f,%.3f\n",
            target_gyro_z,        // ch1: 目标角速度
            imu_data.gyro_z,      // ch2: 原始角速度
            pid.gyro_z_filtered,  // ch3: 滤波后角速度
            pid.err,              // ch4: 误差
            diff_out);            // ch5: PID 输出（差速量）
        udp.udp_send(buf, len);

        // 串口低频打印
        if (++print_count >= 100) {  // 每 100 帧 (~0.5s) 打印一次
            print_count = 0;
            std::printf("tgt:%6.1f raw:%7.1f filt:%7.1f err:%7.1f out:%.3f\n",
                        target_gyro_z, imu_data.gyro_z,
                        pid.gyro_z_filtered, pid.err, diff_out);
        }

        gpio_protect_heartbeat_pulse();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::printf("Gyro rate tune demo exited.\n");
    return 0;
}
