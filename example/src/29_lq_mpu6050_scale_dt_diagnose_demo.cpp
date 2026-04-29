#include "lq_all_demo.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <limits>

// ==========================================
// 【MPU6050 量程 / dt / 零偏 联合诊断 Demo】
// 作用：
// 1. 先静止采样，估计 gz 零偏
// 2. 同时用固定 dt 和真实 dt 做积分，对比 dt 误差
// 3. 用户手动旋转已知角度后退出，估算等效灵敏度
//
// 推荐使用方法：
// - 上电后先保持车体静止不动，等待自动完成零偏采样
// - 看到提示后，手动缓慢旋转车体“恰好一圈(360°)”
// - 旋转结束后按 Ctrl+C 退出
// - 退出时程序会打印：
//   angle_real_dt_no_bias      使用真实 dt，但不扣零偏
//   angle_real_dt_bias_fixed   使用真实 dt，并扣零偏
//   angle_fixed_dt_bias_fixed  使用固定 10ms dt，并扣零偏
//   estimated_lsb_per_dps      反推得到的等效灵敏度
// ==========================================

static std::atomic<bool> exit_demo(false);

static void signal_handler_mpu6050_diag(int)
{
    exit_demo = true;
}

void lq_mpu6050_scale_dt_diagnose_demo(void)
{
    signal(SIGINT, signal_handler_mpu6050_diag);
    signal(SIGTERM, signal_handler_mpu6050_diag);

    lq_i2c_mpu6050 mpu6050;
    while (mpu6050.get_mpu6050_id() != 0x68)
    {
        std::cout << "MPU6050 初始化失败, 检查连线..." << std::endl;
        usleep(1000000);
        if (exit_demo)
            return;
    }
    std::cout << "MPU6050 初始化成功!" << std::endl;

    constexpr float DEFAULT_LSB_PER_DPS = 16.4f;
    constexpr float FIXED_DT = 0.010f;
    constexpr float KNOWN_TEST_ANGLE_DEG = 360.0f;
    constexpr int CALIBRATION_SAMPLES = 300;
    constexpr int PRINT_INTERVAL = 10;

    std::cout << "\n[阶段1] 请保持设备静止，开始自动采样零偏..." << std::endl;

    double bias_sum = 0.0;
    double dt_sum_cal = 0.0;
    auto last_time = std::chrono::steady_clock::now();

    for (int i = 0; i < CALIBRATION_SAMPLES && !exit_demo; ++i)
    {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz);
        (void)ax;
        (void)ay;
        (void)az;
        (void)gx;
        (void)gy;

        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> fs = now_time - last_time;
        last_time = now_time;

        if (i > 0)
            dt_sum_cal += fs.count();

        bias_sum += gz;
        usleep(10000);
    }

    if (exit_demo)
        return;

    double gz_bias = bias_sum / CALIBRATION_SAMPLES;
    double avg_dt_cal = dt_sum_cal / (CALIBRATION_SAMPLES - 1);

    std::cout << "[阶段1完成] 零偏采样结束" << std::endl;
    std::cout << "gz_bias(raw LSB) = " << gz_bias << std::endl;
    std::cout << "avg_dt_cal(s)   = " << avg_dt_cal << std::endl;

    std::cout << "\n[阶段2] 现在请手动旋转车体恰好一圈(360°)，结束后按 Ctrl+C。" << std::endl;
    std::cout << "建议缓慢匀速旋转，避免剧烈抖动。\n" << std::endl;

    double angle_real_dt_no_bias = 0.0;
    double angle_real_dt_bias_fixed = 0.0;
    double angle_fixed_dt_bias_fixed = 0.0;
    double bias_corrected_gz_dt_integral = 0.0;

    int loop_count = 0;
    double dt_min = std::numeric_limits<double>::max();
    double dt_max = 0.0;
    double dt_sum = 0.0;
    int dt_count = 0;

    last_time = std::chrono::steady_clock::now();

    while (!exit_demo)
    {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz);
        (void)ax;
        (void)ay;
        (void)az;
        (void)gx;
        (void)gy;

        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> fs = now_time - last_time;
        double dt_real = fs.count();
        last_time = now_time;

        if (dt_real > 0.1)
            dt_real = FIXED_DT;

        if (dt_real < dt_min)
            dt_min = dt_real;
        if (dt_real > dt_max)
            dt_max = dt_real;
        dt_sum += dt_real;
        dt_count++;

        double gz_no_bias_dps = static_cast<double>(gz) / DEFAULT_LSB_PER_DPS;
        double gz_bias_fixed_dps = static_cast<double>(gz - gz_bias) / DEFAULT_LSB_PER_DPS;

        angle_real_dt_no_bias += gz_no_bias_dps * dt_real;
        angle_real_dt_bias_fixed += gz_bias_fixed_dps * dt_real;
        angle_fixed_dt_bias_fixed += gz_bias_fixed_dps * FIXED_DT;

        bias_corrected_gz_dt_integral += static_cast<double>(gz - gz_bias) * dt_real;

        loop_count++;
        if (loop_count >= PRINT_INTERVAL)
        {
            double avg_dt_run = dt_sum / dt_count;
            std::cout << "gz_raw: " << gz
                      << " | gz_dps(16.4): " << gz_no_bias_dps
                      << " | gz_dps(bias_fixed): " << gz_bias_fixed_dps
                      << " | dt_ms: " << dt_real * 1000.0
                      << " | avg_dt_ms: " << avg_dt_run * 1000.0
                      << " | ang_real_no_bias: " << angle_real_dt_no_bias
                      << " | ang_real_bias: " << angle_real_dt_bias_fixed
                      << " | ang_fixed_bias: " << angle_fixed_dt_bias_fixed
                      << std::endl;
            loop_count = 0;
        }

        usleep(10000);
    }

    double avg_dt_run = (dt_count > 0) ? (dt_sum / dt_count) : 0.0;
    double estimated_lsb_per_dps = 0.0;
    if (std::fabs(KNOWN_TEST_ANGLE_DEG) > 1e-6)
        estimated_lsb_per_dps = bias_corrected_gz_dt_integral / KNOWN_TEST_ANGLE_DEG;

    std::cout << "\n================ 诊断结果 ================" << std::endl;
    std::cout << "angle_real_dt_no_bias      = " << angle_real_dt_no_bias << " deg" << std::endl;
    std::cout << "angle_real_dt_bias_fixed   = " << angle_real_dt_bias_fixed << " deg" << std::endl;
    std::cout << "angle_fixed_dt_bias_fixed  = " << angle_fixed_dt_bias_fixed << " deg" << std::endl;
    std::cout << "estimated_lsb_per_dps      = " << estimated_lsb_per_dps << std::endl;
    std::cout << "default_lsb_per_dps        = " << DEFAULT_LSB_PER_DPS << std::endl;
    std::cout << "dt_avg_ms / min / max      = "
              << avg_dt_run * 1000.0 << " / "
              << dt_min * 1000.0 << " / "
              << dt_max * 1000.0 << std::endl;

    std::cout << "\n判读方法:" << std::endl;
    std::cout << "1) 如果静止时 angle_real_dt_no_bias 自己持续漂，而 angle_real_dt_bias_fixed 明显改善，说明零偏是主要问题。" << std::endl;
    std::cout << "2) 如果 angle_fixed_dt_bias_fixed 和 angle_real_dt_bias_fixed 差异很大，说明 dt 假设 10ms 不准。" << std::endl;
    std::cout << "3) 如果已经用了真实 dt 且扣了零偏，转一圈后 angle_real_dt_bias_fixed 仍明显不是 360°，并且 estimated_lsb_per_dps 明显偏离 16.4，说明量程/驱动配置更可疑。" << std::endl;
    std::cout << "4) estimated_lsb_per_dps > 16.4 往往说明你实际应该用更大的灵敏度分母；< 16.4 则相反。" << std::endl;
    std::cout << "==========================================" << std::endl;
}
