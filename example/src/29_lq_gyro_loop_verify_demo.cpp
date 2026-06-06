#include "lq_all_demo.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unistd.h>

#include "../../user_app/include/my_cascade_pid.hpp"
#include "../../user_app/include/my_mpu6050.hpp"
#include "../../user_app/include/car_config.hpp"

static std::atomic<bool> exit_demo(false);

void signal_handler_gyro_verify(int signum) {
    (void)signum;
    exit_demo = true;
}

// ===================================================================
// 零前进速度角速度环测试（原地转向）
//    绕开速度环，直接用角速度环输出差速映射 PWM，
//    使用 car_config.hpp 中的 kGyroPidP/D/kGyroFeedforwardGain 等参数。
//    目标序列: 0 → ±35 → ±70 → 0 dps 阶跃。
// ===================================================================
void lq_gyro_loop_verify_demo(void)
{
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_INV);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_INV);

    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    MyMpu6050 imu;
    if (!imu.init()) {
        std::cout << "IMU 初始化失败，退出测试" << std::endl;
        return;
    }

    AttitudeCascade cascade_ctrl;
    cascade_ctrl.init();

    const int max_pwm = 6000;

    // 目标角速度阶跃序列 (dps)
    const float gyro_targets[] = {0.0f, 35.0f, 70.0f, 0.0f, -35.0f, -70.0f, 0.0f};
    const int num_targets = sizeof(gyro_targets) / sizeof(gyro_targets[0]);
    const int hold_ms = 900;

    // PWM 映射参数 (差速 m/s → PWM)
    const float pwm_gain = 500.0f;     // diff_speed 到 PWM 的增益
    const float pwm_min  = 800.0f;     // 最低 PWM (克服静摩擦)
    const float pwm_max  = 4500.0f;    // PWM 上限
    const float pwm_lpf  = 0.40f;      // PWM 低通滤波 (加快响应)

    int target_idx = 0;
    float target_gyro_z = gyro_targets[target_idx];
    float pwm_cmd_l = 0.0f, pwm_cmd_r = 0.0f;
    int elapsed_ms = 0;
    int loop_count = 0;

    auto last_time = std::chrono::steady_clock::now();

    std::cout << "====== 零前进速度角速度环阶跃测试 ======" << std::endl;
    std::cout << "目标序列: 0 -> +35 -> +70 -> 0 -> -35 -> -70 -> 0 dps" << std::endl;
    std::cout << "每阶保持 " << hold_ms << " ms，共 " << num_targets << " 阶" << std::endl;
    printf("[参数] gyro(P=%.3f D=%.4f ff=%.3f dead=%.1f lpf=%.2f dOutMax=%.1f)\n",
           car_config::kGyroPidP, car_config::kGyroPidD, car_config::kGyroFeedforwardGain,
           car_config::kGyroDeadBandDps, car_config::kGyroLpfAlpha,
           car_config::kDiffOutputMaxDelta);
    std::cout << "小车会原地左右摆动，手边准备 Ctrl+C。" << std::endl;
    std::cout << "按 Ctrl+C 退出。" << std::endl;

    while (!exit_demo) {
        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = now_time - last_time;
        float dt = fs.count();
        last_time = now_time;
        if (dt < 0.001f || dt > 0.1f) dt = 0.01f;

        Mpu6050Data imu_data;
        if (!imu.read_processed_data(imu_data)) {
            usleep(5000);
            continue;
        }

        // 阶跃切换
        elapsed_ms += (int)(dt * 1000.0f);
        if (elapsed_ms >= hold_ms) {
            elapsed_ms = 0;
            target_idx++;
            if (target_idx >= num_targets) {
                std::cout << "测试序列完成。" << std::endl;
                break;
            }
            target_gyro_z = gyro_targets[target_idx];
            printf("[GYRO_STEP] idx=%d/%d target=%.1f dps\n",
                   target_idx + 1, num_targets, target_gyro_z);
        }

        // 应用方向补偿
        float cmd = target_gyro_z;
        if (cmd > 0.0f) cmd *= car_config::kGyroTargetPositiveGain;
        else if (cmd < 0.0f) cmd *= car_config::kGyroTargetNegativeGain;

        // 通过标准角速度环计算差速
        float diff_speed = cascade_ctrl.calc_diff_speed_bypass_angle(cmd, imu_data, dt);
        float diff_before = diff_speed;

        // 差速限幅 (原地转向需较大差速克服静摩擦)
        float max_diff = 4.0f * car_config::kMaxDiffRatio;
        int cap = 0;
        if (diff_speed > max_diff) { diff_speed = max_diff; cap = 1; }
        if (diff_speed < -max_diff) { diff_speed = -max_diff; cap = 1; }

        // 差速 → PWM (绕开速度环)
        float raw_pwm = 0.0f;
        if (std::fabs(diff_speed) > 0.05f) {
            raw_pwm = pwm_gain * std::fabs(diff_speed) + pwm_min;
        }
        if (raw_pwm > pwm_max) raw_pwm = pwm_max;

        float raw_l = (diff_speed >= 0.0f) ? -raw_pwm : raw_pwm;
        float raw_r = (diff_speed >= 0.0f) ? raw_pwm : -raw_pwm;

        pwm_cmd_l = pwm_cmd_l * (1.0f - pwm_lpf) + raw_l * pwm_lpf;
        pwm_cmd_r = pwm_cmd_r * (1.0f - pwm_lpf) + raw_r * pwm_lpf;

        // 输出 PWM
        if (pwm_cmd_l >= 0.0f) {
            motor_L_Dir.gpio_level_set(GPIO_LOW);
            motor_L.atim_pwm_set_duty((int)pwm_cmd_l);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_HIGH);
            motor_L.atim_pwm_set_duty((int)(-pwm_cmd_l));
        }
        if (pwm_cmd_r >= 0.0f) {
            motor_R_Dir.gpio_level_set(GPIO_LOW);
            motor_R.atim_pwm_set_duty((int)pwm_cmd_r);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_HIGH);
            motor_R.atim_pwm_set_duty((int)(-pwm_cmd_r));
        }

        // 打印 (每10个循环 ≈ 100ms 一次)
        loop_count++;
        if (loop_count >= 10) {
            float gf = cascade_ctrl.last_filtered_gyro_z();
            // PID 各分量分解（便于远程调试角速度环参数）
            float pid_p = cascade_ctrl.gyro_pid.P * cascade_ctrl.gyro_pid.err;
            float pid_i = cascade_ctrl.gyro_pid.I * cascade_ctrl.gyro_pid.integral;
            float ff    = cmd * car_config::kGyroFeedforwardGain;
            float pid_d = cascade_ctrl.gyro_pid.output - (pid_p + pid_i + ff);
            printf("[GYRO] t=%.2f target=%.1f cmd=%.1f real=%.1f gf=%.1f err=%.1f "
                   "| P=%.1f I=%.1f D=%.1f FF=%.1f out=%.1f "
                   "| diff0=%.1f diff=%.1f cap=%d max=%.1f pwm=%.0f/%.0f\n",
                   elapsed_ms / 1000.0f, target_gyro_z, cmd,
                   imu_data.gyro_z, gf, (cmd - gf),
                   pid_p, pid_i, pid_d, ff, cascade_ctrl.gyro_pid.output,
                   diff_before, diff_speed, cap, max_diff,
                   pwm_cmd_l, pwm_cmd_r);
            loop_count = 0;
        }

        usleep(10000);
    }

    std::cout << "关闭电机..." << std::endl;
    motor_L.atim_pwm_set_duty(0);
    motor_R.atim_pwm_set_duty(0);
    motor_L.atim_pwm_disable();
    motor_R.atim_pwm_disable();
}
