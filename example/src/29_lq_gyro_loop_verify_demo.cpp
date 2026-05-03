#include "lq_all_demo.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unistd.h>

#include "../../user_app/include/my_cascade_pid.hpp"
#include "../../user_app/include/my_mpu6050.hpp"

static std::atomic<bool> exit_demo(false);

void signal_handler_gyro_verify(int signum) {
    (void)signum;
    exit_demo = true;
}

struct GyroTurnStage {
    float target_yaw_deg;
    float max_gyro_dps;
    float hold_sec;
};

static float clamp_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void apply_signed_pwm(ls_atim_pwm& motor,
                             ls_gpio& dir,
                             float signed_pwm) {
    if (signed_pwm >= 0.0f) {
        dir.gpio_level_set(GPIO_LOW);
        motor.atim_pwm_set_duty(static_cast<int>(signed_pwm));
    } else {
        dir.gpio_level_set(GPIO_HIGH);
        motor.atim_pwm_set_duty(static_cast<int>(-signed_pwm));
    }
}

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

    // 测试目标从“固定角速度跑固定时间”改为“积分角度目标”。
    // 内层仍然调用 calc_diff_speed_bypass_angle()，所以调的还是角速度环；
    // 这里只负责根据 Yaw 误差生成 Cmd_Gz，并在接近目标角度时自动减速。
    const GyroTurnStage test_stages[] = {
        {0.0f,   0.0f, 1.0f},
        {90.0f, 45.0f, 0.35f},
        {0.0f,  45.0f, 0.35f},
        {-90.0f,45.0f, 0.35f},
        {0.0f,  45.0f, 0.35f},
    };

    const float pos_target_gain = 1.03f;
    const float neg_target_gain = 1.00f;
    const float yaw_kp = 1.55f;
    const float yaw_tolerance_deg = 1.2f;
    const float settle_gyro_dps = 5.0f;
    const float yaw_integral_deadband_dps = 1.2f;
    const float min_turn_rate_dps = 18.0f;
    const float fine_turn_rate_dps = 12.0f;
    const float min_turn_rate_start_deg = 6.0f;
    const float fine_turn_rate_start_deg = 1.2f;

    const float cmd_slew_rate_dps_per_sec = 160.0f;
    const float diff_lpf_alpha = 0.22f;
    const float hold_err_dps = 1.2f;
    const float hold_diff_limit = 0.35f;

    // Demo 专用：直接把角速度环输出映射到 PWM，避免低速编码器量化导致一顿一顿。
    const float spin_pwm_gain = 620.0f;
    const float spin_pwm_min = 980.0f;
    const float spin_diff_deadband = 0.22f;
    const float spin_pwm_lpf_alpha = 0.30f;

    int current_stage = 0;
    float yaw_estimated = 0.0f;
    float stage_hold_time = 0.0f;
    float target_yaw = test_stages[0].target_yaw_deg;
    float target_gyro_z = 0.0f;
    float cmd_target_gyro_z_smooth = 0.0f;
    float diff_speed_filtered = 0.0f;
    float pwm_cmd_l = 0.0f;
    float pwm_cmd_r = 0.0f;
    int loop_count = 0;

    auto last_time = std::chrono::steady_clock::now();

    std::cout << "====== 角速度环 + 角度积分跟踪测试开始 ======" << std::endl;
    std::cout << "目标序列: 0 -> +90 -> 0 -> -90 -> 0 deg" << std::endl;
    std::cout << "观察 Yaw 是否到达目标角度，同时观察 Real_Gz 是否跟随 Cmd_Gz" << std::endl;

    while (!exit_demo) {
        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = now_time - last_time;
        float dt = fs.count();
        last_time = now_time;
        if (dt < 0.001f || dt > 0.1f) dt = 0.01f;

        Mpu6050Data imu_data;
        if (!imu.read_processed_data(imu_data)) {
            usleep(10000);
            continue;
        }

        float yaw_integral_gyro_z = imu_data.gyro_z;
        if (std::fabs(yaw_integral_gyro_z) < yaw_integral_deadband_dps) {
            yaw_integral_gyro_z = 0.0f;
        }
        yaw_estimated += yaw_integral_gyro_z * dt;

        target_yaw = test_stages[current_stage].target_yaw_deg;
        float yaw_error = target_yaw - yaw_estimated;
        float abs_yaw_error = std::fabs(yaw_error);
        bool yaw_reached = abs_yaw_error <= yaw_tolerance_deg &&
                           std::fabs(imu_data.gyro_z) <= settle_gyro_dps;

        if (test_stages[current_stage].max_gyro_dps <= 0.001f) {
            target_gyro_z = 0.0f;
            stage_hold_time += dt;
        } else if (yaw_reached) {
            target_gyro_z = 0.0f;
            stage_hold_time += dt;
        } else {
            float yaw_rate_cmd = yaw_kp * yaw_error;
            float max_rate = test_stages[current_stage].max_gyro_dps;
            yaw_rate_cmd = clamp_float(yaw_rate_cmd, -max_rate, max_rate);
            if (abs_yaw_error > min_turn_rate_start_deg &&
                std::fabs(yaw_rate_cmd) < min_turn_rate_dps) {
                yaw_rate_cmd = (yaw_error >= 0.0f) ? min_turn_rate_dps : -min_turn_rate_dps;
            } else if (abs_yaw_error > fine_turn_rate_start_deg &&
                       std::fabs(yaw_rate_cmd) < fine_turn_rate_dps) {
                yaw_rate_cmd = (yaw_error >= 0.0f) ? fine_turn_rate_dps : -fine_turn_rate_dps;
            }
            target_gyro_z = yaw_rate_cmd;
            stage_hold_time = 0.0f;
        }

        if (stage_hold_time >= test_stages[current_stage].hold_sec) {
            current_stage++;
            if (current_stage >= static_cast<int>(sizeof(test_stages) / sizeof(test_stages[0]))) {
                std::cout << "测试序列完成！最终 Yaw: " << yaw_estimated << " deg" << std::endl;
                break;
            }

            target_yaw = test_stages[current_stage].target_yaw_deg;
            target_gyro_z = 0.0f;
            stage_hold_time = 0.0f;

            // 清控制器与输出历史，但保留 yaw_estimated，它是整段测试的角度参考。
            cascade_ctrl.reset_runtime_state();
            pwm_cmd_l = 0.0f;
            pwm_cmd_r = 0.0f;
            cmd_target_gyro_z_smooth = 0.0f;
            diff_speed_filtered = 0.0f;
            motor_L.atim_pwm_set_duty(0);
            motor_R.atim_pwm_set_duty(0);

            std::cout << "\n>>> 切换目标角度至: " << target_yaw
                      << " deg, 当前 Yaw: " << yaw_estimated << " deg <<<\n" << std::endl;
        }

        float cmd_target_gyro_z = target_gyro_z;
        if (cmd_target_gyro_z > 0.0f) {
            cmd_target_gyro_z *= pos_target_gain;
        } else if (cmd_target_gyro_z < 0.0f) {
            cmd_target_gyro_z *= neg_target_gain;
        }

        float max_step = cmd_slew_rate_dps_per_sec * dt;
        float cmd_delta = cmd_target_gyro_z - cmd_target_gyro_z_smooth;
        cmd_delta = clamp_float(cmd_delta, -max_step, max_step);
        cmd_target_gyro_z_smooth += cmd_delta;

        float diff_speed_raw = cascade_ctrl.calc_diff_speed_bypass_angle(
            cmd_target_gyro_z_smooth, imu_data, dt);
        diff_speed_filtered = diff_speed_filtered * (1.0f - diff_lpf_alpha) +
                              diff_speed_raw * diff_lpf_alpha;

        float diff_speed = diff_speed_filtered;
        float gyro_err = cmd_target_gyro_z_smooth - imu_data.gyro_z;
        if (std::fabs(gyro_err) < hold_err_dps && std::fabs(diff_speed) < hold_diff_limit) {
            diff_speed = 0.0f;
        }

        if (std::fabs(target_gyro_z) < 0.001f) {
            pwm_cmd_l = 0.0f;
            pwm_cmd_r = 0.0f;
            cascade_ctrl.reset_runtime_state();
            cmd_target_gyro_z_smooth = 0.0f;
            diff_speed_filtered = 0.0f;
            motor_L.atim_pwm_set_duty(0);
            motor_R.atim_pwm_set_duty(0);

            loop_count++;
            if (loop_count >= 10) {
                std::cout << "Trg_Yaw: " << target_yaw
                          << " | Yaw: " << yaw_estimated
                          << " | Yaw_Err: " << (target_yaw - yaw_estimated)
                          << " | Cmd_Gz: " << target_gyro_z
                          << " | Real_Gz: " << imu_data.gyro_z
                          << " | Diff: " << 0.0f
                          << std::endl;
                loop_count = 0;
            }

            usleep(10000);
            continue;
        }

        float raw_pwm = 0.0f;
        if (std::fabs(diff_speed) >= spin_diff_deadband) {
            raw_pwm = spin_pwm_gain * std::fabs(diff_speed) + spin_pwm_min;
        }
        if (raw_pwm > max_pwm) raw_pwm = max_pwm;

        float raw_pwm_l = (diff_speed >= 0.0f) ? -raw_pwm : raw_pwm;
        float raw_pwm_r = (diff_speed >= 0.0f) ? raw_pwm : -raw_pwm;

        pwm_cmd_l = pwm_cmd_l * (1.0f - spin_pwm_lpf_alpha) + raw_pwm_l * spin_pwm_lpf_alpha;
        pwm_cmd_r = pwm_cmd_r * (1.0f - spin_pwm_lpf_alpha) + raw_pwm_r * spin_pwm_lpf_alpha;

        apply_signed_pwm(motor_L, motor_L_Dir, pwm_cmd_l);
        apply_signed_pwm(motor_R, motor_R_Dir, pwm_cmd_r);

        loop_count++;
        if (loop_count >= 10) {
            std::cout << "Trg_Yaw: " << target_yaw
                      << " | Yaw: " << yaw_estimated
                      << " | Yaw_Err: " << (target_yaw - yaw_estimated)
                      << " | Cmd_Gz: " << target_gyro_z
                      << " | Real_Gz: " << imu_data.gyro_z
                      << " | Gz_Err: " << (target_gyro_z - imu_data.gyro_z)
                      << " | Diff: " << diff_speed
                      << std::endl;
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
