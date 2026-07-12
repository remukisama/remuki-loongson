#include "my_cascade_pid.hpp"
#include "car_config.hpp"
#include <cmath>

namespace {
float clamp_float(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}

AttitudeCascade::AttitudeCascade()
    : current_yaw(0.0f), gyro_z_filtered_(0.0f),
      last_gyro_z_measurement_(0.0f), last_gyro_output_(0.0f) {}

void AttitudeCascade::init() {
    angle_pid.P = car_config::kAnglePidP;
    angle_pid.I = car_config::kAnglePidI;
    angle_pid.D = car_config::kAnglePidD;
    angle_pid.int_max = car_config::kAnglePidIntegralLimit;
    angle_pid.out_max = car_config::kAnglePidOutputLimit;
    gyro_pid.P = car_config::kGyroPidP;
    gyro_pid.I = car_config::kGyroPidI;
    gyro_pid.D = car_config::kGyroPidD;
    gyro_pid.int_max = car_config::kGyroPidIntegralLimit;
    gyro_pid.out_max = car_config::kGyroPidOutputLimit;
    reset_runtime_state();
}

void AttitudeCascade::reset_runtime_state() {
    current_yaw = 0.0f;
    angle_pid.err = angle_pid.last_err = angle_pid.integral = angle_pid.output = 0.0f;
    gyro_pid.err = gyro_pid.last_err = gyro_pid.integral = gyro_pid.output = 0.0f;
    gyro_z_filtered_ = last_gyro_z_measurement_ = last_gyro_output_ = 0.0f;
}

void AttitudeCascade::initialize_recovery_state(float gyro_z, float diff_speed) {
    gyro_z_filtered_ = gyro_z;
    last_gyro_z_measurement_ = gyro_z;
    last_gyro_output_ = clamp_float(diff_speed, -gyro_pid.out_max, gyro_pid.out_max);
    gyro_pid.err = gyro_pid.last_err = 0.0f;
    gyro_pid.integral = std::fabs(gyro_pid.I) > 0.0001f ?
        clamp_float(last_gyro_output_ / gyro_pid.I, -gyro_pid.int_max, gyro_pid.int_max) : 0.0f;
    gyro_pid.output = last_gyro_output_;
    angle_pid.err = angle_pid.last_err = angle_pid.integral = angle_pid.output = 0.0f;
    current_yaw = 0.0f;
}

float AttitudeCascade::calc_diff_speed(float target_angle, const Mpu6050Data& data, float dt) {
    return calc_diff_speed_bypass_angle(calc_target_gyro_for_yaw(target_angle, data, dt), data, dt);
}

float AttitudeCascade::calc_target_gyro_for_yaw(float target_angle, const Mpu6050Data& data, float dt) {
    return calc_target_gyro_for_yaw(target_angle, data, dt, angle_pid.out_max);
}

float AttitudeCascade::calc_target_gyro_for_yaw(float target_angle,
                                                const Mpu6050Data& data,
                                                float dt,
                                                float max_gyro_dps) {
    if (dt <= 0.0001f || dt > 0.1f) dt = 0.01f;
    current_yaw += data.gyro_z * dt;
    angle_pid.err = target_angle - current_yaw;
    angle_pid.integral = clamp_float(angle_pid.integral + angle_pid.err * dt,
                                     -angle_pid.int_max,
                                     angle_pid.int_max);
    const float derivative = (angle_pid.err - angle_pid.last_err) / dt;
    angle_pid.output = angle_pid.P * angle_pid.err +
                       angle_pid.I * angle_pid.integral +
                       angle_pid.D * derivative;
    angle_pid.output = clamp_float(angle_pid.output, -max_gyro_dps, max_gyro_dps);
    angle_pid.last_err = angle_pid.err;
    return angle_pid.output;
}

bool AttitudeCascade::yaw_target_reached(float target_angle, const Mpu6050Data& data) const {
    return std::fabs(target_angle - current_yaw) <= car_config::kAngleTargetToleranceDeg &&
           std::fabs(data.gyro_z) <= car_config::kAngleSettleGyroDps;
}

float AttitudeCascade::calc_diff_speed_bypass_angle(float target_gyro_z,
                                                    const Mpu6050Data& data,
                                                    float dt) {
    if (dt <= 0.0001f || dt > 0.1f) dt = 0.005f;

    // --- 信号调理：尖峰拒绝 → 测量限幅 → 一阶低通滤波 → 死区 ---
    float gyro_z = data.gyro_z;

    // 1. 尖峰拒绝：单帧异常值（如 MPU6050 静止时偶发 -359°/s）用上次有效值替代
    if (std::fabs(gyro_z) > car_config::kGyroRawSpikeRejectDps) {
        gyro_z = last_gyro_z_measurement_;
    }

    // 2. 测量限幅：防止超出物理量程的异常值进入 PID
    gyro_z = clamp_float(gyro_z,
                         -car_config::kGyroMeasurementLimitDps,
                         car_config::kGyroMeasurementLimitDps);

    // 3. 一阶低通滤波：y[n] = y[n-1] + alpha * (x[n] - y[n-1])
    gyro_z_filtered_ = gyro_z_filtered_ * (1.0f - car_config::kGyroLpfAlpha) +
                       gyro_z * car_config::kGyroLpfAlpha;

    // 4. 死区：滤除静止零漂
    if (std::fabs(gyro_z_filtered_) < car_config::kGyroDeadBandDps) {
        gyro_z_filtered_ = 0.0f;
    }

    gyro_pid.err = target_gyro_z - gyro_z_filtered_;
    gyro_pid.integral = clamp_float(gyro_pid.integral + gyro_pid.err * dt,
                                    -gyro_pid.int_max,
                                    gyro_pid.int_max);
    const float derivative = (gyro_pid.err - gyro_pid.last_err) / dt;
    const float feedforward = target_gyro_z * car_config::kGyroFeedforwardGain;
    float out = feedforward +
                gyro_pid.P * gyro_pid.err +
                gyro_pid.I * gyro_pid.integral +
                gyro_pid.D * derivative;
    out = clamp_float(out, -gyro_pid.out_max, gyro_pid.out_max);

    last_gyro_z_measurement_ = gyro_z;
    gyro_pid.output = out;
    gyro_pid.last_err = gyro_pid.err;
    last_gyro_output_ = out;
    return out;
}
