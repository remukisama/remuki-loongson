#include "my_cascade_pid.hpp"
#include "car_config.hpp"

#include <cmath>

static float clamp_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

AttitudeCascade::AttitudeCascade()
    : current_yaw(0.0f),
      gyro_z_filtered_(0.0f),
      last_gyro_z_measurement_(0.0f),
      last_gyro_output_(0.0f) {
}

void AttitudeCascade::init() {
    angle_pid.P = car_config::kAnglePidP;
    angle_pid.I = car_config::kAnglePidI;
    angle_pid.D = car_config::kAnglePidD;
    angle_pid.err = 0.0f;
    angle_pid.last_err = 0.0f;
    angle_pid.integral = 0.0f;
    angle_pid.int_max = car_config::kAnglePidIntegralLimit;
    angle_pid.out_max = car_config::kAnglePidOutputLimit;
    angle_pid.output = 0.0f;

    gyro_pid.P = car_config::kGyroPidP;
    gyro_pid.I = car_config::kGyroPidI;
    gyro_pid.D = car_config::kGyroPidD;
    gyro_pid.err = 0.0f;
    gyro_pid.last_err = 0.0f;
    gyro_pid.integral = 0.0f;
    gyro_pid.int_max = car_config::kGyroPidIntegralLimit;
    gyro_pid.out_max = car_config::kGyroPidOutputLimit;
    gyro_pid.output = 0.0f;

    reset_runtime_state();
}

void AttitudeCascade::reset_runtime_state() {
    current_yaw = 0.0f;

    angle_pid.err = 0.0f;
    angle_pid.last_err = 0.0f;
    angle_pid.integral = 0.0f;
    angle_pid.output = 0.0f;

    gyro_pid.err = 0.0f;
    gyro_pid.last_err = 0.0f;
    gyro_pid.integral = 0.0f;
    gyro_pid.output = 0.0f;

    gyro_z_filtered_ = 0.0f;
    last_gyro_z_measurement_ = 0.0f;
    last_gyro_output_ = 0.0f;
}

void AttitudeCascade::initialize_recovery_state(float current_gyro_z, float current_diff_speed) {
    gyro_z_filtered_ = current_gyro_z;
    last_gyro_z_measurement_ = current_gyro_z;
    last_gyro_output_ = current_diff_speed;

    gyro_pid.err = 0.0f;
    gyro_pid.last_err = 0.0f;
    if (gyro_pid.I > 0.0001f) {
        gyro_pid.integral = current_diff_speed / gyro_pid.I;
        if (gyro_pid.integral > gyro_pid.int_max) gyro_pid.integral = gyro_pid.int_max;
        if (gyro_pid.integral < -gyro_pid.int_max) gyro_pid.integral = -gyro_pid.int_max;
    } else {
        gyro_pid.integral = 0.0f;
    }
    gyro_pid.output = current_diff_speed;

    angle_pid.err = 0.0f;
    angle_pid.last_err = 0.0f;
    angle_pid.integral = 0.0f;
    angle_pid.output = 0.0f;
    current_yaw = 0.0f;
}

float AttitudeCascade::calc_diff_speed(float target_angle,
                                       const Mpu6050Data& imu_data,
                                       float dt) {
    float target_gyro_z = calc_target_gyro_for_yaw(target_angle, imu_data, dt);
    return calc_diff_speed_bypass_angle(target_gyro_z, imu_data, dt);
}

float AttitudeCascade::calc_target_gyro_for_yaw(float target_angle,
                                                const Mpu6050Data& imu_data,
                                                float dt) {
    return calc_target_gyro_for_yaw(
        target_angle, imu_data, dt, car_config::kYawTrackDefaultMaxRateDps);
}

float AttitudeCascade::calc_target_gyro_for_yaw(float target_angle,
                                                const Mpu6050Data& imu_data,
                                                float dt,
                                                float max_gyro_dps) {
    if (dt <= 0.0001f || dt > 0.1f) {
        dt = car_config::kOuterLoopFallbackPeriodSec;
    }

    float gyro_z = imu_data.gyro_z;
    if (std::fabs(gyro_z) > car_config::kGyroRawSpikeRejectDps) {
        gyro_z = last_gyro_z_measurement_;
    }
    gyro_z = clamp_float(
        gyro_z,
        -car_config::kGyroMeasurementLimitDps,
        car_config::kGyroMeasurementLimitDps);
    if (std::fabs(gyro_z) < car_config::kYawIntegralDeadBandDps) {
        gyro_z = 0.0f;
    }

    current_yaw += gyro_z * dt;

    angle_pid.err = target_angle - current_yaw;
    angle_pid.output = car_config::kYawTrackKp * angle_pid.err;
    angle_pid.output = clamp_float(angle_pid.output, -max_gyro_dps, max_gyro_dps);

    float abs_yaw_error = std::fabs(angle_pid.err);
    if (abs_yaw_error <= car_config::kYawTrackToleranceDeg &&
        std::fabs(imu_data.gyro_z) <= car_config::kYawTrackSettleGyroDps) {
        angle_pid.output = 0.0f;
    } else if (abs_yaw_error > car_config::kYawTrackMinRateStartDeg &&
               std::fabs(angle_pid.output) < car_config::kYawTrackMinRateDps) {
        angle_pid.output = (angle_pid.err >= 0.0f) ?
                           car_config::kYawTrackMinRateDps :
                           -car_config::kYawTrackMinRateDps;
    } else if (abs_yaw_error > car_config::kYawTrackFineRateStartDeg &&
               std::fabs(angle_pid.output) < car_config::kYawTrackFineRateDps) {
        angle_pid.output = (angle_pid.err >= 0.0f) ?
                           car_config::kYawTrackFineRateDps :
                           -car_config::kYawTrackFineRateDps;
    }

    angle_pid.last_err = angle_pid.err;
    return angle_pid.output;
}

bool AttitudeCascade::yaw_target_reached(float target_angle,
                                         const Mpu6050Data& imu_data) const {
    float yaw_error = target_angle - current_yaw;
    return std::fabs(yaw_error) <= car_config::kYawTrackToleranceDeg &&
           std::fabs(imu_data.gyro_z) <= car_config::kYawTrackSettleGyroDps;
}

float AttitudeCascade::calc_diff_speed_bypass_angle(float target_gyro_z,
                                                    const Mpu6050Data& imu_data,
                                                    float dt) {
    if (dt <= 0.0001f || dt > 0.1f) {
        dt = 0.01f;
    }

    float gyro_z = imu_data.gyro_z;
    if (std::fabs(gyro_z) > car_config::kGyroRawSpikeRejectDps) {
        gyro_z = last_gyro_z_measurement_;
    }
    gyro_z = clamp_float(
        gyro_z,
        -car_config::kGyroMeasurementLimitDps,
        car_config::kGyroMeasurementLimitDps);

    const float gyro_lpf_alpha = car_config::kGyroLpfAlpha;
    gyro_z_filtered_ = gyro_z_filtered_ * (1.0f - gyro_lpf_alpha) +
                       gyro_z * gyro_lpf_alpha;
    gyro_z = gyro_z_filtered_;
    if (std::fabs(gyro_z) < car_config::kGyroDeadBandDps) {
        gyro_z = 0.0f;
    }

    gyro_pid.err = target_gyro_z - gyro_z;
    gyro_pid.integral += gyro_pid.err * dt;
    gyro_pid.integral = clamp_float(
        gyro_pid.integral, -gyro_pid.int_max, gyro_pid.int_max);

    float feedforward = target_gyro_z * car_config::kGyroFeedforwardGain;
    float d_term = -gyro_pid.D * (gyro_z - last_gyro_z_measurement_) / dt;
    last_gyro_z_measurement_ = gyro_z;

    gyro_pid.output = feedforward +
                      gyro_pid.P * gyro_pid.err +
                      gyro_pid.I * gyro_pid.integral +
                      d_term;

    gyro_pid.last_err = gyro_pid.err;

    float max_delta_output = car_config::kDiffOutputMaxDelta;
    if ((target_gyro_z > car_config::kReverseGyroTargetThresholdDps &&
         gyro_z < -car_config::kReverseGyroRealThresholdDps) ||
        (target_gyro_z < -car_config::kReverseGyroTargetThresholdDps &&
         gyro_z > car_config::kReverseGyroRealThresholdDps)) {
        max_delta_output = car_config::kDiffOutputReverseMaxDelta;
    }

    float delta_output = gyro_pid.output - last_gyro_output_;
    delta_output = clamp_float(
        delta_output,
        -max_delta_output,
        max_delta_output);
    gyro_pid.output = last_gyro_output_ + delta_output;
    last_gyro_output_ = gyro_pid.output;

    gyro_pid.output = clamp_float(
        gyro_pid.output, -gyro_pid.out_max, gyro_pid.out_max);

    if (target_gyro_z == 0.0f) {
        float abs_gyro_z = std::fabs(gyro_z);
        if (abs_gyro_z > car_config::kStraightHighGyroLimitDps) {
            gyro_pid.output = clamp_float(
                gyro_pid.output,
                -car_config::kStraightHighGyroDiffLimit,
                car_config::kStraightHighGyroDiffLimit);
        }
    }

    return gyro_pid.output;
}
