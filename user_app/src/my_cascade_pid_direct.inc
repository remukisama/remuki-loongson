float AttitudeCascade::calc_diff_speed(float target_angle, const Mpu6050Data& data, float dt) {
    return calc_diff_speed_bypass_angle(calc_target_gyro_for_yaw(target_angle, data, dt), data, dt);
}

float AttitudeCascade::calc_target_gyro_for_yaw(float target_angle, const Mpu6050Data& data, float dt) {
    return calc_target_gyro_for_yaw(target_angle, data, dt, car_config::kYawTrackDefaultMaxRateDps);
}
