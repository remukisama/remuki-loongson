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
