#include "my_cascade_pid.hpp"
#include "car_config.hpp"

AttitudeCascade::AttitudeCascade() : current_yaw(0.0f) {
}

void AttitudeCascade::init() {
    // 1. 初始化外环（角度环）的参数：输出是目标角速度 (Yaw Rate)
    // 通常角度环只需要比例控制 (P)，因为角速度和角度有严格积分关系
    angle_pid.P = car_config::kAnglePidP;  // 对齐 demo 27 的角度环参数
    angle_pid.I = car_config::kAnglePidI;
    angle_pid.D = car_config::kAnglePidD;
    angle_pid.err = 0;
    angle_pid.last_err = 0;
    angle_pid.integral = 0;
    angle_pid.int_max = car_config::kAnglePidIntegralLimit;
    angle_pid.out_max = car_config::kAnglePidOutputLimit; // 限制最大角速度(度/秒)，与示例角度环量程保持一致
    angle_pid.output = 0;

    // 2. 初始化中环（角速度环）的参数：输出是左右轮目标差速 (Speed Diff_Goal)
    // 角速度环起到阻尼和平滑作用，一般使用 PD 或 PI 控制
    gyro_pid.P = car_config::kGyroPidP;  // 迁移自 Demo26 的稳定参数
    gyro_pid.I = car_config::kGyroPidI;
    gyro_pid.D = car_config::kGyroPidD; // 迁移自 Demo26 的稳定参数
    gyro_pid.err = 0;
    gyro_pid.last_err = 0;
    gyro_pid.integral = 0;
    gyro_pid.int_max = car_config::kGyroPidIntegralLimit;
    gyro_pid.out_max = car_config::kGyroPidOutputLimit;   // 对齐 Demo26 差速上限，优先避免甩尾
    gyro_pid.output = 0;
    
    current_yaw = 0.0f;
}

float AttitudeCascade::calc_diff_speed(float target_angle, const Mpu6050Data& imu_data, float dt) {
    // Linux 下循环周期会抖动，避免 dt 异常导致微分项尖峰
    if (dt <= 0.0001f || dt > 0.1f) dt = 0.01f;

    // 1. 根据角速度积分粗略计算偏航角 (注意: 精度要求高时可混入地磁计或使用DMP)
    float gyro_z = imu_data.gyro_z;
    if (gyro_z > -car_config::kGyroDeadBandDps && gyro_z < car_config::kGyroDeadBandDps) gyro_z = 0.0f; // 对齐 demo 的去底噪逻辑
    current_yaw += gyro_z * dt;     // 计算当期航向角(度)

    // ==================【最外层：角度环(Yaw)】===================
    // 计算角度偏差
    angle_pid.err = target_angle - current_yaw;
    
    // 角度环 PI 计算
    angle_pid.integral += angle_pid.err * dt;
    if (angle_pid.integral > angle_pid.int_max) angle_pid.integral = angle_pid.int_max;
    if (angle_pid.integral < -angle_pid.int_max) angle_pid.integral = -angle_pid.int_max;

    // 角度环输出 -> [目标角速度_Z]
    angle_pid.output = angle_pid.P * angle_pid.err + 
                       angle_pid.I * angle_pid.integral +
                       angle_pid.D * (angle_pid.err - angle_pid.last_err) / dt;
    angle_pid.last_err = angle_pid.err;
                       
    // 输出限幅保护
    if (angle_pid.output > angle_pid.out_max) angle_pid.output = angle_pid.out_max;
    if (angle_pid.output < -angle_pid.out_max) angle_pid.output = -angle_pid.out_max;
    
    float target_gyro_z = angle_pid.output;

    // ==================【中间层：角速度环(Yaw Rate)】===================
    // 计算角速度偏差
    gyro_pid.err = target_gyro_z - gyro_z;

    // 角速度环 PID 计算
    gyro_pid.integral += gyro_pid.err * dt;
    if (gyro_pid.integral > gyro_pid.int_max) gyro_pid.integral = gyro_pid.int_max;
    if (gyro_pid.integral < -gyro_pid.int_max) gyro_pid.integral = -gyro_pid.int_max;

    // 角速度环输出 -> [左右轮差速分配量]
    gyro_pid.output = gyro_pid.P * gyro_pid.err + 
                      gyro_pid.I * gyro_pid.integral + 
                      gyro_pid.D * (gyro_pid.err - gyro_pid.last_err) / dt;

    gyro_pid.last_err = gyro_pid.err;

    // 差速限幅保护，避免在底盘侧滑
    if (gyro_pid.output > gyro_pid.out_max) gyro_pid.output = gyro_pid.out_max;
    if (gyro_pid.output < -gyro_pid.out_max) gyro_pid.output = -gyro_pid.out_max;

    // 返回要传递给内环(左右轮速度环)的差速目标值
    return gyro_pid.output;
}

float AttitudeCascade::calc_diff_speed_bypass_angle(float target_gyro_z, const Mpu6050Data& imu_data, float dt) {
    if (dt <= 0.0001f || dt > 0.1f) dt = 0.01f;

    float gyro_z = imu_data.gyro_z;
    // IMU测量限幅与低通：迁移自 Demo26 的抗尖峰策略
    if (gyro_z > car_config::kGyroMeasurementLimitDps) gyro_z = car_config::kGyroMeasurementLimitDps;
    if (gyro_z < -car_config::kGyroMeasurementLimitDps) gyro_z = -car_config::kGyroMeasurementLimitDps;
    static float gyro_z_filt = 0.0f;
    const float gyro_lpf_alpha = car_config::kGyroLpfAlpha;
    gyro_z_filt = gyro_z_filt * (1.0f - gyro_lpf_alpha) + gyro_z * gyro_lpf_alpha;
    gyro_z = gyro_z_filt;
    if (gyro_z > -car_config::kGyroDeadBandDps && gyro_z < car_config::kGyroDeadBandDps) gyro_z = 0.0f; // 去底噪

    // 不计算角度环，直接将 target_gyro_z 输入给角速度环
    gyro_pid.err = target_gyro_z - gyro_z;

    // 角速度环 PID 计算
    gyro_pid.integral += gyro_pid.err * dt;
    if (gyro_pid.integral > gyro_pid.int_max) gyro_pid.integral = gyro_pid.int_max;
    if (gyro_pid.integral < -gyro_pid.int_max) gyro_pid.integral = -gyro_pid.int_max;

    // 保留小前馈，减小突刺同时兼顾转向响应
    float feedforward = target_gyro_z * car_config::kGyroFeedforwardGain;
    
    // 【新增2】：由于目标角速度来自视觉PD，会发生突变。改为对测量值(实际角速度)进行微分，防止"微分突变"导致电机抽搐
    static float last_gyro_z = gyro_z;
    float d_term = -gyro_pid.D * (gyro_z - last_gyro_z) / dt;
    last_gyro_z = gyro_z;

    // 角速度环输出 -> 前馈 + PI控制 + 测量值微分
    gyro_pid.output = feedforward + 
                      gyro_pid.P * gyro_pid.err + 
                      gyro_pid.I * gyro_pid.integral + 
                      d_term;

    gyro_pid.last_err = gyro_pid.err;

    // 对差速输出增加变化率限制，抑制摆尾
    static float last_output = 0.0f;
    float max_delta_output = car_config::kDiffOutputMaxDelta; // 对齐Demo26节奏，进一步抑制突刺
    float delta_output = gyro_pid.output - last_output;
    if (delta_output > max_delta_output) delta_output = max_delta_output;
    if (delta_output < -max_delta_output) delta_output = -max_delta_output;
    gyro_pid.output = last_output + delta_output;
    last_output = gyro_pid.output;

    // 差速限幅保护，避免在底盘侧滑
    if (gyro_pid.output > gyro_pid.out_max) gyro_pid.output = gyro_pid.out_max;
    if (gyro_pid.output < -gyro_pid.out_max) gyro_pid.output = -gyro_pid.out_max;

    // 直行目标下遇到异常高角速度时，进一步收紧差速，避免瞬时甩尾
    if (target_gyro_z == 0.0f) {
        float abs_gyro_z = (gyro_z >= 0.0f) ? gyro_z : -gyro_z;
        if (abs_gyro_z > car_config::kStraightHighGyroLimitDps) {
            if (gyro_pid.output > car_config::kStraightHighGyroDiffLimit) gyro_pid.output = car_config::kStraightHighGyroDiffLimit;
            if (gyro_pid.output < -car_config::kStraightHighGyroDiffLimit) gyro_pid.output = -car_config::kStraightHighGyroDiffLimit;
        }
    }

    // 返回差速目标值
    return gyro_pid.output;
}
