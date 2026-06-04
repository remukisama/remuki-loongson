#include "user_app.hpp"
#include <atomic>
#include <iostream>

// 初始化编码器计数通道
ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

// =====================================================================
// 返回车轮线速度 (m/s)
// encoder_get_count() 返回电机轴 RPS
// m/s = RPS × (30/68) × π × 0.064 ≈ RPS × 0.08870
// =====================================================================

float encoder_num_L(void)
{
    return -encoder_L.encoder_get_count() * car_config::kEncoderRpsToMS;
}

float encoder_num_R(void)
{
    return encoder_R.encoder_get_count() * car_config::kEncoderRpsToMS;
}