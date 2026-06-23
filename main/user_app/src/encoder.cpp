#include "user_app.hpp"
#include <atomic>
#include <iostream>

// 初始化编码器计数通道
ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

// =====================================================================
// 返回车轮线速度 (m/s)，带简单一阶低通滤波
// =====================================================================

static float g_filtered_L = 0.0f;
static float g_filtered_R = 0.0f;
static const float kFilterAlpha = 0.3f;

float encoder_num_L(void)
{
    float raw = encoder_L.encoder_get_count() * car_config::kEncoderRpsToMS;
    g_filtered_L += (raw - g_filtered_L) * kFilterAlpha;
    return g_filtered_L;
}

float encoder_num_R(void)
{
    float raw = -encoder_R.encoder_get_count() * car_config::kEncoderRpsToMS;
    g_filtered_R += (raw - g_filtered_R) * kFilterAlpha;
    return g_filtered_R;
}
