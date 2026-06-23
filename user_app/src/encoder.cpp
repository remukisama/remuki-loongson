#include "user_app.hpp"
#include <atomic>
#include <iostream>
#include <cmath>

// 初始化编码器计数通道
ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

// =====================================================================
// 编码器滤波：针对 A相+方向信号 的噪声优化
// 
// 噪声来源：
// 1. 方向信号受电机PWM电磁干扰，瞬间翻转导致编码器符号跳变
// 2. A相脉冲在低速时采样不足，读数波动大
// 
// 滤波策略（折中方案，延迟≈33ms）：
// 1. 冷启动：直接接受第一个读数（不管是否超过阈值）
// 2. 垃圾检测：|raw| > 3.0 m/s 视为无效读数
// 3. 尖峰检测：|raw - filtered| > 2.0 m/s 视为方向信号大翻转
// 4. 低通滤波：alpha=0.6 平滑剩余噪声
// =====================================================================

static float g_filtered_L = 0.0f;
static float g_filtered_R = 0.0f;
static bool g_initialized_L = false;
static bool g_initialized_R = false;
static const float kFilterAlpha = 0.8f;
static const float kMaxValidSpeed = 3.0f;    // 垃圾值阈值
static const float kSpikeThreshold = 2.0f;   // 尖峰检测阈值

static float filter_encoder(float raw, float filtered, bool &initialized) {
    // 1. 冷启动：直接接受第一个读数（优先于垃圾检测）
    if (!initialized) {
        initialized = true;
        return raw;
    }
    
    // 2. 垃圾检测：|raw| > 3.0 视为无效
    if (std::fabs(raw) > kMaxValidSpeed) {
        return filtered;  // 保持上一次滤波值
    }
    
    // 3. 尖峰检测：|raw - filtered| > 2.0 视为方向信号大翻转
    float diff = raw - filtered;
    if (std::fabs(diff) > kSpikeThreshold) {
        return filtered;  // 保持上一次滤波值
    }
    
    // 4. 低通滤波
    return filtered + diff * kFilterAlpha;
}

float encoder_num_L(void)
{
    // 取反：落地后 DIR=LOW=前进，编码器原始值为负，取反后为正
    float raw = -encoder_L.encoder_get_count() * car_config::kEncoderRpsToMS;
    g_filtered_L = filter_encoder(raw, g_filtered_L, g_initialized_L);
    return g_filtered_L;
}

float encoder_num_R(void)
{
    // 取反：落地后 DIR=LOW=前进，编码器原始值为负，取反后为正
    float raw = encoder_R.encoder_get_count() * car_config::kEncoderRpsToMS;
    g_filtered_R = filter_encoder(raw, g_filtered_R, g_initialized_R);
    return g_filtered_R;
}
