#include "user_app.hpp"
#include <atomic>
#include <iostream>

// 初始化编码器计数通道
// ls_quadrature_encoder encoder_L(0);
// ls_quadrature_encoder encoder_R(1);
ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

//正转是返回正数
int encoder_num_L(void)
{
    int Temp;
    Temp = (int)encoder_L.encoder_get_count();
    // encoder_L.encoder_reset_counter();

    // 极性翻转：如果原来是负号导致正反馈，试试改成正的
    // 根据刚才的数据，左轮得到的是正向读数（随目标值一直增加），所以原有的逻辑可能是合理的
    return -Temp; 
}

int encoder_num_R(void)
{
    int Temp;
    Temp = (int)encoder_R.encoder_get_count();
    // encoder_R.encoder_reset_counter();

    // 极性翻转：如果原来是正号导致右轮不读数或出错，试试翻转极性
    return Temp;
}