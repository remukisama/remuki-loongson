#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_encoder_pwm_demo.cpp
 * @brief   编码器 PWM 输出模式测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 编码器功能，用于测试 PWM 控制器的编码器功能.
 ********************************************************************************/

/********************************************************************************
 * @brief   编码器 PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    初始构造方法有两种, 多文件赋值使用时也有两种方法.
 ********************************************************************************/
void lq_encoder_pwm_demo(void)
{
    ls_encoder_pwm enc1(ENC_PWM0_PIN64, PIN_72);
    ls_encoder_pwm enc2(ENC_PWM1_PIN65, PIN_73);
    ls_encoder_pwm enc3(ENC_PWM2_PIN66, PIN_74);
    ls_encoder_pwm enc4(ENC_PWM3_PIN67, PIN_75);

    while(ls_system_running.load())
    {
        printf("encoder count1: %-6.2f\n", enc1.encoder_get_count());
        printf("encoder count2: %-6.2f\n", enc2.encoder_get_count());
        printf("encoder count3: %-6.2f\n", enc3.encoder_get_count());
        printf("encoder count4: %-6.2f\n\n", enc4.encoder_get_count());
        usleep(50000);
    }
}
