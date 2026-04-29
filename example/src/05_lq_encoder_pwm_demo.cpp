#include "main.hpp"

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
    // ls_encoder_pwm enc2(ENC_PWM1_PIN65, PIN_73);
    // ls_encoder_pwm enc3(ENC_PWM2_PIN66, PIN_74);
    ls_encoder_pwm enc4(ENC_PWM3_PIN67, PIN_75);

    // ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 10000, 3000, ATIM_PWM_POL_INV);
    // ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 10000, 3000, ATIM_PWM_POL_INV);
    // ls_gpio pwm1dir(PIN_21, GPIO_MODE_OUT);
    // ls_gpio pwm2dir(PIN_22, GPIO_MODE_OUT);
    // pwm1.atim_pwm_enable();
    // pwm2.atim_pwm_enable();

    while(1)
    {
        printf("encoder count1: %-6.2f\n", enc1.encoder_get_count());
        // printf("encoder count2: %-6.2f\n", enc2.encoder_get_count());
        // printf("encoder count3: %-6.2f\n", enc3.encoder_get_count());
        printf("encoder count4: %-6.2f\n\n", enc4.encoder_get_count());
        usleep(50000);
    }
}
