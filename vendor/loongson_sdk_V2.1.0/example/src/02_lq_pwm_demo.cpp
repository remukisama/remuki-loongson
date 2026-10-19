#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_pwm_demo.cpp
 * @brief   PWM 输出模式测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 PWM 输出功能，用于测试 PWM 引脚的基本功能.
 *!         注意：该例程进攻测试 PWM，因给定占空比过大，请勿连接电机等设备，否则可能会损坏设备.
 ********************************************************************************/

/********************************************************************************
 * @brief   PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    PWM 输出测试, 使用引脚 64, 65, 66, 67 作为输出引脚, 读取当前信息并打印到终端.
 ********************************************************************************/
void lq_pwm_demo(void)
{
    ls_pwm pwm1(PWM0_PIN64, 50, 2000);
    ls_pwm pwm2(PWM1_PIN65, 60, 2000);
    ls_pwm pwm3(PWM2_PIN66, 70, 2000);
    ls_pwm pwm4(PWM3_PIN67, 80, 2000);

    while (ls_system_running.load())
    {
        pwm1.pwm_set_duty(3000);
        pwm2.pwm_set_duty(3000);
        pwm3.pwm_set_duty(3000);
        pwm4.pwm_set_duty(3000);
        printf("pwm1->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm1.pwm_get_gpio(), pwm1.pwm_get_mux(), pwm1.pwm_get_channel(), pwm1.pwm_get_period(), pwm1.pwm_get_duty());
        printf("pwm2->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm2.pwm_get_gpio(), pwm2.pwm_get_mux(), pwm2.pwm_get_channel(), pwm2.pwm_get_period(), pwm2.pwm_get_duty());
        printf("pwm3->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm3.pwm_get_gpio(), pwm3.pwm_get_mux(), pwm3.pwm_get_channel(), pwm3.pwm_get_period(), pwm3.pwm_get_duty());
        printf("pwm4->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm4.pwm_get_gpio(), pwm4.pwm_get_mux(), pwm4.pwm_get_channel(), pwm4.pwm_get_period(), pwm4.pwm_get_duty());
        sleep(1);
        pwm1.pwm_set_duty(7000);
        pwm2.pwm_set_duty(7000);
        pwm3.pwm_set_duty(7000);
        pwm4.pwm_set_duty(7000);
        printf("pwm1->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm1.pwm_get_gpio(), pwm1.pwm_get_mux(), pwm1.pwm_get_channel(), pwm1.pwm_get_period(), pwm1.pwm_get_duty());
        printf("pwm2->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm2.pwm_get_gpio(), pwm2.pwm_get_mux(), pwm2.pwm_get_channel(), pwm2.pwm_get_period(), pwm2.pwm_get_duty());
        printf("pwm3->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm3.pwm_get_gpio(), pwm3.pwm_get_mux(), pwm3.pwm_get_channel(), pwm3.pwm_get_period(), pwm3.pwm_get_duty());
        printf("pwm4->gpio%d, mux=%d, ch=%d, period=%d, duty=%d\n",
            pwm4.pwm_get_gpio(), pwm4.pwm_get_mux(), pwm4.pwm_get_channel(), pwm4.pwm_get_period(), pwm4.pwm_get_duty());
        sleep(1);
    }
}
