#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_gtim_pwm_demo.cpp
 * @brief   GTIM PWM 输出模式测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 GTIM PWM 输出功能，用于测试 PWM 引脚的基本功能.
 *!         注意：该例程进攻测试 PWM，因给定占空比过大，请勿连接电机等设备，否则可能会损坏设备.
 ********************************************************************************/

/********************************************************************************
 * @brief   GTIM PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    初始构造方法有两种, 多文件赋值使用时也有两种方法.
 ********************************************************************************/
void lq_gtim_pwm_demo(void)
{
    // 默认极性的构造方式
    ls_gtim_pwm pwm1(GTIM_PWM0_PIN87, 100, 2000);
    // 自定义极性的构造方式
    ls_gtim_pwm pwm2(GTIM_PWM1_PIN88, 100, 2000, GTIM_PWM_POL_NORMAL);

    while (ls_system_running.load())
    {
        pwm1.gtim_pwm_set_duty(1000);
        pwm2.gtim_pwm_set_duty(2000);
        sleep(1);
        pwm1.gtim_pwm_set_duty(3000);
        pwm2.gtim_pwm_set_duty(4000);
        sleep(1);
        pwm1.gtim_pwm_set_duty(5000);
        pwm2.gtim_pwm_set_duty(6000);
        sleep(1);
        pwm1.gtim_pwm_set_duty(7000);
        pwm2.gtim_pwm_set_duty(8000);
        sleep(1);
    }
}
