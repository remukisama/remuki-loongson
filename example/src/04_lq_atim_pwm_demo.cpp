#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_atim_pwm_demo.cpp
 * @brief   ATIM PWM 输出模式测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 ATIM PWM 输出功能，用于测试 PWM 引脚的基本功能.
 *!         注意：该例程进攻测试 PWM，因给定占空比过大，请勿连接电机等设备，否则可能会损坏设备.
 ********************************************************************************/

/********************************************************************************
 * @brief   ATIM PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    初始构造方法有两种, 多文件赋值使用时也有两种方法.
 ********************************************************************************/
void lq_atim_pwm_demo(void)
{
    // 默认极性的构造方式
    ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 25000, 5000, ATIM_PWM_POL_INV);
    // // 自定义极性的构造方式
    ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 25000, 5000, ATIM_PWM_POL_INV);
    // ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 10000, 0);
    // // 拷贝构造使用方法, 调用 pwm3 与调用 pwm1 同效
    // ls_atim_pwm pwm3(pwm1);
    // // 拷贝赋值使用方法, 调用 pwm4 与调用 pwm2 同效
    // ls_atim_pwm pwm4 = pwm2;

    ls_gpio pwm1dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio pwm2dir(PIN_22, GPIO_MODE_OUT);

    pwm1.atim_pwm_enable();
    pwm2.atim_pwm_enable();
    // pwm3.atim_pwm_enable();
    // pwm4.atim_pwm_enable();
    // pwm2dir.gpio_level_set(GPIO_HIGH);
    // pwm1.atim_pwm_set_duty(5000);
    // pwm2.atim_pwm_set_duty(7000);
    while (1)
    {
        pwm1dir.gpio_level_set(GPIO_HIGH);//左反
        pwm2dir.gpio_level_set(GPIO_HIGH);//右正
        // sleep(2);
        // pwm1dir.gpio_level_set(GPIO_LOW);//左正
        // pwm2dir.gpio_level_set(GPIO_LOW);
        // sleep(2);
    }
}
