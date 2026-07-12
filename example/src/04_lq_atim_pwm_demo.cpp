#include "lq_all_demo.hpp"
#include "car_config.hpp"
#include "gpio_protect_heartbeat.hpp"
#include <signal.h>
#include <unistd.h>

namespace {

volatile sig_atomic_t g_atim_pwm_test_stop = 0;

void atim_pwm_test_signal_handler(int)
{
    // 信号处理函数中只置位；PWM/GPIO 操作放回正常执行流中完成。
    g_atim_pwm_test_stop = 1;
}

} // namespace

/********************************************************************************
 * @file    lq_atim_pwm_demo.cpp
 * @brief   ATIM PWM 输出模式测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 ATIM PWM 输出功能，用于测试 PWM 引脚的基本功能.
 *!         注意：当前项目电机驱动为 DRV8701E PH/EN，PWM 为 EN 高有效。
 ********************************************************************************/

/********************************************************************************
 * @brief   ATIM PWM 输出模式测试.
 * @param   none.
 * @return  none.
 * @note    初始构造方法有两种, 多文件赋值使用时也有两种方法.
 ********************************************************************************/
void lq_atim_pwm_demo(void)
{
    constexpr uint32_t kTestFrequencyHz = car_config::kMotorPwmFrequencyHz;
    constexpr uint32_t kTestDuty = 3000; // EN high duty: 3000=30% high

    // 默认极性的构造方式
    ls_atim_pwm pwm1(ATIM_PWM0_PIN81, kTestFrequencyHz, car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    // // 自定义极性的构造方式
    ls_atim_pwm pwm2(ATIM_PWM1_PIN82, kTestFrequencyHz, car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    // ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 10000, 0);
    // // 拷贝构造使用方法, 调用 pwm3 与调用 pwm1 同效
    // ls_atim_pwm pwm3(pwm1);
    // // 拷贝赋值使用方法, 调用 pwm4 与调用 pwm2 同效
    // ls_atim_pwm pwm4 = pwm2;

    ls_gpio pwm1dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio pwm2dir(PIN_22, GPIO_MODE_OUT);

    // 覆盖 main 中会直接 exit() 的处理函数，使局部 PWM 对象能够正常停机。
    g_atim_pwm_test_stop = 0;
    signal(SIGINT, atim_pwm_test_signal_handler);
    signal(SIGTERM, atim_pwm_test_signal_handler);
    signal(SIGHUP, atim_pwm_test_signal_handler);
    signal(SIGQUIT, atim_pwm_test_signal_handler);

    pwm1dir.gpio_level_set(GPIO_HIGH);
    pwm2dir.gpio_level_set(GPIO_HIGH);
    pwm1.atim_pwm_set_duty(kTestDuty);
    pwm2.atim_pwm_set_duty(kTestDuty);
    pwm1.atim_pwm_enable();
    pwm2.atim_pwm_enable();
    // pwm3.atim_pwm_enable();
    // pwm4.atim_pwm_enable();
    printf("[ATIM_PWM_TEST] CH0(pin81) and CH1(pin82): %u Hz, EN high duty=%u/10000. Press Ctrl+C to stop.\n",
           kTestFrequencyHz, kTestDuty);

    while (!g_atim_pwm_test_stop)
    {
        gpio_protect_heartbeat_pulse();
        usleep(10000);
    }

    // DRV8701E PH/EN: EN=0 is stop/brake.
    pwm1.atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
    pwm2.atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
    pwm1.atim_pwm_enable();
    pwm2.atim_pwm_enable();
    pwm1dir.gpio_level_set(GPIO_HIGH);
    pwm2dir.gpio_level_set(GPIO_HIGH);
    gpio_protect_heartbeat_deinit();

    printf("\n[ATIM_PWM_TEST] Both PWM channels stopped at EN high duty=0 with PWM enabled.\n");
    fflush(stdout);

    // 不返回，避免局部 PWM 对象析构时 disable 通道；停机态已由 EN=0 主动保持。
    _exit(0);
}
