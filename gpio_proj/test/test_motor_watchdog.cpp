/**
 * @file test_motor_watchdog.cpp
 * @brief 全通道看门狗实测 — 4路电机 + 舵机 + 方向GPIO
 *
 * 测试流程：
 *   A. 全部通道输出，正常心跳
 *   B. 改变占空比，正常心跳
 *   C. 停止心跳 → 看门狗接管全部引脚
 *   D. 恢复心跳 → 全部通道恢复输出
 *   E. 再次停止 → 验证可重复触发
 *   F. 再次恢复
 *
 * 示波器观察点：
 *   GPIO81 — 电机1 PWM (TIM1_CH1)
 *   GPIO82 — 电机2 PWM (TIM1_CH2)
 *   GPIO83 — 电机3 PWM (TIM1_CH3)
 *   GPIO76 — 电机4 PWM (TIM1_CH4)
 *   GPIO88 — 舵机 PWM  (TIM2_CH2)
 *   GPIO21 — 电机1方向
 *   GPIO22 — 电机2方向
 *   GPIO23 — 电机3方向
 *   GPIO24 — 电机4方向
 *
 * 使用方法：
 *   ./gpio_manager &
 *   ./test_motor_watchdog
 */

#include "../src/client_lib.hpp"
#include "../config/board_2k301.hpp"
#include "lq_reg_atim_pwm.hpp"
#include "lq_reg_gtim_pwm.hpp"
#include "lq_reg_gpio.hpp"
#include "lq_signal_handle.hpp"

#include <cstdio>
#include <unistd.h>

static void print_status(int elapsed_ms)
{
    printf("  [%6d ms] safe=%d alive=%d\n",
           elapsed_ms,
           gpio::is_safe_mode() ? 1 : 0,
           gpio::is_heartbeat_alive() ? 1 : 0);
}

static void set_all_motor_active_low(ls_atim_pwm &m1,
                                     ls_atim_pwm &m2,
                                     ls_atim_pwm &m3,
                                     ls_atim_pwm &m4,
                                     uint32_t active_low_duty)
{
    m1.atim_pwm_set_active_low_duty(active_low_duty);
    m2.atim_pwm_set_active_low_duty(active_low_duty);
    m3.atim_pwm_set_active_low_duty(active_low_duty);
    m4.atim_pwm_set_active_low_duty(active_low_duty);
}

int main()
{
    printf("========================================\n");
    printf("  全通道看门狗实测\n");
    printf("  gpio_manager 必须已在后台运行\n");
    printf("========================================\n\n");

    // 连接看门狗
    if (gpio::init() != 0) {
        printf("[FAIL] 无法连接看门狗\n");
        return -1;
    }
    printf("[OK] 已连接看门狗\n");

    // === 初始化硬件（小车线程直接操作）===
    printf("[INIT] 初始化 4路电机 + 1路舵机 + 4路方向GPIO...\n");

    // 4路电机 ATIM PWM (25kHz, NORMAL；DRV8701P IN1=HIGH/IN2=HIGH 停车)
    ls_atim_pwm m1(PIN_MOTOR_PWM_1, PIN_MOTOR_PWM_FREQ, SAFE_MOTOR_DUTY, PIN_MOTOR_PWM_POLARITY);
    ls_atim_pwm m2(PIN_MOTOR_PWM_2, PIN_MOTOR_PWM_FREQ, SAFE_MOTOR_DUTY, PIN_MOTOR_PWM_POLARITY);
    ls_atim_pwm m3(PIN_MOTOR_PWM_3, PIN_MOTOR_PWM_FREQ, SAFE_MOTOR_DUTY, PIN_MOTOR_PWM_POLARITY);
    ls_atim_pwm m4(PIN_MOTOR_PWM_4, PIN_MOTOR_PWM_FREQ, SAFE_MOTOR_DUTY, PIN_MOTOR_PWM_POLARITY);

    // 1路舵机 GTIM PWM (50Hz, 反极性)
    ls_gtim_pwm servo(PIN_SERVO_PWM, PIN_SERVO_PWM_FREQ, 0, PIN_SERVO_PWM_POLARITY);

    // 4路方向 GPIO
    ls_gpio d1(static_cast<gpio_pin_t>(PIN_MOTOR_DIR_1), GPIO_MODE_OUT, GPIO_MUX_GPIO);
    ls_gpio d2(static_cast<gpio_pin_t>(PIN_MOTOR_DIR_2), GPIO_MODE_OUT, GPIO_MUX_GPIO);
    ls_gpio d3(static_cast<gpio_pin_t>(PIN_MOTOR_DIR_3), GPIO_MODE_OUT, GPIO_MUX_GPIO);
    ls_gpio d4(static_cast<gpio_pin_t>(PIN_MOTOR_DIR_4), GPIO_MODE_OUT, GPIO_MUX_GPIO);

    // 初始：全部停止（IN1=HIGH, IN2=HIGH）
    set_all_motor_active_low(m1, m2, m3, m4, SAFE_MOTOR_ACTIVE_LOW_DUTY);
    servo.gtim_pwm_set_duty(0);
    d1.gpio_level_set(GPIO_HIGH); d2.gpio_level_set(GPIO_HIGH);
    d3.gpio_level_set(GPIO_HIGH); d4.gpio_level_set(GPIO_HIGH);

    printf("[OK] 硬件初始化完成\n\n");

    // ==================== 阶段 A ====================
    printf("=== A: DRV8701P 正向功率20%% 舵机30%% IN1=HIGH 心跳正常 (3秒) ===\n");
    set_all_motor_active_low(m1, m2, m3, m4, 2000);
    servo.gtim_pwm_set_duty(3000);
    d1.gpio_level_set(GPIO_HIGH); d2.gpio_level_set(GPIO_HIGH);
    d3.gpio_level_set(GPIO_HIGH); d4.gpio_level_set(GPIO_HIGH);

    for (int i = 0; i < 300; i++) {
        gpio::heartbeat();
        usleep(10000);
        if (i % 100 == 99) print_status((i + 1) * 10);
    }

    // ==================== 阶段 B ====================
    printf("\n=== B: DRV8701P 正向功率50%% 舵机60%% IN1=HIGH 心跳正常 (3秒) ===\n");
    set_all_motor_active_low(m1, m2, m3, m4, 5000);
    servo.gtim_pwm_set_duty(6000);

    for (int i = 0; i < 300; i++) {
        gpio::heartbeat();
        usleep(10000);
        if (i % 100 == 99) print_status(3000 + (i + 1) * 10);
    }

    // ==================== 阶段 C ====================
    printf("\n=== C: 停止心跳！模拟小车线程掉线 (5秒) ===\n");
    printf("  >>> 50ms后看门狗接管：4路电机 IN1=HIGH/IN2=HIGH 刹车 + 舵机归零 <<<\n\n");

    for (int i = 0; i < 500; i++) {
        usleep(10000);
        if (i % 50 == 49) print_status(6000 + (i + 1) * 10);
    }

    // ==================== 阶段 D ====================
    printf("\n=== D: 恢复心跳！小车线程重启 (5秒) ===\n");
    printf("  >>> 全部通道恢复输出 <<<\n\n");

    for (int i = 0; i < 500; i++) {
        gpio::heartbeat();
        if (i == 0) {
            set_all_motor_active_low(m1, m2, m3, m4, 5000);
            servo.gtim_pwm_set_duty(6000);
            d1.gpio_level_set(GPIO_HIGH); d2.gpio_level_set(GPIO_HIGH);
            d3.gpio_level_set(GPIO_HIGH); d4.gpio_level_set(GPIO_HIGH);
        }
        usleep(10000);
        if (i % 100 == 99) print_status(11000 + (i + 1) * 10);
    }

    // ==================== 阶段 E ====================
    printf("\n=== E: 再次停止心跳 (3秒) ===\n");
    for (int i = 0; i < 300; i++) {
        usleep(10000);
        if (i % 100 == 99) print_status(16000 + (i + 1) * 10);
    }

    // ==================== 阶段 F ====================
    printf("\n=== F: 再次恢复心跳 (2秒) ===\n");
    for (int i = 0; i < 200; i++) {
        gpio::heartbeat();
        if (i == 0) {
            set_all_motor_active_low(m1, m2, m3, m4, 3000);
            servo.gtim_pwm_set_duty(4000);
        }
        usleep(10000);
        if (i % 100 == 99) print_status(19000 + (i + 1) * 10);
    }

    // === 清理 ===
    printf("\n=== 测试结束 ===\n");
    set_all_motor_active_low(m1, m2, m3, m4, SAFE_MOTOR_ACTIVE_LOW_DUTY);
    servo.gtim_pwm_set_duty(0);
    d1.gpio_level_set(GPIO_HIGH); d2.gpio_level_set(GPIO_HIGH);
    d3.gpio_level_set(GPIO_HIGH); d4.gpio_level_set(GPIO_HIGH);
    gpio::deinit();

    printf("========================================\n");
    printf("  预期示波器现象：\n");
    printf("    A: IN1=HIGH，IN2低电平约20%% 舵机30%%方波\n");
    printf("    B: IN1=HIGH，IN2低电平约50%% 舵机60%%方波\n");
    printf("    C: 50ms后 全部IN1=HIGH且IN2=HIGH(安全)\n");
    printf("    D: PWM方波恢复\n");
    printf("    E: 再次50ms后 全部IN1=HIGH且IN2=HIGH\n");
    printf("    F: IN1=HIGH，IN2低电平约30%% 舵机40%%方波恢复\n");
    printf("========================================\n");
    return 0;
}
