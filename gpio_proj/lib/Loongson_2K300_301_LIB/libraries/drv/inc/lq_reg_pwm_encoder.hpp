#ifndef __LQ_REG_PWM_ENCODER_HPP
#define __LQ_REG_PWM_ENCODER_HPP

#include <iostream>
#include <pthread.h>
#include "lq_reg_gpio.hpp"
#include "lq_clock.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

/*********************************** 编码器线数 **************************************/

#define LQ_NUM_ENCODER_LINE             ( 512 )

/*********************************** 寄存器地址 **************************************/

#define LS_ENC_PWM_OFFSET               ( 0x10 )        // pwm控制器偏移量
#define LS_ENC_PWM_LOW_BUF_OFFSET       ( 0x04 )        // 低脉冲缓冲寄存器
#define LS_ENC_PWM_FULL_BUF_OFFSET      ( 0x08 )        // 脉冲周期缓冲寄存器
#define LS_ENC_PWM_CTRL_OFFSET          ( 0x0C )        // 控制寄存器

/********************************控制寄存器各位域***********************************/

#define LS_ENC_PWM_CTRL_EN              BIT(0)          // 计数器使能   （1--计数；0--停止）
#define LS_ENC_PWM_CTRL_INTE            BIT(5)          // 中断使能     （1--使能；0--失能）
#define LS_ENC_PWM_CTRL_RST             BIT(7)          // 计数器重置   （1--重置；0--正常工作）
#define LS_ENC_PWM_CTRL_CAPTE           BIT(8)          // 测量脉冲使能 （1--测量脉冲；0--脉冲输出）

#define LS_ENC_PWM_CLK                  ( LS_PMON_CLOCK_FREQ )  // PWM 控制器时钟周期

/****************************************************************************************************
 * @brief   枚举值定义
 ****************************************************************************************************/

/* 编码器 PWM 通道, 请勿修改 */
typedef enum enc_pwm_channel
{
    ENC_PWM_CH0 = 0x00, // 通道0
    ENC_PWM_CH1,        // 通道1
    ENC_PWM_CH2,        // 通道2
    ENC_PWM_CH3,        // 通道3
    ENC_PWM_CH_INVALID  // 无效通道
} enc_pwm_channel_t;

/* 编码器 PWM 可选引脚, 请勿修改 */
typedef enum ls_enc_pwm_pin
{
    ENC_PWM0_PIN64 = (GPIO_MUX_ALT1<<10)|(ENC_PWM_CH0<<8)|PIN_64,
    ENC_PWM1_PIN65 = (GPIO_MUX_ALT1<<10)|(ENC_PWM_CH1<<8)|PIN_65,
    ENC_PWM2_PIN66 = (GPIO_MUX_ALT1<<10)|(ENC_PWM_CH2<<8)|PIN_66,
    ENC_PWM3_PIN67 = (GPIO_MUX_ALT1<<10)|(ENC_PWM_CH3<<8)|PIN_67,

    ENC_PWM0_PIN86 = (GPIO_MUX_ALT2<<10)|(ENC_PWM_CH0<<8)|PIN_86,
    // ENC_PWM1_PIN87 = (GPIO_MUX_ALT2<<10)|(ENC_PWM_CH1<<8)|PIN_87,    // 暂不可用
    // ENC_PWM2_PIN88 = (GPIO_MUX_ALT2<<10)|(ENC_PWM_CH2<<8)|PIN_88,    // 暂不可用
    // ENC_PWM3_PIN89 = (GPIO_MUX_ALT2<<10)|(ENC_PWM_CH3<<8)|PIN_89,    // 暂不可用
} ls_enc_pwm_pin_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_encoder_pwm
{
public:
    ls_encoder_pwm();       // 构造函数
    ~ls_encoder_pwm();      // 析构函数
    // 有参构造函数
    ls_encoder_pwm(ls_enc_pwm_pin_t _pin, gpio_pin_t _dir);

public:
    float encoder_get_count(void);              // 获取编码器值

    void  encoder_reset_counter(void);          // 重置编码器计数器
    void  encoder_close_reset_counter(void);    // 关闭重置编码器计数器

    ls_encoder_pwm(const ls_encoder_pwm& other) = delete;            // 拷贝构造
    ls_encoder_pwm& operator=(const ls_encoder_pwm& other) = delete; // 拷贝赋值
    ls_encoder_pwm(ls_encoder_pwm&& other) = delete;                 // 移动构造
    ls_encoder_pwm& operator=(ls_encoder_pwm&& other) = delete;      // 移动赋值

public:
    gpio_pin_t        encoder_get_pulse(void);      // 获取脉冲引脚
    gpio_pin_t        encoder_get_dir(void);        // 获取方向引脚
    gpio_mux_mode_t   encoder_get_mux(void);        // 获取脉冲引脚复用模式
    enc_pwm_channel_t encoder_get_channel(void);    // 获取编码器 PWM 通道

private:
    gpio_pin_t           gpio;      // 脉冲引脚
    ls_gpio              dir;       // 方向引脚
    gpio_mux_mode_t      mux;       // 脉冲引脚复用模式
    enc_pwm_channel_t    ch;        // 编码器 PWM 通道

    mutable std::mutex   mtx;       // 互斥锁

private:
    // PWM 控制器基地址
    static const ls_reg_base_t LS_ENC_PWM_BASE_ADDR = 0x1611B000;   

    ls_reg32_addr_t enc_pwm_base;       // PWM 基地址
    ls_reg32_addr_t enc_pwm_low_buf;    // 低脉冲缓冲寄存器
    ls_reg32_addr_t enc_pwm_full_buf;   // 脉冲周期缓冲寄存器
    ls_reg32_addr_t enc_pwm_ctrl;       // 控制寄存器
};

#endif
