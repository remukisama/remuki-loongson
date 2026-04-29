#ifndef __LQ_REG_ATIM_PWM_HPP
#define __LQ_REG_ATIM_PWM_HPP

#include <iostream>
#include <pthread.h>
#include "lq_reg_gpio.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

/********************************** 寄存器偏移 *************************************/

#define LS_ATIM_CR1                 ( 0x00 )        // 控制寄存器1
#define LS_ATIM_EGR                 ( 0x14 )        // 事件产生寄存器
#define LS_ATIM_CCMR1               ( 0x18 )        // 捕获/比较模式寄存器1
#define LS_ATIM_CCMR2               ( 0x1C )        // 捕获/比较模式寄存器2
#define LS_ATIM_CCER                ( 0x20 )        // 捕获/比较使能寄存器
#define LS_ATIM_CNT                 ( 0x24 )        // 计数器
#define LS_ATIM_ARR                 ( 0x2C )        // 自动重装载寄存器
#define LS_ATIM_CCR1                ( 0x34 )        // 捕获/比较寄存器1
#define LS_ATIM_BDTR                ( 0x44 )        // 刹车和死区寄存器（ATIM特有）

#define LS_ATIM_CCRx_OFS            ( 0x04 )        // 捕获/比较寄存器偏移量

/********************************* PWM控制时钟周期 ************************************/

#define ATIM_PWM_CLK_FRE        ( 160000000L )  // ATIM 控制器时钟周期

/********************************* 占空比最大设置值 ***********************************/

#define ATIM_PWM_DUTY_MAX       ( 10000 )

/****************************************************************************************************
 * @brief   枚举值定义
 ****************************************************************************************************/

/* ATIM PWM 极性, 请勿修改 */
typedef enum atim_pwm_polarity
{
    ATIM_PWM_POL_NORMAL = 0x00,  // 正常极性
    ATIM_PWM_POL_INV,            // 反向极性
    ATIM_PWM_POL_INVALID         // 无效极性
} atim_pwm_polarity_t;

/* ATIM PWM 模式, 请勿修改 */
typedef enum atim_pwm_mode
{
    ATIM_PWM_MODE_1 = 0x06, // PWM 模式 1
    ATIM_PWM_MODE_2 = 0x07, // PWM 模式 2
    ATIM_PWM_MODE_INVALID,  // 无效模式
} atim_pwm_mode_t;

/* ATIM PWM 通道, 请勿修改 */
typedef enum atim_pwm_channel
{
    ATIM_PWM_CH1 = 0x00,    // 通道1
    ATIM_PWM_CH2,           // 通道2
    ATIM_PWM_CH3,           // 通道3
    ATIM_PWM_CH4,           // 通道4
    ATIM_PWM_CH_INVALID     // 无效通道
} atim_pwm_channel_t;

/* ATIM PWM 可选引脚, 请勿修改 */
typedef enum atim_pwm_pin
{
    ATIM_PWM0_PIN28 = (GPIO_MUX_ALT2<<10)|(ATIM_PWM_CH1<<8)|PIN_28,
    ATIM_PWM1_PIN29 = (GPIO_MUX_ALT2<<10)|(ATIM_PWM_CH2<<8)|PIN_29,
    ATIM_PWM2_PIN30 = (GPIO_MUX_ALT2<<10)|(ATIM_PWM_CH3<<8)|PIN_30,
    ATIM_PWM3_PIN76 = (GPIO_MUX_ALT1<<10)|(ATIM_PWM_CH4<<8)|PIN_76,

    ATIM_PWM0_PIN81 = (GPIO_MUX_MAIN<<10)|(ATIM_PWM_CH1<<8)|PIN_81,
    ATIM_PWM1_PIN82 = (GPIO_MUX_MAIN<<10)|(ATIM_PWM_CH2<<8)|PIN_82,
    ATIM_PWM2_PIN83 = (GPIO_MUX_MAIN<<10)|(ATIM_PWM_CH3<<8)|PIN_83,
    // ATIM_PWM3_PIN101= (GPIO_MUX_ALT1<<10)|(ATIM_PWM_CH4<<8)|PIN_101, // 该引脚未引出
} atim_pwm_pin_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_atim_pwm
{
public:
    ls_atim_pwm();   // 空构造函数
    ~ls_atim_pwm();  // 析构函数
    // 有参构造函数
    ls_atim_pwm(atim_pwm_pin_t _pin, uint32_t period, uint32_t duty, atim_pwm_polarity_t _pola = ATIM_PWM_POL_INV);

public:
    void atim_pwm_enable(void);     // 使能 ATIM PWM 通道
    void atim_pwm_disable(void);    // 失能 ATIM PWM 通道

    void atim_pwm_set_period(uint32_t _period);             // 设置 ATIM PWM 周期
    void atim_pwm_set_duty(uint32_t _duty);                 // 设置 ATIM PWM 占空比度
    void atim_pwm_set_polarity(atim_pwm_polarity_t _pola);  // 设置 ATIM PWM 极性
    void atim_pwm_set_mode(atim_pwm_mode_t _mode);          // 设置 ATIM PWM 模式

    ls_atim_pwm(const ls_atim_pwm& other);              // 拷贝构造
    ls_atim_pwm& operator=(const ls_atim_pwm& other);   // 拷贝赋值  

public:
    gpio_pin_t          atim_pwm_get_gpio(void);        // 获取当前 ATIM PWM 所使用引脚
    gpio_mux_mode_t     atim_pwm_get_mux(void);         // 获取当前 ATIM PWM 所使用引脚的复用模式
    atim_pwm_channel_t  atim_pwm_get_channel(void);     // 获取当前 ATIM PWM 所使用通道
    atim_pwm_polarity_t atim_pwm_get_polarity(void);    // 获取当前 ATIM PWM 极性
    uint32_t            atim_pwm_get_period(void);      // 获取当前 ATIM PWM 周期
    uint32_t            atim_pwm_get_duty(void);        // 获取当前 ATIM PWM 占空比度

private:
    gpio_pin_t           gpio;      // GPIO 引脚
    gpio_mux_mode_t      mux;       // GPIO 复用模式
    atim_pwm_channel_t   ch;        // PWM 通道
    atim_pwm_polarity_t  pola;      // PWM 极性
    atim_pwm_mode_t      mode;      // PWM 模式
    uint32_t             period;    // PWM 周期
    uint32_t             duty;      // PWM 占空比度

    mutable std::mutex   mtx;       // 其他操作互斥锁
    mutable std::mutex   enable_mtx;// PWM 使能互斥锁
    std::shared_ptr<int> ref_count; // 引用计数互斥锁

private:
    // ATIM 控制器基地址
    static const ls_reg_base_t LS_ATIM_BASE_ADDR = 0x16118000;

    ls_reg32_addr_t atim_base;      // ATIM 控制器基地址
    ls_reg32_addr_t atim_arr;       // 自动重装载寄存器（周期）
    ls_reg32_addr_t atim_ccrx;      // 捕获/比较寄存器1
    ls_reg32_addr_t atim_ccmr[2];   // 模式寄存器（0--输入; 1--输出）
    ls_reg32_addr_t atim_ccer;      // 使能寄存器
    ls_reg32_addr_t atim_cnt;       // 计数器寄存器
    ls_reg32_addr_t atim_bdtr;      // 刹车和死区寄存器（ATIM特有）
    ls_reg32_addr_t atim_egr;       // 事件产生寄存器
    ls_reg32_addr_t atim_cr1;       // 控制寄存器
};

#endif
