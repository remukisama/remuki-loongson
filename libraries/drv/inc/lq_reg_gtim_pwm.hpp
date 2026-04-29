#ifndef __LQ_REG_GTIM_PWM_HPP
#define __LQ_REG_GTIM_PWM_HPP

#include <iostream>
#include <pthread.h>
#include "lq_reg_gpio.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

/********************************** 寄存器偏移 *************************************/

#define LS_GTIM_CR1             ( 0x00 )    // 控制寄存器1
#define LS_GTIM_EGR             ( 0x14 )    // 事件产生寄存器
#define LS_GTIM_CCMR1           ( 0x18 )    // 捕获/比较模式寄存器1
#define LS_GTIM_CCMR2           ( 0x1C )    // 捕获/比较模式寄存器2
#define LS_GTIM_CCER            ( 0x20 )    // 捕获/比较使能寄存器
#define LS_GTIM_CNT             ( 0x24 )    // 计数器
#define LS_GTIM_ARR             ( 0x2C )    // 自动重装载寄存器
#define LS_GTIM_CCR1            ( 0x34 )    // 捕获/比较寄存器1

#define LS_GTIM_CCRx_OFS        ( 0x04 )    // 捕获/比较寄存器偏移量

/********************************* PWM控制时钟周期 ************************************/

#define GTIM_PWM_CLK_FRE        ( 160000000L )  // GTIM 控制器时钟周期

/********************************* 占空比最大设置值 ***********************************/

#define GTIM_PWM_DUTY_MAX       ( 10000 )

/****************************************************************************************************
 * @brief   枚举值定义
 ****************************************************************************************************/

/* GTIM PWM 极性, 请勿修改 */
typedef enum gtim_pwm_polarity
{
    GTIM_PWM_POL_NORMAL = 0x00,  // 正常极性
    GTIM_PWM_POL_INV,            // 反向极性
    GTIM_PWM_POL_INVALID         // 无效极性
} gtim_pwm_polarity_t;

/* GTIM PWM 模式, 请勿修改 */
typedef enum gtim_pwm_mode
{
    GTIM_PWM_MODE_1 = 0x06, // PWM 模式 1
    GTIM_PWM_MODE_2 = 0x07, // PWM 模式 2
    GTIM_PWM_MODE_INVALID,  // 无效模式
} gtim_pwm_mode_t;

/* GTIM PWM 通道, 请勿修改 */
typedef enum gtim_pwm_channel
{
    GTIM_PWM_CH1 = 0x00,    // 通道1
    GTIM_PWM_CH2,           // 通道2
    GTIM_PWM_CH3,           // 通道3
    GTIM_PWM_CH4,           // 通道4
    GTIM_PWM_CH_INVALID     // 无效通道
} gtim_pwm_channel_t;

/* GTIM PWM 可选引脚, 请勿修改 */
typedef enum gtim_pwm_pin
{
    GTIM_PWM0_PIN34 = (GPIO_MUX_ALT2<<10)|(GTIM_PWM_CH1<<8)|PIN_34,
    GTIM_PWM1_PIN35 = (GPIO_MUX_ALT2<<10)|(GTIM_PWM_CH2<<8)|PIN_35,
    GTIM_PWM2_PIN36 = (GPIO_MUX_ALT2<<10)|(GTIM_PWM_CH3<<8)|PIN_36,
    GTIM_PWM3_PIN77 = (GPIO_MUX_ALT1<<10)|(GTIM_PWM_CH4<<8)|PIN_77,

    GTIM_PWM0_PIN87 = (GPIO_MUX_MAIN<<10)|(GTIM_PWM_CH1<<8)|PIN_87,
    GTIM_PWM1_PIN88 = (GPIO_MUX_MAIN<<10)|(GTIM_PWM_CH2<<8)|PIN_88,
    GTIM_PWM2_PIN89 = (GPIO_MUX_MAIN<<10)|(GTIM_PWM_CH3<<8)|PIN_89,
    // GTIM_PWM3_PIN102= (GPIO_MUX_ALT1<<10)|(GTIM_PWM_CH4<<8)|PIN_102, // 该引脚未引出
} gtim_pwm_pin_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_gtim_pwm
{
public:
    ls_gtim_pwm();   // 空构造函数
    ~ls_gtim_pwm();  // 析构函数
    // 有参构造函数
    ls_gtim_pwm(gtim_pwm_pin_t _pin, uint32_t period, uint32_t duty, gtim_pwm_polarity_t _pola = GTIM_PWM_POL_INV);

public:
    void gtim_pwm_enable(void);     // 使能 GTIM PWM 通道
    void gtim_pwm_disable(void);    // 失能 GTIM PWM 通道

    void gtim_pwm_set_period(uint32_t _period);             // 设置 GTIM PWM 周期
    void gtim_pwm_set_duty(uint32_t _duty);                 // 设置 GTIM PWM 占空比度
    void gtim_pwm_set_polarity(gtim_pwm_polarity_t _pola);  // 设置 GTIM PWM 极性
    void gtim_pwm_set_mode(gtim_pwm_mode_t _mode);          // 设置 GTIM PWM 模式

    ls_gtim_pwm(const ls_gtim_pwm& other);              // 拷贝构造
    ls_gtim_pwm& operator=(const ls_gtim_pwm& other);   // 拷贝赋值  

public:
    gpio_pin_t          gtim_pwm_get_gpio(void);        // 获取当前 GTIM PWM 所使用引脚
    gpio_mux_mode_t     gtim_pwm_get_mux(void);         // 获取当前 GTIM PWM 所使用引脚的复用模式
    gtim_pwm_channel_t  gtim_pwm_get_channel(void);     // 获取当前 GTIM PWM 所使用通道
    gtim_pwm_polarity_t gtim_pwm_get_polarity(void);    // 获取当前 GTIM PWM 极性
    uint32_t            gtim_pwm_get_period(void);      // 获取当前 GTIM PWM 周期
    uint32_t            gtim_pwm_get_duty(void);        // 获取当前 GTIM PWM 占空比度

private:
    gpio_pin_t           gpio;      // GPIO 引脚
    gpio_mux_mode_t      mux;       // GPIO 复用模式
    gtim_pwm_channel_t   ch;        // PWM 通道
    gtim_pwm_polarity_t  pola;      // PWM 极性
    gtim_pwm_mode_t      mode;      // PWM 模式
    uint32_t             period;    // PWM 周期
    uint32_t             duty;      // PWM 占空比度

    mutable std::mutex   mtx;       // 其他操作互斥锁
    mutable std::mutex   enable_mtx;// PWM 使能互斥锁
    std::shared_ptr<int> ref_count; // 引用计数互斥锁

private:
    // GTIM 控制器基地址
    static const ls_reg_base_t LS_GTIM_BASE_ADDR = 0x16119000;

    ls_reg32_addr_t gtim_base;      // GTIM 控制器基地址
    ls_reg32_addr_t gtim_arr;       // 自动重装载寄存器（周期）
    ls_reg32_addr_t gtim_ccrx;      // 捕获/比较寄存器1
    ls_reg32_addr_t gtim_ccmr[2];   // 模式寄存器（0--输入; 1--输出）
    ls_reg32_addr_t gtim_ccer;      // 使能寄存器
    ls_reg32_addr_t gtim_cnt;       // 计数器寄存器
    ls_reg32_addr_t gtim_egr;       // 事件产生寄存器
    ls_reg32_addr_t gtim_cr1;       // 控制寄存器
};

#endif
