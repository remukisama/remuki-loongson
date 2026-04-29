#ifndef __LQ_REG_PWM_HPP
#define __LQ_REG_PWM_HPP

#include <iostream>
#include "lq_reg_gpio.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

/*********************************** 寄存器地址 **************************************/

#define LS_PWM_OFF                  ( 0x10 )        // pwm控制器偏移量
#define LS_PWM_LOW_BUF_OFF          ( 0x04 )        // 低脉冲缓冲寄存器
#define LS_PWM_FULL_BUF_OFF         ( 0x08 )        // 脉冲周期缓冲寄存器
#define LS_PWM_CTRL_OFF             ( 0x0C )        // 控制寄存器

/******************************** 控制寄存器各位域 ************************************/

#define LS_PWM_CTRL_EN              BIT(0)          // 计数器使能       （1--计数；0--停止）
#define LS_PWM_CTRL_OE              BIT(3)          // 脉冲输出使能控制位（1--失能；0--使能）
#define LS_PWM_CTRL_SINGLE          BIT(4)          // 单脉冲控制位     （1--单；0--持续）
#define LS_PWM_CTRL_INTE            BIT(5)          // 中断使能         （1--使能；0--失能）
#define LS_PWM_CTRL_INT             BIT(6)          // 中断位           （读：1--有中断；0--没有. 写：1--清中断）
#define LS_PWM_CTRL_RST             BIT(7)          // 计数器重置       （1--重置；0--正常工作）
#define LS_PWM_CTRL_CAPTE           BIT(8)          // 测量脉冲使能      （1--测量脉冲；0--脉冲输出）
#define LS_PWM_CTRL_INVERT          BIT(9)          // 输出翻转使能      （1--翻转；0--不翻转）
#define LS_PWM_CTRL_DZONE           BIT(10)         // 防死区           （1--开启；0--关闭）

/********************************* PWM控制时钟周期 ************************************/

#define PWM_CLK_FRE                 ( 160000000L )  // PWM 控制器时钟周期

/********************************* 占空比最大设置值 ***********************************/

#define PWM_DUTY_MAX                ( 10000 )

/****************************************************************************************************
 * @brief   枚举值定义
 ****************************************************************************************************/

/* PWM 极性, 请勿修改 */
typedef enum pwm_polarity
{
    PWM_POL_NORMAL = 0x00,  // 正常极性
    PWM_POL_INV,            // 反向极性
    PWM_POL_INVALID         // 无效极性
} pwm_polarity_t;

/* PWM 通道, 请勿修改 */
typedef enum pwm_channel
{
    PWM_CH0 = 0x00,     // 通道0
    PWM_CH1,            // 通道1
    PWM_CH2,            // 通道2
    PWM_CH3,            // 通道3
    PWM_CH_INVALID      // 无效通道
} pwm_channel_t;

/* PWM 可选引脚, 请勿修改 */
typedef enum pwm_pin
{
    PWM0_PIN64 = (GPIO_MUX_ALT1<<10)|(PWM_CH0<<8)|PIN_64,
    PWM1_PIN65 = (GPIO_MUX_ALT1<<10)|(PWM_CH1<<8)|PIN_65,
    PWM2_PIN66 = (GPIO_MUX_ALT1<<10)|(PWM_CH2<<8)|PIN_66,
    PWM3_PIN67 = (GPIO_MUX_ALT1<<10)|(PWM_CH3<<8)|PIN_67,

    PWM0_PIN86 = (GPIO_MUX_ALT2<<10)|(PWM_CH0<<8)|PIN_86,
    PWM1_PIN87 = (GPIO_MUX_ALT2<<10)|(PWM_CH1<<8)|PIN_87,
    PWM2_PIN88 = (GPIO_MUX_ALT2<<10)|(PWM_CH2<<8)|PIN_88,
    PWM3_PIN89 = (GPIO_MUX_ALT2<<10)|(PWM_CH3<<8)|PIN_89,
} pwm_pin_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_pwm
{
public:
    ls_pwm();   // 空构造函数
    ~ls_pwm();  // 析构函数
    // 有参构造函数
    ls_pwm(pwm_pin_t _ch, uint32_t _period, uint32_t _duty, pwm_polarity_t _pola = PWM_POL_INV);
    
public:
    void pwm_enable(void);  // 使能 PWM 通道
    void pwm_disable(void); // 失能 PWM 通道

    void pwm_set_period(uint32_t _period);      // 设置 PWM 周期
    void pwm_set_duty(uint32_t _duty);          // 设置 PWM 占空比度
    void pwm_set_polarity(pwm_polarity_t _pola);// 设置 PWM 极性

    ls_pwm(const ls_pwm& other);            // 拷贝构造
    ls_pwm& operator=(const ls_pwm& other); // 拷贝赋值
    
public:
    gpio_pin_t      pwm_get_gpio(void);     // 获取当前 PWM 所使用引脚
    gpio_mux_mode_t pwm_get_mux(void);      // 获取当前 PWM 所使用引脚的复用模式
    pwm_channel_t   pwm_get_channel(void);  // 获取当前 PWM 所使用通道
    pwm_polarity_t  pwm_get_polarity(void); // 获取当前 PWM 极性
    uint32_t        pwm_get_period(void);   // 获取当前 PWM 周期
    uint32_t        pwm_get_duty(void);     // 获取当前 PWM 占空比度

private:
    gpio_pin_t           gpio;      // GPIO 引脚
    gpio_mux_mode_t      mux;       // GPIO 复用
    pwm_channel_t        ch;        // PWM 通道
    pwm_polarity_t       pola;      // PWM极性
    uint32_t             period;    // PWM 周期
    uint32_t             duty;      // PWM 脉冲宽度

    mutable std::mutex   mtx;       // 其他操作互斥锁
    mutable std::mutex   enable_mtx;// PWM 使能互斥锁
    std::shared_ptr<int> ref_count; // 引用计数
private:
    // pwm 控制器总基地址
    static const ls_reg_base_t LS_PWM_BASE_ADDR = 0x1611B000;

    ls_reg32_addr_t pwm_base;       // PWM 基地址
    ls_reg32_addr_t pwm_low_buf;    // 低脉冲缓冲寄存器
    ls_reg32_addr_t pwm_full_buf;   // 脉冲周期缓冲寄存器
    ls_reg32_addr_t pwm_ctrl;       // 控制寄存器
};

#endif
