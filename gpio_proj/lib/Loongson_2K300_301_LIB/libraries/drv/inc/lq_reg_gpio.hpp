#ifndef __LQ_REG_GPIO_HPP
#define __LQ_REG_GPIO_HPP

#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <mutex>
#include <memory>

#include "lq_common.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

#define LS_GPIO_REUSE_ADDR              ( 0x16000490 )  // GPIO复用配置寄存器基地址
#define LS_GPIO_REUSE_OFS               ( 0x04 )        // 复用配置寄存器偏移量

#define LS_GPIO_OEN_OFF(_pin)           ( 0x00 + (_pin) / 8 * 0x01 )
#define LS_GPIO_O_OFF(_pin)             ( 0x10 + (_pin) / 8 * 0x01 )
#define LS_GPIO_I_OFF(_pin)             ( 0x20 + (_pin) / 8 * 0x01 )

/****************************************************************************************************
 * @brief   枚举定义
 ****************************************************************************************************/

/* gpio 引脚枚举类型 */
typedef enum gpio_pin
{
    PIN_0  = 0x00, PIN_1,  PIN_2,  PIN_3,  PIN_4,   PIN_5,   PIN_6,   PIN_7,   PIN_8,   PIN_9,   PIN_10,  PIN_11,  PIN_12,  PIN_13,  PIN_14,  PIN_15,
    PIN_16 = 0x10, PIN_17, PIN_18, PIN_19, PIN_20,  PIN_21,  PIN_22,  PIN_23,  PIN_24,  PIN_25,  PIN_26,  PIN_27,  PIN_28,  PIN_29,  PIN_30,  PIN_31,
    PIN_32 = 0x20, PIN_33, PIN_34, PIN_35, PIN_36,  PIN_37,  PIN_38,  PIN_39,  PIN_40,  PIN_41,  PIN_42,  PIN_43,  PIN_44,  PIN_45,  PIN_46,  PIN_47,
    PIN_48 = 0x30, PIN_49, PIN_50, PIN_51, PIN_52,  PIN_53,  PIN_54,  PIN_55,  PIN_56,  PIN_57,  PIN_58,  PIN_59,  PIN_60,  PIN_61,  PIN_62,  PIN_63,
    PIN_64 = 0x40, PIN_65, PIN_66, PIN_67, PIN_68,  PIN_69,  PIN_70,  PIN_71,  PIN_72,  PIN_73,  PIN_74,  PIN_75,  PIN_76,  PIN_77,  PIN_78,  PIN_79,
    PIN_80 = 0x50, PIN_81, PIN_82, PIN_83, PIN_84,  PIN_85,  PIN_86,  PIN_87,  PIN_88,  PIN_89,  PIN_90,  PIN_91,  PIN_92,  PIN_93,  PIN_94,  PIN_95,
    PIN_96 = 0x60, PIN_97, PIN_98, PIN_99, PIN_100, PIN_101, PIN_102, PIN_103, PIN_104, PIN_105,
    PIN_INVALID
} gpio_pin_t;

/* gpio 模式枚举类型 */
typedef enum gpio_mode
{
    GPIO_MODE_OUT = 0x00,   /* GPIO 输出模式 */
    GPIO_MODE_IN  = 0x01,   /* GPIO 输入模式 */
    GPIO_MODE_INVALID,      /* GPIO 无效模式 */
} gpio_mode_t;

/* gpio 电平枚举类型 */
typedef enum gpio_level
{
    GPIO_LOW  = 0x00,   /* GPIO 低电平(逻辑 0) */
    GPIO_HIGH = 0x01,   /* GPIO 高电平(逻辑 1) */
    GPIO_LEVEL_INVALID, /* GPIO 无效电平 */
} gpio_level_t;

/* gpio 复用模式枚举类型 */
typedef enum gpio_mux_mode
{
    GPIO_MUX_GPIO    = 0x00,    /* GPIO 复用 */
    GPIO_MUX_ALT1    = 0x01,    /* 第一复用 */
    GPIO_MUX_ALT2    = 0x02,    /* 第二复用 */
    GPIO_MUX_MAIN    = 0x03,    /* 主复用 */
    GPIO_MUX_INVALID,           /* 无效复用模式 */
} gpio_mux_mode_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/************************************ gpio类 ****************************************/
class ls_gpio
{
public:
    ls_gpio();  // 空构造函数
    ~ls_gpio(); // 析构函数

    // 设置引脚和模式的构造函数
    ls_gpio(gpio_pin_t _gpio, gpio_mode_t _mode, gpio_mux_mode_t _mux = GPIO_MUX_GPIO);

public:
    void gpio_direction_set(gpio_mode_t _mode); // 设置 GPIO 输入输出模式
    void gpio_level_set(gpio_level_t value);    // 设置 GPIO 输出值
    
    gpio_level_t gpio_level_get();              // 获取 GPIO 输入值

    ls_gpio(const ls_gpio& other);              // 拷贝构造
    ls_gpio& operator=(const ls_gpio& other);   // 拷贝赋值

public:
    gpio_pin_t      gpio_get_pin();     // 获取 GPIO 引脚号
    gpio_mode_t     gpio_get_mode();    // 获取 GPIO 模式
    gpio_mux_mode_t gpio_get_mux();     // 获取 GPIO 复用模式

private:
    gpio_pin_t           gpio;      // GPIO 引脚号
    gpio_mode_t          mode;      // GPIO 模式
    gpio_mux_mode_t      mux;       // GPIO 复用模式

    mutable std::mutex   mtx;       // 互斥锁
    std::shared_ptr<int> ref_count; // 引用计数

private:
    // GPIO 基地址
    static const ls_reg_base_t LS_GPIO_BASE_ADDR = 0x16104000;

    ls_reg32_addr_t gpio_base;    // 基础寄存器映射地址
    ls_reg32_addr_t gpio_one;     // 方向寄存器映射地址
    ls_reg32_addr_t gpio_o;       // 输出寄存器映射地址
    ls_reg32_addr_t gpio_i;       // 输入寄存器映射地址
};

/****************************************************************************************************
 * @brief   内部调用函数定义
 ****************************************************************************************************/

// 设置复用 GPIO
void gpio_mux_set(gpio_pin_t gpio, gpio_mux_mode_t mux);

#endif
