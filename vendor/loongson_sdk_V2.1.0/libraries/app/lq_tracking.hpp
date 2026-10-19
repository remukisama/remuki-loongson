#ifndef __LQ_TRACKING_HPP__
#define __LQ_TRACKING_HPP__

#include "lq_drv_inc.hpp"

/********************************************************************************
 * @brief    8路灰度循迹传感器驱动类，3根IO译码选通+单ADC采集
 * @param    none
 * @return   none
 * @note     3路GPIO组合译码可输出8种状态，分时选通8路灰度电阻，共用一路ADC采样
 *           因龙芯板卡上 ADC 资源有限, 所以暂时只支持单通道采样模式
 ********************************************************************************/
class lq_tracking
{
public:
    // 循迹模块构造函数，初始化ADC与3路控制GPIO
    lq_tracking(ls_adc_channel_t adc_ch, gpio_pin_t s1, gpio_pin_t s2, gpio_pin_t s3);
    // 循迹模块析构函数
    ~lq_tracking();

public:
    void     get_polling_value();       // 轮询读取 8 路灰度传感器值

    uint16_t get_value(uint8_t index);  // 获取指定索引的传感器值，index范围0-7

private:
    void set_io(uint8_t v1, uint8_t v2, uint8_t v3); // 设置控制引脚状态，v1~v3为0或1

private:
    ls_adc   adc;               // ADC实例，关联通道

    ls_gpio  ctrl_1;            // 控制引脚 1
    ls_gpio  ctrl_2;            // 控制引脚 2
    ls_gpio  ctrl_3;            // 控制引脚 3

    std::atomic<uint16_t> value_buffer[8];   // 存储 8 路传感器的采样值
};

#endif
