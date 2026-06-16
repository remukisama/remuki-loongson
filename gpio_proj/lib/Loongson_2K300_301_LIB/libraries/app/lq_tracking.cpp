#include "lq_tracking.hpp"

/********************************************************************************
 * @brief    循迹对象构造，初始化ADC与三路控制GPIO
 * @param    adc_ch：ADC通道
 * @param    s1：控制引脚1
 * @param    s2：控制引脚2
 * @param    s3：控制引脚3
 * @return   none
 * @note     初始化列表完成ADC、GPIO构造，GPIO默认输出模式
 ********************************************************************************/
lq_tracking::lq_tracking(ls_adc_channel_t adc_ch, gpio_pin_t s1, gpio_pin_t s2, gpio_pin_t s3)
    : adc(adc_ch), ctrl_1(s1, GPIO_MODE_OUT), ctrl_2(s2, GPIO_MODE_OUT), ctrl_3(s3, GPIO_MODE_OUT)
{
}

/********************************************************************************
 * @brief    循迹模块析构函数
 * @param    none
 * @return   none
 * @note     成员adc、ctrl引脚自动析构释放资源
 ********************************************************************************/
lq_tracking::~lq_tracking()
{
}

/********************************************************************************
 * @brief    设置三路译码IO电平
 * @param    v1：引脚1输出电平(0/1)
 * @param    v2：引脚2输出电平(0/1)
 * @param    v3：引脚3输出电平(0/1)
 * @return   none
 * @note     入参非0则输出高电平，否则输出低电平，3bit组合选通8路传感器
 ********************************************************************************/
void lq_tracking::set_io(uint8_t v1, uint8_t v2, uint8_t v3)
{
    this->ctrl_1.gpio_level_set(v1 ? GPIO_HIGH : GPIO_LOW);
    this->ctrl_2.gpio_level_set(v2 ? GPIO_HIGH : GPIO_LOW);
    this->ctrl_3.gpio_level_set(v3 ? GPIO_HIGH : GPIO_LOW);
}

/********************************************************************************
 * @brief    一轮全8通道灰度采集+均值滤波
 * @param    none
 * @return   none
 * @note     1、循环0~7生成3位译码电平，调用set_io选通对应通道；
             2、单通道连续采样5次，丢弃前3次不稳定数据，剩余数据求和取平均；
             3、原始ADC换算到0~100量程并限幅，最终结果存入value_buffer对应位置
 ********************************************************************************/
void lq_tracking::get_polling_value()
{
    // 定义变量，存储每次采样得到的原始数据、累加有效的采样数据、最终经过处理后的采样数据
    uint32_t data = 0, sum = 0;
    // 定义变量，对该通道采样的总次数、要抛弃的前几次数据、循环变量
    uint8_t num_samples = 5, discard_samples = 3, i = 0, j = 0;
    for (i = 0; i < 8; i++)
    {
        this->set_io((i>>2)&1, (i>>1)&1, (i>>0)&1);
        for (j = 0; j < num_samples; j++)
        {
            // 采样并缩放范围为 0-100
            data = this->adc.read_raw() * 0.02442;
            // 限幅
            data = data >= 100 ? 100 : (data <= 0 ? 0 : data);
            // 抛弃前几次数据
            if (j >= discard_samples)
                sum += data;
        }
        this->value_buffer[i] = sum / (num_samples - discard_samples);
        sum = 0;
    }
}

/********************************************************************************
 * @brief    读取指定索引通道缓存数据
 * @param    index：通道编号 0~7
 * @return   uint16_t：有效返回缓存灰度值，index>=8越界返回0
 * @note     value_buffer为原子类型，多线程安全读取缓存数据
 ********************************************************************************/
uint16_t lq_tracking::get_value(uint8_t index)
{
    if (index >= 8)
        return 0;
    return this->value_buffer[index];
}
