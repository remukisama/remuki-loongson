#ifndef __LQ_DHT11_HPP__
#define __LQ_DHT11_HPP__

#include "lq_drv_inc.hpp"

/********************************************************************************
 * @brief    DHT11 温湿度传感器驱动类
 * @note     基于单总线协议
 ********************************************************************************/
class lq_dht11
{
public:
    lq_dht11(gpio_pin_t data_pin);  // 构造函数
    ~lq_dht11();                    // 析构函数

public:
    void set_rst();                 // 发送复位信号

    uint16_t read_data(void);       // 读取数据

    float get_temperature(void);    // 获取温度值, 执行该函数前需先调用 read_data() 获取最新数据
    float get_humidity(void);       // 获取湿度值, 执行该函数前需先调用 read_data() 获取最新数据

private:
    void delay_us(uint32_t us);     // 微秒级延时函数
    void delay_ms(uint32_t ms);     // 毫秒级延时函数

private:
    ls_gpio             data_gpio;  // DHT11 数据引脚

    std::atomic<float>  humidity;   // 缓存：解析完成的湿度数据
    std::atomic<float>  temperature;// 缓存：解析完成的温度数据
};

#endif
