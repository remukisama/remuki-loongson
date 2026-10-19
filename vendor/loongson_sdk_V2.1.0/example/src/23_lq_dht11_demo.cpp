#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_dht11_demo.cpp
 * @brief   DHT11温湿度传感器 demo.
 * @author  龙邱科技-012
 * @date    2026-04-21
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 DHT11 温湿度传感器功能.
 ********************************************************************************/

/********************************************************************************
 * @brief   DHT11温湿度传感器 demo.
 * @param   none.
 * @return  none.
 * @note    本 demo 实现 DHT11 温湿度传感器功能.
 ********************************************************************************/
void lq_dht11_demo(void)
{
    // 初始化 DHT11 温湿度传感器，数据引脚连接到 PIN_72 (可任意修改)
    lq_dht11 dht11(PIN_72);

    while (ls_system_running.load())
    {
        // 打印数据前要先调用 read_data() 获取最新数据，否则 get_temperature() 和 get_humidity() 可能返回过期数据
        dht11.read_data();
        printf("tem : %5.2f\n"  , dht11.get_temperature());
        printf("hum : %5.2f\n\n", dht11.get_humidity());
        usleep(100 * 1000);
    }
}
