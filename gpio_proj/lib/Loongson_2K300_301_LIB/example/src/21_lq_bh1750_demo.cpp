#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_bh1750_demo.cpp
 * @brief   BH1750 光照传感器 demo.
 * @author  龙邱科技-012
 * @date    2026-04-21
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 BH1750 光照传感器功能.
 ********************************************************************************/

/********************************************************************************
 * @brief   BH1750 光照传感器 demo.
 * @param   none.
 * @return  none.
 * @note    本 demo 实现 BH1750 光照传感器功能.
 ********************************************************************************/
void lq_bh1750_demo(void)
{
    lq_bh1750 light_sensor(PIN_66, PIN_74, BH1750_I2C_ADDR);

    while (ls_system_running.load())
    {
        printf("Light Intensity: %d lx\n", light_sensor.Illuminance_read());
        sleep(1);
    }
}