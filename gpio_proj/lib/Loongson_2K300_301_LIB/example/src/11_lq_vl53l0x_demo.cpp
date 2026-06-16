#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_vl53l0x_demo.cpp
 * @brief   VL53L0X 测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 VL53L0X 功能，用于测试 VL53L0X 控制器的基本功能.
 ********************************************************************************/

/********************************************************************************
 *  @brief   VL53L0X 设备驱动测试.
 *  @param   none.
 *  @return  none.
 *  @note    VL53L0X 设备驱动测试.
 *! @note    使用前需要加载 driver 目录下的 VL53L0X 设备驱动模块和总的lq_i2c_all_dev.ko设备端模块.
 *! @note    加载模块前请务必将 VL53L0X 传感器连接到 I2C 总线上，否则模块会加载失败.
 ********************************************************************************/
void lq_vl53l0x_demo(void)
{
    uint16_t dis;

    lq_i2c_vl53l0x vl53l0x;

    while (ls_system_running.load())
    {
        printf("VL53L0X distance = %05u\n\n", vl53l0x.get_vl53l0x_dis());
        usleep(100*100);
    }
}
