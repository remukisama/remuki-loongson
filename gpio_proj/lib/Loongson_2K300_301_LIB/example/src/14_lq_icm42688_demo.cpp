#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_icm42688_demo.cpp
 * @brief   ICM42688 测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 ICM42688 功能，用于测试 ICM42688 控制器的基本功能.
 ********************************************************************************/

/********************************************************************************
 *  @brief   ICM42688 设备驱动测试.
 *  @param   none.
 *  @return  none.
 *  @note    ICM42688 设备驱动测试.
 *! @note    使用前需要加载 driver 目录下的 ICM42688 设备驱动模块和总的lq_i2c_all_dev.ko设备端模块.
 *! @note    加载模块前请务必将 ICM42688 传感器连接到 I2C 总线上，否则模块会加载失败.
 ********************************************************************************/
void lq_icm42688_demo(void)
{
    uint8_t id;
    int16_t ax, ay, az, gx, gy, gz;

    lq_i2c_icm42688 icm42688;

    while (ls_system_running.load())
    {
        icm42688.get_icm42688_gyro(&ax, &ay, &az, &gx, &gy, &gz);
        printf("ID = 0x%02x, ax=%08d, ay=%08d, az=%08d, gx=%08d, gy=%08d, gz=%08d, tem=%7.2f\n\n",
                icm42688.get_icm42688_id(), ax, ay, az, gx, gy, gz, icm42688.get_icm42688_tem());
        usleep(100*100);
    }
}