#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_mpu6050_demo.cpp
 * @brief   MPU6050 测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 MPU6050 功能，用于测试 MPU6050 控制器的基本功能.
 ********************************************************************************/

/********************************************************************************
 *  @brief   MPU6050 设备驱动测试.
 *  @param   none.
 *  @return  none.
 *  @note    MPU6050 设备驱动测试.
 *! @note    使用前需要加载 driver 目录下的 MPU6050 设备驱动端模块和总的lq_i2c_all_dev.ko设备端模块.
 *! @note    加载模块前请务必将 MPU6050 传感器连接到 I2C 总线上，否则模块会加载失败.
 ********************************************************************************/
void lq_mpu6050_demo(void)
{
    uint8_t id;
    int16_t ax, ay, az, gx, gy, gz;

    lq_i2c_mpu6050 mpu6050;

    while (ls_system_running.load())
    {
        mpu6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz);
        printf("ID = 0x%02x, ax=%05d, ay=%05d, az=%05d, gx=%05d, gy=%05d, gz=%05d\n\n", mpu6050.get_mpu6050_id(), ax, ay, az, gx, gy, gz);
        usleep(100*100);
    }
}
