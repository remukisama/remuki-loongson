#ifndef __LQ_I2C_ICM42688_HPP
#define __LQ_I2C_ICM42688_HPP 

#include "lq_i2c_dev.hpp"

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

#define ICM42688_DEV_NAME        ( "/dev/lq_i2c_icm42688" )

/* ICM42688 相关幻数号, 请勿修改 */
#define I2C_ICM42688_MAGIC      ( 'i' )                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_ICM42688_ID     ( _IO(I2C_ICM42688_MAGIC, 21) ) // 获取 ICM42688 ID
#define I2C_GET_ICM42688_TEM    ( _IO(I2C_ICM42688_MAGIC, 22) ) // 获取 ICM42688 温度
#define I2C_GET_ICM42688_ANG    ( _IO(I2C_ICM42688_MAGIC, 23) ) // 获取 ICM42688 加速度值
#define I2C_GET_ICM42688_ACC    ( _IO(I2C_ICM42688_MAGIC, 24) ) // 获取 ICM42688 陀螺仪值
#define I2C_GET_ICM42688_GYRO   ( _IO(I2C_ICM42688_MAGIC, 25) ) // 获取 ICM42688 加速度、陀螺仪、温度值

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/*!
 * @brief    ICM42688 设备驱动类
 */
class lq_i2c_icm42688 : public lq_i2c_devs
{
public:
    lq_i2c_icm42688(const std::string _dev_path = ICM42688_DEV_NAME); // 有参构造函数
    lq_i2c_icm42688(const lq_i2c_icm42688&) = delete;                 // 禁用拷贝构造函数
    lq_i2c_icm42688& operator=(const lq_i2c_icm42688&) = delete;      // 禁用赋值运算符
    ~lq_i2c_icm42688();                                              // 析构函数

public:
    uint8_t get_icm42688_id();      // 获取 ICM42688 ID
    float   get_icm42688_tem();     // 获取 ICM42688 温度

    bool    get_icm42688_ang(int16_t *gx, int16_t *gy, int16_t *gz);    // 获取 ICM42688 角速度值
    bool    get_icm42688_acc(int16_t *ax, int16_t *ay, int16_t *az);    // 获取 ICM42688 加速度值

    // 获取 ICM42688 角速度值
    bool    get_icm42688_gyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
};

#endif
