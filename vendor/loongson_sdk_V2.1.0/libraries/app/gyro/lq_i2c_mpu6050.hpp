#ifndef __LQ_I2C_MPU6050_HPP
#define __LQ_I2C_MPU6050_HPP 

#include "lq_i2c_dev.hpp"

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

#define MPU6050_DEV_NAME        ( "/dev/lq_i2c_mpu6050" )

// MPU6050 相关幻数号
#define I2C_MPU6050_MAGIC       'i'                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_MPU6050_ID      _IO(I2C_MPU6050_MAGIC, 1)   // 获取 MPU6050 ID
#define I2C_GET_MPU6050_TEM     _IO(I2C_MPU6050_MAGIC, 2)   // 获取 MPU6050 温度
#define I2C_GET_MPU6050_ANG     _IO(I2C_MPU6050_MAGIC, 3)   // 获取 MPU6050 角度值
#define I2C_GET_MPU6050_ACC     _IO(I2C_MPU6050_MAGIC, 4)   // 获取 MPU6050 加速度
#define I2C_GET_MPU6050_GYRO    _IO(I2C_MPU6050_MAGIC, 5)   // 获取 MPU6050 角度和加速度值

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/*!
 * @brief    MPU6050 设备驱动类
 */
class lq_i2c_mpu6050 : public lq_i2c_devs
{
public:
    lq_i2c_mpu6050(const std::string _dev_path = MPU6050_DEV_NAME); // 有参构造函数
    lq_i2c_mpu6050(const lq_i2c_mpu6050&) = delete;                 // 禁用拷贝构造函数
    lq_i2c_mpu6050& operator=(const lq_i2c_mpu6050&) = delete;      // 禁用赋值运算符
    ~lq_i2c_mpu6050();                                              // 析构函数

public:
    uint8_t get_mpu6050_id();      // 获取 MPU6050 ID
    float   get_mpu6050_tem();     // 获取 MPU6050 温度

    bool    get_mpu6050_ang(int16_t *gx, int16_t *gy, int16_t *gz);    // 获取 MPU6050 角速度值
    bool    get_mpu6050_acc(int16_t *ax, int16_t *ay, int16_t *az);    // 获取 MPU6050 加速度值

    // 获取 MPU6050 角速度和加速度值
    bool    get_mpu6050_gyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
};

#endif
