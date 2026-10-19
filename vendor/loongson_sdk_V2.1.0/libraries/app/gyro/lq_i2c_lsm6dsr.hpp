#ifndef __LQ_I2C_LSM6DSR_HPP
#define __LQ_I2C_LSM6DSR_HPP 

#include "lq_i2c_dev.hpp"

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

#define LSM6DSR_DEV_NAME        ( "/dev/lq_i2c_lsm6dsr" )

// MPU6050 相关幻数号
#define I2C_LSM6DSR_MAGIC       'i'                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_LSM6DSR_ID      _IO(I2C_LSM6DSR_MAGIC, 11)  // 获取 LSM6DSR ID
#define I2C_GET_LSM6DSR_GYRO    _IO(I2C_LSM6DSR_MAGIC, 12)  // 获取 LSM6DSR 角度和加速度值

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/*!
 * @brief    LSM6DSR 设备驱动类
 */
class lq_i2c_lsm6dsr : public lq_i2c_devs
{
public:
    lq_i2c_lsm6dsr(const std::string _dev_path = LSM6DSR_DEV_NAME); // 有参构造函数
    lq_i2c_lsm6dsr(const lq_i2c_lsm6dsr&) = delete;                 // 禁用拷贝构造函数
    lq_i2c_lsm6dsr& operator=(const lq_i2c_lsm6dsr&) = delete;      // 禁用赋值运算符
    ~lq_i2c_lsm6dsr();                                              // 析构函数

public:
    uint8_t get_lsm6dsr_id();      // 获取 MPU6050 ID

    // 获取 LSM6DSR 角速度和加速度值
    bool    get_lsm6dsr_gyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
};

#endif
