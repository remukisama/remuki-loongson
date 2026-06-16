#ifndef __LQ_I2C_VL53L0X_HPP
#define __LQ_I2C_VL53L0X_HPP

#include "lq_i2c_dev.hpp"

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

 #define VL53L0X_DEV_NAME        ( "/dev/lq_i2c_vl53l0x" )

// VL53L0X 相关幻数号
#define I2C_VL53L0X_MAGIC       'v'                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_VL53L0X_DIS     _IO(I2C_VL53L0X_MAGIC, 1)   // 获取 VL53L0X 距离值

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/*!
 * @brief   VL53L0X 设备驱动类
 */
class lq_i2c_vl53l0x : public lq_i2c_devs
{
public:
    lq_i2c_vl53l0x(const std::string _dev_path = VL53L0X_DEV_NAME); // 有参构造函数
    lq_i2c_vl53l0x(const lq_i2c_vl53l0x&) = delete;                 // 禁用拷贝构造函数
    lq_i2c_vl53l0x& operator=(const lq_i2c_vl53l0x&) = delete;      // 禁用赋值运算符
    ~lq_i2c_vl53l0x();                                              // 析构函数

public:
    uint16_t get_vl53l0x_dis(); // 获取 VL53L0X 距离值
};

#endif
