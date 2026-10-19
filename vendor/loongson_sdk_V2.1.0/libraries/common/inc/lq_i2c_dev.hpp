#ifndef __LQ_I2C_DEV_HPP
#define __LQ_I2C_DEV_HPP

#include <string>
#include <mutex>
#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/*!
 * @brief   该类用于操作 I2C 设备文件, 封装了 I2C 设备文件的打开、关闭等操作.
 */

class lq_i2c_devs
{
public:
    lq_i2c_devs();                                      // I2C 设备文件使用无参构造函数
    lq_i2c_devs(const std::string _dev_path);           // I2C 设备文件使用有参构造函数
    
    lq_i2c_devs(const lq_i2c_devs& _other);             // 拷贝构造函数
    lq_i2c_devs& operator=(const lq_i2c_devs& _other);  // 赋值构造函数

    ~lq_i2c_devs();                                     // 析构函数

public:
    bool i2c_dev_open(const std::string _dev_path);     // 初始化 I2C 设备文件
    void i2c_dev_close();                               // 关闭 I2C 设备文件

protected:
    int         fd_;        // 文件描述符
    std::string dev_path_;  // 设备路径
    std::mutex  mtx_;       // 保护互斥锁
};

#endif
