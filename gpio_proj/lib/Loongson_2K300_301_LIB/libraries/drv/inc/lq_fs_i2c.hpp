#ifndef __LQ_FS_I2C_HPP
#define __LQ_FS_I2C_HPP

#include "lq_common.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

#define LS_I2C_BUS0         "/dev/i2c-4"    // 对应SCL=PIN_48, SDA=PIN_49
#define LS_I2C_BUS1         "/dev/i2c-5"    // 对应SCL=PIN_50, SDA=PIN_51 (母板引出的为该总线)
#define LS_I2C_BUS_MAX      ( 2 )           // I2C 总线最大数量

/****************************************************************************************************
 * @brief   常量定义
 ****************************************************************************************************/

namespace ls_fs_i2c_constants
{
    constexpr uint8_t   MIN_SLAVE_ADDR = 0x08;  // I2C 从设备地址最小值
    constexpr uint8_t   MAX_SLAVE_ADDR = 0x77;  // I2C 从设备地址最大值
    constexpr int       I2C_INVALID_FD = -1;    // I2C 无效文件描述符
    constexpr size_t    I2C_WRITE_MAX  = 256;   // I2C 写入最大字节数
}

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/* 该类通过调用设备文件控制底层I2C控制器, 一个控制器可以控制多个从设备, 也就是说同一个控制器可以初始化多个不同从地址的变量 */
class ls_fs_i2c
{
public:
    ls_fs_i2c(std::string _bus, uint8_t _addr);     // 构造函数，用于初始化 ls_fs_i2c 对象
    ~ls_fs_i2c() noexcept;                          // 析构函数，用于释放 ls_fs_i2c 对象占用的资源

public:
    ls_fs_i2c(const ls_fs_i2c&) = delete;               // 禁用拷贝构造函数
    ls_fs_i2c(ls_fs_i2c&&)      = delete;               // 禁用移动构造函数
    ls_fs_i2c& operator=(const ls_fs_i2c&) = delete;    // 禁用赋值运算符
    ls_fs_i2c& operator=(ls_fs_i2c&&)      = delete;    // 禁用移动赋值运算符

public:
    ssize_t i2c_reg_read_byte (uint8_t _reg, uint8_t *_byte);   // 读取寄存器单字节数据
    ssize_t i2c_reg_write_byte(uint8_t _reg, uint8_t  _byte);   // 写入寄存器单字节数据

    ssize_t i2c_reg_read_bytes (uint8_t _reg, uint8_t *_buf, size_t _len);  // 读取寄存器多字节数据
    ssize_t i2c_reg_write_bytes(uint8_t _reg, uint8_t *_buf, size_t _len);  // 写入寄存器多字节数据

    ssize_t i2c_read_byte (uint8_t *_byte);                     // 读取单字节数据
    ssize_t i2c_write_byte(uint8_t  _byte);                     // 写入单字节数据

    ssize_t i2c_read_bytes (uint8_t *_buf, size_t _len);        // 读取多字节数据
    ssize_t i2c_write_bytes(uint8_t *_buf, size_t _len);        // 写入多字节数据

public:
    std::string get_i2c_bus_path()   const  { return this->bus; }      // 获取I2C总线路径
    uint8_t     get_i2c_bus_idx()    const  { return this->bus_idx; }  // 获取I2C总线ID
    uint8_t     get_i2c_slave_addr() const  { return this->s_addr; }   // 获取I2C从设备地址
    int         get_i2c_fd()         const  { return this->fd; }       // 获取I2C文件描述符

private:
    bool set_slave_addr(uint8_t _addr);    // 设置从设备地址

private:
    std::string         bus;        // I2C 总线路径
    uint8_t             bus_idx;    // I2C 总线ID
    uint8_t             s_addr;     // 从设备地址
    int                 fd;         // 文件描述符
};

#endif
