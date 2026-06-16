#ifndef __LQ_SOFT_I2C_HPP
#define __LQ_SOFT_I2C_HPP

#include "lq_reg_gpio.hpp"

/* 注意 IIC总线规定，IIC空闲时 SCL和SDA都为高电平 最好外部上拉（一定不能下拉） */
/* 模拟 IIC需要注意，IIC地址左移一位 例如MPU6050 模拟就是地址 0xD0 */

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_soft_i2c
{
public:
    ls_soft_i2c();                                                  // 空构造函数
    ls_soft_i2c(gpio_pin_t _scl, gpio_pin_t _sda, uint8_t _addr);   // 有参构造函数
    ~ls_soft_i2c();                                                 // 析构函数

    ls_soft_i2c(const ls_soft_i2c& other);              // 拷贝构造
    ls_soft_i2c& operator=(const ls_soft_i2c& other);   // 拷贝赋值

public:
    uint8_t soft_i2c_read_byte(const uint8_t _reg, uint8_t *_buf);  // 读取单字节数据
    uint8_t soft_i2c_write_byte(const uint8_t _reg, uint8_t _byte); // 写入单字节数据

    uint8_t soft_i2c_read_n_byte(const uint8_t _reg, uint8_t *_buf, uint8_t _len);  // 读取多个字节数据
    uint8_t soft_i2c_write_n_byte(const uint8_t _reg, uint8_t *_buf, uint8_t _len); // 写入多个字节数据

private:
    void soft_i2c_delay_ms(uint16_t _ms);   // 软件 I2C 毫秒级延时
    void soft_i2c_delay_us(uint16_t _us);   // 软件 I2C 微秒级延时

    void    soft_i2c_start(void);                       // 软件 I2C 开始信号
    void    soft_i2c_stop(void);                        // 软件 I2C 停止信号
    uint8_t soft_i2c_wait_ack(void);                    // 软件 I2C 等待 ACK 信号
    void    soft_i2c_ack(void);                         // 软件 I2C ACK 信号
    void    soft_i2c_nack(void);                        // 软件 I2C NACK 信号
    void    soft_i2c_send_byte(uint8_t _byte);          // 软件 I2C 发送字节数据
    uint8_t soft_i2c_read_byte_internal(uint8_t _ack);  // 软件 I2C 读取字节数据

private:
    ls_gpio scl;    // I2C 时钟引脚
    ls_gpio sda;    // I2C 数据引脚
    uint8_t addr;   // 器件地址
};

#endif
