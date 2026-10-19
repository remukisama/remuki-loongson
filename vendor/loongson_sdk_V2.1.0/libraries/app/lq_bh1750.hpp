#ifndef __LQ_BH1750_HPP__
#define __LQ_BH1750_HPP__

#include "lq_drv_inc.hpp"

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

#define BH1750_I2C_ADDR         ( 0x46 )    // BH1750 I2C 地址

#define BH1750_SCL_H(n)         ( (n).gpio_level_set(GPIO_HIGH) )   // SCL 线拉高
#define BH1750_SCL_L(n)         ( (n).gpio_level_set(GPIO_LOW)  )   // SCL 线拉低

#define BH1750_SDA_H(n)         ( (n).gpio_level_set(GPIO_HIGH) )   // SDA 线拉高
#define BH1750_SDA_L(n)         ( (n).gpio_level_set(GPIO_LOW)  )   // SDA 线拉低

#define BH1750_SDA_READ(n)      ( (n).gpio_level_get() )            // 读取 SDA 线电平

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/******************************************************************************************
 * @brief   BH1750 光照强度传感器类，软件模拟I2C驱动
 * @note    采用IO模拟I2C时序，支持寄存器命令写入、光照数据连续读取
 ******************************************************************************************/
class lq_bh1750
{
public:
    lq_bh1750(gpio_pin_t scl, gpio_pin_t sda, uint8_t addr);           // 构造函数
    ~lq_bh1750();                                               // 析构函数

public:
    uint16_t Illuminance_read(void);  // 读取 BH1750 光照强度数据

private:
    void    init(void);                 // 初始化 BH1750 传感器

    void    start(void);                // 发送起始条件
    void    stop(void);                 // 发送停止条件

    void    send_ack(uint8_t ack);      // 发送 ACK 信号
    uint8_t recv_ack(void);             // 接收 ACK 信号

    void    send_byte(uint8_t data);    // 发送一个字节
    uint8_t recv_byte(void);            // 接收一个字节

    void    single_write(uint8_t cmd);  // 发送单字节命令

private:
    ls_gpio     i2c_scl;    // I2C 时钟线
    ls_gpio     i2c_sda;    // I2C 数据线

    uint8_t     i2c_addr;   // I2C 设备地址

    uint8_t     BUF[8];     // 数据缓冲区
};

#endif
