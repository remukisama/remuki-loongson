#ifndef __LQ_SOFT_SPI_HPP
#define __LQ_SOFT_SPI_HPP

#include "lq_reg_gpio.hpp"

/****************************************************************************************************
 * @brief   宏值定义
 ****************************************************************************************************/

#define LS_SOFT_SPI_MAX_TRANS_LEN       ( 256 )     // 软件 SPI 一次最大传输长度

/****************************************************************************************************
 * @brief   枚举值定义
 ****************************************************************************************************/

/* 软件 SPI 模式设置, 请勿修改 */
typedef enum ls_spi_mode
{
    LS_SOFT_SPI_MODE_0 = 0x00,  // CPOL=0, CPHA=0 (时钟默认低电平, 第一个跳变沿采样数据)
    LS_SOFT_SPI_MODE_1,         // CPOL=0, CPHA=1 (时钟默认低电平, 第二个跳变沿采样数据)
    LS_SOFT_SPI_MODE_2,         // CPOL=1, CPHA=0 (时钟默认高电平, 第一个跳变沿采样数据)
    LS_SOFT_SPI_MODE_3,         // CPOL=1, CPHA=1 (时钟默认高电平, 第二个跳变沿采样数据)cd
} ls_spi_mode_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_soft_spi
{
public:
    ls_soft_spi();  // 空构造函数
    // 有参构造函数
    ls_soft_spi(gpio_pin_t _sck, gpio_pin_t _miso, gpio_pin_t _mosi, gpio_pin_t _cs, ls_spi_mode_t _mode);
    ~ls_soft_spi(); // 析构函数

    ls_soft_spi(const ls_soft_spi& other);              // 拷贝构造
    ls_soft_spi& operator=(const ls_soft_spi& other);   // 拷贝赋值

public:
    uint8_t soft_spi_read_byte(const uint8_t _reg);                                 // 读取单字节数据
    void    soft_spi_read_n_byte(const uint8_t _reg, uint8_t *_buf, uint16_t _len); // 读取多个字节数据

    void    soft_spi_write_byte(const uint8_t _reg, const uint8_t _val);                    // 写入单字节数据
    void    soft_spi_write_n_byte(const uint8_t _reg, const uint8_t *_buf, uint16_t _len);  // 写入多个字节数据

private:
    void soft_spi_read_write_n_byte(uint8_t *_buf, uint16_t _len);  // 模拟 SPI 读写数据及长度

private:
    ls_gpio       sck;  // SPI 时钟引脚
    ls_gpio       miso; // SPI 数据引脚
    ls_gpio       mosi; // SPI 数据引脚
    ls_gpio       cs;   // SPI 使能引脚
    ls_spi_mode_t mode; // SPI 模式
};

#endif
