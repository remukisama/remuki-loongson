#ifndef __LQ_UART_HPP
#define __LQ_UART_HPP

#include <iostream>
#include <string>
#include <cstring>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include "lq_common.hpp"

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

#define UART1       "/dev/ttyS1"

/****************************************************************************************************
 * @brief   枚举定义
 ****************************************************************************************************/

/* 停止位相关枚举类型, 请勿修改 */
typedef enum ls_uart_stop_bits
{
    LS_UART_STOP1 = 0x00,   // 1 位停止位
    LS_UART_STOP2,          // 2 位停止位
} ls_uart_stop_bits_t;

/* 数据位相关枚举类型, 请勿修改 */
typedef enum ls_uart_data_bits
{
    LS_UART_DATA5 = 0x00,   // 5 位数据位
    LS_UART_DATA6,          // 6 位数据位
    LS_UART_DATA7,          // 7 位数据位
    LS_UART_DATA8,          // 8 位数据位
} ls_uart_data_bits_t;

/* 校验位相关枚举类型, 请勿修改 */
typedef enum ls_uart_parity
{
    LS_UART_PARITY_NONE = 0x00, // 无校验位
    LS_UART_PARITY_ODD,         // 偶校验位
    LS_UART_PARITY_EVEN,        // 奇校验位
} ls_uart_parity_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_uart
{
public:
    ls_uart(const std::string&  _path,                          /* 串口设备文件路径 */
            speed_t             _baud   = B115200,              /* 默认波特率为 115200 */
            ls_uart_stop_bits_t _stop   = LS_UART_STOP1,        /* 默认 1 位停止位 */
            ls_uart_data_bits_t _data   = LS_UART_DATA8,        /* 默认 8 位数据位 */
            ls_uart_parity_t    _parity = LS_UART_PARITY_NONE); /* 默认无校验位 */
    ~ls_uart();

    // 禁用拷贝构造和赋值
    ls_uart(const ls_uart&) = delete;
    ls_uart& operator=(const ls_uart&) = delete;

public:
    ssize_t write_data(const uint8_t* _buf, size_t _len);   // 写入函数
    ssize_t read_data(uint8_t* _buf, size_t _len);          // 读取函数

    bool flush_buffer(void);    // 清空串口缓冲区
    void close_serial(void);    // 关闭文件描述符

private:
    int                 fd;         // 文件描述符
    std::string         dev_path;   // 串口设备文件路径
    struct termios      ts;         // 串口参数结构体
    speed_t             buadrate;   // 波特率
    ls_uart_stop_bits_t stop_bits;  // 停止位
    ls_uart_data_bits_t data_bits;  // 数据位
    ls_uart_parity_t    parity;     // 校验位
};

#endif
