#ifndef __LQ_REG_SPI_HPP
#define __LQ_REG_SPI_HPP

#include <iostream>
#include <mutex>
#include <memory>
#include <pthread.h>
#include <cstdint>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include "lq_reg_gpio.hpp"
#include "lq_map_addr.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

/******************** SPI2/3相关寄存器偏移 ********************/

#define LS_SPI_CR1                  ( 0x00 )        // 控制寄存器1
#define LS_SPI_CR2                  ( 0x04 )        // 控制寄存器2
#define LS_SPI_CR3                  ( 0x08 )        // 控制寄存器3
#define LS_SPI_CR4                  ( 0x0C )        // 控制寄存器4
#define LS_SPI_IER                  ( 0x10 )        // 中断寄存器
#define LS_SPI_SR1                  ( 0x14 )        // 状态寄存器1
#define LS_SPI_SR2                  ( 0x18 )        // 状态寄存器2
#define LS_SPI_CFG1                 ( 0x20 )        // 配置寄存器1
#define LS_SPI_CFG2                 ( 0x24 )        // 配置寄存器2
#define LS_SPI_CFG3                 ( 0x28 )        // 配置寄存器3
#define LS_SPI_CRC1                 ( 0x30 )        // CRC寄存器1
#define LS_SPI_CRC2                 ( 0x34 )        // CRC寄存器2
#define LS_SPI_DR                   ( 0x40 )        // 数据寄存器

/******************* SPI2/3相关寄存器控制位 *******************/

// CR1 位定义
#define LS_SPI_CR1_SSREV            ( 1 << 8 )      // SS极性反转
#define LS_SPI_CR1_AUTOSUS          ( 1 << 2 )      // 自动挂起
#define LS_SPI_CR1_CSTART           ( 1 << 1 )      // 传输启动
#define LS_SPI_CR1_SPE              ( 1 << 0 )      // SPI 使能

// CR2 位定义
#define LS_SPI_CR2_TXDMAEN          ( 1 << 15 )     // TX DMA使能
#define LS_SPI_CR2_TXFTHLV_SHIFT    ( 8 )           // TX FIFO阈值偏移
#define LS_SPI_CR2_TXFTHLV          ( 0x3 << 8 )    // TX FIFO阈值掩码
#define LS_SPI_CR2_RXDMAEN          ( 1 << 7 )      // RX DMA使能
#define LS_SPI_CR2_RXFTHLV_SHIFT    ( 0 )           // RX FIFO阈值偏移
#define LS_SPI_CR2_RXFTHLV          ( 0x3 << 0 )    // RX FIFO阈值掩码

// CFG1位定义
#define LS_SPI_CFG1_DSIZE_SHIFT     ( 8 )           // 数据位宽偏移
#define LS_SPI_CFG1_DSIZE           ( 0x1F << 8 )   // 数据位宽掩码
#define LS_SPI_CFG1_LSBFRST         ( 1 << 7 )      // LSB优先
#define LS_SPI_CFG1_CPHA            ( 1 << 1 )      // 时钟相位
#define LS_SPI_CFG1_CPOL            ( 1 << 0 )      // 时钟极性

// CFG2位定义
#define LS_SPI_CFG2_BRINT_SHIFT     ( 8 )           // 分频整数部分偏移
#define LS_SPI_CFG2_BRINT           ( 0xFF << 8 )   // 分频整数部分掩码
#define LS_SPI_CFG2_BRDEC_SHIFT     ( 2 )           // 分频小数部分偏移
#define LS_SPI_CFG2_BRDEC           ( 0x3F << 2 )   // 分频小数部分掩码

// CFG3位定义
#define LS_SPI_CFG3_SSMODE_SHIFT    ( 8 )           // 软件从机模式偏移
#define LS_SPI_CFG3_SSMODE          ( 0x3 << 8 )    // 软件从机模式掩码
#define LS_SPI_CFG3_DOE             ( 1 << 3 )      // DO使能
#define LS_SPI_CFG3_DIE             ( 1 << 2 )      // DI使能
#define LS_SPI_CFG3_DIOSWP          ( 1 << 1 )      // DI/DO交换
#define LS_SPI_CFG3_MSTR            ( 1 << 0 )      // 主机模式

// SR1位定义
#define LS_SPI_SR1_EOT             ( 1 << 15 )      // 传输结束
#define LS_SPI_SR1_MODF            ( 1 << 11 )      // 模式错误
#define LS_SPI_SR1_OVR             ( 1 << 8 )       // 溢出错误
#define LS_SPI_SR1_SUSP            ( 1 << 7 )       // 挂起
#define LS_SPI_SR1_TXE             ( 1 << 6 )       // TX FIFO空
#define LS_SPI_SR1_RXE             ( 1 << 4 )       // RX FIFO满
#define LS_SPI_SR1_TXA             ( 1 << 1 )       // TX FIFO有空间
#define LS_SPI_SR1_RXA             ( 1 << 0 )       // RX FIFO有数据

// SR2位定义
#define LS_SPI_SR2_TXFLV_SHIFT      ( 8 )           // TX FIFO级别偏移
#define LS_SPI_SR2_TXFLV            ( 0x7 << 8 )    // TX FIFO级别掩码
#define LS_SPI_SR2_RXFLV_SHIFT      ( 0 )           // RX FIFO级别偏移
#define LS_SPI_SR2_RXFLV            ( 0x7 << 0 )    // RX FIFO级别掩码

/******************** SPI 常量定义 ********************/

#define LS_SPI_CLK_FRE              ( 160000000L )  // SPI 时钟频率

/****************************************************************************************************
 * @brief   枚举定义
 ****************************************************************************************************/

/* SPI 端口设置 */
typedef enum ls_port
{
    LS_SPI0 = 0x00, // SPI0 端口
    LS_SPI1,        // SPI1 端口
    LS_SPI2,        // SPI2 端口
    LS_SPI3,        // SPI3 端口
    LS_SPI_INVALID
} ls_spi_port_t;

/* SPI 模式设置, 请勿修改 */
typedef enum ls_reg_spi_mode
{
    LS_SPI_MODE_0 = 0x00,  // CPOL=0, CPHA=0 (时钟默认低电平, 第一个跳变沿采样数据)
    LS_SPI_MODE_1,         // CPOL=0, CPHA=1 (时钟默认低电平, 第二个跳变沿采样数据)
    LS_SPI_MODE_2,         // CPOL=1, CPHA=0 (时钟默认高电平, 第一个跳变沿采样数据)
    LS_SPI_MODE_3,         // CPOL=1, CPHA=1 (时钟默认高电平, 第二个跳变沿采样数据)
    LS_SPI_MODE_INVALID
} ls_reg_spi_mode_t;

/* SPI 数据位宽 */
typedef enum ls_spi_bits_per_word
{
    LS_SPI_BPW_4  = 4,
    LS_SPI_BPW_8  = 8,
    LS_SPI_BPW_16 = 16,
    LS_SPI_BPW_32 = 32,
    LS_SPI_BPW_INVALID
} ls_spi_bits_per_word_t;

/* SPI 传输顺序 */
typedef enum ls_spi_data_order
{
    LS_SPI_MSB_FIRST = 0x00,                // MSB 优先
    LS_SPI_LSB_FIRST = LS_SPI_CFG1_LSBFRST, // LSB优先
    LS_SPI_DATA_ORDER_INVALID
} ls_spi_data_order_t;

/* SPI FIFO 阈值 */
typedef enum ls_spi_fifo_threshould
{
    LS_SPI_FIFO_THRESH_1 = 0x00,    // 1 个数据
    LS_SPI_FIFO_THRESH_2 = 0x01,    // 2 个数据
    LS_SPI_FIFO_THRESH_4 = 0x03,    // 4 个数据
    LS_SPI_FIFO_THRESH_INVALID,
} ls_spi_fifo_threshould_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_spi
{
public:
    // 无参构造函数
    ls_spi();
    // 有参构造函数
    ls_spi(ls_spi_port_t _port, uint32_t _speed, ls_reg_spi_mode_t _mode);

    ls_spi(const ls_spi& _other);               // 拷贝构造函数
    ls_spi& operator=(const ls_spi& _other);    // 拷贝赋值运算符

    // 析构函数
    ~ls_spi();

public:
    void spi_enable();   // 使能 SPI 控制器
    void spi_disable();  // 失能 SPI 控制器

    ssize_t spi_transfer(const void* _tx_buf, void* _rx_buf, size_t _len);  // SPI 读写传输（全双工）
    ssize_t spi_write(const void* _tx_buf, size_t _len);                    // SPI 发送数据
    ssize_t spi_read(void* _rx_buf, size_t _len);                           // SPI 接收数据

public:
    uint32_t               spi_get_speed() const;           // 获取 SPI 波特率
    ls_reg_spi_mode_t      spi_get_mode() const;            // 获取 SPI 模式
    ls_spi_bits_per_word_t spi_get_bits_per_word() const;   // 获取 SPI 数据位宽
    ls_spi_data_order_t    spi_get_data_order() const;      // 获取当前数据传输顺序

    bool spi_check_status();     // 检查 SPI 状态寄存器

private:
    void spi_init_0_1();     // 初始化 SPI0 或 SPI1
    void spi_init_2_3();     // 初始化 SPI2 或 SPI3

    void spi_set_speed(const uint32_t _speed);          // 设置 SPI 波特率
    void spi_set_mode(const ls_reg_spi_mode_t _mode);   // 设置 SPI 模式

    void spi_set_bits_per_word(ls_spi_bits_per_word_t _bpw);    // 设置 SPI 数据位宽
    void spi_set_data_order(ls_spi_data_order_t _ord);          // 设置 SPI 数据传输顺序

    void spi_set_fifo_threshold(ls_spi_fifo_threshould_t _tx, ls_spi_fifo_threshould_t _rx);    // 设置 SPI FIFO 阈值

    bool spi_wait_transfer_complete(uint32_t timeout_ms = 1000);    // 等待传输完成

    ssize_t spi_write_fifo(const uint8_t* _tx_buf, size_t _len);    // 写入 TX FIFO
    ssize_t spi_read_fifo(uint8_t* _rx_buf, size_t _len);           // 读取 RX FIFO

private:
    ls_spi_port_t            port;              // SPI 端口号
    uint32_t                 speed_hz;          // SPI 波特率
    ls_reg_spi_mode_t        spi_mode;          // SPI 模式
    ls_spi_bits_per_word_t   bpw;               // SPI 数据位宽
    ls_spi_data_order_t      ord;               // SPI 数据传输顺序
    ls_spi_fifo_threshould_t tx_thresh;         // SPI TX 阈值
    ls_spi_fifo_threshould_t rx_thresh;         // SPI RX 阈值

    ls_gpio                  cs_gpio;                // 
    mutable std::mutex       mtx;               // SPI 操作互斥锁
    mutable std::mutex       mtx_transfer;      // SPI 传输互斥锁
    std::shared_ptr<int>     ref_count;         // SPI 引用计数
    bool                     is_initialized;    // SPI 是否初始化

private:
    // SPI 控制器基地址
    static const ls_reg_base_t LS_SPI2_BASE_ADDR = 0x1610C000; // SPI2 基地址
    static const ls_reg_base_t LS_SPI3_BASE_ADDR = 0x1610E000; // SPI3 基地址

    ls_reg32_addr_t spi_base;   // SPI 基地址
    ls_reg32_addr_t spi_cr1;    // SPI2/3 控制寄存器1
    ls_reg32_addr_t spi_cr2;    // SPI2/3 控制寄存器2
    ls_reg32_addr_t spi_cr3;    // SPI2/3 控制寄存器3
    ls_reg32_addr_t spi_cr4;    // SPI2/3 控制寄存器4
    ls_reg32_addr_t spi_ier;    // SPI2/3 中断寄存器
    ls_reg32_addr_t spi_sr1;    // SPI2/3 状态寄存器1
    ls_reg32_addr_t spi_sr2;    // SPI2/3 状态寄存器2
    ls_reg32_addr_t spi_cfg1;   // SPI2/3 配置寄存器1
    ls_reg32_addr_t spi_cfg2;   // SPI2/3 配置寄存器2
    ls_reg32_addr_t spi_cfg3;   // SPI2/3 配置寄存器3
    ls_reg32_addr_t spi_crc1;   // SPI2/3 CRC寄存器1
    ls_reg32_addr_t spi_crc2;   // SPI2/3 CRC寄存器2
    ls_reg32_addr_t spi_dr;     // SPI2/3 数据寄存器
};

#endif
