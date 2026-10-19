#include <cstring>
#include <thread>
#include <chrono>
#include <vector>
#include "lq_reg_spi.hpp"
#include "lq_assert.hpp"

/*!
 * @brief   后续需要修改的问题
 * @note    1. CS 引脚没波形, 需要改成软件控制
 *          2. 设置模式0、1、2都可用, 但模式3有问题, 暂时屏蔽
 *          3. 暂时只有 SPI2 和 SPI3 可用, SPI0 和 SPI1 因为寄存器不同暂时未添加
 */

/********************************************************************************
 * @brief   硬件 SPI 无参构造函数.
 * @param   none.
 * @return  none.
 * @example ls_spi MySPI;
 * @note    none.
 ********************************************************************************/
ls_spi::ls_spi() :speed_hz(0), spi_mode(LS_SPI_MODE_INVALID), bpw(LS_SPI_BPW_INVALID),
                  ord(LS_SPI_DATA_ORDER_INVALID), is_initialized(false), port(LS_SPI_INVALID),
                  tx_thresh(LS_SPI_FIFO_THRESH_1), rx_thresh(LS_SPI_FIFO_THRESH_1),
                  ref_count(std::make_shared<int>(0))
{
    this->spi_base = this->spi_cr1  = this->spi_cr2  = this->spi_cr3  = nullptr;
    this->spi_cr4  = this->spi_ier  = this->spi_sr1  = this->spi_sr2  = nullptr;
    this->spi_cfg1 = this->spi_cfg2 = this->spi_cfg3 = this->spi_crc1 = nullptr;
    this->spi_crc2 = this->spi_dr   = nullptr;
}

/********************************************************************************
 * @brief   硬件 SPI 有参构造函数.
 * @param   _port  : SPI 端口, 参考 ls_spi_port_t 枚举.
 * @param   _speed : SPI 频率.
 * @param   _mode  : SPI 模式, 参考 ls_reg_spi_mode_t 枚举.
 * @return  none.
 * @example ls_spi MySPI(LS_SPI2, 5000000, LS_SPI_MODE0);
 * @note    none.
 ********************************************************************************/
ls_spi::ls_spi(ls_spi_port_t _port, uint32_t _speed, ls_reg_spi_mode_t _mode) : port(_port),
    speed_hz(_speed), spi_mode(_mode), bpw(LS_SPI_BPW_8), ord(LS_SPI_MSB_FIRST), is_initialized(false),
    tx_thresh(LS_SPI_FIFO_THRESH_1), rx_thresh(LS_SPI_FIFO_THRESH_1), ref_count(std::make_shared<int>(1))
{
    ls_reg_base_t _spi_base;
    if (_port == LS_SPI2)
    {
        gpio_mux_set(PIN_64, GPIO_MUX_MAIN);
        gpio_mux_set(PIN_65, GPIO_MUX_MAIN);
        gpio_mux_set(PIN_66, GPIO_MUX_MAIN);
        // gpio_mux_set(PIN_67, GPIO_MUX_MAIN);
        this->cs_gpio = ls_gpio(PIN_67, GPIO_MODE_OUT);
        _spi_base = LS_SPI2_BASE_ADDR;
    }
    else if (_port == LS_SPI3)
    {
        gpio_mux_set(PIN_82, GPIO_MUX_ALT1);
        gpio_mux_set(PIN_83, GPIO_MUX_ALT1);
        gpio_mux_set(PIN_84, GPIO_MUX_ALT1);
        // gpio_mux_set(PIN_85, GPIO_MUX_ALT1);
        this->cs_gpio = ls_gpio(PIN_85, GPIO_MODE_OUT);
        _spi_base = LS_SPI3_BASE_ADDR;
    }
    // 映射基地址
    this->spi_base = LQ::ls_addr_mmap(_spi_base);
    // 计算各寄存器地址
    this->spi_cr1  = ls_reg_addr_calc(this->spi_base, LS_SPI_CR1);
    this->spi_cr2  = ls_reg_addr_calc(this->spi_base, LS_SPI_CR2);
    this->spi_cr3  = ls_reg_addr_calc(this->spi_base, LS_SPI_CR3);
    this->spi_cr4  = ls_reg_addr_calc(this->spi_base, LS_SPI_CR4);
    this->spi_ier  = ls_reg_addr_calc(this->spi_base, LS_SPI_IER);
    this->spi_sr1  = ls_reg_addr_calc(this->spi_base, LS_SPI_SR1);
    this->spi_sr2  = ls_reg_addr_calc(this->spi_base, LS_SPI_SR2);
    this->spi_cfg1 = ls_reg_addr_calc(this->spi_base, LS_SPI_CFG1);
    this->spi_cfg2 = ls_reg_addr_calc(this->spi_base, LS_SPI_CFG2);
    this->spi_cfg3 = ls_reg_addr_calc(this->spi_base, LS_SPI_CFG3);
    this->spi_crc1 = ls_reg_addr_calc(this->spi_base, LS_SPI_CRC1);
    this->spi_crc2 = ls_reg_addr_calc(this->spi_base, LS_SPI_CRC2);
    this->spi_dr   = ls_reg_addr_calc(this->spi_base, LS_SPI_DR);
    this->cs_gpio.gpio_level_set(GPIO_HIGH);
    // 初始化 SPI
    if (_port == LS_SPI2 || _port == LS_SPI3)
    {
        this->spi_init_2_3();
    }
}

/********************************************************************************
 * @brief   硬件 SPI 析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    none.
 ********************************************************************************/
ls_spi::~ls_spi()
{
    std::lock_guard<std::mutex> lock(mtx);

    // 禁用 SPI
    if (this->is_initialized)
        this->spi_disable();

    // 释放内存映射(最后一个引用)
    if (this->ref_count && --(*ref_count) == 0 && this->spi_base != nullptr)
    {
        LQ::ls_addr_munmap(this->spi_base);
    }

    // 清空寄存器地址
    this->spi_base = this->spi_cr1 = this->spi_cr2 = this->spi_cr3 = this->spi_cr4 = nullptr;
    this->spi_ier = this->spi_sr1 = this->spi_sr2 = this->spi_cfg1 = this->spi_cfg2 = nullptr;
    this->spi_cfg3 = this->spi_crc1 = this->spi_crc2 = this->spi_dr = nullptr;

    // 重置状态
    this->speed_hz = 0;
    this->spi_mode = LS_SPI_MODE_INVALID;
    this->bpw = LS_SPI_BPW_INVALID;
    this->ord = LS_SPI_DATA_ORDER_INVALID;
    this->is_initialized = false;
}

/********************************************************************************
 * @brief   硬件 SPI 拷贝构造函数.
 * @param   _other : 待拷贝的 SPI 对象.
 * @return  none.
 * @example ls_spi MySPI(OtherSPI);
 * @note    none.
 ********************************************************************************/
ls_spi::ls_spi(const ls_spi& _other)
{
    std::lock_guard<std::mutex> lock1(this->mtx);
    std::lock_guard<std::mutex> lock2(_other.mtx);

    // 拷贝配置参数
    this->speed_hz  = _other.speed_hz;
    this->spi_mode  = _other.spi_mode;
    this->bpw       = _other.bpw;
    this->ord       = _other.ord;
    this->tx_thresh = _other.tx_thresh;
    this->rx_thresh = _other.rx_thresh;

    // 共享寄存器和引用计数
    this->spi_base = _other.spi_base;
    this->spi_cr1  = _other.spi_cr1;
    this->spi_cr2  = _other.spi_cr2;
    this->spi_cr3  = _other.spi_cr3;
    this->spi_cr4  = _other.spi_cr4;
    this->spi_ier  = _other.spi_ier;
    this->spi_sr1  = _other.spi_sr1;
    this->spi_sr2  = _other.spi_sr2;
    this->spi_cfg1 = _other.spi_cfg1;
    this->spi_cfg2 = _other.spi_cfg2;
    this->spi_cfg3 = _other.spi_cfg3;
    this->spi_crc1 = _other.spi_crc1;
    this->spi_crc2 = _other.spi_crc2;
    this->spi_dr   = _other.spi_dr;

    this->ref_count = _other.ref_count;
    (*this->ref_count)++;
    this->is_initialized = _other.is_initialized;
}

/********************************************************************************
 * @brief   硬件 SPI 赋值运算符.
 * @param   _other : 待赋值的 SPI 对象.
 * @return  none.
 * @example MySPI = OtherSPI;
 * @note    none.
 ********************************************************************************/
ls_spi& ls_spi::operator=(const ls_spi& _other)
{
    if (this == &_other)
        return *this;

    std::lock_guard<std::mutex> lock1(this->mtx);
    std::lock_guard<std::mutex> lock2(_other.mtx);

    // 禁用 SPI
    if (this->is_initialized)
        this->spi_disable();

    // 拷贝配置参数
    this->speed_hz  = _other.speed_hz;
    this->spi_mode  = _other.spi_mode;
    this->bpw       = _other.bpw;
    this->ord       = _other.ord;
    this->tx_thresh = _other.tx_thresh;
    this->rx_thresh = _other.rx_thresh;

    // 共享寄存器和引用计数
    this->spi_base = _other.spi_base;
    this->spi_cr1  = _other.spi_cr1;
    this->spi_cr2  = _other.spi_cr2;
    this->spi_cr3  = _other.spi_cr3;
    this->spi_cr4  = _other.spi_cr4;
    this->spi_ier  = _other.spi_ier;
    this->spi_sr1  = _other.spi_sr1;
    this->spi_sr2  = _other.spi_sr2;
    this->spi_cfg1 = _other.spi_cfg1;
    this->spi_cfg2 = _other.spi_cfg2;
    this->spi_cfg3 = _other.spi_cfg3;
    this->spi_crc1 = _other.spi_crc1;
    this->spi_crc2 = _other.spi_crc2;
    this->spi_dr   = _other.spi_dr;

    this->ref_count = _other.ref_count;
    (*this->ref_count)++;
    this->is_initialized = _other.is_initialized;

    return *this;
}

/********************************************************************************
 * @brief   硬件 SPI2/3 初始化.
 * @param   none..
 * @return  none.
 * @example none
 * @note    内部调用.
 ********************************************************************************/
void ls_spi::spi_init_2_3()
{
    // 先禁用 SPI
    this->spi_disable();
    // 清空状态寄存器和关闭所有中断
    ls_writel(this->spi_sr1, 0xFFFFFFFF);
    ls_writel(this->spi_ier, 0);
    // 配置为主机模式
    uint32_t cfg3_val = ls_readl(this->spi_cfg3);
    cfg3_val |= LS_SPI_CFG3_MSTR;    // 主机模式
    cfg3_val &= ~LS_SPI_CFG3_DIOSWP; // DI/DO 不交换
    ls_writel(this->spi_cfg3, cfg3_val);
    // 禁用自动挂起
    uint32_t cr1_val = ls_readl(this->spi_cr1);
    cr1_val &= ~LS_SPI_CR1_AUTOSUS;
    ls_writel(this->spi_cr1, cr1_val);
    // 设置默认 FIFO 阈值
    this->spi_set_fifo_threshold(this->tx_thresh, this->rx_thresh);
    // 设置 SPI 模式
    this->spi_set_mode(this->spi_mode);
    // 设置数据位宽
    this->spi_set_bits_per_word(this->bpw);
    // 设置数据传输顺序
    this->spi_set_data_order(this->ord);
    // 设置时钟频率
    this->spi_set_speed(this->speed_hz);
    // 标记为已初始化
    this->is_initialized = true;
}

/********************************************************************************
 * @brief   硬件 SPI 启用函数.
 * @param   none..
 * @return  none.
 * @example MySPI.spi_enable();
 * @note    none.
 ********************************************************************************/
void ls_spi::spi_enable()
{
    // 加锁, 防止并发访问
    std::lock_guard<std::mutex> lock(mtx);
    if (this->spi_cr1 == nullptr)
    {
        lq_log_error("spi%d cr1 reg is nullptr", this->port);
        return;
    }
    // 使能 SPI
    uint32_t cr1_val = ls_readl(this->spi_cr1);
    cr1_val |= LS_SPI_CR1_SPE;
    ls_writel(this->spi_cr1, cr1_val);
}

/********************************************************************************
 * @brief   硬件 SPI 禁用函数.
 * @param   none.
 * @return  none.
 * @example MySPI.spi_disable();
 * @note    none.
 ********************************************************************************/
void ls_spi::spi_disable()
{
    // 加锁, 防止并发访问
    std::lock_guard<std::mutex> lock(mtx);
    if (this->spi_cr1 == nullptr)
    {
        lq_log_error("spi%d cr1 reg is nullptr", this->port);
        return;
    }

    uint32_t cr1_val = ls_readl(this->spi_cr1);
    // 停止传输
    cr1_val = ls_readl(this->spi_cr1);
    cr1_val &= ~LS_SPI_CR1_CSTART; // 停止传输
    ls_writel(this->spi_cr1, cr1_val);

    cr1_val = ls_readl(this->spi_cr1);
    cr1_val &= ~LS_SPI_CR1_SPE; // 失能 SPI
    ls_writel(this->spi_cr1, cr1_val);
}

/********************************************************************************
 * @brief   硬件 SPI 设置 FIFO 阈值.
 * @param   _tx : TX FIFO 阈值, 参考 ls_spi_fifo_threshould_t 枚举.
 * @param   _rx : RX FIFO 阈值, 参考 ls_spi_fifo_threshould_t 枚举.
 * @return  none.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
void ls_spi::spi_set_fifo_threshold(ls_spi_fifo_threshould_t _tx, ls_spi_fifo_threshould_t _rx)
{
    // 加锁, 防止并发访问
    std::lock_guard<std::mutex> lock(mtx);
    if (_tx > LS_SPI_FIFO_THRESH_4 || _rx > LS_SPI_FIFO_THRESH_4)
    {
        lq_log_error("spi%d fifo threshould is invalid", this->port);
        return;
    }
    if (this->spi_cr2 == nullptr)
    {
        lq_log_error("spi%d cr2 reg is nullptr", this->port);
        return;
    }

    uint32_t cr2_val = ls_readl(this->spi_cr2);
    cr2_val &= ~(LS_SPI_CR2_TXFTHLV | LS_SPI_CR2_RXFTHLV); // 清除原有阈值
    cr2_val |= (_tx << LS_SPI_CR2_TXFTHLV_SHIFT);          // 设置 TX 阈值
    cr2_val |= (_rx << LS_SPI_CR2_RXFTHLV_SHIFT);          // 设置 RX 阈值
    ls_writel(this->spi_cr2, cr2_val);
    this->tx_thresh = _tx;
    this->rx_thresh = _rx;
}

/********************************************************************************
 * @brief   硬件 SPI 模式配置.
 * @param   _mode : SPI 模式, 参考 ls_reg_spi_mode_t 枚举.
 * @return  none.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
void ls_spi::spi_set_mode(ls_reg_spi_mode_t _mode)
{
    // 加锁, 防止并发访问
    std::lock_guard<std::mutex> lock(mtx);
    if (_mode > LS_SPI_MODE_3)
    {
        lq_log_error("spi%d mode is invalid", this->port);
        return;
    }
    if (this->spi_cfg1 == nullptr)
    {
        lq_log_error("spi%d cfg1 reg is nullptr", this->port);
        return;
    }

    uint32_t cfg1_val = ls_readl(this->spi_cfg1);
    cfg1_val &= ~(LS_SPI_CFG1_CPOL | LS_SPI_CFG1_CPHA); // 清除原有模式
    cfg1_val |= _mode;                                  // 设置新模式
    ls_writel(this->spi_cfg1, cfg1_val);
    this->spi_mode = _mode;
}

/*********************************************************************************
 * @brief   硬件 SPI 设置数据位宽.
 * @param   _bpw : 数据位宽, 参考 ls_spi_bits_per_word_t 枚举.
 * @return  none.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
void ls_spi::spi_set_bits_per_word(ls_spi_bits_per_word_t _bpw)
{
    // 加锁, 防止并发访问
    std::lock_guard<std::mutex> lock(mtx);
    if (_bpw > LS_SPI_BPW_32)
    {
        lq_log_error("spi%d bpw is invalid", this->port);
        return;
    }
    if (this->spi_cfg1 == nullptr)
    {
        lq_log_error("spi%d cfg1 reg is nullptr", this->port);
        return;
    }

    uint32_t cfg1_val = ls_readl(this->spi_cfg1);
    cfg1_val &= ~LS_SPI_CFG1_DSIZE;                      // 清除原有数据位宽
    cfg1_val |= ((_bpw - 1) << LS_SPI_CFG1_DSIZE_SHIFT); // 设置新数据位宽
    ls_writel(this->spi_cfg1, cfg1_val);
    this->bpw = _bpw;
}

/********************************************************************************
 * @brief   硬件 SPI 数据顺序配置.
 * @param   _ord : SPI 数据顺序, 参考 ls_spi_data_order_t 枚举.
 * @return  none.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
void ls_spi::spi_set_data_order(ls_spi_data_order_t _ord)
{
    // 加锁, 防止并发访问
    std::lock_guard<std::mutex> lock(mtx);
    if (_ord > LS_SPI_LSB_FIRST)
    {
        lq_log_error("spi%d order is invalid", this->port);
        return;
    }
    if (this->spi_cfg1 == nullptr)
    {
        lq_log_error("spi%d cfg1 reg is nullptr", this->port);
        return;
    }

    uint32_t cfg1_val = ls_readl(this->spi_cfg1);
    if (_ord == LS_SPI_LSB_FIRST)
    {
        cfg1_val |= LS_SPI_CFG1_LSBFRST;
    }
    else
    {
        cfg1_val &= ~LS_SPI_CFG1_LSBFRST;
    }
    ls_writel(this->spi_cfg1, cfg1_val);
    this->ord = _ord;
}

/********************************************************************************
 * @brief   硬件 SPI 速度配置.
 * @param   _speed : SPI 速度, 单位为 Hz.
 * @return  none.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
void ls_spi::spi_set_speed(uint32_t _speed)
{
    // 加锁, 防止并发访问
    std::lock_guard<std::mutex> lock(mtx);
    if (_speed == 0)
    {
        lq_log_error("spi%d speed must be greater than 0", this->port);
        return;
    }
    if (this->spi_cfg2 == nullptr)
    {
        lq_log_error("spi%d cfg2 reg is nullptr", this->port);
        return;
    }

    // 计算分频值
    uint32_t div = LS_SPI_CLK_FRE / _speed;
    if (div < 2)
    {
        div = 2;
        lq_log_warn("spi%d speed too high, adjusted to max 8MHz", this->port);
    }    
    if (div > 255)
    {
        div = 255;
        lq_log_warn("spi%d speed too low, adjusted to min 627.35KHz", this->port);
    }
    // 实际频率
    uint32_t actual_speed = LS_SPI_CLK_FRE / div;
    // 配置分频寄存器
    uint32_t cfg2_val = ls_readl(this->spi_cfg2);
    cfg2_val &= ~LS_SPI_CFG2_BRINT;               // 清除原有值
    cfg2_val |= (div << LS_SPI_CFG2_BRINT_SHIFT); // 设置分频值
    ls_writel(this->spi_cfg2, cfg2_val);

    std::this_thread::sleep_for(std::chrono::nanoseconds(10));
    // 更新当前频率
    this->speed_hz = actual_speed;
}

/********************************************************************************
 * @brief   硬件 SPI 等待传输完成.
 * @param   timeout_ms : 超时时间, 单位为毫秒.
 * @return  传输完成返回 true, 超时返回 false.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
bool ls_spi::spi_wait_transfer_complete(uint32_t timeout_ms)
{
    auto start = std::chrono::steady_clock::now();

    while (true)
    {
        // 检查超时
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (duration.count() > timeout_ms)
        {
            lq_log_warn("spi%d transfer timeout!", this->port);
            return false;
        }

        // 检查传输完成标志
        uint32_t sr1_val = ls_readl(this->spi_sr1);
        if (sr1_val & LS_SPI_SR1_EOT)
        {
            // 清除 EOT 标志
            ls_writel(this->spi_sr1, LS_SPI_SR1_EOT);
            return true;
        }

        // 检查错误
        if (sr1_val & (LS_SPI_SR1_MODF | LS_SPI_SR1_OVR))
        {
            lq_log_error("spi%d transfer error: SR1=0x%x", this->port, this->spi_sr1);
            // 清除错误标志
            ls_writel(this->spi_sr1, sr1_val & (LS_SPI_SR1_MODF | LS_SPI_SR1_OVR));
            return false;
        }
        // 短暂延时
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/********************************************************************************
 * @brief   硬件 SPI 从 FIFO 读取数据.
 * @param   _rx_buf : 读取数据存放的缓冲区.
 * @param   _len    : 读取数据长度.
 * @return  实际读取的数据长度.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
ssize_t ls_spi::spi_write_fifo(const uint8_t *_tx_buf, size_t _len)
{
    ssize_t written = 0;

    while (written < _len && (ls_readl(this->spi_sr1) & LS_SPI_SR1_TXA))
    {
        // 检查 TX FIFO 是否有空间
        // uint32_t sr1_val = ls_readl(this->spi_sr1);
        // if (!(sr1_val & LS_SPI_SR1_TXA)) break;

        // 根据位宽写入数据
        if (this->bpw == LS_SPI_BPW_32 && (_len - written) >= 4)
        {
            uint32_t dat = *((const uint32_t *)(_tx_buf + written));
            ls_writel(this->spi_dr, dat);
            written += 4;
        }
        else if (this->bpw == LS_SPI_BPW_16 && (_len - written) >= 2)
        {
            uint16_t dat = *((const uint16_t *)(_tx_buf + written));
            ls_writel(this->spi_dr, dat);
            written += 2;
        }
        else
        {
            uint8_t dat = _tx_buf[written];
            ls_writeb(this->spi_dr, dat);
            written += 1;
        }
    }
    return written;
}

/********************************************************************************
 * @brief   硬件 SPI 读取 FIFO 数据.
 * @param   _rx_buf : 读取数据存放的缓冲区.
 * @param   _len    : 读取数据长度.
 * @return  实际读取的数据长度.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
ssize_t ls_spi::spi_read_fifo(uint8_t *_rx_buf, size_t _len)
{
    ssize_t read = 0;

    while (read < _len && (ls_readl(this->spi_sr1) & LS_SPI_SR1_RXA))
    {
        // 检查 RX FIFO 是否有数据
        // uint32_t sr1_val = ls_readl(this->spi_sr1);
        // if (!(sr1_val & LS_SPI_SR1_RXA)) break;

        // 根据位宽读取数据
        if (this->bpw == LS_SPI_BPW_32 && (_len - read) >= 4)
        {
            uint32_t dat = ls_readl(this->spi_dr);
            *((uint32_t *)(_rx_buf + read)) = dat;
            read += 4;
        }
        else if (this->bpw == LS_SPI_BPW_16 && (_len - read) >= 2)
        {
            uint16_t dat = (uint16_t)ls_readl(this->spi_dr);
            *((uint16_t *)(_rx_buf + read)) = dat;
            read += 2;
        }
        else
        {
            uint8_t dat = (uint8_t)ls_readb(this->spi_dr);
            _rx_buf[read] = dat;
            read += 1;
        }
    }
    return read;
}

/********************************************************************************
 * @brief   硬件 SPI 数据传输.
 * @param   _tx_buf : 传输数据缓冲区.
 * @param   _rx_buf : 接收数据缓冲区.
 * @param   _len    : 传输数据长度.
 * @return  实际传输的数据长度.
 * @example MySPI.spi_transfer(tx_buf, rx_buf, len);.
 * @note    none.
 ********************************************************************************/
ssize_t ls_spi::spi_transfer(const void *_tx_buf, void *_rx_buf, size_t _len)
{
    if (_len == 0)
    {
        lq_log_error("spi%d transfer length is zero!", this->port);
        return -1;
    }
    if (this->spi_base == nullptr)
    {
        lq_log_error("spi%d base reg is nullptr!", this->port);
        return -1;
    }
    if (!this->is_initialized)
    {
        lq_log_error("spi%d is not initialized!", this->port);
        return -1;
    }
    std::lock_guard<std::mutex> lock(this->mtx_transfer);

    this->cs_gpio.gpio_level_set(GPIO_LOW);

    // 转换缓冲区类型
    const uint8_t *tx_data = static_cast<const uint8_t *>(_tx_buf);
    uint8_t *rx_data = static_cast<uint8_t *>(_rx_buf);

    // 清除所有状态标志
    ls_writel(this->spi_sr1, 0xFFFFFFFF);
    ls_writel(this->spi_sr2, 0xFFFFFFFF);

    // 失能 SPI
    this->spi_disable();

    // 配置数据方向(全双工)
    uint32_t cfg3_val = ls_readl(this->spi_cfg3);
    if (_tx_buf && _rx_buf)
    {
        cfg3_val |= (LS_SPI_CFG3_DOE | LS_SPI_CFG3_DIE);
    }
    else if (_tx_buf)
    { // 只写模式
        cfg3_val |= LS_SPI_CFG3_DOE;
        cfg3_val &= ~LS_SPI_CFG3_DIE;
    }
    else if (_rx_buf)
    { // 只读模式
        cfg3_val |= LS_SPI_CFG3_DIE;
        cfg3_val &= ~LS_SPI_CFG3_DOE;
    }
    cfg3_val &= ~LS_SPI_CFG3_DIOSWP;
    ls_writel(this->spi_cfg3, cfg3_val);

    // 设置传输长度
    uint32_t tsize = (_len * 8 + this->bpw - 1) / this->bpw;
    ls_writel(this->spi_cr3, tsize - 1);
    ls_writel(this->spi_cr4, 0);

    // 使能 SPI
    this->spi_enable();

    // 写入初始数据到 FIFO
    uint32_t tx_idx = 0;
    if (_tx_buf != nullptr)
    {
        while ((ls_readl(this->spi_sr1) & LS_SPI_SR1_TXA) && (tx_idx < _len))
        {
            tx_idx += this->spi_write_fifo(tx_data + tx_idx, _len - tx_idx);
        }
    }

    // 启动传输
    uint32_t cr1_val = ls_readl(this->spi_cr1);
    cr1_val |= LS_SPI_CR1_CSTART;
    ls_writel(this->spi_cr1, cr1_val);

    // 主循环传输
    uint32_t rx_idx = 0;
    while (tx_idx < _len || rx_idx < _len)
    {
        // 写入 TX FIFO
        if ((tx_idx < _len) && (ls_readl(this->spi_sr1) & LS_SPI_SR1_TXA))
        {
            tx_idx += this->spi_write_fifo(tx_data + tx_idx, _len - tx_idx);
        }

        // 读取 RX FIFO
        if (rx_idx < _len && (ls_readl(this->spi_sr1) & LS_SPI_SR1_RXA))
        {
            rx_idx += this->spi_read_fifo(rx_data + rx_idx, _len - rx_idx);
        }

        // 检查传输是否完成
        if (ls_readl(this->spi_sr1) & LS_SPI_SR1_EOT)
            break;
    }

    // 等待传输完成
    if (!this->spi_wait_transfer_complete())
    {
        this->spi_disable();
        return -2;
    }

    // 读取剩余数据
    while ((ls_readl(this->spi_sr1) & LS_SPI_SR1_RXA) && (rx_idx < _len))
    {
        rx_idx += this->spi_read_fifo(rx_data + rx_idx, _len - rx_idx);
    }

    // 清除EOT标志（新增）
    ls_writel(this->spi_sr1, LS_SPI_SR1_EOT);

    // 停止传输
    cr1_val = ls_readl(this->spi_cr1);
    cr1_val &= ~LS_SPI_CR1_CSTART;
    ls_writel(this->spi_cr1, cr1_val);

    // 关闭 SPI
    this->spi_disable();
    this->cs_gpio.gpio_level_set(GPIO_HIGH);

    return (tx_idx == _len && rx_idx == _len) ? 0 : -3;
}

/********************************************************************************
 * @brief   硬件 SPI 写入数据.
 * @param   _tx_buf : 写入数据缓冲区.
 * @param   _len    : 写入数据长度.
 * @return  实际写入的数据长度.
 * @example MySPI.spi_write(tx_buf, len);.
 * @note    none.
 ********************************************************************************/
ssize_t ls_spi::spi_write(const void *_tx_buf, size_t _len)
{
    return this->spi_transfer(_tx_buf, nullptr, _len);
}

/********************************************************************************
 * @brief   硬件 SPI 读取数据.
 * @param   _rx_buf : 读取数据缓冲区.
 * @param   _len    : 读取数据长度.
 * @return  实际读取的数据长度.
 * @example MySPI.spi_read(rx_buf, len);.
 * @note    none.
 ********************************************************************************/
ssize_t ls_spi::spi_read(void *_rx_buf, size_t _len)
{
    // 发送全 0 数据
    std::vector<uint8_t> dummy_data(_len, 0);
    return this->spi_transfer(dummy_data.data(), _rx_buf, _len);
}

/********************************************************************************
 * @brief   获取硬件 SPI 速度.
 * @param   none.
 * @return  硬件 SPI 速度.
 * @example MySPI.spi_get_speed();.
 * @note    none.
 ********************************************************************************/
uint32_t ls_spi::spi_get_speed() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->speed_hz;
}

/********************************************************************************
 * @brief   获取硬件 SPI 模式.
 * @param   none.
 * @return  硬件 SPI 模式.
 * @example MySPI.spi_get_mode();.
 * @note    none.
 ********************************************************************************/
ls_reg_spi_mode_t ls_spi::spi_get_mode() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->spi_mode;
}

/********************************************************************************
 * @brief   获取硬件 SPI 数据位宽.
 * @param   none.
 * @return  硬件 SPI 数据位宽.
 * @example MySPI.spi_get_bits_per_word();.
 * @note    none.
 ********************************************************************************/
ls_spi_bits_per_word_t ls_spi::spi_get_bits_per_word() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->bpw;
}

/********************************************************************************
 * @brief   获取硬件 SPI 数据方向.
 * @param   none.
 * @return  硬件 SPI 数据方向.
 * @example MySPI.spi_get_data_order();.
 * @note    none.
 ********************************************************************************/
ls_spi_data_order_t ls_spi::spi_get_data_order() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->ord;
}

/********************************************************************************
 * @brief   检测硬件 SPI 状态.
 * @param   none.
 * @return  硬件 SPI 状态.
 * @example MySPI.spi_check_status();.
 * @note    none.
 ********************************************************************************/
bool ls_spi::spi_check_status()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    if (this->spi_sr1 == nullptr)
    {
        lq_log_error("spi%d sr1 reg is nullptr!");
        return false;
    }
    uint32_t sr1_val = ls_readl(this->spi_sr1);
    // 检查错误标志
    if (sr1_val & (LS_SPI_SR1_MODF | LS_SPI_SR1_OVR))
        return false;
    return true;
}
