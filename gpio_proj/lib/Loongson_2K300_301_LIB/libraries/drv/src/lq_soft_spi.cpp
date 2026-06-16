#include "lq_soft_spi.hpp"

/********************************************************************************
 * @fn      ls_soft_spi::ls_soft_spi();
 * @brief   空构造函数.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_spi::ls_soft_spi()
{
}

/********************************************************************************
 * @fn      ls_soft_spi::ls_soft_spi(gpio_pin_t _sck, gpio_pin_t _miso, gpio_pin_t _mosi, gpio_pin_t _cs, ls_spi_mode_t _mode);
 * @brief   构造函数.
 * @param   _sck  : SPI 时钟引脚号, 参考 gpio_pin_t 枚举.
 * @param   _miso : SPI 数据引脚号, 参考 gpio_pin_t 枚举.
 * @param   _mosi : SPI 数据引脚号, 参考 gpio_pin_t 枚举.
 * @param   _cs   : SPI 使能引脚号, 参考 gpio_pin_t 枚举.
 * @param   _mode : SPI 模式, 参考 ls_spi_mode_t 枚举.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_spi::ls_soft_spi(gpio_pin_t _sck, gpio_pin_t _miso, gpio_pin_t _mosi, gpio_pin_t _cs, ls_spi_mode_t _mode)
    : sck(_sck, GPIO_MODE_OUT), miso(_miso, GPIO_MODE_IN), mosi(_mosi, GPIO_MODE_OUT), cs(_cs, GPIO_MODE_OUT), mode(_mode)
{
    this->cs.gpio_level_set(GPIO_HIGH); // 拉高片选信号
}

/********************************************************************************
 * @fn      ls_soft_spi::~ls_soft_spi();
 * @brief   析构函数.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_spi::~ls_soft_spi()
{
}

/********************************************************************************
 * @fn      ls_soft_spi::ls_soft_spi(const ls_soft_spi& other);
 * @brief   拷贝构造函数.
 * @param   other : 其他对象.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_spi::ls_soft_spi(const ls_soft_spi& other) :
    sck(other.sck), miso(other.miso), mosi(other.mosi), cs(other.cs), mode(other.mode)
{
}

/********************************************************************************
 * @fn      ls_soft_spi::operator=(const ls_soft_spi& other);
 * @brief   拷贝赋值运算符.
 * @param   other : 其他对象.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_spi& ls_soft_spi::operator=(const ls_soft_spi& other)
{
    if (this != &other) {
        this->sck = other.sck;
        this->miso = other.miso;
        this->mosi = other.mosi;
        this->cs = other.cs;
        this->mode = other.mode;
    }
    return *this;
}

/********************************************************************************
 * @code    void ls_soft_spi::soft_spi_read_write_n_byte(uint8_t *_buf, uint16_t _len)
 * @brief   模拟SPI读写数据及长度.
 * @param   _buf : 数据指针.
 * @param   _len : 长度.
 * @return  none.
 * @date    2025-12-26.
 ********************************************************************************/
void ls_soft_spi::soft_spi_read_write_n_byte(uint8_t *_buf, uint16_t _len)
{
    uint8_t i;
    if ((_buf == NULL) || (LS_SOFT_SPI_MODE_3 < this->mode))
        return;
    this->cs.gpio_level_set(GPIO_LOW);          // 拉低片选
    if ((LS_SOFT_SPI_MODE_0 == this->mode) || (LS_SOFT_SPI_MODE_1 == this->mode))
        this->sck.gpio_level_set(GPIO_LOW);     // 初始SCK设为低(CPOL=0)
    else
        this->sck.gpio_level_set(GPIO_HIGH);    // 初始SCK设为高(CPOL=1)
    do {
        for (i = 0; i < 8; i++)
        {
            if ((*_buf) >= 0x80) this->mosi.gpio_level_set(GPIO_HIGH);
            else                 this->mosi.gpio_level_set(GPIO_LOW);
            if (LS_SOFT_SPI_MODE_0 == this->mode)
            {
                this->sck.gpio_level_set(GPIO_HIGH);        // 产生一个上升沿
                (*_buf) = (*_buf) << 1;
                (*_buf) |= this->miso.gpio_level_get();     // 上升沿采样MISO(CPHA=0)
                this->sck.gpio_level_set(GPIO_LOW);         // 产生一个下降沿
            } else if (LS_SOFT_SPI_MODE_1 == this->mode) {
                this->sck.gpio_level_set(GPIO_HIGH);        // 产生一个上升沿
                (*_buf) = (*_buf) << 1;
                this->sck.gpio_level_set(GPIO_LOW);         // 产生一个下降沿
                (*_buf) |= this->miso.gpio_level_get();     // 下降沿采样MISO(CPHA=1)
            } else if (LS_SOFT_SPI_MODE_2 == this->mode) {
                this->sck.gpio_level_set(GPIO_LOW);         // 产生一个下降沿
                (*_buf) = (*_buf) << 1;
                (*_buf) |= this->miso.gpio_level_get();     // 下降沿采样MISO(CPHA=0)
                this->sck.gpio_level_set(GPIO_HIGH);        // 产生一个上升沿
            } else {
                this->sck.gpio_level_set(GPIO_LOW);         // 产生一个下降沿
                (*_buf) = (*_buf) << 1;
                this->sck.gpio_level_set(GPIO_HIGH);
                (*_buf) |= this->miso.gpio_level_get();     // 上升沿采样MISO(CPHA=1)
            }
        }
        _buf++;
    } while(--_len);
    this->cs.gpio_level_set(GPIO_HIGH);         // 拉高片选
}

/********************************************************************************
 * @code    uint8_t ls_soft_spi::soft_spi_read_byte(const uint8_t _reg)
 * @brief   模拟 SPI 从设备读取单字节数据.
 * @param   _reg : 寄存器地址.
 * @return  数据.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
uint8_t ls_soft_spi::soft_spi_read_byte(const uint8_t _reg)
{
    uint8_t buff[2];
    if (LS_SOFT_SPI_MODE_3 < this->mode) return 0;
    buff[0] = _reg | 0x80;
    this->soft_spi_read_write_n_byte(buff, 2);
    return buff[1];
}

/********************************************************************************
 * @code    void ls_soft_spi::soft_spi_read_n_byte(const uint8_t _reg, uint8_t *_buf, uint16_t _len)
 * @brief   模拟 SPI 从设备读取多个字节数据.
 * @param   _reg : 寄存器地址.
 * @param   _buf : 数据指针.
 * @param   _len : 长度.
 * @return  none.
 * @date    2026-02-05.
 * @note    注意发送字节数不要超过最大传输长度 LS_SOFT_SPI_MAX_TRANS_LEN.
 ********************************************************************************/
void ls_soft_spi::soft_spi_read_n_byte(const uint8_t _reg, uint8_t *_buf, uint16_t _len)
{
    uint8_t rx_buf[LS_SOFT_SPI_MAX_TRANS_LEN] = {0}, i;
    if ((_buf == NULL) || (_len == 0) || (LS_SOFT_SPI_MODE_3 < this->mode))
        return;
    if (_len > (LS_SOFT_SPI_MAX_TRANS_LEN - 1)) // 防止数组越界
        _len = LS_SOFT_SPI_MAX_TRANS_LEN - 1;
    rx_buf[0] = _reg | 0x80;
    this->soft_spi_read_write_n_byte(rx_buf, _len + 1);
    for (i = 0; i < _len; i++)
    {
        _buf[i] = rx_buf[i + 1];
    }
}

/*************************************************************************
 * @code    void ls_soft_spi::soft_spi_write_byte(const uint8_t _reg, const uint8_t _val)
 * @brief   模拟SPI向设备写入一字节数据.
 * @param   _reg : 寄存器地址.
 * @param   _val : 待写入的数据.
 * @return  none.
 * @date    2025-12-26.
 *************************************************************************/
void ls_soft_spi::soft_spi_write_byte(const uint8_t _reg, const uint8_t _val)
{
    uint8_t buff[2];
    if (LS_SOFT_SPI_MODE_3 < this->mode) return;
    buff[0] = _reg & 0x7f;  //先发送寄存器
    buff[1] = _val;         //再发送数据
    this->soft_spi_read_write_n_byte(buff, 2);
}

/*************************************************************************
 * @code    void ls_soft_spi::soft_spi_write_n_byte(const uint8_t _reg, const uint8_t *_buf, uint16_t _len)
 * @brief   模拟SPI向设备写入 n 字节数据.
 * @param   _reg : 寄存器地址.
 * @param   _buf : 待写入的数据.
 * @param   _len : 待写入的数据长度.
 * @return  none.
 * @date    2025-12-26.
 *************************************************************************/
void ls_soft_spi::soft_spi_write_n_byte(const uint8_t _reg, const uint8_t *_buf, uint16_t _len)
{
    uint8_t tx_buf[LS_SOFT_SPI_MAX_TRANS_LEN] = {0}, i;
    if ((_buf == NULL) || (_len == 0) || (LS_SOFT_SPI_MODE_3 < this->mode))
        return;
    if (_len > (LS_SOFT_SPI_MAX_TRANS_LEN - 1)) // 防止数组越界
        _len = LS_SOFT_SPI_MAX_TRANS_LEN - 1;
    tx_buf[0] = _reg & 0x7f;
    for (i = 0; i < _len; i++)
    {
        tx_buf[i + 1] = _buf[i];
    }
    this->soft_spi_read_write_n_byte(tx_buf, _len + 1);
}
