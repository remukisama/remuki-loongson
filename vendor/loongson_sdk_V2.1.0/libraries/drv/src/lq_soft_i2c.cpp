#include "lq_soft_i2c.hpp"

/********************************************************************************
 * @fn      ls_soft_i2c::ls_soft_i2c();
 * @brief   空构造函数.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_i2c::ls_soft_i2c() : addr(0xFF)
{
}

/********************************************************************************
 * @fn      ls_soft_i2c::ls_soft_i2c(gpio_pin_t _scl, gpio_pin_t _sda, uint8_t _addr);
 * @brief   构造函数.
 * @param   _scl : I2C 时钟引脚号, 参考 gpio_pin_t 枚举.
 * @param   _sda : I2C 数据引脚号, 参考 gpio_pin_t 枚举.
 * @param   _addr : 器件地址.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_i2c::ls_soft_i2c(gpio_pin_t _scl, gpio_pin_t _sda, uint8_t _addr)
    : scl(_scl, GPIO_MODE_OUT), sda(_sda, GPIO_MODE_OUT), addr(_addr)
{
    // 设置 I2C 引脚初始化后为高电平
    this->scl.gpio_level_set(GPIO_HIGH);
    this->sda.gpio_level_set(GPIO_HIGH);
}

/********************************************************************************
 * @fn      ls_soft_i2c::~ls_soft_i2c();
 * @brief   析构函数.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_i2c::~ls_soft_i2c()
{
}

/********************************************************************************
 * @fn      ls_soft_i2c::ls_soft_i2c(const ls_soft_i2c& other);
 * @brief   拷贝构造.
 * @param   other : 要拷贝的 ls_soft_i2c 对象.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_i2c::ls_soft_i2c(const ls_soft_i2c& other)
    : scl(other.scl), sda(other.sda), addr(other.addr)
{
}

/********************************************************************************
 * @fn      ls_soft_i2c::operator=(const ls_soft_i2c& other);
 * @brief   赋值运算符重载.
 * @param   other : 要赋值的 ls_soft_i2c 对象.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
ls_soft_i2c& ls_soft_i2c::operator=(const ls_soft_i2c& other)
{
    if (this != &other)
    {
        this->scl  = other.scl;
        this->sda  = other.sda;
        this->addr = other.addr;
    }
    return *this;
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_delay_ms(uint16_t _ms);
 * @brief   延时 _ms 毫秒.
 * @param   _ms : 延时时间, 单位为毫秒.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
void ls_soft_i2c::soft_i2c_delay_ms(uint16_t _ms)
{
    usleep(_ms * 1000);
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_delay_us(uint16_t _us);
 * @brief   延时 _us 微秒.
 * @param   _us : 延时时间, 单位为微秒.
 * @return  none.
 * @date    2026-02-05.
 * @note    none.
 ********************************************************************************/
void ls_soft_i2c::soft_i2c_delay_us(uint16_t _us)
{
    usleep(_us);
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_start(void);
 * @brief   模拟 I2C 起始信号.
 * @return  none.
 * @date    2026-02-05.
 * @note    模拟 I2C 内部使用.
 ********************************************************************************/
void ls_soft_i2c::soft_i2c_start(void)
{
    this->sda.gpio_direction_set(GPIO_MODE_OUT);
    this->sda.gpio_level_set(GPIO_HIGH);
    this->scl.gpio_level_set(GPIO_HIGH);
    this->soft_i2c_delay_us(2);
    this->soft_i2c_delay_us(2);
    this->sda.gpio_level_set(GPIO_LOW); // START:when CLK is high,DATA change form high to low
    this->soft_i2c_delay_us(2);
    this->scl.gpio_level_set(GPIO_LOW); // 钳住I2C总线，准备发送或接收数据
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_stop(void);
 * @brief   模拟 I2C 停止信号.
 * @return  none.
 * @date    2026-02-05.
 * @note    模拟 I2C 内部使用.
 ********************************************************************************/
void ls_soft_i2c::soft_i2c_stop(void)
{
    this->sda.gpio_direction_set(GPIO_MODE_OUT);
    this->soft_i2c_delay_us(2);
    this->scl.gpio_level_set(GPIO_LOW);
    this->sda.gpio_level_set(GPIO_LOW); // STOP:when CLK is high DATA change form low to high
    this->soft_i2c_delay_us(2);
    this->scl.gpio_level_set(GPIO_HIGH);
    this->soft_i2c_delay_us(2);
    this->sda.gpio_level_set(GPIO_HIGH);// 发送I2C总线结束信号
    this->soft_i2c_delay_us(2);
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_wait_ack(void);
 * @brief   模拟 I2C 等待应答信号.
 * @return  0 : 成功, 1 : 失败.
 * @date    2026-02-05.
 * @note    模拟I2C内部使用 有效应答：从机第9个 SCL=0 时 SDA 被从机拉低,并且 SCL = 1时 SDA依然为低.
 ********************************************************************************/
uint8_t ls_soft_i2c::soft_i2c_wait_ack(void)
{
    uint8_t ucErrTime = 0;
    this->sda.gpio_direction_set(GPIO_MODE_IN);
    this->sda.gpio_level_set(GPIO_HIGH);
    this->soft_i2c_delay_us(2);
    this->scl.gpio_level_set(GPIO_HIGH);
    this->soft_i2c_delay_us(2);
    while (this->sda.gpio_level_get())
    {
        ucErrTime++;
        if (ucErrTime > 100)
        {
            this->soft_i2c_stop();
            return 1;
        }
    }
    this->scl.gpio_level_set(GPIO_LOW); // 时钟输出0
    return 0;
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_ack(void);
 * @brief   模拟 I2C 产生 ACK 应答.
 * @return  none.
 * @date    2026-02-05.
 * @note    内部调用 主机接收完一个字节数据后，主机产生的ACK通知从机一个字节数据已正确接收.
 ********************************************************************************/
void ls_soft_i2c::soft_i2c_ack(void)
{
    this->scl.gpio_level_set(GPIO_LOW);
    this->sda.gpio_direction_set(GPIO_MODE_OUT);
    this->sda.gpio_level_set(GPIO_LOW);
    this->soft_i2c_delay_us(2);
    this->scl.gpio_level_set(GPIO_HIGH);
    this->soft_i2c_delay_us(2);
    this->scl.gpio_level_set(GPIO_LOW);
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_nack(void);
 * @brief   模拟 I2C 不产生 ACK 应答.
 * @return  none.
 * @date    2026-02-05.
 * @note    内部调用 主机接收完最后一个字节数据后，主机产生的NACK通知从机发送结束，释放SDA,以便主机产生停止信号.
 ********************************************************************************/
void ls_soft_i2c::soft_i2c_nack(void)
{
    this->scl.gpio_level_set(GPIO_LOW);
    this->sda.gpio_direction_set(GPIO_MODE_OUT);
    this->sda.gpio_level_set(GPIO_HIGH);
    this->soft_i2c_delay_us(2);
    this->scl.gpio_level_set(GPIO_HIGH);
    this->soft_i2c_delay_us(2);
    this->scl.gpio_level_set(GPIO_LOW);
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_send_byte(uint8_t _byte);
 * @brief   模拟 I2C 发送一个字节数据.
 * @param   _byte : 要发送的字节数据.
 * @return  none.
 * @date    2026-02-05.
 * @note    内部调用 主机发送一个字节数据后，主机产生的ACK通知从机一个字节数据已正确接收.
 ********************************************************************************/
void ls_soft_i2c::soft_i2c_send_byte(uint8_t _byte)
{
    uint8_t t;
    this->sda.gpio_direction_set(GPIO_MODE_OUT);
    this->scl.gpio_level_set(GPIO_LOW);
    for (t = 0; t < 8; t++)
    {
        if (_byte & 0x80) {
            this->sda.gpio_level_set(GPIO_HIGH);
        } else {
            this->sda.gpio_level_set(GPIO_LOW);
        }
        this->scl.gpio_level_set(GPIO_HIGH);
        this->soft_i2c_delay_us(2);
        _byte <<= 1;
        this->soft_i2c_delay_us(2);
        this->scl.gpio_level_set(GPIO_LOW);
        this->soft_i2c_delay_us(2);
    }
    this->soft_i2c_delay_us(2);
}

/********************************************************************************
 * @fn      ls_soft_i2c::soft_i2c_read_byte_internal(uint8_t ack);
 * @brief   模拟 I2C 读取一个字节数据.
 * @param   ack : 为 1 时，主机数据还没接收完，为0 时主机数据已全部接收完成.
 * @return  读取到的字节数据.
 * @date    2026-02-05.
 * @note    内部调用 主机接收一个字节数据后，主机产生的ACK通知从机一个字节数据已正确接收.
 ********************************************************************************/
uint8_t ls_soft_i2c::soft_i2c_read_byte_internal(uint8_t _ack)
{
    uint8_t i, receive = 0;
    this->sda.gpio_direction_set(GPIO_MODE_IN);
    for (i = 0; i < 8; i++)
    {
        this->scl.gpio_level_set(GPIO_LOW);
        this->soft_i2c_delay_us(2);
        this->scl.gpio_level_set(GPIO_HIGH);
        receive <<= 1;
        if (this->sda.gpio_level_get())
        {
            receive++;  // 从机发送的电平
        }
        this->soft_i2c_delay_us(2);
    }
    if (_ack)
    {
        this->soft_i2c_ack();   // 发送 ACK
    } else {
        this->soft_i2c_nack();  // 发送 NACK
    }
    return receive;
}

/********************************************************************************
 * @fn      uint8_t ls_soft_i2c::soft_i2c_read_byte(const uint8_t _reg, uint8_t *_buf);
 * @brief   模拟 I2C 读取指定设备, 指定寄存器的一个值.
 * @param   _reg : 寄存器地址.
 * @param   _buf : 读取到的字节数据指针.
 * @return  成功返回 0, 失败返回 1.
 * @date    2026-02-05.
 * @note    内部调用.
 ********************************************************************************/
uint8_t ls_soft_i2c::soft_i2c_read_byte(const uint8_t _reg, uint8_t *_buf)
{
    this->soft_i2c_start();
    this->soft_i2c_send_byte((this->addr) << 1);    // 发送从机地址
    // if {
        this->soft_i2c_wait_ack();
    // } else { // 如果从机未应答则数据发送失败
    //     this->soft_i2c_stop();
    //     return 1;
    this->soft_i2c_send_byte(_reg); // 发送寄存器地址
    this->soft_i2c_wait_ack();
    this->soft_i2c_start();
    this->soft_i2c_send_byte(((this->addr) << 1) + 1);  // 进入接收模式
    this->soft_i2c_wait_ack();
    *_buf = soft_i2c_read_byte_internal(0);
    this->soft_i2c_stop();          // 产生一个停止条件
    return 0;
}

/********************************************************************************
 * @fn      uint8_t ls_soft_i2c::soft_i2c_write_byte(const uint8_t _reg, uint8_t _byte);
 * @brief   模拟 I2C 写入指定设备, 指定寄存器的一个值.
 * @param   _reg  : 寄存器地址.
 * @param   _byte : 要写入的字节数据.
 * @return  成功返回 0, 失败返回 1.
 * @date    2026-02-05.
 * @note    内部调用.
 ********************************************************************************/
uint8_t ls_soft_i2c::soft_i2c_write_byte(const uint8_t _reg, uint8_t _byte)
{
    this->soft_i2c_start();
    this->soft_i2c_send_byte((this->addr) << 1);    // 发送从机地址
    // if {
        this->soft_i2c_wait_ack();
    // } else {
        // this->soft_i2c_stop();
        // return 1;   // 从机地址写入失败
    // }
    this->soft_i2c_send_byte(_reg);     // 发送寄存器地址
    this->soft_i2c_wait_ack();
    this->soft_i2c_send_byte(_byte);    // 发送字节数据
    // if {
        this->soft_i2c_wait_ack();
    // } else {
        // this->soft_i2c_stop();
        // return 1;   // 寄存器地址写入失败
    // }
    this->soft_i2c_stop();          // 产生一个停止条件
    return 0;
}

/********************************************************************************
 * @fn      uint8_t ls_soft_i2c::soft_i2c_read_n_byte(const uint8_t _reg, uint8_t *_buf, uint8_t _len);
 * @brief   模拟 I2C 读取指定设备, 指定寄存器的多个字节数据.
 * @param   _reg : 寄存器地址.
 * @param   _buf : 读取到的字节数据指针.
 * @param   _len : 读取的字节数.
 * @return  成功返回 0, 失败返回 1.
 * @date    2026-02-05.
 * @note    内部调用.
 ********************************************************************************/
uint8_t ls_soft_i2c::soft_i2c_read_n_byte(const uint8_t _reg, uint8_t *_buf, uint8_t _len)
{
    uint8_t temp, count = 0;
    this->soft_i2c_start();
    this->soft_i2c_send_byte((this->addr) << 1);    // 发送从机地址
    // if {
        this->soft_i2c_wait_ack();
    // } else {
    //     this->soft_i2c_stop();
    //     return 1;       // 从机地址读取失败
    // }
    this->soft_i2c_send_byte(_reg); // 发送寄存器地址
    this->soft_i2c_wait_ack();
    this->soft_i2c_start();
    this->soft_i2c_send_byte(((this->addr) << 1) + 1);  // 进入接收模式
    this->soft_i2c_wait_ack();
    for (count = 0; count < _len; count++)
    {
        if (count != (_len - 1))
            temp = this->soft_i2c_read_byte_internal(1);    // 带 ADK 的读数据
        else
            temp = this->soft_i2c_read_byte_internal(0);    // 最后一个字节 NACK
        _buf[count] = temp;
    }
    this->soft_i2c_stop();  // 产生一个停止条件
    return 0;
}

/********************************************************************************
 * @fn      uint8_t ls_soft_i2c::soft_i2c_write_n_byte(const uint8_t _reg, uint8_t *_buf, uint8_t _len);
 * @brief   模拟 I2C 写入指定设备, 指定寄存器的多个字节数据.
 * @param   _reg : 寄存器地址.
 * @param   _buf : 要写入的字节数据指针.
 * @param   _len : 写入的字节数.
 * @return  成功返回 0, 失败返回 1.
 * @date    2026-02-05.
 * @note    内部调用.
 ********************************************************************************/
uint8_t ls_soft_i2c::soft_i2c_write_n_byte(const uint8_t _reg, uint8_t *_buf, uint8_t _len)
{
    uint8_t count = 0;
    this->soft_i2c_start();
    this->soft_i2c_send_byte((this->addr) << 1);    // 发送从机地址
    // if {
        this->soft_i2c_wait_ack();
    // } else {
    //     this->soft_i2c_stop();
    //     return 1;   // 从机地址写入失败
    // }
    this->soft_i2c_send_byte(_reg); // 发送寄存器地址
    this->soft_i2c_wait_ack();
    for (count = 0; count < _len; count++)
    {
        this->soft_i2c_send_byte(_buf[count]);
        // if {
            this->soft_i2c_wait_ack();
        // } else {    // 每一个字节都要等从机应答
        //     this->soft_i2c_stop();
        //     return 1;   // 数据写入失败
        // }
    }
    this->soft_i2c_stop();  // 产生一个停止条件
    return 0;
}
