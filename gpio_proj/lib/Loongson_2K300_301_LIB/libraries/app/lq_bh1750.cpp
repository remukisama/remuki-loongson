#include "lq_bh1750.hpp"

/********************************************************************************
 * @brief   毫秒级阻塞延时函数
 * @param   ms : 延时毫秒数
 * @return  none
 * @note    基于usleep实现，ms*1000转为微秒入参
 ********************************************************************************/
static void delay_ms(uint16_t ms)
{
    usleep(ms * 1000);  // 将毫秒转换为微秒
}

/********************************************************************************
 * @brief   微秒级阻塞延时函数
 * @param   us : 延时微秒数
 * @return  none
 * @note    直接调用系统usleep实现短延时，用于I2C时序
 ********************************************************************************/
static void delay_us(uint16_t us)
{
    usleep(us);         // 直接使用微秒
}

/********************************************************************************
 * @brief   BH1750构造实现，初始化IO电平+传感器初始化
 * @param   scl  : SCL时钟引脚
 * @param   sda  : SDA数据引脚
 * @param   addr : BH1750 I2C器件地址
 * @return  none
 * @note    I2C总线默认拉高，随后调用init配置传感器
 ********************************************************************************/
lq_bh1750::lq_bh1750(gpio_pin_t scl, gpio_pin_t sda, uint8_t addr)
    : i2c_scl(scl, GPIO_MODE_OUT), i2c_sda(sda, GPIO_MODE_OUT), i2c_addr(addr)
{
    // 初始化 I2C 线为高电平
    BH1750_SCL_H(i2c_scl);
    BH1750_SDA_H(i2c_sda);
    this->init();  // 初始化 BH1750 传感器
}

/********************************************************************************
 * @brief   I2C起始信号时序实现
 * @param   none
 * @return  none
 * @note    SDA输出，先拉高总线，再SDA拉低产生起始
 ********************************************************************************/
void lq_bh1750::start(void)
{
    i2c_sda.gpio_direction_set(GPIO_MODE_OUT);  // 将数据线设置为输出
    BH1750_SDA_H(i2c_sda);  // 拉高数据线
    BH1750_SCL_H(i2c_scl);  // 拉高时钟线
    delay_us(5);            // 短暂延时
    BH1750_SDA_L(i2c_sda);  // 拉低数据线，产生起始条件
    delay_us(5);
    BH1750_SCL_L(i2c_scl);
}

/********************************************************************************
 * @brief   I2C停止信号时序实现
 * @param   none
 * @return  none
 * @note    SCL高电平期间SDA由低变高，生成I2C停止
 ********************************************************************************/
void lq_bh1750::stop(void)
{
    i2c_sda.gpio_direction_set(GPIO_MODE_OUT);  // 将数据线设置为输出
    BH1750_SDA_L(i2c_sda);  // 拉低数据线
    BH1750_SCL_H(i2c_scl);  // 拉高时钟线
    delay_us(5);
    BH1750_SDA_H(i2c_sda);  // 拉高数据线，产生停止条件
    delay_us(5);
}

/********************************************************************************
 * @brief   主机发送ACK/NACK应答时序
 * @param   ack : 1=NACK拉高SDA，0=ACK拉低SDA
 * @return  none
 * @note    SCL拉高采样应答电平，完成时钟时序
 ********************************************************************************/
void lq_bh1750::send_ack(uint8_t ack)
{
    i2c_sda.gpio_direction_set(GPIO_MODE_OUT);  // 将数据线设置为输出
    if (ack) {
        BH1750_SDA_H(i2c_sda);  // 拉高数据线，发送 NACK
    } else {
        BH1750_SDA_L(i2c_sda);  // 拉低数据线，发送 ACK
    }
    BH1750_SCL_H(i2c_scl);  // 拉高时钟线
    delay_us(5);
    BH1750_SCL_L(i2c_scl);
    delay_us(5);
}

/********************************************************************************
 * @brief   读取从机应答电平
 * @param   none
 * @return  uint8_t：读取SDA电平，0=ACK 1=NACK
 * @note    SDA切换输入，SCL拉高读取从机应答
 ********************************************************************************/
uint8_t lq_bh1750::recv_ack(void)
{
    i2c_sda.gpio_direction_set(GPIO_MODE_IN);   // 将数据线设置为输入
    BH1750_SCL_H(i2c_scl);  // 拉高时钟线
    delay_us(5);
    uint8_t ack = BH1750_SDA_READ(i2c_sda);     // 读取 ACK 信号
    BH1750_SCL_L(i2c_scl);
    delay_us(5);
    return ack;
}

/********************************************************************************
 * @brief   I2C发送单字节，高位优先
 * @param   data : 待发送1字节数据
 * @return  none
 * @note    逐bit输出，发送结束自动接收从机应答
 ********************************************************************************/
void lq_bh1750::send_byte(uint8_t data)
{
    i2c_sda.gpio_direction_set(GPIO_MODE_OUT);  // 将数据线设置为输出
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) {
            BH1750_SDA_H(i2c_sda);  // 发送高位
        } else {
            BH1750_SDA_L(i2c_sda);  // 发送低位
        }
        BH1750_SCL_H(i2c_scl);      // 拉高时钟线
        delay_us(5);
        BH1750_SCL_L(i2c_scl);      // 拉低时钟线
        delay_us(5);
        data <<= 1;                 // 左移一位，准备发送下一位
    }
    this->recv_ack();               // 发送完一个字节后接收 ACK 信号
}

/********************************************************************************
 * @brief   I2C读取单字节数据
 * @param   none
 * @return  uint8_t：读取到的一字节数据
 * @note    SDA输入，SCL逐位采样，高位在前拼接数据
 ********************************************************************************/
uint8_t lq_bh1750::recv_byte(void)
{
    uint8_t data = 0;
    BH1750_SDA_H(i2c_sda);          // 拉高数据线，准备接收数据
    i2c_sda.gpio_direction_set(GPIO_MODE_IN);   // 将数据线设置为输入
    for (int i = 0; i < 8; i++) {
        BH1750_SCL_H(i2c_scl);      // 拉高时钟线
        delay_us(5);
        data <<= 1;                 // 左移一位，为接收下一位做准备
        if (BH1750_SDA_READ(i2c_sda)) {
            data |= 0x01;           // 如果数据线为高电平，设置最低位
        }
        BH1750_SCL_L(i2c_scl);      // 拉低时钟线
        delay_us(5);
    }
    return data;
}

/********************************************************************************
 * @brief   单字节命令写入BH1750寄存器
 * @param   cmd : BH1750控制命令码
 * @return  none
 * @note    起始+器件写地址+指令+停止时序
 ********************************************************************************/
void lq_bh1750::single_write(uint8_t cmd)
{
    this->start();                      //起始信号
    this->send_byte(this->i2c_addr);    //发送设备地址+写信号
    this->send_byte(cmd);               //内部寄存器地址
    this->stop();                       //发送停止信号
}

/********************************************************************************
 * @brief   连续读取BH1750光照原始数据
 * @param   none
 * @return  none
 * @note    发送读地址，连续读2字节，最后一字节发NACK，数据存入BUF
 ********************************************************************************/
uint16_t lq_bh1750::Illuminance_read(void)
{
    this->start();                      // 起始信号
    this->send_byte(this->i2c_addr+1);  // 发送设备地址+读信号
	
	for (int i=0; i<3; i++)             // 连续读取2个地址数据，存储中BUF
    {
        BUF[i] = this->recv_byte();     // BUF[0]存储0x32地址中的数据
        if (i == 3)
        {
           this->send_ack(1);           // 最后一个数据需要回NOACK
        }
        else
        {		
          this->send_ack(0);            // 回应ACK
       }
   }
    this->stop();                       // 停止信号
    delay_ms(5);
    return (BUF[0] << 8) | BUF[1];      // 将高低字节拼接成 16 位数据返回
}

/********************************************************************************
 * @brief   BH1750初始化：上电+高分辨率模式配置
 * @param   none
 * @return  none
 * @note    0x01上电，0x10开启H分辨率，等待180ms传感器稳定
 ********************************************************************************/
void lq_bh1750::init(void)
{
    this->single_write(0x01);
    this->single_write(0x01);   // power on
    this->single_write(0x10);   // H- resolution mode	
    delay_ms(180);              //延时180ms
}

/********************************************************************************
 * @brief   BH1750析构函数实现
 * @param   none
 * @return  none
 * @note    预留，暂无资源释放逻辑
 ********************************************************************************/
lq_bh1750::~lq_bh1750()
{
    // 析构函数可以在这里进行资源清理，如果有需要的话
}
