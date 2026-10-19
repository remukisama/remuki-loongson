#include "lq_dht11.hpp"

/********************************************************************************
 * @brief    读取龙芯硬件系统定时器计数值
 * @param    none
 * @return   uint64_t：当前rdtime.d计数器原始数值
 * @note     LoongArch专用汇编rdtime.d，定时器基准频率LS_PMON_CLOCK_FREQ
 ********************************************************************************/
static inline uint64_t loongarch_get_tick(void)
{
    uint64_t tick;
    __asm__ __volatile__("rdtime.d %0, $zero" : "=r"(tick));
    return tick;
}

/********************************************************************************
 * @brief    忙等微秒延时实现
 * @param    us：延时微秒值
 * @return   none
 * @note     根据硬件定时器频率换算计数，自旋循环实现精准us延时，DHT时序专用
 ********************************************************************************/
void lq_dht11::delay_us(uint32_t us)
{
    // 1us 需要计数 = freq / 1000000
    uint64_t per_us = LS_PMON_CLOCK_FREQ / 1000000ULL;
    uint64_t start = loongarch_get_tick();
    uint64_t end   = start + (uint64_t)us * per_us;

    while(loongarch_get_tick() < end);
}

/********************************************************************************
 * @brief    毫秒延时函数
 * @param    ms：延时毫秒值
 * @return   none
 * @note     转换为微秒调用usleep阻塞休眠，适用于≥1ms粗延时
 ********************************************************************************/
void lq_dht11::delay_ms(uint32_t ms)
{
    usleep(ms * 1000);
}

/********************************************************************************
 * @brief    DHT11构造实现：引脚初始化+上电等待+硬件复位
 * @param    data_pin：传入数据引脚
 * @return   none
 * @note     初始化温湿度成员默认0.0f，拉高IO等待100ms传感器上电稳定，调用复位
 ********************************************************************************/
lq_dht11::lq_dht11(gpio_pin_t data_pin) : data_gpio(data_pin, GPIO_MODE_OUT), humidity(0.0f), temperature(0.0f)
{
    this->data_gpio.gpio_level_set(GPIO_HIGH);
    this->delay_ms(100);    // 等待传感器稳定
    this->set_rst();        // 发送复位信号，准备读取数据
}

/********************************************************************************
 * @brief    DHT11发送复位信号
 * @param    none
 * @return   none
 * @note     IO切输出→拉低22ms→拉高30us，DHT11标准主机复位时序
 ********************************************************************************/
void lq_dht11::set_rst()
{
    this->data_gpio.gpio_direction_set(GPIO_MODE_OUT);  // 设置为输出模式
    this->data_gpio.gpio_level_set(GPIO_LOW);           // 拉低数据线
    this->delay_ms(22);                                 // 保持至少18ms的低电平
    this->data_gpio.gpio_level_set(GPIO_HIGH);          // 拉高数据线
    this->delay_us(30);                                 // 主机拉高30us ~ 40us
}

/********************************************************************************
 * @brief    DHT11一次完整数据采集+校验+数据解析
 * @param    none
 * @return   uint16_t：校验成功返回原始数据，校验出错返回0
 * @note     时序流程：主机起始信号→切换输入等待从机应答→循环采集40bit→校验和判断→温湿度浮点换算
             40bit数据排布：湿度整数8bit、湿度小数8bit、温度整数8bit、温度小数8bit、校验8bit
 ********************************************************************************/
uint16_t lq_dht11::read_data(void)
{
    int       i;
    int       timeout     = 0;
    float     small_point = 0;
    uint8_t   verify_num  = 0;//验证值
    long long val         = 0;

    this->data_gpio.gpio_level_set(GPIO_LOW);   // 数据线输出低电平
    this->delay_ms(19);                         // 起始信号保持时间 19ms
    this->data_gpio.gpio_level_set(GPIO_HIGH);  // 数据线拉高
    this->delay_us(20);                         // 主机拉高30us ~ 40
    
    this->data_gpio.gpio_direction_set(GPIO_MODE_IN); // 数据线转为输入模式
    // 如果前面没有错误，则模块会发出低电平的应答信号，所以直接等待 DHT11 拉高，80us
    timeout = 80;
    // 等待高电平的到来
    while((!this->data_gpio.gpio_level_get()) && (timeout > 0))
    {
        this->delay_us(1);
        timeout--;
    }
    // 模块当前处于拉高准备输出数据，所以直接等待 DHT11 拉低，80us
    timeout = 80; // 设置超时时间
    // DATA_GPIO_IN=0 时, while 条件不成立退出 while 说明接收到响应信号
    // 当 timeout<=0 时，while 条件不成立退出 while 说明超时
    // 等待低电平的到来
    while(this->data_gpio.gpio_level_get() && (timeout > 0) )
    {
        this->delay_us(1);
        timeout--;
    }
    #define DHT11_CHECK_TIME 28 //实测发现超过0值的高电平时间
    for(i=0;i<40;i++)//循环接收40位数据
    {
        timeout = 80;
        while( (!this->data_gpio.gpio_level_get()) && (timeout > 0))    //等待低电平过去
        {
            this->delay_us(1);
            timeout--;
        }
        // 超过 0 值的高电平时间
        this->delay_us(DHT11_CHECK_TIME);
        // 如果还是高电平，说明是 1 值
        if (this->data_gpio.gpio_level_get())
        {
            val=(val<<1)+1;
        }
        else //如果是低电平，说明是 0 值
        {
            val<<=1;
        }

        timeout = 80;
        while(this->data_gpio.gpio_level_get() && (timeout > 0))        //如果还是高电平
        {
            this->delay_us(1);
            timeout--;
        }
    }

    this->data_gpio.gpio_direction_set(GPIO_MODE_OUT);  // 转为输出模式
    this->data_gpio.gpio_level_set(GPIO_HIGH);          // 主机拉高数据线，释放总线

    //  湿高8     + 湿低8      + 温高8     + 温低8
    verify_num = (val>>32) + (val>>24) + (val>>16) + (val>>8);
    //计算的校验和 与 接收的校验和 的差为0说明一致，不为0说明不一致
    verify_num = verify_num - (val&0xff);
    //进行校验
    if( verify_num  )
    {
        return 0;
    }
    else //校验成功
    {
        //数据处理
        this->humidity = (val>>32) & 0xff;              // 湿度前 8 位（小数点前数据）
        small_point    = (val>>24) & 0x00ff;            // 湿度后 8 位（小数点后数据）
        small_point    = small_point * 0.1;             // 换算为小数点
        this->humidity = this->humidity + small_point;  // 小数前+小数后

        this->temperature = (val>>16) & 0x0000ff;           // 温度前 8 位（小数点前数据）
        small_point       = (val>>8) & 0x000000ff;          // 温度后 8 位（小数点后数据）
        small_point       = small_point * 0.1;              // 换算为小数点
        this->temperature = this->temperature + small_point;// 小数前+小数后

        return val>>8; //返回未处理的数据
    }
}

/********************************************************************************
 * @brief    读取缓存温度
 * @param    none
 * @return   float：缓存温度
 * @note     直接返回成员变量，不会自动采集，采集需要提前调用read_data()
 ********************************************************************************/
float lq_dht11::get_temperature(void)
{
    return this->temperature;
}

/********************************************************************************
 * @brief    读取缓存湿度
 * @param    none
 * @return   float：缓存湿度
 * @note     直接返回成员变量，不会自动采集，采集需要提前调用read_data()
 ********************************************************************************/
float lq_dht11::get_humidity(void)
{
    return this->humidity;
}

/********************************************************************************
 * @brief    DHT11析构函数
 * @param    none
 * @return   none
 * @note     ls_gpio成员自动析构，无需额外引脚释放
 ********************************************************************************/
lq_dht11::~lq_dht11()
{
}
