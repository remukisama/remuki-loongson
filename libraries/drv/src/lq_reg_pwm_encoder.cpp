#include "lq_reg_pwm_encoder.hpp"
#include "lq_map_addr.hpp"

/********************************************************************************
 * @brief   PWM 配置编码器的无参构造函数
 * @param   none.
 * @return  none.
 * @example ls_encoder_pwm MyEncoder;
 * @note    none.
 ********************************************************************************/
ls_encoder_pwm::ls_encoder_pwm() : gpio(PIN_INVALID), mux(GPIO_MUX_INVALID), ch(ENC_PWM_CH_INVALID),
    enc_pwm_base(nullptr), enc_pwm_low_buf(nullptr), enc_pwm_full_buf(nullptr), enc_pwm_ctrl(nullptr),
    ref_count(std::make_shared<int>(0))
{
}

/********************************************************************************
 * @brief   PWM 配置编码器的有参构造函数
 * @param   _pin : 编码器 PWM 引脚
 * @param   _dir : 编码器 PWM 方向引脚
 * @return  none.
 * @example ls_encoder_pwm MyEncoder(ENC_PWM0_PIN64, PIN_72);
 * @note    none.
 ********************************************************************************/
ls_encoder_pwm::ls_encoder_pwm(ls_enc_pwm_pin_t _pin, gpio_pin_t _dir)
    : dir(_dir, GPIO_MODE_IN), ref_count(std::make_shared<int>(1))
{
    this->gpio = (gpio_pin_t)((_pin) & 0xFF);
    this->mux  = (gpio_mux_mode_t)((_pin>>10) & 0x03);
    this->ch   = (enc_pwm_channel_t)((_pin>>8) & 0x03);
    // 配置引脚复用
    gpio_mux_set(this->gpio, this->mux);
    // 获取 PWM 控制器基地址
    this->enc_pwm_base = LQ::ls_addr_mmap(this->LS_ENC_PWM_BASE_ADDR + this->ch * LS_ENC_PWM_OFFSET);
    if (this->enc_pwm_base == nullptr)
    {
        std::cerr << "ls_encoder_pwm: enc_pwm_base is nullptr" << std::endl;
        return;
    }
    // 计算各寄存器地址
    this->enc_pwm_low_buf  = ls_reg_addr_calc(this->enc_pwm_base, LS_ENC_PWM_LOW_BUF_OFFSET);
    this->enc_pwm_full_buf = ls_reg_addr_calc(this->enc_pwm_base, LS_ENC_PWM_FULL_BUF_OFFSET);
    this->enc_pwm_ctrl     = ls_reg_addr_calc(this->enc_pwm_base, LS_ENC_PWM_CTRL_OFFSET);
    // 初始化 PWM 控制器为计数模式
    ls_writel(this->enc_pwm_ctrl, 0);
    uint32_t reg = 0 | LS_ENC_PWM_CTRL_EN | LS_ENC_PWM_CTRL_CAPTE | LS_ENC_PWM_CTRL_INTE;
    ls_writel(this->enc_pwm_ctrl, ls_readl(this->enc_pwm_ctrl) | reg);
}

/********************************************************************************
 * @brief   重置编码器计数器
 * @param   none.
 * @return  none.
 * @example ls_encoder_pwm MyEncoder.encoder_reset_counter();
 * @note    none.
 ********************************************************************************/
void ls_encoder_pwm::encoder_reset_counter(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    ls_writel(this->enc_pwm_ctrl, ls_readl(this->enc_pwm_ctrl) | LS_ENC_PWM_CTRL_RST);
}

/********************************************************************************
 * @brief   关闭重置编码器计数器
 * @param   none.
 * @return  none.
 * @example ls_encoder_pwm MyEncoder.encoder_close_reset_counter();
 * @note    none.
 ********************************************************************************/
void ls_encoder_pwm::encoder_close_reset_counter(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    ls_writel(this->enc_pwm_ctrl, ls_readl(this->enc_pwm_ctrl) & ~LS_ENC_PWM_CTRL_RST);
}

/********************************************************************************
 * @brief   获取编码器值
 * @param   none.
 * @return  float 编码器值
 * @example float count = MyEncoder.encoder_get_count();
 * @note    none.
 ********************************************************************************/
float ls_encoder_pwm::encoder_get_count(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    uint32_t val = ls_readl(this->enc_pwm_full_buf);
    if (val == 0) {
        return 0.0f;
    }
    return (float)(LS_ENC_PWM_CLK / val / LQ_NUM_ENCODER_LINE * (this->dir.gpio_level_get() * 2 - 1));
}

/********************************************************************************
 * @brief   拷贝构造函数.
 * @param   other : 要拷贝的 编码器 PWM 实例.
 * @return  none.
 * @example ls_encoder_pwm MyEncoder2(MyEncoder);
 * @note    none.
 ********************************************************************************/
ls_encoder_pwm::ls_encoder_pwm(const ls_encoder_pwm& other)
{
    // 加锁保护, 避免并发赋值时的竞态
    std::lock_guard<std::mutex> lock_this(this->mtx);
    std::lock_guard<std::mutex> lock_other(other.mtx);
    // 拷贝所有硬件资源
    this->gpio = other.gpio;
    this->mux  = other.mux;
    this->ch   = other.ch;
    // 寄存器资源
    this->enc_pwm_base     = other.enc_pwm_base;
    this->enc_pwm_low_buf  = other.enc_pwm_low_buf;
    this->enc_pwm_full_buf = other.enc_pwm_full_buf;
    this->enc_pwm_ctrl     = other.enc_pwm_ctrl;
    // 共享引用计数(多个变量指向同一个计数)
    this->ref_count = other.ref_count;
    (*this->ref_count)++;
}

/********************************************************************************
 * @brief   拷贝赋值运算符重载.
 * @param   other : 要拷贝的 编码器 PWM 实例.
 * @return  none.
 * @example ls_encoder_pwm MyEncoder = MyEncoder;
 * @note    none.
 ********************************************************************************/
ls_encoder_pwm& ls_encoder_pwm::operator=(const ls_encoder_pwm& other)
{
    if (this == &other) return *this;
    // 加锁保护, 避免并发赋值时的竞态
    std::lock_guard<std::mutex> lock_this(this->mtx);
    std::lock_guard<std::mutex> lock_other(other.mtx);
    // 拷贝所有硬件资源
    this->gpio = other.gpio;
    this->mux  = other.mux;
    this->ch   = other.ch;
    // 寄存器资源
    this->enc_pwm_base     = other.enc_pwm_base;
    this->enc_pwm_low_buf  = other.enc_pwm_low_buf;
    this->enc_pwm_full_buf = other.enc_pwm_full_buf;
    this->enc_pwm_ctrl     = other.enc_pwm_ctrl;
    // 共享引用计数(多个变量指向同一个计数)
    this->ref_count = other.ref_count;
    (*this->ref_count)++;
    return *this;
}

/********************************************************************************
 * @brief   析构函数
 * @param   none.
 * @return  none.
 * @example none.
 * @note    在使用完成后调用, 释放相关资源.
 ********************************************************************************/
ls_encoder_pwm::~ls_encoder_pwm()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    if (this->ref_count && --(*this->ref_count) == 0)
    {
        LQ::ls_addr_munmap(this->enc_pwm_base);
    }
    this->enc_pwm_base     = this->enc_pwm_low_buf = nullptr;
    this->enc_pwm_full_buf = this->enc_pwm_ctrl    = nullptr;
    this->gpio = PIN_INVALID;
    this->mux  = GPIO_MUX_INVALID;
    this->ch   = ENC_PWM_CH_INVALID;
}

/********************************************************************************
 * @brief   获取脉冲引脚
 * @param   none.
 * @return  gpio_pin_t 脉冲引脚
 * @example gpio_pin_t pulse = MyEncoder.encoder_get_pulse();
 * @note    none.
 ********************************************************************************/
gpio_pin_t ls_encoder_pwm::encoder_get_pulse(void)
{
    return this->gpio;
}

/********************************************************************************
 * @brief   获取方向引脚
 * @param   none.
 * @return  gpio_pin_t 方向引脚
 * @example gpio_pin_t dir = MyEncoder.encoder_get_dir();
 * @note    none.
 ********************************************************************************/
gpio_pin_t ls_encoder_pwm::encoder_get_dir(void)
{
    return this->dir.gpio_get_pin();
}

/********************************************************************************
 * @brief   获取脉冲引脚复用模式
 * @param   none.
 * @return  gpio_mux_mode_t 脉冲引脚复用模式
 * @example gpio_mux_mode_t mux = MyEncoder.encoder_get_mux();
 * @note    none.
 ********************************************************************************/
gpio_mux_mode_t ls_encoder_pwm::encoder_get_mux(void)
{
    return this->mux;
}

/********************************************************************************
 * @brief   获取编码器 PWM 通道
 * @param   none.
 * @return  enc_pwm_channel_t 编码器 PWM 通道
 * @example enc_pwm_channel_t ch = MyEncoder.encoder_get_channel();
 * @note    none.
 ********************************************************************************/
enc_pwm_channel_t ls_encoder_pwm::encoder_get_channel(void)
{
    return this->ch;
}


