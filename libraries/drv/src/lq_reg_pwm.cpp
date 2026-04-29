#include "lq_reg_pwm.hpp"
#include "lq_map_addr.hpp"

/********************************************************************************
 * @brief   硬件配置 PWM 的无参构造函数.
 * @param   none.
 * @return  none.
 * @example ls_pwm MyPwm;
 * @note    none.
 ********************************************************************************/
ls_pwm::ls_pwm() : gpio(PIN_INVALID), mux(GPIO_MUX_INVALID), ch(PWM_CH_INVALID), pola(PWM_POL_INVALID),
    period(0), duty(0), ref_count(std::make_shared<int>(0)),
    pwm_base(nullptr), pwm_low_buf(nullptr), pwm_full_buf(nullptr), pwm_ctrl(nullptr)
{
}

/********************************************************************************
 * @brief   硬件配置 PWM 的有参构造函数.
 * @param   _ch     : 对应使用的引脚号, 参考 pwm_pin_t 枚举.
 * @param   _period : PWM 周期.
 * @param   _duty   : PWM 占空比.
 * @param   _pola   : PWM 极性, 参考 pwm_polarity_t 枚举, 默认为 PWM_POL_INV.
 * @return  none.
 * @example ls_pwm MyPwm(PWM0_PIN64, 50, 2000);
 * @note    该类设置周期时, 输入值是多少, 周期就是多少.
 *          例如: 要设置周期为 50Hz, 周期值填入 50 即可.
 *          设置占空比的话, 无论周期是多少, 占空比范围都为 0 - 10000
 ********************************************************************************/
ls_pwm::ls_pwm(pwm_pin_t _ch, uint32_t _period, uint32_t _duty, pwm_polarity_t _pola) : ref_count(std::make_shared<int>(1))
{
    this->gpio = (gpio_pin_t)(_ch & 0xFF);
    this->ch   = (pwm_channel_t)((_ch >> 8) & 0x03);
    this->mux  = (gpio_mux_mode_t)((_ch >> 10) & 0x03);
    // 配置 gpio 复用
    gpio_mux_set(this->gpio, this->mux);
    // 获取 PWMx 控制器基地址
    this->pwm_base = LQ::ls_addr_mmap(this->LS_PWM_BASE_ADDR + (this->ch * LS_PWM_OFF));
    if (this->pwm_base == nullptr)
    {
        std::cerr << "ls_pwm: pwm_base is nullptr" << std::endl;
        return;
    }
    // 计算各寄存器地址
    this->pwm_low_buf  = ls_reg_addr_calc(this->pwm_base, LS_PWM_LOW_BUF_OFF);
    this->pwm_full_buf = ls_reg_addr_calc(this->pwm_base, LS_PWM_FULL_BUF_OFF);
    this->pwm_ctrl     = ls_reg_addr_calc(this->pwm_base, LS_PWM_CTRL_OFF);
    
    this->pwm_set_polarity(_pola);
    this->pwm_set_period(_period);
    this->pwm_set_duty(_duty);
}

/********************************************************************************
 * @brief   设置 PWM 极性.
 * @param   _pola : PWM 极性, 参考 pwm_polarity_t 枚举.
 * @return  none.
 * @example MyPwm.pwm_set_polarity(PWM_POL_NORMAL);
 * @note    none.
 ********************************************************************************/
void ls_pwm::pwm_set_polarity(pwm_polarity_t _pola)
{
    if (_pola >= PWM_POL_INVALID)
    {
        std::cerr << "pwm_set_polarity: pola is invalid" << std::endl;
        return;
    }
    std::lock_guard<std::mutex> lock(this->mtx);
    this->pola = _pola;
    // 设置极性
    if (this->pola == PWM_POL_NORMAL)
    {
        ls_writel(this->pwm_ctrl, ls_readl(this->pwm_ctrl) & ~LS_PWM_CTRL_INVERT);
    }
    else if (this->pola == PWM_POL_INV)
    {
        ls_writel(this->pwm_ctrl, ls_readl(this->pwm_ctrl) | LS_PWM_CTRL_INVERT);
    }
}

/********************************************************************************
 * @brief   设置 PWM 周期.
 * @param   _period : PWM 周期.
 * @return  none.
 * @example MyPwm.pwm_set_period(50);
 * @note    none.
 ********************************************************************************/
void ls_pwm::pwm_set_period(uint32_t _period)
{
    // 周期值不可以大于 PWM 控制器时钟周期且不可以小于 0
    if (_period > PWM_CLK_FRE || _period < 0)
    {
        std::cerr << "pwm_set_period: period is invalid" << std::endl;
        return;
    }
    std::lock_guard<std::mutex> lock(this->mtx);
    this->period = _period;
    // 重新设置周期前最好先关闭使能位
    this->pwm_disable();
    uint32_t val = PWM_CLK_FRE / this->period - 1;
    // 设置周期
    ls_writel(this->pwm_full_buf, val);
    // 设置完周期后再打开使能
    this->pwm_enable();
}

/********************************************************************************
 * @brief   设置 PWM 占空比值.
 * @param   _duty : PWM 占空比值.
 * @return  none.
 * @example MyPwm.pwm_set_duty(2000);
 * @note    none.
 ********************************************************************************/
void ls_pwm::pwm_set_duty(uint32_t _duty)
{
    if (_duty > PWM_DUTY_MAX || _duty < 0)
    {
        std::cerr << "pwm_set_duty: duty is invalid" << std::endl;
        return;
    }
    std::lock_guard<std::mutex> lock(this->mtx);
    this->duty = _duty;
    this->pwm_enable();
    uint32_t val = PWM_CLK_FRE / this->period * this->duty / 10000;
    // 设置脉冲宽度
    ls_writel(this->pwm_low_buf, val);
    // 设置完脉冲宽度后再打开使能
    this->pwm_enable();
}

/********************************************************************************
 * @brief   使能 PWM.
 * @return  none.
 * @example MyPwm.pwm_enable();
 * @note    none.
 ********************************************************************************/
void ls_pwm::pwm_enable(void)
{
    std::lock_guard<std::mutex> lock(this->enable_mtx);
    ls_writel(this->pwm_ctrl, ls_readl(this->pwm_ctrl) | LS_PWM_CTRL_EN);
}

/********************************************************************************
 * @brief   禁用 PWM.
 * @return  none.
 * @example MyPwm.pwm_disable();
 * @note    none.
 ********************************************************************************/
void ls_pwm::pwm_disable(void)
{
    std::lock_guard<std::mutex> lock(this->enable_mtx);
    ls_writel(this->pwm_ctrl, ls_readl(this->pwm_ctrl) & ~LS_PWM_CTRL_EN);
}

/********************************************************************************
 * @brief   拷贝构造函数.
 * @param   other : 要拷贝的 PWM 实例.
 * @return  none.
 * @example ls_pwm MyPwm2(MyPwm);
 * @note    none.
 ********************************************************************************/
ls_pwm::ls_pwm(const ls_pwm& other)
{
    // 加锁保护, 避免并发赋值时的竞态
    std::lock_guard<std::mutex> lock_this(this->mtx);
    std::lock_guard<std::mutex> lock_other(other.mtx);
    // 拷贝所有硬件资源
    this->gpio   = other.gpio;
    this->ch     = other.ch;
    this->mux    = other.mux;
    this->pola   = other.pola;
    this->period = other.period;
    this->duty   = other.duty;
    this->pwm_base     = other.pwm_base;
    this->pwm_low_buf  = other.pwm_low_buf;
    this->pwm_full_buf = other.pwm_full_buf;
    this->pwm_ctrl     = other.pwm_ctrl;
    // 共享引用计数(多个变量指向同一个计数)
    this->ref_count = other.ref_count;
    (*this->ref_count)++;
}

/********************************************************************************
 * @brief   拷贝赋值运算符重载.
 * @param   other : 要拷贝的 PWM 实例.
 * @return  none.
 * @example ls_pwm MyPwm2 = MyPwm;
 * @note    none.
 ********************************************************************************/
ls_pwm& ls_pwm::operator=(const ls_pwm& other)
{
    if (this == &other) return *this;   // 防止自赋值
    // 加锁保护, 避免并发赋值时的竞态
    std::lock_guard<std::mutex> lock_this(this->mtx);
    std::lock_guard<std::mutex> lock_other(other.mtx);
    // 拷贝所有硬件资源
    this->gpio   = other.gpio;
    this->ch     = other.ch;
    this->mux    = other.mux;
    this->pola   = other.pola;
    this->period = other.period;
    this->duty   = other.duty;
    this->pwm_base     = other.pwm_base;
    this->pwm_low_buf  = other.pwm_low_buf;
    this->pwm_full_buf = other.pwm_full_buf;
    this->pwm_ctrl     = other.pwm_ctrl;
    // 共享引用计数(多个变量指向同一个计数)
    this->ref_count = other.ref_count;
    (*this->ref_count)++;
    return *this;
}

/********************************************************************************
 * @brief   析构函数.
 * @param   none.
 * @return  none.
 * @note    变量生命周期结束时, 自动调用析构函数, 释放 PWMx 控制器基地址映射.
 ********************************************************************************/
ls_pwm::~ls_pwm()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    this->pwm_disable();
    if (this->ref_count && --(*this->ref_count) == 0)
    {
        LQ::ls_addr_munmap(this->pwm_base);
    }
    this->pwm_base    = this->pwm_full_buf = nullptr;
    this->pwm_low_buf = this->pwm_ctrl     = nullptr;
    this->gpio = PIN_INVALID;
    this->mux  = GPIO_MUX_INVALID;
    this->ch   = PWM_CH_INVALID;
    this->pola = PWM_POL_INVALID;
    this->period = this->duty = 0;
}

/********************************************************************************
 * @brief   获取当前 PWM 所使用引脚.
 * @return  返回获取到的引脚值.
 * @example MyPwm.pwm_get_gpio();
 * @note    none.
 ********************************************************************************/
gpio_pin_t ls_pwm::pwm_get_gpio(void)
{
    return this->gpio;
}

/********************************************************************************
 * @brief   获取当前 PWM 所使用引脚的复用模式.
 * @return  返回获取到的复用模式值.
 * @example MyPwm.pwm_get_mux();
 * @note    none.
 ********************************************************************************/
gpio_mux_mode_t ls_pwm::pwm_get_mux(void)
{
    return this->mux;
}

/********************************************************************************
 * @brief   获取当前 PWM 所使用通道.
 * @return  返回获取到的通道值.
 * @example MyPwm.pwm_get_channel();
 * @note    none.
 ********************************************************************************/
pwm_channel_t ls_pwm::pwm_get_channel(void)
{
    return this->ch;
}

/********************************************************************************
 * @brief   获取当前 PWM 极性.
 * @return  返回获取到的极性值.
 * @example MyPwm.pwm_get_polarity();
 * @note    none.
 ********************************************************************************/
pwm_polarity_t ls_pwm::pwm_get_polarity(void)
{
    return this->pola;
}

/********************************************************************************
 * @brief   获取当前 PWM 周期.
 * @return  返回获取到的周期值.
 * @example MyPwm.pwm_get_period();
 * @note    none.
 ********************************************************************************/
uint32_t ls_pwm::pwm_get_period(void)
{
    return this->period;
}

/********************************************************************************
 * @brief   获取当前 PWM 占空比值.
 * @return  返回获取到的占空比值.
 * @example MyPwm.pwm_get_duty();
 * @note    none.
 ********************************************************************************/
uint32_t ls_pwm::pwm_get_duty(void)
{
    return this->duty;
}
