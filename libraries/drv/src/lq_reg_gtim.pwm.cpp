#include "lq_reg_gtim_pwm.hpp"
#include "lq_map_addr.hpp"

// 创建一个互斥锁
static pthread_mutex_t GTIM_MUTEX = PTHREAD_MUTEX_INITIALIZER;

/********************************************************************************
 * @brief   硬件配置 GTIM PWM 的无参构造函数.
 * @param   none.
 * @return  none.
 * @example ls_gtim_pwm MyPwm;
 * @note    none.
 ********************************************************************************/
ls_gtim_pwm::ls_gtim_pwm() : gpio(PIN_INVALID), ch(GTIM_PWM_CH_INVALID), mux(GPIO_MUX_INVALID), pola(GTIM_PWM_POL_INVALID),
    period(0), duty(0), ref_count(std::make_shared<int>(0)), gtim_base(nullptr), gtim_arr(nullptr), gtim_ccrx(nullptr),
    gtim_ccer(nullptr), gtim_cnt(nullptr), gtim_egr(nullptr), gtim_cr1(nullptr)
{
    this->gtim_ccmr[0] = nullptr;
    this->gtim_ccmr[1] = nullptr;
}

/******************************************************************************** 
 * @brief   硬件配置 GTIM PWM 的有参构造函数.
 * @param   _pin    : 对应使用的引脚号, 参考 gtim_pwm_pin_t 枚举.
 * @param   _period : PWM 周期
 * @param   _duty   : PWM 占空比
 * @param   _pola   : PWM 极性, 参考 gtim_pwm_polarity_t 枚举, 默认为 GTTIM_PWM_POL_INV.
 * @return  none.
 * @example ls_gtim_pwm MyPwm(GTIM_PWM_PIN64, 50, 2000);
 * @note    该类设置周期时, 输入值是多少, 周期就是多少.
 *          例如: 要设置周期为 50Hz, 周期值填入 50 即可.
 *          设置占空比的话, 无论周期是多少, 占空比范围都为 0 - 10000
 ********************************************************************************/
ls_gtim_pwm::ls_gtim_pwm(gtim_pwm_pin_t _pin, uint32_t _period, uint32_t _duty, gtim_pwm_polarity_t _pola) :
    ref_count(std::make_shared<int>(1))
{
    this->gpio = (gpio_pin_t)((_pin) & 0xFF);
    this->ch   = (gtim_pwm_channel_t)((_pin>>8) & 0x03);
    this->mux  = (gpio_mux_mode_t)((_pin>>10) & 0x03);
    // 配置引脚复用
    gpio_mux_set(this->gpio, this->mux);
    // 获取 Gtim 基地址
    this->gtim_base = LQ::ls_addr_mmap(this->LS_GTIM_BASE_ADDR);
    if (this->gtim_base == nullptr)
    {
        std::cerr << "ls_gtim_pwm: gtim_base is nullptr" << std::endl;
        return;
    }
    // 计算各寄存器地址
    this->gtim_arr     = ls_reg_addr_calc(this->gtim_base, LS_GTIM_ARR);
    this->gtim_ccrx    = ls_reg_addr_calc(this->gtim_base, LS_GTIM_CCR1 + this->ch * LS_GTIM_CCRx_OFS);
    this->gtim_ccmr[0] = ls_reg_addr_calc(this->gtim_base, LS_GTIM_CCMR1);
    this->gtim_ccmr[1] = ls_reg_addr_calc(this->gtim_base, LS_GTIM_CCMR2);
    this->gtim_ccer    = ls_reg_addr_calc(this->gtim_base, LS_GTIM_CCER);
    this->gtim_cnt     = ls_reg_addr_calc(this->gtim_base, LS_GTIM_CNT);
    this->gtim_egr     = ls_reg_addr_calc(this->gtim_base, LS_GTIM_EGR);
    this->gtim_cr1     = ls_reg_addr_calc(this->gtim_base, LS_GTIM_CR1);
    // 初始化配置
    ls_writel(this->gtim_egr, 0x01);            // 初始化所有寄存器
    this->gtim_pwm_set_mode(GTIM_PWM_MODE_2);   // 设置 PWM 模式 2
    this->gtim_pwm_set_polarity(_pola);         // 设置 PWM 极性
    this->gtim_pwm_set_period(_period);         // 设置 PWM 周期
    this->gtim_pwm_set_duty(_duty);             // 设置 PWM 占空比
    ls_writel(this->gtim_cr1, 0b10000001);      // 启动计数器和预装载值
}

/********************************************************************************
 * @brief   设置 GTIM PWM 模式.
 * @param   _mode : GTIM PWM 模式, 参考 gtim_pwm_mode_t 枚举.
 * @return  none.
 * @example MyPwm.gtim_pwm_set_mode(GTIM_PWM_MODE_2);
 * @note    none
 ********************************************************************************/
void ls_gtim_pwm::gtim_pwm_set_mode(gtim_pwm_mode_t _mode)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    this->mode = _mode;
    // 配置所选通道的 PWM 模式
    uint32_t reg = ls_readl(this->gtim_ccmr[this->ch / 2]) & ~(0x07 << (this->ch % 2 * 8 + 4)) | (_mode << (this->ch % 2 * 8 + 4));
    // 输出模式
    reg &= ~(0x03 << this->ch % 2 * 8);
    // 开启预装载
    reg &= ~(1 << (this->ch % 2 * 8 + 3));
    reg |=  (1 << (this->ch % 2 * 8 + 3));
    ls_writel(this->gtim_ccmr[this->ch / 2], reg);
}

/********************************************************************************
 * @brief   设置 GTIM PWM 极性.
 * @param   _pola : GTIM PWM 极性, 参考 gtim_pwm_polarity_t 枚举.
 * @return  none.
 * @example MyPwm.gtim_pwm_set_polarity(GTIM_PWM_POL_NORMAL);
 * @note    none
 ********************************************************************************/
void ls_gtim_pwm::gtim_pwm_set_polarity(gtim_pwm_polarity_t _pola)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    this->pola = _pola;
    // 配置所选通道的 PWM 极性
    uint32_t reg = ls_readl(this->gtim_ccer) & ~(0x01 << (this->ch * 4 + 1)) | (_pola << (this->ch * 4 + 1));
    ls_writel(this->gtim_ccer, reg);
}

/********************************************************************************
 * @brief   设置 GTIM PWM 周期.
 * @param   _period : GTIM PWM 周期.
 * @return  none.
 * @example MyPwm.gtim_pwm_set_period(50);
 * @note    想要设置的值为多少, 填入多少即可.
 ********************************************************************************/
void ls_gtim_pwm::gtim_pwm_set_period(uint32_t _period)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    this->gtim_pwm_disable();
    this->period = _period;
    ls_writel(this->gtim_arr, GTIM_PWM_CLK_FRE / this->period - 1);
    this->gtim_pwm_enable();
}

/********************************************************************************
 * @brief   设置 GTIM PWM 占空比.
 * @param   _duty : GTIM PWM 占空比.
 * @return  none.
 * @example MyPwm.gtim_pwm_set_duty(5000);
 * @note    想要设置的值为多少, 填入多少即可, 范围为 0 - 10000.
 ********************************************************************************/
void ls_gtim_pwm::gtim_pwm_set_duty(uint32_t _duty)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    pthread_mutex_lock(&GTIM_MUTEX);
    this->duty = _duty;
    ls_writel(this->gtim_ccrx, GTIM_PWM_CLK_FRE / this->period * this->duty / GTIM_PWM_DUTY_MAX);
    pthread_mutex_unlock(&GTIM_MUTEX);
}

/********************************************************************************
 * @brief   使能 GTIM PWM 通道.
 * @return  none.
 * @example MyPwm.gtim_pwm_enable();
 * @note    none
 ********************************************************************************/
void ls_gtim_pwm::gtim_pwm_enable(void)
{
    std::lock_guard<std::mutex> lock(this->enable_mtx);
    ls_writel(this->gtim_ccer, ls_readl(this->gtim_ccer) | (0x01 << (this->ch * 4 + 0)));
}

/********************************************************************************
 * @brief   失能 GTIM PWM 通道.
 * @return  none.
 * @example MyPwm.gtim_pwm_disable();
 * @note    none
 ********************************************************************************/
void ls_gtim_pwm::gtim_pwm_disable(void)
{
    std::lock_guard<std::mutex> lock(this->enable_mtx);
    ls_writel(this->gtim_ccer, ls_readl(this->gtim_ccer) & ~(0x01 << (this->ch * 4 + 0)));
}

/********************************************************************************
 * @brief   拷贝构造函数.
 * @param   other : 要拷贝的 GTIM PWM 实例.
 * @return  none.
 * @example ls_gtim_pwm MyPwm2(MyPwm);
 * @note    none.
 ********************************************************************************/
ls_gtim_pwm::ls_gtim_pwm(const ls_gtim_pwm& other)
{
    // 加锁保护, 避免并发赋值时的竞态
    std::lock_guard<std::mutex> lock_this(this->mtx);
    std::lock_guard<std::mutex> lock_other(other.mtx);
    // 拷贝所有硬件资源
    this->gpio   = other.gpio;
    this->mux    = other.mux;
    this->ch     = other.ch;
    this->pola   = other.pola;
    this->mode   = other.mode;
    this->period = other.period;
    this->duty   = other.duty;
    // 寄存器资源
    this->gtim_base    = other.gtim_base;
    this->gtim_arr     = other.gtim_arr;
    this->gtim_ccrx    = other.gtim_ccrx;
    this->gtim_ccmr[0] = other.gtim_ccmr[0];
    this->gtim_ccmr[1] = other.gtim_ccmr[1];
    this->gtim_cnt     = other.gtim_cnt;
    this->gtim_egr     = other.gtim_egr;
    this->gtim_cr1     = other.gtim_cr1;
    // 共享引用计数(多个变量指向同一个计数)
    this->ref_count = other.ref_count;
    (*this->ref_count)++;
}

/********************************************************************************
 * @brief   拷贝赋值运算符重载.
 * @param   other : 要拷贝的 GTIM PWM 实例.
 * @return  none.
 * @example ls_gtim_pwm MyPwm2 = MyPwm;
 * @note    none.
 ********************************************************************************/
ls_gtim_pwm& ls_gtim_pwm::operator=(const ls_gtim_pwm& other)
{
    if (this == &other) return *this;   // 防止自赋值
    // 加锁保护, 避免并发赋值时的竞态
    std::lock_guard<std::mutex> lock_this(this->mtx);
    std::lock_guard<std::mutex> lock_other(other.mtx);
    // 拷贝所有硬件资源
    this->gpio   = other.gpio;
    this->mux    = other.mux;
    this->ch     = other.ch;
    this->pola   = other.pola;
    this->mode   = other.mode;
    this->period = other.period;
    this->duty   = other.duty;
    // 寄存器资源
    this->gtim_base    = other.gtim_base;
    this->gtim_arr     = other.gtim_arr;
    this->gtim_ccrx    = other.gtim_ccrx;
    this->gtim_ccmr[0] = other.gtim_ccmr[0];
    this->gtim_ccmr[1] = other.gtim_ccmr[1];
    this->gtim_cnt     = other.gtim_cnt;
    this->gtim_egr     = other.gtim_egr;
    this->gtim_cr1     = other.gtim_cr1;
    // 共享引用计数(多个变量指向同一个计数)
    this->ref_count = other.ref_count;
    (*this->ref_count)++;
    return *this;
}

/********************************************************************************
 * @brief   析构函数.
 * @param   none.
 * @return  none.
 * @note    变量生命周期结束时, 自动调用析构函数, 释放 Gtim 控制器基地址映射.
 ********************************************************************************/
ls_gtim_pwm::~ls_gtim_pwm()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    this->gtim_pwm_disable();
    if (this->ref_count && --(*this->ref_count) == 0)
    {
        LQ::ls_addr_munmap(this->gtim_base);
    }
    this->gtim_base    = this->gtim_arr     = nullptr;
    this->gtim_ccrx    = this->gtim_ccmr[0] = nullptr;
    this->gtim_ccmr[1] = this->gtim_ccer    = nullptr;
    this->gtim_cnt     = this->gtim_egr     = nullptr;
    this->gtim_cr1     = nullptr;
    this->gpio = PIN_INVALID;
    this->mux  = GPIO_MUX_INVALID;
    this->ch   = GTIM_PWM_CH_INVALID;
    this->pola = GTIM_PWM_POL_INVALID;
    this->mode = GTIM_PWM_MODE_INVALID;
    this->period = 0;
    this->duty   = 0;
}

/********************************************************************************
 * @brief   获取当前 GTIM PWM 所使用引脚.
 * @return  返回获取到的引脚值.
 * @example MyPwm.gtim_pwm_get_gpio();
 * @note    none.
 ********************************************************************************/
gpio_pin_t ls_gtim_pwm::gtim_pwm_get_gpio(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->gpio;
}

/********************************************************************************
 * @brief   获取当前 GTIM PWM 所使用引脚的复用模式.
 * @return  返回获取到的复用模式值.
 * @example MyPwm.gtim_pwm_get_mux();
 * @note    none.
 ********************************************************************************/
gpio_mux_mode_t ls_gtim_pwm::gtim_pwm_get_mux(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->mux;
}

/********************************************************************************
 * @brief   获取当前 GTIM PWM 所使用通道.
 * @return  返回获取到的通道值.
 * @example MyPwm.gtim_pwm_get_channel();
 * @note    none.
 ********************************************************************************/
gtim_pwm_channel_t ls_gtim_pwm::gtim_pwm_get_channel(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->ch;
}

/********************************************************************************
 * @brief   获取当前 GTIM PWM 极性.
 * @return  返回获取到的极性值.
 * @example MyPwm.gtim_pwm_get_polarity();
 * @note    none.
 ********************************************************************************/
gtim_pwm_polarity_t ls_gtim_pwm::gtim_pwm_get_polarity(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->pola;
}

/********************************************************************************
 * @brief   获取当前 GTIM PWM 周期.
 * @return  返回获取到的周期值.
 * @example MyPwm.gtim_pwm_get_period();
 * @note    none.
 ********************************************************************************/
uint32_t ls_gtim_pwm::gtim_pwm_get_period(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->period;
}

/********************************************************************************
 * @brief   获取当前 GTIM PWM 占空比.
 * @return  返回获取到的占空比值.
 * @example MyPwm.gtim_pwm_get_duty();
 * @note    none.
 ********************************************************************************/
uint32_t ls_gtim_pwm::gtim_pwm_get_duty(void)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->duty;
}
