#include "lq_reg_gpio.hpp"
#include "lq_map_addr.hpp"

// 复用寄存器全局锁(保护多线程修改复用配置)
static std::mutex gpio_mux_mutex;

/********************************************************************************
 * @brief   配置 GPIO 复用.
 * @param   _gpio : 复用的 GPIO 引脚号, 参考 gpio_pin_t 枚举.
 * @param   _mux  : 复用的模式, 参考 gpio_mux_mode_t 枚举.
 * @brief   none.
 * @example gpio_mux_set(PIN_88, GPIO_MUX_GPIO);
 * @note    在配置第二个参数时，请参考《龙芯2k0300用户处理器手册》
 ********************************************************************************/
void gpio_mux_set(gpio_pin_t _gpio, gpio_mux_mode_t _mux)
{
    // 加锁保护
    std::lock_guard<std::mutex> lock(gpio_mux_mutex);
    // 参数校验
    if (_gpio >= PIN_INVALID || _mux >= GPIO_MUX_INVALID)
    {
        std::cerr << "gpio_mux_set: gpio or mux is invalid" << std::endl;
        return;
    }
    // 映射 GPIO 复用配置寄存器
    ls_reg32_addr_t GPIO_REUSE_REG = LQ::ls_addr_mmap(LS_GPIO_REUSE_ADDR + (_gpio / 16) * LS_GPIO_REUSE_OFS);
    ls_writel(GPIO_REUSE_REG, (ls_readl(GPIO_REUSE_REG) & ~(0b11 << (_gpio % 16 * 2))) | (_mux << (_gpio % 16 * 2)));
    LQ::ls_addr_munmap(GPIO_REUSE_REG);
}

/********************************************************************************
 * @brief   硬件配置 GPIO 的无参构造函数.
 * @param   none.
 * @return  none.
 * @example ls_gpio MyGpio;
 * @note    none.
 ********************************************************************************/
ls_gpio::ls_gpio() : gpio(PIN_INVALID), mode(GPIO_MODE_INVALID), mux(GPIO_MUX_INVALID),
    gpio_base(nullptr), gpio_one(nullptr), gpio_o(nullptr), gpio_i(nullptr), ref_count(std::make_shared<int>(0))
{
}
  
/********************************************************************************
 * @brief   硬件配置 GPIO 的有参构造函数.
 * @param   _gpio : 对应使用的引脚号, 参考 gpio_pin_t 枚举.
 * @param   _mode : GPIO 模式, 参考 gpio_mode_t 枚举.
 * @param   _mux : GPIO 复用模式, 参考 gpio_mux_mode_t 枚举, 默认为 GPIO_MUX_GPIO.
 * @return  none.
 * @example ls_gpio MyGpio(PIN_88, GPIO_MODE_OUT);
 * @note    none.
 ********************************************************************************/
ls_gpio::ls_gpio(gpio_pin_t _gpio, gpio_mode_t _mode, gpio_mux_mode_t _mux) : gpio(_gpio), mode(_mode), mux(_mux),
    gpio_base(nullptr), gpio_one(nullptr), gpio_o(nullptr), gpio_i(nullptr), ref_count(std::make_shared<int>(1))
{
    // 参数校验
    if (_gpio  >= PIN_INVALID || _mode >= GPIO_MODE_INVALID || _mux >= GPIO_MUX_INVALID)
    {
        std::cerr << "ls_gpio: gpio, mode or mux is invalid" << std::endl;
        return;
    }
    // 初始化 GPIO 基地址, 并校验是否成功映射到虚拟地址
    this->gpio_base = LQ::ls_addr_mmap(LS_GPIO_BASE_ADDR);
    if (this->gpio_base == nullptr)
    {
        std::cerr << "ls_gpio: gpio_base is nullptr" << std::endl;
        return;
    }
    // 计算各寄存器地址
    this->gpio_one  = ls_reg_addr_calc(this->gpio_base, LS_GPIO_OEN_OFF(_gpio));
    this->gpio_o    = ls_reg_addr_calc(this->gpio_base, LS_GPIO_O_OFF(_gpio));
    this->gpio_i    = ls_reg_addr_calc(this->gpio_base, LS_GPIO_I_OFF(_gpio));
    // 配置引脚复用为 GPIO 模式
    gpio_mux_set(this->gpio, _mux);
    // 配置引脚模式
    this->gpio_direction_set(_mode);
}

/********************************************************************************
 * @brief   拷贝构造函数.
 * @param   other : 要拷贝的 GPIO 实例.
 * @return  none.
 * @example ls_gpio MyGpio2(MyGpio);
 * @note    none.
 ********************************************************************************/
ls_gpio::ls_gpio(const ls_gpio& other)
{
    // 加锁保护, 避免并发赋值时的竞态
    std::lock_guard<std::mutex> lock_this(this->mtx);
    std::lock_guard<std::mutex> lock_other(other.mtx);
    // 拷贝所有硬件资源
    this->gpio      = other.gpio;
    this->mode      = other.mode;
    this->mux       = other.mux;
    this->gpio_base = other.gpio_base;
    this->gpio_one  = other.gpio_one;
    this->gpio_o    = other.gpio_o;
    this->gpio_i    = other.gpio_i;
    // 共享引用计数(多个变量指向同一个计数)
    this->ref_count = other.ref_count;
    (*this->ref_count)++;
}

/********************************************************************************
 * @brief   赋值运算符重载.
 * @param   other : 要拷贝的 GPIO 实例.
 * @return  none.
 * @example ls_gpio MyGpio2 = MyGpio;
 * @note    none.
 ********************************************************************************/
ls_gpio& ls_gpio::operator=(const ls_gpio& other)
{
    if (this == &other) return *this;   // 防止自赋值
    // 加锁保护, 避免并发赋值时的竞态
    std::lock_guard<std::mutex> lock_this(this->mtx);
    std::lock_guard<std::mutex> lock_other(other.mtx);
    // 拷贝所有硬件资源
    this->gpio      = other.gpio;
    this->mode      = other.mode;
    this->mux       = other.mux;
    this->gpio_base = other.gpio_base;
    this->gpio_one  = other.gpio_one;
    this->gpio_o    = other.gpio_o;
    this->gpio_i    = other.gpio_i;
    // 共享引用计数(多个变量指向同一个计数)
    this->ref_count = other.ref_count;
    (*this->ref_count)++;
    return *this;
}

/********************************************************************************
 * @brief   硬件配置 GPIO 的模式.
 * @param   _mode : GPIO 模式.
 * @return  none.
 * @example MyGpio.gpio_direction_set(GPIO_MODE_OUT);
 * @note    none.
 ********************************************************************************/
void ls_gpio::gpio_direction_set(gpio_mode_t _mode)
{
    // 加锁保护, 防止并发访问
    std::lock_guard<std::mutex> lock(this->mtx);
    // 参数校验
    if (_mode >= GPIO_MODE_INVALID)
    {
        std::cerr << "gpio_direction_set: mode is invalid" << std::endl;
        return;
    }
    if (this->gpio_one == nullptr)
    {
        std::cerr << "gpio_direction_set: gpio_one is nullptr" << std::endl;
        return;
    }
    // 设置 GPIO脚模式
    if(_mode == GPIO_MODE_IN){
        ls_writel(this->gpio_one, ls_readl(this->gpio_one) | BIT(this->gpio % 8));
    }else if(_mode == GPIO_MODE_OUT){
        ls_writel(this->gpio_one, ls_readl(this->gpio_one) & ~BIT(this->gpio % 8));
    }
}
  
/********************************************************************************
 * @brief   硬件配置 GPIO 的输出值.
 * @param   _val : GPIO 输出值.
 * @return  none.
 * @example MyGpio.gpio_level_set(GPIO_HIGH);
 * @note    none.
 ********************************************************************************/
void ls_gpio::gpio_level_set(gpio_level_t _val)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    // 参数校验
    if (_val >= GPIO_LEVEL_INVALID)
    {
        std::cerr << "gpio_level_set: value is invalid" << std::endl;
        return;
    }
    if (this->gpio_o == nullptr)
    {
        std::cerr << "gpio_level_set: gpio_o is nullptr" << std::endl;
        return;
    }
    // 设置 GPIO 输出值
    if(_val == GPIO_HIGH){
        ls_writel(this->gpio_o, ls_readl(this->gpio_o) | BIT(this->gpio % 8));
    }else if(_val == GPIO_LOW){
        ls_writel(this->gpio_o, ls_readl(this->gpio_o) & ~BIT(this->gpio % 8));
    }
}
  
/********************************************************************************
 * @brief   硬件配置 GPIO 的输出值.
 * @param   none.
 * @return  none.
 * @example int val = MyGpio.gpio_level_get();
 * @note    none.
 ********************************************************************************/
gpio_level_t ls_gpio::gpio_level_get()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    // 参数校验
    if (this->gpio_i == nullptr)
    {
        std::cerr << "gpio_level_get: gpio_i is nullptr" << std::endl;
        return GPIO_LEVEL_INVALID;
    }
    return (gpio_level_t)((ls_readl(this->gpio_i) & BIT(this->gpio % 8)) == BIT(this->gpio % 8));
}

/********************************************************************************
 * @brief   硬件配置 GPIO 的析构函数.
 * @param   none.
 * @return  none.
 * @example 创建的对象生命周期结束后系统自动调用.
 * @date    none.
 ********************************************************************************/
ls_gpio::~ls_gpio()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    // 仅当引用计数为 0 时, 才执行 munmap
    if (ref_count && --(*ref_count) == 0 && this->gpio_base != nullptr)
    {
        LQ::ls_addr_munmap(this->gpio_base);
    }
    this->gpio_base = this->gpio_one  = nullptr;
    this->gpio_o    = this->gpio_i    = nullptr;
    this->gpio      = PIN_INVALID;
    this->mode      = GPIO_MODE_INVALID;
    this->mux       = GPIO_MUX_INVALID;
}

/********************************************************************************
 * @brief   获取 GPIO 的引脚号.
 * @param   none.
 * @return  GPIO 引脚号.
 * @note    none.
 ********************************************************************************/
gpio_pin_t ls_gpio::gpio_get_pin()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->gpio;
}

/********************************************************************************
 * @brief   获取 GPIO 的模式.
 * @param   none.
 * @return  GPIO 模式.
 * @note    none.
 ********************************************************************************/
gpio_mode_t ls_gpio::gpio_get_mode()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->mode;
}

/********************************************************************************
 * @brief   获取 GPIO 的复用模式.
 * @param   none.
 * @return  GPIO 复用模式.
 * @note    none.
 ********************************************************************************/
gpio_mux_mode_t ls_gpio::gpio_get_mux()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->mux;
}
