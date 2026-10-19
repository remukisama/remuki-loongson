#include "lq_reg_adc.hpp"

/****************************** 单例硬件管理类(ls_adc_sing_mgmt)实现 ******************************/

/********************************************************************************
 * @fn      ls_adc_sing_mgmt::get_instance(void);
 * @brief   获取单例实例, 静态局部变量, 确保全局唯一(首次调用时初始化).
 * @param   none.
 * @return  静态单例实例引用.
 * @date    2026-02-04.
 ********************************************************************************/
ls_adc_sing_mgmt &ls_adc_sing_mgmt::get_instance(void)
{
    static ls_adc_sing_mgmt instance;
    return instance;
}

/********************************************************************************
 * @fn      ls_adc_sing_mgmt::init_hardware(void);
 * @brief   初始化 ADC 硬件寄存器(寄存器映射+基础配置+校准, 仅执行一次).
 * @param   none.
 * @return  true: 初始化成功; false: 初始化失败.
 * @date    2026-02-04.
 ********************************************************************************/
bool ls_adc_sing_mgmt::init_hardware(void)
{
    // 加锁, 确保线程安全访问
    std::lock_guard<std::mutex> lock(mtx);
    if (this->is_inited)
    {
        // 增加 Dummy 转换(任意通道均可, 这里使用通道0), 消耗首次全局延迟
        this->switch_smap_channel(LS_ADC_CH0);
        ls_writel(this->adc_sr,  ls_readl(this->adc_sr)  & (1 << 1));   // 清除EOC标志
        ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) | (1 << 22));  // 软件触发转换
        // 等待 Dummy 转换完成(清除EOF标志, 避免影响后续采集)
        int dummy_timeout = 0;
        while (((ls_readl(this->adc_sr) & (1 << 1)) == 0) && (dummy_timeout < ADC_CONV_TIMEOUT * 2))
        {
            usleep(1);
            dummy_timeout++;
        }
        // 必须读取 DR 寄存器(清除EOF标志, 避免影响后续采集)
        if (dummy_timeout < ADC_CONV_TIMEOUT * 2)
        {
            ls_readl(this->adc_dr); // 丢弃 Dummy 转换结果, 仅清除EOF标志
        }
        else
        {
            std::clog << "ls_adc_sing_mgmt::init_hardware: Dummy 转换超时, 硬件可能异常" << std::endl;
        }
        return true;    // 已经初始化, 直接返回
    }
    this->adc_base  = LQ::ls_addr_mmap(LS_ADC_BASE_ADDR);
    this->adc_sr    = ls_reg_addr_calc(this->adc_base, ADC_SR_OFFSET);
    this->adc_cr1   = ls_reg_addr_calc(this->adc_base, ADC_CR1_OFFSET);
    this->adc_cr2   = ls_reg_addr_calc(this->adc_base, ADC_CR2_OFFSET);
    this->adc_smpr2 = ls_reg_addr_calc(this->adc_base, ADC_SMPR2_OFFSET);
    this->adc_sqr3  = ls_reg_addr_calc(this->adc_base, ADC_SQR3_OFFSET);
    this->adc_dr    = ls_reg_addr_calc(this->adc_base, ADC_DR_OFFSET);
    // 配置CR1：独立模式、关闭扫描、禁用中断
    ls_writel(this->adc_cr1, 0);                                        // 清空CR1
    ls_writel(this->adc_cr1, ls_readl(this->adc_cr1) & ~(1 << 8));      // SCAN=0（单通道模式）
    ls_writel(this->adc_cr1, ls_readl(this->adc_cr1) & ~(1 << 9));      // EOCIE=0（禁用转换完成中断）
    ls_writel(this->adc_cr1, ls_readl(this->adc_cr1) & ~(1 << 7));      // JEOCIE=0（禁用插入通道中断）
    // 配置CR2：右对齐、单次转换、软件触发、禁用DMA
    ls_writel(this->adc_cr2, 0);
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) & ~(1 << 11));     // ALIGN=0（数据右对齐，取低12位）
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) & ~(1 << 1));      // CONT=0（单次转换，不循环）
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) | (1 << 20));      // EXTTRIG=1（允许外部触发）
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) | (0x0E << 17));   // EXTSEL=0x0E（软件触发转换）
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) & ~(1 << 8));      // DMA=0（禁用DMA，简化操作）
    // 硬件校准
    if (!this->hard_calibrate())
    {
        this->unmap_registers();
        return false;
    }
    this->is_inited = true;
    return true;
}

/********************************************************************************
 * @fn      ls_adc_sing_mgmt::read_channel_raw(ls_adc_channel_t ch);
 * @brief   读取指定通道的原始 ADC 值(未经转换).
 * @param   ch : 要读取的 ADC 通道.
 * @return  成功读取的原始 ADC 值, 失败返回负数.
 * @date    2026-02-04.
 ********************************************************************************/
int ls_adc_sing_mgmt::read_channel_raw(ls_adc_channel_t ch)
{
    // 加锁, 确保线程安全访问
    std::lock_guard<std::mutex> lock(mtx);
    // 检查硬件状态和通道合法性
    if (!this->is_inited)
    {
        std::cerr << "ls_adc_sing_mgmt::read_channel_raw: hardware not initialized" << std::endl;
        return -1;
    }
    if (ch >= LS_ADC_CH_INVALID)
    {
        std::cerr << "ls_adc_sing_mgmt::read_channel_raw: invalid channel" << ch << std::endl;
        return -2;
    }
    // 切换到目标通道
    this->switch_smap_channel(ch);
    // 触发 ADC 转换
    ls_writel(this->adc_sr,  ls_readl(this->adc_sr)  & ~(1 << 1));  // 清除EOC标志(避免上一次转换的残留)
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) |  (1 << 22)); // 置位SWSTART, 触发软件转换
    // 等待转换完成(EOF标志位置1), 带超时保护
    int timeout = 0;
    while ((((ls_readl(this->adc_sr) & (1 << 1)) == 0) && (timeout < ADC_CONV_TIMEOUT)))
    {
        usleep(1);  // 微秒级等待，降低CPU占用
        timeout++;
    }

    // 处理超时或读取数据
    if (timeout >= ADC_CONV_TIMEOUT)
    {
        std::cerr << "ls_adc_sing_mgmt::read_channel_raw: timeout, channel" << ch << std::endl;
        return -3;
    }
    // 读取数据寄存器，仅保留低12位有效数据
    return (ls_readl(this->adc_dr) & 0x0FFF);
}

/********************************************************************************
 * @fn      ls_adc_sing_mgmt::ls_adc_sing_mgmt();
 * @brief   私有构造：初始化成员变量（仅调用一次）.
 * @param   none.
 * @return  none.
 * @date    2026-02-04.
 ********************************************************************************/
ls_adc_sing_mgmt::ls_adc_sing_mgmt() : is_inited(false), adc_base(nullptr), adc_sr(nullptr),
    adc_cr1(nullptr), adc_cr2(nullptr), adc_smpr2(nullptr), adc_sqr3(nullptr), adc_dr(nullptr)
{
    // 构造时仅初始化变量, 不执行硬件操作(延迟到init_hardware()中)
}

/********************************************************************************
 * @fn      ls_adc_sing_mgmt::~ls_adc_sing_mgmt();
 * @brief   私有析构：释放内存资源.
 * @param   none.
 * @return  none.
 * @date    2026-02-04.
 ********************************************************************************/
ls_adc_sing_mgmt::~ls_adc_sing_mgmt()
{
    this->unmap_registers();
}

/********************************************************************************
 * @fn      ls_adc_sing_mgmt::hard_calibrate(void);
 * @brief   ADC硬件校准（必需步骤，否则采样值不准）.
 * @param   none.
 * @return  bool: 成功返回true，失败返回false.
 * @date    2026-02-04.
 ********************************************************************************/
bool ls_adc_sing_mgmt::hard_calibrate(void)
{
    if (this->adc_cr2 == NULL)
    {
        std::cerr << "ls_adc_sing_mgmt::hard_calibrate: adc_cr2 not mapped" << std::endl;
        return false;
    }
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) | (1 << 0)); // 使能ADC（ADON位，需两次置1激活，部分硬件要求）
    usleep(20);                                                   // 等待ADC稳定（20微秒）
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) | (1 << 0)); // 使能ADC（ADON位，需两次置1激活，部分硬件要求）
    usleep(20);
    // 重置校准（RSTCAL位）
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) | (1 << 3));
    usleep(10);
    // 等待复位校准完成（RSTCAL位：bit3，硬件完成后自动清0）
    while ((ls_readl(this->adc_cr2) & (1 << 3)) != 0) { usleep(1); }    // 微秒级等待，降低CPU占用（可选，避免CPU空转）
    // 开始校准（CAL位）
    ls_writel(this->adc_cr2, ls_readl(this->adc_cr2) | (1 << 2));
    usleep(10);
    // 等待ADC校准完成（CAL位：bit2，硬件完成后自动清0）
    while ((ls_readl(this->adc_cr2) & (1 << 2)) != 0) { usleep(1); }    // 同上，建议保留
    // 校准后稳定时间
    usleep(20); 
    return true;
}

/********************************************************************************
 * @fn      ls_adc_sing_mgmt::switch_smap_channel(ls_adc_channel_t ch);
 * @brief   切换当前采样通道（动态配置通道参数）.
 * @param   ch : 目标通道编号, 参考 ls_adc_channel_t 枚举值.
 * @return  none.
 * @date    2026-02-04.
 ********************************************************************************/
void ls_adc_sing_mgmt::switch_smap_channel(ls_adc_channel_t ch)
{
    // 配置目标通道的采样时间（64个ADC时钟周期）
    ls_writel(this->adc_smpr2, ls_readl(this->adc_smpr2) & ~(0x07 << (3 * ch)));            // 清除该通道原有采样时间
    ls_writel(this->adc_smpr2, ls_readl(this->adc_smpr2) | (ADC_SAMPLE_TIME << (3 * ch)));  // 设置新采样时间
    // 配置规则序列：仅采样当前通道（SQ1=目标通道，序列长度=1
    ls_writel(this->adc_sqr3, ls_readl(this->adc_sqr3) & ~(0x1F << 0));     // 清除SQ1（第一个转换通道）
    ls_writel(this->adc_sqr3, ls_readl(this->adc_sqr3) | (ch << 0));        // SQ1=目标通道
}

/********************************************************************************
 * @fn      ls_adc_sing_mgmt::unmap_registers(void);
 * @brief   解除寄存器映射，释放内存资源.
 * @param   none.
 * @return  none.
 * @date    2026-02-04
 ********************************************************************************/
void ls_adc_sing_mgmt::unmap_registers(void)
{
    if (this->adc_base != NULL)
    {
        LQ::ls_addr_munmap(this->adc_base);
    }
    this->adc_base = NULL;
    this->adc_sr = this->adc_cr1 = this->adc_cr2 = NULL;
    this->adc_smpr2 = this->adc_sqr3 = this->adc_dr = NULL;
}

/****************************** 轻量级通道实例类（ls_adc）实现 ******************************/

/********************************************************************************
 * @fn      ls_adc::ls_adc(ls_adc_channel_t _ch);
 * @brief   构造函数：创建指定通道的ADC实例，关联通道号，初始化共享硬件实例.
 * @param   _ch : 目标通道编号（ls_adc_channel_t 枚举值）.
 * @return  none.
 * @date    2026-02-04.
 ********************************************************************************/
ls_adc::ls_adc(ls_adc_channel_t _ch) : ch(_ch), is_valid(false)
{
    // 初始化共享硬件（仅首次调用时执行，后续实例直接复用）
    if (ls_adc_sing_mgmt::get_instance().init_hardware())
    {
        is_valid = true;
    }
    else
    {
        std::clog << "ls_adc: " << (int)ch << " already defined" << std::endl;
    }
}

/********************************************************************************
 * @fn      int ls_adc::read_raw();
 * @brief   读取指定通道的原始ADC值（调用共享硬件接口）.
 * @param   none.
 * @return  int: 成功返回原始值，失败或超时返回-1/-2/-3.
 * @date    2026-02-04.
 ********************************************************************************/
int ls_adc::read_raw()
{
    if (!is_valid)
    {
        std::clog << "ls_adc: " << (int)ch << "实例无效" << std::endl;
        return -1;
    }
    // 调用单例硬件的读取接口
    return ls_adc_sing_mgmt::get_instance().read_channel_raw(ch);
}

/********************************************************************************
 * @fn      float ls_adc::read_voltage();
 * @brief   读取指定通道的电压值（原始值转换为电压）.
 * @param   none.
 * @return  float: 成功返回电压值，失败或超时返回-1.0f.
 * @date    2026-02-04.
 ********************************************************************************/
float ls_adc::read_voltage()
{
    int raw_val = this->read_raw();
    if (raw_val < 0)
    {
        return -1.0f; // 读取失败，返回负电压标识
    }
    // 电压计算公式：(原始值 / 分辨率) * 参考电压（单位：V）
    return (float)raw_val * ADC_REF_VOLTAGE / ADC_RESOLUTION / 1000.0f;
}
