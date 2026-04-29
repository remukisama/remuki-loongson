#ifndef __LQ_REG_ADC_HPP
#define __LQ_REG_ADC_HPP

#include <mutex>
#include "lq_map_addr.hpp"

#include "lq_common.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

/****************************** ADC 硬件参数宏定义 ******************************/

// 寄存器偏移地址（与硬件手册对应）
#define ADC_SR_OFFSET               ( 0x00 )    // 状态寄存器（Status Register）
#define ADC_CR1_OFFSET              ( 0x04 )    // 控制寄存器1（Control Register 1）
#define ADC_CR2_OFFSET              ( 0x08 )    // 控制寄存器2（Control Register 2）
#define ADC_SMPR2_OFFSET            ( 0x10 )    // 采样时间寄存器2（通道0~9）
#define ADC_SQR3_OFFSET             ( 0x34 )    // 规则序列寄存器3（通道选择）
#define ADC_DR_OFFSET               ( 0x4C )    // 数据寄存器（Data Register）
// 采样时间配置（所有通道共用64个ADC时钟周期，兼顾精度和速度）
#define ADC_SAMPLE_TIME             ( 0x06 )    // 对应 ADC_SampleTime_64Cycles
// ADC 硬件属性
#define ADC_RESOLUTION              ( 4096 )    // 12位ADC分辨率（输出范围0~4095）
#define ADC_REF_VOLTAGE             ( 1800 )    // 参考电压1.8V（单位：mV）
#define ADC_CONV_TIMEOUT            ( 10000 )   // 转换超时计数（防止死循环）

/****************************************************************************************************
 * @brief   枚举值定义
 ****************************************************************************************************/

/* ADC 通道枚举, 请勿修改 */
typedef enum ls_adc_channel
{
    LS_ADC_CH0 = 0x00,    // 通道0
    LS_ADC_CH1,           // 通道1
    LS_ADC_CH2,           // 通道2
    LS_ADC_CH3,           // 通道3
    LS_ADC_CH4,           // 通道4
    LS_ADC_CH5,           // 通道5
    LS_ADC_CH6,           // 通道6
    LS_ADC_CH7,           // 通道7
    LS_ADC_CH_INVALID,     // 无效通道
} ls_adc_channel_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

/****************************** 单例硬件管理类声明（管理共享资源） ******************************/
/* 单例核心：禁止外部构造、拷贝、赋值(确保唯一实例), 内部调用 */
class ls_adc_sing_mgmt
{
public:
    /* 单例实例获取：静态局部变量, 确保全局唯一(首次调用时初始化) */
    static ls_adc_sing_mgmt& get_instance(void);

    /* 初始化 ADC 硬件寄存器(寄存器映射+基础配置+校准, 仅执行一次) */
    bool init_hardware(void);

    /* 读取指定通道的原始 ADC 值(未经转换) */
    int read_channel_raw(ls_adc_channel_t ch);

private:
    ls_adc_sing_mgmt(void);     // 私有构造函数
    ~ls_adc_sing_mgmt(void);    // 私有析构函数

    ls_adc_sing_mgmt(const ls_adc_sing_mgmt&) = delete;             // 禁止拷贝构造
    ls_adc_sing_mgmt& operator=(const ls_adc_sing_mgmt&) = delete;  // 禁止赋值

    /* 内部函数：ADC 硬件校准(必须步骤) */
    bool hard_calibrate(void);

    /* 内部函数：切换当前采样通道(动态配置通道参数) */
    void switch_smap_channel(ls_adc_channel_t ch);

    /* 内部函数：配置寄存器映射, 释放内存资源 */
    void unmap_registers(void);

private:
    bool is_inited;     // 硬件是否已初始化(标记位)
    std::mutex mtx;     // 互斥锁, 用于保护 ADC 寄存器访问

private:
    // ADC 控制寄存器基地址
    static const ls_reg_base_t LS_ADC_BASE_ADDR = 0x1611c000;

    ls_reg32_addr_t adc_base;   // 映射基地址
    ls_reg32_addr_t adc_sr;     // 状态寄存器
    ls_reg32_addr_t adc_cr1;    // 控制寄存器1
    ls_reg32_addr_t adc_cr2;    // 控制寄存器2
    ls_reg32_addr_t adc_smpr2;  // 采样时间寄存器2
    ls_reg32_addr_t adc_sqr3;   // 规则序列寄存器3
    ls_reg32_addr_t adc_dr;     // 数据寄存器
};

/****************************** 轻量级通道实例类声明（仅关联通道号） ******************************/
/* 外部调用 */
class ls_adc
{
public:
    // 构造函数：创建指定通道的 ADC 实例, 关联通道号, 初始化共享硬件实例
    explicit ls_adc(ls_adc_channel_t _ch);

    // 读取指定通道的原始 ADC 值
    int read_raw(void);

    // 读取 ADC 电压值(单位：V, 自动计算)
    float read_voltage(void);

    // 显示声明, 编译器自动生成实现, 无需源文件代码
    ls_adc(const ls_adc& other) = default;
    ls_adc& operator=(const ls_adc& other) = default;

private:
    ls_adc_channel_t ch;        // 仅存储当前通道号(不管理硬件)
    bool             is_valid;  // 实例是否有效(标记硬件初始化结果)
};

#endif