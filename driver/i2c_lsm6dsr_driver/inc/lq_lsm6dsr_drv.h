#ifndef __LQ_MPU6050_DRI_H__
#define __LQ_MPU6050_DRI_H__ 

#include "lq_drv_common.h"

//****************************************
// 定义LSM6DSR内部地址
//****************************************
typedef enum
{
    LSM6DSR_AD0         = 0,        // 根据 AD0 引脚电平，选择是 低电平为0，高电平为 1

    LSM6DSR_BASE_ADDR   = 0x35,
    LSM6DSR_ADDR        = (LSM6DSR_BASE_ADDR << 1) + LSM6DSR_AD0,  // IIC通信设备 写地址 读则加一

    LSM6DSR_DRV_ID      = 0x6B,     //设备ID
    LSM6DSR_WHO_AM      = 0x0F,     //设备ID寄存器

    LSM6DSR_FUNC_CFG    = 0X01,     //控制寄存器
    LSM6DSR_INT1_CTRL   = 0X0D,
    LSM6DSR_INT2_CTRL   = 0X0E,
    // 加速度计控制寄存器1 (r/w) bit1 : 0 -> 一级数字滤波输出. 1 -> LPF2第二级滤波输出
    // bit[2:3] : 加速度计量程选择, 默认为 00 -> ±2g, 01 -> ±16g, 10 -> ±4g, 11 -> ±8g
    LSM6DSR_CTRL1_XL    = 0X10,
    LSM6DSR_CTRL2_G     = 0X11,
    LSM6DSR_CTRL3_C     = 0X12,
    LSM6DSR_CTRL4_C     = 0X13,
    LSM6DSR_CTRL5_C     = 0X14,
    LSM6DSR_CTRL6_C     = 0X15,
    LSM6DSR_CTRL7_G     = 0X16,
    LSM6DSR_CTRL8_XL    = 0X17,
    LSM6DSR_CTRL9_XL    = 0X18,
    LSM6DSR_CTRL10_C    = 0X19,

    LSM6DSR_STATUS_REG  = 0X1E,
    
    LSM6DSR_OUT_TEMP_L  = 0X20,
    LSM6DSR_OUT_TEMP_H  = 0X21,

    LSM6DSR_OUTX_L_GYRO = 0X22,
    LSM6DSR_OUTX_H_GYRO = 0X23,
    LSM6DSR_OUTY_L_GYRO = 0X24,
    LSM6DSR_OUTY_H_GYRO = 0X25,
    LSM6DSR_OUTZ_L_GYRO = 0X26,
    LSM6DSR_OUTZ_H_GYRO = 0X27,

    LSM6DSR_OUTX_L_ACC  = 0X28,
    LSM6DSR_OUTX_H_ACC  = 0X29,
    LSM6DSR_OUTY_L_ACC  = 0X2A,
    LSM6DSR_OUTY_H_ACC  = 0X2B,
    LSM6DSR_OUTZ_L_ACC  = 0X2C,
    LSM6DSR_OUTZ_H_ACC  = 0X2D,

    LSM6DSR_I3C_BUS_AVB = 0x62,
    PROPERTY_ENABLE     = 1U,
    PROPERTY_DISABLE    = 0U,
} lsm6dsr_reg_addr_t;

// 寄存器设置的值
typedef enum
{
    ACC_LPF1_SEL_0      = 0x00,   /* CTRL1_XL[1]  0:一级数字滤波输出. 1:LPF2第二级滤波输出 */
    ACC_LPF1_SEL_1      = 0x02,
    
    /* CTRL1_XL[3:2] 加速度计 量程 */
    ACC_FS_XL_2G        = 0x00,   // FS_XL[1:0]=00 
    ACC_FS_XL_16G       = 0x04,   // FS_XL[1:0]=01
    ACC_FS_XL_4G        = 0x08,   // FS_XL[1:0]=10
    ACC_FS_XL_8G        = 0x0C,   // FS_XL[1:0]=11   

    BW0XL_1_5KHz        = 0x00,    /* 模块模拟链带宽 CTRL1_XL[0]  0:1.5KHz, 1:400Hz */
    BW0XL_400Hz         = 0x01,
    
    /* CTRL1_XL[7:4]加速度输出 速度；CTRL2_G[7:4]加速度输出 速度 */
    RATE_OUT_0Hz        = 0x00,   // ODR_XL[3:0] = 0000 关闭输出 
    RATE_12_5Hz         = 0x10,   // ODR_XL[3:0] = 0100 12.5Hz输出 
    RATE_26Hz           = 0x20,
    RATE_52Hz           = 0x30,
    RATE_104Hz          = 0x40,
    RATE_208Hz          = 0x50,
    RATE_416Hz          = 0x60,
    RATE_833Hz          = 0x70,
    RATE_1_66kHz        = 0x80,
    RATE_3_33kHz        = 0x90,
    RATE_6_66kHz        = 0xA0,
     
    /* 加速度计带宽选择 */
    ACC_LOW_PASS_ODR_50 = 0x88,     // 低通滤波器
    ACC_LOW_PASS_ODR_100= 0xA8,
    ACC_LOW_PASS_ODR_9	= 0xC8,
} lsm6dsr_set_reg_t;

/************************************* 陀螺仪相关函数 **************************************/

uint8_t lq_i2c_lsm6dsr_init         (struct ls_i2c_dev *dev);   /* 初始化LSM6DSR */
uint8_t lq_i2c_lsm6dsr_get_id       (struct ls_i2c_dev *dev);   /* 获取陀螺仪设备 ID */

uint8_t i2c_lsm6dsr_get_raw_data    (struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz); /* 获取原始数据 */

void    delay_ms                    (uint16_t ms);              /* 内核毫秒级延时函数 */

#endif
