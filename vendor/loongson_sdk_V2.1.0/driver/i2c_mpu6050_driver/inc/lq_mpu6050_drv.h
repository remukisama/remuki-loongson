#ifndef __LQ_MPU6050_DRI_H__
#define __LQ_MPU6050_DRI_H__ 

#include "lq_drv_common.h"

//****************************************
// 定义MPU6050内部地址
//****************************************
typedef enum
{
    MPU_SELF_TESTX_REG   = 0X0D,    // 自检寄存器X
    MPU_SELF_TESTY_REG   = 0X0E,    // 自检寄存器Y
    MPU_SELF_TESTZ_REG   = 0X0F,    // 自检寄存器z
    MPU_SELF_TESTA_REG   = 0X10,    // 自检寄存器A
    MPU_SAMPLE_RATE_REG  = 0X19,    // 采样频率分频器
    MPU_CFG_REG          = 0X1A,    // 配置寄存器
    MPU_GYRO_CFG_REG     = 0X1B,    // 陀螺仪配置寄存器
    MPU_ACCEL_CFG_REG    = 0X1C,    // 加速度计配置寄存器
    MPU_MOTION_DET_REG   = 0X1F,    // 运动检测阀值设置寄存器
    MPU_FIFO_EN_REG      = 0X23,    // FIFO使能寄存器

    MPU_I2CMST_STA_REG   = 0X36,    // IIC主机状态寄存器
    MPU_INTBP_CFG_REG    = 0X37,    // 中断/旁路设置寄存器
    MPU_INT_EN_REG       = 0X38,    // 中断使能寄存器
    MPU_INT_STA_REG      = 0X3A,    // 中断状态寄存器

    MPU_ACCEL_XOUTH_REG  = 0X3B,    // 加速度值,X轴高8位寄存器
    MPU_ACCEL_XOUTL_REG  = 0X3C,    // 加速度值,X轴低8位寄存器
    MPU_ACCEL_YOUTH_REG  = 0X3D,    // 加速度值,Y轴高8位寄存器
    MPU_ACCEL_YOUTL_REG  = 0X3E,    // 加速度值,Y轴低8位寄存器
    MPU_ACCEL_ZOUTH_REG  = 0X3F,    // 加速度值,Z轴高8位寄存器
    MPU_ACCEL_ZOUTL_REG  = 0X40,    // 加速度值,Z轴低8位寄存器

    MPU_TEMP_OUTH_REG    = 0X41,    // 温度值高八位寄存器
    MPU_TEMP_OUTL_REG    = 0X42,    // 温度值低8位寄存器

    MPU_GYRO_XOUTH_REG   = 0X43,    // 陀螺仪值,X轴高8位寄存器
    MPU_GYRO_XOUTL_REG   = 0X44,    // 陀螺仪值,X轴低8位寄存器
    MPU_GYRO_YOUTH_REG   = 0X45,    // 陀螺仪值,Y轴高8位寄存器
    MPU_GYRO_YOUTL_REG   = 0X46,    // 陀螺仪值,Y轴低8位寄存器
    MPU_GYRO_ZOUTH_REG   = 0X47,    // 陀螺仪值,Z轴高8位寄存器
    MPU_GYRO_ZOUTL_REG   = 0X48,    // 陀螺仪值,Z轴低8位寄存器

    MPU_I2CSLV0_DO_REG   = 0X63,    // IIC从机0数据寄存器
    MPU_I2CSLV1_DO_REG   = 0X64,    // IIC从机1数据寄存器
    MPU_I2CSLV2_DO_REG   = 0X65,    // IIC从机2数据寄存器
    MPU_I2CSLV3_DO_REG   = 0X66,    // IIC从机3数据寄存器

    MPU_I2CMST_DELAY_REG = 0X67,    // IIC主机延时管理寄存器
    MPU_SIGPATH_RST_REG  = 0X68,    // 信号通道复位寄存器
    MPU_MDETECT_CTRL_REG = 0X69,    // 运动检测控制寄存器
    MPU_USER_CTRL_REG    = 0X6A,    // 用户控制寄存器
    MPU_PWR_MGMT1_REG    = 0X6B,    // 电源管理寄存器1
    MPU_PWR_MGMT2_REG    = 0X6C,    // 电源管理寄存器2
    MPU_FIFO_CNTH_REG    = 0X72,    // FIFO计数寄存器高八位
    MPU_FIFO_CNTL_REG    = 0X73,    // FIFO计数寄存器低八位
    MPU_FIFO_RW_REG      = 0X74,    // FIFO读写寄存器
    WHO_AM_I             = 0X75,    // 器件ID寄存器
} mpu6050_reg_addr_t;

/************************************* 陀螺仪相关函数 **************************************/

uint8_t lq_i2c_mpu6050_init         (struct ls_i2c_dev *dev);   /* 初始化陀螺仪 */
uint8_t lq_i2c_mpu6050_get_id       (struct ls_i2c_dev *dev);   /* 获取设备 ID */

int16_t mpu6050_get_temperature     (struct ls_i2c_dev *dev);                                                                               /* 获取温度值 */
uint8_t mpu6050_get_gyro_data       (struct ls_i2c_dev *dev, int16_t *gx, int16_t *gy, int16_t *gz);                                        /* 获取陀螺仪值 */
uint8_t mpu6050_get_accelerometer   (struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az);                                        /* 获取加速度值 */
uint8_t mpu6050_get_raw_data        (struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz); /* 获取 加速度值 角速度值 */

void    delay_ms                    (uint16_t ms);              /* 内核毫秒级延时函数 */

#endif
