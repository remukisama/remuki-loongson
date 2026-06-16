#ifndef __LQ_MPU6050_DRI_H__
#define __LQ_MPU6050_DRI_H__ 

#include "lq_drv_common.h"

//****************************************
// 定义ICM42688内部地址
//****************************************
typedef enum
{
    // BANK 0
    UB0_REG_DEVICE_CONFIG           = 0x11,
    UB0_REG_DRIVE_CONFIG            = 0x13,
    UB0_REG_INT_CONFIG              = 0x14,
    UB0_REG_FIFO_CONFIG             = 0x16,
    UB0_REG_TEMP_DATA1              = 0x1D,
    UB0_REG_TEMP_DATA0              = 0x1E,
    UB0_REG_ACCEL_DATA_X1           = 0x1F,
    UB0_REG_ACCEL_DATA_X0           = 0x20,
    UB0_REG_ACCEL_DATA_Y1           = 0x21,
    UB0_REG_ACCEL_DATA_Y0           = 0x22,
    UB0_REG_ACCEL_DATA_Z1           = 0x23,
    UB0_REG_ACCEL_DATA_Z0           = 0x24,
    UB0_REG_GYRO_DATA_X1            = 0x25,
    UB0_REG_GYRO_DATA_X0            = 0x26,
    UB0_REG_GYRO_DATA_Y1            = 0x27,
    UB0_REG_GYRO_DATA_Y0            = 0x28,
    UB0_REG_GYRO_DATA_Z1            = 0x29,
    UB0_REG_GYRO_DATA_Z0            = 0x2A,
    UB0_REG_TMST_FSYNCH             = 0x2B,
    UB0_REG_TMST_FSYNCL             = 0x2C,
    UB0_REG_INT_STATUS              = 0x2D,
    UB0_REG_FIFO_COUNTH             = 0x2E,
    UB0_REG_FIFO_COUNTL             = 0x2F,
    UB0_REG_FIFO_DATA               = 0x30,
    UB0_REG_APEX_DATA0              = 0x31,
    UB0_REG_APEX_DATA1              = 0x32,
    UB0_REG_APEX_DATA2              = 0x33,
    UB0_REG_APEX_DATA3              = 0x34,
    UB0_REG_APEX_DATA4              = 0x35,
    UB0_REG_APEX_DATA5              = 0x36,
    UB0_REG_INT_STATUS2             = 0x37,
    UB0_REG_INT_STATUS3             = 0x38,
    UB0_REG_SIGNAL_PATH_RESET       = 0x4B,
    UB0_REG_INTF_CONFIG0            = 0x4C,
    UB0_REG_INTF_CONFIG1            = 0x4D,
    UB0_REG_PWR_MGMT0               = 0x4E,
    UB0_REG_GYRO_CONFIG0            = 0x4F,
    UB0_REG_ACCEL_CONFIG0           = 0x50,
    UB0_REG_GYRO_CONFIG1            = 0x51,
    UB0_REG_GYRO_ACCEL_CONFIG0      = 0x52,
    UB0_REG_ACCEFL_CONFIG1          = 0x53,
    UB0_REG_TMST_CONFIG             = 0x54,
    UB0_REG_APEX_CONFIG0            = 0x56,
    UB0_REG_SMD_CONFIG              = 0x57,
    UB0_REG_FIFO_CONFIG1            = 0x5F,
    UB0_REG_FIFO_CONFIG2            = 0x60,
    UB0_REG_FIFO_CONFIG3            = 0x61,
    UB0_REG_FSYNC_CONFIG            = 0x62,
    UB0_REG_INT_CONFIG0             = 0x63,
    UB0_REG_INT_CONFIG1             = 0x64,
    UB0_REG_INT_SOURCE0             = 0x65,
    UB0_REG_INT_SOURCE1             = 0x66,
    UB0_REG_INT_SOURCE3             = 0x68,
    UB0_REG_INT_SOURCE4             = 0x69,
    UB0_REG_FIFO_LOST_PKT0          = 0x6C,
    UB0_REG_FIFO_LOST_PKT1          = 0x6D,
    UB0_REG_SELF_TEST_CONFIG        = 0x70,
    UB0_REG_WHO_AM_I                = 0x75,
    REG_BANK_SEL                    = 0x76,
    // BANK 1
    UB1_REG_SENSOR_CONFIG0          = 0x03,
    UB1_REG_GYRO_CONFIG_STATIC2     = 0x0B,
    UB1_REG_GYRO_CONFIG_STATIC3     = 0x0C,
    UB1_REG_GYRO_CONFIG_STATIC4     = 0x0D,
    UB1_REG_GYRO_CONFIG_STATIC5     = 0x0E,
    UB1_REG_GYRO_CONFIG_STATIC6     = 0x0F,
    UB1_REG_GYRO_CONFIG_STATIC7     = 0x10,
    UB1_REG_GYRO_CONFIG_STATIC8     = 0x11,
    UB1_REG_GYRO_CONFIG_STATIC9     = 0x12,
    UB1_REG_GYRO_CONFIG_STATIC10    = 0x13,
    UB1_REG_XG_ST_DATA              = 0x5F,
    UB1_REG_YG_ST_DATA              = 0x60,
    UB1_REG_ZG_ST_DATA              = 0x61,
    UB1_REG_TMSTVAL0                = 0x62,
    UB1_REG_TMSTVAL1                = 0x63,
    UB1_REG_TMSTVAL2                = 0x64,
    UB1_REG_INTF_CONFIG4            = 0x7A,
    UB1_REG_INTF_CONFIG5            = 0x7B,
    UB1_REG_INTF_CONFIG6            = 0x7C,
    // BANK 2
    UB2_REG_ACCEL_CONFIG_STATIC2    = 0x03,
    UB2_REG_ACCEL_CONFIG_STATIC3    = 0x04,
    UB2_REG_ACCEL_CONFIG_STATIC4    = 0x05,
    UB2_REG_XA_ST_DATA              = 0x3B,
    UB2_REG_YA_ST_DATA              = 0x3C,
    UB2_REG_ZA_ST_DATA              = 0x3D,
    // BANK 4
    UB4_REG_APEX_CONFIG1            = 0x40,
    UB4_REG_APEX_CONFIG2            = 0x41,
    UB4_REG_APEX_CONFIG3            = 0x42,
    UB4_REG_APEX_CONFIG4            = 0x43,
    UB4_REG_APEX_CONFIG5            = 0x44,
    UB4_REG_APEX_CONFIG6            = 0x45,
    UB4_REG_APEX_CONFIG7            = 0x46,
    UB4_REG_APEX_CONFIG8            = 0x47,
    UB4_REG_APEX_CONFIG9            = 0x48,
    UB4_REG_ACCEL_WOM_X_THR         = 0x4A,
    UB4_REG_ACCEL_WOM_Y_THR         = 0x4B,
    UB4_REG_ACCEL_WOM_Z_THR         = 0x4C,
    UB4_REG_INT_SOURCE6             = 0x4D,
    UB4_REG_INT_SOURCE7             = 0x4E,
    UB4_REG_INT_SOURCE8             = 0x4F,
    UB4_REG_INT_SOURCE9             = 0x50,
    UB4_REG_INT_SOURCE10            = 0x51,
    UB4_REG_OFFSET_USER0            = 0x77,
    UB4_REG_OFFSET_USER1            = 0x78,
    UB4_REG_OFFSET_USER2            = 0x79,
    UB4_REG_OFFSET_USER3            = 0x7A,
    UB4_REG_OFFSET_USER4            = 0x7B,
    UB4_REG_OFFSET_USER5            = 0x7C,
    UB4_REG_OFFSET_USER6            = 0x7D,
    UB4_REG_OFFSET_USER7            = 0x7E,
    UB4_REG_OFFSET_USER8            = 0x7F,
} icm42688_reg_addr_t;

// ICM42688 加速度计满量程范围
typedef enum
{
    ICM42688_ACCEL_FSR_16G = 0x00,
    ICM42688_ACCEL_FSR_8G,
    ICM42688_ACCEL_FSR_4G,
    ICM42688_ACCEL_FSR_2G,
} icm42688_accel_fsr_t;

// ICM42688 陀螺仪满量程范围
typedef enum
{
    ICM42688_GYRO_FSR_2000DPS = 0x00,
	ICM42688_GYRO_FSR_1000DPS,
	ICM42688_GYRO_FSR_500DPS,
	ICM42688_GYRO_FSR_250DPS,
	ICM42688_GYRO_FSR_125DPS,
	ICM42688_GYRO_FSR_62_5DPS,
	ICM42688_GYRO_FSR_31_25DPS,
	ICM42688_GYRO_FSR_15_625DPS,
} icm42688_gyro_fsr_t;

/************************************* 陀螺仪相关函数 **************************************/

int     lq_i2c_icm42688_init        (struct ls_i2c_dev *dev);                   /* 初始化陀螺仪 */
uint8_t lq_i2c_icm42688_get_id      (struct ls_i2c_dev *dev);                   /* 获取设备 ID */

int16_t icm42688_get_temperature    (struct ls_i2c_dev *dev);                   /* 获取温度值 */
int     icm42688_get_gyro_data      (struct ls_i2c_dev *dev, int16_t *dat);     /* 获取陀螺仪值 */
int     icm42688_get_accelerometer  (struct ls_i2c_dev *dev, int16_t *dat);     /* 获取加速度值 */
int     icm42688_get_raw_data       (struct ls_i2c_dev *dev, int16_t *dat);     /* 获取 加速度值 角速度值 温度值 原始数据 */

void    delay_ms                    (uint16_t ms);                              /* 内核毫秒级延时函数 */

#endif
