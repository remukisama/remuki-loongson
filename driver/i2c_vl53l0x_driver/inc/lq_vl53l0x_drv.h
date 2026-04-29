#ifndef __LQ_VL53L0X_DRI_H__
#define __LQ_VL53L0X_DRI_H__ 

#include "lq_drv_common.h"

//****************************************
// 定义VL53L0X内部地址
//****************************************
typedef enum
{
    MAX_DISTABCE                                = 80000,
    INVALID_DISTANCE                            = 20,

    VL53L0X_REG_IDENTIFICATION_MODEL_ID         = 0xc0,
    VL53L0X_REG_IDENTIFICATION_REVISION_ID      = 0xc2,
    VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   = 0x50,
    VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
    VL53L0X_REG_SYSRANGE_START                  = 0x00,
    VL53L0X_REG_RESULT_INTERRUPT_STATUS         = 0x13,
    VL53L0X_REG_RESULT_RANGE_STATUS             = 0x14,
    VL53_REG_DIS                                = 0x1E,
    VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS        = 0x8a,
    VL53ADDR                                    = 0x29,    // 0x52   默认地址
    VL53NEWADDR                                 = 0x30,    // VL53新地址
} vl53l0x_reg_addr_t;

/************************************* VL53L0X相关函数 **************************************/

int  lq_i2c_vl53l0x_init    (struct ls_i2c_dev *dev);                       /* 初始化陀螺仪 */

int  vl53l0x_get_distance   (struct ls_i2c_dev *dev, uint16_t *distance);   /* 获取距离 */

void delay_ms               (uint16_t ms);                                  /* 内核毫秒级延时函数 */

#endif
