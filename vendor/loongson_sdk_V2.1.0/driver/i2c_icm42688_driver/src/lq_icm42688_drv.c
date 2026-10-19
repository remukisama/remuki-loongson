#include "lq_icm42688_drv.h"
#include "lq_i2c_read_write_drv.h"

static float   _accelScale;
static uint8_t _accelFS;

/********************************************************************************
 * @brief   设置陀螺仪测量范围
 * @param   dev : 自定义 I2C 相关结构体
 * @param   fsr : 陀螺仪满量程范围, 可见 icm42688_gyro_fsr_t
 * @return  为 0 表示设置成功，其他值 表示设置失败
 * @date    2025/3/20
 ********************************************************************************/
int icm42688_set_gyro_fsr(struct ls_i2c_dev *dev, icm42688_gyro_fsr_t fsr)
{
    uint8_t reg;
    i2c_write_reg(dev, REG_BANK_SEL, 0x00); // 切换到 BANK 0
    // 读取当前寄存器值
    if (i2c_read_regs(dev, UB0_REG_GYRO_CONFIG0, &reg, 1) < 0) {
        pr_err("%s: read GYRO_CONFIG0 failed\n", DEVICE_NAME);
        return -1;
    }
    // 仅修改 FS_SEL 位
    reg = (fsr << 5) | (reg & 0x1F);
    if (i2c_write_reg(dev, UB0_REG_GYRO_CONFIG0, reg) < 1) {
        pr_err("%s: write GYRO_CONFIG0 failed\n", DEVICE_NAME);
        return -1;
    }
    _gyroFS    = fsr;
    return 0;
}

/********************************************************************************
 * @brief   设置加速度计测量范围
 * @param   dev : 自定义 I2C 相关结构体
 * @param   fsr : 加速度计满量程范围, 可见 icm42688_accel_fsr_t
 * @return  为 0 表示设置成功，其他值 表示设置失败
 * @date    2025/3/20
 ********************************************************************************/
int icm42688_set_accel_fsr(struct ls_i2c_dev *dev, icm42688_accel_fsr_t fsr)
{
    uint8_t reg;
    i2c_write_reg(dev, REG_BANK_SEL, 0x00); // 切换到 BANK 0
    // 读取当前寄存器值
    if (i2c_read_regs(dev, UB0_REG_ACCEL_CONFIG0, &reg, 1) < 0) {
        pr_err("%s: read ACCEL_CONFIG0 failed\n", DEVICE_NAME);
        return -1;
    }
    // 仅修改 FS_SEL 位
    reg = (fsr << 5) | (reg & 0x1F);
    if (i2c_write_reg(dev, UB0_REG_ACCEL_CONFIG0, reg) < 1) {
        pr_err("%s: write ACCEL_CONFIG0 failed\n", DEVICE_NAME);
        return -1;
    }
    _accelFS = fsr;
    return 0;
}

/********************************************************************************
 * @brief   设置滤波器
 * @param   dev : 自定义 I2C 相关结构体
 * @param   af  : 是否开启加速度计滤波
 * @param   gf  : 是否开启陀螺仪滤波
 * @return  为 0 表示设置成功，其他值 表示设置失败
 * @date    2025/3/20
 ********************************************************************************/
int icm42688_set_filters(struct ls_i2c_dev *dev, bool af, bool gf)
{
    i2c_write_reg(dev, REG_BANK_SEL, 0x02); // 切换到 BANK 2
    if (af == true) {
        if (i2c_write_reg(dev, UB2_REG_ACCEL_CONFIG_STATIC2, 0x00) < 1) {
            pr_err("%s: write ACCEL_CONFIG_STATIC2 failed\n", DEVICE_NAME);
            return -1;
        }
    } else {
        if (i2c_write_reg(dev, UB2_REG_ACCEL_CONFIG_STATIC2, 0x01) < 1) {
            pr_err("%s: write ACCEL_CONFIG_STATIC2 failed\n", DEVICE_NAME);
            return -1;
        }
    }

    i2c_write_reg(dev, REG_BANK_SEL, 0x01); // 切换到 BANK 1
    if (gf == true) {
        if (i2c_write_reg(dev, UB1_REG_GYRO_CONFIG_STATIC2, 0x00 | 0x00) < 1) {
            pr_err("%s: write GYRO_CONFIG_STATIC2 failed\n", DEVICE_NAME);
            return -1;
        }
    } else {
        if (i2c_write_reg(dev, UB1_REG_GYRO_CONFIG_STATIC2, 0x01 | 0x02) < 1) {
            pr_err("%s: write GYRO_CONFIG_STATIC2 failed\n", DEVICE_NAME);
            return -1;
        }
    }
    return 0;
}

/********************************************************************************
 * @brief    获取温度值
 * @param    dev : 自定义 I2C 相关结构体
 * @return   温度原始值
 * @date     2025/3/20
 ********************************************************************************/
int16_t icm42688_get_temperature(struct ls_i2c_dev *dev)
{
    uint8_t buf[2];
    i2c_write_reg(dev, REG_BANK_SEL, 0x00); // 切换到BANK 0
    if (i2c_read_regs(dev, UB0_REG_TEMP_DATA1, buf, 2) != 0) {
        pr_err("%s: read TEMP_DATA1 failed\n", DEVICE_NAME);
        return -1;
    }
    return (((int16_t)buf[0] << 8) | buf[1]);
}

/********************************************************************************
 * @brief   获取陀螺仪值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   gx,gy,gz: 陀螺仪 x,y,z 轴的原始读数(带符号)
 * @return  为 0 表示读取成功，其他则失败
 * @date    2025/3/20
 ********************************************************************************/
int icm42688_get_gyro_data(struct ls_i2c_dev *dev, int16_t *dat)
{
    uint8_t buf[6];
    i2c_write_reg(dev, REG_BANK_SEL, 0x00); // 切换到BANK 0
    if (i2c_read_regs(dev, UB0_REG_GYRO_DATA_X1, buf, 6) != 0) {
        pr_err("%s: read GYRO_DATA_X1 failed\n", DEVICE_NAME);
        return -1;
    }
    dat[0] = ((int16_t)buf[0] << 8) | buf[1];
    dat[1] = ((int16_t)buf[2] << 8) | buf[3];
    dat[2] = ((int16_t)buf[4] << 8) | buf[5];
    return 0;
}

/********************************************************************************
 * @brief   获取加速度值
 * @param   dev     : 自定义 I2C 相关结构体
 * @param   ax,ay,az: 加速度 x,y,z 轴的原始读数(带符号)
 * @return  为 0 表示读取成功，其他则失败
 * @date    2025/3/20
 ********************************************************************************/
int icm42688_get_accelerometer(struct ls_i2c_dev *dev, int16_t *dat)
{
    uint8_t buf[6];
    i2c_write_reg(dev, REG_BANK_SEL, 0x00); // 切换到BANK 0
    if (i2c_read_regs(dev, UB0_REG_ACCEL_DATA_X1, buf, 6) != 0) {
        pr_err("%s: read ACCEL_DATA_X1 failed\n", DEVICE_NAME);
        return -1;
    }
    dat[0] = ((int16_t)buf[0] << 8) | buf[1];
    dat[1] = ((int16_t)buf[2] << 8) | buf[3];
    dat[2] = ((int16_t)buf[4] << 8) | buf[5];
    return 0;
}

/********************************************************************************
 * @brief   获取 加速度值 陀螺仪值 温度值 原始数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   dat : 原始数据缓冲区
 * @return  为 2 表示读取成功，其他则失败
 * @date    2025/3/20
 ********************************************************************************/
int icm42688_get_raw_data(struct ls_i2c_dev *dev, int16_t *dat)
{
    uint8_t buf[14], res;
    i2c_write_reg(dev, REG_BANK_SEL, 0x00);             // 切换到BANK 0
    res = i2c_read_regs(dev, UB0_REG_ACCEL_DATA_X1, buf, 12);
    if (res != 0) {
        pr_err("%s: read ALL_DATA failed\n", DEVICE_NAME);
        return -1;
    }
    dat[0] = ((int16_t)buf[0]  << 8) | buf[1];
    dat[1] = ((int16_t)buf[2]  << 8) | buf[3];
    dat[2] = ((int16_t)buf[4]  << 8) | buf[5];
    dat[3] = ((int16_t)buf[6]  << 8) | buf[7];
    dat[4] = ((int16_t)buf[8]  << 8) | buf[9];
    dat[5] = ((int16_t)buf[10] << 8) | buf[11];
    return res;
}

/********************************************************************************
 * @brief   初始化MPU6050
 * @param   dev : 自定义 I2C 相关结构体
 * @param   mod : 自定义模块相关结构体
 * @return  成功返回 0，失败返回 1
 * @date    2025/3/20
 ********************************************************************************/
int lq_i2c_icm42688_init(struct ls_i2c_dev *dev)
{
    int ret;
    u8 res;
    /* 跳过软件复位 -- GPIO I2C 软复位后无法通信, 芯片 probe 阶段已确认正常 */
    /* 复位操作 */
    ret = i2c_write_reg(dev, REG_BANK_SEL, 0x00);             // 切换到BANK 0
    if (ret < 1) {
        pr_err("%s, REG_BANK_SEL failed, ret=%d\n", DEVICE_NAME, ret);
        return -1;
    }
    delay_ms(2);
    
    ret = i2c_write_reg(dev, UB0_REG_INT_CONFIG, 0x02);       // 中断配置 0x02
    if (ret < 1) {
        pr_err("%s: INT_CONFIG failed, ret=%d\n", DEVICE_NAME, ret);
        return -1;
    }
    
    res = i2c_read_reg_byte(dev, UB0_REG_WHO_AM_I);     // 读取设备 ID
    if (res != ICM42688_WHO_AM_I) {
        pr_err("%s: init failed, who_am_i = 0x47 -> 0x%02x\n", DEVICE_NAME, res);
        return -1;
    }
    pr_err("%s: init success, who_am_i = 0x%02x\n", DEVICE_NAME, res);
    // 启用加速度计和陀螺仪，在低噪声(LN)模式
    ret = i2c_write_reg(dev, UB0_REG_PWR_MGMT0, 0x0F);
    if (ret < 1) {
        pr_err("%s: init failed, pwr_mgmt0 = 0x%02x\n", DEVICE_NAME, ret);
        return -1;
    }
    delay_ms(20);   // 加长等待电源稳定
    ret = i2c_write_reg(dev, UB0_REG_INT_CONFIG0, 0x00);       // 中断配置 0x00
    if (ret < 1) {
        pr_err("%s: INT_CONFIG failed, ret=%d\n", DEVICE_NAME, ret);
        return -1;
    }
    delay_ms(5);
    ret = i2c_write_reg(dev, UB0_REG_INT_SOURCE0, 0x08);       // 中断源 0x08
    if (ret < 1) {
        pr_err("%s: INT_SOURCEO failed, ret=%d\n", DEVICE_NAME, ret);
        return -1;
    }
    delay_ms(5);
    ret = i2c_write_reg(dev, REG_BANK_SEL, 0x01);             // 切换到BANK 1
    if (ret < 1) {
        pr_err("%s: REG_BANK_SEL failed, ret=%d\n", DEVICE_NAME, ret);
        return -1;
    }
    delay_ms(5);
    i2c_write_reg(dev, UB1_REG_SENSOR_CONFIG0, 0x80);   // 开启加速度计和陀螺仪
    if (ret < 1) {
        pr_err("%s: SENSOR_CONFIG0 failed, ret=%d\n", DEVICE_NAME, ret);
        return -1;
    }
    delay_ms(5);
    // 设置加速度计的满量程范围
    if (icm42688_set_accel_fsr(dev, ICM42688_ACCEL_FSR_4G) != 0) {
        pr_err("%s: set_accel_fsr failed\n", DEVICE_NAME);
        return -1;
    }
    // 设置陀螺仪的满量程范围
    if (icm42688_set_gyro_fsr(dev, ICM42688_GYRO_FSR_2000DPS) != 0) {
        pr_err("%s: set_gyro_fsr failed\n", DEVICE_NAME);
        return -1;
    }
    // 设置滤波器
    if (icm42688_set_filters(dev, false, false) != 0) {
        pr_err("%s: set_filters failed\n", DEVICE_NAME);
        return -1;
    }

    return 0;
}

/********************************************************************************
 * @brief   读取陀螺仪的设备ID
 * @param   dev : 自定义 I2C 相关结构体
 * @param   mod : 自定义模块相关结构体
 * @return  陀螺仪的设备ID
 * @date    2025/3/20
 ********************************************************************************/
uint8_t lq_i2c_icm42688_get_id(struct ls_i2c_dev *dev)
{
    return i2c_read_reg_byte(dev, UB0_REG_WHO_AM_I); //获取ICM42688设备 ID
}

/********************************************************************************
 * @brief   内核毫秒级延时函数
 * @param   ms : 毫秒值
 * @return  无
 * @date    2025/3/20
 ********************************************************************************/
void delay_ms(uint16_t ms)
{
    mdelay(ms);
}
