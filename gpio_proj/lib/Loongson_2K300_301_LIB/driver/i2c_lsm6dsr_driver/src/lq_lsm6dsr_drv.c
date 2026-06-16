#include "lq_lsm6dsr_drv.h"
#include "lq_i2c_read_write_drv.h"

/*************************************************************************
 * @code    void lsm6dsr_setacc_fullscale(u8 _scale)
 * @brief   设置 LSM6DSR 的加速度计满量程
 * @param   u8 _scale  ,设置量程
 * @return  none.
 * @date    2025-12-28.
 * @example void lsm6dsr_setacc_fullscale(u8 _scale)
 *************************************************************************/
void lsm6dsr_setacc_fullscale(struct ls_i2c_dev *dev, u8 _scale)
{
	u8 buf = 0;
    i2c_read_regs(dev, LSM6DSR_CTRL1_XL, &buf, 1);	 // 读取CTRL1_XL寄存器配置
	buf |= _scale;			                         // 设置加速度计的满量程
    i2c_write_regs(dev, LSM6DSR_CTRL1_XL, &buf, 1);
}

/*************************************************************************
 * @code    void lsm6dsr_setgyro_rate(u8 _rate)
 * @brief   设置 LSM6DSR 陀螺仪回报率
 * @param   u8 _rate  ,设置回报率
 * @return  none.
 * @date    2025-12-28.
 * @example void lsm6dsr_setgyro_rate(u8 _rate)
 *************************************************************************/
void lsm6dsr_setgyro_rate(struct ls_i2c_dev *dev, u8 _rate)
{
	u8 buf = 0;
    i2c_read_regs(dev, LSM6DSR_CTRL2_G, &buf, 1);    // 读取LSM6DSR_CTRL2_G寄存器配置
	buf |= _rate;			                         // 设置陀螺仪的满量程
    i2c_write_regs(dev, LSM6DSR_CTRL2_G, &buf, 1);
}

/********************************************************************************
 * @brief   初始化LSM6DSR
 * @param   dev : 自定义 I2C 相关结构体
 * @param   mod : 自定义模块相关结构体
 * @return  成功返回 0，失败返回 1
 * @date    2025/11/22
 ********************************************************************************/
uint8_t lq_i2c_lsm6dsr_init(struct ls_i2c_dev *dev)
{
    i2c_write_reg(dev, LSM6DSR_FUNC_CFG , 0x00);        // 关闭LSM6DSR嵌入式高级功能的配置寄存器组
    i2c_write_reg(dev, LSM6DSR_CTRL3_C , 0x01);         // 关闭LSM6DSR控制寄存器组
    delay_ms(100);   // 延时等待传感器启动完成
    i2c_write_reg(dev, LSM6DSR_FUNC_CFG , 0x00);

    i2c_write_reg(dev, LSM6DSR_CTRL1_XL , RATE_833Hz);   // 设置加速度计回报率,注意不能太高 否则可能死机
    lsm6dsr_setacc_fullscale(dev, ACC_FS_XL_2G);         // acc,2g量程，分辨率高
    i2c_write_reg(dev, LSM6DSR_CTRL9_XL , 0x38);         // 使能加速度计x, y, z轴
    i2c_write_reg(dev, LSM6DSR_CTRL6_C  , 0X40|0x10);    // 陀螺仪电平触发，加速度计高性能使能
    i2c_write_reg(dev, LSM6DSR_CTRL7_G  , 0X80);         // 陀螺仪高性能使能
    i2c_write_reg(dev, LSM6DSR_INT2_CTRL, 0X03);         // 加速度计INT2引脚失能,陀螺仪数据INT2使能
    i2c_write_reg(dev, LSM6DSR_CTRL2_G  , 0X0C);         // 陀螺仪2000dps（0x0C）,选择833Hz（0x70) 默认1c, 12.5K
    lsm6dsr_setgyro_rate(dev, RATE_833Hz);               // 设置陀螺仪回报率,注意：需要与加速度保持相同回报率，且不能太高 否则可能死机
    i2c_write_reg(dev, LSM6DSR_CTRL10_C , 0X38);         // 使能陀螺仪x,y,z轴

    printk("LSM6DSR Init success\n");
    return 0;
}

/********************************************************************************
 * @brief   获取原始数据
 * @param   dev : 自定义 I2C 相关结构体
 * @param   ax : 加速度X轴数据
 * @param   ay : 加速度Y轴数据
 * @param   az : 加速度Z轴数据
 * @param   gx : 陀螺仪X轴数据
 * @param   gy : 陀螺仪Y轴数据
 * @param   gz : 陀螺仪Z轴数据
 ********************************************************************************/
uint8_t i2c_lsm6dsr_get_raw_data(struct ls_i2c_dev *dev, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[12], res;
    res = i2c_read_regs(dev, LSM6DSR_OUTX_L_GYRO, buf, 12);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[1] << 8) | buf[0];
        *ay = ((uint16_t)buf[3] << 8) | buf[2];
        *az = ((uint16_t)buf[5] << 8) | buf[4];
        *gx = ((uint16_t)buf[7] << 8) | buf[6];
        *gy = ((uint16_t)buf[9] << 8) | buf[8];
        *gz = ((uint16_t)buf[11]<< 8) | buf[10];
    }
    return res;
}

/********************************************************************************
 * @brief   读取陀螺仪的设备ID
 * @param   dev : 自定义 I2C 相关结构体
 * @param   mod : 自定义模块相关结构体
 * @return  陀螺仪的设备ID
 * @date    2025/3/20
 ********************************************************************************/
uint8_t lq_i2c_lsm6dsr_get_id(struct ls_i2c_dev *dev)
{
    return i2c_read_reg_byte(dev, LSM6DSR_WHO_AM); //获取陀螺仪设备 ID
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
