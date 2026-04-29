#include "lq_vl53l0x_drv.h"
#include "lq_i2c_read_write_drv.h"

const unsigned char VL53_STAR = 0x02; // 0x02 连续测量模式    0x01 单次测量模式

/********************************************************************************
 * @brief    获取温度值
 * @param    dev : 自定义 I2C 相关结构体
 * @return   温度值(扩大了100倍)
 * @date     2019/6/12
 ********************************************************************************/
int vl53l0x_get_distance(struct ls_i2c_dev *dev, uint16_t *distance)
{
    uint8_t dis_buf[2] = {0};
    uint16_t dis = 0;
    static uint16_t last_dis = 0;   // 静态变量保存上一次有效距离(过滤抖动)
    // 检查合法性
    if (!distance) {
        printk(KERN_INFO "%s read distance invalid param! distance=NULL\n", DEVICE_NAME);
        return -EINVAL;
    }
    // 读取2字节距离数据
    i2c_read_regs(dev, VL53_REG_DIS, dis_buf, 2);
    // 数据转换
    dis = (dis_buf[0] << 8) | dis_buf[1];
    // 异常值过滤
    if (dis > MAX_DISTABCE) {
        dis = 0;
    } 
    if (dis == INVALID_DISTANCE) {
        dis = last_dis;
    }
    last_dis = dis;
    *distance = dis;
    return 0;
}

/********************************************************************************
 * @brief   初始化VL53L0X
 * @param   dev : 自定义 I2C 相关结构体
 * @return  成功返回 0，失败返回 1
 * @date    2025/3/20
 ********************************************************************************/
int lq_i2c_vl53l0x_init(struct ls_i2c_dev *dev)
{
    i2c_write_reg(dev, VL53L0X_REG_SYSRANGE_START, VL53_STAR);
    msleep(50);
    printk(KERN_INFO "%s Init success (mode: %s)\n", DEVICE_NAME, VL53_STAR == 0x01 ? "single measurement" : "continuous measurement");
    return 0;
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
