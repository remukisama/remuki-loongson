#include "lq_i2c_icm42688.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   ICM42688 设备驱动类有参构造函数.
 * @param   _dev_path: ICM42688 设备路径.
 * @return  none.
 * @example lq_i2c_icm42688 MyICM42688("/dev/lq_i2c_icm42688");
 * @note    none.
 ********************************************************************************/
lq_i2c_icm42688::lq_i2c_icm42688(const std::string _dev_path)
{
    this->i2c_dev_open(_dev_path);
}

/********************************************************************************
 * @brief   ICM42688 设备驱动类析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束后自动调用.
 ********************************************************************************/
lq_i2c_icm42688::~lq_i2c_icm42688()
{
}

/********************************************************************************
 * @brief   获取 ICM42688 设备 ID.
 * @param   none.
 * @return  ICM42688 设备 ID.
 * @example uint16_t id = MyICM42688.get_icm42688_id();
 * @note    none.
 ********************************************************************************/
uint8_t lq_i2c_icm42688::get_icm42688_id()
{
    uint8_t id = 0;
    ioctl(this->fd_, I2C_GET_ICM42688_ID, &id);
    return id;
}

/********************************************************************************
 * @brief   获取 ICM42688 温度.
 * @param   none.
 * @return  ICM42688 温度.
 * @example float tem = MyICM42688.get_icm42688_tem();
 * @note    none.
 ********************************************************************************/
float lq_i2c_icm42688::get_icm42688_tem()
{
    int16_t tem = 0;
    ioctl(this->fd_, I2C_GET_ICM42688_TEM, &tem);
    return (static_cast<float>(tem) / 132.48f) + 25.0f;
}

/********************************************************************************
 * @brief   获取 ICM42688 角速度值.
 * @param   gx : 角速度值指针.
 * @param   gy : 角速度值指针.
 * @param   gz : 角速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t gx, gy, gz;
 *          MyICM42688.get_icm42688_ang(&gx, &gy, &gz);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_icm42688::get_icm42688_ang(int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t ang[3] = {0};
    if (ioctl(this->fd_, I2C_GET_ICM42688_ANG, ang) != 0)
    {
        lq_log_error("get_icm42688_ang failed");
        return false;
    }
    *gx = ang[0];
    *gy = ang[1];
    *gz = ang[2];
    return true;
}

/********************************************************************************
 * @brief   获取 ICM42688 加速度值.
 * @param   ax : 加速度值指针.
 * @param   ay : 加速度值指针.
 * @param   az : 加速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t ax, ay, az;
 *          MyICM42688.get_icm42688_acc(&ax, &ay, &az);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_icm42688::get_icm42688_acc(int16_t *ax, int16_t *ay, int16_t *az)
{
    int16_t acc[3] = {0};
    if (ioctl(this->fd_, I2C_GET_ICM42688_ACC, acc) != 0)
    {
        lq_log_error("get_icm42688_acc failed");
        return false;
    }
    *ax = acc[0];
    *ay = acc[1];
    *az = acc[2];
    return true;
}

/********************************************************************************
 * @brief   获取 ICM42688 角速度和加速度值.
 * @param   ax : 角速度值指针.
 * @param   ay : 角速度值指针.
 * @param   az : 角速度值指针.
 * @param   gx : 角速度值指针.
 * @param   gy : 角速度值指针.
 * @param   gz : 角速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t ax, ay, az, gx, gy, gz;
 *          MyICM42688.get_icm42688_gyro(&ax, &ay, &az, &gx, &gy, &gz);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_icm42688::get_icm42688_gyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[6] = {0};
    if (ioctl(this->fd_, I2C_GET_ICM42688_GYRO, data) != 0)
    {
        lq_log_error("get_icm42688_gyro failed");
        return false;
    }
    *ax = data[0];
    *ay = data[1];
    *az = data[2];
    *gx = data[3];
    *gy = data[4];
    *gz = data[5];
    return true;
}
