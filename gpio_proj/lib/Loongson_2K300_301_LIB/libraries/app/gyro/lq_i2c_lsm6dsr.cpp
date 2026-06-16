#include "lq_i2c_lsm6dsr.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   LSM6DSR 设备驱动类有参构造函数.
 * @param   _dev_path: LSM6DSR 设备路径.
 * @return  none.
 * @example lq_i2c_lsm6dsr MyLSM6DSR("/dev/lq_i2c_lsm6dsr");
 * @note    none.
 ********************************************************************************/
lq_i2c_lsm6dsr::lq_i2c_lsm6dsr(const std::string _dev_path)
{
    this->i2c_dev_open(_dev_path);
}

/********************************************************************************
 * @brief   LSM6DSR 设备驱动类析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束后自动调用.
 ********************************************************************************/
lq_i2c_lsm6dsr::~lq_i2c_lsm6dsr()
{
}

/********************************************************************************
 * @brief   获取 LSM6DSR 设备 ID.
 * @param   none.
 * @return  LSM6DSR 设备 ID.
 * @example uint16_t id = MyLSM6DSR.get_lsm6dsr_id();
 * @note    none.
 ********************************************************************************/
uint8_t lq_i2c_lsm6dsr::get_lsm6dsr_id()
{
    uint8_t id = 0;
    ioctl(this->fd_, I2C_GET_LSM6DSR_ID, &id);
    return id;
}

/********************************************************************************
 * @brief   获取 LSM6DSR 角速度和加速度值.
 * @param   ax : 角速度值指针.
 * @param   ay : 角速度值指针.
 * @param   az : 角速度值指针.
 * @param   gx : 角速度值指针.
 * @param   gy : 角速度值指针.
 * @param   gz : 角速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t ax, ay, az, gx, gy, gz;
 *          MyMPU6050.get_lsm6dsr_gyro(&ax, &ay, &az, &gx, &gy, &gz);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_lsm6dsr::get_lsm6dsr_gyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[6] = {0};
    if (ioctl(this->fd_, I2C_GET_LSM6DSR_GYRO, data) != 0)
    {
        lq_log_error("get_lsm6dsr_gyro failed");
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
