#include "lq_i2c_mpu6050.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   MPU6050 设备驱动类有参构造函数.
 * @param   _dev_path: MPU6050 设备路径.
 * @return  none.
 * @example lq_i2c_mpu6050 MyMPU6050("/dev/i2c-0");
 * @note    none.
 ********************************************************************************/
lq_i2c_mpu6050::lq_i2c_mpu6050(const std::string _dev_path)
{
    this->i2c_dev_open(_dev_path);
}

/********************************************************************************
 * @brief   MPU6050 设备驱动类析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束后自动调用.
 ********************************************************************************/
lq_i2c_mpu6050::~lq_i2c_mpu6050()
{
}

/********************************************************************************
 * @brief   获取 MPU6050 设备 ID.
 * @param   none.
 * @return  MPU6050 设备 ID.
 * @example uint16_t id = MyMPU6050.get_mpu6050_id();
 * @note    none.
 ********************************************************************************/
uint8_t lq_i2c_mpu6050::get_mpu6050_id()
{
    uint8_t id = 0;
    ioctl(this->fd_, I2C_GET_MPU6050_ID, &id);
    return id;
}

/********************************************************************************
 * @brief   获取 MPU6050 温度.
 * @param   none.
 * @return  MPU6050 温度.
 * @example float tem = MyMPU6050.get_mpu6050_tem();
 * @note    none.
 ********************************************************************************/
float lq_i2c_mpu6050::get_mpu6050_tem()
{
    float tem = 0.0f;
    ioctl(this->fd_, I2C_GET_MPU6050_TEM, &tem);
    return (float)(tem / 100.0);
}

/********************************************************************************
 * @brief   获取 MPU6050 角速度值.
 * @param   gx : 角速度值指针.
 * @param   gy : 角速度值指针.
 * @param   gz : 角速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t gx, gy, gz;
 *          MyMPU6050.get_mpu6050_ang(&gx, &gy, &gz);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_mpu6050::get_mpu6050_ang(int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t ang[3] = {0};
    if (ioctl(this->fd_, I2C_GET_MPU6050_ANG, ang) != 0)
    {
        lq_log_error("get_mpu6050_ang failed");
        return false;
    }
    *gx = ang[0];
    *gy = ang[1];
    *gz = ang[2];
    return true;
}

/********************************************************************************
 * @brief   获取 MPU6050 加速度值.
 * @param   ax : 加速度值指针.
 * @param   ay : 加速度值指针.
 * @param   az : 加速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t ax, ay, az;
 *          MyMPU6050.get_mpu6050_acc(&ax, &ay, &az);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_mpu6050::get_mpu6050_acc(int16_t *ax, int16_t *ay, int16_t *az)
{
    int16_t acc[3] = {0};
    if (ioctl(this->fd_, I2C_GET_MPU6050_ACC, acc) != 0)
    {
        lq_log_error("get_mpu6050_acc failed");
        return false;
    }
    *ax = acc[0];
    *ay = acc[1];
    *az = acc[2];
    return true;
}

/********************************************************************************
 * @brief   获取 MPU6050 角速度和加速度值.
 * @param   ax : 角速度值指针.
 * @param   ay : 角速度值指针.
 * @param   az : 角速度值指针.
 * @param   gx : 角速度值指针.
 * @param   gy : 角速度值指针.
 * @param   gz : 角速度值指针.
 * @return  true: 成功, false: 失败.
 * @example int16_t ax, ay, az, gx, gy, gz;
 *          MyMPU6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz);
 * @note    none.
 ********************************************************************************/
bool lq_i2c_mpu6050::get_mpu6050_gyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    int16_t data[6] = {0};
    if (ioctl(this->fd_, I2C_GET_MPU6050_GYRO, data) != 0)
    {
        lq_log_error("get_mpu6050_gyro failed");
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
