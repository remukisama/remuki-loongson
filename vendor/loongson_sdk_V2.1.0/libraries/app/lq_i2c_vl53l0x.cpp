#include "lq_i2c_vl53l0x.hpp"

/********************************************************************************
 * @brief   VL53L0X 设备驱动类有参构造函数.
 * @param   _dev_path: VL53L0X 设备路径.
 * @return  none.
 * @example lq_i2c_vl53l0x MyVL53L0X("/dev/i2c-0");
 * @note    none.
 ********************************************************************************/
lq_i2c_vl53l0x::lq_i2c_vl53l0x(const std::string _dev_path)
{
    this->i2c_dev_open(_dev_path);
}

/********************************************************************************
 * @brief   VL53L0X 设备驱动类析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束后自动调用.
 ********************************************************************************/
lq_i2c_vl53l0x::~lq_i2c_vl53l0x()
{
}

/********************************************************************************
 * @brief   获取 VL53L0X 距离值.
 * @param   none.
 * @return  VL53L0X 距离值.
 * @example uint16_t dis = MyVL53L0X.get_vl53l0x_dis();
 * @note    none.
 ********************************************************************************/
uint16_t lq_i2c_vl53l0x::get_vl53l0x_dis()
{
    uint16_t dis = 0;
    ioctl(this->fd_, I2C_GET_VL53L0X_DIS, &dis);
    return dis;
}
