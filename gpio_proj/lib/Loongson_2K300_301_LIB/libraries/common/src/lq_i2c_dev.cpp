#include "lq_i2c_dev.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   I2C 设备类无参构造函数.
 * @param   none.
 * @return  none.
 * @example lq_i2c_devs MyI2C;
 * @note    none.
 ********************************************************************************/
lq_i2c_devs::lq_i2c_devs() : fd_(-1), dev_path_("")
{
}

/********************************************************************************
 * @brief   I2C 设备类有参构造函数.
 * @param   _dev_path: 设备路径.
 * @return  none.
 * @example lq_i2c_devs MyI2C("/dev/i2c-0");
 * @note    none.
 ********************************************************************************/
lq_i2c_devs::lq_i2c_devs(const std::string _dev_path) : fd_(-1), dev_path_("")
{
    this->i2c_dev_open(_dev_path);
}

/********************************************************************************
 * @brief   I2C 设备类拷贝构造函数.
 * @param   _other: 其他I2C设备对象.
 * @return  none.
 * @example lq_i2c_devs MyI2C("/dev/i2c-0");
 *          lq_i2c_devs MyI2C2(MyI2C);
 * @note    none.
 ********************************************************************************/
lq_i2c_devs::lq_i2c_devs(const lq_i2c_devs& _other) : fd_(_other.fd_), dev_path_(_other.dev_path_)
{
}

/********************************************************************************
 * @brief   I2C 设备类赋值运算符函数.
 * @param   _other: 其他I2C设备对象.
 * @return  none.
 * @example lq_i2c_devs MyI2C("/dev/i2c-0");
 *          lq_i2c_devs MyI2C2 = MyI2C;
 * @note    none.
 *********************************************************************************/
lq_i2c_devs& lq_i2c_devs::operator=(const lq_i2c_devs& _other)
{
    if (this == &_other)
        return *this;

    std::lock_guard<std::mutex> lock(this->mtx_);
    this->fd_       = _other.fd_;
    this->dev_path_ = _other.dev_path_;

    return *this;
}

/********************************************************************************
 * @brief   I2C 设备类初始化函数.
 * @param   _dev_path: 设备路径.
 * @return  none.
 * @example lq_i2c_devs MyI2C();
 *          MyI2C.i2c_dev_open("/dev/i2c-0");
 * @note    none.
 ********************************************************************************/
bool lq_i2c_devs::i2c_dev_open(const std::string _dev_path)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    // 检查设备路径是否为空
    if (_dev_path.empty())
    {
        lq_log_error("i2c device path is empty!");
        return false;
    }
    // 检查是否重复初始化
    if (this->fd_ >= 0)
    {
        lq_log_error("i2c device has been initialized!");
        return false;
    }
    // 打开设备文件
    this->fd_ = open(_dev_path.c_str(), O_RDWR);
    if (this->fd_ < 0)
    {
        lq_log_error("open i2c device %s failed!", _dev_path.c_str());
        return false;
    }
    this->dev_path_ = _dev_path;
    return true; 
}

/********************************************************************************
 * @brief   I2C 设备类关闭函数.
 * @param   none.
 * @return  none.
 * @example lq_i2c_devs MyI2C;
 *          MyI2C.i2c_dev_close();
 * @note    none.
 ********************************************************************************/
void lq_i2c_devs::i2c_dev_close()
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    if (this->fd_ >= 0)
    {
        close(this->fd_);
        this->fd_ = -1;
        lq_log_info("i2c device %s closed!", this->dev_path_.c_str());
    }
}

/********************************************************************************
 * @brief   I2C 设备类析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束时自动调用.
 ********************************************************************************/
lq_i2c_devs::~lq_i2c_devs()
{
    this->i2c_dev_close();
}
