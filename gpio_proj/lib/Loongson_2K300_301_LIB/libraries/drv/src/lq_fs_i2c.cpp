#include "lq_fs_i2c.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

static std::mutex i2c_fs_mtx[LS_I2C_BUS_MAX];   // 互斥锁

/********************************************************************************
 * @brief   构造函数，用于初始化 ls_fs_i2c 对象.
 * @param   _bus  : I2C 总线路径.
 * @param   _addr : I2C 从设备地址.
 * @return  none.
 * @note    初始化时会打开 I2C 总线路径对应的文件描述符, 并设置从设备地址.
 * @date    2026-04-15.
 ********************************************************************************/
ls_fs_i2c::ls_fs_i2c(std::string _bus, uint8_t _addr) : bus(_bus), s_addr(_addr), fd(-1), bus_idx(0)
{
    int temp;
    // 参数有效性检验
    if (_bus.empty()) {
        lq_log_error("I2C 总线路径为空");
        return;
    }
    if (_addr > ls_fs_i2c_constants::MAX_SLAVE_ADDR || _addr < ls_fs_i2c_constants::MIN_SLAVE_ADDR) {
        lq_log_error("I2C 从设备地址不在有效范围内: 0x%02x", _addr);
        return;
    }
    // 打开 I2C 总线
    this->fd = open(this->bus.c_str(), O_RDWR);
    if(this->fd < 0) {
        lq_log_error("打开 %s 失败", this->bus.c_str());
        return;
    }
    // 解析总线索引
    if (sscanf(this->bus.c_str(), "/dev/i2c-%d", &temp) != 1) {
        lq_log_error("解析 %s 错误", this->bus.c_str());
        goto open_error;
    }
    // 验证总线索引
    this->bus_idx = static_cast<uint8_t>(temp - 4);
    if (this->bus_idx >= LS_I2C_BUS_MAX) {
        lq_log_error("I2C 总线索引超出范围: %d", this->bus_idx);
        goto open_error;
    }
    return;
open_error:
    close(this->fd);
    this->fd = ls_fs_i2c_constants::I2C_INVALID_FD;
}

/********************************************************************************
 * @brief   析构函数，用于释放 ls_fs_i2c 对象占用的资源.
 * @param   none.
 * @return  none.
 * @note    析构时会关闭 I2C 总线路径对应的文件描述符.
 * @date    2026-04-15.
 ********************************************************************************/
ls_fs_i2c::~ls_fs_i2c() noexcept
{
    if (this->fd >= 0) {
        if (close(this->fd) == -1) {
            lq_log_error("I2C 设备 0x%02x 关闭失败", this->s_addr);
        } else {
            lq_log_info("I2C 设备 0x%02x 关闭成功", this->s_addr);
        }
        this->fd = ls_fs_i2c_constants::I2C_INVALID_FD;
    }
}

/********************************************************************************
 * @brief   I2C 向从设备寄存器中读取一个字节.
 * @param   _reg : 要读取的寄存器地址.
 * @param   _byte: 用于存储读取结果的字节指针.
 * @return  读取的字节数, 成功时为 1, 失败时为 -1.
 * @note    该方法会先设置从设备地址, 然后写入寄存器地址, 最后读取一个字节.
 * @date    2026-04-15.
 ********************************************************************************/
ssize_t ls_fs_i2c::i2c_reg_read_byte(uint8_t _reg, uint8_t *_byte)
{
    return this->i2c_reg_read_bytes(_reg, _byte, 1);
}

/********************************************************************************
 * @brief   I2C 向从设备寄存器中写入一个字节.
 * @param   _reg : 要写入的寄存器地址.
 * @param   _byte: 要写入的字节.
 * @return  写入的字节数, 成功时为 1, 失败时为 -1.
 * @note    该方法会先设置从设备地址, 然后写入寄存器地址, 最后写入一个字节.
 * @date    2026-04-15.
 ********************************************************************************/
ssize_t ls_fs_i2c::i2c_reg_write_byte(uint8_t _reg, uint8_t _byte)
{
    return this->i2c_reg_write_bytes(_reg, &_byte, 1);
}

/********************************************************************************
 * @brief   I2C 向从设备寄存器中读取多个字节.
 * @param   _reg : 要读取的寄存器地址.
 * @param   _byte: 用于存储读取结果的字节指针.
 * @param   _len : 要读取的字节数.
 * @return  读取的字节数, 成功时为 _len, 失败时为 -1.
 * @note    该方法会先设置从设备地址, 然后写入寄存器地址, 最后读取多个字节.
 * @date    2026-04-15.
 ********************************************************************************/
ssize_t ls_fs_i2c::i2c_reg_read_bytes(uint8_t _reg, uint8_t *_byte, size_t _len)
{
    // 参数有效性检验
    if (_byte == nullptr || _len == 0) {
        lq_log_error("I2C 寄存器读取参数无效: _byte 为空或 _len 为 0");
        return -1;
    }
    std::lock_guard<std::mutex> lock(i2c_fs_mtx[this->bus_idx]);
    // 重置 I2C 从设备地址
    if (!this->set_slave_addr(this->s_addr)) {
        return -1;
    }
    // 写入寄存器地址
    if (write(this->fd, &_reg, 1) != 1) {
        return -1;
    }
    // 读取数据
    ssize_t ret = read(this->fd, _byte, _len);
    if (ret < 0) {
        lq_log_error("I2C%d 从机设备 0x%02x 从寄存器 0x%02x 读取 %d 字节失败", this->bus_idx+4, this->s_addr, _reg, _len);
        return -1;
    } else if (static_cast<size_t>(ret) != _len) {
        lq_log_warn("I2C%d 从机设备 0x%02x 从寄存器 0x%02x 读取 %d 字节, 实际读取 %d 字节", this->bus_idx+4, this->s_addr, _reg, _len, ret);
    }
    return ret;
}

/********************************************************************************
 * @brief   I2C 向从设备寄存器中写入多个字节.
 * @param   _reg : 要写入的寄存器地址.
 * @param   _byte: 要写入的字节指针.
 * @param   _len : 要写入的字节数.
 * @return  写入的字节数, 成功时为 _len, 失败时为 -1.
 * @note    该方法会先设置从设备地址, 然后写入寄存器地址, 最后写入多个字节.
 * @date    2026-04-15.
 ********************************************************************************/
ssize_t ls_fs_i2c::i2c_reg_write_bytes(uint8_t _reg, uint8_t *_byte, size_t _len)
{
    // 参数有效性检查
    if (_byte == nullptr || _len == 0) {
        lq_log_error("I2C 寄存器写入参数无效: _byte 为空或 _len 为 0");
        return -1;
    }
    std::lock_guard<std::mutex> lock(i2c_fs_mtx[this->bus_idx]);
    if(!this->set_slave_addr(this->s_addr)) {
        return -1;
    }
    uint8_t dat[ls_fs_i2c_constants::I2C_WRITE_MAX];
    dat[0] = _reg;
    memcpy(dat+1, _byte, _len);
    // 写入数据
    ssize_t ret = write(this->fd, dat, _len+1);
    if (ret < 0) {
        lq_log_error("I2C 寄存器 %d 写入 %d 字节失败", _reg, _len);
        return -1;
    } else if (static_cast<size_t>(ret) != _len+1) {
        lq_log_warn("I2C 寄存器 %d 写入 %d 字节, 实际写入 %d 字节", _reg, _len, ret-1);
    }
    return ret-1;
}

/********************************************************************************
 * @brief   I2C 向从设备读取一个字节.
 * @param   _byte: 用于存储读取结果的字节指针.
 * @return  读取的字节数, 成功时为 1, 失败时为 -1.
 * @note    该方法会先设置从设备地址, 然后读取一个字节.
 * @date    2026-04-15.
 ********************************************************************************/
ssize_t ls_fs_i2c::i2c_read_byte(uint8_t *_byte)
{
    return this->i2c_read_bytes(_byte, 1);
}

/********************************************************************************
 * @brief   I2C 向从设备写入一个字节.
 * @param   _byte: 要写入的字节.
 * @return  写入的字节数, 成功时为 1, 失败时为 -1.
 * @note    该方法会先设置从设备地址, 然后写入一个字节.
 * @date    2026-04-15.
 ********************************************************************************/
ssize_t ls_fs_i2c::i2c_write_byte(uint8_t _byte)
{
    return this->i2c_write_bytes(&_byte, 1);
}

/********************************************************************************
 * @brief   I2C 向从设备读取多个字节.
 * @param   _buf : 用于存储读取结果的字节指针.
 * @param   _len : 要读取的字节数.
 * @return  读取的字节数, 成功时为 _len, 失败时为 -1.
 * @note    该方法会先设置从设备地址, 然后读取多个字节.
 * @date    2026-04-15.
 ********************************************************************************/
ssize_t ls_fs_i2c::i2c_read_bytes(uint8_t *_buf, size_t _len)
{
    // 参数有效性检查
    if (_buf == nullptr || _len == 0) {
        lq_log_error("I2C 读取参数无效: _buf 为空或 _len 为 0");
        return -1;
    }
    std::lock_guard<std::mutex> lock(i2c_fs_mtx[this->bus_idx]);
    // 重置 I2C 从设备地址
    if(!this->set_slave_addr(this->s_addr)) {
        return -1;
    }
    // 读取数据
    ssize_t ret = read(this->fd, _buf, _len);
    if (ret < 0) {
        lq_log_error("I2C 读取 %d 字节失败", _len);
        return -1;
    } else if (static_cast<size_t>(ret) != _len) {
        lq_log_warn("I2C 读取 %d 字节, 实际读取 %d 字节", _len, ret);
    }
    return ret;
}

/********************************************************************************
 * @brief   I2C 向从设备写入多个字节.
 * @param   _buf : 要写入的字节指针.
 * @param   _len : 要写入的字节数.
 * @return  写入的字节数, 成功时为 _len, 失败时为 -1.
 * @note    该方法会先设置从设备地址, 然后写入多个字节.
 * @date    2026-04-15.
 ********************************************************************************/
ssize_t ls_fs_i2c::i2c_write_bytes(uint8_t *_buf, size_t _len)
{
    // 参数有效性检查
    if (_buf == nullptr || _len == 0) {
        lq_log_error("I2C 写入参数无效: _buf 为空或 _len 为 0");
        return -1;
    }
    std::lock_guard<std::mutex> lock(i2c_fs_mtx[this->bus_idx]);
    // 重置 I2C 从设备地址
    if(!this->set_slave_addr(this->s_addr)) {
        return -1;
    }
    // 写入数据
    ssize_t ret = write(this->fd, _buf, _len);
    if (ret < 0) {
        lq_log_error("I2C 写入 %d 字节失败", _len);
        return -1;
    } else if (static_cast<size_t>(ret) != _len) {
        lq_log_warn("I2C 写入 %d 字节, 实际写入 %d 字节", _len, ret);
    }
    return ret;
}

/********************************************************************************
 * @brief   设置 I2C 从设备地址.
 * @param   _addr: 要设置的从设备地址.
 * @return  true  : 成功.
 * @return  false : 失败.
 * @note    该方法会先设置从设备地址.
 * @date    2026-04-15.
 ********************************************************************************/
bool ls_fs_i2c::set_slave_addr(uint8_t _addr)
{
    if (this->fd < 0) {
        lq_log_error("I2C 文件描述符无效");
        return false;
    }
    if(ioctl(this->fd, I2C_SLAVE, this->s_addr) < 0) {
        lq_log_error("设置从设备地址 0x%02X 失败\n", this->s_addr);
        return false;
    }
    this->s_addr = _addr;
    return true;
}
