#include "lq_uart.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @fn      ls_uart::ls_uart(const std::string& _path, speed_t _baud, ls_uart_stop_bits_t _stop, ls_uart_data_bits_t _data, ls_uart_parity_t _parity)
                : fd(-1), dev_path(_path), buadrate(_baud), stop_bits(_stop), data_bits(_data), parity(_parity)
 * @brief   串口类的带参构造函数.
 * @param   _path   : 所使用的串口设备文件.
 * @param   _baud   : 波特率设置, 默认为 115200.
 *          可选波特率：B0      B50     B75     B110    B134    B150
 *                    B200    B300    B600    B1200   B1800   B2400
 *                    B4800   B9600   B19200  B38400  B57600  B115200
 *                    B230400 B460800 B500000 B576000 B921600
 * @param   _stop   : 停止位选择, 参考 ls_uart_stop_bits_t, 默认为 LS_UART_STOP1.
 * @param   _data   : 数据位选择, 参考 ls_uart_data_bits_t, 默认为 LS_UART_DATA8.
 * @param   _parity : 校验位选择, 参考 ls_uart_parity_t, 默认为 LS_UART_PARITY_NONE.
 * @return  none.
 * @example ls_uart uart(/dev/ttyS1)
 * @note    除了设备文件地址外, 其他参数都有默认值, 一般使用默认值即可.
 ********************************************************************************/
ls_uart::ls_uart(const std::string& _path, speed_t _baud, ls_uart_stop_bits_t _stop, ls_uart_data_bits_t _data, ls_uart_parity_t _parity)
    : fd(-1), dev_path(_path), buadrate(_baud), stop_bits(_stop), data_bits(_data), parity(_parity)
{
    // 打开串口设备文件,O_RDWR->读写 | O_NOCTTY->不设为控制终端 | O_NDELAY->非阻塞
    this->fd = open(_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (this->fd < 0) {
        lq_log_error("Open %s filed", _path.c_str());
    }
    // 清空当前配置
    memset(&this->ts, 0, sizeof(this->ts));
    // 读取当前配置
    if (tcgetattr(this->fd, &this->ts) != 0) {
        lq_log_error("tcgetattr failed");
    }
    // 基层配置：原始模式、无流控
    this->ts.c_cflag |= CREAD | CLOCAL;                     // 启用接收、忽略 Modem 控制线
    this->ts.c_cflag &= ~CRTSCTS;                           // 关闭硬件流控
    this->ts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    // 原始模式
    this->ts.c_iflag &= ~(IXON | IXOFF | IXANY);            // 关闭软件流控
    this->ts.c_oflag &= ~OPOST;                             // 原始输出
    // 设置波特率
    cfsetispeed(&this->ts, this->buadrate);
    cfsetospeed(&this->ts, this->buadrate);
    // 设置数据位
    this->ts.c_cflag &= ~CSIZE; // 清空数据位掩码
    switch (this->data_bits) {
        case LS_UART_DATA5: this->ts.c_cflag |= CS5; break;
        case LS_UART_DATA6: this->ts.c_cflag |= CS6; break;
        case LS_UART_DATA7: this->ts.c_cflag |= CS7; break;
        case LS_UART_DATA8: this->ts.c_cflag |= CS8; break;
        default: lq_log_error("Invalid data bits") ; break;
    }
    // 设置校验位
    switch (this->parity) {
        case LS_UART_PARITY_NONE: this->ts.c_cflag &= ~PARENB; break;   // 关闭校验
        case LS_UART_PARITY_ODD : this->ts.c_cflag |=  PARENB;
                                  this->ts.c_cflag |=  PARODD; break;   // 启用校验 | 奇校验
        case LS_UART_PARITY_EVEN: this->ts.c_cflag |=  PARENB;
                                  this->ts.c_cflag &= ~PARODD; break;   // 启用校验 | 偶校验
        default: lq_log_error("Invalid parity"); break;
    }
    // 设置停止位
    switch (this->stop_bits) {
        case LS_UART_STOP1: this->ts.c_cflag &= ~CSTOPB; break; // 一位停止位
        case LS_UART_STOP2: this->ts.c_cflag |=  CSTOPB; break; // 两位停止位
        default: lq_log_error("Invalid stop bits"); break;
    }
    // 设置超时
    this->ts.c_cc[VMIN] = 0;    // 最小读取 0 字节
    this->ts.c_cc[VTIME] = 5;   // 超时时间 0.5s
    // 应用配置(TCSANOW: 立即生效)
    if (tcsetattr(this->fd, TCSANOW, &this->ts) != 0) {
        lq_log_error("tcsetattr failed");
    }
    // 清空缓冲区(避免就数据干扰)
    this->flush_buffer();
}

/********************************************************************************
 * @fn      ssize_t ls_uart::write_data(const uint8_t* _buf, size_t _len)
 * @brief   串口类的发送函数.
 * @param   _buf : 需要发送的数据缓冲区.
 * @param   _len : 需要发送的数据长度.
 * @return  发送成功的数据长度.
 * @example uart.write_data(buf, sizeof(buf));
 * @note    none.
 ********************************************************************************/
ssize_t ls_uart::write_data(const uint8_t* _buf, size_t _len)
{
    if (this->fd < 0) {
        lq_log_error("Serial not open");
    }
    ssize_t ret = write(this->fd, _buf, _len);
    if (ret < 0) {
        lq_log_error("Write failed");
    }
    return ret;
}

/********************************************************************************
 * @fn      ssize_t ls_uart::read_data(uint8_t* _buf, size_t _len)
 * @brief   串口类的接收函数.
 * @param   _buf : 需要接收的数据缓冲区.
 * @param   _len : 需要接收的数据长度.
 * @return  发送接收的数据长度.
 * @example uart.read_data(buf, sizeof(buf));
 * @note    none.
 ********************************************************************************/
ssize_t ls_uart::read_data(uint8_t* _buf, size_t _len)
{
    if (this->fd < 0) {
        lq_log_error("Serial not open");
    }
    return read(this->fd, _buf, _len);
}

/********************************************************************************
 * @fn      void ls_uart::close_serial(void)
 * @brief   串口类的文件描述符关闭函数.
 * @param   none.
 * @return  none.
 * @note    none.
 ********************************************************************************/
void ls_uart::close_serial(void)
{
    if (this->fd >= 0) {
        close(this->fd);
        this->fd = -1;
    }
}

/********************************************************************************
 * @fn      bool ls_uart::flush_buffer(void)
 * @brief   串口类的清空缓冲区函数.
 * @param   none.
 * @return  none.
 * @note    none.
 ********************************************************************************/
bool ls_uart::flush_buffer(void)
{
    if (this->fd < 0) return false;
    return tcflush(this->fd, TCIOFLUSH) == 0;
}

/********************************************************************************
 * @fn      ls_uart::~ls_uart()
 * @brief   串口类的析构函数.
 * @param   none.
 * @return  none.
 * @note    none.
 ********************************************************************************/
ls_uart::~ls_uart()
{
    this->close_serial();
}
