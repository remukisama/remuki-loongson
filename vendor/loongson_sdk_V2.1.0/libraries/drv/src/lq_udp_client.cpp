#include "lq_udp_client.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   无参构造函数.
 * @param   none.
 * @return  none.
 * @example lq_udp_client MyClient;
 * @note    none.
 ********************************************************************************/
lq_udp_client::lq_udp_client() noexcept : socket_fd_(-1)
{
}

/********************************************************************************
 * @brief   有参构造函数.
 * @param   _ip   : 服务器IP地址.
 * @param   _port : 服务器端口号.
 * @return  none.
 * @example lq_udp_client MyClient("197.168.1.100", 8080);
 * @note    none.
 ********************************************************************************/
lq_udp_client::lq_udp_client(const std::string _ip, uint16_t _port) : socket_fd_(-1)
{
    this->udp_client_init(_ip, _port);
}

/********************************************************************************
 * @brief   析构函数.
 * @param   none.
 * @return  none.
 * @note    变量生命周期结束自动执行.
 ********************************************************************************/
lq_udp_client::~lq_udp_client() noexcept
{
    this->udp_close();
}

/********************************************************************************
 * @brief   初始化UDP客户端.
 * @param   _ip   : 服务器IP地址.
 * @param   _port : 服务器端口号.
 * @return  none.
 * @example MyClient.udp_client_init("197.168.1.100", 8080);
 * @note    none.
 ********************************************************************************/
void lq_udp_client::udp_client_init(const std::string _ip, uint16_t _port)
{
    // 创建套接字
    this->socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->socket_fd_ == -1) {
        lq_log_error("socket create failed");
        return;
    }
    // 设置端口复用
    int val = 1;
    if (setsockopt(this->socket_fd_, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) == -1) {
        lq_log_error("setsockopt failed");
        close(this->socket_fd_);
        this->socket_fd_ = -1;
        return;
    }
    // 配置服务器地址
    struct sockaddr_in seraddr;
    memset(&this->addr_, 0, sizeof(this->addr_));
    this->addr_.sin_family = AF_INET;      // 设置地址族为IPv4
    this->addr_.sin_port = htons(_port);   // 设置服务器端口, htons 转换字节序
    // 设置服务器 IP 地址
    if (inet_pton(AF_INET, _ip.c_str(), &this->addr_.sin_addr) < 1) {
        lq_log_error("inet_pton failed");
        close(this->socket_fd_);
        this->socket_fd_ = -1;
        return;
    }
    // 记录IP地址和端口号
    this->ip_ = _ip;
    this->port_ = _port;
    lq_log_info("UDP client init success, ip: %s, port: %d", _ip.c_str(), _port);
}

/********************************************************************************
 * @brief   发送数据.
 * @param   _buf : 发送数据缓冲区指针.
 * @param   _len : 发送数据缓冲区长度.
 * @return  ssize_t, 发送数据字节数, 失败返回负数值.
 * @example MyClient.udp_send(buf, sizeof(buf));
 * @note    none.
 ********************************************************************************/
ssize_t lq_udp_client::udp_send(const void *_buf, size_t _len)
{
    // 加锁保护, 避免并发发送时的竞态
    std::lock_guard<std::mutex> lock(this->mtx_);
    // 检查参数是否有效
    if (_buf == nullptr || _len == 0) {
        lq_log_error("Invalid param");
        return -2;
    }
    // 发送数据
    ssize_t ret = sendto(this->socket_fd_, _buf, _len, 0, (struct sockaddr*)&this->addr_, sizeof(this->addr_));
    if (ret < 0) {
        lq_log_error("sendto failed");
    }
    return ret;
}

/********************************************************************************
 * @brief   接收数据.
 * @param   _buf : 接收数据缓冲区指针.
 * @param   _len : 接收数据缓冲区长度.
 * @return  ssize_t, 接收数据字节数, 失败返回负数值.
 * @example ssize_t len = MyClient.udp_recv(buf, sizeof(buf));
 * @note    none.
 ********************************************************************************/
ssize_t lq_udp_client::udp_recv(void *_buf, size_t _len)
{
    // 加锁保护, 避免并发接收时的竞态
    std::lock_guard<std::mutex> lock(this->mtx_);
    // 检查参数是否有效
    if (_buf == nullptr || _len == 0) {
        lq_log_error("Invalid param");
        return -2;
    }
    // 接收数据
    socklen_t addr_len = sizeof(this->addr_);
    ssize_t ret = recvfrom(this->socket_fd_, _buf, _len, 0, (struct sockaddr*)&this->addr_, &addr_len);
    if (ret < 0) {
        lq_log_error("recvfrom failed");
    }
    return ret;
}

/********************************************************************************
 * @brief   发送字符串.
 * @param   _str : 要发送的字符串.
 * @return  ssize_t, 发送数据字节数, 失败返回负数值.
 * @example MyClient.udp_send_string("Hello");
 * @note    none.
 ********************************************************************************/
ssize_t lq_udp_client::udp_send_string(const std::string &_str)
{
    return this->udp_send(_str.c_str(), _str.length());
}

#ifdef LQ_HAVE_OPENCV
/********************************************************************************
 * @brief   发送图片（OpenCV图像）.
 * @param   _img       : 要发送的OpenCV图像.
 * @param   _quality   : JPEG编码质量 (1-100), 默认80.
 * @return  ssize_t, 发送数据字节数, 失败返回负数值.
 * @example MyClient.udp_send_image(img, 80);
 * @note    图片会被编码为JPEG格式发送，协议格式：4字节长度+JPEG数据
 ********************************************************************************/
ssize_t lq_udp_client::udp_send_image(const cv::Mat &_img, int _quality)
{
    // 检查图像是否有效
    if (_img.empty()) {
        lq_log_error("Image is empty");
        return -1;
    }

    // 编码为JPEG
    std::vector<uint8_t> jpeg_buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, _quality};

    if (!cv::imencode(".jpg", _img, jpeg_buf, params)) {
        lq_log_error("JPEG encoding failed");
        return -2;
    }

    // 发送长度（4字节）
    uint32_t data_len = jpeg_buf.size();
    uint8_t len_buf[4];
    len_buf[0] = (data_len >> 0)  & 0xFF;
    len_buf[1] = (data_len >> 8)  & 0xFF;
    len_buf[2] = (data_len >> 16) & 0xFF;
    len_buf[3] = (data_len >> 24) & 0xFF;

    ssize_t sent = this->udp_send(len_buf, 4);
    if (sent != 4) {
        lq_log_error("Failed to send image length");
        return -3;
    }

    // 发送JPEG数据
    sent = this->udp_send(jpeg_buf.data(), jpeg_buf.size());
    if (sent != (ssize_t)jpeg_buf.size()) {
        lq_log_error("Failed to send image data");
        return -4;
    }

    return sent;
}
#endif

/********************************************************************************
 * @brief   获取UDP套接字文件描述符.
 * @param   none.
 * @return  int, 套接字文件描述符.
 * @example int fd = MyClient.get_udp_socket_fd();
 * @note    none.
 ********************************************************************************/
int lq_udp_client::get_udp_socket_fd() const noexcept
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    return this->socket_fd_;
}

/********************************************************************************
 * @brief   主动关闭UDP套接字.
 * @param   none.
 * @return  none.
 * @example MyClient.udp_close();
 * @note    none.
 ********************************************************************************/
void lq_udp_client::udp_close() noexcept
{
    // 加锁保护, 避免并发关闭时的竞态
    std::lock_guard<std::mutex> lock(this->mtx_);
    if (this->socket_fd_ >= 0) {
        close(this->socket_fd_);
        this->socket_fd_ = -1;
    }
}
