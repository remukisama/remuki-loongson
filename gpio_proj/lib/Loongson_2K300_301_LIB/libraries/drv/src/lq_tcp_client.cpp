#include "lq_tcp_client.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   无参构造函数.
 * @param   none.
 * @return  none.
 * @example lq_tcp_client MyClient;
 * @note    none.
 ********************************************************************************/
lq_tcp_client::lq_tcp_client() noexcept : socket_fd_(-1)
{
}

/********************************************************************************
 * @brief   有参构造函数.
 * @param   _ip   : 服务器IP地址.
 * @param   _port : 服务器端口号.
 * @return  none.
 * @example lq_tcp_client MyClient("197.168.1.100", 8080);
 * @note    none.
 ********************************************************************************/
lq_tcp_client::lq_tcp_client(const std::string _ip, uint16_t _port) : socket_fd_(-1)
{
    this->tcp_client_init(_ip, _port);
}

/********************************************************************************
 * @brief   析构函数.
 * @param   none.
 * @return  none.
 * @note    变量生命周期结束自动执行.
 ********************************************************************************/
lq_tcp_client::~lq_tcp_client() noexcept
{
    this->tcp_close();
}

/********************************************************************************
 * @brief   初始化TCP客户端.
 * @param   _ip   : 服务器IP地址.
 * @param   _port : 服务器端口号.
 * @return  none.
 * @example MyClient.tcp_client_init("197.168.1.100", 8080);
 * @note    none.
 ********************************************************************************/
void lq_tcp_client::tcp_client_init(const std::string _ip, uint16_t _port)
{
    // 创建套接字
    this->socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
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
    memset(&seraddr, 0, sizeof(seraddr));
    seraddr.sin_family = AF_INET;      // 设置地址族为IPv4
    seraddr.sin_port = htons(_port);   // 设置服务器端口, htons 转换字节序
    // 设置服务器 IP 地址
    if (inet_aton(_ip.c_str(), &seraddr.sin_addr) == 0) {
        lq_log_error("inet_aton failed");
        close(this->socket_fd_);
        this->socket_fd_ = -1;
        return;
    }
    // 连接服务器
    if (connect(this->socket_fd_, (struct sockaddr*)&seraddr, sizeof(seraddr)) == -1) {
        lq_log_error("connect failed");
        close(this->socket_fd_);
        this->socket_fd_ = -1;
        return;
    }
    // 记录IP地址和端口号
    this->ip_ = _ip;
    this->port_ = _port;
    lq_log_info("TCP client init success, ip: %s, port: %d", this->ip_.c_str(), this->port_);
}

/********************************************************************************
 * @brief   发送数据.
 * @param   _buf : 发送数据缓冲区指针.
 * @param   _len : 发送数据缓冲区长度.
 * @return  ssize_t, 发送数据字节数, 失败返回负数值.
 * @example MyClient.tcp_send(buf, sizeof(buf));
 * @note    none.
 ********************************************************************************/
ssize_t lq_tcp_client::tcp_send(const void *_buf, size_t _len)
{
    // 加锁保护, 避免并发发送时的竞态
    std::lock_guard<std::mutex> lock(this->mtx_);
    // 检查是否已连接
    if (!this->is_connected()) {
        lq_log_error("Not connected");
        return -1;
    }
    // 检查参数是否有效
    if (_buf == nullptr || _len == 0) {
        lq_log_error("Invalid param");
        return -2;
    }
    // 发送数据
    ssize_t ret = send(this->socket_fd_, _buf, _len, 0);
    if (ret < 0) {
        lq_log_error("send failed");
    }
    return ret;
}

/********************************************************************************
 * @brief   接收数据.
 * @param   _buf : 接收数据缓冲区指针.
 * @param   _len : 接收数据缓冲区长度.
 * @return  ssize_t, 接收数据字节数, 失败返回负数值.
 * @example ssize_t len = MyClient.tcp_recv(buf, sizeof(buf));
 * @note    none.
 ********************************************************************************/
ssize_t lq_tcp_client::tcp_recv(void *_buf, size_t _len)
{
    // 加锁保护, 避免并发接收时的竞态
    std::lock_guard<std::mutex> lock(this->mtx_);
    // 检查是否已连接
    if (!this->is_connected()) {
        lq_log_error("Not connected");
        return -1;
    }
    // 检查参数是否有效
    if (_buf == nullptr || _len == 0) {
        lq_log_error("Invalid param");
        return -2;
    }
    // 接收数据
    ssize_t ret = recv(this->socket_fd_, _buf, _len, 0);
    if (ret < 0) {
        lq_log_error("recv failed");
    }
    return ret;
}

/********************************************************************************
 * @brief   获取TCP套接字文件描述符.
 * @param   none.
 * @return  int, 套接字文件描述符.
 * @example int fd = MyClient.get_tcp_socket_fd();
 * @note    none.
 ********************************************************************************/
int lq_tcp_client::get_tcp_socket_fd() const noexcept
{
    return this->socket_fd_;
}

/********************************************************************************
 * @brief   主动关闭TCP套接字.
 * @param   none.
 * @return  none.
 * @example MyClient.tcp_close();
 * @note    none.
 ********************************************************************************/
void lq_tcp_client::tcp_close() noexcept
{
    // 加锁保护, 避免并发关闭时的竞态
    std::lock_guard<std::mutex> lock(this->mtx_);
    if (this->socket_fd_ >= 0) {
        close(this->socket_fd_);
        this->socket_fd_ = -1;
    }
}

/********************************************************************************
 * @brief   检查连接是否有效.
 * @param   none.
 * @return  bool, 有效返回 true,无效返回 false.
 * @example bool is_connected = MyClient.is_connected();
 * @note    none.
 ********************************************************************************/
bool lq_tcp_client::is_connected() const noexcept
{
    return this->socket_fd_ >= 0;
}
