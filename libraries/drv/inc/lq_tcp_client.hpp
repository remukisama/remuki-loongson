#ifndef __LQ_TCP_CLIENT_HPP
#define __LQ_TCP_CLIENT_HPP

#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <memory>
#include <mutex>

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class lq_tcp_client
{
public:
    lq_tcp_client() noexcept;                               // 默认构造函数
    lq_tcp_client(const std::string _ip, uint16_t _port);   // 有参构造函数

    lq_tcp_client(const lq_tcp_client &_other) noexcept;            // 复制构造函数
    lq_tcp_client &operator=(const lq_tcp_client &_other) noexcept; // 赋值运算符重载

    ~lq_tcp_client() noexcept;                              // 析构函数

public:
    void tcp_client_init(const std::string _ip, uint16_t _port);    // 初始化TCP客户端

    ssize_t tcp_send(const void *_buf, size_t _len);    // 发送数据
    ssize_t tcp_recv(void *_buf, size_t _len);          // 接收数据

    int get_tcp_socket_fd() const noexcept;             // 获取TCP套接字文件描述符

    void tcp_close() noexcept;                          // 主动关闭TCP套接字

    bool is_connected() const noexcept;                 // 检查连接是否有效

private:
    int                  socket_fd_;    // 套接字文件描述符
    mutable std::mutex   mtx_;          // 互斥锁，用于保护socket_fd_
    std::shared_ptr<int> ref_count_;    // 引用计数，用于自动管理内存

};

#endif
