#ifndef __LQ_UDP_CLIENT_HPP
#define __LQ_UDP_CLIENT_HPP

#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <memory>
#include <mutex>

// OpenCV头文件（可选编译）
#ifdef LQ_HAVE_OPENCV
    #include <opencv2/opencv.hpp>
#endif

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class lq_udp_client
{
public:
    lq_udp_client() noexcept;                               // 默认构造函数
    lq_udp_client(const std::string _ip, uint16_t _port);   // 有参构造函数

    lq_udp_client(const lq_udp_client &_other) noexcept;            // 复制构造函数
    lq_udp_client &operator=(const lq_udp_client &_other) noexcept; // 赋值运算符重载

    ~lq_udp_client();

public:
    void udp_client_init(const std::string _ip, uint16_t _port);    // 初始化UDP客户端

    ssize_t udp_send(const void *_buf, size_t _len);    // 发送数据
    ssize_t udp_recv(void *_buf, size_t _len);          // 接收数据

    ssize_t udp_send_string(const std::string &_str);               // 发送字符串

    #ifdef LQ_HAVE_OPENCV
    ssize_t udp_send_image(const cv::Mat &_img, int _quality = 80); // 发送图片（OpenCV图像，JPEG压缩）
    #endif

    int get_udp_socket_fd() const noexcept;             // 获取UDP套接字文件描述符

    void udp_close() noexcept;                          // 主动关闭UDP套接字

private:
    int                  socket_fd_;    // 套接字文件描述符
    struct sockaddr_in   addr_;         // 服务器地址信息结构体
    mutable std::mutex   mtx_;          // 互斥锁，用于保护socket_fd_
    std::shared_ptr<int> ref_count_;    // 引用计数，用于自动管理内存

};

#endif
