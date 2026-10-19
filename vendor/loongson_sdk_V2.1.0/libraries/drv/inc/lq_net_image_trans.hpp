#ifndef __LQ_NET_IMAGE_TRANS_HPP
#define __LQ_NET_IMAGE_TRANS_HPP

#include <iostream>
#include <string>
#include <cstdint>
#include <cstdlib>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>
#include "lq_common.hpp"

/****************************************************************************************************
 * @brief   HTTP图像流服务器类
 * @details 实现一个轻量级的HTTP服务器，通过浏览器查看OpenCV图像流
 * @note    使用MJPEG格式推送图像，不依赖第三方库
 * @example 示例代码如下:
 * @code
 *      lq_http_image_server server(8080);                      // 构造HTTP服务器，监听端口8080
 *      cv::Mat frame;                                          // 用于存储图像帧的Mat对象
 *      lq_camera_ex cam(320, 240, 120, LQ_CAMERA_0CPU_MJPG);   // 构造相机对象，分辨率320x240，帧率120，使用0CPU MJPEG模式
 *      while (ls_system_running.load())                        // 循环获取图像帧并推送
 *      {
 *          frame = cam.get_frame_raw();                        // 获取图像帧
 *          server.push_frame(frame, 50);                       // 推送图像帧，质量50
 *      }
 * @endcode
 ****************************************************************************************************/
class lq_http_image_server : public lq_auto_cleanup
{
public:
    lq_http_image_server() noexcept;                        // 默认构造函数
    explicit lq_http_image_server(uint16_t port) noexcept;  // 有参构造函数
    ~lq_http_image_server() noexcept;                       // 析构函数

public:
    bool start(uint16_t port = 8080);       // 启动HTTP服务器
    void stop() noexcept;                   // 停止HTTP服务器
    
    bool push_frame(const cv::Mat &img, int quality = 50);  // 推送图像帧（应为JPEG格式）
    
    bool     is_running() const noexcept;   // 检查服务器是否运行
    uint16_t get_port()   const noexcept;   // 获取服务器端口
    void     cleanup() override;            // 清理资源

private:
    void server_loop();                     // 服务器主循环
    void handle_client(int client_fd);      // 处理客户端连接

public:
    lq_http_image_server(const lq_http_image_server &_other) = delete;              // 禁用复制构造函数
    lq_http_image_server(lq_http_image_server &&_other)      = delete;              // 禁用移动构造函数
    lq_http_image_server &operator=(const lq_http_image_server &_other) = delete;   // 禁用赋值运算符重载
    lq_http_image_server &operator=(lq_http_image_server &&_other)      = delete;   // 禁用移动赋值运算符重载

private:
    int                  server_fd_;        // 服务器套接字文件描述符
    bool                 running_;          // 服务器运行状态
    uint16_t             port_;             // 服务器端口
    std::thread          server_thread_;    // 服务器线程
    mutable std::mutex   server_mtx_;       // 保护服务器状态的互斥锁
    std::vector<int>     clients_;          // 活跃客户端连接列表
    mutable std::mutex   clients_mtx_;      // 保护客户端列表的互斥锁
    std::vector<uint8_t> frame_buffer_;     // 图像帧缓冲区
    mutable std::mutex   frame_mtx_;        // 保护帧缓冲区的互斥锁
};

#endif
