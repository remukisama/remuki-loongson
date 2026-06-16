#include "lq_net_image_trans.hpp"
#include "lq_assert.hpp"
#include <cstring>
#include <sstream>
#include <algorithm>
#include <fcntl.h>
#include "lq_signal_handle.hpp"

/********************************************************************************
 * @brief   默认构造函数.
 * @param   none.
 * @return  none.
 * @example lq_http_image_server server;
 * @note    none.
 ********************************************************************************/
lq_http_image_server::lq_http_image_server() noexcept : server_fd_(-1), running_(false), port_(0)
{
}

/********************************************************************************
 * @brief   有参构造函数.
 * @param   port : HTTP服务器端口号.
 * @return  none.
 * @example lq_http_image_server server(8080);
 * @note    none.
 ********************************************************************************/
lq_http_image_server::lq_http_image_server(uint16_t port) noexcept : server_fd_(-1), running_(false), port_(0)
{
    this->start(port);
}

/********************************************************************************
 * @brief   析构函数.
 * @param   none.
 * @return  none.
 * @note    停止服务器并清理资源.
 ********************************************************************************/
lq_http_image_server::~lq_http_image_server() noexcept
{
    this->cleanup();
}

/********************************************************************************
 * @brief   启动HTTP服务器.
 * @param   port : HTTP服务器端口, 默认8080.
 * @return  bool, 启动成功返回true, 失败返回false.
 * @example server.start(8080);
 * @note    在指定端口启动HTTP服务器，用于在浏览器中查看图像流
 ********************************************************************************/
bool lq_http_image_server::start(uint16_t port)
{
    std::lock_guard<std::mutex> lock(this->server_mtx_);
    // 检查服务器是否已经运行
    if (this->running_) {
        lq_log_warn("HTTP 服务器已启动, 端口: %d", this->port_);
        return true;
    }
    // 创建TCP套接字
    this->server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (this->server_fd_ == -1) {
        lq_log_error("HTTP 服务器套接字创建失败");
        return false;
    }
    // 设置端口复用
    int val = 1;
    if (setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) == -1) {
        lq_log_error("HTTP 服务器设置端口复用失败");
        goto lq_http_server_error;
    }
    // 绑定地址
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 监听所有网络接口
    server_addr.sin_port = htons(port);
    if (bind(this->server_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        lq_log_error("HTTP 服务器绑定地址失败, 端口: %d", port);
        goto lq_http_server_error;
    }
    // 开始监听
    if (listen(this->server_fd_, 10) == -1) {
        lq_log_error("HTTP 服务器监听失败");
        goto lq_http_server_error;
    }
    // 启动服务器线程
    this->running_ = true;
    this->port_ = port;
    this->server_thread_ = std::thread(&lq_http_image_server::server_loop, this);
    lq_log_info("HTTP 服务器已启动, 端口: %d", port);
    return true;
lq_http_server_error:
    close(this->server_fd_);
    this->server_fd_ = -1;
    return false;
}

/********************************************************************************
 * @brief   停止HTTP服务器.
 * @param   none.
 * @return  none.
 * @example server.stop();
 * @note    停止HTTP服务器并清理所有相关资源
 ********************************************************************************/
void lq_http_image_server::stop() noexcept
{
    {
        std::lock_guard<std::mutex> lock(this->server_mtx_);
        if (!this->running_) {
            lq_log_warn("HTTP 服务器未启动");
            return;
        }
        this->running_ = false;
        // 关闭服务器套接字，唤醒阻塞在accept的线程
        if (this->server_fd_ >= 0) {
            close(this->server_fd_);
            this->server_fd_ = -1;
        }
    }
    // 等待服务器线程结束（在锁外等待，避免死锁）
    if (this->server_thread_.joinable()) {
        this->server_thread_.join();
    }
    // 关闭所有客户端连接
    {
        std::lock_guard<std::mutex> clients_lock(this->clients_mtx_);
        for (int client_fd : this->clients_) {
            close(client_fd);
        }
        this->clients_.clear();
    }
    lq_log_info("HTTP 服务器已停止");
}

/********************************************************************************
 * @brief   清理HTTP服务器资源.
 * @param   none.
 * @return  none.
 * @example server.cleanup();
 * @note    停止HTTP服务器并清理所有相关资源
 ********************************************************************************/
void lq_http_image_server::cleanup()
{
    this->stop();
}

/********************************************************************************
 * @brief   服务器主循环.
 * @param   none.
 * @return  none.
 * @note    内部方法，持续接受客户端连接并处理请求
 ********************************************************************************/
void lq_http_image_server::server_loop()
{
    // 设置套接字为非阻塞模式，以便能够及时响应信号
    int flags = fcntl(this->server_fd_, F_GETFL, 0);
    fcntl(this->server_fd_, F_SETFL, flags | O_NONBLOCK);
    // 持续接受客户端连接
    while (this->running_ && ls_system_running.load()) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        // 接受客户端连接
        int client_fd = accept(this->server_fd_, (struct sockaddr*)&client_addr, &client_addr_len);
        if (client_fd == -1) {
            usleep(10);
            continue;
        }
        // 添加到客户端列表
        {
            std::lock_guard<std::mutex> lock(this->clients_mtx_);
            this->clients_.push_back(client_fd);
        }
        // 在独立线程中处理客户端请求
        std::thread(&lq_http_image_server::handle_client, this, client_fd).detach();
    }
}

/********************************************************************************
 * @brief   处理HTTP客户端连接.
 * @param   client_fd : 客户端套接字文件描述符.
 * @return  none.
 * @note    内部方法，解析HTTP请求并持续推送MJPEG图像流
 ********************************************************************************/
void lq_http_image_server::handle_client(int client_fd)
{
    // 统一资源清理Lambda
    auto clean_client = [this, client_fd](const char* reason) {
        close(client_fd);
        std::lock_guard<std::mutex> lock(this->clients_mtx_);
        auto it = std::find(this->clients_.begin(), this->clients_.end(), client_fd);
        if (it != this->clients_.end())
        {
            this->clients_.erase(it);
        }
        lq_log_info("HTTP 客户端 %d 已断开：%s", client_fd, reason);
    };
    char buffer[1024] = {0};
    if (read(client_fd, buffer, sizeof(buffer) - 1) <= 0) {
        clean_client("读取请求失败");
        return;
    }
    // ====================== 过滤浏览器自动请求 ======================
    if (strstr(buffer, "GET /favicon.ico") != nullptr) {
        close(client_fd);
        return;
    }
    if (strstr(buffer, "HEAD /") != nullptr) {
        close(client_fd);
        return;
    }
    // 判断是否为MJPEG流请求以及超时时间
    bool is_stream_request = strstr(buffer, "/stream") != nullptr;
    const int CONNECTION_TIMEOUT = 45;
    if (is_stream_request) {
        // MJPEG 标准响应头
        std::string response =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "Connection: keep-alive\r\n"
            "Cache-Control: no-cache, no-store, must-revalidate\r\n"
            "Pragma: no-cache\r\n"
            "Expires: 0\r\n"
            "Access-Control-Allow-Origin: *\r\n\r\n";
        if (send(client_fd, response.c_str(), response.length(), MSG_NOSIGNAL) <= 0) {
            lq_log_error("HTTP 客户端 %d 写入响应头失败", client_fd);
            clean_client("响应头发送失败");
            return;
        }
        lq_log_info("HTTP 客户端 %d 已连接，请求MJPEG流", client_fd);
        time_t last_activity = time(nullptr);
        std::vector<uint8_t> last_frame_data;
        // 无帧率限制：有新帧立刻发送，全速推流
        while (this->running_ && ls_system_running.load()) {
            // 空闲超时检测
            if (time(nullptr) - last_activity > CONNECTION_TIMEOUT) {
                lq_log_info("HTTP 客户端 %d 连接超时", client_fd);
                break;
            }
            // 最小锁粒度获取最新帧
            std::vector<uint8_t> frame_data;
            {
                std::lock_guard<std::mutex> lock(this->frame_mtx_);
                frame_data = this->frame_buffer_;
            }
            // 无帧时极短休眠，不空转、不占高CPU; 相同帧跳过，减少无效发包
            if ((frame_data.empty()) || last_frame_data == frame_data) {
                usleep(1000);
                continue;
            }
            // 构造MJPEG帧分隔头
            std::string frame_header =
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: " + std::to_string(frame_data.size()) + "\r\n\r\n";
            // 分段安全发送
            if (send(client_fd, frame_header.c_str(), frame_header.length(), MSG_NOSIGNAL) <= 0) break;
            if (send(client_fd, frame_data.data(), frame_data.size(), MSG_NOSIGNAL) <= 0) break;
            if (send(client_fd, "\r\n", 2, MSG_NOSIGNAL) <= 0) break;
            // 更新状态
            last_activity = time(nullptr);
            last_frame_data.swap(frame_data);
        }
        clean_client("MJPEG流连接结束");
    } else {
        // ===================== HTML界面+样式+龙邱UI =====================
        std::string html_response = "HTTP/1.1 200 OK\r\n";
        html_response += "Content-Type: text/html\r\n";
        html_response += "Connection: close\r\n";
        html_response += "\r\n";
        html_response += "<!DOCTYPE html>\r\n";
        html_response += "<html lang=\"zh-CN\">\r\n";
        html_response += "<head>\r\n";
        html_response += "<meta charset=\"UTF-8\">\r\n";
        html_response += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\r\n";
        html_response += "<title>摄像头图像流 - 龙邱科技</title>\r\n";
        html_response += "<style>\r\n";
        html_response += "* { margin: 0; padding: 0; box-sizing: border-box; }\r\n";
        html_response += "body { background-color: #121212; min-height: 100vh; display: flex; flex-direction: column; justify-content: center; align-items: center; font-family: system-ui, sans-serif; padding: 20px; }\r\n";
        html_response += ".header { position: fixed; top: 0; left: 0; right: 0; background: rgba(0,0,0,0.8); padding: 30px 20px; text-align: center; z-index: 100; }\r\n";
        html_response += ".company-name { color: #3b82f6; font-size: 32px; font-weight: bold; letter-spacing: 3px; text-shadow: 0 0 20px rgba(59, 130, 246, 0.5); font-family: 'Arial', sans-serif; }\r\n";
        html_response += ".stream-container { display: flex; justify-content: center; align-items: center; border-radius: 16px; overflow: hidden; box-shadow: 0 12px 40px rgba(59, 130, 246, 0.2); border: 2px solid rgba(59, 130,246, 0.3); margin: 100px 0 20px; }\r\n";
        html_response += "#stream-img { width: 640px; height: 480px; object-fit: fill; display: block; }\r\n";
        html_response += ".footer { position: fixed; bottom: 0; left: 0; right: 0; background: rgba(0,0,0,0.8); padding: 15px; text-align: center; z-index: 100; }\r\n";
        html_response += ".footer-text { color: #3b82f6; font-size: 14px; }\r\n";
        html_response += "</style>\r\n";
        html_response += "</head>\r\n";
        html_response += "<body>\r\n";
        html_response += "<div class=\"header\">\r\n";
        html_response += "<div class=\"company-name\">龙邱科技</div>\r\n";
        html_response += "</div>\r\n";
        html_response += "<div class=\"stream-container\">\r\n";
        html_response += "<img id=\"stream-img\" src=\"/stream\" alt=\"图像流\">\r\n";
        html_response += "</div>\r\n";
        html_response += "<div class=\"footer\">\r\n";
        html_response += "<div class=\"footer-text\">© 2026 龙邱科技 - 摄像头图像流</div>\r\n";
        html_response += "</div>\r\n";
        html_response += "</body>\r\n";
        html_response += "</html>\r\n";
        // =============================================================================
        send(client_fd, html_response.c_str(), html_response.length(), MSG_NOSIGNAL);
        lq_log_info("HTTP 客户端 %d 已连接，发送HTML页面", client_fd);
        clean_client("HTML页面请求完成");
    }
}

/********************************************************************************
 * @brief   推送OpenCV图像到HTTP流（自动JPEG编码）.
 * @param   img     : OpenCV图像矩阵.
 * @param   quality : JPEG编码质量 (1-100), 默认80.
 * @return  bool, 推送成功返回true, 失败返回false.
 * @example server.push_frame(frame, 80);
 * @note    自动将OpenCV图像编码为JPEG并推送到HTTP流
 ********************************************************************************/
bool lq_http_image_server::push_frame(const cv::Mat &img, int quality)
{
    // 检查图像是否有效
    if (img.empty()) {
        lq_log_error("图像为空");
        return false;
    }
    // 检查服务器是否运行
    if (!this->running_) {
        lq_log_error("HTTP 服务器未启动");
        return false;
    }
    // 编码为JPEG
    std::vector<uint8_t> jpeg_buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
    cv::Mat small_img;
    cv::resize(img, small_img, cv::Size(160, 120), 0, 0, cv::INTER_LINEAR);
    if (!cv::imencode(".jpg", small_img, jpeg_buf, params)) {
        lq_log_error("JPEG 编码图像失败");
        return false;
    }
    // 更新帧缓冲区
    std::lock_guard<std::mutex> lock(this->frame_mtx_);
    this->frame_buffer_ = jpeg_buf;
    
    return true;
}

/********************************************************************************
 * @brief   检查服务器是否运行.
 * @param   none.
 * @return  bool, 服务器运行返回true, 否则返回false.
 * @example bool running = server.is_running();
 * @note    none.
 ********************************************************************************/
bool lq_http_image_server::is_running() const noexcept
{
    std::lock_guard<std::mutex> lock(this->server_mtx_);
    return this->running_;
}

/********************************************************************************
 * @brief   获取服务器端口.
 * @param   none.
 * @return  uint16_t, 服务器端口号.
 * @example uint16_t port = server.get_port();
 * @note    none.
 ********************************************************************************/
uint16_t lq_http_image_server::get_port() const noexcept
{
    std::lock_guard<std::mutex> lock(this->server_mtx_);
    return this->port_;
}
