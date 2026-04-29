#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_udp_img_trans_demo.cpp
 * @brief   UDP 图像传输测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 UDP 图像传输功能，用于测试 UDP 图像传输的基本功能.
 ********************************************************************************/

// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
// 目标IP地址（UDP接收端）
const std::string TARGET_IP    = "192.168.77.22";
// UDP目标端口
const uint16_t    TARGET_PORT  = 8080;
// 摄像头参数
const uint16_t    CAM_WIDTH    = 160;     // 宽
const uint16_t    CAM_HEIGHT   = 120;     // 高
const uint16_t    CAM_FPS      = 120;     // 帧率
// 方块参数（画面中间）
const uint16_t    RECT_SIZE    = 20;
// JPEG编码质量 (1-100)
const uint8_t     JPEG_QUALITY = 30;

/********************************************************************************
 *  @brief   UDP 图像传输测试.
 *  @param   none.
 *  @return  none.
 *  @note    测试内容为 UDP 图像传输，使用OpenCV 读取摄像头图像，并使用 UDP 发送图像数据.
 *! @note    使用时需搭配对应上位机 LoongHost.exe，用于接收并显示图像.
 ********************************************************************************/
void lq_udp_img_trans_demo(void)
{
    printf("=========================================\r\n");
    printf("  UDP Camera + Encoder Stream\r\n");
    printf("=========================================\r\n");
    printf("Target IP:   %s\r\n", TARGET_IP.c_str());
    printf("Target Port: %d\r\n", TARGET_PORT);
    printf("Resolution:  %dx%d\r\n", CAM_WIDTH, CAM_HEIGHT);
    printf("FPS:         %d\r\n", CAM_FPS);
    printf("=========================================\r\n");

    // 初始化UDP客户端
    lq_udp_client udp_client;
    udp_client.udp_client_init(TARGET_IP, TARGET_PORT);
    printf("UDP client initialized\r\n");

    // 初始化摄像头
    lq_camera cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
    if (!cam.is_opened()) {
        printf("ERROR: Failed to open camera!\r\n");
        return;
    }
    printf("Camera opened: %dx%d @ %dfps\r\n", cam.get_width(), cam.get_height(), cam.get_fps());

    // 发送帧计数
    uint32_t frame_count = 0;
    uint32_t encoder_count = 0;
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();

    printf("Start streaming... Press Ctrl+C to stop\r\n");
    // printf("端口号：%d\n",cam.get_dev_id());

    while (true) {
        // ===================== 获取并发送图像 =====================
        // 获取原始图像
        cv::Mat frame = cam.get_binary_frame();
        if (frame.empty()) {
            printf("ERROR: Failed to read frame\r\n");
            continue;
        }

        // // 在画面中间画方块
        // int rows = frame.rows;
        // int cols = frame.cols;
        // int x1 = (cols - RECT_SIZE) / 2;
        // int y1 = (rows - RECT_SIZE) / 2;
        // cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x1 + RECT_SIZE, y1 + RECT_SIZE), cv::Scalar(0, 255, 0), 2);

        // 发送JPEG压缩图像
        ssize_t sent = udp_client.udp_send_image(frame, JPEG_QUALITY);
        if (sent < 0) {
            printf("ERROR: Failed to send image\r\n");
        }

        frame_count++;

        // ===================== 每秒打印状态 =====================
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed >= 1) {
            float fps = (float)frame_count / (float)elapsed;
            printf("FPS: %.2f\r\n", fps);
            frame_count = 0;
            encoder_count = 0;
            start_time = now;
        }
    }
}
