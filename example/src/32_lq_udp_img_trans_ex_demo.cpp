#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    32_lq_udp_img_trans_ex_demo.cpp
 * @brief   UDP 图像传输测试（增强版摄像头 lq_camera_ex）.
 * @author  龙邱科技-012
 * @date    2026-04-23
 * @version V1.0.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 使用 lq_camera_ex 增强版摄像头类实现 UDP 图像传输功能.
 ********************************************************************************/

// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
// 目标IP地址（UDP接收端）
const std::string EX_TARGET_IP    = "192.168.255.22";
// UDP目标端口
const uint16_t    EX_TARGET_PORT  = 8080;
// 摄像头参数
const uint16_t    EX_CAM_WIDTH    = 160;     // 宽
const uint16_t    EX_CAM_HEIGHT   = 120;     // 高
const uint16_t    EX_CAM_FPS      = 180;     // 帧率
// JPEG编码质量 (1-100)
const uint8_t     EX_JPEG_QUALITY = 30;

/********************************************************************************
 *  @brief   UDP 图像传输测试（增强版摄像头）.
 *  @param   none.
 *  @return  none.
 *  @note    使用 lq_camera_ex 增强版摄像头类读取图像，并通过 UDP 发送图像数据.
 *! @note    使用时需搭配对应上位机 LoongHost.exe，用于接收并显示图像.
 ********************************************************************************/
void lq_udp_img_trans_ex_demo(void)
{
    printf("=========================================\r\n");
    printf("  UDP Camera_EX + Encoder Stream\r\n");
    printf("=========================================\r\n");
    printf("Target IP:   %s\r\n", EX_TARGET_IP.c_str());
    printf("Target Port: %d\r\n", EX_TARGET_PORT);
    printf("Resolution:  %dx%d\r\n", EX_CAM_WIDTH, EX_CAM_HEIGHT);
    printf("FPS:         %d\r\n", EX_CAM_FPS);
    printf("=========================================\r\n");

    // 初始化UDP客户端
    lq_udp_client udp_client;
    udp_client.udp_client_init(EX_TARGET_IP, EX_TARGET_PORT);
    printf("UDP client initialized\r\n");

    // 初始化增强版摄像头（默认使用高帧率 MJPG 模式）
    lq_camera_ex cam(EX_CAM_WIDTH, EX_CAM_HEIGHT, EX_CAM_FPS);
    if (!cam.is_cam_opened()) {
        printf("ERROR: Failed to open camera!\r\n");
        return;
    }
    printf("Camera opened: %dx%d @ %dfps\r\n",
           cam.get_camera_width(), cam.get_camera_height(), cam.get_camera_fps());

    // 开始采集
    if (cam.start_collect() < 0) {
        printf("ERROR: Failed to start camera collect!\r\n");
        return;
    }
    printf("Camera collect started\r\n");

    // 发送帧计数
    uint32_t frame_count = 0;
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();

    printf("Start streaming... Press Ctrl+C to stop\r\n");

    while (true) {
        // ===================== 获取并发送图像 =====================
        cv::Mat frame = cam.get_frame_raw();
        if (frame.empty()) {
            printf("ERROR: Failed to read frame\r\n");
            continue;
        }

        // 发送JPEG压缩图像
        ssize_t sent = udp_client.udp_send_image(frame, EX_JPEG_QUALITY);
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
            start_time = now;
        }
    }

    // 停止采集（实际上不会执行到这里，因为上面是无限循环）
    cam.stop_collect();
}
