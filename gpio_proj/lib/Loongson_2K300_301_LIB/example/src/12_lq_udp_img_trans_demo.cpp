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
const std::string TARGET_IP    = "192.168.1.37";
// UDP目标端口（需与上位机保持一致）
const uint16_t    TARGET_PORT  = 8080;
// 摄像头参数
const uint16_t    CAM_WIDTH    = 160;     // 宽
const uint16_t    CAM_HEIGHT   = 120;     // 高
const uint16_t    CAM_FPS      = 120;     // 帧率
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
    printf("=========================================\n");
    printf("  UDP 图传\n");
    printf("=========================================\n");
    printf("Target IP:   %s\n", TARGET_IP.c_str());
    printf("Target Port: %d\n", TARGET_PORT);
    printf("Resolution:  %dx%d\n", CAM_WIDTH, CAM_HEIGHT);
    printf("FPS:         %d\n", CAM_FPS);
    printf("=========================================\n");
    // 初始化UDP客户端
    lq_udp_client udp_client(TARGET_IP, TARGET_PORT);
    printf("UDP 客户端初始化完成\n");
    // 初始化摄像头
    lq_camera_ex cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS, LQ_CAMERA_0CPU_MJPG);
    if (!cam.is_cam_opened()) {
        printf("ERROR: 打开摄像头失败!\n");
        return;
    }
    printf("摄像头参数: %dx%d @ %dfps\n", cam.get_camera_width(), cam.get_camera_height(), cam.get_camera_fps());
    // 发送帧计数
    uint32_t frame_count = 0, encoder_count = 0;
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    printf("图传开始... 按下 Ctrl+C 停止\n");

    while (ls_system_running.load()) {
        // ===================== 获取并发送图像 =====================
        // 获取原始图像
        cv::Mat frame = cam.get_frame_raw();
        if (frame.empty()) {
            printf("ERROR: 读取图像失败!\n");
            continue;
        }
        // 发送JPEG压缩图像
        if (udp_client.udp_send_image(frame, JPEG_QUALITY) < 0) {
            printf("ERROR: 发送图像失败!\n");
        }
        frame_count++;
        // ===================== 每秒打印状态 =====================
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed >= 1) {
            float fps = (float)frame_count / (float)elapsed;
            printf("FPS: %.2f\n", fps);
            frame_count = encoder_count = 0;
            start_time = now;
        }
    }
}
