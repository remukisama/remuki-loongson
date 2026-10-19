#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_http_img_trans_demo.cpp
 * @brief   HTTP 图像传输测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 TCP 图像传输功能，用于测试 HTTP 图像传输的基本功能.
 * @note    可先查看当前板卡 IP，然后再运行程序
 *          随后可直接在浏览器输入 http://<IP>:<PORT> 查看图像
 *          默认端口为 8080，可在程序中修改.
 ********************************************************************************/

/********************************************************************************
 *  @brief   HTTP 图像传输测试.
 *  @param   none.
 *  @return  none.
 *  @note    测试内容为 HTTP 图像传输，使用 OpenCV 读取摄像头图像，并使用 HTTP 发送图像数据.
 ********************************************************************************/
void lq_http_img_trans_demo(void)
{
    cv::Mat frame;
    // 初始化 HTTP 服务器
    lq_http_image_server server(8080);
    // 初始化摄像头
    lq_camera_ex cam(320, 240, 120, LQ_CAMERA_0CPU_MJPG);
    if (!cam.is_cam_opened()) {
        printf("ERROR: 打开摄像头失败!\n");
        return;
    }
    while (ls_system_running.load())
    {
        // 获取一帧图像
        frame = cam.get_frame_raw();
        if (frame.empty()) {
            printf("ERROR: 读取摄像头图像失败!\n");
            continue;
        }
        // 发送图像
        server.push_frame(frame, 50);
    }
}
