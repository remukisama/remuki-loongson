#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_ips20_show_img_demo.cpp
 * @brief   显示摄像头图像到 IPS20 屏幕上.
 * @author  龙邱科技-012
 * @date    2026-03-18
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 显示摄像头图像到 IPS20 屏幕.
 ********************************************************************************/

/********************************************************************************
 * @brief   显示摄像头图像到 IPS20 屏幕上.
 * @param   none.
 * @return  none.
 * @note    摄像头图像会被实时显示在 IPS20 屏幕上，用户可以通过调整摄像头参数来改变显示效果.
 * @note    使用了两种图像获取方式，可以根据需要选择其中一种.
 ********************************************************************************/
void lq_ips20_show_img_demo(void)
{
    cv::Mat frame, gray;
    lq_camera_ex cam(160, 120, 180);

    lq_ips20_drv_init(1);
    lq_ips20_drv_cls(U16BLUE);

    // cam.set_exposure_manual(50); // 设置手动曝光

    sleep(1);
    while (ls_system_running.load())
    {
        // frame = cam.get_frame_raw();
        cam.get_frame_raw_gray(frame, gray);
        lq_ips20_drv_road_color(0, 0, frame);
    }
}
