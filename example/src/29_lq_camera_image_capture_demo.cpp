#include <iostream>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "lq_camera_ex.hpp"

void lq_camera_capture_demo(void)
{
    std::cout << "====== 龙芯2K0300/2K301 摄像头基础拍照测试 ======" << std::endl;

    // 使用了 LQ_CAMERA_0CPU_MJPG 降低CPU占用
    lq_camera_ex camera(160, 120, 180, LQ_CAMERA_0CPU_MJPG);

    std::cout << "正在启动摄像头采集..." << std::endl;
    if (camera.start_collect() != 0)
    {
        std::cerr << "摄像头启动失败！请检查摄像头是否连接。" << std::endl;
        return;
    }

    // 使用增强版摄像头类接口设置手动曝光
    std::cout << "正在配置手动曝光..." << std::endl;
    if (!camera.set_exposure_manual(100))
    {
        std::cerr << "警告: 设置手动曝光失败" << std::endl;
    } 

    // 延时等待硬件配置生效
    usleep(500 * 1000);

    cv::Mat frame;
    int img_index = 0;

    std::cout << "\n------------------------------------------" << std::endl;
    std::cout << " 按下回车键 (Enter) 拍摄一张照片并保存。" << std::endl;
    std::cout << " 输入 'q' 并回车退出程序。" << std::endl;
    std::cout << "------------------------------------------\n等待输入: ";

    while (true)
    {
        std::string input;
        std::getline(std::cin, input);
        if (input == "q" || input == "Q")
        {
            break;
        }

        std::cout << "正在抓取图像帧..." << std::endl;
        
        // 连续抓取几次以清空缓存，保证获得按下按键时的最新图像
        for (int i = 0; i < 3; i++) {
            frame = camera.get_frame_raw();
        }

        frame = camera.get_frame_raw();
        if (!frame.empty())
        {
            // 如果摄像头物理安放是倒置的，使用 cv::flip 旋转 180 度 (上下和左右都翻转)
            cv::flip(frame, frame, -1);

            std::string filename = "test_image_" + std::to_string(img_index++) + "_160x120.jpg";
            // 写入本地保存为jpg格式
            if (cv::imwrite(filename, frame))
            {
                std::cout << "图像抓取成功！已保存至: " << filename << std::endl;
            }
            else
            {
                std::cerr << "图像保存失败，请检查目录读写权限。" << std::endl;
            }
        }
        else
        {
            std::cerr << "获取图像帧失败或图像为空！" << std::endl;
        }
        std::cout << "\n继续按回车键拍照，输入 'q' 退出: ";
    }

    std::cout << "停止摄像头采集..." << std::endl;
    camera.stop_collect();
}
