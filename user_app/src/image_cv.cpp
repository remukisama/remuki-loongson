#include "user_app.hpp"
#include <atomic>
#include <iostream>
#include <memory>
#include <unistd.h>
#include "lq_camera_ex.hpp"

// 创建一个增强摄像头智能指针对象
std::unique_ptr<lq_camera_ex> cam_ex;
// 获取视频流
cv::Mat frame;

char img[60][80];
char img3[60][80];  //显示图像的数组
char img4[60][160];
char img5[120][160];


//摄像头初始化，成功返回true，失败返回false；
bool image_cv_Init(void)
{
    // 摄像头格式、分辨率、帧率和设备路径统一在 car_config.hpp 中配置
    cam_ex = std::make_unique<lq_camera_ex>(
        car_config::kCameraWidth,
        car_config::kCameraHeight,
        car_config::kCameraFps,
        car_config::kCameraFormat,
        car_config::kCameraDevicePath);

    if (!cam_ex->is_cam_opened())
    {
        std::cerr << "Error open video stream" << std::endl;
        return false;
    }
    
    // 启动图像采集
    if (cam_ex->start_collect() < 0) 
    {
        std::cerr << "Error start collect" << std::endl;
        return false;
    }

    // 通过增强版摄像头类接口设置手动曝光
    if (!cam_ex->set_exposure_manual(car_config::kCameraManualExposure))
    {
        std::cerr << "警告: 设置手动曝光失败" << std::endl;
    }

    // 获取并打印设定成功的摄像头图像宽高和帧率信息
    int frame_width = cam_ex->get_camera_width();
    int frame_height = cam_ex->get_camera_height();
    double frame_fps = (double)cam_ex->get_camera_fps();
    
    printf("frame:%d*%d, fps:%3f\n", frame_width, frame_height, frame_fps);
    sleep(1);
    return true;
}


bool image_cv_zip(void)
{
    if (!cam_ex) return false;

    // 读取摄像头一帧原始图像
    frame = cam_ex->get_frame_raw();
    if (frame.empty())
    {
        std::cerr << "Error read frame" << std::endl;
        return false;
    }

    // ================= 核心优化点 1：解决倒像问题 =================
    // 将图像翻转180度（上下颠倒+左右镜像）保证透视关系正常
    cv::flip(frame, frame, -1);

    // ================= 核心优化点 2：在处理前先裁切感兴趣区域(ROI) =================
    // 只截取配置中的感兴趣区域，默认是下半部分图像
    // 处理的数据量直接减半，极大降低嵌入式 CPU 负载
    cv::Mat roi_frame = frame(cv::Rect(
        car_config::kCameraRoiX,
        car_config::kCameraRoiY,
        car_config::kCameraRoiWidth,
        car_config::kCameraRoiHeight));

    cv::Mat hsv, blue_mask, binary;
    // 使用裁剪后的小图(roi_frame)进行 HSV 转换
    cv::cvtColor(roi_frame, hsv, cv::COLOR_BGR2HSV);

    // 1. 提取蓝色背景
    cv::inRange(hsv,
        cv::Scalar(car_config::kBlueHueLower, car_config::kBlueSaturationLower, car_config::kBlueValueLower),
        cv::Scalar(car_config::kBlueHueUpper, car_config::kBlueSaturationUpper, car_config::kBlueValueUpper),
        blue_mask);

    // 2. 反相 (蓝色变黑，非蓝色变白)
    cv::bitwise_not(blue_mask, binary);

    // 3. 降噪 (滤除躁点，提高寻线稳定性)
    cv::GaussianBlur(binary, binary, cv::Size(3, 3), 0);

    // ================= 核心优化点 3：指针快速遍历 =================
    // 由于 roi_frame 已经是 160x60，所以行数从 0~59
    for(int i = 0; i < car_config::kCameraRoiHeight; i++)
    {
        // 获取第 i 行首地址，跳过 OpenCV at() 安全检查，遍历速度极快
        uchar* row_ptr = binary.ptr<uchar>(i); 
        
        for(int j = 0, k = 0; j < car_config::kCameraRoiWidth; j += car_config::kCameraColumnSampleStep, k++)
        {
            // 通过指针直接取值压缩
            char val = row_ptr[j] ? 1 : 0;
            img1[i][k] = val;
            img3[i][k] = val;
        }
    }
    
    return true;
}
