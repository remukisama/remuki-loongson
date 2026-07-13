#include "user_app.hpp"
#include <atomic>
#include <iostream>
#include <memory>
#include <unistd.h>
#include "lq_camera_ex.hpp"

// 增强摄像头智能指针对象
std::unique_ptr<lq_camera_ex> cam_ex;
// 循迹用下采样帧（160x120）
cv::Mat frame;
// 高分辨率原始帧（320x240），供 NCNN 推理等下游使用
static cv::Mat frame_hires;

char img[60][80];
char img3[60][80];  //显示图像的数组
char img4[60][160];
char img5[120][160];


//摄像头初始化，成功返回true，失败返回false；
bool image_cv_Init(void)
{
    // 按配置采集；高速赛道模式使用160x120原生180fps，循迹尺寸保持不变。
    cam_ex = std::make_unique<lq_camera_ex>(
        car_config::kCameraHiresWidth,
        car_config::kCameraHiresHeight,
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
    
    printf("[CAM] hires:%d*%d, patrol:%d*%d, fps:%3f\n",
        frame_width, frame_height,
        car_config::kCameraWidth, car_config::kCameraHeight, frame_fps);
    sleep(1);
    return true;
}


bool image_cv_zip(void)
{
    if (!cam_ex) return false;

    // 读取摄像头一帧原始图像（尺寸由car_config配置）
    cv::Mat raw = cam_ex->get_frame_raw();
    if (raw.empty())
    {
        std::cerr << "Error read frame" << std::endl;
        return false;
    }

    // 翻转 180°（上下颠倒+左右镜像）保证透视关系正常
    cv::flip(raw, raw, -1);

    // ---- 保存高分辨率帧供 NCNN 推理等下游使用 ----
    // 这里只保留 Mat 引用计数，不做每帧深拷贝；异步消费者需要长期持有时自行 clone。
    frame_hires = raw;

    // ---- 下采样到 160x120 供循迹算法 ----
    cv::resize(raw, frame, cv::Size(car_config::kCameraWidth, car_config::kCameraHeight));

    // ---- 以下为既有循迹预处理管线（160x120） ----

    // ROI 裁切：只截取配置中的感兴趣区域，默认是下半部分图像
    cv::Mat roi_frame = frame(cv::Rect(
        car_config::kCameraRoiX,
        car_config::kCameraRoiY,
        car_config::kCameraRoiWidth,
        car_config::kCameraRoiHeight));

    cv::Mat binary;

    // 按 car_config::kImageAlgorithm 选择真实二值化链路：
    // 0 = HSV蓝色背景提取后反相；1 = 灰度静态阈值；2 = 灰度+OTSU自适应阈值。
    if (car_config::kImageAlgorithm == 0)
    {
        cv::Mat hsv, blue_mask;
        cv::cvtColor(roi_frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv,
            cv::Scalar(car_config::kBlueHueLower, car_config::kBlueSaturationLower, car_config::kBlueValueLower),
            cv::Scalar(car_config::kBlueHueUpper, car_config::kBlueSaturationUpper, car_config::kBlueValueUpper),
            blue_mask);
        // 蓝色赛道背景为0，非蓝色线/标记为1。
        cv::bitwise_not(blue_mask, binary);
    }
    else
    {
        cv::Mat gray;
        cv::cvtColor(roi_frame, gray, cv::COLOR_BGR2GRAY);

        if (car_config::kImageAlgorithm == 2)
        {
            const int blur_kernel = (car_config::kImageOtsuBlurKernel % 2 == 1)
                ? car_config::kImageOtsuBlurKernel
                : (car_config::kImageOtsuBlurKernel + 1);
            if (blur_kernel > 1) {
                cv::GaussianBlur(gray, gray, cv::Size(blur_kernel, blur_kernel), 0);
            }
            cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        }
        else
        {
            cv::threshold(gray, binary, car_config::kImageThresholdStatic, 255, cv::THRESH_BINARY);
        }
    }

    // 统一形态学后处理：先开运算去孤点，再闭运算补小孔；kernel<=1时跳过。
    if (car_config::kImageMorphKernel > 1)
    {
        static int cached_morph_kernel = 0;
        static cv::Mat cached_kernel;
        const int morph_kernel = (car_config::kImageMorphKernel % 2 == 1)
            ? car_config::kImageMorphKernel
            : (car_config::kImageMorphKernel + 1);
        if (cached_kernel.empty() || cached_morph_kernel != morph_kernel) {
            cached_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel, morph_kernel));
            cached_morph_kernel = morph_kernel;
        }
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, cached_kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, cached_kernel);
    }

    // 指针快速遍历：roi_frame 已经是 160x60，行数 0~59
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

const cv::Mat &image_cv_get_hires_frame()
{
    return frame_hires;
}
