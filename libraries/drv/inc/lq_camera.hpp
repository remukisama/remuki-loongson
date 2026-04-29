#ifndef _LQ_CAMERA_HPP
#define _LQ_CAMERA_HPP

#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <mutex>

#ifdef LQ_HAVE_OPENCV
    #include <opencv2/opencv.hpp>
    #define LQ_OPENCV_AVAILABLE 1
#else
    #define LQ_OPENCV_AVAILABLE 0
#endif

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class lq_camera
{
public:
    // 通过 ID 初始化摄像头对象
    lq_camera(uint16_t _width, uint16_t _height, uint16_t _fps, int _dev_id = 0);
    
    lq_camera(const lq_camera&) = delete;
    lq_camera& operator=(const lq_camera&) = delete;

    // 析构函数
    ~lq_camera();

public:
    bool open(uint16_t _width, uint16_t _height, uint16_t _fps);    // 打开摄像头
    void close();                                                   // 关闭摄像头

#if LQ_OPENCV_AVAILABLE
    cv::Mat get_raw_frame();                                            // 获取原始图像
    cv::Mat get_gray_frame();                                           // 获取灰度图像
    cv::Mat get_binary_frame(double _thresh = 127, double _max = 255);  // 获取二值图像

    std::vector<uint8_t> get_raw_frame_data();      // 获取原始图像数据
    std::vector<uint8_t> get_gray_frame_data();     // 获取灰度图像数据
    std::vector<uint8_t> get_binary_frame_data(double _thresh = 127, double _max = 255);   // 获取二值图像数据
#endif

    bool is_opened() const;             // 判断摄像头是否已打开

public:
    uint16_t get_height() const;        // 获取图像高度
    uint16_t get_width() const;         // 获取图像宽度
    uint16_t get_fps() const;           // 获取帧率
    int      get_dev_id() const;        // 获取设备 ID

private:
#if LQ_OPENCV_AVAILABLE
    bool read_new_frame();                          // 读取新帧
    bool is_mat_valid(const cv::Mat& mat) const;    // 判断图像是否有效
#endif

private:
#if LQ_OPENCV_AVAILABLE
    cv::VideoCapture   cap;             // 摄像头对象
    cv::Mat            frame;           // 原始图像
    cv::Mat            gray_frame;      // 灰度图像
    cv::Mat            blur_frame;
    cv::Mat            binary_frame;    // 二值图像
#endif

    int                dev_id  = -1;    // 设备 ID
    uint16_t           width   = 0;     // 图像宽度
    uint16_t           height  = 0;     // 图像高度
    uint16_t           fps     = 0;     // 帧率
    bool               is_open = false; // 摄像头是否已打开
    mutable std::mutex mtx;             // 互斥锁
};

#endif
