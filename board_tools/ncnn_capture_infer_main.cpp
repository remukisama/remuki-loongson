#include "car_config.hpp"
#include "lq_camera_ex.hpp"
#include "lq_ncnn.hpp"
#include "red_marker_detect.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cctype>
#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>

namespace
{

std::string trim(const std::string &s)
{
    size_t first = 0;
    while (first < s.size() && std::isspace(static_cast<unsigned char>(s[first]))) first++;
    size_t last = s.size();
    while (last > first && std::isspace(static_cast<unsigned char>(s[last - 1]))) last--;
    return s.substr(first, last - first);
}

bool is_absolute_path(const std::string &path)
{
    return !path.empty() && path[0] == '/';
}

std::string resolve_image_path(const std::string &name)
{
    if (is_absolute_path(name)) return name;
    return "./" + name;
}

bool load_model(LQ_NCNN &ncnn)
{
    ncnn.SetModelPath(car_config::kNcnnModelParam, car_config::kNcnnModelBin);
    ncnn.SetInputSize(car_config::kNcnnInputWidth, car_config::kNcnnInputHeight);
    ncnn.SetLabels({
        car_config::kNcnnLabel0,
        car_config::kNcnnLabel1,
        car_config::kNcnnLabel2,
    });

    float mean_vals[3] = {
        0.485f * 255.0f,
        0.456f * 255.0f,
        0.406f * 255.0f,
    };
    float norm_vals[3] = {
        1.0f / (0.229f * 255.0f),
        1.0f / (0.224f * 255.0f),
        1.0f / (0.225f * 255.0f),
    };
    ncnn.SetNormalize(mean_vals, norm_vals);

    if (!ncnn.Init()) {
        std::cerr << "[ERROR] 模型加载失败，请确认当前目录存在 "
                  << car_config::kNcnnModelParam << " 和 "
                  << car_config::kNcnnModelBin << std::endl;
        return false;
    }
    return true;
}

bool capture_photo(const std::string &filename)
{
    lq_camera_ex camera(320, 240, 180, LQ_CAMERA_0CPU_MJPG);
    if (camera.start_collect() != 0) {
        std::cerr << "[ERROR] 摄像头启动失败" << std::endl;
        return false;
    }
    if (!camera.set_exposure_manual(100)) {
        std::cerr << "[WARN] 手动曝光设置失败，继续使用当前曝光" << std::endl;
    }
    usleep(500 * 1000);

    cv::Mat frame;
    for (int i = 0; i < 4; ++i) frame = camera.get_frame_raw();
    camera.stop_collect();

    if (frame.empty()) {
        std::cerr << "[ERROR] 获取图像帧失败" << std::endl;
        return false;
    }

    cv::flip(frame, frame, -1);
    if (!cv::imwrite(filename, frame)) {
        std::cerr << "[ERROR] 图片保存失败: " << filename << std::endl;
        return false;
    }

    std::cout << "[CAPTURE] 已保存: " << filename
              << " (" << frame.cols << "x" << frame.rows << ")" << std::endl;
    return true;
}

bool infer_image(LQ_NCNN &ncnn, const std::string &image_path)
{
    auto total0 = std::chrono::high_resolution_clock::now();

    cv::Mat bgr = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        std::cerr << "[ERROR] 无法读取图片: " << image_path << std::endl;
        return false;
    }

    auto pre0 = std::chrono::high_resolution_clock::now();
    RedMarker marker = find_red_marker(bgr, 20.0f);
    if (!marker.valid) {
        std::cout << "[RESULT] 类型: <未检测到红色标记>" << std::endl;
        std::cout << "[DETAIL] 图片: " << image_path
                  << " | 尺寸: " << bgr.cols << "x" << bgr.rows
                  << " | best_red_area=" << marker.contour_area << std::endl;
        return false;
    }

    cv::Mat roi = preprocess_for_ncnn(
        bgr,
        car_config::kNcnnCropSide,
        car_config::kNcnnObjectBottomGap);
    auto pre1 = std::chrono::high_resolution_clock::now();
    if (roi.empty()) {
        std::cout << "[RESULT] 类型: <ROI 提取失败>" << std::endl;
        std::cout << "[DETAIL] 图片: " << image_path
                  << " | red_box=(" << marker.box.x << "," << marker.box.y
                  << "," << marker.box.width << "," << marker.box.height << ")"
                  << " | area=" << marker.contour_area << std::endl;
        return false;
    }

    auto infer0 = std::chrono::high_resolution_clock::now();
    std::string label = ncnn.Infer(roi);
    auto infer1 = std::chrono::high_resolution_clock::now();
    auto total1 = std::chrono::high_resolution_clock::now();

    const auto preprocess_ms = std::chrono::duration_cast<std::chrono::milliseconds>(pre1 - pre0).count();
    const auto infer_us = std::chrono::duration_cast<std::chrono::microseconds>(infer1 - infer0).count();
    const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total1 - total0).count();

    std::cout << "[RESULT] 类型: " << label << std::endl;
    std::cout << "[TIME] 推理时间: " << (infer_us / 1000.0) << " ms" << std::endl;
    std::cout << "[TIME] 预处理时间: " << preprocess_ms << " ms" << std::endl;
    std::cout << "[TIME] 总耗时: " << total_ms << " ms" << std::endl;
    std::cout << "[DETAIL] 图片: " << image_path
              << " | 尺寸: " << bgr.cols << "x" << bgr.rows
              << " | ROI: " << roi.cols << "x" << roi.rows
              << " | red_box=(" << marker.box.x << "," << marker.box.y
              << "," << marker.box.width << "," << marker.box.height << ")"
              << " | area=" << marker.contour_area << std::endl;
    return true;
}

} // namespace

int main()
{
    std::cout << "====== NCNN 拍照/图片推理工具 ======" << std::endl;
    std::cout << "模型: " << car_config::kNcnnModelParam << " / "
              << car_config::kNcnnModelBin << std::endl;
    std::cout << "标签: " << car_config::kNcnnLabel0 << " / "
              << car_config::kNcnnLabel1 << " / "
              << car_config::kNcnnLabel2 << std::endl;

    LQ_NCNN ncnn;
    if (!load_model(ncnn)) return 1;

    int capture_index = 0;
    while (true) {
        std::cout << std::endl;
        std::cout << "操作: 直接回车=拍照并推理；输入图片名=推理同目录图片；q=退出" << std::endl;
        std::cout << "ncnn_test> " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) break;
        line = trim(line);

        if (line == "q" || line == "Q") break;

        if (line.empty()) {
            std::string filename = "capture_" + std::to_string(capture_index++) + "_320x240.jpg";
            if (capture_photo(filename)) infer_image(ncnn, filename);
            continue;
        }

        infer_image(ncnn, resolve_image_path(line));
    }

    std::cout << "退出 ncnn_test。" << std::endl;
    return 0;
}
