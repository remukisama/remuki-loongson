#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "lq_camera_ex.hpp"
#include "lq_ncnn.hpp"
#include "red_marker_detect.hpp"

// ===================================================================
//  调试可视化：在原图上叠加红色标记框 + ROI裁剪框
//  （与 crop_red_roi.py 中 draw_rotated_debug_image 一致）
// ===================================================================
static cv::Mat draw_debug_overlay(const cv::Mat &bgr, const RedMarker &marker, const RotatedRoi &roi)
{
    cv::Mat debug = bgr.clone();
    std::vector<cv::Point> red_poly;
    for (auto &p : marker.box_points)
        red_poly.push_back(cv::Point((int)p.x, (int)p.y));
    cv::polylines(debug, red_poly, true, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    std::vector<cv::Point> roi_poly;
    for (auto &p : roi.points)
        roi_poly.push_back(cv::Point((int)p.x, (int)p.y));
    cv::polylines(debug, roi_poly, true, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    char text[64];
    snprintf(text, sizeof(text), "red=%.0f side=%d", marker.contour_area, roi.side);
    cv::putText(debug, text, cv::Point(2, 12), cv::FONT_HERSHEY_SIMPLEX,
                0.35, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    return debug;
}

// ===================================================================
//  NCNN 模型统一加载
// ===================================================================
static bool load_ncnn_model(LQ_NCNN &ncnn)
{
    ncnn.SetModelPath("tiny_classifier_fp32.ncnn.param", "tiny_classifier_fp32.ncnn.bin");
    ncnn.SetInputSize(48, 48);
    ncnn.SetLabels({"supplies", "vehicles", "weapons"});
    // ImageNet 归一化（与 train_tiny_classifier.py 一致）
    // PyTorch: (x/255 - mean) / std   →   NCNN: (x - mean*255) * (1/(std*255))
    float mean_vals[3] = {0.485f * 255.0f, 0.456f * 255.0f, 0.406f * 255.0f};
    float norm_vals[3] = {1.0f / (0.229f * 255.0f), 1.0f / (0.224f * 255.0f), 1.0f / (0.225f * 255.0f)};
    ncnn.SetNormalize(mean_vals, norm_vals);

    if (!ncnn.Init()) {
        std::cerr << "模型加载失败！请确认 tiny_classifier_fp32.ncnn.param/.bin 在当前目录。" << std::endl;
        return false;
    }
    std::cout << "模型加载成功 | 48x48 | supplies/vehicles/weapons" << std::endl;
    return true;
}

// ===================================================================
//  Demo ①：摄像头实时推理（检测红色标记 → 透视校正 → NCNN）
// ===================================================================
void lq_ncnn_infer_demo(void)
{
    std::cout << "====== NCNN 推理验证 Demo (走马观碑) ======" << std::endl;

    // ---- 摄像头初始化（与实车完全一致）----
    lq_camera_ex camera(320, 240, 180, LQ_CAMERA_0CPU_MJPG);
    if (camera.start_collect() != 0) {
        std::cerr << "摄像头启动失败！" << std::endl;
        return;
    }
    if (!camera.set_exposure_manual(55))
        std::cerr << "警告: 手动曝光设置失败" << std::endl;
    usleep(500 * 1000);
    std::cout << "摄像头: " << camera.get_camera_width() << "x" << camera.get_camera_height()
              << " @ " << camera.get_camera_fps() << "fps | 曝光: 55" << std::endl;

    // ---- NCNN 模型 ----
    LQ_NCNN ncnn;
    if (!load_ncnn_model(ncnn)) { camera.stop_collect(); return; }

    // ---- 键盘线程 ----
    std::atomic<bool> exit_flag(false);
    std::thread input_thread([&]() {
        while (!exit_flag.load()) {
            std::string line;
            std::getline(std::cin, line);
            if (line == "q" || line == "Q") exit_flag.store(true);
        }
    });

    std::cout << "\n------------------------------------------" << std::endl;
    std::cout << "  检测到红色标记后自动推理并打印结果。" << std::endl;
    std::cout << "  输入 'q' 并回车退出。" << std::endl;
    std::cout << "------------------------------------------\n" << std::endl;

    int frame_cnt  = 0, detect_cnt = 0, infer_cnt = 0;
    std::string last_label;

    while (!exit_flag.load()) {
        cv::Mat frame = camera.get_frame_raw();
        if (frame.empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        cv::flip(frame, frame, -1);
        frame_cnt++;

        // 红色标记检测（含底部横条回退）
        RedMarker marker = find_red_marker(frame, 20.0f);
        if (!marker) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        detect_cnt++;

        // 透视校正裁剪 48×48
        RotatedRoi roi = extract_rotated_roi(frame, marker, 48, 1.0f, 5); // gap=5 与训练一致
        if (!roi) continue;

        // NCNN 推理
        try {
            std::string label = ncnn.Infer(roi.crop);
            infer_cnt++;
            if (label != last_label) {
                std::cout << "[NCNN] frame=" << frame_cnt
                          << " det=" << detect_cnt
                          << " infer=" << infer_cnt
                          << " → " << label << std::endl;
                last_label = label;
            }
        } catch (const std::exception &e) {
            std::cerr << "[NCNN] 推理异常: " << e.what() << std::endl;
        }
    }

    camera.stop_collect();
    input_thread.join();
    std::cout << "\n退出。总帧数: " << frame_cnt
              << " | 检测到红色: " << detect_cnt
              << " | 推理成功: " << infer_cnt << std::endl;
}

// ===================================================================
//  Demo ②：单图推理测试（读取 test.jpg → 检测 → 裁剪 → 推理 → 保存）
// ===================================================================
void lq_ncnn_image_infer_demo(void)
{
    std::cout << "====== NCNN 单图推理测试 ======" << std::endl;

    // ---- NCNN 模型 ----
    LQ_NCNN ncnn;
    if (!load_ncnn_model(ncnn)) return;

    // ---- 读取图片 ----
    std::string test_file = "test.jpg";
    {
        std::ifstream f(test_file);
        if (!f.good()) {
            std::cerr << "未找到 test.jpg，请将测试图片放到当前目录。" << std::endl;
            return;
        }
    }
    std::cout << "读取: " << test_file << std::endl;
    cv::Mat bgr = cv::imread(test_file);
    if (bgr.empty()) { std::cerr << "无法读取图片。" << std::endl; return; }
    std::cout << "尺寸: " << bgr.cols << "x" << bgr.rows << std::endl;

    // ---- 红色标记检测（含底部横条回退）----
    RedMarker marker = find_red_marker(bgr, 20.0f);
    if (!marker) {
        std::cout << "→ 未检测到红色标记，跳过推理。" << std::endl;
        return;
    }
    std::cout << "红色标记: 面积=" << marker.contour_area
              << " | box=(" << marker.box.x << "," << marker.box.y
              << "," << marker.box.width << "," << marker.box.height << ")" << std::endl;

    // ---- 透视校正裁剪 48×48 ----
    RotatedRoi roi = extract_rotated_roi(bgr, marker, 48, 1.0f, 5); // gap=5 与训练一致
    if (!roi) { std::cerr << "ROI 提取失败" << std::endl; return; }

    // ---- NCNN 推理 ----
    try {
        auto t0 = std::chrono::high_resolution_clock::now();
        std::string label = ncnn.Infer(roi.crop);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "→ " << label
                  << " | 耗时: " << ms << " ms" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "推理异常: " << e.what() << std::endl;
    }
}
