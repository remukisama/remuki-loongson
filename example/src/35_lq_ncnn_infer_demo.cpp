#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <sys/stat.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "lq_ncnn.hpp"
#include "red_marker_detect.hpp"
#include "ncnn_inference.hpp"
#include "car_config.hpp"
#include "lq_camera_ex.hpp"

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
    std::cout << "====== NCNN 实时推理测试（同实车推理逻辑）======" << std::endl;

    // ---- 摄像头只负责提供 320x240 BGR 原始帧，不走循迹二值化/扫线处理 ----
    lq_camera_ex camera(
        car_config::kCameraHiresWidth,
        car_config::kCameraHiresHeight,
        car_config::kCameraFps,
        car_config::kCameraFormat,
        car_config::kCameraDevicePath);
    if (camera.start_collect() != 0) {
        std::cerr << "摄像头启动失败！" << std::endl;
        return;
    }
    if (!camera.set_exposure_manual(car_config::kCameraManualExposure))
        std::cerr << "警告: 手动曝光设置失败" << std::endl;
    usleep(500 * 1000);
    std::cout << "摄像头: " << camera.get_camera_width() << "x" << camera.get_camera_height()
              << " @ " << camera.get_camera_fps() << "fps | 曝光: "
              << car_config::kCameraManualExposure << std::endl;

    // ---- NCNN 推理封装：与实车主循环 NcnnInference::infer(...) 完全一致 ----
    NcnnInference ncnn;
    if (!ncnn.init()) {
        camera.stop_collect();
        return;
    }

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
    std::cout << "  只运行 NCNN 实时推理测试，不启动电机闭环。" << std::endl;
    std::cout << "  摄像头取 320x240 原始帧；推理逻辑使用实车同一个 NcnnInference::infer。" << std::endl;
    std::cout << "  未检测到可分类元素时不打印结果；检测到时打印分类与推理耗时。" << std::endl;
    std::cout << "  推理成功帧会保存到 /home/root/ncnn_debug/，黄色框=红色色块，绿色框=ROI。" << std::endl;
    std::cout << "  输入 'q' 并回车退出。" << std::endl;
    std::cout << "------------------------------------------\n" << std::endl;

    const char *debug_dir = "/home/root/ncnn_debug";
    mkdir(debug_dir, 0755);
    int frame_cnt  = 0, infer_cnt = 0;

    while (!exit_flag.load()) {
        cv::Mat frame = camera.get_frame_raw();
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        cv::flip(frame, frame, -1);
        frame_cnt++;

        auto start = std::chrono::high_resolution_clock::now();
        std::string label;
        if (ncnn.infer(frame, label)) {
            auto end = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(end - start).count();
            RedMarker marker = find_red_marker(frame, car_config::kRedMinArea);
            infer_cnt++;
            std::string raw_path;
            std::string overlay_path;
            std::cout << "[NCNN] 类型: " << label
                      << " | 推理耗时: " << ms << " ms"
                      << " | frame=" << frame_cnt
                      << " | infer=" << infer_cnt;
            if (marker.valid) {
                RotatedRoi roi = extract_rotated_roi(frame, marker,
                    car_config::kNcnnCropSide, 1.0f, car_config::kNcnnObjectBottomGap);
                char raw_name[160];
                char overlay_name[160];
                snprintf(raw_name, sizeof(raw_name), "%s/infer_%04d_frame_%06d_raw.jpg",
                    debug_dir, infer_cnt, frame_cnt);
                snprintf(overlay_name, sizeof(overlay_name), "%s/infer_%04d_frame_%06d_overlay.jpg",
                    debug_dir, infer_cnt, frame_cnt);
                raw_path = raw_name;
                overlay_path = overlay_name;
                bool raw_saved = cv::imwrite(raw_path, frame);
                bool overlay_saved = false;
                if (roi) {
                    cv::Mat debug = draw_debug_overlay(frame, marker, roi);
                    overlay_saved = cv::imwrite(overlay_path, debug);
                }
                std::cout << " | red_box=(" << marker.box.x << "," << marker.box.y
                          << "," << marker.box.width << "," << marker.box.height
                          << ") | red_area=" << marker.contour_area
                          << " | raw_saved=" << (raw_saved ? raw_path : "FAILED")
                          << " | overlay_saved=" << (overlay_saved ? overlay_path : "FAILED");
            }
            std::cout << std::endl;
        }
    }

    camera.stop_collect();
    input_thread.join();
    std::cout << "\n退出。总帧数: " << frame_cnt
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
