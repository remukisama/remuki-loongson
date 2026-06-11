#include "ncnn_inference.hpp"
#include "red_marker_detect.hpp"
#include "car_config.hpp"

#include "lq_ncnn.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

// PIMPL 实现：包装 LQ_NCNN
struct NcnnInference::Impl
{
    LQ_NCNN ncnn;
};

NcnnInference::NcnnInference()
    : m_initialized(false)
    , m_crop_side(car_config::kNcnnCropSide)
    , m_input_w(car_config::kNcnnInputWidth)
    , m_input_h(car_config::kNcnnInputHeight)
    , m_impl(new Impl())
{
}

NcnnInference::~NcnnInference()
{
    delete m_impl;
}

bool NcnnInference::init()
{
    // 模型路径（训练产物：tiny_classifier_fp32.ncnn.{param,bin}）
    m_impl->ncnn.SetModelPath(car_config::kNcnnModelParam, car_config::kNcnnModelBin);

    // 输入尺寸（与 run_all.ps1 --img-width/--img-height 一致）
    m_impl->ncnn.SetInputSize(m_input_w, m_input_h);

    // 标签（与 ImageFolder 子目录名一致，按字母排序）
    m_impl->ncnn.SetLabels({
        car_config::kNcnnLabel0,
        car_config::kNcnnLabel1,
        car_config::kNcnnLabel2
    });

    // ImageNet 归一化（与 train_tiny_classifier.py 一致）
    // 训练: mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225]
    // NCNN: mean_vals = mean×255, norm_vals = 1/(std×255)
    float mean_vals[3]  = { 0.485f * 255.0f, 0.456f * 255.0f, 0.406f * 255.0f };
    float norm_vals[3]  = { 1.0f / (0.229f * 255.0f), 1.0f / (0.224f * 255.0f), 1.0f / (0.225f * 255.0f) };
    m_impl->ncnn.SetNormalize(mean_vals, norm_vals);

    if (!m_impl->ncnn.Init()) {
        std::cerr << "[NCNN] 模型加载失败！请确认 " << car_config::kNcnnModelParam
                  << " / " << car_config::kNcnnModelBin << " 在当前目录。" << std::endl;
        return false;
    }

    m_initialized = true;
    std::cout << "[NCNN] 模型加载成功 | 输入: " << m_input_w << "x" << m_input_h
              << " | ROI: " << m_crop_side << "x" << m_crop_side
              << " | 标签: " << car_config::kNcnnLabel0 << "/"
              << car_config::kNcnnLabel1 << "/" << car_config::kNcnnLabel2 << std::endl;
    return true;
}

bool NcnnInference::infer(const cv::Mat &bgr_320x240, std::string &label)
{
    if (!m_initialized) return false;
    if (bgr_320x240.empty()) return false;

    // 1. 红色标记检测 + 透视校正裁剪（48×48，gap=5 与训练 --object-bottom-gap 5 一致）
    cv::Mat roi = preprocess_for_ncnn(bgr_320x240, m_crop_side, car_config::kNcnnObjectBottomGap);
    if (roi.empty()) return false;

    // 2. 缩放到模型输入尺寸（roi 已为 48×48 则无操作）
    cv::Mat input;
    if (roi.cols != m_input_w || roi.rows != m_input_h)
        cv::resize(roi, input, cv::Size(m_input_w, m_input_h));
    else
        input = roi;

    // 3. NCNN 推理（BGR→RGB + resize + normalize 在 LQ_NCNN::Infer 内完成）
    std::string result = m_impl->ncnn.Infer(input);
    if (result.empty()) return false;

    label = result;
    return true;
}
