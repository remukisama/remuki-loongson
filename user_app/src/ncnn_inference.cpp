#include "ncnn_inference.hpp"
#include "red_marker_detect.hpp"
#include "car_config.hpp"

#include "lq_ncnn.hpp"
#include <opencv2/imgproc.hpp>
#include <exception>
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
    NcnnResult result;
    const bool accepted = infer_with_result(bgr_320x240, result);
    label = result.label;
    return accepted;
}

bool NcnnInference::infer_with_result(const cv::Mat &bgr_320x240, NcnnResult &result)
{
    return infer_with_result(bgr_320x240, result, car_config::kNcnnObjectBottomGap);
}

bool NcnnInference::infer_with_result(const cv::Mat &bgr_320x240, NcnnResult &result, int object_bottom_gap)
{
    result = NcnnResult();
    result.object_bottom_gap = object_bottom_gap;
    if (!m_initialized) return false;
    if (bgr_320x240.empty()) return false;

    const RedMarker marker = find_red_marker(bgr_320x240, car_config::kRedMinArea);
    if (!marker.valid) return false;

    result.red_marker_valid = true;
    result.red_box = marker.box;
    result.red_box_points = marker.box_points;
    result.red_area = marker.contour_area;

    const RotatedRoi roi = extract_rotated_roi(
        bgr_320x240,
        marker,
        m_crop_side,
        1.0f,
        object_bottom_gap);
    if (roi.crop.empty()) return false;

    result.roi_valid = true;
    result.roi_points = roi.points;

    // 2. 缩放到模型输入尺寸（roi 已为 48×48 则无操作）
    cv::Mat input;
    if (roi.crop.cols != m_input_w || roi.crop.rows != m_input_h)
        cv::resize(roi.crop, input, cv::Size(m_input_w, m_input_h));
    else
        input = roi.crop;
    result.input_roi = input.clone();

    // 3. NCNN 推理（BGR→RGB + resize + normalize 在 LQ_NCNN::Infer 内完成）
    try {
        const LQNcnnResult scored = m_impl->ncnn.InferWithScore(input);
        m_last_logits = scored.logits;

        result.label = scored.label;
        result.class_id = scored.class_id;
        result.confidence = scored.confidence;
        result.logits = scored.logits;
        result.probabilities = scored.probabilities;
        result.valid = scored.valid && result.confidence >= car_config::kNcnnConfidenceThreshold;
    } catch (const std::exception &ex) {
        std::cerr << "[NCNN] 推理异常: " << ex.what() << std::endl;
        return false;
    }

    return result.valid;
}
