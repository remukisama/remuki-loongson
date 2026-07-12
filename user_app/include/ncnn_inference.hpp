#ifndef LQ_NCNN_INFERENCE_HPP_
#define LQ_NCNN_INFERENCE_HPP_

#include <string>
#include <vector>
#include <opencv2/core.hpp>

struct NcnnResult {
    std::string label;
    int class_id = -1;
    float confidence = 0.0f;
    bool valid = false;
    bool red_marker_valid = false;
    bool roi_valid = false;
    int object_bottom_gap = 0;
    float red_area = 0.0f;
    cv::Rect red_box;
    std::vector<cv::Point2f> red_box_points;
    std::vector<cv::Point2f> roi_points;
    cv::Mat input_roi;
    std::vector<float> logits;
    std::vector<float> probabilities;
};

/// @brief NCNN 推理封装：红色标记检测 → ROI 提取 → 模型推理
class NcnnInference
{
public:
    NcnnInference();
    ~NcnnInference();

    /// @brief 初始化 NCNN 模型（模型路径、输入尺寸、标签等从 car_config 读取）
    /// @return 成功返回 true
    bool init();

    /// @brief 对单帧 320x240 BGR 图像执行完整推理管线
    ///        检测红色标记 → 透视校正裁剪 → NCNN 分类
    /// @param bgr_320x240 输入 320x240 BGR 图像
    /// @param[out] label 输出推理标签
    /// @return 推理成功返回 true；未检测到红色标记或置信度不足返回 false
    bool infer(const cv::Mat &bgr_320x240, std::string &label);

    /// @brief 对单帧执行完整推理管线，并返回红标、ROI、logits、置信度等观测信息
    /// @return 置信度达到 car_config::kNcnnConfidenceThreshold 时返回 true
    bool infer_with_result(const cv::Mat &bgr_320x240, NcnnResult &result);

    /// @brief 使用指定 ROI bottom gap 执行推理，用于多裁剪位置观测调试
    bool infer_with_result(const cv::Mat &bgr_320x240, NcnnResult &result, int object_bottom_gap);

    /// @brief 推理是否已初始化
    bool is_initialized() const { return m_initialized; }

    /// @brief 获取最后一次推理的原始输出 logit 向量（用于调试）
    const std::vector<float> &last_logits() const { return m_last_logits; }

private:
    bool m_initialized;
    std::vector<float> m_last_logits;   // 最近一次推理的原始输出
    int m_crop_side;                    // ROI 裁剪边长
    int m_input_w, m_input_h;           // 模型输入尺寸

    // PIMPL 隐藏 LQ_NCNN 实现细节
    struct Impl;
    Impl *m_impl;
};

#endif // LQ_NCNN_INFERENCE_HPP_
