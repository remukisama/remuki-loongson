#ifndef LQ_RED_MARKER_DETECT_HPP_
#define LQ_RED_MARKER_DETECT_HPP_

#include <opencv2/core.hpp>
#include <vector>

/// @brief 红色标记检测结果
struct RedMarker
{
    cv::Rect box;                           // 正外接矩形 (x, y, w, h)
    cv::RotatedRect rotated_rect;            // 最小外接旋转矩形
    std::vector<cv::Point2f> box_points;    // 旋转矩形的 4 个顶点
    float contour_area = 0.0f;              // 轮廓像素面积
    bool valid = false;                     // 是否有效

    RedMarker() : valid(false) {}
    explicit operator bool() const { return valid; }
};

/// @brief 透视校正裁剪结果
struct RotatedRoi
{
    cv::Mat crop;                       // 透视校正后的正方形裁剪图像
    std::vector<cv::Point2f> points;    // 源图中 ROI 的 4 个顶点（顺时针）
    int side = 0;                       // 裁剪正方形边长（像素）
    float angle = 0.0f;                 // 旋转角度（度）
    float red_long = 0.0f;              // 红色标记长边长度（像素）
    float red_short = 0.0f;             // 红色标记短边长度（像素）

    RotatedRoi() {}
    explicit operator bool() const { return !crop.empty(); }
};

/// @brief 生成红色 HSV 掩膜
/// @param bgr 输入 BGR 图像
/// @param close_iterations 形态学闭合迭代次数
/// @return 二值掩膜（红色区域为 255）
cv::Mat make_red_mask(const cv::Mat &bgr, int close_iterations = 2);

/// @brief 在一张图中检测红色标记（12x5cm）
/// @param bgr 输入 BGR 图像
/// @param min_red_area 最小像素面积阈值
/// @return 检测到的红色标记；valid=false 表示未找到
RedMarker find_red_marker(const cv::Mat &bgr, float min_red_area = 20.0f);

/// @brief 从红色标记上方通过透视变换提取正方形 ROI（12x12cm 目标区域）
/// @param bgr 输入 BGR 图像
/// @param marker 已检测到的红色标记
/// @param crop_side 输出正方形边长（像素）
/// @param object_size_ratio 目标尺寸相对于红色标记宽度的比例
/// @param object_bottom_gap 目标底边与红色标记顶边的间隙（像素）
/// @return 透视校正后的裁剪结果；crop 为空表示失败
RotatedRoi extract_rotated_roi(
    const cv::Mat &bgr,
    const RedMarker &marker,
    int crop_side,
    float object_size_ratio = 1.0f,
    int object_bottom_gap = 0);

/// @brief 高层接口：从 320x240 图像中检测红色标记并提取 NCNN 推理输入
/// @param bgr_320x240 输入 320x240 BGR 图像
/// @param crop_side 输出正方形边长（像素）
/// @param bottom_gap ROI 底边与红块顶边的间隔（像素，与训练 --object-bottom-gap 一致）
/// @return 推理就绪的裁剪图像；空 Mat 表示未检测到红色标记
cv::Mat preprocess_for_ncnn(const cv::Mat &bgr_320x240, int crop_side, int bottom_gap = 5);

#endif // LQ_RED_MARKER_DETECT_HPP_
