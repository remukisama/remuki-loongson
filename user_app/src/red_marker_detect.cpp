#include "red_marker_detect.hpp"
#include "car_config.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

using namespace car_config;

// ===================================================================
//  HSV 红色掩膜生成
// ===================================================================
cv::Mat make_red_mask(const cv::Mat &bgr, int close_iterations)
{
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 红色在 HSV 色相环上跨 0°/180° 边界，分两个区间
    cv::Mat mask1, mask2;
    cv::inRange(hsv,
        cv::Scalar(kRedMarkerHsvLowH1,  kRedMarkerHsvLowS1,  kRedMarkerHsvLowV1),
        cv::Scalar(kRedMarkerHsvHighH1, kRedMarkerHsvHighS1, kRedMarkerHsvHighV1),
        mask1);
    cv::inRange(hsv,
        cv::Scalar(kRedMarkerHsvLowH2,  kRedMarkerHsvLowS2,  kRedMarkerHsvLowV2),
        cv::Scalar(kRedMarkerHsvHighH2, kRedMarkerHsvHighS2, kRedMarkerHsvHighV2),
        mask2);

    cv::Mat mask = mask1 | mask2;

    // 形态学：开运算去噪，闭运算补孔
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1,-1), 1);
    if (close_iterations > 0)
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), close_iterations);

    return mask;
}

// ===================================================================
//  底部红色横条回退检测（放宽 HSV 阈值）
// ===================================================================
static RedMarker find_bottom_red_band_marker(const cv::Mat &bgr, float min_red_area)
{
    // 放宽阈值的掩膜
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask1, mask2;
    cv::inRange(hsv,
        cv::Scalar(kRedBandHsvLowH1,  kRedBandHsvLowS1,  kRedBandHsvLowV1),
        cv::Scalar(kRedBandHsvHighH1, kRedBandHsvHighS1, kRedBandHsvHighV1),
        mask1);
    cv::inRange(hsv,
        cv::Scalar(kRedBandHsvLowH2,  kRedBandHsvLowS2,  kRedBandHsvLowV2),
        cv::Scalar(kRedBandHsvHighH2, kRedBandHsvHighS2, kRedBandHsvHighV2),
        mask2);
    cv::Mat mask = mask1 | mask2;

    // 开运算，不做闭合（保留原始边界）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1,-1), 1);

    int h = mask.rows, w = mask.cols;
    int min_run   = std::max(12, (int)std::round(w * 0.06));
    int min_y     = (int)std::round(h * 0.55);

    // 逐行扫描连续红色水平段
    struct RowRun { int y, x0, x1; };
    std::vector<RowRun> row_runs;

    for (int y = min_y; y < h; ++y) {
        uchar *row = mask.ptr<uchar>(y);
        int run_start = -1;
        int best_start = -1, best_end = -1, best_len = 0;
        for (int x = 0; x < w; ++x) {
            if (row[x]) {
                if (run_start < 0) run_start = x;
            } else {
                if (run_start >= 0) {
                    int len = x - run_start;
                    if (len > best_len) { best_len = len; best_start = run_start; best_end = x - 1; }
                    run_start = -1;
                }
            }
        }
        if (run_start >= 0) {
            int len = w - run_start;
            if (len > best_len) { best_len = len; best_start = run_start; best_end = w - 1; }
        }
        if (best_len >= min_run)
            row_runs.push_back({y, best_start, best_end});
    }

    if (row_runs.empty()) return {};

    // 聚合相邻行
    std::vector<std::vector<RowRun>> groups;
    groups.push_back({row_runs[0]});
    for (size_t i = 1; i < row_runs.size(); ++i) {
        if (row_runs[i].y <= groups.back().back().y + 1)
            groups.back().push_back(row_runs[i]);
        else
            groups.push_back({row_runs[i]});
    }

    // 取最低的一组（红色标记在画面底部）
    auto &group = *std::max_element(groups.begin(), groups.end(),
        [](const auto &a, const auto &b) { return a.back().y < b.back().y; });

    int x0 = w, x1 = 0, y0 = h, y1 = 0;
    for (auto &run : group) {
        if (run.x0 < x0) x0 = run.x0;
        if (run.x1 > x1) x1 = run.x1;
        if (run.y  < y0) y0 = run.y;
        if (run.y  > y1) y1 = run.y;
    }

    int band_w = x1 - x0 + 1;
    int band_h = y1 - y0 + 1;
    float area = (float)(band_w * band_h);
    if (area < min_red_area) return {};

    float aspect_ratio = (float)band_w / std::max(1.0f, (float)band_h);
    if (aspect_ratio < kRedAspectLow || aspect_ratio > kRedAspectHigh) return {};

    // 构造人工轮廓并计算最小外接矩形
    std::vector<cv::Point> pts = { {x0,y0}, {x1,y0}, {x1,y1}, {x0,y1} };
    cv::RotatedRect rect = cv::minAreaRect(pts);
    std::vector<cv::Point2f> box_pts(4);
    rect.points(box_pts.data());

    RedMarker m;
    m.box         = cv::Rect(x0, y0, band_w, band_h);
    m.rotated_rect = rect;
    m.box_points   = box_pts;
    m.contour_area = area;
    m.valid        = true;
    return m;
}

// ===================================================================
//  红色标记主检测
// ===================================================================
RedMarker find_red_marker(const cv::Mat &bgr, float min_red_area)
{
    cv::Mat mask = make_red_mask(bgr, kRedMorphCloseIter);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    float best_score = -1.0f;
    float best_seen_area = 0.0f;
    std::vector<cv::Point> best_contour;
    float frame_area = (float)(bgr.rows * bgr.cols);

    for (auto &contour : contours) {
        float area = (float)cv::contourArea(contour);
        best_seen_area = std::max(best_seen_area, area);
        if (area < min_red_area) continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);
        cv::Size2f rsz = rect.size;
        float cy = rect.center.y;

        float long_len  = std::max(rsz.width, rsz.height);
        float short_len = std::max(1.0f, std::min(rsz.width, rsz.height));
        float rect_area = std::max(1.0f, long_len * short_len);
        float aspect_ratio = long_len / short_len;
        float area_ratio   = area / std::max(1.0f, frame_area);
        float fill_ratio   = area / rect_area;
        float center_y_ratio = cy / std::max(1.0f, (float)bgr.rows);

        if (aspect_ratio < kRedAspectLow || aspect_ratio > kRedAspectHigh) continue;
        if (area_ratio < kRedAreaRatioLow || area_ratio > kRedAreaRatioHigh) continue;
        if (fill_ratio < kRedFillRatioMin) continue;
        if (center_y_ratio < kRedCenterYMinRatio) continue;

        float score = area * fill_ratio;
        if (score > best_score) {
            best_score  = score;
            best_contour = contour;
        }
    }

    if (best_contour.empty()) {
        // 回退：底部红色横条
        RedMarker band = find_bottom_red_band_marker(bgr, min_red_area);
        if (band.valid) return band;
        RedMarker none;
        none.contour_area = best_seen_area;
        return none;
    }

    cv::RotatedRect rect = cv::minAreaRect(best_contour);
    std::vector<cv::Point2f> box_pts(4);
    rect.points(box_pts.data());

    RedMarker m;
    m.box          = cv::boundingRect(best_contour);
    m.rotated_rect = rect;
    m.box_points   = box_pts;
    m.contour_area = best_score > 0 ? best_score : (float)cv::contourArea(best_contour);
    m.valid        = true;
    return m;
}

// ===================================================================
//  透视变换 ROI 提取
// ===================================================================
RotatedRoi extract_rotated_roi(
    const cv::Mat &bgr,
    const RedMarker &marker,
    int crop_side,
    float object_size_ratio,
    int object_bottom_gap)
{
    RotatedRoi roi;
    cv::RotatedRect rect = marker.rotated_rect;
    std::vector<cv::Point2f> pts = marker.box_points;

    // 计算 4 条边的长度，找到长边和短边
    struct Edge { float len; cv::Point2f p0, p1; };
    std::vector<Edge> edges;
    for (int i = 0; i < 4; ++i) {
        cv::Point2f p0 = pts[i], p1 = pts[(i + 1) % 4];
        edges.push_back({(float)cv::norm(p1 - p0), p0, p1});
    }

    auto long_edge  = *std::max_element(edges.begin(), edges.end(),
        [](const Edge &a, const Edge &b) { return a.len < b.len; });
    auto short_edge = *std::min_element(edges.begin(), edges.end(),
        [](const Edge &a, const Edge &b) { return a.len < b.len; });

    // 长边方向向量 u，归一化
    cv::Point2f u = long_edge.p1 - long_edge.p0;
    float u_norm = (float)cv::norm(u);
    if (u_norm < 1.0f) return roi;
    u /= u_norm;

    // 法向量 n（指向长边上方），即指向目标区域方向
    cv::Point2f n(-u.y, u.x);
    if (n.y > 0) n = -n;  // 确保 n 指向上方

    float long_len  = long_edge.len;
    float short_len = short_edge.len;

    // 目标中心 = 红色标记中心 + 法向量方向偏移
    float half = crop_side * 0.5f;
    cv::Point2f red_center(rect.center.x, rect.center.y);
    cv::Point2f target_center = red_center
        + n * (short_len * 0.5f + (float)object_bottom_gap + half);

    // 4 个源点：以 target_center 为中心的正方形
    std::vector<cv::Point2f> src(4);
    src[0] = target_center - u * half + n * half;
    src[1] = target_center + u * half + n * half;
    src[2] = target_center + u * half - n * half;
    src[3] = target_center - u * half - n * half;

    // 目标正方形
    std::vector<cv::Point2f> dst(4);
    dst[0] = cv::Point2f(0, 0);
    dst[1] = cv::Point2f((float)(crop_side - 1), 0);
    dst[2] = cv::Point2f((float)(crop_side - 1), (float)(crop_side - 1));
    dst[3] = cv::Point2f(0, (float)(crop_side - 1));

    cv::Mat M = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(bgr, roi.crop, M, cv::Size(crop_side, crop_side),
        cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    roi.points    = src;
    roi.side      = crop_side;
    roi.angle     = rect.angle;
    roi.red_long  = long_len;
    roi.red_short = short_len;

    return roi;
}

// ===================================================================
//  高层接口：检测红色标记 → 透视校正 → 输出 NCNN 推理输入
// ===================================================================
cv::Mat preprocess_for_ncnn(const cv::Mat &bgr_320x240, int crop_side, int bottom_gap)
{
    if (bgr_320x240.empty()) return {};

    RedMarker marker = find_red_marker(bgr_320x240, kRedMinArea);
    if (!marker.valid) return {};

    RotatedRoi roi = extract_rotated_roi(bgr_320x240, marker, crop_side, 1.0f, bottom_gap);
    if (!roi) return {};

    return roi.crop;
}
