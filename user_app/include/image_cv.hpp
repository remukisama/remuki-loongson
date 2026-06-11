#ifndef __IMAGE_CV_H__
#define __IMAGE_CV_H__

#include <opencv2/core.hpp>

extern char img[60][80];
extern char img3[60][80];
extern char img4[60][160];
extern char img5[120][160];

bool image_cv_Init(void);
bool image_cv_zip(void);

/// @brief 获取高分辨率原始帧（320x240 BGR，已做 180° 翻转）
///        供 NCNN 推理等下游模块使用。
/// @note  必须在 image_cv_zip() 之后调用，返回的是当前帧的引用。
const cv::Mat &image_cv_get_hires_frame();

#endif // !__