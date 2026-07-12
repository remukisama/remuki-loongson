#pragma once

#include <opencv2/core.hpp>

int lq_vision_capture_demo(void);
void lq_vision_capture_maybe_save_race_frame(int frame_index);

// NCNN 观测图传：异步执行红色标记检测+NCNN分类，只叠加图像与日志，不参与控制动作。
bool lq_ncnn_observe_start(const char *host, int port);
void lq_ncnn_observe_submit(const cv::Mat &frame, int frame_index, const char *tag);
void lq_ncnn_observe_stop(void);

// NCNN/视觉观测模式：不驱动电机，普通元素处理关闭，UDP图传/TFT屏/TFT+UDP三种显示方式
// display_mode: 0=udp, 1=tft, 2=both
int lq_element_observe_demo(int display_mode);
