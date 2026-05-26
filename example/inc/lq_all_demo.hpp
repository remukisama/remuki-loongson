#ifndef __LQ_ALL_DEMO_H
#define __LQ_ALL_DEMO_H

#include "lq_drv_inc.hpp"
#include "lq_app_inc.hpp"
#include "lq_common.hpp"

void lq_mpu6050_demo(void);         // MPU6050 测试
void lq_motor_pid_demo(void);
void lq_direaction(void);
void lq_quadrature_encoder_demo(void);
void lq_quadrature_encoder_kernel_demo(void);
void lq_user_encoder_demo(void);
void lq_car_encoder_test_demo(void);
void lq_line_patrol_debug_demo(void); // 添加巡线算法调试图传 Demo
void lq_speed_pid_tuning_demo(void);  // 速度环PID调参Demo
void lq_vofa_speed_pid_demo(void);    // VOFA+ 速度环PID波形调参Demo
void lq_tft18_demo(void);           // TFT18 屏幕测试
void lq_camera_tft18_demo(void);    // TFT18 摄像头补线/元素状态显示测试
void lq_angle_pid_tuning_demo(void); // 角度环PID调参Demo
void lq_gyro_pid_tuning_demo(void);  // 角速度环PID调参Demo
void lq_mpu6050_scale_dt_diagnose_demo(void); // MPU6050量程、dt、零偏联合诊断Demo
void lq_car_steer_debug_demo(void); // 30. 查偏向不出弯原因调试连线
void lq_pos_pid_tuning_demo(void);  // 31. 联合调参位置环和角速度环
void lq_gyro_loop_verify_demo(void); // 29. 角速度环输出差速验证Demo
void lq_camera_capture_demo(void); // 摄像头拍照测试
void lq_parameter_step_tuning_demo(void); // 33. PARAMETER_TUNING_METHODS.md 一体化分步调参Demo
int lq_left_ring_exit_recommend_demo(void); // 34. 左环岛出环防兜圈专项Demo
int lq_param_test_00_precheck_config(void);
int lq_param_test_01_motor_encoder_direction(void);
int lq_param_test_02_camera_center_angle_static(void);
int lq_param_test_03_exposure_otsu_quality(void);
int lq_param_test_03_exposure_hsv_quality(void);
int lq_param_test_04_roi_scan_straight(void);
int lq_param_test_05_speed_loop_airborne(void);
int lq_param_test_06_low_speed_visual_gyro_closed_loop(void);
int lq_param_test_07_speed_up_regression(void);
int lq_param_test_08_element_debug(void);
int lq_param_test_08h_element_push_image_debug(void);
int lq_param_test_09_voltage_light_regression(void);
int lq_param_test_10_gyro_static_noise(void);
int lq_param_flow_visual(void);
int lq_param_flow_airborne(void);
int lq_param_flow_sensor(void);
int lq_param_flow_low_speed(void);
int lq_param_flow_element(void);

#endif
