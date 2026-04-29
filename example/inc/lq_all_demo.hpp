#ifndef __LQ_ALL_DEMO_H
#define __LQ_ALL_DEMO_H

#include "lq_drv_inc.hpp"
#include "lq_app_inc.hpp"
#include "lq_common.hpp"

void lq_gpio_output_demo(void);     // GPIO 输出模式测试
void lq_gpio_input_demo(void);      // GPIO 输入模式测试
void lq_pwm_demo(void);             // PWM 输出模式测试
void lq_gtim_pwm_demo(void);        // GTIM PWM 输出模式测试
void lq_atim_pwm_demo(void);        // ATIM PWM 输出模式测试
void lq_encoder_pwm_demo(void);     // 编码器 PWM 输出模式测试
void lq_canfd_demo(void);           // CANFD 测试
void lq_ncnn_demo(void);            // NCNN 测试
void lq_ncnn_photo_demo(void);      // NCNN 图像分类测试
void lq_ips20_demo(void);           // IPS屏幕测试
void lq_mpu6050_demo(void);         // MPU6050 测试
void lq_lsm6dsr_demo(void);         // LSM6DSR 测试
void lq_vl53l0x_demo(void);         // VL53L0X 测试
void lq_udp_img_trans_demo(void);   // UDP 图像传输测试
void lq_udp_wavefrom_demo(void);    // UDP 波形传输测试
void lq_icm42688_demo(void);        // ICM42688 测试
void lq_ntp_demo(void);             // NTP 获取网络时间并更新到系统中测试
void lq_timer_demo(void);           // 定时器回调测试
void lq_module_load_demo(void);     // 模块加载函数测试
void lq_ips20_show_img_demo(void);  // IPS20 显示图像测试
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
void lq_angle_pid_tuning_demo(void); // 角度环PID调参Demo
void lq_gyro_pid_tuning_demo(void);  // 角速度环PID调参Demo
void lq_mpu6050_scale_dt_diagnose_demo(void); // MPU6050量程、dt、零偏联合诊断Demo
void lq_car_steer_debug_demo(void); // 30. 查偏向不出弯原因调试连线
void lq_pos_pid_tuning_demo(void);  // 31. 联合调参位置环和角速度环
void lq_gyro_loop_verify_demo(void); // 29. 角速度环输出差速验证Demo
void lq_camera_capture_demo(void); // 摄像头拍照测试
void lq_udp_img_trans_ex_demo(void); // 32. UDP 图像传输测试（增强版摄像头）

#endif
