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
void lq_http_img_trans_demo(void);  // HTTP 图像传输测试
void lq_uart_demo(void);            // UART 通信测试
void lq_uart_thread_demo(void);     // UART 通信测试（独立线程接收模式）
void lq_bh1750_demo(void);          // BH1750 光照传感器测试
void lq_bd1202_gps_demo(void);      // BD1202 GPS模块测试
void lq_dht11_demo(void);           // DHT11 温湿度传感器测试
void lq_tracking_demo(void);        // 8 路灰度循迹传感器测试

#endif
