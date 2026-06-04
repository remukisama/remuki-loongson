#include "main.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

// ==============================================================================
// @file    31_lq_pos_pid_tuning_demo.cpp
// @brief   位置环(视觉巡线)与角速度环融合调参专用Demo
// ==============================================================================

extern std::atomic<bool> exit_flag;
extern ImageStatustypedef ImageStatus;
extern bool g_enable_element_process;

extern float Speed_Goal;
extern float Speed_Goal_l;
extern float Speed_Goal_r;
extern float speed_encoder_L;
extern float speed_encoder_R;
extern float dbg_steer_error;
extern float dbg_target_gyro_z;
extern float dbg_diff_speed;
extern float dbg_raw_steer_error;
extern float dbg_diff_steer;
extern float dbg_visual_pd_out;
extern float dbg_cmd_target_gyro_z;
extern float dbg_real_gyro_z;
extern float dbg_filtered_gyro_z;
extern float dbg_gyro_error;
extern float dbg_target_delta;
extern float dbg_dt;
extern float dbg_max_diff;
extern float dbg_diff_before_limit;
extern float dbg_speed_err_l;
extern float dbg_speed_err_r;
extern int dbg_diff_limited;
extern int dbg_imu_ok;
extern int dbg_deadband_active;

extern void timer_thread_1();
extern void emergency_stop();

// 可调参数展示，需去 my_motor.cpp 中手动修改以下对应变量再重新编译
// STEER_KP, STEER_KD, DEAD_BAND, MAX_DIFF_RATIO
// gyro_pid.P, gyro_pid.D

void lq_pos_pid_tuning_demo(void)
{
    printf("\r\n");
    printf("=========================================\r\n");
    printf("   [DEMO 31] Pos PID & Cascade Tuning    \r\n");
    printf("=========================================\r\n");
    printf(">> 当前Demo默认关闭元素判断/处理，只保留基础巡线，方便调试 STEER_KP / STEER_KD。\r\n");
    printf(">> 目的: 定量观察【视觉偏差】->【目标角速度】->【真实角速度】->【给轮子的差速】的转换过程\r\n");
    printf(">> 你可以将小车架空或者以极低的速度在赛道上运行来观察以下终端数据。\r\n");
    printf(">> 看法: PD/Tgt 经常打满=视觉外环过猛或前瞻偏小；dT 经常顶住=目标角速度变化率正在限制。\r\n");
    printf(">> 看法: GErr 用角速度环滤波后的 GF 计算；GErr 长时间很大=角速度环/差速能力跟不上。\r\n");
    printf(">> 看法: Cap=1 频繁出现=差速上限限制；Real 与 GF 差很多=陀螺仪滤波/死区影响明显。\r\n");
    printf(">> 看法: SE_L/SE_R 长时间很大=速度环或轮子抓地/供电跟不上；IMU=0 表示陀螺仪读取失败。\r\n");
    printf(">> 退出: 按 Ctrl+C 停止\r\n\r\n");

    g_enable_element_process = false;

    // 默认小车主线程
    std::thread threadA(timer_thread_1);

    auto last_print_time = std::chrono::steady_clock::now();

    while (!exit_flag)
    {
        if (image_cv_zip() == -1) {
            printf("[DEMO] Fetch image frame FAILED!\n");
            emergency_stop();
            exit_flag = true;
            break;
        }

        imageprocess(); // 视觉处理更新 ImageStatus 变量

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count();
        
        if (elapsed >= 100) { // 10Hz 更新打印
            last_print_time = now;
            
            // 使用 VOFA+ 的浮点数协议格式打印通道，方便接入 VOFA+ 查看实时玄学波形
            // 通道 1: 滤波后的视觉偏差
            // 通道 2: 限速前PD输出
            // 通道 3: 目标角速度
            // 通道 4: 真实角速度
            // 通道 5: 最终差速
            
            float vofa_data[5];
            vofa_data[0] = dbg_steer_error;
            vofa_data[1] = dbg_visual_pd_out;
            vofa_data[2] = dbg_target_gyro_z;
            vofa_data[3] = dbg_real_gyro_z;
            vofa_data[4] = dbg_diff_speed;
            
            // VOFA+ JustFloat 协议尾部
            unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f}; 
            
            // 以十进制直接打印方便在终端看，如果要上 VOFA+ 需打印成字节流，这里先用终端友好格式：
            printf("Raw:%5.1f Filt:%5.1f dE:%5.1f | PD:%6.1f Tgt:%6.1f Cmd:%6.1f Real:%6.1f GF:%6.1f GErr:%6.1f dT:%5.1f | D0:%6.1f D:%6.1f Cap:%d Max:%4.1f | L:%5.1f R:%5.1f Enc:%4.0f/%4.0f SE:%5.1f/%5.1f | dt:%5.3f IMU:%d DB:%d\n",
                   dbg_raw_steer_error,
                   dbg_steer_error,
                   dbg_diff_steer,
                   dbg_visual_pd_out,
                   dbg_target_gyro_z,
                   dbg_cmd_target_gyro_z,
                   dbg_real_gyro_z,
                   dbg_filtered_gyro_z,
                   dbg_gyro_error,
                   dbg_target_delta,
                   dbg_diff_before_limit,
                   dbg_diff_speed,
                   dbg_diff_limited,
                   dbg_max_diff,
                   Speed_Goal_l,
                   Speed_Goal_r,
                   speed_encoder_L,
                   speed_encoder_R,
                   dbg_speed_err_l,
                   dbg_speed_err_r,
                   dbg_dt,
                   dbg_imu_ok,
                   dbg_deadband_active);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    threadA.join();
    g_enable_element_process = true;
    printf(">> Tuning Demo exited.\n");
}
