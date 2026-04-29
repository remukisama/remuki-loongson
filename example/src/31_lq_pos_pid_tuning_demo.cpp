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

extern int Speed_Goal;
extern float Speed_Goal_l;
extern float Speed_Goal_r;
extern float dbg_steer_error;
extern float dbg_target_gyro_z;
extern float dbg_diff_speed;

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
    printf(">> 目的: 定量观察【视觉偏差】->【目标角速度】->【给轮子的差速】的转换过程\r\n");
    printf(">> 你可以将小车架空或者以极低的速度在赛道上运行来观察以下终端数据。\r\n");
    printf(">> 关注: 若 dbg_target_gyro_z 数值跳变极大, 说明 STEER_KP 或 STEER_KD 太大。\r\n");
    printf(">> 关注: 若 dbg_diff_speed 跳变极大, 说明 gyro_pid 的 P 太大。\r\n");
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
            // 通道 1: 视觉原始偏差 (像素)
            // 通道 2: 目标角速度 (传给角速度环的)
            // 通道 3: 算出给轮子的真实差速增量
            // 通道 4: 左轮最终目标速度
            // 通道 5: 右轮最终目标速度
            
            float vofa_data[5];
            vofa_data[0] = dbg_steer_error;
            vofa_data[1] = dbg_target_gyro_z;
            vofa_data[2] = dbg_diff_speed;
            vofa_data[3] = (float)Speed_Goal_l;
            vofa_data[4] = (float)Speed_Goal_r;
            
            // VOFA+ JustFloat 协议尾部
            unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f}; 
            
            // 以十进制直接打印方便在终端看，如果要上 VOFA+ 需打印成字节流，这里先用终端友好格式：
            printf("Visual_Err:%6.1f | Tgt_YawRate:%6.1f | Out_Diff:%6.1f | L_Tgt:%6.1f | R_Tgt:%6.1f\n",
                   dbg_steer_error, dbg_target_gyro_z, dbg_diff_speed, Speed_Goal_l, Speed_Goal_r);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    threadA.join();
    g_enable_element_process = true;
    printf(">> Tuning Demo exited.\n");
}