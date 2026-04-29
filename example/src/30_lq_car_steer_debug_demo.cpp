#include "main.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

// ==============================================================================
// @file    30_lq_car_steer_debug_demo.cpp
// @brief   差速打角与闭环连通性分析 Demo (取代原版 main)
// ==============================================================================

// 提取我们在 my_motor.cpp 中增加的关键系统变量
extern std::atomic<bool> exit_flag;
extern ImageStatustypedef ImageStatus;

extern int Speed_Goal;
extern float Speed_Goal_l;
extern float Speed_Goal_r;
extern float speed_out_l;
extern float speed_out_r;
extern float speed_encoder_L;
extern float speed_encoder_R;

extern float dbg_steer_error;
extern float dbg_target_gyro_z;
extern float dbg_diff_speed;

// 这些函数原本在 my_motor.cpp 与 imageprocess.cpp 等处提供给外部
extern void timer_thread_1();
extern void emergency_stop();

void lq_car_steer_debug_demo(void)
{
    printf("\r\n");
    printf("=========================================\r\n");
    printf("   [DEMO 30] Car Steer DBG Started       \r\n");
    printf("=========================================\r\n");
    printf(">> This demo manually runs the normal    \r\n");
    printf(">> Threads while printing debug states.  \r\n");
    printf(">> Press Ctrl+C directly to stop.        \r\n\r\n");

    // 0. 启动原有的独立任务线程 (因为我们在主函数调用这个demo前，并未启动 threadA)
    std::thread threadA(timer_thread_1);

    auto last_print_time = std::chrono::steady_clock::now();

    while (!exit_flag)
    {
        // 1. 获取图像并二值化
        if (image_cv_zip() == -1) {
            printf("[DEMO] Fetch image frame FAILED!\n");
            emergency_stop();
            exit_flag = true;
            break;
        }

        // 2. 图像处理（巡线找中线，识别元素等）
        imageprocess();

        // 3. 定期打印控制链路的节点信息 (10Hz / 每100ms)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count();
        
        if (elapsed >= 100) {
            last_print_time = now;
            
            // 清屏可选项，由于有时想看完整Log，就不清屏了，直接连续打印
            printf("--- Control Chain Debug [%d ms] ---\n", (int)elapsed);
            printf("[1. Vision ] MidLine: %d \t| Det_True: %d\n", 
                ImageStatus.MiddleLine, ImageStatus.Det_True);
                
            printf("[2. Steer  ] raw_error (P): %.2f \t| Tgt_Gyro_Z: %.2f\n", 
                dbg_steer_error, dbg_target_gyro_z);
                
            printf("[3. Cascade] Act_Diff: %.2f\n", 
                dbg_diff_speed);
                
            printf("[4. Tgt_Spd] SpeedGoal: %d \t| L_Goal: %.1f \t| R_Goal: %.1f\n", 
                Speed_Goal, Speed_Goal_l, Speed_Goal_r);
                
            printf("[5. PID_Out] Pwm_L_out: %.1f \t| Pwm_R_out: %.1f\n", 
                speed_out_l, speed_out_r);
                
            printf("[6. Encoder] Enc_L: %.1f \t\t| Enc_R: %.1f\n", 
           speed_encoder_L, speed_encoder_R);
                
            printf("----------------------------------\n");
        }
        
        // 预防死循环占用100%单核，稍微切一下上下文
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 回收线程
    threadA.join();
    printf(">> Demo exited.\n");
}
