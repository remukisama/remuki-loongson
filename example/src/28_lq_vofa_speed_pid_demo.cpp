#include "lq_all_demo.hpp"
#include "lq_tcp_client.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <cstdio>

// ==========================================
// 【基于 VOFA+ 的速度环(PID) 波形调参演示程序】
// ==========================================

// 用于安全退出
static std::atomic<bool> exit_vofa_demo(false);

void signal_handler_vofa_tuning(int signum) {
    exit_vofa_demo = true;
}

// 可在下面修改 VOFA+ 服务器（PC端）的 IP 和 端口
#define SERVER_IP       "192.168.77.22"
#define SERVER_PORT     8080

void lq_vofa_speed_pid_demo(void)
{
    std::cout << "========== 基于 VOFA+ 的速度环波形调参演示程序启动 ==========" << std::endl;
    std::cout << ">>> 尝试连接 VOFA+ 服务器..." << std::endl;
    std::cout << ">>> 服务器 IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> 服务器 Port: " << SERVER_PORT << std::endl;
    std::cout << "按 Ctrl+C 安全退出并停止电机。" << std::endl;

    // 1. 初始化 TCP 客户端并连接 VOFA+ 服务器
    lq_tcp_client client;
    while (!exit_vofa_demo && !client.is_connected())
    {
        client.tcp_client_init(SERVER_IP, SERVER_PORT);
        if (!client.is_connected()) {
            std::cout << "VOFA+ 服务器连接失败, 1秒后重试..." << std::endl;
            usleep(1000000); // 延时 1 秒
        }
    }
    
    if (exit_vofa_demo) return;
    std::cout << ">>> VOFA+ 服务器连接成功！" << std::endl;

    // 2. 初始化电机 PWM 控制 (频率10KHz，初始占空比0)
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_INV);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_INV);
    
    // 3. 初始化电机方向控制引脚
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    // 4. 初始化编码器 (使用普通PWM脉冲+方向编码器)
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // --- PID 调参变量区 ---
    // 【请在这里修改您的 PID 参数】
    float Kp = 57.0;
    float Ki = 26.5;
    float Kd = 0.0;

    int target_speed = 0; // 设定目标转速
    int MAX_PWM = 7000;    // PWM最大限制

    // 左轮 PID 变量
    int speed_out_L = 0;
    int last_error_L = 0;
    
    // 右轮 PID 变量
    int speed_out_R = 0;
    int last_error_R = 0;

    int loop_count = 0;

    while(!exit_vofa_demo)
    {
        // ==============================================
        // 1. 获取当前速度
        // ==============================================
        int current_encoder_L = - encoder_L.encoder_get_count();
        int current_encoder_R = encoder_R.encoder_get_count();

        // ==============================================
        // 2. 计算误差
        // ==============================================
        int error_L = target_speed - current_encoder_L;
        int error_R = target_speed - current_encoder_R; 

        // ==============================================
        // 3. 增量式(近似) PID 计算 (P项为加号)
        // ==============================================
        speed_out_L += Ki * error_L + Kp * (error_L - last_error_L) + Kd * (error_L - 2 * last_error_L + current_encoder_L);
        speed_out_R += Ki * error_R + Kp * (error_R - last_error_R) + Kd * (error_R - 2 * last_error_R + current_encoder_R);

        last_error_L = error_L;
        last_error_R = error_R;

        // ==============================================
        // 4. PWM 限幅
        // ==============================================
        if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;

        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        // // 停车保护
        // if (target_speed == 0) {
        //     speed_out_L = 0;
        //     speed_out_R = 0;
        //     last_error_L = 0;
        //     last_error_R = 0;
        // }

        // ==============================================
        // 5. 电机方向控制及 PWM 占空比输出
        // ==============================================
        // 左轮
        if (speed_out_L >= 0) {
            motor_L_Dir.gpio_level_set(GPIO_LOW);  
            motor_L.atim_pwm_set_duty(speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_HIGH); 
            motor_L.atim_pwm_set_duty(-speed_out_L);
        }

        // 右轮
        if (speed_out_R >= 0) {
            motor_R_Dir.gpio_level_set(GPIO_LOW);  
            motor_R.atim_pwm_set_duty(speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_HIGH); 
            motor_R.atim_pwm_set_duty(-speed_out_R);
        }

        // ==============================================
        // 6. 通过 VOFA+ 客户端发送波形数据 (通道: 目标速度, 左轮实际, 左轮PWM, 右轮实际, 右轮PWM)
        //    使用 FireWater 协议: ware:xxx,xxx,xxx\n
        // ==============================================
        if(client.is_connected()) {
            char send_buf[128];
            // int len = snprintf(send_buf, sizeof(send_buf), "ware:%.3f,%.3f,%.3f,%.3f,%.3f\n", 
            //                    (float)target_speed, 
            //                    (float)current_encoder_L, (float)speed_out_L,
            //                    (float)current_encoder_R, (float)speed_out_R);
            int len = snprintf(send_buf, sizeof(send_buf), "ware:%.3f,%.3f\n", 
                               (float)target_speed, (float)current_encoder_L);
            if (len > 0) {
                client.tcp_send(send_buf, len);
            }
        } else {
            client.tcp_client_init(SERVER_IP, SERVER_PORT); // 尝试重连
        }

        // 控制台打印 (降低频率防止卡顿)
        loop_count++;
        if (loop_count >= 50) { 
            std::cout << ">>> 正在持续发送 VOFA 波形数据... Target Speed: " << target_speed << std::endl;
            loop_count = 0;
        }

        // 控制周期 20ms (闭环频率 50Hz) (可根据实际需求调整为 5000)
        usleep(20000); 
    }

    // ==============================================
    // 安全断电与回收资源
    // ==============================================
    std::cout << "正在关闭电机与连接..." << std::endl;
    motor_L.atim_pwm_set_duty(0);
    motor_R.atim_pwm_set_duty(0);
    motor_L.atim_pwm_disable();
    motor_R.atim_pwm_disable();
    
    if (client.is_connected()) {
        client.tcp_close();
    }
}