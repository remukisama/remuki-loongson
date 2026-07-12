#include "lq_all_demo.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>

// ==========================================
// 【LLM 自动PID调参演示程序】
// 配合 llm-pid-tuner 上位机使用
// ==========================================

// 用于安全退出
static std::atomic<bool> exit_demo(false);

void signal_handler_autotune(int signum) {
    exit_demo = true;
}

// ==========================================
// PID 控制器结构体
// ==========================================
struct PIDController {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_limit;
    float integral_limit;
    
    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
    }
    
    float compute(float error, float dt) {
        // 积分项（带抗饱和）
        integral += error * dt;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;
        
        // 微分项
        float derivative = (error - prev_error) / dt;
        
        // PID输出
        float output = kp * error + ki * integral + kd * derivative;
        
        // 输出限幅
        if (output > output_limit) output = output_limit;
        if (output < -output_limit) output = -output_limit;
        
        prev_error = error;
        return output;
    }
};

// ==========================================
// 串口数据上报函数
// ==========================================
void reportPidData(unsigned long timestamp_ms, float setpoint, float input, 
                   float pwm, float error, float p, float i, float d) {
    // 上报格式: timestamp_ms,setpoint,input,pwm,error,p,i,d
    printf("%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n",
           timestamp_ms, setpoint, input, pwm, error, p, i, d);
    fflush(stdout);
}

// ==========================================
// 串口指令接收处理
// ==========================================
bool processSerialCommand(PIDController& pid) {
    static char cmd_buf[64];
    static int cmd_len = 0;
    
    // 非阻塞读取串口
    fd_set readfds;
    struct timeval tv;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10000; // 10ms超时
    
    if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0) {
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\n' || c == '\r') {
                if (cmd_len > 0) {
                    cmd_buf[cmd_len] = '\0';
                    
                    // 解析 "SET P:xxx I:xxx D:xxx" 格式
                    float new_p, new_i, new_d;
                    if (sscanf(cmd_buf, "SET P:%f I:%f D:%f", &new_p, &new_i, &new_d) == 3) {
                        pid.kp = new_p;
                        pid.ki = new_i;
                        pid.kd = new_d;
                        pid.reset(); // 重置积分项
                        printf("# PID updated: P=%.1f I=%.1f D=%.1f\n", new_p, new_i, new_d);
                        fflush(stdout);
                        cmd_len = 0;
                        return true;
                    }
                    cmd_len = 0;
                }
            } else if (cmd_len < 63) {
                cmd_buf[cmd_len++] = c;
            }
        }
    }
    return false;
}

// ==========================================
// 主演示函数
// ==========================================
void lq_llm_pid_autotune_demo(void)
{
    std::cout << "====================================================" << std::endl;
    std::cout << "    LLM 自动PID调参演示程序 (配合 llm-pid-tuner)" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "功能说明:" << std::endl;
    std::cout << "  1. 程序会通过串口上报PID数据给上位机" << std::endl;
    std::cout << "  2. 上位机(llm-pid-tuner)会分析数据并通过LLM生成新的PID参数" << std::endl;
    std::cout << "  3. 新参数会通过串口发送回本程序并自动应用" << std::endl;
    std::cout << std::endl;
    std::cout << "使用方法:" << std::endl;
    std::cout << "  1. 在上位机运行: python3 tuner.py" << std::endl;
    std::cout << "  2. 本程序会自动接收并应用LLM建议的参数" << std::endl;
    std::cout << "  3. 按 Ctrl+C 安全退出" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << std::endl;

    // 1. 初始化电机 PWM 控制 (频率25kHz，初始占空比0)
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_NORMAL);
    
    // 2. 初始化电机方向控制引脚
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    // 3. 初始化编码器
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // 4. 初始化PID控制器（使用你当前的参数作为起点）
    PIDController pid_left;
    pid_left.kp = 1800.0f;      // kSpeedPidP
    pid_left.ki = 35.0f;        // kSpeedPidI
    pid_left.kd = 0.0f;         // kSpeedPidD
    pid_left.integral = 0.0f;
    pid_left.prev_error = 0.0f;
    pid_left.output_limit = 8000.0f;  // kMotorMaxPwm
    pid_left.integral_limit = 100.0f;

    PIDController pid_right;
    pid_right.kp = 1800.0f;
    pid_right.ki = 35.0f;
    pid_right.kd = 0.0f;
    pid_right.integral = 0.0f;
    pid_right.prev_error = 0.0f;
    pid_right.output_limit = 8000.0f;
    pid_right.integral_limit = 100.0f;

    // 5. 控制参数
    float target_speed = 0.0f;       // 目标速度 (m/s)
    float current_speed_L = 0.0f;    // 当前左轮速度 (m/s)
    float current_speed_R = 0.0f;    // 当前右轮速度 (m/s)
    float pwm_L = 0.0f;              // 左轮PWM输出
    float pwm_R = 0.0f;              // 右轮PWM输出
    
    // 编码器换算系数: m/s = RPS * 0.08870
    const float encoder_rps_to_ms = 0.08870f;
    const float control_dt = 0.02f;  // 20ms控制周期 (50Hz)
    const int max_pwm = 8000;
    
    // 编码器计数变量
    int encoder_count_L = 0;
    int encoder_count_R = 0;
    int last_encoder_count_L = 0;
    int last_encoder_count_R = 0;
    
    // 时间管理
    auto start_time = std::chrono::steady_clock::now();
    unsigned long timestamp_ms = 0;
    int loop_count = 0;
    
    // 数据上报计数器
    int report_counter = 0;
    const int report_interval = 1; // 每1个控制周期上报一次

    std::cout << "# LLM PID Autotuner Firmware v1.0 Ready" << std::endl;
    std::cout << "# Format: timestamp_ms,setpoint,input,pwm,error,p,i,d" << std::endl;
    std::cout << "# Initial PID: P=" << pid_left.kp << " I=" << pid_left.ki << " D=" << pid_left.kd << std::endl;
    fflush(stdout);

    while(!exit_demo)
    {
        // ==============================================
        // 1. 获取当前时间
        // ==============================================
        auto current_time = std::chrono::steady_clock::now();
        timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count();
        
        // ==============================================
        // 2. 读取编码器并计算速度
        // ==============================================
        encoder_count_L = -encoder_L.encoder_get_count();
        encoder_count_R = encoder_R.encoder_get_count();
        
        // 计算编码器增量（每控制周期）
        int delta_encoder_L = encoder_count_L - last_encoder_count_L;
        int delta_encoder_R = encoder_count_R - last_encoder_count_R;
        
        // 转换为转速 (RPS = 增量 / (编码器每转脉冲数 * 控制周期))
        // 假设编码器每转11个脉冲（根据实际调整）
        float rps_L = (float)delta_encoder_L / (11.0f * control_dt);
        float rps_R = (float)delta_encoder_R / (11.0f * control_dt);
        
        // 转换为线速度 (m/s)
        current_speed_L = rps_L * encoder_rps_to_ms;
        current_speed_R = rps_R * encoder_rps_to_ms;
        
        last_encoder_count_L = encoder_count_L;
        last_encoder_count_R = encoder_count_R;
        
        // ==============================================
        // 3. PID计算（左轮）
        // ==============================================
        float error_L = target_speed - current_speed_L;
        pwm_L = pid_left.compute(error_L, control_dt);
        
        // ==============================================
        // 4. PID计算（右轮）
        // ==============================================
        float error_R = target_speed - current_speed_R;
        pwm_R = pid_right.compute(error_R, control_dt);
        
        // ==============================================
        // 5. 设置电机PWM输出
        // ==============================================
        // 左轮
        if (pwm_L >= 0) {
            motor_L_Dir.gpio_level_set(GPIO_HIGH);
            motor_L.atim_pwm_set_duty((int)pwm_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_LOW);
            motor_L.atim_pwm_set_duty((int)(-pwm_L));
        }
        
        // 右轮
        if (pwm_R >= 0) {
            motor_R_Dir.gpio_level_set(GPIO_HIGH);
            motor_R.atim_pwm_set_duty((int)pwm_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_LOW);
            motor_R.atim_pwm_set_duty((int)(-pwm_R));
        }
        
        // ==============================================
        // 6. 上报数据给LLM调参器
        // ==============================================
        report_counter++;
        if (report_counter >= report_interval) {
            report_counter = 0;
            
            // 上报左轮数据
            reportPidData(timestamp_ms, target_speed, current_speed_L, 
                         pwm_L, error_L, pid_left.kp, pid_left.ki, pid_left.kd);
        }
        
        // ==============================================
        // 7. 处理串口指令（接收LLM返回的新参数）
        // ==============================================
        processSerialCommand(pid_left);
        
        // 右轮使用相同参数（可以修改为独立调参）
        pid_right.kp = pid_left.kp;
        pid_right.ki = pid_left.ki;
        pid_right.kd = pid_left.kd;
        
        // ==============================================
        // 8. 控制周期延时
        // ==============================================
        loop_count++;
        
        // 每10秒打印一次状态
        if (loop_count % 500 == 0) {
            fprintf(stderr, "# [T=%lums] Target=%.2f L=%.2f R=%.2f PWM_L=%.0f PWM_R=%.0f P=%.1f I=%.1f D=%.1f\n",
                    timestamp_ms, target_speed, current_speed_L, current_speed_R,
                    pwm_L, pwm_R, pid_left.kp, pid_left.ki, pid_left.kd);
        }
        
        // 使用高精度延时
        auto next_time = start_time + std::chrono::milliseconds((long long)(loop_count * control_dt * 1000));
        std::this_thread::sleep_until(next_time);
    }

    // ==============================================
    // 清理：关闭电机
    // ==============================================
    motor_L.atim_pwm_set_duty(0);
    motor_R.atim_pwm_set_duty(0);
    
    std::cout << std::endl;
    std::cout << "# Demo stopped. Final PID: P=" << pid_left.kp 
              << " I=" << pid_left.ki << " D=" << pid_left.kd << std::endl;
    std::cout << "# Total loops: " << loop_count << std::endl;
}
