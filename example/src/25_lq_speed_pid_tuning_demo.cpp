#include "lq_all_demo.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>

// ==========================================
// 【速度环(PID) 调参演示程序】
// ==========================================

// 用于安全退出
static std::atomic<bool> exit_demo(false);

void signal_handler_tuning(int signum) {
    exit_demo = true;
}

void lq_speed_pid_tuning_demo(void)
{
    std::cout << "========== 速度环(PID) 调参演示程序启动 ==========" << std::endl;
    std::cout << "说明: 您可以在代码中修改Kp, Ki, Kd参数来观察电机响应。" << std::endl;
    std::cout << "按 Ctrl+C 安全退出。" << std::endl;

    // 1. 初始化电机 PWM 控制 (频率10KHz，初始占空比0)
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 10000, 10000, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 10000, 10000, ATIM_PWM_POL_NORMAL);
    
    // 2. 初始化电机方向控制引脚
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    // 3. 初始化编码器 (使用普通PWM脉冲+方向编码器)
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // encoder_L.encoder_reset_counter();
    // encoder_R.encoder_reset_counter();

    // --- PID 调参变量区 ---
    // 【请在这里修改您的 PID 参数】
    float Kp = 60.0;
    float Ki = 6.0;
    float Kd = 0.0;

    int target_speed = 0; // 设定目标转速（比如每控制周期的编码器增量）
    int MAX_PWM = 4000;    // PWM最大限制

    // 左轮 PID 变量
    int speed_out_L = 0;
    int last_error_L = 0;
    
    // 右轮 PID 变量
    int speed_out_R = 0;
    int last_error_R = 0;

    int loop_count = 0;

    while(!exit_demo)
    {
        // ==============================================
        // 1. 获取当前速度 (通过两次读取的差值或者重新清零)
        // ==============================================
        int current_encoder_L = - encoder_L.encoder_get_count();
        int current_encoder_R = encoder_R.encoder_get_count();

        // 读完后立刻清零，这样读出的值就是这一个控制周期内的速度
        // encoder_L.encoder_reset_counter();
        // encoder_R.encoder_reset_counter();

        // ==============================================
        // 2. 计算误差
        // ==============================================
        int error_L = target_speed - current_encoder_L;
        int error_R = target_speed - current_encoder_R; // 视右轮安装方向，可能需要取反，请根据实际调整

        // ==============================================
        // 3. 增量式 PID 计算 (P项为加号)
        // ==============================================
        speed_out_L += Ki * error_L + Kp * (error_L - last_error_L);
        speed_out_R += Ki * error_R + Kp * (error_R - last_error_R);

        last_error_L = error_L;
        last_error_R = error_R;

        // ==============================================
        // 4. PWM 限幅
        // ==============================================
        if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;

        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        // // 停车保护 (目标速度为0时，彻底关断)
        // if (target_speed == 0) {
        //     speed_out_L = 0;
        //     speed_out_R = 0;
        //     last_error_L = 0;
        //     last_error_R = 0;
        // }

        // ==============================================
        // 5. 电机方向控制及 PWM 占空比输出
        // ==============================================
        // 左轮输出
        if (speed_out_L >= 0) {
            motor_L_Dir.gpio_level_set(GPIO_LOW);  // 假设 LOW 为正转
            motor_L.atim_pwm_set_active_low_duty(speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_HIGH); // 反转
            motor_L.atim_pwm_set_active_low_duty(-speed_out_L);
        }

        // 右轮输出
        if (speed_out_R >= 0) {
            motor_R_Dir.gpio_level_set(GPIO_LOW);  // 假设 LOW 为正转
            motor_R.atim_pwm_set_active_low_duty(speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_HIGH); // 反转
            motor_R.atim_pwm_set_active_low_duty(-speed_out_R);
        }

        // ==============================================
        // 6. 打印输出 (可以通过波形软件如串口上位机观察)
        // ==============================================
        loop_count++;
        if (loop_count >= 20) { // 降低打印频率防止终端卡顿 (控制周期 20ms，打印周期 100ms)
            std::cout << "Target: " << target_speed 
                      << " | RealL: " << current_encoder_L 
                      << " | PwmL: " << speed_out_L
                      << " | errorL: " << error_L
                      << " || RealR: " << current_encoder_R // 取反以匹配目标速度的符号显示
                      << " | PwmR: " << speed_out_R
                      << " | errorR: " << error_R
                      << std::endl;
            loop_count = 0;
        }

        // 控制周期 20ms (闭环频率 50Hz)
        usleep(5000); 
    }

    // 退出程序时的安全断电逻辑
    std::cout << "正在关闭电机..." << std::endl;
    motor_L.atim_pwm_set_active_low_duty(0);
    motor_R.atim_pwm_set_active_low_duty(0);
    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();
}
