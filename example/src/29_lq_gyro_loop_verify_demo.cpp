#include "lq_all_demo.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <chrono>

// 引入你工程中的控制类（如果编译找不到，请在CMake或Makefile中添加user_app/include的包含路径）
#include "../../user_app/include/my_cascade_pid.hpp"
#include "../../user_app/include/my_mpu6050.hpp"

// ==========================================
// 【角速度环(中环) 输出差速验证演示程序】
// 用途：通过给定一个阶跃的“目标角速度”（如45度/秒），
//       观察小车的实际角速度能否快速、平稳地跟随目标，
//       以此来判断你的前馈系数、P、I、D参数是否合适。
// ==========================================

static std::atomic<bool> exit_demo(false);

void signal_handler_gyro_verify(int signum) {
    exit_demo = true;
}

void lq_gyro_loop_verify_demo(void)
{
    // 1. 初始化两路电机 PWM 
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_INV);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_INV);
    
    // 2. 初始化电机方向引脚
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    // 3. 初始化编码器
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // 4. 初始化你写好的 IMU 和 控制器
    MyMpu6050 imu;
    if (!imu.init()) {
        std::cout << "IMU初始化失败，退出测试!" << std::endl;
        return;
    }

    AttitudeCascade cascade_ctrl;
    cascade_ctrl.init(); // 初始化PID参数

    // =================参数配置区=================

    // 已经调好的内层速度环参数（照抄你工程里的）
    float speed_Kp = 57.0;
    float speed_Ki = 26.5;
    int MAX_PWM = 4000;

    // 基础直行速度
    int base_speed = 0; // 设为0代表原地转圈测试。如果想测行进间转向，可设为 20

    // 测试周期：每隔几秒改变一次目标角速度
    float test_targets[] = {0.0f, 30.0f, 0.0f, -30.0f, 45.0f, 0.0f};
    int current_stage = 0;
    auto stage_start_time = std::chrono::steady_clock::now();
    float target_gyro_z = test_targets[0];

    // 内环(Velocity)控制变量
    float speed_out_L = 0;
    float last_speed_error_L = 0;
    float speed_out_R = 0;
    float last_speed_error_R = 0;
    int loop_count = 0;

    auto last_time = std::chrono::steady_clock::now();

    std::cout << "====== 角速度环跟踪测试开始 ======" << std::endl;
    std::cout << "观察实际角速度 (Real_Gz) 是否能贴合 目标角速度 (Trg_Gz)" << std::endl;

    while(!exit_demo)
    {
        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = now_time - last_time;
        float dt = fs.count();
        last_time = now_time;
        if (dt < 0.001f || dt > 0.1f) dt = 0.01f;
        
        // 阶段切换逻辑：每3秒切换一次目标角速度
        std::chrono::duration<float> stage_elapsed = now_time - stage_start_time;
        if (stage_elapsed.count() > 3.0f) {
            current_stage++;
            if (current_stage >= (sizeof(test_targets)/sizeof(float))) {
                std::cout << "测试序列完成！" << std::endl;
                break; // 结束测试
            }
            target_gyro_z = test_targets[current_stage];
            stage_start_time = now_time;
            std::cout << "\n>>> 切换目标角速度至: " << target_gyro_z << " 度/秒 <<<\n" << std::endl;
        }

        // 1. 获取统一处理后的 IMU 数据
        Mpu6050Data imu_data;
        if (!imu.read_processed_data(imu_data)) {
            usleep(10000);
            continue;
        }

        // 2. 将目标角速度送入你修改后的串级控制器
        //    观察算出的 differ_speed 是否能让车体真正达到 target_gyro_z
        float diff_speed = cascade_ctrl.calc_diff_speed_bypass_angle(target_gyro_z, imu_data, dt);

        // 3. 差速分配给左右轮
        float target_speed_L = base_speed - diff_speed;
        float target_speed_R = base_speed + diff_speed;

        // 4. 执行原有的速度内环控制
        float current_encoder_L = -encoder_L.encoder_get_count();
        float current_encoder_R = encoder_R.encoder_get_count();

        float speed_error_L = target_speed_L - current_encoder_L;
        float speed_error_R = target_speed_R - current_encoder_R; 

        speed_out_L += speed_Ki * speed_error_L + speed_Kp * (speed_error_L - last_speed_error_L);
        speed_out_R += speed_Ki * speed_error_R + speed_Kp * (speed_error_R - last_speed_error_R);

        last_speed_error_L = speed_error_L;
        last_speed_error_R = speed_error_R;

        // 电机限幅与输出
        if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        if (speed_out_L >= 0) {
            motor_L_Dir.gpio_level_set(GPIO_LOW);  
            motor_L.atim_pwm_set_duty(speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_HIGH); 
            motor_L.atim_pwm_set_duty(-speed_out_L);
        }

        if (speed_out_R >= 0) {
            motor_R_Dir.gpio_level_set(GPIO_LOW);  
            motor_R.atim_pwm_set_duty(speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_HIGH); 
            motor_R.atim_pwm_set_duty(-speed_out_R);
        }

        // 5. 打印对比调试信息
        loop_count++;
        if (loop_count >= 10) { // 每 100ms 打印一次
            std::cout << "Trg_Gz: " << target_gyro_z 
                      << " | Real_Gz: " << imu_data.gyro_z
                      << " | 误差: " << (target_gyro_z - imu_data.gyro_z)
                      << " | 输出差速(Diff): " << diff_speed 
                      << std::endl;
            loop_count = 0;
        }

        usleep(10000); // 周期 10ms
    }

    std::cout << "关闭电机..." << std::endl;
    motor_L.atim_pwm_set_duty(0);
    motor_R.atim_pwm_set_duty(0);
    motor_L.atim_pwm_disable();
    motor_R.atim_pwm_disable();
}
