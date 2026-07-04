#include "lq_all_demo.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <chrono>

// ==========================================
// 【角速度环(陀螺仪) PID 调参演示程序】
// 该演示建立在已经调好底层“速度闭环”的基础上
// 用于调节三串级差速中的“中层：角速度环”
// ==========================================

static std::atomic<bool> exit_demo(false);

void signal_handler_gyro_tuning(int signum) {
    exit_demo = true;
}

void lq_gyro_pid_tuning_demo(void)
{
    // 1. 初始化两路电机 PWM 
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 10000, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 10000, ATIM_PWM_POL_NORMAL);
    
    // 2. 初始化电机方向引脚
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    // 3. 初始化编码器
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // 4. 初始化 MPU6050 陀螺仪
    lq_i2c_mpu6050 mpu6050;
    while(mpu6050.get_mpu6050_id() != 0x68) {
        std::cout << "MPU6050 初始化失败, 检查连线..." << std::endl;
        usleep(1000000); // 等待1秒
        if (exit_demo) return;
    }
    std::cout << "MPU6050 初始化成功!" << std::endl;

    // 静止零偏校准：将 gz 的静止偏置减掉，避免积分和角速度闭环被漂移污染
    constexpr float GYRO_LSB_PER_DPS = 16.4f;
    constexpr int GYRO_BIAS_SAMPLES = 200;
    double gz_bias_sum = 0.0;
    std::cout << "开始采样陀螺仪零偏，请保持车体静止..." << std::endl;
    for (int i = 0; i < GYRO_BIAS_SAMPLES && !exit_demo; ++i) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz);

        (void)ax;
        (void)ay;
        (void)az;
        (void)gx;
        (void)gy;
        gz_bias_sum += gz;
        usleep(10000);
    }
    if (exit_demo) return;
    const double gz_bias = gz_bias_sum / GYRO_BIAS_SAMPLES;
    std::cout << "陀螺仪零偏 gz_bias(raw) = " << gz_bias << std::endl;

    // =================参数配置区=================

    // 【1. 已经调好的内层速度环参数】 (不可再随意变动)
    float speed_Kp = 57.0;
    float speed_Ki = 26.5;

    // 【2. 请在本区调节您的陀螺仪 角速度 PID 参数】
    // float gyro_Kp = 0.8;
    // float gyro_Ki = 0.05;
    // float gyro_Kd = 0.1;
    float gyro_Kp = 0.12; // <--- 请修改为你实际测得的值
    float gyro_Ki = 0.0;
    float gyro_Kd = 0.006;

    // 基础直行速度 (建议给一个安全的速度，因为要测“被推偏”时的抵抗力)
    int base_speed = 12;  
    int target_gyro_z = 0; // 目标角速度为0，即保持直线直行

    // ===========================================

    // 防积分饱和、限幅相关常量
    int MAX_PWM = 4000;
    float GYRO_INT_MAX = 50.0f;
    float DIFF_MAX = 30.0f;    // 最大左右差速配比（先收紧，防甩尾）


    // 内环(Velocity)控制变量
    int speed_out_L = 0;
    int last_speed_error_L = 0;
    int speed_out_R = 0;
    int last_speed_error_R = 0;

    // 外环(Gyro)控制变量
    float gyro_integral = 0;
    float last_gyro_error = 0;

    int loop_count = 0;

    auto last_time = std::chrono::steady_clock::now();

    while(!exit_demo)
    {
        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = now_time - last_time;
        float dt = fs.count();
        last_time = now_time;
        // 对齐 verify demo 的 chrono 实时 dt 计算，并补充极值保护避免 D 项突刺
        if (dt < 0.001f || dt > 0.1f) dt = 0.01f;

        // ==============================================
        // 1. 获取并处理 MPU6050 数据
        // ==============================================
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz);

        // 离群值抑制：当raw跳变异常大时，使用上一帧raw值替代，避免单帧尖峰拉爆控制
        static int16_t last_gz_raw = 0;
        int16_t gz_raw_used = gz;
        if (std::abs((int)gz - (int)last_gz_raw) > 900) {
            gz_raw_used = last_gz_raw;
        }
        last_gz_raw = gz_raw_used;
        
        // 简单转换为 度/秒 (MPU6050默认量程量程 +/-2000 dps 下，灵敏度约为 16.4 LSB/°/s)
        // 实际你的底层库可能有零漂校准补偿，演示这里直接转换
        float real_gyro_z = ((float)gz_raw_used - static_cast<float>(gz_bias)) / GYRO_LSB_PER_DPS;

        // IMU测量硬限幅：抑制异常尖峰直接拉爆控制输出
        if (real_gyro_z > 180.0f) real_gyro_z = 180.0f;
        if (real_gyro_z < -180.0f) real_gyro_z = -180.0f;

        // 一阶低通：降低高频噪声对误差和D项的放大
        static float gyro_z_filt = 0.0f;
        const float gyro_lpf_alpha = 0.25f;
        gyro_z_filt = gyro_z_filt * (1.0f - gyro_lpf_alpha) + real_gyro_z * gyro_lpf_alpha;
        real_gyro_z = gyro_z_filt;

        // 屏蔽陀螺仪静止状态的微小零漂(去底噪)
        if(real_gyro_z > -1.5f && real_gyro_z < 1.5f) real_gyro_z = 0.0f;

        // ==============================================
        // 2. 角速度环(中层) PID 计算 -> 得到左右差速
        // ==============================================
        float gyro_error = target_gyro_z - real_gyro_z;
        
        gyro_integral += gyro_error * dt;
        if(gyro_integral > GYRO_INT_MAX) gyro_integral = GYRO_INT_MAX;
        if(gyro_integral < -GYRO_INT_MAX) gyro_integral = -GYRO_INT_MAX;

        // PI(D) 公式 
        float diff_speed = gyro_Kp * gyro_error + 
                           gyro_Ki * gyro_integral + 
                           gyro_Kd * (gyro_error - last_gyro_error) / dt;
                           
        last_gyro_error = gyro_error;

        // 差速变化率限制：减小单周期输出突变导致的甩尾
        static float last_diff_speed = 0.0f;
        const float max_diff_step = 2.0f; // 每10ms最大差速变化
        float diff_step = diff_speed - last_diff_speed;
        if (diff_step > max_diff_step) diff_step = max_diff_step;
        if (diff_step < -max_diff_step) diff_step = -max_diff_step;
        diff_speed = last_diff_speed + diff_step;
        last_diff_speed = diff_speed;

        // 差速限幅保护
        if (diff_speed > DIFF_MAX) diff_speed = DIFF_MAX;
        if (diff_speed < -DIFF_MAX) diff_speed = -DIFF_MAX;

        // 直行目标下的极端角速度保护：遇到异常高角速度时额外收紧差速，防止瞬时甩尾
        if (target_gyro_z == 0 && std::abs(real_gyro_z) > 120.0f) {
            if (diff_speed > 8.0f) diff_speed = 8.0f;
            if (diff_speed < -8.0f) diff_speed = -8.0f;
        }

        // ==============================================
        // 3. 将差速分配给左右轮目标速度
        // ==============================================
        int target_speed_L = base_speed - diff_speed;
        int target_speed_R = base_speed + diff_speed;

        // ==============================================
        // 4. 执行原有的速度内环控制
        // ==============================================
        int current_encoder_L = - encoder_L.encoder_get_count();
        int current_encoder_R = encoder_R.encoder_get_count();

        // 计算轮速误差
        int speed_error_L = target_speed_L - current_encoder_L;
        int speed_error_R = target_speed_R - current_encoder_R; 

        // 增量式轮速 PID 计算
        speed_out_L += speed_Ki * speed_error_L + speed_Kp * (speed_error_L - last_speed_error_L);
        speed_out_R += speed_Ki * speed_error_R + speed_Kp * (speed_error_R - last_speed_error_R);

        last_speed_error_L = speed_error_L;
        last_speed_error_R = speed_error_R;

        // 轮速最终限幅
        if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        // // 停车保护
        // if (base_speed == 0 && target_gyro_z == 0) {
        //     speed_out_L = 0; speed_out_R = 0;
        //     last_speed_error_L = 0; last_speed_error_R = 0;
        // }

        // 输出PWM给左轮
        if (speed_out_L >= 0) {
            motor_L_Dir.gpio_level_set(GPIO_HIGH);
            motor_L.atim_pwm_set_active_low_duty(speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_LOW);
            motor_L.atim_pwm_set_duty(-speed_out_L);
        }

        // 输出PWM给右轮
        if (speed_out_R >= 0) {
            motor_R_Dir.gpio_level_set(GPIO_HIGH);
            motor_R.atim_pwm_set_active_low_duty(speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_LOW);
            motor_R.atim_pwm_set_duty(-speed_out_R);
        }

        // ==============================================
        // 5. 打印串口调试波形信息 (每10个周期=100ms打印一次)
        // ==============================================
        loop_count++;
        if (loop_count >= 10) { 
            std::cout << "GZ_Raw: " << gz
                      << " | real_Gz: " << real_gyro_z 
                      << " | gyro_Err: " << gyro_error
                      << " | DIFF: " << diff_speed
                      << " || trg_L: " << target_speed_L
                      << " | trg_R: " << target_speed_R
                      << std::endl;
            loop_count = 0;
        }

        // 延时 10ms (100Hz 的陀螺仪PID更新频率)
        usleep(10000); 
    }

    std::cout << "正在关闭电机..." << std::endl;
    motor_L_Dir.gpio_level_set(GPIO_HIGH);
    motor_R_Dir.gpio_level_set(GPIO_HIGH);
    motor_L.atim_pwm_set_active_low_duty(0);
    motor_R.atim_pwm_set_active_low_duty(0);
    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();
}
