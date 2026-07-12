#include "lq_all_demo.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <chrono>

// ==========================================
// 【角度环(偏航角) PID 调参演示程序】
// 该演示建立在已经调好底层“速度环”和“角速度环”的基础上
// 用于调节三串级差速中的“外层：角度环”
// ==========================================

static std::atomic<bool> exit_demo(false);

void signal_handler_angle_tuning(int signum) {
    exit_demo = true;
}

void lq_angle_pid_tuning_demo(void)
{
    // 1. 初始化两路电机 PWM 
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_NORMAL);
    
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
        usleep(1000000); 
        if (exit_demo) return;
    }
    std::cout << "MPU6050 初始化成功!" << std::endl;

    // 静止零偏校准：把陀螺仪静止时的原始偏置减掉，减少角度积分漂移
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

    // 【2. 已经调好的中层角速度环参数】 (填入你刚刚测试得出最完美的 Kp，如0.3、0.5，I 和 D 保持 0)
    float gyro_Kp = 0.2; // <--- 请修改为你刚刚实际测得的值
    float gyro_Ki = 0.0;
    float gyro_Kd = 0.0;

    // 【3. 请在本区调节您的 最外层：角度环 PID 参数】
    // 角度环通常只用 Kp 即可 (纯比例控制：角度偏差越大，期望的角速度就越大)
    float angle_Kp = 11.0; 
    float angle_Ki = 0.0;
    float angle_Kd = 0.1;

    // 基础直行速度 这里设为 0，专门看原地转向的表现
    int base_speed = 0;  
    
    // 目标角度：比如开机后要求车子左转 45 度！
    float target_angle = 0.0f; 

    // 【角度补偿系数】
    // 结合诊断结果，这里先恢复为 1.0，避免把原始角速度人为放大。
    float ANGLE_COMPENSATE_RATIO = 1.0f;

    // ===========================================

    // 限幅相关常量
    int MAX_PWM = 4000;
    float GYRO_INT_MAX = 50.0f;
    float DIFF_MAX = 40.0f;    // 最大左右差速配比
    float TARGET_GYRO_Z_MAX = 120.0f; // 限制最大的期望旋转角速度 (度/秒)，避免车子转得太疯狂

    // 内环(Velocity)控制变量
    int speed_out_L = 0;
    int last_speed_error_L = 0;
    int speed_out_R = 0;
    int last_speed_error_R = 0;

    // 中环(Gyro)控制变量
    float gyro_integral = 0;
    float last_gyro_error = 0;

    // 外环(Angle)控制变量
    float current_angle = 0.0f; // 积累的当前角度
    float angle_integral = 0;
    float last_angle_error = 0;

    int loop_count = 0;

    // 弃用固定的 dt = 0.010f，因为 Linux 下 usleep(10ms) 不够精准
    // 使用 chrono 记录真实的时间差以实现完美的陀螺仪积分
    auto last_time = std::chrono::steady_clock::now();

    while(!exit_demo)
    {
        // 动态计算真实的 dt
        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = now_time - last_time;
        float dt = fs.count();
        last_time = now_time;
        if(dt > 0.1f) dt = 0.01f; // 防止系统级卡顿导致单次积分暴增

        // ==============================================
        // 1. 获取并处理 MPU6050 数据
        // ==============================================
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz);
        
        // 简单转换为 度/秒，并乘上补偿系数
        float real_gyro_z = (((float)gz - static_cast<float>(gz_bias)) / GYRO_LSB_PER_DPS) * ANGLE_COMPENSATE_RATIO;

        // 屏蔽陀螺仪静止状态的微小零漂(去底噪)
        if(real_gyro_z > -1.5f && real_gyro_z < 1.5f) real_gyro_z = 0.0f;

        // 积分得到当前绝对角度 (Yaw)
        current_angle += real_gyro_z * dt;

        // ==============================================
        // 2. 角度环(外层) PID 计算 -> 得到目标角速度
        // ==============================================
        float angle_error = target_angle - current_angle;
        
        // （可选）角度环积分，一般角度环不需要 I 和 D
        angle_integral += angle_error * dt;
        
        float target_gyro_z = angle_Kp * angle_error + 
                              angle_Ki * angle_integral + 
                              angle_Kd * (angle_error - last_angle_error) / dt;
        last_angle_error = angle_error;

        // 【限制目标角速度】避免偏差 90 度时，车子渴望以光速旋转
        if (target_gyro_z > TARGET_GYRO_Z_MAX) target_gyro_z = TARGET_GYRO_Z_MAX;
        if (target_gyro_z < -TARGET_GYRO_Z_MAX) target_gyro_z = -TARGET_GYRO_Z_MAX;

        // ==============================================
        // 3. 角速度环(中层) PID 计算 -> 得到左右差速
        // ==============================================
        float gyro_error = target_gyro_z - real_gyro_z;
        
        gyro_integral += gyro_error * dt;
        if(gyro_integral > GYRO_INT_MAX) gyro_integral = GYRO_INT_MAX;
        if(gyro_integral < -GYRO_INT_MAX) gyro_integral = -GYRO_INT_MAX;

        float diff_speed = gyro_Kp * gyro_error + 
                           gyro_Ki * gyro_integral + 
                           gyro_Kd * (gyro_error - last_gyro_error) / dt;
        last_gyro_error = gyro_error;

        if (diff_speed > DIFF_MAX) diff_speed = DIFF_MAX;
        if (diff_speed < -DIFF_MAX) diff_speed = -DIFF_MAX;

        // ==============================================
        // 4. 将差速分配给左右轮目标速度 (套速度环)
        // ==============================================
        int target_speed_L = base_speed - diff_speed;
        int target_speed_R = base_speed + diff_speed;

        // ==============================================
        // 5. 执行原有的速度内环控制
        // ==============================================
        int current_encoder_L = - encoder_L.encoder_get_count();
        int current_encoder_R = encoder_R.encoder_get_count();

        int speed_error_L = target_speed_L - current_encoder_L;
        int speed_error_R = target_speed_R - current_encoder_R; 

        speed_out_L += speed_Ki * speed_error_L + speed_Kp * (speed_error_L - last_speed_error_L);
        speed_out_R += speed_Ki * speed_error_R + speed_Kp * (speed_error_R - last_speed_error_R);

        last_speed_error_L = speed_error_L;
        last_speed_error_R = speed_error_R;

        if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        // 停车与死区保护
        if (base_speed == 0 && target_angle == 0 && (current_angle > -2.0f && current_angle < 2.0f)) {
            speed_out_L = 0; speed_out_R = 0;
            last_speed_error_L = 0; last_speed_error_R = 0;
        }

        if (speed_out_L >= 0) {
            motor_L_Dir.gpio_level_set(GPIO_HIGH);
            motor_L.atim_pwm_set_duty(speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_LOW);
            motor_L.atim_pwm_set_duty(-speed_out_L);
        }

        if (speed_out_R >= 0) {
            motor_R_Dir.gpio_level_set(GPIO_HIGH);
            motor_R.atim_pwm_set_duty(speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_LOW);
            motor_R.atim_pwm_set_duty(-speed_out_R);
        }

        // ==============================================
        // 6. 打印串口调试波形信息 (每10个周期=100ms打印一次)
        // ==============================================
        loop_count++;
        if (loop_count >= 10) { 
            std::cout << "CurAng(deg): " << current_angle
                      << " | ErrAng: " << angle_error
                      << " || Trg_Gz: " << target_gyro_z
                      << " | real_Gz: " << real_gyro_z
                      << " | DIFF: " << diff_speed
                      << std::endl;
            loop_count = 0;
        }

        // 延时 10ms
        usleep(10000); 
    }

    std::cout << "正在关闭电机..." << std::endl;
    motor_L.atim_pwm_set_duty(0);
    motor_R.atim_pwm_set_duty(0);
    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();
}
