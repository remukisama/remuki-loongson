#include "lq_all_demo.hpp"
#include <cmath>



/********************************************************************************
 * @brief   纯软件指令前馈闭环调速 Demo (解决 AB 相跳变问题)
 ********************************************************************************/
void lq_motor_pid_demo(void)
{
    // // 1. 初始化电机 PWM 控制 (频率10KHz，初始占空比0)
    // ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 10000, 10000, ATIM_PWM_POL_NORMAL);
    // ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 10000, 10000, ATIM_PWM_POL_NORMAL);
    
    // // 2. 初始化电机方向控制引脚
    // ls_gpio pwm1dir(PIN_21, GPIO_MODE_OUT);
    // ls_gpio pwm2dir(PIN_22, GPIO_MODE_OUT);
    
    // // 3. 初始化编码器
    // // ls_encoder_pwm enc1(ENC_PWM0_PIN64, PIN_72);
    // // ls_encoder_pwm enc2(ENC_PWM3_PIN67, PIN_75);

    // // --- PID 和速度控制变量 ---
    // float target_speed = 50.0;     // 目标转速 (例如：5转/秒)
    // float current_pwm1  = 0.0;     // 累计的 PWM 占空比输出 (-10000 到 10000)
    // float current_pwm2  = 0.0;     // 累计的 PWM 占空比输出 (-10000 到 10000)
    // float Kp = 1.0;              // 简单的 比例系数 (P)，需根据实际慢慢调大
    // float Ki = 0.1;
    // float Kd = 4.0;
    // int loop_cnt = 0;             // 用来演示改变方向的计数器

    // printf("--- Motor PID Control Demo Started ---\n");

    // while(1)
    // {
    //     // ====================================================================
    //     // 【核心 Hack 机制】
    //     // 1. 从硬件获取精准的速度绝对值（这本质上是在读频率，不会丢步，不会跳变）
    //     float raw_speed1 = std::abs(enc1.encoder_get_count());
    //     float raw_speed2 = std::abs(enc2.encoder_get_count());

    //     // 2. 根据我们要电机转动的目标方向（target_speed的符号），人为赋予速度符号！
    //     float real_speed1 = raw_speed1;
    //     float real_speed2 = raw_speed2;
    //     if (target_speed < 0) 
    //     {
    //         real_speed1 = -raw_speed1; // 强制设定为负速
    //         real_speed2 = -raw_speed2;
    //     }
    //     // ====================================================================

    //     // 3. 计算速度误差 (误差 = 期望 - 实际)
    //     float error1 = target_speed - real_speed1;
    //     float error2 = target_speed - real_speed2;
    //     float last_error1 = error1;
    //     float last_error2 = error2;
    //     // 4. 进行简单的 P 闭环计算 (此处可以将就用增量式P控制)
    //     current_pwm1 += (Kp * error1) + (Ki * error1) - (Kd * (error1 - last_error1));
    //     current_pwm2 += (Kp * error2) + (Ki * error2) - (Kd * (error2 - last_error2));
    //     // 5. 限制 PWM 占空比极值 (库里的 API 是 0~10000)
    //     if (current_pwm1 >  2000.0) current_pwm1 =  2000.0;
    //     if (current_pwm1 < -2000.0) current_pwm1 = -2000.0;
    //     if (current_pwm2 >  2000.0) current_pwm2 =  2000.0;
    //     if (current_pwm2 < -2000.0) current_pwm2 = -2000.0;

    //     // 6. 执行到底层电机驱动
    //     if (current_pwm1 >= 0) 
    //     {
    //         pwm1dir.gpio_level_set(GPIO_HIGH); // DRV8701P: IN1=HIGH, IN2低有效PWM为正转
    //         pwm1.atim_pwm_set_active_low_duty((uint32_t)current_pwm1);
    //     } 
    //     else 
    //     {
    //         pwm1dir.gpio_level_set(GPIO_LOW); // DRV8701P: IN1=LOW, IN2高有效PWM为反转
    //         pwm1.atim_pwm_set_duty((uint32_t)(-current_pwm1)); // duty不能接收负数
    //     }

    //     if (current_pwm2 >= 0) 
    //     {
    //         pwm2dir.gpio_level_set(GPIO_HIGH); // DRV8701P: IN1=HIGH, IN2低有效PWM为正转
    //         pwm2.atim_pwm_set_active_low_duty((uint32_t)current_pwm2);
    //     } 
    //     else 
    //     {
    //         pwm2dir.gpio_level_set(GPIO_LOW); // DRV8701P: IN1=LOW, IN2高有效PWM为反转
    //         pwm2.atim_pwm_set_duty((uint32_t)(-current_pwm2)); // duty不能接收负数
    //     }

    //     // // --- 打印信息查看平滑度 ---
    //     // printf("Target: %-6.2f | Real: %-6.2f | PWM Oup: %-6.0f\n", target_speed, real_speed1, current_pwm1);
    //     // printf("Target: %-6.2f | Real: %-6.2f | PWM Oup: %-6.0f\n", target_speed, real_speed2, current_pwm2);
    //     printf("L_Tar: %5.1f  | L_Real: %7.1f | L_PWM: %-5.0f || R_Tar: %5.1f | R_Real: %7.1f | R_PWM: %-5.0f \n", 
    //             target_speed, real_speed1, current_pwm1, target_speed, real_speed2, current_pwm2);
    //     // ====================================================================
    //     // 演示：每 5 秒钟左右自动翻转一次目标速度方向，检测平滑性
    //     // ====================================================================
    //     loop_cnt++;
    //     if (loop_cnt > 2500) 
    //     {
    //         target_speed = -target_speed; // 目标速度正负反转
    //         loop_cnt = 0;
    //         printf("\n>>> Target Speed Inverse! <<<\n\n");
    //     }

    //     // 以 50Hz (20ms) 的频率运行这个控制循环 (闭环频率)
    //     usleep(20000); 
    // }
}
