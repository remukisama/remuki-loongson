#ifndef __MY_CASCADE_PID_HPP
#define __MY_CASCADE_PID_HPP

#include "my_mpu6050.hpp"

// 通用 PID 参数结构体
struct GeneralPID {
    float P;
    float I;
    float D;
    float err;
    float last_err;
    float integral;
    float int_max; // 积分限幅
    float out_max; // 输出限幅
    float output;
};

// 三串级控制中的前两环（角度环、角速度环）
class AttitudeCascade {
public:
    AttitudeCascade();
    void init();

    /**
     * @brief 级联PID计算函数
     * @param target_angle 目标角度 (比如直行时目标偏航角为上次的值，转向时为新角度)
     * @param imu_data     当前MPU6050读取的数据
     * @param dt           控制周期 (秒, 例如10ms传 0.01f)
     * @return 返回分配给左右轮的差速量(diff_speed)，送入内层速度环
     */
    float calc_diff_speed(float target_angle, const Mpu6050Data& imu_data, float dt);

    /**
     * @brief 级联PID计算函数（旁路角度环，直接控制角速度环）
     * @param target_gyro_z 目标角速度
     * @param imu_data     当前MPU6050读取的数据
     * @param dt           控制周期
     * @return 返回分配给左右轮的差速量(diff_speed)
     */
    float calc_diff_speed_bypass_angle(float target_gyro_z, const Mpu6050Data& imu_data, float dt);

    GeneralPID angle_pid; // [最外环]：角度环
    GeneralPID gyro_pid;  // [中间环]：角速度环
    
    float current_yaw;    // 航向角积分值
};

#endif
