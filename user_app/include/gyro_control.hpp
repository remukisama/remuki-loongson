// #ifndef __GYRO_CONTROL_HPP
// #define __GYRO_CONTROL_HPP

// #include <stdint.h>

// // ================== 内外环PID参数 (首次调试请从这里改) ==================
// // 外环：视觉偏差 -> 目标角速度
// #define OUT_KP 2.5f    // 视觉外环 P：偏差多大时期望产生多大的转向角速度
// #define OUT_KD 0.8f    // 视觉外环 D：预测弯道，消除画龙

// // 内环：角速度误差 -> 电机差速
// #define IN_KP  3.0f    // 陀螺仪内环 P：抵抗外力的能力，越大响应越快
// #define IN_KD  0.5f    // 陀螺仪内环 D：提供阻尼，防止震荡打手
// #define IN_KI  0.0f    // 内环积分：方向环通常强烈建议设为了0
// // =======================================================================

// // 差速输出最大极值限幅（根据基础速度动态调整，或写死）
// #define MAX_DIFF_SPEED 300 

// typedef struct {
//     float Kp;
//     float Ki;
//     float Kd;

//     float target;       // 目标值
//     float actual;       // 实际测量值
    
//     float error;        // 当前误差
//     float last_error;   // 上次误差
    
//     float out;          // 最终输出
// } CascadePID_t;

// extern CascadePID_t Visual_Outer_PID;
// extern CascadePID_t Gyro_Inner_PID;

// // 初始化串级PID参数
// void Cascade_PID_Init(void);

// // 计算串级PID，返回最终的电机目标差速（diff_speed）
// // 参数：
// // steer_error: 视觉计算出的赛道偏差 (MiddleLine - Det_True)
// // actual_gz  : 从陀螺仪获取的实时 Z 轴角速度
// float Calculate_Cascade_PID(float steer_error, float actual_gz);

// #endif // __GYRO_CONTROL_HPP
