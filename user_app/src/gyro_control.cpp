// #include "gyro_control.hpp"
// #include <math.h>

// // 实例化内外环控制器
// CascadePID_t Visual_Outer_PID;
// CascadePID_t Gyro_Inner_PID;

// /**
//  * @brief  初始化串级 PID 参数
//  */
// void Cascade_PID_Init(void)
// {
//     // 初始化外环（视觉）参数
//     Visual_Outer_PID.Kp = OUT_KP;
//     Visual_Outer_PID.Kd = OUT_KD;
//     Visual_Outer_PID.Ki = 0.0f; // 外环禁止使用积分！
//     Visual_Outer_PID.error = 0;
//     Visual_Outer_PID.last_error = 0;
//     Visual_Outer_PID.out = 0;

//     // 初始化内环（陀螺仪）参数
//     Gyro_Inner_PID.Kp = IN_KP;
//     Gyro_Inner_PID.Kd = IN_KD;
//     Gyro_Inner_PID.Ki = IN_KI; 
//     Gyro_Inner_PID.error = 0;
//     Gyro_Inner_PID.last_error = 0;
//     Gyro_Inner_PID.out = 0;
// }

// /**
//  * @brief   计算串级PID并返回电机差速请求
//  * @param   steer_error: 外环输入（视觉偏差）
//  * @param   actual_gz  : 内环输入（当前真实角速度，例如来自ICM的icm_gz）
//  * @return  diff_speed : 返回给左右电机的速度差(供分配)
//  */
// float Calculate_Cascade_PID(float steer_error, float actual_gz)
// {
//     // =========================================================
//     // 1. 【外环闭环】位置式PD计算：输入视觉偏差，输出目标角速度
//     // =========================================================
//     Visual_Outer_PID.actual = steer_error; 
//     Visual_Outer_PID.target = 0; // 我们的最终目标永远是偏差归零
    
//     // 计算外环误差
//     Visual_Outer_PID.error = Visual_Outer_PID.target - Visual_Outer_PID.actual;

//     // 外环PD计算 (目标角速度产生)
//     Visual_Outer_PID.out = Visual_Outer_PID.Kp * Visual_Outer_PID.error + 
//                            Visual_Outer_PID.Kd * (Visual_Outer_PID.error - Visual_Outer_PID.last_error);

//     Visual_Outer_PID.last_error = Visual_Outer_PID.error;

//     // 选做：外环输出限幅，即限制小车能接受的最大目标转弯速率 (例如 300°/s)
//     if(Visual_Outer_PID.out > 300.0f)  Visual_Outer_PID.out = 300.0f;
//     if(Visual_Outer_PID.out < -300.0f) Visual_Outer_PID.out = -300.0f;


//     // =========================================================
//     // 2. 【内环闭环】位置式PD计算：输入目标角速度和实际角速度，输出电机差速
//     // =========================================================
//     // 把外环的输出作为内环的输入目标
//     Gyro_Inner_PID.target = Visual_Outer_PID.out;
//     Gyro_Inner_PID.actual = actual_gz;
    
//     Gyro_Inner_PID.error = Gyro_Inner_PID.target - Gyro_Inner_PID.actual;

//     // 内环PD计算 (电机差速量产生)
//     Gyro_Inner_PID.out = Gyro_Inner_PID.Kp * Gyro_Inner_PID.error + 
//                          Gyro_Inner_PID.Kd * (Gyro_Inner_PID.error - Gyro_Inner_PID.last_error);
    
//     Gyro_Inner_PID.last_error = Gyro_Inner_PID.error;

//     // =========================================================
//     // 3. 【限幅输出】防止差速过大导致翻车或原地打死
//     // =========================================================
//     float diff_speed = Gyro_Inner_PID.out;

//     if(diff_speed > MAX_DIFF_SPEED)  diff_speed = MAX_DIFF_SPEED;
//     if(diff_speed < -MAX_DIFF_SPEED) diff_speed = -MAX_DIFF_SPEED;

//     return diff_speed;
// }
