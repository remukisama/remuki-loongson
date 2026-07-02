#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <atomic>

// MAX_SPEED 已废弃，使用 car_config::kMaxWheelSpeedGoal 代替
// #define MAX_SPEED 100

extern std::atomic<bool> exit_flag;
void emergency_stop();
void signal_handler(int signum);

extern float speed_encoder_L;
extern float speed_encoder_R;
extern float speed_out_r;
extern float speed_out_l;
extern float Speed_Goal_l;
extern float Speed_Goal_r;

// extern ls_gpio beep;

void motor_Init(void);
void motor_Duty_L(int duty);
void motor_Duty_R(int duty);
void control_mark_vision_frame_ready(void);
void control_reset_vision_health(void);

//闭环
bool Speed_decision(void);
void Speed_PID_L(void);
void Speed_PID_R(void);
void Speed_BiHuan(void);
#endif // !__MOTOR_H__
