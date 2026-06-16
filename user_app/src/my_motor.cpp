#include "main.hpp"
#include "my_cascade_pid.hpp"
#include "car_config.hpp"
#include <chrono>
#include <cmath>

// ===============================================================================
// [1] 全局变量与硬件对象定义
// ===============================================================================

// 全局退出标志与多线程安全
std::atomic<bool> g_program_exit(false);
std::atomic<bool> exit_flag(false);

// 电机与引脚配置
// 左电机 PWM：PIN81, 频率10000Hz, 初始占空比3000, 极性反转
ls_atim_pwm* motor_L = nullptr;  // 延迟初始化，避免全局构造时访问 /dev/mem 导致崩溃
// 右电机 PWM：PIN82, 频率10000Hz, 初始占空比3000, 极性反转
ls_atim_pwm* motor_R = nullptr;  // 延迟初始化，避免全局构造时访问 /dev/mem 导致崩溃
// 电机方向 GPIO 输出
ls_gpio* motor_L_Dir = nullptr;  // 延迟初始化，避免全局构造时访问 /dev/mem 导致崩溃
ls_gpio* motor_R_Dir = nullptr;  // 延迟初始化，避免全局构造时访问 /dev/mem 导致崩溃

// 全局控制参数
float Left_Speed_Co = 0.0010, Right_Speed_Co =0.0010;
float Speed_Goal = 0.0f;
float CS=0.1; // 0.55 0.57 0.58

float ZD_Speed_Goal;
float WD_Speed_Goal;

// 三轮车差速转向专用参数（已完全同步 motor.cpp 验证无误的参数）
// 最新调试参数：恢复部分死区和适中的P，但保留足够的打角能力以应付U型弯
float STEER_KP = car_config::kSteerKp;               // 外环P，集中在 car_config.hpp 调整
float STEER_KD = car_config::kSteerKd;               // 外环D，集中在 car_config.hpp 调整
float MAX_DIFF_RATIO = car_config::kMaxDiffRatio;    // 差速上限=基础速度*倍率
int DEAD_BAND = car_config::kSteerDeadBandDefault;   // 巡线偏差死区
float MIN_SPEED = car_config::kMinWheelSpeedGoal;    // 内轮最低目标速度

// ========================
// 调试追踪专用变量
// ========================
float dbg_steer_error = 0.0f;
float dbg_target_gyro_z = 0.0f;
float dbg_diff_speed = 0.0f;
float dbg_raw_steer_error = 0.0f;
float dbg_diff_steer = 0.0f;
float dbg_visual_pd_out = 0.0f;
float dbg_cmd_target_gyro_z = 0.0f;
float dbg_real_gyro_z = 0.0f;
float dbg_filtered_gyro_z = 0.0f;
float dbg_gyro_error = 0.0f;
float dbg_target_delta = 0.0f;
float dbg_dt = 0.0f;
float dbg_max_diff = 0.0f;
float dbg_diff_before_limit = 0.0f;
float dbg_speed_err_l = 0.0f;
float dbg_speed_err_r = 0.0f;
int dbg_diff_limited = 0;
int dbg_imu_ok = 0;
int dbg_deadband_active = 0;

// 姿态/航向 PID串级控制对象与IMU
AttitudeCascade cascade_ctrl;
MyMpu6050 imu;

// ===============================================================================
// [2] 闭环控制系统变量定义
// ===============================================================================

// 左轮闭环变量
float Speed_Goal_l = 0;
float speed_P_l, speed_I_l, speed_D_l;
float speed_out_l;
float speed_erro_L, speed_last_erro_L;
float speed_encoder_L;
float speed_encoder_raw_L = 0.0f;

// 右轮闭环变量
float Speed_Goal_r = 0;
float speed_P_r, speed_I_r, speed_D_r;
float speed_out_r;
float speed_erro_R, speed_last_erro_R;
float speed_encoder_R;
float speed_encoder_raw_R = 0.0f;

// PWM与路程
int MAX_PWM; // PWM最大限制
float Record_encoder = 0.0f; // 记录编码器 (m/s)


// ===============================================================================
// [3] 硬件初始化与底层驱动函数
// ===============================================================================

void motor_Init(void)
{
    // 延迟初始化：首次调用时才构造硬件对象，避免全局构造时访问 /dev/mem 导致崩溃
    if (!motor_L) {
        motor_L = new ls_atim_pwm(ATIM_PWM0_PIN81, car_config::kMotorPwmFrequencyHz, car_config::kMotorInitialDuty, ATIM_PWM_POL_INV);
        motor_R = new ls_atim_pwm(ATIM_PWM1_PIN82, car_config::kMotorPwmFrequencyHz, car_config::kMotorInitialDuty, ATIM_PWM_POL_INV);
        motor_L_Dir = new ls_gpio(PIN_21, GPIO_MODE_OUT);
        motor_R_Dir = new ls_gpio(PIN_22, GPIO_MODE_OUT);
    }

    motor_R->atim_pwm_set_duty(0);
    motor_L->atim_pwm_set_duty(0);
    
    printf("R:OK!");
    
    motor_R->atim_pwm_enable();
    motor_L->atim_pwm_enable();
    motor_R_Dir->gpio_level_set(GPIO_LOW);
    motor_L_Dir->gpio_level_set(GPIO_LOW);
}

void motor_Duty_R(int duty)
{
    if (duty <= 0)
    {
        motor_R->atim_pwm_set_duty(-duty);
        motor_R_Dir->gpio_level_set(GPIO_HIGH);
    }
    else
    {
        motor_R->atim_pwm_set_duty(duty);
        motor_R_Dir->gpio_level_set(GPIO_LOW);
    }
}

void motor_Duty_L(int duty)
{
    if (duty <= 0)
    {
        motor_L->atim_pwm_set_duty(-duty);
        motor_L_Dir->gpio_level_set(GPIO_HIGH);
    }
    else
    {
        motor_L->atim_pwm_set_duty(duty);
        motor_L_Dir->gpio_level_set(GPIO_LOW);
    }
}


// ===============================================================================
// [4] 控制器初始化与PID计算
// ===============================================================================

// ==========================
// [4.1] 速度环 PID
// ==========================

void Speed_decision(void)
{
    // 参考demo，增量式PI控制
    speed_P_l = car_config::kSpeedPidP;  // 对齐demo参数
    speed_I_l = car_config::kSpeedPidI;  // 对齐demo参数
    speed_D_l = car_config::kSpeedPidD;  // 微分D
    
    speed_P_r = car_config::kSpeedPidP;
    speed_I_r = car_config::kSpeedPidI;
    speed_D_r = car_config::kSpeedPidD;
    
    MAX_PWM = car_config::kMotorMaxPwm;    // PWM最大限制

    speed_out_l = 0;
    speed_out_r = 0;

    WD_Speed_Goal = car_config::kCurveSpeedGoal;      // 弯道基础速度
    ZD_Speed_Goal = car_config::kStraightSpeedGoal;   // 直道基础速度
    cascade_ctrl.init(); // 初始化姿态三串级控制器参数
    static bool imu_inited = false;
    if (!imu_inited) {
        imu.init();          // 初始化IMU
        imu_inited = true;
    }
}

void Speed_PID_L(void)
{
    speed_erro_L = Speed_Goal_l - speed_encoder_L;
    
    // 增量式 PID 计算 (注意加号)
    speed_out_l += speed_I_l * speed_erro_L + speed_P_l * (speed_erro_L - speed_last_erro_L);
                
    speed_last_erro_L = speed_erro_L;

    // 限幅 ±4000
    if (speed_out_l > MAX_PWM) speed_out_l = MAX_PWM;
    if (speed_out_l < -MAX_PWM) speed_out_l = -MAX_PWM;

    // 停车保护
    if (Speed_Goal_l == 0) {
        speed_out_l = 0;
        speed_last_erro_L = 0;
    }
}


void Speed_PID_R(void)
{
    speed_erro_R = Speed_Goal_r - speed_encoder_R;
    
    // 增量式 PID 计算
    speed_out_r += speed_I_r * speed_erro_R + speed_P_r * (speed_erro_R - speed_last_erro_R);
                
    speed_last_erro_R = speed_erro_R;

    // 限幅 ±MAX_PWM
    if (speed_out_r > MAX_PWM) speed_out_r = MAX_PWM;
    if (speed_out_r < -MAX_PWM) speed_out_r = -MAX_PWM;

    // 停车保护
    if (Speed_Goal_r == 0) {
        speed_out_r = 0;
        speed_last_erro_R = 0;
    }
}


// ===============================================================================
// [5] 高级控制与串级逻辑
// ===============================================================================

//差速
// uint16_t CS_LM=50;

// 【中外环替代：姿态串级差速逻辑】将基于 MPU6050 实现三串级：目标角度 -> 目标角角速度 -> 目标差速
void speed_dif_cascade()
{
    static auto last_time = std::chrono::steady_clock::now();
    auto now_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = now_time - last_time;
    float dt = fs.count();
    
    // 用于保存上一次算出的差速分配值
    static float last_diff_speed = 0.0f;
    static float last_effective_speed_goal = 0.0f;
    static int s_curve_hold_frames = 0;
    static int curve_entry_brake_frames = 0;
    static bool was_in_ring = false;
    static float last_target_gyro_z = 0.0f;

    // 停车时彻底清理串级控制运行态，避免重新起步时继承历史状态导致抽动
    if (Speed_Goal <= 0) {
        Speed_Goal_l = 0;
        Speed_Goal_r = 0;
        last_diff_speed = 0.0f;
        last_effective_speed_goal = 0.0f;
        s_curve_hold_frames = 0;
        curve_entry_brake_frames = 0;
        dbg_steer_error = 0.0f;
        dbg_target_gyro_z = 0.0f;
        dbg_diff_speed = 0.0f;
        dbg_raw_steer_error = 0.0f;
        dbg_diff_steer = 0.0f;
        dbg_visual_pd_out = 0.0f;
        dbg_cmd_target_gyro_z = 0.0f;
        dbg_real_gyro_z = 0.0f;
        dbg_filtered_gyro_z = 0.0f;
        dbg_gyro_error = 0.0f;
        dbg_target_delta = 0.0f;
        dbg_max_diff = 0.0f;
        dbg_diff_before_limit = 0.0f;
        dbg_speed_err_l = 0.0f;
        dbg_speed_err_r = 0.0f;
        dbg_diff_limited = 0;
        dbg_imu_ok = 0;
        dbg_deadband_active = 0;
        cascade_ctrl.reset_runtime_state();
        return;
    }

    // 【隐患消除 1 & 3】：执行频率分频。限制外环最高 50Hz (20ms执行一次)
    // 保护内环速度PID足够快，同时防止 I2C 读取耗时占用定时器周期
    if (dt < car_config::kOuterLoopMinPeriodSec) {
        // 如果外环没轮到刷新，沿用上一帧的差速结果，直接分配给当前的 Speed_Goal
        float reuse_speed_goal = (last_effective_speed_goal > 0.0f) ? last_effective_speed_goal : Speed_Goal;
        Speed_Goal_l = reuse_speed_goal - last_diff_speed;
        Speed_Goal_r = reuse_speed_goal + last_diff_speed;
        
        if (reuse_speed_goal <= 0.0f) {
            Speed_Goal_l = 0; Speed_Goal_r = 0;
        } else {
            if (Speed_Goal_l < MIN_SPEED) Speed_Goal_l = MIN_SPEED;
            if (Speed_Goal_r < MIN_SPEED) Speed_Goal_r = MIN_SPEED;
            if (Speed_Goal_l > car_config::kMaxWheelSpeedGoal) Speed_Goal_l = car_config::kMaxWheelSpeedGoal;
            if (Speed_Goal_r > car_config::kMaxWheelSpeedGoal) Speed_Goal_r = car_config::kMaxWheelSpeedGoal;
        }
        return; // 直接返回，跳过外环耗时的 I2C 和 视觉处理
    }

    last_time = now_time;
    if (dt <= 0.0001f || dt > 0.1f) dt = car_config::kOuterLoopFallbackPeriodSec; // 防止系统级卡顿导致的 dt 暴增

    // 1. 【隐患消除 2】：获取赛道中心偏差 (配合外围较低刷新率和低通滤波，彻底杜绝D项尖峰)
    float raw_steer_error = ImageStatus.MiddleLine - ImageStatus.Det_True;
    dbg_raw_steer_error = raw_steer_error;
    dbg_dt = dt;
    const bool ring_now = (ImageFlag.image_element_rings != 0);
    const bool ring_entry_phase = ring_now && ImageFlag.image_element_rings_flag <= 6;
    const bool ring_just_finished = was_in_ring && !ring_now;
    was_in_ring = ring_now;
    const bool bad_vision = ImageStatus.OFFLine >= car_config::kBadVisionOfflineThreshold;
    const bool full_loss_non_ring =
        !ring_now && ImageStatus.OFFLine >= car_config::kBadVisionCenterHoldOfflineThreshold;

    // 限制单帧视觉跳边：真实 Raw 仍写入调试量，控制链路使用限速后的值。
    static bool raw_steer_initialized = false;
    static float last_control_raw_steer_error = 0.0f;
    float control_raw_steer_error = raw_steer_error;
    if (full_loss_non_ring) {
        control_raw_steer_error = 0.0f;
        raw_steer_initialized = false;
        last_diff_speed = 0.0f;
        last_target_gyro_z = 0.0f;
        s_curve_hold_frames = 0;
        curve_entry_brake_frames = 0;
        cascade_ctrl.reset_runtime_state();
    }
    if (ring_just_finished) {
        raw_steer_initialized = false;
        last_control_raw_steer_error = raw_steer_error;
        last_diff_speed = 0.0f;
        last_target_gyro_z = 0.0f;
        s_curve_hold_frames = 0;
        curve_entry_brake_frames = 0;
        cascade_ctrl.reset_runtime_state();
    }
    if (!raw_steer_initialized) {
        raw_steer_initialized = true;
        last_control_raw_steer_error = control_raw_steer_error;
    } else {
        float raw_delta = raw_steer_error - last_control_raw_steer_error;
        if (std::fabs(raw_delta) > car_config::kSteerRawJumpThreshold) {
            raw_delta = (raw_delta > 0.0f) ?
                        car_config::kSteerRawMaxStepPerFrame :
                        -car_config::kSteerRawMaxStepPerFrame;
            control_raw_steer_error = last_control_raw_steer_error + raw_delta;
        }
        last_control_raw_steer_error = control_raw_steer_error;
    }

    // 【新增低通滤波】彻底解决摄像头噪点引起的齿轮咔咔响！
    static float steer_error = 0;
    static float last_steer_error = 0;
    if (ring_just_finished || full_loss_non_ring) {
        steer_error = control_raw_steer_error;
        last_steer_error = steer_error;
    }
    float alpha = car_config::kSteerFilterAlpha;
    bool fast_flip = (control_raw_steer_error * steer_error < 0.0f) &&
                     (std::fabs(control_raw_steer_error) > car_config::kSteerFastFlipRawThreshold) &&
                     (std::fabs(control_raw_steer_error - steer_error) > car_config::kSteerFastFlipGap);
    if (fast_flip) {
        alpha = car_config::kSteerFastFlipAlpha;
    }
    float next_steer_error = steer_error * (1.0f - alpha) + control_raw_steer_error * alpha;
    bool extreme_exit_unwind =
        (std::fabs(control_raw_steer_error) <= car_config::kExtremeTurnExitRawThreshold) &&
        (std::fabs(steer_error) >= car_config::kExtremeTurnExitFiltThreshold);
    if (extreme_exit_unwind) {
        next_steer_error = steer_error * (1.0f - car_config::kExtremeTurnExitFilterAlpha) +
                           control_raw_steer_error * car_config::kExtremeTurnExitFilterAlpha;
    }
    float max_steer_step = car_config::kSteerFilterMaxStepPerFrame;
    if ((next_steer_error * steer_error < 0.0f) &&
        (std::fabs(next_steer_error) > car_config::kSCurveRawMinError) &&
        (std::fabs(steer_error) > car_config::kSCurveRawMinError)) {
        max_steer_step = car_config::kSteerFilterSignFlipMaxStepPerFrame;
    }
    if (extreme_exit_unwind && max_steer_step < car_config::kSteerRawMaxStepPerFrame) {
        max_steer_step = car_config::kSteerRawMaxStepPerFrame;
    }
    float steer_step = next_steer_error - steer_error;
    if (steer_step > max_steer_step) steer_step = max_steer_step;
    if (steer_step < -max_steer_step) steer_step = -max_steer_step;
    steer_error += steer_step;

    // 环岛入口(flag 5/6)需要保持最小转向，防止刚识别后走直；
    // 但到环内/出环阶段(flag>=7)若还一直强制同向转，会在环岛里兜圈，所以释放并给轻微反向出环量。
    if (ImageFlag.image_element_rings == 1 && ImageFlag.image_element_rings_flag >= 5) {
        if (ImageFlag.image_element_rings_flag < car_config::kRingForcedSteerReleaseFlag) {
            if (steer_error < car_config::kRingForcedSteerError) {
                steer_error = car_config::kRingForcedSteerError;
            }
        } else if (steer_error > car_config::kRingExitSteerError) {
            steer_error = car_config::kRingExitSteerError;
        }
    } else if (ImageFlag.image_element_rings == 2 && ImageFlag.image_element_rings_flag >= 5) {
        if (ImageFlag.image_element_rings_flag < car_config::kRingForcedSteerReleaseFlag) {
            if (steer_error > -car_config::kRingForcedSteerError) {
                steer_error = -car_config::kRingForcedSteerError;
            }
        } else if (steer_error < -car_config::kRingExitSteerError) {
            steer_error = -car_config::kRingExitSteerError;
        }
    }

    // 【开机防暴走保护】
    static bool camera_is_ready = false;
    if (!camera_is_ready) {
        if (ImageStatus.Det_True == 0) {
            Speed_Goal_l = 0; 
            Speed_Goal_r = 0;
            return;
        } else {
            camera_is_ready = true; 
        }
    }

    float diff_steer = steer_error - last_steer_error;
    
    dbg_steer_error = steer_error;
    dbg_diff_steer = diff_steer;
    dbg_deadband_active = (abs(steer_error) < DEAD_BAND) ? 1 : 0;

    bool raw_filter_gap = (std::fabs(control_raw_steer_error - steer_error) > car_config::kSCurveRawFiltGap) &&
                          (std::fabs(control_raw_steer_error) > car_config::kSCurveRawMinError);
    bool raw_filter_sign_conflict = (control_raw_steer_error * steer_error < 0.0f) &&
                                    (std::fabs(control_raw_steer_error) > car_config::kSCurveRawMinError);
    bool early_sign_cross = (control_raw_steer_error * steer_error < 0.0f) &&
                            (std::fabs(control_raw_steer_error) > car_config::kSCurveEarlyRawMinError) &&
                            (std::fabs(steer_error) > car_config::kSCurveEarlyFiltMinError) &&
                            (std::fabs(diff_steer) > car_config::kSCurveEarlyDerivativeThreshold);
    bool fast_steer_change = (std::fabs(diff_steer) > car_config::kSCurveDerivativeThreshold) &&
                             (std::fabs(control_raw_steer_error) > car_config::kSCurveRawMinError);
    bool sharp_raw_error = std::fabs(control_raw_steer_error) > car_config::kSCurveSharpRawThreshold;
    bool s_curve_prepare = raw_filter_gap || raw_filter_sign_conflict ||
                           early_sign_cross || fast_steer_change || sharp_raw_error;
    if (s_curve_prepare) {
        s_curve_hold_frames = car_config::kSCurveHoldFrames;
    } else if (s_curve_hold_frames > 0) {
        --s_curve_hold_frames;
    }
    bool s_curve_active = s_curve_prepare || (s_curve_hold_frames > 0);
    bool extreme_turn_active =
        (std::fabs(raw_steer_error) > car_config::kExtremeTurnRawThreshold) ||
        (std::fabs(steer_error) > car_config::kExtremeTurnFiltThreshold);
    const int single_side_loss = std::max(ImageStatus.Left_Line, ImageStatus.Right_Line);
    const int side_loss_delta =
        static_cast<int>(ImageStatus.Left_Line) -
        static_cast<int>(ImageStatus.Right_Line);
    static int u_turn_hold_frames = 0;
    static int u_turn_forced_dir = 0; // +1: left U-turn, -1: right U-turn.
    if (full_loss_non_ring || ring_just_finished) {
        u_turn_hold_frames = 0;
        u_turn_forced_dir = 0;
    }
    const bool u_turn_raw_filt_conflict =
        (std::fabs(control_raw_steer_error) >=
             car_config::kUTurnRawFiltConflictRawThreshold) &&
        (std::fabs(steer_error) >=
             car_config::kUTurnRawFiltConflictFiltThreshold) &&
        (control_raw_steer_error * steer_error < 0.0f);
    if (u_turn_raw_filt_conflict && u_turn_forced_dir == 0) {
        u_turn_hold_frames = 0;
    }
    bool u_turn_prepare =
        !ring_now &&
        (ImageStatus.Road_type == Normol || ImageStatus.Road_type == Straight) &&
        !u_turn_raw_filt_conflict &&
        single_side_loss >= car_config::kUTurnSingleSideLossThreshold &&
        (std::fabs(raw_steer_error) >= car_config::kUTurnRawThreshold ||
         std::fabs(steer_error) >= car_config::kUTurnRawThreshold);
    const bool u_turn_exit_centered =
        !u_turn_prepare &&
        u_turn_forced_dir != 0 &&
        single_side_loss <= car_config::kUTurnReleaseSingleSideLossMax &&
        std::fabs(control_raw_steer_error) <= car_config::kUTurnReleaseRawThreshold &&
        std::fabs(steer_error) <= car_config::kUTurnReleaseFiltThreshold;
    if (u_turn_exit_centered) {
        u_turn_hold_frames = 0;
        u_turn_forced_dir = 0;
    } else if (u_turn_prepare) {
        u_turn_hold_frames = car_config::kUTurnHoldFrames;
        if (side_loss_delta >= car_config::kUTurnSideLossDominance) {
            u_turn_forced_dir = 1;
        } else if (side_loss_delta <= -car_config::kUTurnSideLossDominance) {
            u_turn_forced_dir = -1;
        } else if (u_turn_forced_dir == 0 && std::fabs(control_raw_steer_error) > 0.1f) {
            u_turn_forced_dir = (control_raw_steer_error > 0.0f) ? 1 : -1;
        }
    } else if (u_turn_hold_frames > 0) {
        --u_turn_hold_frames;
    } else {
        u_turn_forced_dir = 0;
    }
    bool u_turn_risk = u_turn_prepare || (u_turn_hold_frames > 0);
    bool curve_entry_brake_prepare =
        !ring_now &&
        (ImageStatus.Road_type == Normol || ImageStatus.Road_type == Straight) &&
        !s_curve_active &&
        !extreme_turn_active &&
        Speed_Goal > car_config::kCurveEntryBrakeSpeedGoal &&
        (std::fabs(control_raw_steer_error) >= car_config::kCurveEntryBrakeRawThreshold) &&
        (std::fabs(steer_error) >= car_config::kCurveEntryBrakeFiltThreshold) &&
        (control_raw_steer_error * diff_steer >= 0.0f) &&
        (std::fabs(diff_steer) >= car_config::kCurveEntryBrakeDerivativeThreshold);
    if (curve_entry_brake_prepare) {
        curve_entry_brake_frames = car_config::kCurveEntryBrakeHoldFrames;
    } else if (curve_entry_brake_frames > 0) {
        --curve_entry_brake_frames;
    }

    // 弯道大偏角强制降速保护
    if (abs(steer_error) > car_config::kSharpTurnSteerError) {
        Speed_Goal = WD_Speed_Goal; // 弯道强制降速
    }
    if (Speed_Goal > car_config::kCurveEntryBrakeSpeedGoal &&
        curve_entry_brake_frames > 0) {
        Speed_Goal = car_config::kCurveEntryBrakeSpeedGoal;
    }
    if (Speed_Goal > car_config::kSCurveSpeedGoal && s_curve_active) {
        Speed_Goal = car_config::kSCurveSpeedGoal;
    }
    if (Speed_Goal > car_config::kExtremeTurnSpeedGoal && extreme_turn_active) {
        Speed_Goal = car_config::kExtremeTurnSpeedGoal;
    }
    if (Speed_Goal > car_config::kUTurnSpeedGoal && u_turn_risk) {
        Speed_Goal = car_config::kUTurnSpeedGoal;
    }
    if (ring_now && Speed_Goal > car_config::kRingSpeedGoal) {
        Speed_Goal = car_config::kRingSpeedGoal;
    }
    if (bad_vision && ImageFlag.image_element_rings == 0 &&
        Speed_Goal > car_config::kBadVisionSpeedGoal) {
        Speed_Goal = car_config::kBadVisionSpeedGoal;
    }
    last_effective_speed_goal = Speed_Goal;

    Mpu6050Data imu_data;
    if(imu.read_processed_data(imu_data)) {
        static float last_valid_gyro_z = 0.0f;
        if (std::fabs(imu_data.gyro_z) > car_config::kGyroRawSpikeRejectDps) {
            imu_data.gyro_z = last_valid_gyro_z;
        } else {
            last_valid_gyro_z = imu_data.gyro_z;
        }

        float target_gyro_z = 0.0f;
        float visual_pd_out = 0.0f;
        float delta_target = 0.0f;
        static int opposing_gyro_recovery_frames = 0;
        dbg_imu_ok = 1;
        dbg_real_gyro_z = imu_data.gyro_z;
        const float filtered_gyro_before_update = cascade_ctrl.last_filtered_gyro_z();
	        
        if (abs(steer_error) < DEAD_BAND) {
            const bool straight_gyro_damping =
                !ring_now &&
                ImageFlag.image_element_rings == 0 &&
                (ImageStatus.Road_type == Normol ||
                 ImageStatus.Road_type == Straight ||
                 ImageStatus.Road_type == Cross) &&
                std::fabs(control_raw_steer_error) <= car_config::kSteerDeadBandStraight &&
                std::fabs(steer_error) <= car_config::kSteerDeadBandStraight &&
                std::fabs(filtered_gyro_before_update) >=
                    car_config::kStraightGyroDampingThresholdDps;
            if (straight_gyro_damping) {
                target_gyro_z =
                    -filtered_gyro_before_update * car_config::kStraightGyroDampingGain;
                if (target_gyro_z > car_config::kStraightGyroDampingLimitDps) {
                    target_gyro_z = car_config::kStraightGyroDampingLimitDps;
                }
                if (target_gyro_z < -car_config::kStraightGyroDampingLimitDps) {
                    target_gyro_z = -car_config::kStraightGyroDampingLimitDps;
                }
                delta_target = target_gyro_z - last_target_gyro_z;
                if (delta_target > car_config::kStraightGyroDampingMaxDeltaDps) {
                    delta_target = car_config::kStraightGyroDampingMaxDeltaDps;
                }
                if (delta_target < -car_config::kStraightGyroDampingMaxDeltaDps) {
                    delta_target = -car_config::kStraightGyroDampingMaxDeltaDps;
                }
                target_gyro_z = last_target_gyro_z + delta_target;
            } else {
                // 死区内直行时也要把旧方向的角速度目标收回，否则 S 弯换向会慢半拍。
                delta_target = -last_target_gyro_z;
                if (delta_target > car_config::kTargetGyroDeadbandReturnDeltaDps) {
                    delta_target = car_config::kTargetGyroDeadbandReturnDeltaDps;
                }
                if (delta_target < -car_config::kTargetGyroDeadbandReturnDeltaDps) {
                    delta_target = -car_config::kTargetGyroDeadbandReturnDeltaDps;
                }
                target_gyro_z = last_target_gyro_z + delta_target;
            }
            if (std::fabs(target_gyro_z) < car_config::kGyroDeadBandDps) {
                target_gyro_z = 0.0f;
                delta_target = -last_target_gyro_z;
                opposing_gyro_recovery_frames = 0;
            } else if (opposing_gyro_recovery_frames > 0) {
                --opposing_gyro_recovery_frames;
            }
            last_target_gyro_z = target_gyro_z;
            last_steer_error = steer_error;
        } else {
            // 【核心】使用 motor.cpp 里跑得完美的PD逻辑，算出平滑无噪点的转角力道
            visual_pd_out = steer_error * STEER_KP + diff_steer * STEER_KD;
            if (u_turn_raw_filt_conflict) {
                visual_pd_out = control_raw_steer_error * STEER_KP +
                                diff_steer * STEER_KD;
            }

            // 外环硬限幅，避免单帧视觉跳变把角速度目标拉爆
            if (visual_pd_out > car_config::kVisualGyroLimitDps) visual_pd_out = car_config::kVisualGyroLimitDps;
            if (visual_pd_out < -car_config::kVisualGyroLimitDps) visual_pd_out = -car_config::kVisualGyroLimitDps;

            if ((ImageStatus.Road_type == LeftCirque || ImageStatus.Road_type == RightCirque) &&
                std::fabs(visual_pd_out) > car_config::kRingSceneGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kRingSceneGyroLimitDps, visual_pd_out);
            }
            if (ring_entry_phase &&
                std::fabs(visual_pd_out) > car_config::kRingEntryGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kRingEntryGyroLimitDps, visual_pd_out);
            }

            const bool unclassified_extreme_turn =
                (extreme_turn_active ||
                 std::fabs(control_raw_steer_error) >= car_config::kUnclassifiedTurnTriggerError ||
                 std::fabs(steer_error) >= car_config::kUnclassifiedTurnTriggerError) &&
                ImageFlag.image_element_rings == 0 &&
                (ImageStatus.Road_type == Normol || ImageStatus.Road_type == Straight);
            if (!u_turn_risk &&
                unclassified_extreme_turn &&
                std::fabs(visual_pd_out) > car_config::kUnclassifiedTurnGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kUnclassifiedTurnGyroLimitDps,
                                              visual_pd_out);
            }
            if (u_turn_risk &&
                std::fabs(visual_pd_out) > car_config::kUTurnGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kUTurnGyroLimitDps,
                                              visual_pd_out);
            }
            if (u_turn_risk && u_turn_forced_dir != 0) {
                float forced_abs = std::fabs(visual_pd_out);
                if (forced_abs < car_config::kUTurnForcedGyroDps) {
                    forced_abs = car_config::kUTurnForcedGyroDps;
                }
                if (forced_abs > car_config::kUTurnGyroLimitDps) {
                    forced_abs = car_config::kUTurnGyroLimitDps;
                }
                visual_pd_out = std::copysign(forced_abs,
                                              static_cast<float>(u_turn_forced_dir));
            } else if (u_turn_risk &&
                !u_turn_raw_filt_conflict &&
                std::fabs(visual_pd_out) < car_config::kUTurnForcedGyroDps) {
                const float u_turn_sign_source =
                    (std::fabs(visual_pd_out) > 0.1f) ? visual_pd_out :
                    ((std::fabs(steer_error) > 0.1f) ? steer_error :
                     control_raw_steer_error);
                if (std::fabs(u_turn_sign_source) > 0.1f) {
                    visual_pd_out = std::copysign(car_config::kUTurnForcedGyroDps,
                                                  u_turn_sign_source);
                }
            }
            const bool near_straight_centered =
                !ring_now &&
                (ImageStatus.Road_type == Straight || ImageStatus.Road_type == Normol) &&
                std::fabs(control_raw_steer_error) <= car_config::kSteerDeadBandStraight &&
                std::fabs(steer_error) <= car_config::kSteerDeadBandStraight;
            if (near_straight_centered &&
                std::fabs(control_raw_steer_error) <= car_config::kSteerDeadBandStraight &&
                std::fabs(steer_error) <= car_config::kSteerDeadBandStraight &&
                std::fabs(visual_pd_out) > car_config::kStraightSmallErrorGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kStraightSmallErrorGyroLimitDps,
                                              visual_pd_out);
            }
            if (bad_vision && !ring_now && !u_turn_risk &&
                std::fabs(visual_pd_out) > car_config::kBadVisionGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kBadVisionGyroLimitDps,
                                              visual_pd_out);
            }

            // 对目标角速度做变化率限制，抑制S弯切换时的目标突刺
            float max_delta_target = car_config::kTargetGyroMaxDeltaDps; // 20ms外环周期下，每次最多变化10deg/s
            if (s_curve_active) {
                max_delta_target = car_config::kTargetGyroSCurveMaxDeltaDps;
            }
            if ((visual_pd_out > 0.0f && last_target_gyro_z < 0.0f) ||
                (visual_pd_out < 0.0f && last_target_gyro_z > 0.0f)) {
                max_delta_target = car_config::kTargetGyroSignFlipMaxDeltaDps;
            }
            if (extreme_exit_unwind &&
                max_delta_target < car_config::kExtremeTurnExitTargetDeltaDps) {
                max_delta_target = car_config::kExtremeTurnExitTargetDeltaDps;
            }
            delta_target = visual_pd_out - last_target_gyro_z;
            if (delta_target > max_delta_target) delta_target = max_delta_target;
            if (delta_target < -max_delta_target) delta_target = -max_delta_target;
            target_gyro_z = last_target_gyro_z + delta_target;
            bool opposing_gyro_limited = false;
            if (s_curve_active &&
                target_gyro_z * filtered_gyro_before_update < 0.0f &&
                std::fabs(filtered_gyro_before_update) > car_config::kOpposingGyroHoldDps &&
                std::fabs(target_gyro_z) > car_config::kOpposingGyroTargetLimitDps) {
                target_gyro_z = std::copysign(car_config::kOpposingGyroTargetLimitDps, target_gyro_z);
                delta_target = target_gyro_z - last_target_gyro_z;
                opposing_gyro_recovery_frames = car_config::kOpposingGyroRecoveryFrames;
                opposing_gyro_limited = true;
            }
            if (!opposing_gyro_limited && s_curve_active && opposing_gyro_recovery_frames > 0) {
                if (std::fabs(target_gyro_z) > car_config::kOpposingGyroRecoveryLimitDps) {
                    target_gyro_z = std::copysign(car_config::kOpposingGyroRecoveryLimitDps, target_gyro_z);
                    delta_target = target_gyro_z - last_target_gyro_z;
                }
                --opposing_gyro_recovery_frames;
            } else if (!s_curve_active) {
                opposing_gyro_recovery_frames = 0;
            }
            last_target_gyro_z = target_gyro_z;
            last_steer_error = steer_error;
        }
        dbg_visual_pd_out = visual_pd_out;
        dbg_target_delta = delta_target;

        // 调用基于 MPU6050 的角速度闭环（内环）
        // 方向补偿：用于修正正反向角速度响应不对称
        float cmd_target_gyro_z = target_gyro_z;
        if (cmd_target_gyro_z > 0.0f) {
            cmd_target_gyro_z *= car_config::kGyroTargetPositiveGain;
        } else if (cmd_target_gyro_z < 0.0f) {
            cmd_target_gyro_z *= car_config::kGyroTargetNegativeGain;
        }

        float diff_speed = cascade_ctrl.calc_diff_speed_bypass_angle(cmd_target_gyro_z, imu_data, dt);
        dbg_cmd_target_gyro_z = cmd_target_gyro_z;
        dbg_filtered_gyro_z = cascade_ctrl.last_filtered_gyro_z();
        dbg_gyro_error = cascade_ctrl.gyro_pid.err;
        dbg_diff_before_limit = diff_speed;

        // 【关键修复】恢复 motor.cpp 中的最大差速限幅，防止内环输出的差速过大导致转弯太早/过猛内切
        float max_diff = Speed_Goal * MAX_DIFF_RATIO;
        if (s_curve_active && max_diff < car_config::kSCurveMinDiff) {
            max_diff = car_config::kSCurveMinDiff;
        }
        if (extreme_turn_active && max_diff < car_config::kExtremeTurnMinDiff) {
            max_diff = car_config::kExtremeTurnMinDiff;
        }
        if (u_turn_risk && max_diff < car_config::kUTurnMinDiff) {
            max_diff = car_config::kUTurnMinDiff;
        }
        if (ring_now) {
            float ring_max_diff = ring_entry_phase ? car_config::kRingEntryMaxDiff : car_config::kRingMaxDiff;
            if (max_diff > ring_max_diff) {
                max_diff = ring_max_diff;
            }
        }
        if (bad_vision && !ring_now && !u_turn_risk && max_diff > car_config::kBadVisionMaxDiff) {
            max_diff = car_config::kBadVisionMaxDiff;
        }
        const bool normol_moderate_error =
            !ring_now &&
            !bad_vision &&
            ImageStatus.Road_type == Normol &&
            std::fabs(control_raw_steer_error) <= car_config::kNormolModerateErrorThreshold &&
            std::fabs(steer_error) <= car_config::kNormolModerateErrorThreshold;
        if (normol_moderate_error &&
            max_diff > car_config::kNormolModerateErrorMaxDiff) {
            max_diff = car_config::kNormolModerateErrorMaxDiff;
        }
        const bool near_straight_centered_diff =
            !ring_now &&
            (ImageStatus.Road_type == Straight || ImageStatus.Road_type == Normol) &&
            std::fabs(control_raw_steer_error) <= car_config::kSteerDeadBandStraight &&
            std::fabs(steer_error) <= car_config::kSteerDeadBandStraight;
        const bool near_straight_swing =
            !ring_now &&
            ImageStatus.Road_type == Straight &&
            std::fabs(cascade_ctrl.last_filtered_gyro_z()) >= car_config::kStraightHighGyroLimitDps;
        if (near_straight_centered_diff || near_straight_swing) {
            const float straight_limit = near_straight_centered_diff ?
                car_config::kStraightSmallErrorMaxDiff : car_config::kStraightHighGyroDiffLimit;
            if (max_diff > straight_limit) {
                max_diff = straight_limit;
            }
        }
        dbg_max_diff = max_diff;
        dbg_diff_limited = 0;
        if (diff_speed > max_diff)  {
            diff_speed = max_diff;
            dbg_diff_limited = 1;
        }
        if (diff_speed < -max_diff) {
            diff_speed = -max_diff;
            dbg_diff_limited = 1;
        }

        dbg_target_gyro_z = target_gyro_z;
        dbg_diff_speed = diff_speed;

        // 记录在静态变量给频率降频（跳帧）时复用
        last_diff_speed = diff_speed;

        // 4: 分配给【最内层：轮速闭环】
        Speed_Goal_l = Speed_Goal - diff_speed;
        Speed_Goal_r = Speed_Goal + diff_speed;

        // 5: 速度限幅与防失速保护
        if (Speed_Goal <= 0) {
            Speed_Goal_l = 0;
            Speed_Goal_r = 0;
        } else {
            const float min_wheel_goal = u_turn_risk ?
                car_config::kUTurnMinWheelSpeedGoal : MIN_SPEED;
            // U型弯允许内轮接近停转；普通弯仍保留基础防失速保护。
            if (Speed_Goal_l < min_wheel_goal) Speed_Goal_l = min_wheel_goal;
            if (Speed_Goal_r < min_wheel_goal) Speed_Goal_r = min_wheel_goal;
            if (Speed_Goal_l > car_config::kMaxWheelSpeedGoal) Speed_Goal_l = car_config::kMaxWheelSpeedGoal;
            if (Speed_Goal_r > car_config::kMaxWheelSpeedGoal) Speed_Goal_r = car_config::kMaxWheelSpeedGoal;
        }
        dbg_speed_err_l = Speed_Goal_l - speed_encoder_L;
        dbg_speed_err_r = Speed_Goal_r - speed_encoder_R;
    } else {
        // 如果 IMU 断联，无缝降级回原汁原味的 motor.cpp 逻辑
        dbg_imu_ok = 0;
        dbg_real_gyro_z = 0.0f;
        dbg_filtered_gyro_z = 0.0f;
        dbg_gyro_error = 0.0f;
        dbg_cmd_target_gyro_z = 0.0f;
        dbg_target_delta = 0.0f;
        if (abs(steer_error) < DEAD_BAND) {
            last_diff_speed = 0.0f;
            dbg_target_gyro_z = 0.0f;
            dbg_diff_speed = 0.0f;
            dbg_visual_pd_out = 0.0f;
            dbg_diff_before_limit = 0.0f;
            dbg_max_diff = Speed_Goal * MAX_DIFF_RATIO;
            dbg_diff_limited = 0;
            Speed_Goal_l = Speed_Goal;
            Speed_Goal_r = Speed_Goal;
            last_steer_error = steer_error;
        } else {
            float fallback_diff = steer_error * STEER_KP + diff_steer * STEER_KD;
            dbg_visual_pd_out = fallback_diff;
            dbg_diff_before_limit = fallback_diff;

            if (fallback_diff > car_config::kFallbackDiffLimit) fallback_diff = car_config::kFallbackDiffLimit;
            if (fallback_diff < -car_config::kFallbackDiffLimit) fallback_diff = -car_config::kFallbackDiffLimit;

            dbg_target_gyro_z = 0.0f;

            last_steer_error = steer_error;
            
            float max_diff = Speed_Goal * MAX_DIFF_RATIO;
            if (s_curve_active && max_diff < car_config::kSCurveMinDiff) {
                max_diff = car_config::kSCurveMinDiff;
            }
            if (extreme_turn_active && max_diff < car_config::kExtremeTurnMinDiff) {
                max_diff = car_config::kExtremeTurnMinDiff;
            }
            if (ring_now) {
                float ring_max_diff = ring_entry_phase ? car_config::kRingEntryMaxDiff : car_config::kRingMaxDiff;
                if (max_diff > ring_max_diff) {
                    max_diff = ring_max_diff;
                }
            }
            dbg_max_diff = max_diff;
            dbg_diff_limited = 0;
            if (fallback_diff > max_diff)  {
                fallback_diff = max_diff;
                dbg_diff_limited = 1;
            }
            if (fallback_diff < -max_diff) {
                fallback_diff = -max_diff;
                dbg_diff_limited = 1;
            }
            
            // 兼容保存差速变量。注意fallback时的正负号约定。
            last_diff_speed = -fallback_diff;
            dbg_diff_speed = last_diff_speed;

            Speed_Goal_l = Speed_Goal + fallback_diff;
            Speed_Goal_r = Speed_Goal - fallback_diff;
            
            if (Speed_Goal_l < MIN_SPEED) Speed_Goal_l = MIN_SPEED;
            if (Speed_Goal_r < MIN_SPEED) Speed_Goal_r = MIN_SPEED;
        }
        dbg_speed_err_l = Speed_Goal_l - speed_encoder_L;
        dbg_speed_err_r = Speed_Goal_r - speed_encoder_R;
    }

    if (car_config::kSteerDebugPrintEnable) {
        static int debug_print_count = 0;
        debug_print_count++;
        if (debug_print_count >= car_config::kSteerDebugPrintDivider) {
            debug_print_count = 0;
            printf("[steer] det:%d mid:%d raw:%.1f filt:%.1f dE:%.1f pd:%.1f tgt:%.1f cmd:%.1f gz:%.1f gf:%.1f ge:%.1f dT:%.1f d0:%.1f diff:%.1f cap:%d max:%.1f goal:%.1f L:%.1f R:%.1f encL:%.1f encR:%.1f eL:%.1f eR:%.1f dt:%.3f imu:%d db:%d off:%d ll:%d rl:%d ut:%d ud:%d\n",
                   ImageStatus.Det_True,
                   ImageStatus.MiddleLine,
                   raw_steer_error,
                   dbg_steer_error,
                   dbg_diff_steer,
                   dbg_visual_pd_out,
                   dbg_target_gyro_z,
                   dbg_cmd_target_gyro_z,
                   dbg_real_gyro_z,
                   dbg_filtered_gyro_z,
                   dbg_gyro_error,
                   dbg_target_delta,
                   dbg_diff_before_limit,
                   dbg_diff_speed,
                   dbg_diff_limited,
                   dbg_max_diff,
                   Speed_Goal,
                   Speed_Goal_l,
                   Speed_Goal_r,
                   speed_encoder_L,
                   speed_encoder_R,
                   dbg_speed_err_l,
                   dbg_speed_err_r,
                   dbg_dt,
                   dbg_imu_ok,
                   dbg_deadband_active,
                   ImageStatus.OFFLine,
                   ImageStatus.Left_Line,
                   ImageStatus.Right_Line,
                   u_turn_risk ? 1 : 0,
                   u_turn_forced_dir);
        }
    }
}



// ===============================================================================
// [6] 外设与状态监控
// ===============================================================================

//编码器记录路程
void Car_distance(void)
{
    //记录保护编码器
    Record_encoder=(speed_encoder_L+speed_encoder_R)/2;
    SystemData.SpeedData.Length+=Record_encoder;

    if(SystemData.Stop==2)
    {
        ImageStatus.Stop_lenth+=Record_encoder;
    }
    else
        ImageStatus.Stop_lenth=0;




}

// ===============================================================================
// [7] 主控与调度函数
// ===============================================================================

//变速控制
void Car_Speed_Contrl(void)
{


    //直线加速
    if(ImageStatus.straight_acc==1)
    {
        Speed_Goal=ZD_Speed_Goal;
        // beep.SetGpioValue(1);
    }
    else
    {
        Speed_Goal=WD_Speed_Goal;
        // beep.SetGpioValue(0);
    }




    
    if(SystemData.Stop==1)  //停车（不再因为车库/斑马线识别停车）
    {
        Speed_Goal=0;
        Speed_Goal_l=0;
        Speed_Goal_r=0;
    }

}



void Speed_BiHuan(void)
{
    // encoder_num() 返回的已经是当前的瞬时速度（引脚脉冲测频），不需要再做差分差值计算！
    speed_encoder_raw_L = encoder_num_L();
    speed_encoder_raw_R = encoder_num_R();
    speed_encoder_L = speed_encoder_raw_L;
    speed_encoder_R = speed_encoder_raw_R;

    Car_distance();         // 编码器记录路程
    Car_Speed_Contrl();     // 变速控制，更新基础速度Speed_Goal

    static float ramped_speed_goal = 0.0f;
    static auto last_speed_ramp_time = std::chrono::steady_clock::now();
    auto speed_ramp_now = std::chrono::steady_clock::now();
    std::chrono::duration<float> speed_ramp_elapsed = speed_ramp_now - last_speed_ramp_time;
    float speed_ramp_dt = speed_ramp_elapsed.count();
    last_speed_ramp_time = speed_ramp_now;
    if (speed_ramp_dt <= 0.0001f || speed_ramp_dt > 0.05f) {
        speed_ramp_dt = 0.005f;
    }

    const float desired_speed_goal = Speed_Goal;
    if (desired_speed_goal <= 0.0f) {
        ramped_speed_goal = 0.0f;
        Speed_Goal = 0.0f;
    } else {
        const float speed_goal_delta = desired_speed_goal - ramped_speed_goal;
        const float speed_goal_limit =
            (speed_goal_delta > 0.0f) ?
            car_config::kSpeedTargetAccelLimit :
            car_config::kSpeedTargetDecelLimit;
        const float speed_goal_max_step = speed_goal_limit * speed_ramp_dt;
        if (speed_goal_delta > speed_goal_max_step) {
            ramped_speed_goal += speed_goal_max_step;
        } else if (speed_goal_delta < -speed_goal_max_step) {
            ramped_speed_goal -= speed_goal_max_step;
        } else {
            ramped_speed_goal = desired_speed_goal;
        }
        if (ramped_speed_goal < 0.0f) ramped_speed_goal = 0.0f;
        Speed_Goal = ramped_speed_goal;
    }

    // 【替换为】：基于陀螺仪的角度/角速度/速度的三串级差速控制
    speed_dif_cascade();     

    // 外环弯道/丢线保护可能在 speed_dif_cascade() 内把目标速度直接压低。
    // 同步斜坡状态，避免保护解除时从旧的高速记忆值瞬间跳回去。
    if (Speed_Goal < ramped_speed_goal) {
        ramped_speed_goal = Speed_Goal;
    }

    const bool reverse_feedback_l =
        (Speed_Goal_l > 0.20f) && (speed_encoder_L < -0.60f);
    const bool reverse_feedback_r =
        (Speed_Goal_r > 0.20f) && (speed_encoder_R < -0.60f);
    if (reverse_feedback_l || reverse_feedback_r) {
        static int speed_guard_warn_count = 0;
        const float guard_goal_l = Speed_Goal_l;
        const float guard_goal_r = Speed_Goal_r;
        Speed_Goal = 0.0f;
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        ramped_speed_goal = 0.0f;
        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_last_erro_R = 0.0f;
        if (motor_L && motor_R) {
            motor_Duty_L(0);
            motor_Duty_R(0);
        }
        if (speed_guard_warn_count < 8) {
            ++speed_guard_warn_count;
            printf("[SPEED_GUARD] reverse encoder feedback, stop output. "
                   "goalL:%.2f goalR:%.2f encL:%.2f encR:%.2f\n",
                   guard_goal_l,
                   guard_goal_r,
                   speed_encoder_L,
                   speed_encoder_R);
        }
        return;
    }

    Speed_PID_L();           // 【最内环】：左右轮速度环PI闭环运算
    Speed_PID_R();           // 【最内环】：左右轮速度环PI闭环运算
    
    motor_Duty_R(speed_out_r); // 输出到PWM驱动
    motor_Duty_L(speed_out_l);
}

// 差速速度调整函数
// void Speed_Adjust_By_Offset(float offset)
// {
//     float KP = 0.3f;  // 方向调整比例系数，可调试

//     float turn = KP * offset;  // 转向量

//     int base_speed = Speed_Goal;  // 当前目标速度，三轮车车型最大速度设定

//     // 左右轮速度由基准速度和偏差计算
//     Speed_Goal_l = base_speed - turn;
//     Speed_Goal_r = base_speed + turn;

//     // 限幅保护，防止速度超过范围或为负
//     if (Speed_Goal_l > MAX_SPEED) Speed_Goal_l = MAX_SPEED;
//     if (Speed_Goal_l < 0)         Speed_Goal_l = 0;

//     if (Speed_Goal_r > MAX_SPEED) Speed_Goal_r = MAX_SPEED;
//     if (Speed_Goal_r < 0)         Speed_Goal_r = 0;
// }

// 电机紧急停止函数
void emergency_stop() {
    Speed_Goal = 0.0f;
    Speed_Goal_l = 0.0f;
    Speed_Goal_r = 0.0f;
    speed_out_r = 0.0f;
    speed_out_l = 0.0f;
    speed_last_erro_R = 0.0f;
    speed_last_erro_L = 0.0f;

    if (motor_R && motor_L) {
        motor_Duty_R(0);    // 右电机停转
        motor_Duty_L(0);    // 左电机停转
    }
    if (motor_R_Dir) motor_R_Dir->gpio_level_set(GPIO_LOW);
    if (motor_L_Dir) motor_L_Dir->gpio_level_set(GPIO_LOW);

    if (motor_R && motor_L) {
        if (car_config::kMotorStopKeepPwmEnabled) {
            motor_R->atim_pwm_enable();
            motor_L->atim_pwm_enable();
            motor_R->atim_pwm_set_duty(0);
            motor_L->atim_pwm_set_duty(0);
        } else {
            motor_R->atim_pwm_disable();
            motor_L->atim_pwm_disable();
        }
    }
    // stree_M.SetDutyCycle(170000);  // 舵机归中
    printf("程序终止，电机已停止。\n");
}

// 信号处理函数（捕获 Ctrl+C 等终止信号）
void signal_handler(int signum) {
    exit_flag = true;    // 触发子线程退出
    emergency_stop();    // 立即停止电机
    gpio_protect_heartbeat_deinit();
    exit(signum);        // 终止进程
}
