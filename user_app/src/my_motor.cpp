#include "main.hpp"
#include "my_cascade_pid.hpp"
#include "car_config.hpp"
#include "lq_udp_client.hpp"
#include <algorithm>
#include <cstdio>
#include <mutex>
#include <thread>
#include <sched.h>
#include <cstring>

static inline float clamp_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}



struct VisionSnapshot {
    int MiddleLine;
    int Det_True;
    int OFFLine;
    int Road_type;
    int image_element_rings;
    int image_element_rings_flag;
    int Left_Line;
    int Right_Line;
    int straight_acc;
    int TowCenter;
    int TowWidth;
    int TowLeftFound;
    int TowRightFound;
    int NearCenter;
    int NearLeftFound;
    int NearRightFound;
    uint32_t seq;
};

static VisionSnapshot g_vision_snapshot;
static std::mutex g_vision_mutex;

#include "gpio_protect_heartbeat.hpp"
#include <chrono>
#include <cmath>

// ===============================================================================
// [1] Global state and hardware objects
// ===============================================================================

// Global exit flags.
std::atomic<bool> g_program_exit(false);
std::atomic<bool> exit_flag(false);

// Motor PWM and direction GPIO objects. They are constructed lazily so global
// initialization never touches /dev/mem before main() is ready.
ls_atim_pwm* motor_L = nullptr;
ls_atim_pwm* motor_R = nullptr;
ls_gpio* motor_L_Dir = nullptr;
ls_gpio* motor_R_Dir = nullptr;

// Runtime speed target.
float Speed_Goal = 0.0f;
// Basic three-stage cascade: visual PID -> gyro PID -> wheel-speed PID.
// ========================
// Debug telemetry state
// ========================
float dbg_steer_error = 0.0f;
float dbg_target_gyro_z = 0.0f;
float dbg_diff_speed = 0.0f;          // 角速度环原始输出（限幅前）
float dbg_diff_cmd = 0.0f;            // 动态限幅后实际执行差速
float dbg_diff_limit = 0.0f;          // 当前动态差速限幅值
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
float dbg_cross_heading_error_px = 0.0f;
float dbg_cross_align_dps = 0.0f;
int dbg_cross_align_active = 0;
float dbg_near_center_error_px = 0.0f;
float dbg_near_center_add_dps = 0.0f;
float dbg_near_control_error_px = 0.0f;
float dbg_near_far_scale = 1.0f;
int dbg_near_far_ambiguous = 0;
int dbg_near_far_release_reason = 0;

// 姿�?航向 PID串级控制对象与IMU
AttitudeCascade cascade_ctrl;
MyMpu6050 imu;

void control_set_imu_async_cache_only(bool enabled)
{
    imu.set_async_cache_only(enabled);
}

void imu_thread_1(void)
{
    // 提升IMU线程为SCHED_FIFO实时优先级，防止摄像头帧处理/图像算法
    // 抢占I2C读取调度，导致弯道中IMU数据stale级联断连。
    {
        struct sched_param sp;
        std::memset(&sp, 0, sizeof(sp));
        sp.sched_priority = 10;  // 低实时优先级，足够抢占SCHED_OTHER但不阻塞更高优先级内核线程
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
            perror("[IMU_THREAD] SCHED_FIFO failed, using default scheduler");
        }
    }
    const auto imu_interval = std::chrono::milliseconds(10);
    while (!exit_flag.load(std::memory_order_relaxed)) {
        const auto tick_start = std::chrono::steady_clock::now();
        imu.update_async_sample();
        const auto elapsed = std::chrono::steady_clock::now() - tick_start;
        if (elapsed < imu_interval) {
            std::this_thread::sleep_for(imu_interval - elapsed);
        }
    }
}

static constexpr uint32_t kVisionFreshTimeoutMs = 150;      // 视觉帧超时阈值：150ms容忍偶发卡顿
static constexpr uint32_t kVisionStaleTurnHoldMs = 280;     // 急弯中短暂卡帧时先保持转向，避免被拉直
static constexpr uint32_t kVisionFatalTimeoutMs = 500;      // 视觉帧致命超时：500ms无帧才退出
static constexpr uint32_t kVisionStaleWarnIntervalMs = 150; // 超时降速期间每150ms打印一次警告
static constexpr float kVisionStaleSpeedGoal = 0.12f;       // 短暂卡帧时低速直行等待新视觉
static constexpr float kVisionRecoverySpeedGoal = 0.18f;    // 丢线恢复/直角弯保持转向时的低速目标
static constexpr float kTurnRecoveryMinWheel = 0.03f;       // 直角弯允许慢轮接近停转以缩小转弯半径
static constexpr float kGuardedTurnReleaseRatio = 0.30f;
static std::atomic<uint32_t> g_control_vision_frame_seq(0);
static std::atomic<long long> g_control_last_vision_frame_ms(0);

static long long control_now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void control_mark_vision_frame_ready(void)
{
    int tow_point = ImageStatus.TowPoint;
    if (tow_point < 0) tow_point = 0;
    if (tow_point > 59) tow_point = 59;
    int near_row = car_config::kCrossEntryHeadingNearRow;
    if (near_row < 0) near_row = 0;
    if (near_row > 59) near_row = 59;
    {
        std::lock_guard<std::mutex> lock(g_vision_mutex);
        g_vision_snapshot.MiddleLine = ImageStatus.MiddleLine;
        g_vision_snapshot.Det_True = ImageStatus.Det_True;
        g_vision_snapshot.OFFLine = ImageStatus.OFFLine;
        g_vision_snapshot.Road_type = ImageStatus.Road_type;
        g_vision_snapshot.image_element_rings = ImageFlag.image_element_rings;
        g_vision_snapshot.image_element_rings_flag = ImageFlag.image_element_rings_flag;
        g_vision_snapshot.Left_Line = ImageStatus.Left_Line;
        g_vision_snapshot.Right_Line = ImageStatus.Right_Line;
        g_vision_snapshot.straight_acc = ImageStatus.straight_acc;
        g_vision_snapshot.TowCenter = ImageDeal[tow_point].Center;
        g_vision_snapshot.TowWidth =
            ImageDeal[tow_point].RightBorder - ImageDeal[tow_point].LeftBorder;
        g_vision_snapshot.TowLeftFound =
            (ImageDeal[tow_point].IsLeftFind == 'T' || ImageDeal[tow_point].IsLeftFind == 'H') ? 1 : 0;
        g_vision_snapshot.TowRightFound =
            (ImageDeal[tow_point].IsRightFind == 'T' || ImageDeal[tow_point].IsRightFind == 'H') ? 1 : 0;
        g_vision_snapshot.NearCenter = ImageDeal[near_row].Center;
        g_vision_snapshot.NearLeftFound =
            (ImageDeal[near_row].IsLeftFind == 'T' || ImageDeal[near_row].IsLeftFind == 'H') ? 1 : 0;
        g_vision_snapshot.NearRightFound =
            (ImageDeal[near_row].IsRightFind == 'T' || ImageDeal[near_row].IsRightFind == 'H') ? 1 : 0;
        g_vision_snapshot.seq = g_control_vision_frame_seq.load(std::memory_order_relaxed) + 1;
    }
    g_control_last_vision_frame_ms.store(control_now_ms(), std::memory_order_release);
    g_control_vision_frame_seq.fetch_add(1, std::memory_order_release);
}

void control_reset_vision_health(void)
{
    g_control_last_vision_frame_ms.store(0, std::memory_order_release);
    g_control_vision_frame_seq.store(0, std::memory_order_release);
}

// ===============================================================================
// [2] 闭环控制系统变量定义
// ===============================================================================

// 左轮闭环变量
float Speed_Goal_l = 0;
float speed_P_l, speed_I_l, speed_D_l;
float speed_out_l;
float speed_erro_L, speed_last_erro_L;
float speed_integral_L = 0.0f;   // 位置式积分项
float speed_last_out_l = 0.0f;    // 输出变化率限制用
float speed_encoder_L;
float speed_encoder_raw_L = 0.0f;

// 右轮闭环变量
float Speed_Goal_r = 0;
float speed_P_r, speed_I_r, speed_D_r;
float speed_out_r;
float speed_erro_R, speed_last_erro_R;
float speed_integral_R = 0.0f;   // 位置式积分项
float speed_last_out_r = 0.0f;    // 输出变化率限制用
float speed_encoder_R;
float speed_encoder_raw_R = 0.0f;

// PWM与路�?
int MAX_PWM; // PWM最大限�?
float Record_encoder = 0.0f; // 记录编码�?(m/s)
static float planned_speed_goal = 0.0f;


// ===============================================================================
// [3] 硬件初始化与底层驱动函数
// ===============================================================================

void motor_Init(void)
{
    // 延迟初始化：首次调用时才构造硬件对象，避免全局构造时访问 /dev/mem 导致崩溃
    if (!motor_L) {
        motor_L_Dir = new ls_gpio(PIN_22, GPIO_MODE_OUT);
        motor_R_Dir = new ls_gpio(PIN_21, GPIO_MODE_OUT);
        motor_L = new ls_atim_pwm(ATIM_PWM1_PIN82, car_config::kMotorPwmFrequencyHz, car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
        motor_R = new ls_atim_pwm(ATIM_PWM0_PIN81, car_config::kMotorPwmFrequencyHz, car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    }

    motor_R->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
    motor_L->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);

    printf("R:OK!");

    motor_R_Dir->gpio_level_set(GPIO_HIGH);
    motor_L_Dir->gpio_level_set(GPIO_HIGH);
    motor_R->atim_pwm_enable();
    motor_L->atim_pwm_enable();
}

void motor_Duty_R(int duty)
{
    if (!motor_R || !motor_R_Dir) return;  // 电机未初始化时安全跳过
    if (duty == 0)
    {
        motor_R->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
        motor_R_Dir->gpio_level_set(GPIO_HIGH);
    }
    else if (duty < 0)
    {
        // 反转：PH=LOW + 反转PWM占空比（对称力矩）
        motor_R->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
        const int magnitude = std::min(-duty, car_config::kMotorPwmFullScale);
        motor_R_Dir->gpio_level_set(GPIO_LOW);
        motor_R->atim_pwm_set_duty(magnitude);
    }
    else
    {
        // 正转：PH=HIGH + EN duty
        motor_R->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
        const int magnitude = std::min(duty, car_config::kMotorPwmFullScale);
        motor_R_Dir->gpio_level_set(GPIO_HIGH);
        motor_R->atim_pwm_set_duty(magnitude);
    }
}

void motor_Duty_L(int duty)
{
    if (!motor_L || !motor_L_Dir) return;  // 电机未初始化时安全跳过
    if (duty == 0)
    {
        motor_L->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
        motor_L_Dir->gpio_level_set(GPIO_HIGH);
    }
    else if (duty < 0)
    {
        // 反转：PH=LOW + 反转PWM占空比（对称力矩）
        motor_L->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
        const int magnitude = std::min(-duty, car_config::kMotorPwmFullScale);
        motor_L_Dir->gpio_level_set(GPIO_LOW);
        motor_L->atim_pwm_set_duty(magnitude);
    }
    else
    {
        // 正转：PH=HIGH + EN duty
        motor_L->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
        const int magnitude = std::min(duty, car_config::kMotorPwmFullScale);
        motor_L_Dir->gpio_level_set(GPIO_HIGH);
        motor_L->atim_pwm_set_duty(magnitude);
    }
}


// ===============================================================================
// [4] Controller initialization and PID loops
// ===============================================================================

// ==========================
// [4.1] Wheel-speed PID
// ==========================

bool Speed_decision(void)
{
    // 位置式 PID wheel-speed loop gains.
    speed_P_l = car_config::kSpeedPidP;
    speed_I_l = car_config::kSpeedPidI;
    speed_D_l = car_config::kSpeedPidD;

    speed_P_r = car_config::kSpeedPidP;
    speed_I_r = car_config::kSpeedPidI;
    speed_D_r = car_config::kSpeedPidD;

    MAX_PWM = car_config::kMotorMaxPwm;

    speed_out_l = 0;
    speed_out_r = 0;
    speed_integral_L = 0;
    speed_integral_R = 0;
    speed_last_out_l = 0;
    speed_last_out_r = 0;

    cascade_ctrl.init(); // 初始化姿态三串级控制器参�?
    static bool imu_inited = false;
    if (!imu_inited) {
        if (!imu.init()) {   // 初始化IMU
            dbg_imu_ok = 0;
            return false;
        }
        imu_inited = true;
    }
    dbg_imu_ok = 1;
    return true;
}

static void update_speed_integral(float error, float &integral)
{
    constexpr float kIntegralSignResetErr = 0.005f;
    if (error > kIntegralSignResetErr && integral < 0.0f) integral = 0.0f;
    if (error < -kIntegralSignResetErr && integral > 0.0f) integral = 0.0f;

    integral += error;
    integral = clamp_float(
        integral,
        -car_config::kSpeedPidIntegralLimit,
        car_config::kSpeedPidIntegralLimit);
}

void Speed_PID_L(void)
{
    speed_erro_L = Speed_Goal_l - speed_encoder_L;

    if (Speed_Goal_l <= 0.02f) {
        speed_out_l = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_integral_L = 0.0f;
        speed_last_out_l = 0.0f;
        return;
    }

    // 位置式 PID: output = P*err + Ki*integral + Kd*derivative
    update_speed_integral(speed_erro_L, speed_integral_L);

    const float derivative = speed_erro_L - speed_last_erro_L;
    float raw_out = speed_P_l * speed_erro_L +
                    speed_I_l * speed_integral_L +
                    speed_D_l * derivative;
    if (speed_erro_L > 0.0f) {
        raw_out += car_config::kSpeedPidFeedForwardPerMS * Speed_Goal_l;
    }

    // 输出限幅。普通超速用 EN=0 制动；只有明确弯道内轮允许有限主动反拖。
    if (raw_out > MAX_PWM) raw_out = MAX_PWM;
    if (raw_out < 0.0f) {
        const float overshoot = speed_encoder_L - Speed_Goal_l;
        const bool turn_inside_wheel =
            ImageStatus.Road_type != Cross &&
            ImageStatus.Road_type != Cross_ture &&
            Speed_Goal_l + 0.08f < Speed_Goal;
        if (turn_inside_wheel && overshoot > car_config::kSpeedBrakeThreshold) {
            if (raw_out < car_config::kSpeedTurnBrakeMaxPwm)
                raw_out = car_config::kSpeedTurnBrakeMaxPwm;
        } else {
            raw_out = car_config::kSpeedBrakeMaxPwm;
        }
    }
    float slew = raw_out - speed_last_out_l;
    if (slew > car_config::kSpeedPidSlewRate) raw_out = speed_last_out_l + car_config::kSpeedPidSlewRate;
    if (slew < -car_config::kSpeedPidSlewRate) raw_out = speed_last_out_l - car_config::kSpeedPidSlewRate;

    speed_last_erro_L = speed_erro_L;
    speed_last_out_l = raw_out;
    speed_out_l = raw_out;
}



void Speed_PID_R(void)
{
    speed_erro_R = Speed_Goal_r - speed_encoder_R;

    if (Speed_Goal_r <= 0.02f) {
        speed_out_r = 0.0f;
        speed_last_erro_R = 0.0f;
        speed_integral_R = 0.0f;
        speed_last_out_r = 0.0f;
        return;
    }

    // 位置式 PID: output = P*err + Ki*integral + Kd*derivative
    update_speed_integral(speed_erro_R, speed_integral_R);

    const float derivative = speed_erro_R - speed_last_erro_R;
    float raw_out = speed_P_r * speed_erro_R +
                    speed_I_r * speed_integral_R +
                    speed_D_r * derivative;
    if (speed_erro_R > 0.0f) {
        raw_out += car_config::kSpeedPidFeedForwardPerMS * Speed_Goal_r;
    }

    if (raw_out > MAX_PWM) raw_out = MAX_PWM;
    if (raw_out < 0.0f) {
        const float overshoot = speed_encoder_R - Speed_Goal_r;
        const bool turn_inside_wheel =
            ImageStatus.Road_type != Cross &&
            ImageStatus.Road_type != Cross_ture &&
            Speed_Goal_r + 0.08f < Speed_Goal;
        if (turn_inside_wheel && overshoot > car_config::kSpeedBrakeThreshold) {
            if (raw_out < car_config::kSpeedTurnBrakeMaxPwm)
                raw_out = car_config::kSpeedTurnBrakeMaxPwm;
        } else {
            raw_out = car_config::kSpeedBrakeMaxPwm;
        }
    }

    // 输出变化率限制
    float slew = raw_out - speed_last_out_r;
    if (slew > car_config::kSpeedPidSlewRate) raw_out = speed_last_out_r + car_config::kSpeedPidSlewRate;
    if (slew < -car_config::kSpeedPidSlewRate) raw_out = speed_last_out_r - car_config::kSpeedPidSlewRate;

    speed_last_erro_R = speed_erro_R;
    speed_last_out_r = raw_out;
    speed_out_r = raw_out;
}



// ===============================================================================
// [5] Basic cascade control
// ===============================================================================

//差�?
// 【中外环替代：姿态串级差速逻辑】将基于 MPU6050 实现三串级：目标角度 -> 目标角角速度 -> 目标差�?
static void distribute_wheel_speeds(float speed_goal, float diff_speed, float effective_min_wheel) {
    // 动态差速限幅：偏差大时放大差速，回中后退回温和差速。
    const float diff_limit_err = std::fabs(dbg_steer_error);
    float diff_limit_ratio = 0.65f;
    if (diff_limit_err >= 8.0f) {
        diff_limit_ratio = 0.80f;
    } else if (diff_limit_err > 3.0f) {
        const float u = (diff_limit_err - 3.0f) / (8.0f - 3.0f);
        diff_limit_ratio = 0.65f + (0.80f - 0.65f) * u;
    }
    float dynamic_diff_limit = speed_goal * diff_limit_ratio;
    if (effective_min_wheel > 0.0f && effective_min_wheel < speed_goal) {
        const float tight_turn_limit = speed_goal - effective_min_wheel;
        if (tight_turn_limit > dynamic_diff_limit) {
            dynamic_diff_limit = tight_turn_limit;
        }
    }
    if (dynamic_diff_limit > car_config::kGyroPidOutputLimit)
        dynamic_diff_limit = car_config::kGyroPidOutputLimit;
    if (diff_speed > dynamic_diff_limit) diff_speed = dynamic_diff_limit;
    if (diff_speed < -dynamic_diff_limit) diff_speed = -dynamic_diff_limit;

    // 记录限幅后实际执行值（供日志打印）
    dbg_diff_cmd = diff_speed;
    dbg_diff_limit = dynamic_diff_limit;

    Speed_Goal_l = speed_goal - diff_speed;
    Speed_Goal_r = speed_goal + diff_speed;

    if (speed_goal <= 0.0f) {
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
    } else {
        if (Speed_Goal_l < 0.0f) Speed_Goal_l = 0.0f;
        if (Speed_Goal_r < 0.0f) Speed_Goal_r = 0.0f;
        if (Speed_Goal_l > car_config::kMaxWheelSpeedGoal) Speed_Goal_l = car_config::kMaxWheelSpeedGoal;
        if (Speed_Goal_r > car_config::kMaxWheelSpeedGoal) Speed_Goal_r = car_config::kMaxWheelSpeedGoal;
    }
    dbg_speed_err_l = Speed_Goal_l - speed_encoder_L;
    dbg_speed_err_r = Speed_Goal_r - speed_encoder_R;
}

void speed_dif_cascade(float inner_dt)
{
    static auto last_visual_frame_time_ms = 0LL;
    static uint32_t last_processed_seq = 0;
    static float last_steer_error = 0.0f;
    static float steer_integral = 0.0f;
    static float target_gyro_z = 0.0f;
    static float raw_steer_error = 0.0f;
    static float min_wheel_for_diff = 0.0f;
    static float guarded_turn_sign = 0.0f;
    static float guarded_turn_release_ratio = 0.0f;
    static float last_diff_slew = 0.0f;
    static int imu_consecutive_fail = 0;
    static bool last_vision_was_cross = false;
    static long long cross_exit_speed_cap_until_ms = 0;
    static bool cross_entry_align_armed = true;
    static int cross_entry_rearm_frames = 0;
    static float last_approach_heading_error_px = 0.0f;
    static long long last_approach_heading_ms = 0;
    static float cross_entry_align_dps = 0.0f;
    static long long cross_entry_align_until_ms = 0;
    static long long near_far_conflict_until_ms = 0;
    static float near_far_conflicting_sign = 0.0f;
    static float near_far_last_valid_near_error = 0.0f;
    static long long near_far_last_valid_near_ms = 0;
    static int near_far_release_confirm_frames = 0;
    static long long near_far_recovery_blend_start_ms = 0;
    static int near_far_release_reason = 0;

    if (SystemData.Stop == 1) {
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        last_steer_error = 0.0f;
        steer_integral = 0.0f;
        target_gyro_z = 0.0f;
        raw_steer_error = 0.0f;
        min_wheel_for_diff = 0.0f;
        guarded_turn_sign = 0.0f;
        guarded_turn_release_ratio = 0.0f;
        planned_speed_goal = 0.0f;
        last_diff_slew = 0.0f;
        imu_consecutive_fail = 0;
        last_vision_was_cross = false;
        cross_exit_speed_cap_until_ms = 0;
        cross_entry_align_armed = true;
        cross_entry_rearm_frames = 0;
        last_approach_heading_error_px = 0.0f;
        last_approach_heading_ms = 0;
        cross_entry_align_dps = 0.0f;
        cross_entry_align_until_ms = 0;
        near_far_conflict_until_ms = 0;
        near_far_conflicting_sign = 0.0f;
        near_far_last_valid_near_error = 0.0f;
        near_far_last_valid_near_ms = 0;
        near_far_release_confirm_frames = 0;
        near_far_recovery_blend_start_ms = 0;
        near_far_release_reason = 0;
        cascade_ctrl.reset_runtime_state();

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
        dbg_dt = 0.0f;
        dbg_max_diff = 0.0f;
        dbg_diff_before_limit = 0.0f;
        dbg_speed_err_l = 0.0f;
        dbg_speed_err_r = 0.0f;
        dbg_diff_limited = 0;
        dbg_imu_ok = 0;
        dbg_deadband_active = 0;
        dbg_cross_heading_error_px = 0.0f;
        dbg_cross_align_dps = 0.0f;
        dbg_cross_align_active = 0;
        dbg_near_center_error_px = 0.0f;
        dbg_near_center_add_dps = 0.0f;
        dbg_near_control_error_px = 0.0f;
        dbg_near_far_scale = 1.0f;
        dbg_near_far_ambiguous = 0;
        dbg_near_far_release_reason = 0;
        return;
    }

    const long long last_frame_ms =
        g_control_last_vision_frame_ms.load(std::memory_order_acquire);
    const long long vision_age_ms =
        (last_frame_ms > 0) ? (control_now_ms() - last_frame_ms) : 0;
    const bool vision_frame_stale =
        (last_frame_ms > 0) && (vision_age_ms > kVisionFreshTimeoutMs);
    const uint32_t current_seq = g_control_vision_frame_seq.load(std::memory_order_acquire);
    const bool new_frame_arrived = (current_seq != last_processed_seq);

    if (vision_frame_stale) {
        if (new_frame_arrived) {
            last_processed_seq = current_seq;
        }
        const bool hold_turn =
            (vision_age_ms <= kVisionStaleTurnHoldMs) &&
            (std::fabs(last_steer_error) > (float)car_config::kSteerDeadBand);

        if (hold_turn) {
            raw_steer_error = last_steer_error;
            guarded_turn_sign = (last_steer_error > 0.0f) ? 1.0f : -1.0f;
            guarded_turn_release_ratio = 0.0f;
            planned_speed_goal =
                (min_wheel_for_diff > 0.0f) ? kVisionRecoverySpeedGoal : kVisionStaleSpeedGoal;
        } else {
            last_steer_error = 0.0f;
            steer_integral = 0.0f;
            target_gyro_z = 0.0f;
            raw_steer_error = 0.0f;
            min_wheel_for_diff = 0.0f;
            guarded_turn_sign = 0.0f;
            guarded_turn_release_ratio = 0.0f;
            planned_speed_goal = kVisionStaleSpeedGoal;
        }

        dbg_raw_steer_error = raw_steer_error;
        dbg_steer_error = hold_turn ? last_steer_error : 0.0f;
        dbg_diff_steer = 0.0f;
        dbg_visual_pd_out = target_gyro_z;
        dbg_target_gyro_z = target_gyro_z;
        dbg_target_delta = 0.0f;
        dbg_dt = 0.0f;
        dbg_deadband_active = 5;
        dbg_max_diff = car_config::kGyroPidOutputLimit;
    } else if (new_frame_arrived) {
        VisionSnapshot local_vision;
        {
            std::lock_guard<std::mutex> lock(g_vision_mutex);
            local_vision = g_vision_snapshot;
        }
        last_processed_seq = current_seq;

        const long long current_frame_time_ms = g_control_last_vision_frame_ms.load(std::memory_order_acquire);
        const bool current_vision_is_cross =
            local_vision.Road_type == Cross || local_vision.Road_type == Cross_ture;

        // 弯道末端的近端中心仍明显偏向一侧时，即使远端Det已经回到中线，车头姿态也可能尚未摆正。
        // 只在边界可信的非十字帧记忆“近端中心-前瞻中心”，并在首次十字触发后短时反向对正。
        const bool approach_heading_valid =
            !current_vision_is_cross &&
            local_vision.OFFLine <= car_config::kCrossEntryHeadingMaxOffline &&
            local_vision.NearLeftFound && local_vision.NearRightFound &&
            (local_vision.TowLeftFound || local_vision.TowRightFound);
        if (approach_heading_valid) {
            last_approach_heading_error_px = clamp_float(
                (float)(local_vision.NearCenter - local_vision.TowCenter),
                -car_config::kCrossEntryHeadingLimitPx,
                car_config::kCrossEntryHeadingLimitPx);
            last_approach_heading_ms = current_frame_time_ms;
            if (!cross_entry_align_armed) {
                ++cross_entry_rearm_frames;
                if (cross_entry_rearm_frames >= car_config::kCrossEntryRearmNormalFrames) {
                    cross_entry_align_armed = true;
                    cross_entry_rearm_frames = 0;
                }
            }
        } else {
            cross_entry_rearm_frames = 0;
        }

        if (current_vision_is_cross && !last_vision_was_cross && cross_entry_align_armed) {
            cross_entry_align_armed = false;
            const long long heading_age_ms = current_frame_time_ms - last_approach_heading_ms;
            if (last_approach_heading_ms > 0 &&
                heading_age_ms >= 0 &&
                heading_age_ms <= car_config::kCrossEntryHeadingMemoryMs) {
                cross_entry_align_dps = clamp_float(
                    last_approach_heading_error_px * car_config::kCrossEntryHeadingGainDps,
                    -car_config::kCrossEntryHeadingLimitDps,
                    car_config::kCrossEntryHeadingLimitDps);
                cross_entry_align_until_ms =
                    current_frame_time_ms + car_config::kCrossEntryAlignDurationMs;
            } else {
                cross_entry_align_dps = 0.0f;
                cross_entry_align_until_ms = 0;
            }
        }
        const bool cross_entry_align_active =
            current_frame_time_ms < cross_entry_align_until_ms &&
            std::fabs(cross_entry_align_dps) > 0.1f;
        if (!cross_entry_align_active) {
            cross_entry_align_dps = 0.0f;
        }

        if (last_vision_was_cross && !current_vision_is_cross) {
            cross_exit_speed_cap_until_ms =
                current_frame_time_ms + car_config::kCrossExitSpeedHoldMs;
        }
        last_vision_was_cross = current_vision_is_cross;
        float visual_dt = (last_visual_frame_time_ms > 0) ?
            (float)(current_frame_time_ms - last_visual_frame_time_ms) / 1000.0f : 0.010f;
        last_visual_frame_time_ms = current_frame_time_ms;
        if (visual_dt <= 0.0001f || visual_dt > 0.1f) {
            visual_dt = 0.010f;
        }
        dbg_dt = visual_dt;

        raw_steer_error = local_vision.MiddleLine - local_vision.Det_True;
        float steer_error = raw_steer_error;
        if (current_vision_is_cross) {
            steer_error = clamp_float(
                steer_error,
                -car_config::kCrossSteerErrorLimit,
                car_config::kCrossSteerErrorLimit);
        }

        // --- 丢线/丢边保护 ---
        // 三级保护，越严重干预越强
        //
        // Level 1: 全丢线 OFFLine>40 → steer_error衰减到上次50%
        // Level 2: 单侧严重丢边 (ll>20 && rl<=3) 或 (rl>20 && ll<=3)
        //          → 不信任det，steer_error衰减到上次30%，强制降速
        // Level 3: 正常死区 |err|<=2 → steer_error=0
        //
        // 单侧丢边保护原理：弯道里一侧边界丢失后，DrawExtensionLine补的边界
        // 往往贴图像边缘（tpL=0~2），导致Center和det严重偏移。
        // 此时近端边界也可能不可靠，不能简单外推，只能保护性回中。

        const bool left_severe_loss =
            (local_vision.Left_Line > 20 && local_vision.Right_Line <= 3);
        const bool right_severe_loss =
            (local_vision.Right_Line > 20 && local_vision.Left_Line <= 3);
        const bool both_sides_weak =
            local_vision.Left_Line >= car_config::kCurveCenterOppositeLossMax &&
            local_vision.Right_Line >= car_config::kCurveCenterOppositeLossMax;

        // 单侧中度丢边(>12)时记录持续帧数，用于检测"det持续偏差锁死"
        // 即使丢边不到20，但如果det持续大偏差+单侧丢边，也说明中线不可信
        static int left_loss_persist = 0;
        static int right_loss_persist = 0;
        static int det_stuck_count = 0;  // det持续大偏差计数

        if (local_vision.Left_Line > 12 && local_vision.Right_Line <= 5) {
            left_loss_persist++;
        } else {
            left_loss_persist = 0;
        }
        if (local_vision.Right_Line > 12 && local_vision.Left_Line <= 5) {
            right_loss_persist++;
        } else {
            right_loss_persist = 0;
        }

        // det持续偏差检测：|err|>8 连续超过8帧，且当前有单侧丢边
        // 说明中线被补线拉偏，不可信
        const bool det_stuck_with_loss =
            (std::fabs(steer_error) > 8.0f) &&
            (left_loss_persist > 8 || right_loss_persist > 8) &&
            (local_vision.Left_Line > 8 || local_vision.Right_Line > 8);

        min_wheel_for_diff = 0.0f;

        if (local_vision.OFFLine > 40) {
            // 全丢线：如果刚才已经在急弯，低速保持上一帧转向方向，避免直角弯里被拉直。
            const float max_err_on_full_loss = 26.0f;
            steer_error = clamp_float(
                last_steer_error,
                -max_err_on_full_loss,
                max_err_on_full_loss);
            if (std::fabs(steer_error) > (float)car_config::kSteerDeadBand) {
                min_wheel_for_diff = kTurnRecoveryMinWheel;
                guarded_turn_sign = (steer_error > 0.0f) ? 1.0f : -1.0f;
                guarded_turn_release_ratio = 0.0f;
            } else {
                guarded_turn_sign = 0.0f;
                guarded_turn_release_ratio = 0.0f;
            }
            dbg_deadband_active = 2;
        } else if (left_severe_loss || right_severe_loss) {
            // 单侧严重丢边(>20)：直角弯需要明显更大的角速度目标，不能过早反向刹转。
            const float max_err_on_loss = 34.0f;
            if (steer_error > max_err_on_loss) steer_error = max_err_on_loss;
            if (steer_error < -max_err_on_loss) steer_error = -max_err_on_loss;
            min_wheel_for_diff = kTurnRecoveryMinWheel;
            guarded_turn_sign = (std::fabs(steer_error) > (float)car_config::kSteerDeadBand) ?
                ((steer_error > 0.0f) ? 1.0f : -1.0f) : 0.0f;
            guarded_turn_release_ratio = 0.0f;
            dbg_deadband_active = 3;
        } else if (det_stuck_with_loss) {
            // det持续偏差+单侧丢边：保留较强转弯能力，但略低于明确单侧丢边。
            const float max_err_on_stuck = 28.0f;
            if (steer_error > max_err_on_stuck) steer_error = max_err_on_stuck;
            if (steer_error < -max_err_on_stuck) steer_error = -max_err_on_stuck;
            min_wheel_for_diff = kTurnRecoveryMinWheel;
            guarded_turn_sign = (std::fabs(steer_error) > (float)car_config::kSteerDeadBand) ?
                ((steer_error > 0.0f) ? 1.0f : -1.0f) : 0.0f;
            guarded_turn_release_ratio = kGuardedTurnReleaseRatio;
            dbg_deadband_active = 4;
        } else if (std::fabs(steer_error) <= (float)car_config::kSteerDeadBand) {
            steer_error = 0.0f;
            guarded_turn_sign = 0.0f;
            guarded_turn_release_ratio = 0.0f;
            dbg_deadband_active = 1;
        } else {
            guarded_turn_sign = 0.0f;
            guarded_turn_release_ratio = 0.0f;
            dbg_deadband_active = 0;
        }

        steer_integral = clamp_float(
            steer_integral + steer_error * visual_dt,
            -car_config::kSteerIntegralLimit,
            car_config::kSteerIntegralLimit);

        // D 项 dt 归一化：换算到 10ms 标称周期，消除帧率波动对 D 项增益和 curve_score 的影响
        const float dt_ratio = visual_dt / 0.010f;
        const float diff_steer = (steer_error - last_steer_error) / dt_ratio;

        const float steer_kd = current_vision_is_cross ?
            car_config::kCrossSteerKd : car_config::kSteerKd;

        // Det 使用前瞻中心，适合建立弯道角速度；近端中心反映车身当前横向位置。
        // 两侧近端边界都可信时叠加一个小修正，消除“远端已回中、车身仍贴边”的出弯盲区。
        float near_center_error = 0.0f;
        const bool near_center_valid =
            !current_vision_is_cross &&
            local_vision.OFFLine <= car_config::kNearCenterSteerMaxOffline &&
            local_vision.NearLeftFound && local_vision.NearRightFound;
        if (near_center_valid) {
            near_center_error = (float)(local_vision.MiddleLine - local_vision.NearCenter);
            if (std::fabs(near_center_error) <= car_config::kNearCenterSteerDeadBandPx) {
                near_center_error = 0.0f;
            }
            near_center_error = clamp_float(
                near_center_error,
                -car_config::kNearCenterSteerErrorLimitPx,
                car_config::kNearCenterSteerErrorLimitPx);
            near_far_last_valid_near_error = near_center_error;
            near_far_last_valid_near_ms = current_frame_time_ms;
        }

        // 纯中线几何一致性保护：近端仍有完整双边，而远端中线与近端反向且远端边界
        // 明显丢失时，远端中心很可能落在横向白区/错误支路。歧义期间完全沿近端
        // 中线行驶；只有前瞻与近端重新形成连续、合理宽度的同一走廊并稳定多帧，
        // 才渐进交还远端控制。全程不切换 Road_type，也不做十字元素识别。
        const int far_conflict_side_loss =
            (steer_error < 0.0f) ? local_vision.Left_Line : local_vision.Right_Line;
        const bool near_far_conflict_trigger =
            near_center_valid &&
            std::fabs(steer_error) >= car_config::kNearFarConflictMinFarErrorPx &&
            near_center_error * steer_error < 0.0f &&
            far_conflict_side_loss > car_config::kNearFarConflictSideLossMin;

        const int near_far_center_gap =
            std::abs(local_vision.TowCenter - local_vision.NearCenter);
        const bool tow_corridor_valid =
            local_vision.TowLeftFound && local_vision.TowRightFound &&
            local_vision.TowWidth >= car_config::kNearFarConflictReleaseTowWidthMinPx &&
            local_vision.TowWidth <= car_config::kNearFarConflictReleaseTowWidthMaxPx;
        const bool release_corridor_aligned =
            near_center_valid && tow_corridor_valid &&
            near_far_center_gap <= car_config::kNearFarConflictReleaseCenterGapPx;
        // 横支路错误方向在交叉核心后翻转，并重新形成正常宽度的纵向走廊，
        // 表示真实出口已经出现。连续确认后提前柔和接管，避免等硬超时才大幅回拉。
        const bool release_outlet_reversed =
            near_far_conflicting_sign != 0.0f &&
            near_center_valid && tow_corridor_valid &&
            std::fabs(steer_error) >= car_config::kNearFarConflictMinFarErrorPx &&
            steer_error * near_far_conflicting_sign < 0.0f &&
            near_far_center_gap <= car_config::kNearFarConflictOutletGapMaxPx;
        const bool release_geometry_confirmed =
            release_corridor_aligned || release_outlet_reversed;

        bool near_far_ambiguous = near_far_conflicting_sign != 0.0f;
        if (near_far_ambiguous) {
            if (release_geometry_confirmed) {
                ++near_far_release_confirm_frames;
            } else {
                near_far_release_confirm_frames = 0;
            }
            const bool release_confirmed =
                near_far_release_confirm_frames >= car_config::kNearFarConflictReleaseFrames;
            const bool conflict_timed_out =
                current_frame_time_ms >= near_far_conflict_until_ms;
            if (release_confirmed || conflict_timed_out) {
                const int confirmed_release_reason =
                    release_outlet_reversed ? 3 : 1;
                near_far_conflicting_sign = 0.0f;
                near_far_conflict_until_ms = 0;
                near_far_release_confirm_frames = 0;
                near_far_recovery_blend_start_ms = current_frame_time_ms;
                near_far_release_reason =
                    release_confirmed ? confirmed_release_reason : 2;
                near_far_ambiguous = false;
            }
        }

        bool near_far_recovery_blend_active = false;
        if (!near_far_ambiguous && near_far_recovery_blend_start_ms > 0) {
            const long long blend_age_ms =
                current_frame_time_ms - near_far_recovery_blend_start_ms;
            if (blend_age_ms >= 0 &&
                blend_age_ms < car_config::kNearFarConflictRecoveryBlendMs) {
                near_far_recovery_blend_active = true;
            } else {
                near_far_recovery_blend_start_ms = 0;
                near_far_release_reason = 0;
            }
        }

        if (!near_far_ambiguous && !near_far_recovery_blend_active &&
            near_far_conflict_trigger) {
            near_far_conflicting_sign = (steer_error > 0.0f) ? 1.0f : -1.0f;
            near_far_conflict_until_ms =
                current_frame_time_ms + car_config::kNearFarConflictMaxHoldMs;
            near_far_release_confirm_frames = 0;
            near_far_release_reason = 0;
            near_far_ambiguous = true;
        }

        float near_control_error = near_center_error;
        if (near_far_ambiguous && !near_center_valid && near_far_last_valid_near_ms > 0) {
            const long long near_age_ms = current_frame_time_ms - near_far_last_valid_near_ms;
            if (near_age_ms >= 0 && near_age_ms <= car_config::kNearFarConflictNearHoldMs) {
                near_control_error = near_far_last_valid_near_error;
            } else if (near_age_ms > car_config::kNearFarConflictNearHoldMs &&
                       near_age_ms < car_config::kNearFarConflictNearHoldMs +
                                         car_config::kNearFarConflictNearDecayMs) {
                const float decay = 1.0f -
                    (float)(near_age_ms - car_config::kNearFarConflictNearHoldMs) /
                    (float)car_config::kNearFarConflictNearDecayMs;
                near_control_error = near_far_last_valid_near_error * decay;
            } else {
                near_control_error = 0.0f;
            }
        }

        float far_confidence = 1.0f;
        if (near_far_ambiguous) {
            far_confidence = car_config::kNearFarConflictFarScale;
        } else if (near_far_recovery_blend_active) {
            const long long blend_age_ms =
                current_frame_time_ms - near_far_recovery_blend_start_ms;
            far_confidence = clamp_float(
                (float)blend_age_ms /
                    (float)car_config::kNearFarConflictRecoveryBlendMs,
                0.0f,
                1.0f);
        }
        if (far_confidence < 1.0f) {
            // 丢边保护的锁向来自当前不可靠的远端误差，不能压掉近端回正方向。
            guarded_turn_sign = 0.0f;
            guarded_turn_release_ratio = 0.0f;
        }
        const float near_center_add =
            near_control_error * car_config::kNearCenterSteerGainDpsPerPx;
        const float far_pid =
            car_config::kSteerKp * steer_error +
            car_config::kSteerKi * steer_integral +
            steer_kd * diff_steer;
        float visual_pid_out = far_confidence * far_pid + near_center_add;
        if (cross_entry_align_active) {
            visual_pid_out += cross_entry_align_dps;
        }
        visual_pid_out = clamp_float(
            visual_pid_out,
            -car_config::kSteerOutputLimitDps,
            car_config::kSteerOutputLimitDps);

        target_gyro_z = visual_pid_out;

        // --- 速度规划：基于视觉误差和误差变化率计算目标速度 ---
        // curve_score = |error| + w * |dError/dt|，综合当前偏差和入弯趋势
        const float abs_err = std::fabs(steer_error);
        const float abs_diff = std::fabs(diff_steer);
        const float curve_score = abs_err + car_config::kSpeedPlanDerivWeight * abs_diff;

        // 线性插值：误差 2 开始减速，误差 8 降到弯道速度
        const float curve_u = clamp_float(
            (curve_score - car_config::kSpeedPlanSlowStart) /
            (car_config::kSpeedPlanFullCurve - car_config::kSpeedPlanSlowStart),
            0.0f,
            1.0f);
        const float vision_loss_score = (float)std::max(
            local_vision.OFFLine,
            std::max(local_vision.Left_Line, local_vision.Right_Line));
        const float vision_loss_u = clamp_float(
            (vision_loss_score - car_config::kVisionLossSlowStart) /
            (car_config::kVisionLossFullCurve - car_config::kVisionLossSlowStart),
            0.0f,
            1.0f);
        const float u = std::max(curve_u, vision_loss_u);

        float speed_to_plan = car_config::kBaseSpeedGoal +
            (car_config::kCurveSpeedGoal - car_config::kBaseSpeedGoal) * u;

        if (SystemData.Stop == 1) {
            speed_to_plan = 0.0f;
        }

        // 双侧弱观测只做通用低速保护，不改变道路类型或转向方向。
        // 元素模式中的十字、单侧严重丢边或det持续偏差仍使用各自独立降级速度。
        if (local_vision.OFFLine > 40) {
            speed_to_plan = kVisionRecoverySpeedGoal;
        } else if (both_sides_weak) {
            speed_to_plan = car_config::kDegradedSpeedGoal;
        } else if (local_vision.Road_type == Cross ||
                   local_vision.Road_type == Cross_ture) {
            speed_to_plan = car_config::kDegradedSpeedGoal;
        } else if (left_severe_loss || right_severe_loss) {
            speed_to_plan = car_config::kSingleSideLossSpeedGoal;
        } else if (det_stuck_with_loss) {
            speed_to_plan = car_config::kDetStuckSpeedGoal;
        }

        if (!current_vision_is_cross &&
            current_frame_time_ms < cross_exit_speed_cap_until_ms &&
            speed_to_plan > car_config::kCrossExitSpeedGoal) {
            speed_to_plan = car_config::kCrossExitSpeedGoal;
        }

        planned_speed_goal = speed_to_plan;

        last_steer_error = steer_error;

        dbg_raw_steer_error = raw_steer_error;
        dbg_steer_error = steer_error;
        dbg_diff_steer = diff_steer;
        dbg_visual_pd_out = visual_pid_out;
        dbg_target_gyro_z = target_gyro_z;
        dbg_target_delta = 0.0f;
        dbg_max_diff = car_config::kGyroPidOutputLimit;
        dbg_cross_heading_error_px = last_approach_heading_error_px;
        dbg_cross_align_dps = cross_entry_align_active ? cross_entry_align_dps : 0.0f;
        dbg_cross_align_active = cross_entry_align_active ? 1 : 0;
        dbg_near_center_error_px = near_center_error;
        dbg_near_center_add_dps = near_center_add;
        dbg_near_control_error_px = near_control_error;
        dbg_near_far_scale = far_confidence;
        dbg_near_far_ambiguous = near_far_ambiguous ? 1 : 0;
        dbg_near_far_release_reason = near_far_release_reason;
    }

    Mpu6050Data imu_data;
    if (!imu.read_processed_data(imu_data)) {
        // IMU硬失败：降级为纯视觉PD控制，不退出
        dbg_imu_ok = 0;
        ++imu_consecutive_fail;

        if (imu_consecutive_fail == 1) {
            printf("[IMU_GUARD] I2C stale, switching to visual-only fallback\n");
        }

        // 纯视觉PD差速：用视觉误差直接计算差速，不经过陀螺仪内环
        // 旧版55a90fa验证过的fallback逻辑，按m/s尺度标定
        float visual_diff = 0.0f;
        if (std::fabs(target_gyro_z) > 0.5f) {
            // target_gyro_z已经包含视觉PD信息(Kp*err + Kd*dE)，直接按比例转差速
            // kSteerKp=2.0时 err=20→tgt=40，视觉PD差速约0.15m/s
            visual_diff = target_gyro_z * car_config::kGyroFeedforwardGain * 2.0f;
            // 限幅到动态差速上限
            float vis_limit = Speed_Goal * 0.65f;
            if (vis_limit > car_config::kGyroPidOutputLimit)
                vis_limit = car_config::kGyroPidOutputLimit;
            if (visual_diff > vis_limit) visual_diff = vis_limit;
            if (visual_diff < -vis_limit) visual_diff = -vis_limit;
        }

        // 套slew rate（不复位，保持连续性）
        {
            const float max_slew = car_config::kDiffSlewRatePerSec * inner_dt;
            float slew = visual_diff - last_diff_slew;
            if (slew > max_slew) visual_diff = last_diff_slew + max_slew;
            if (slew < -max_slew) visual_diff = last_diff_slew - max_slew;
            last_diff_slew = visual_diff;
        }

        // IMU fallback 保持基线验证过的降级速度，不随正常弯道提速。
        if (planned_speed_goal > car_config::kDegradedSpeedGoal) {
            planned_speed_goal = car_config::kDegradedSpeedGoal;
        }

        dbg_diff_speed = visual_diff;
        dbg_diff_cmd = visual_diff;
        dbg_diff_before_limit = visual_diff;
        dbg_cmd_target_gyro_z = target_gyro_z;
        dbg_real_gyro_z = 0.0f;
        dbg_filtered_gyro_z = 0.0f;
        dbg_gyro_error = 0.0f;

        distribute_wheel_speeds(Speed_Goal, visual_diff, min_wheel_for_diff);
        Speed_PID_L();
        Speed_PID_R();
        motor_Duty_R(static_cast<int>(speed_out_r));
        motor_Duty_L(static_cast<int>(speed_out_l));
        gpio_protect_heartbeat_pulse();
        return;
    }
    // IMU恢复后重置失败计数
    imu_consecutive_fail = 0;

    dbg_imu_ok = 1;
    const float diff_speed = cascade_ctrl.calc_diff_speed_bypass_angle(target_gyro_z, imu_data, inner_dt);
    float guarded_diff_speed = diff_speed;
    if (guarded_turn_sign != 0.0f && guarded_diff_speed * guarded_turn_sign < 0.0f) {
        float release_limit = (Speed_Goal - min_wheel_for_diff) *
            guarded_turn_release_ratio;
        if (release_limit < 0.0f) release_limit = 0.0f;
        if (guarded_diff_speed > release_limit) {
            guarded_diff_speed = release_limit;
        } else if (guarded_diff_speed < -release_limit) {
            guarded_diff_speed = -release_limit;
        }
    }

    dbg_cmd_target_gyro_z = target_gyro_z;
    dbg_real_gyro_z = imu_data.gyro_z;
    dbg_filtered_gyro_z = cascade_ctrl.last_filtered_gyro_z();
    dbg_gyro_error = cascade_ctrl.gyro_pid.err;
    dbg_diff_before_limit = diff_speed;
    dbg_diff_speed = diff_speed;
    dbg_diff_limited = 0;

    // 视觉死区差速硬限制：|tgt|<5dps时陀螺噪声不应驱动大差速
    if (std::fabs(target_gyro_z) < car_config::kDeadbandDiffTriggerDps) {
        if (guarded_diff_speed > car_config::kDeadbandDiffLimit)
            guarded_diff_speed = car_config::kDeadbandDiffLimit;
        if (guarded_diff_speed < -car_config::kDeadbandDiffLimit)
            guarded_diff_speed = -car_config::kDeadbandDiffLimit;
    }

    // 差速输出变化率限制：按inner_dt时间归一化，抑制逐帧翻转自激
    {
        const bool same_turn_direction = guarded_diff_speed * last_diff_slew >= 0.0f;
        const bool visual_turn_build =
            std::fabs(target_gyro_z) >= 20.0f && guarded_diff_speed * target_gyro_z > 0.0f;
        const bool building_turn =
            visual_turn_build && same_turn_direction &&
            std::fabs(guarded_diff_speed) > std::fabs(last_diff_slew);
        const bool releasing_or_reversing =
            !same_turn_direction || std::fabs(guarded_diff_speed) < std::fabs(last_diff_slew);
        const bool normal_road =
            ImageStatus.Road_type != Cross && ImageStatus.Road_type != Cross_ture;
        float slew_rate = car_config::kDiffSlewRatePerSec;
        if (normal_road && building_turn) {
            slew_rate = car_config::kDiffTurnBuildSlewRatePerSec;
        } else if (normal_road && releasing_or_reversing) {
            slew_rate = car_config::kDiffTurnReleaseSlewRatePerSec;
        }
        const float max_slew = slew_rate * inner_dt;
        float slew = guarded_diff_speed - last_diff_slew;
        if (slew > max_slew) guarded_diff_speed = last_diff_slew + max_slew;
        if (slew < -max_slew) guarded_diff_speed = last_diff_slew - max_slew;
        last_diff_slew = guarded_diff_speed;
    }

    distribute_wheel_speeds(Speed_Goal, guarded_diff_speed, min_wheel_for_diff);

    if (car_config::kSteerDebugPrintEnable) {
        static int debug_print_count = 0;
        debug_print_count++;
        if (debug_print_count >= car_config::kSteerDebugPrintDivider) {
            debug_print_count = 0;
            // 视觉诊断：打印 TowPoint附近行的边界信息
            int tp = ImageStatus.TowPoint;
            if (tp < 0) tp = 0; if (tp > 59) tp = 59;
            int tp_c = ImageDeal[tp].Center;
            int tp_l = ImageDeal[tp].LeftBorder;
            int tp_r = ImageDeal[tp].RightBorder;
            int tp_lf = (ImageDeal[tp].IsLeftFind == 'T' || ImageDeal[tp].IsLeftFind == 'H') ? 1 : 0;
            int tp_rf = (ImageDeal[tp].IsRightFind == 'T' || ImageDeal[tp].IsRightFind == 'H') ? 1 : 0;
            // 近端行(行55)的边界信息
            int n_c = ImageDeal[55].Center;
            int n_l = ImageDeal[55].LeftBorder;
            int n_r = ImageDeal[55].RightBorder;
            int n_lf = (ImageDeal[55].IsLeftFind == 'T' || ImageDeal[55].IsLeftFind == 'H') ? 1 : 0;
            int n_rf = (ImageDeal[55].IsRightFind == 'T' || ImageDeal[55].IsRightFind == 'H') ? 1 : 0;
            static lq_udp_client steer_udp;
            static bool steer_udp_ready = false;
            if (!steer_udp_ready) {
                steer_udp.udp_client_init(car_config::kRaceTelemetryUdpHost,
                                          car_config::kRaceTelemetryUdpPort);
                steer_udp_ready = true;
            }
            char udp_buf[768];
            int udp_len = std::snprintf(udp_buf, sizeof(udp_buf),
                "steer,det=%d,mid=%d,raw=%.1f,err=%.1f,dE=%.1f,tgt=%.1f,hErr=%.1f,xAdd=%.1f,xOn=%d,nErr=%.1f,nUse=%.1f,nAdd=%.1f,fSc=%.2f,nfAmb=%d,nfRel=%d,gz=%.1f,fgz=%.1f,ge=%.1f,dRaw=%.3f,dCmd=%.3f,dLim=%.3f,goal=%.2f,L=%.2f,R=%.2f,encL=%.2f,encR=%.2f,seL=%.3f,seR=%.3f,pwmL=%.0f,pwmR=%.0f,dt=%.3f,imu=%d,db=%d,off=%d,ll=%d,rl=%d,wl=%d,tp=%d,tpC=%d,tpL=%d,tpR=%d,tpLF=%d,tpRF=%d,nC=%d,nL=%d,nR=%d,nLF=%d,nRF=%d,rt=%d,sa=%d\n",
                ImageStatus.Det_True, ImageStatus.MiddleLine, raw_steer_error, dbg_steer_error,
                dbg_diff_steer, dbg_target_gyro_z, dbg_cross_heading_error_px,
                dbg_cross_align_dps, dbg_cross_align_active,
                dbg_near_center_error_px, dbg_near_control_error_px,
                dbg_near_center_add_dps, dbg_near_far_scale,
                dbg_near_far_ambiguous, dbg_near_far_release_reason,
                dbg_real_gyro_z, dbg_filtered_gyro_z, dbg_gyro_error,
                dbg_diff_speed, dbg_diff_cmd, dbg_diff_limit, Speed_Goal, Speed_Goal_l,
                Speed_Goal_r, speed_encoder_L, speed_encoder_R, dbg_speed_err_l,
                dbg_speed_err_r, speed_out_l, speed_out_r, dbg_dt, dbg_imu_ok,
                dbg_deadband_active, ImageStatus.OFFLine, ImageStatus.Left_Line,
                ImageStatus.Right_Line, ImageStatus.WhiteLine, tp, tp_c, tp_l, tp_r, tp_lf, tp_rf, n_c, n_l,
                n_r, n_lf, n_rf, (int)ImageStatus.Road_type, ImageStatus.straight_acc);
            if (udp_len > 0) steer_udp.udp_send(udp_buf, (size_t)udp_len);
            printf("[steer] det:%d mid:%d raw:%.1f err:%.1f dE:%.1f tgt:%.1f gz:%.1f ge:%.1f dRaw:%.3f dCmd:%.3f dLim:%.3f goal:%.2f L:%.2f R:%.2f encL:%.2f encR:%.2f seL:%.3f seR:%.3f pwmL:%.0f pwmR:%.0f dt:%.3f imu:%d db:%d off:%d ll:%d rl:%d\n",
                   ImageStatus.Det_True,
                   ImageStatus.MiddleLine,
                   raw_steer_error,
                   dbg_steer_error,
                   dbg_diff_steer,
                   dbg_target_gyro_z,
                   dbg_real_gyro_z,
                   dbg_gyro_error,
                   dbg_diff_speed,        // dRaw: 角速度环原始输出（限幅前）
                   dbg_diff_cmd,          // dCmd: 动态限幅后实际执行差速
                   dbg_diff_limit,        // dLim: 当前动态差速限幅值
                   Speed_Goal,
                   Speed_Goal_l,
                   Speed_Goal_r,
                   speed_encoder_L,
                   speed_encoder_R,
                   dbg_speed_err_l,       // seL: 左轮速度误差(goal-enc)
                   dbg_speed_err_r,       // seR: 右轮速度误差(goal-enc)
                   speed_out_l,
                   speed_out_r,
                   dbg_dt,
                   dbg_imu_ok,
                   dbg_deadband_active,
                   ImageStatus.OFFLine,
                   ImageStatus.Left_Line,
                   ImageStatus.Right_Line);
            // 视觉诊断行：TowPoint/近端行的边界信息 + Road_type
            printf("[vis] tp:%d tpC:%d tpL:%d tpR:%d tpLF:%d tpRF:%d nC:%d nL:%d nR:%d nLF:%d nRF:%d rt:%d sa:%d\n",
                   tp, tp_c, tp_l, tp_r, tp_lf, tp_rf,
                   n_c, n_l, n_r, n_lf, n_rf,
                   (int)ImageStatus.Road_type,
                   ImageStatus.straight_acc);
        }
    }
}




// ===============================================================================
// [6] 外设与状态监�?
// ===============================================================================

//编码器记录路�?
void Car_distance(float dt)
{
    Record_encoder = 0.5f * (speed_encoder_L + speed_encoder_R);
    SystemData.SpeedData.Length += Record_encoder * dt;

    if (SystemData.Stop == 2)
    {
        ImageStatus.Stop_lenth += Record_encoder * dt;
    }
    else
        ImageStatus.Stop_lenth = 0.0f;
}

// ===============================================================================
// [7] 主控与调度函�?
// ===============================================================================

//变速控�?




void Speed_BiHuan(void)
{
    static long long control_start_ms = 0;
    const long long last_vision_ms =
        g_control_last_vision_frame_ms.load(std::memory_order_acquire);

    // Waiting for first frame on startup: keep output at 0 but do NOT exit unless timeout exceeded
    if (last_vision_ms <= 0) {
        Speed_Goal = 0.0f;
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_last_erro_R = 0.0f;
        speed_integral_L = 0.0f;
        speed_integral_R = 0.0f;
        speed_last_out_l = 0.0f;
        speed_last_out_r = 0.0f;
        if (motor_L && motor_R) {
            motor_Duty_L(0);
            motor_Duty_R(0);
        }

        // Keep watchdog alive during startup waiting
        gpio_protect_heartbeat_pulse();

        if (control_start_ms == 0) {
            control_start_ms = control_now_ms();
        }
        long long wait_time_ms = control_now_ms() - control_start_ms;
        if (wait_time_ms > 2000) { // 2 seconds timeout
            printf("[VISION_GUARD] Timeout waiting for first vision frame (2000ms exceeded). Exiting.\n");
            exit_flag.store(true, std::memory_order_relaxed);
        }
        return;
    }

    // Reset startup timer once first frame is successfully processed
    control_start_ms = 0;

    const long long vision_age_ms = control_now_ms() - last_vision_ms;
    if (vision_age_ms > kVisionFatalTimeoutMs) {
        // 致命超时：500ms无视觉帧，退出程序
        static int vision_guard_warn_count = 0;
        Speed_Goal = 0.0f;
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_last_erro_R = 0.0f;
        speed_integral_L = 0.0f;
        speed_integral_R = 0.0f;
        speed_last_out_l = 0.0f;
        speed_last_out_r = 0.0f;
        if (motor_L && motor_R) {
            motor_Duty_L(0);
            motor_Duty_R(0);
        }
        if (vision_guard_warn_count < 8) {
            ++vision_guard_warn_count;
            printf("[VISION_GUARD] fatal stale frame age=%lldms; exit\n",
                   vision_age_ms);
        }
        exit_flag.store(true, std::memory_order_relaxed);
        return;
    }
    if (vision_age_ms > kVisionFreshTimeoutMs) {
        // 帧超时但未致命：降速到弯道速度，保持循迹，不退出
        static long long last_vision_warn_ms = 0;
        long long now_ms = control_now_ms();
        if (now_ms - last_vision_warn_ms > kVisionStaleWarnIntervalMs) {
            last_vision_warn_ms = now_ms;
            printf("[VISION_GUARD] stale frame age=%lldms; slowing down\n",
                   vision_age_ms);
        }
        // 强制降速到独立降级速度，避免高速盲跑冲出赛道。
        planned_speed_goal = car_config::kDegradedSpeedGoal;
        // 继续运行控制环（不return），让小车减速循迹
    }

    // 运行�?watchdog 活性检测：gpio_manager 崩溃/重启�?heartbeat_last_ms 停止更新
    if (!gpio_protect_heartbeat_is_wd_alive(car_config::kWatchdogHeartbeatStallTimeoutMs)) {
        static int wd_stall_warn_count = 0;
        Speed_Goal = 0.0f;
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_last_erro_R = 0.0f;
        speed_integral_L = 0.0f;
        speed_integral_R = 0.0f;
        speed_last_out_l = 0.0f;
        speed_last_out_r = 0.0f;
        if (motor_L && motor_R) {
            motor_Duty_L(0);
            motor_Duty_R(0);
        }
        if (wd_stall_warn_count < 8) {
            ++wd_stall_warn_count;
            printf("[WATCHDOG_GUARD] gpio_manager heartbeat stalled; stop output and exit\n");
        }
        exit_flag.store(true, std::memory_order_relaxed);
        return;
    }

    // encoder_num() has built-in filtering; use both wheel readings directly.
    speed_encoder_raw_L = encoder_num_L();
    speed_encoder_raw_R = encoder_num_R();
    if (!encoder_health_ok()) {
        static int encoder_guard_warn_count = 0;
        Speed_Goal = 0.0f;
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_last_erro_R = 0.0f;
        speed_integral_L = 0.0f;
        speed_integral_R = 0.0f;
        speed_last_out_l = 0.0f;
        speed_last_out_r = 0.0f;
        if (motor_L && motor_R) {
            motor_Duty_L(0);
            motor_Duty_R(0);
        }
        if (encoder_guard_warn_count < 8) {
            ++encoder_guard_warn_count;
            printf("[ENCODER_GUARD] invalid encoder feedback; rawL=%.3f rawR=%.3f badL=%d badR=%d, stop output and exit\n",
                   encoder_raw_last_L(), encoder_raw_last_R(),
                   encoder_invalid_count_L(), encoder_invalid_count_R());
        }
        exit_flag.store(true, std::memory_order_relaxed);
        return;
    }
    speed_encoder_L = speed_encoder_raw_L;
    speed_encoder_R = speed_encoder_raw_R;

    static auto last_control_time = std::chrono::steady_clock::now();
    const auto control_time_now = std::chrono::steady_clock::now();
    std::chrono::duration<float> control_elapsed = control_time_now - last_control_time;
    float inner_dt = control_elapsed.count();
    last_control_time = control_time_now;
    if (inner_dt <= 0.0001f || inner_dt > 0.05f) {
        inner_dt = 0.005f;
    }

    Car_distance(inner_dt);

    static float ramped_speed_goal = 0.0f;
    const float desired_speed_goal = (planned_speed_goal > 0.0f) ? planned_speed_goal : 0.0f;
    if (desired_speed_goal <= 0.0f) {
        ramped_speed_goal = 0.0f;
        Speed_Goal = 0.0f;
    } else {
        // 加速度斜坡限幅：加速 1.8 m/s^2，减速 4.0 m/s^2
        const float speed_delta = desired_speed_goal - ramped_speed_goal;
        const float speed_limit = (speed_delta > 0.0f) ?
            car_config::kSpeedAccelLimit : car_config::kSpeedDecelLimit;
        const float max_step = speed_limit * inner_dt;
        if (speed_delta > max_step) {
            ramped_speed_goal += max_step;
        } else if (speed_delta < -max_step) {
            ramped_speed_goal -= max_step;
        } else {
            ramped_speed_goal = desired_speed_goal;
        }
        if (ramped_speed_goal < 0.0f) ramped_speed_goal = 0.0f;
        Speed_Goal = ramped_speed_goal;
    }

    speed_dif_cascade(inner_dt);

    // 外环弯道/丢线保护可能在 speed_dif_cascade() 内把目标速度直接压低，
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
        planned_speed_goal = 0.0f;
        SystemData.Stop = 1;
        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_last_erro_R = 0.0f;
        speed_integral_L = 0.0f;
        speed_integral_R = 0.0f;
        speed_last_out_l = 0.0f;
        speed_last_out_r = 0.0f;
        if (motor_L && motor_R) {
            motor_Duty_L(0);
            motor_Duty_R(0);
        }
        if (speed_guard_warn_count < 8) {
            ++speed_guard_warn_count;
            printf("[SPEED_GUARD] reverse encoder feedback, stop output and lock exit. "
                   "goalL:%.2f goalR:%.2f encL:%.2f encR:%.2f\n",
                   guard_goal_l,
                   guard_goal_r,
                   speed_encoder_L,
                   speed_encoder_R);
        }
        exit_flag.store(true, std::memory_order_relaxed);
        return;
    }

    Speed_PID_L();           // 【最内环】：左右轮速度环PI闭环运算
    Speed_PID_R();           // 【最内环】：左右轮速度环PI闭环运算

    motor_Duty_R(speed_out_r); // 输出到PWM驱动
    motor_Duty_L(speed_out_l);
    // 只有完整完成“编码器读取 �?速度�?�?电机输出”的控制周期后才续命�?
    // 图像/UDP/调试线程不能续命，控制线程卡死时 gpio_manager 必须能超时停车�?
    gpio_protect_heartbeat_pulse();
}

//     float turn = KP * offset;  // 转向�?

//     int base_speed = Speed_Goal;  // 当前目标速度，三轮车车型最大速度设定

//     // 左右轮速度由基准速度和偏差计�?
//     Speed_Goal_l = base_speed - turn;
//     Speed_Goal_r = base_speed + turn;

//     // 限幅保护，防止速度超过范围或为�?
//     if (Speed_Goal_l > MAX_SPEED) Speed_Goal_l = MAX_SPEED;
//     if (Speed_Goal_l < 0)         Speed_Goal_l = 0;

//     if (Speed_Goal_r > MAX_SPEED) Speed_Goal_r = MAX_SPEED;
//     if (Speed_Goal_r < 0)         Speed_Goal_r = 0;
// }

// 电机紧急停止函�?
void emergency_stop() {
    Speed_Goal = 0.0f;
    Speed_Goal_l = 0.0f;
    Speed_Goal_r = 0.0f;
    speed_out_r = 0.0f;
    speed_out_l = 0.0f;
    speed_last_erro_R = 0.0f;
    speed_last_erro_L = 0.0f;
    speed_integral_L = 0.0f;
    speed_integral_R = 0.0f;
    speed_last_out_l = 0.0f;
    speed_last_out_r = 0.0f;

    if (motor_R && motor_L) {
        motor_Duty_R(0);    // 右电机停�?
        motor_Duty_L(0);    // 左电机停�?
    }
    // [T01修正] PH极性：正转=HIGH，停车时PH=HIGH与duty=0配对（EN无输出，方向固定正转端）
    if (motor_R_Dir) motor_R_Dir->gpio_level_set(GPIO_HIGH);
    if (motor_L_Dir) motor_L_Dir->gpio_level_set(GPIO_HIGH);

    if (motor_R && motor_L) {
        motor_R->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
        motor_L->atim_pwm_set_duty(car_config::kMotorPwmStopDuty);
        motor_R->atim_pwm_enable();
        motor_L->atim_pwm_enable();
    }
    // stree_M.SetDutyCycle(170000);  // 舵机归中
    printf("程序终止，电机已停止。\n");
}

// 信号处理函数（捕�?Ctrl+C 等终止信号）
void signal_handler(int signum) {
    (void)signum;
    // 信号上下文只通知退出。PWM/GPIO/心跳清理�?main 的正常执行流完成�?
    // 避免在异步信号中加锁、打印或 exit() 导致停机流程被中断�?
    exit_flag.store(true, std::memory_order_relaxed);
}
