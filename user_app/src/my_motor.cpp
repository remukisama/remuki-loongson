#include "main.hpp"
#include "my_cascade_pid.hpp"
#include "car_config.hpp"
#include <mutex>

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
    uint32_t seq;
};

static VisionSnapshot g_vision_snapshot;
static std::mutex g_vision_mutex;

#include "gpio_protect_heartbeat.hpp"
#include <chrono>
#include <cmath>

// ===============================================================================
// [1] 全局变量与硬件对象定�?
// ===============================================================================

// 全局退出标志与多线程安�?
std::atomic<bool> g_program_exit(false);
std::atomic<bool> exit_flag(false);

// 电机与引脚配�?
// 左电�?PWM：PIN81, 频率10000Hz, 初始占空�?000, 极性反�?
ls_atim_pwm* motor_L = nullptr;  // 延迟初始化，避免全局构造时访问 /dev/mem 导致崩溃
// 右电�?PWM：PIN82, 频率10000Hz, 初始占空�?000, 极性反�?
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

// 三轮车差速转向专用参数（已完全同�?motor.cpp 验证无误的参数）
// 最新调试参数：恢复部分死区和适中的P，但保留足够的打角能力以应付U型弯
float STEER_KP = car_config::kSteerKp;               // 外环P，集中在 car_config.hpp 调整
float STEER_KD = car_config::kSteerKd;               // 外环D，集中在 car_config.hpp 调整
float MAX_DIFF_RATIO = car_config::kMaxDiffRatio;    // 差速上�?基础速度*倍率
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

// 姿�?航向 PID串级控制对象与IMU
AttitudeCascade cascade_ctrl;
MyMpu6050 imu;

static constexpr uint32_t kVisionFreshTimeoutMs = 60;
static std::atomic<uint32_t> g_control_vision_frame_seq(0);
static std::atomic<long long> g_control_last_vision_frame_ms(0);

static long long control_now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void control_mark_vision_frame_ready(void)
{
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
float speed_encoder_L;
float speed_encoder_raw_L = 0.0f;

// 右轮闭环变量
float Speed_Goal_r = 0;
float speed_P_r, speed_I_r, speed_D_r;
float speed_out_r;
float speed_erro_R, speed_last_erro_R;
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
        motor_L = new ls_atim_pwm(ATIM_PWM0_PIN81, car_config::kMotorPwmFrequencyHz, car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
        motor_R = new ls_atim_pwm(ATIM_PWM1_PIN82, car_config::kMotorPwmFrequencyHz, car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
        motor_L_Dir = new ls_gpio(PIN_21, GPIO_MODE_OUT);
        motor_R_Dir = new ls_gpio(PIN_22, GPIO_MODE_OUT);
    }

    motor_R->atim_pwm_set_active_low_duty(0);
    motor_L->atim_pwm_set_active_low_duty(0);
    
    printf("R:OK!");
    
    motor_R->atim_pwm_enable();
    motor_L->atim_pwm_enable();
    motor_R_Dir->gpio_level_set(GPIO_LOW);
    motor_L_Dir->gpio_level_set(GPIO_LOW);
}

void motor_Duty_R(int duty)
{
    if (!motor_R || !motor_R_Dir) return;  // 电机未初始化时安全跳�?
    if (duty == 0)
    {
        motor_R->atim_pwm_set_active_low_duty(0);
        motor_R_Dir->gpio_level_set(GPIO_LOW);
    }
    else if (duty < 0)
    {
        // 反转：DIR=HIGH + 反转PWM占空比（对称力矩�?
        const int magnitude = std::min(-duty, car_config::kMotorPwmFullScale);
        motor_R->atim_pwm_set_active_low_duty(magnitude);
        motor_R_Dir->gpio_level_set(GPIO_HIGH);
    }
    else
    {
        // 正转：DIR=LOW + active_low_duty
        const int magnitude = std::min(duty, car_config::kMotorPwmFullScale);
        motor_R->atim_pwm_set_active_low_duty(magnitude);
        motor_R_Dir->gpio_level_set(GPIO_LOW);
    }
}

void motor_Duty_L(int duty)
{
    if (!motor_L || !motor_L_Dir) return;  // 电机未初始化时安全跳�?
    if (duty == 0)
    {
        motor_L->atim_pwm_set_active_low_duty(0);
        motor_L_Dir->gpio_level_set(GPIO_LOW);
    }
    else if (duty < 0)
    {
        // 反转：DIR=HIGH + 反转PWM占空比（对称力矩�?
        const int magnitude = std::min(-duty, car_config::kMotorPwmFullScale);
        motor_L->atim_pwm_set_active_low_duty(magnitude);
        motor_L_Dir->gpio_level_set(GPIO_HIGH);
    }
    else
    {
        // 正转：DIR=LOW + active_low_duty
        const int magnitude = std::min(duty, car_config::kMotorPwmFullScale);
        motor_L->atim_pwm_set_active_low_duty(magnitude);
        motor_L_Dir->gpio_level_set(GPIO_LOW);
    }
}


// ===============================================================================
// [4] 控制器初始化与PID计算
// ===============================================================================

// ==========================
// [4.1] 速度�?PID
// ==========================

bool Speed_decision(void)
{
    // 参考demo，增量式PI控制
    speed_P_l = car_config::kSpeedPidP;  // 对齐demo参数
    speed_I_l = car_config::kSpeedPidI;  // 对齐demo参数
    speed_D_l = car_config::kSpeedPidD;  // 微分D
    
    speed_P_r = car_config::kSpeedPidP;
    speed_I_r = car_config::kSpeedPidI;
    speed_D_r = car_config::kSpeedPidD;
    
    MAX_PWM = car_config::kMotorMaxPwm;    // PWM最大限�?

    speed_out_l = 0;
    speed_out_r = 0;

    WD_Speed_Goal = car_config::kCurveSpeedGoal;      // 弯道基础速度
    ZD_Speed_Goal = car_config::kStraightSpeedGoal;   // 直道基础速度
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

void Speed_PID_L(void)
{
    speed_erro_L = Speed_Goal_l - speed_encoder_L;

    static float last_goal_l = 0.0f;

    if (Speed_Goal_l <= 0.02f) {
        speed_out_l = 0.0f;
        speed_last_erro_L = 0.0f;
        last_goal_l = 0.0f;
        return;
    }

    // If starting from zero, pre-charge the integrator with feedforward to overcome static friction and speed delay
    if (std::fabs(speed_out_l) < 500.0f) {
        speed_out_l = std::copysign(
            car_config::kMotorStartBasePwm + std::fabs(Speed_Goal_l) * car_config::kSpeedFeedforwardGain,
            Speed_Goal_l);
    } else if (last_goal_l > 0.02f) {
        // Step change during runtime
        float delta_goal = Speed_Goal_l - last_goal_l;
        speed_out_l += delta_goal * car_config::kSpeedFeedforwardGain;
    }
    last_goal_l = Speed_Goal_l;

    // Incremental PI speed loop. With no minimum wheel speed, positive targets do not reverse-drive.
    speed_out_l += speed_I_l * speed_erro_L + speed_P_l * (speed_erro_L - speed_last_erro_L);
    speed_last_erro_L = speed_erro_L;

    if (speed_out_l > MAX_PWM) speed_out_l = MAX_PWM;
    if (speed_out_l < -car_config::kSpeedPidBrakePwm) {
        speed_out_l = -car_config::kSpeedPidBrakePwm;
    }
}


void Speed_PID_R(void)
{
    speed_erro_R = Speed_Goal_r - speed_encoder_R;

    static float last_goal_r = 0.0f;

    if (Speed_Goal_r <= 0.02f) {
        speed_out_r = 0.0f;
        speed_last_erro_R = 0.0f;
        last_goal_r = 0.0f;
        return;
    }

    // If starting from zero, pre-charge the integrator with feedforward to overcome static friction and speed delay
    if (std::fabs(speed_out_r) < 500.0f) {
        speed_out_r = std::copysign(
            car_config::kMotorStartBasePwm + std::fabs(Speed_Goal_r) * car_config::kSpeedFeedforwardGain,
            Speed_Goal_r);
    } else if (last_goal_r > 0.02f) {
        // Step change during runtime
        float delta_goal = Speed_Goal_r - last_goal_r;
        speed_out_r += delta_goal * car_config::kSpeedFeedforwardGain;
    }
    last_goal_r = Speed_Goal_r;

    // Incremental PI speed loop. With no minimum wheel speed, positive targets do not reverse-drive.
    speed_out_r += speed_I_r * speed_erro_R + speed_P_r * (speed_erro_R - speed_last_erro_R);
    speed_last_erro_R = speed_erro_R;

    if (speed_out_r > MAX_PWM) speed_out_r = MAX_PWM;
    if (speed_out_r < -car_config::kSpeedPidBrakePwm) {
        speed_out_r = -car_config::kSpeedPidBrakePwm;
    }
}


// ===============================================================================
// [5] 高级控制与串级逻辑
// ===============================================================================

//差�?
// uint16_t CS_LM=50;

// 【中外环替代：姿态串级差速逻辑】将基于 MPU6050 实现三串级：目标角度 -> 目标角角速度 -> 目标差�?
static void distribute_wheel_speeds(float speed_goal, float diff_speed, float effective_min_wheel) {
    (void)effective_min_wheel;
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
    // --- Status and time variables ---
    static auto last_visual_frame_time_ms = 0LL;
    static uint32_t last_processed_seq = 0;

    static float last_diff_speed = 0.0f;
    static float last_target_gyro_z = 0.0f;
    static float last_steer_error = 0.0f;

    static float raw_steer_error = 0.0f;
    static bool simple_curve_active = false;

    static int consecutive_imu_successes = 0;
    static int consecutive_imu_failures = 0;
    static bool imu_in_fallback = false;

// --- planned control parameters from outer loop to inner loop ---
    static float target_gyro_z = 0.0f;
    static float fallback_diff = 0.0f;
    static float max_diff = 0.0f;
    static float effective_min_wheel = MIN_SPEED;

    static bool was_in_ring = false;
    static float last_control_raw_steer_error = 0.0f;
    static bool raw_steer_initialized = false;

    static bool full_loss_stop_latched = false;

    // 单侧丢线预判只在“入弯阶段”生效一次。
    // 一旦视觉误差已经明显进入弯道，就不允许它在出弯阶段重新介入。
    static int preview_turn_sign = 0;
    static bool preview_committed = false;

    // --- Vehicle stopped/idle state reset ---
    if (SystemData.Stop == 1) {
        full_loss_stop_latched = false;
        preview_turn_sign = 0;
        preview_committed = false;

        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        last_diff_speed = 0.0f;
        last_target_gyro_z = 0.0f;
        last_steer_error = 0.0f;
        raw_steer_error = 0.0f;
        simple_curve_active = false;
        consecutive_imu_successes = 0;
        consecutive_imu_failures = 0;
        imu_in_fallback = false;
        target_gyro_z = 0.0f;
        fallback_diff = 0.0f;
        max_diff = 0.0f;
        effective_min_wheel = MIN_SPEED;
        raw_steer_initialized = false;

        dbg_diff_speed = 0.0f;
        dbg_target_gyro_z = 0.0f;
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

    // --- 1. Dual-rate architecture - visual outer loop (event-driven) ---
    uint32_t current_seq = g_control_vision_frame_seq.load(std::memory_order_acquire);
    bool new_frame_arrived = (current_seq != last_processed_seq);

    if (new_frame_arrived) {
        // A. Lock and copy latest vision snapshot to prevent race conditions
        VisionSnapshot local_vision;
        {
            std::lock_guard<std::mutex> lock(g_vision_mutex);
            local_vision = g_vision_snapshot;
        }
        last_processed_seq = current_seq;

        // B. Calculate physical latency visual_dt
        long long current_frame_time_ms = g_control_last_vision_frame_ms.load(std::memory_order_acquire);
        float visual_dt = (last_visual_frame_time_ms > 0) ? (float)(current_frame_time_ms - last_visual_frame_time_ms) / 1000.0f : 0.010f;
        last_visual_frame_time_ms = current_frame_time_ms;

        if (visual_dt <= 0.0001f || visual_dt > 0.1f) {
            visual_dt = 0.010f; // Default to 100Hz
        }
        dbg_dt = visual_dt;

        // C. Calculate visual error and low-pass/jump protection
        raw_steer_error = local_vision.MiddleLine - local_vision.Det_True;
        dbg_raw_steer_error = raw_steer_error;

        const bool ring_now = (local_vision.image_element_rings != 0);
        const bool ring_entry_phase = ring_now && local_vision.image_element_rings_flag <= 6;
        const bool ring_just_finished = was_in_ring && !ring_now;
        was_in_ring = ring_now;

        const bool bad_vision = local_vision.OFFLine >= car_config::kBadVisionOfflineThreshold;
        const bool full_loss_non_ring = !ring_now && local_vision.OFFLine >= car_config::kBadVisionCenterHoldOfflineThreshold;

        float control_raw_steer_error = raw_steer_error;

        float preview_gyro_bias = 0.0f;

        const int left_loss  = local_vision.Left_Line;
        const int right_loss = local_vision.Right_Line;

        const int dominant_loss = std::max(left_loss, right_loss);
        const int loss_delta = left_loss - right_loss;

        const int preview_sign =
            (loss_delta > 0) ? 1 :
            (loss_delta < 0) ? -1 : 0;

        const bool preview_direction_consistent =
            (std::fabs(raw_steer_error) <= 2.0f) ||
            ((raw_steer_error > 0.0f) == (loss_delta > 0));

        const bool preview_candidate =
            !ring_now &&
            local_vision.Road_type == Normol &&
            local_vision.OFFLine <= 10 &&
            preview_sign != 0 &&
            dominant_loss >= car_config::kCurvePreviewLossStart &&
            std::abs(loss_delta) >= car_config::kCurvePreviewLossStart &&
            preview_direction_consistent;

        // 两条边重新稳定后，允许下一次弯道重新使用预判。
        if (!preview_candidate ||
            dominant_loss <= car_config::kCurvePreviewResetLoss) {
            preview_turn_sign = 0;
            preview_committed = false;
        } else {
            // 单侧丢线方向切换，视为新弯道。
            if (preview_turn_sign != preview_sign) {
                preview_turn_sign = preview_sign;
                preview_committed = false;
            }

            // 视觉已经明确进入弯道后，关闭预判。
            // 后续完全交给视觉误差 + 陀螺仪闭环，避免出弯仍被旧方向预判拖住。
            if (std::fabs(raw_steer_error) >=
                car_config::kCurvePreviewCommitRawError) {
                preview_committed = true;
            }
        }

        const bool preview_valid =
            preview_candidate &&
            !preview_committed;

        if (preview_valid) {
            const float loss_level =
                static_cast<float>(
                    dominant_loss -
                    car_config::kCurvePreviewLossStart + 1);

            const float magnitude = clamp_float(
                car_config::kCurvePreviewGyroBaseDps +
                loss_level * car_config::kCurvePreviewGyroPerLossDps,
                0.0f,
                car_config::kCurvePreviewGyroMaxDps);

            preview_gyro_bias = std::copysign(
                magnitude,
                static_cast<float>(loss_delta));
        }

        const bool preview_turn_active =
            std::fabs(preview_gyro_bias) > 0.01f;

        if (full_loss_non_ring) {
            control_raw_steer_error = 0.0f;

            raw_steer_initialized = false;
            last_control_raw_steer_error = 0.0f;

            last_steer_error = 0.0f;
            last_target_gyro_z = 0.0f;
            target_gyro_z = 0.0f;

            last_diff_speed = 0.0f;
            fallback_diff = 0.0f;
            max_diff = 0.0f;

            preview_turn_sign = 0;
            preview_committed = false;

            cascade_ctrl.reset_runtime_state();

            if (car_config::kFullLossStopLatchEnable) {
                full_loss_stop_latched = true;
            }
        } else if (ring_just_finished) {
            control_raw_steer_error = 0.0f;
            raw_steer_initialized = false;
            last_control_raw_steer_error = 0.0f;

            last_steer_error = 0.0f;
            last_target_gyro_z = 0.0f;
            target_gyro_z = 0.0f;

            last_diff_speed = 0.0f;
            cascade_ctrl.reset_runtime_state();
        }

        if (!raw_steer_initialized) {
            raw_steer_initialized = true;
            last_control_raw_steer_error = control_raw_steer_error;
        } else {
            float raw_delta = control_raw_steer_error - last_control_raw_steer_error;
            if (std::fabs(raw_delta) > car_config::kSteerRawJumpThreshold) {
                raw_delta = std::copysign(car_config::kSteerRawMaxStepPerFrame, raw_delta);
                control_raw_steer_error = last_control_raw_steer_error + raw_delta;
            }
            last_control_raw_steer_error = control_raw_steer_error;
        }

        // D. Visual outer loop first-order low-pass filter
        float steer_error = last_steer_error;
        if (ring_just_finished) {
            steer_error = control_raw_steer_error;
        } else {
            float filter_alpha = car_config::kSteerFilterAlpha;
            // S-curve fast flip enhancement
            const bool opposing_direction = (control_raw_steer_error > 0.0f && last_steer_error < 0.0f) ||
                                            (control_raw_steer_error < 0.0f && last_steer_error > 0.0f);
            if (opposing_direction && std::fabs(control_raw_steer_error) >= car_config::kSteerFastFlipRawThreshold) {
                float gap = std::fabs(control_raw_steer_error - last_steer_error);
                if (gap >= car_config::kSteerFastFlipGap) {
                    filter_alpha = car_config::kSteerFastFlipAlpha;
                }
            }
            float error_delta = control_raw_steer_error - last_steer_error;
            float max_step = car_config::kSteerFilterMaxStepPerFrame;
            if (opposing_direction) {
                max_step = car_config::kSteerFilterSignFlipMaxStepPerFrame;
            }
            error_delta = clamp_float(error_delta, -max_step, max_step);
            steer_error = last_steer_error + error_delta * filter_alpha;
        }
        dbg_steer_error = steer_error;

        // E. Visual derivative calculation, normalized by dt ratio for time invariance
        float diff_steer = steer_error - last_steer_error;
        float normalized_dt_ratio = visual_dt / 0.010f; // Nominal 10ms frame period
        float diff_steer_normalized = diff_steer / normalized_dt_ratio;
        dbg_diff_steer = diff_steer_normalized;

        // F. Continuous speed planning using raw error and error derivative
        const float abs_err = std::max(
            std::fabs(control_raw_steer_error),
            std::fabs(steer_error));

        // 用误差变化量提前识别入弯趋势
        const float preview = std::min(
            std::fabs(diff_steer_normalized),
            8.0f);

        const float curve_score = abs_err + 0.35f * preview;

        // 误差约 2 开始减速，误差约 8 降至完整弯道速度
        const float slow_start = 2.0f;
        const float full_curve = 8.0f;

        float u = (curve_score - slow_start) / (full_curve - slow_start);
        u = clamp_float(u, 0.0f, 1.0f);

        // ZD 为直道速度，WD 为弯道速度
        float speed_to_plan =
            ZD_Speed_Goal + (WD_Speed_Goal - ZD_Speed_Goal) * u;

        if (preview_turn_active) {
            speed_to_plan = std::min(
                speed_to_plan,
                car_config::kCurveSpeedGoal);
        }

        simple_curve_active = (u > 0.15f) || preview_turn_active;
        if (SystemData.Stop == 1) {
            speed_to_plan = 0.0f;
        }
        if (ring_now && speed_to_plan > car_config::kRingSpeedGoal) {
            speed_to_plan = car_config::kRingSpeedGoal;
        }
        if (bad_vision && local_vision.image_element_rings == 0 && speed_to_plan > car_config::kBadVisionSpeedGoal) {
            speed_to_plan = car_config::kBadVisionSpeedGoal;
        }
        planned_speed_goal = speed_to_plan;

        // H. Visual outer loop PID calculation for target_gyro_z
        float visual_pd_out = 0.0f;
        float delta_target = 0.0f;
        const float DEAD_BAND = (local_vision.Road_type == Straight || local_vision.Road_type == Normol) ? 
            (float)car_config::kSteerDeadBandStraight : (float)car_config::kSteerDeadBandCurve;

        if (std::fabs(steer_error) <= DEAD_BAND &&
            !preview_turn_active) {
            delta_target = -last_target_gyro_z;
            delta_target = clamp_float(
                delta_target,
                -car_config::kTargetGyroDeadbandReturnDeltaDps,
                car_config::kTargetGyroDeadbandReturnDeltaDps);
            target_gyro_z = last_target_gyro_z + delta_target;
            if (std::fabs(target_gyro_z) < car_config::kGyroDeadBandDps) {
                target_gyro_z = 0.0f;
                delta_target = -last_target_gyro_z;
            }
            last_target_gyro_z = target_gyro_z;
            last_steer_error = steer_error;
        } else {
            visual_pd_out = steer_error * STEER_KP +
                            diff_steer_normalized * STEER_KD +
                            preview_gyro_bias;
            visual_pd_out = clamp_float(
                visual_pd_out,
                -car_config::kVisualGyroLimitDps,
                car_config::kVisualGyroLimitDps);

            if ((local_vision.Road_type == LeftCirque || local_vision.Road_type == RightCirque) &&
                 std::fabs(visual_pd_out) > car_config::kRingSceneGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kRingSceneGyroLimitDps, visual_pd_out);
            }
            if (ring_entry_phase && std::fabs(visual_pd_out) > car_config::kRingEntryGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kRingEntryGyroLimitDps, visual_pd_out);
            }
            if (bad_vision && !ring_now && std::fabs(visual_pd_out) > car_config::kBadVisionGyroLimitDps) {
                visual_pd_out = std::copysign(car_config::kBadVisionGyroLimitDps, visual_pd_out);
            }

            float max_delta_target = car_config::kTargetGyroMaxDeltaDps;
            if ((visual_pd_out > 0.0f && last_target_gyro_z < 0.0f) ||
                (visual_pd_out < 0.0f && last_target_gyro_z > 0.0f)) {
                max_delta_target = car_config::kTargetGyroSignFlipMaxDeltaDps;
            }
            float max_delta_target_scaled = max_delta_target * normalized_dt_ratio;
            delta_target = visual_pd_out - last_target_gyro_z;
            delta_target = clamp_float(delta_target, -max_delta_target_scaled, max_delta_target_scaled);
            target_gyro_z = last_target_gyro_z + delta_target;
            last_target_gyro_z = target_gyro_z;
            last_steer_error = steer_error;
        }
        dbg_target_gyro_z = target_gyro_z;
        dbg_visual_pd_out = visual_pd_out;
        dbg_target_delta = delta_target;

        // Compute basic differential limit and single global minimum wheel speed.
        max_diff = speed_to_plan * MAX_DIFF_RATIO;
        if (ring_now) {
            float ring_max_diff = ring_entry_phase ? car_config::kRingEntryMaxDiff : car_config::kRingMaxDiff;
            if (max_diff > ring_max_diff) {
                max_diff = ring_max_diff;
            }
        }
        if (bad_vision && !ring_now && max_diff > car_config::kBadVisionMaxDiff) {
            max_diff = car_config::kBadVisionMaxDiff;
        }
        effective_min_wheel = MIN_SPEED;

        // Under fallback, pre-compute pure visual diff_speed
        if (imu_in_fallback) {
            if (std::fabs(steer_error) <= DEAD_BAND &&
            !preview_turn_active) {
                fallback_diff = 0.0f;
            } else {
                fallback_diff = steer_error * STEER_KP + diff_steer_normalized * STEER_KD;
                fallback_diff = clamp_float(fallback_diff, -car_config::kFallbackDiffLimit, car_config::kFallbackDiffLimit);
            }
        }
    }

    if (full_loss_stop_latched) {
        planned_speed_goal = 0.0f;

        Speed_Goal = 0.0f;
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;

        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_last_erro_R = 0.0f;

        dbg_diff_speed = 0.0f;
        dbg_target_gyro_z = 0.0f;
        dbg_visual_pd_out = 0.0f;
        dbg_cmd_target_gyro_z = 0.0f;
        dbg_diff_before_limit = 0.0f;
        dbg_max_diff = 0.0f;
        dbg_diff_limited = 0;

        return;
    }

    // --- 2. Dual-rate architecture - inner loop (Runs unconditionally every 5ms) ---
    Mpu6050Data imu_data;
    bool imu_ok = imu.read_processed_data(imu_data);

    if (imu_ok) {
        consecutive_imu_successes++;
        consecutive_imu_failures = 0;

        if (imu_in_fallback) {
            if (consecutive_imu_successes >= car_config::kImuRecoveryMinFrames) {
                imu_in_fallback = false;
                cascade_ctrl.initialize_recovery_state(imu_data.gyro_z, last_diff_speed);
            }
        }
    } else {
        consecutive_imu_failures++;
        consecutive_imu_successes = 0;

        if (consecutive_imu_failures > car_config::kImuMaxAllowedMissedFrames) {
            imu_in_fallback = true;
        }
    }

    float diff_speed = 0.0f;

    if (!imu_in_fallback) {
        dbg_imu_ok = 1;

        if (consecutive_imu_failures > 0) {
            // Transient failure: freeze control action, hold previous frame
            diff_speed = last_diff_speed;
        } else {
            // Normal: execute gyro rate PID loop at 5ms (0.005s)
            float cmd_target_gyro_z = target_gyro_z;

            diff_speed = cascade_ctrl.calc_diff_speed_bypass_angle(cmd_target_gyro_z, imu_data, inner_dt);
            dbg_cmd_target_gyro_z = cmd_target_gyro_z;
            dbg_real_gyro_z = imu_data.gyro_z;
            dbg_filtered_gyro_z = cascade_ctrl.last_filtered_gyro_z();
            dbg_gyro_error = cascade_ctrl.gyro_pid.err;
            dbg_diff_before_limit = diff_speed;
        }
    } else {
        // Fallback: execute pure visual closed loop using pre-computed fallback_diff
        dbg_imu_ok = 0;
        dbg_real_gyro_z = 0.0f;
        dbg_filtered_gyro_z = 0.0f;
        dbg_gyro_error = 0.0f;
        dbg_cmd_target_gyro_z = 0.0f;
        dbg_target_delta = 0.0f;
        dbg_target_gyro_z = 0.0f;
        last_target_gyro_z = 0.0f;

        if (Speed_Goal > car_config::kImuFallbackSpeedGoal) {
            Speed_Goal = car_config::kImuFallbackSpeedGoal;
        }

        diff_speed = fallback_diff;
        dbg_visual_pd_out = fallback_diff;
        dbg_diff_before_limit = fallback_diff;
    }

    // --- 3. Unified clamping and speed distribution ---
    float effective_max_diff = max_diff;

    const bool need_curve_turn_authority =
        simple_curve_active &&
        std::fabs(target_gyro_z) >= car_config::kCurveMinDiffGyroDps;

    if (need_curve_turn_authority) {
        effective_max_diff = std::max(
            effective_max_diff,
            car_config::kCurveMinDiff);
    }

    if (Speed_Goal > 0.0f) {
        const float current_diff_ratio = simple_curve_active ? MAX_DIFF_RATIO : 1.0f;
        float ramp_limited_max_diff =
            Speed_Goal * current_diff_ratio;

        if (need_curve_turn_authority) {
            ramp_limited_max_diff = std::max(
                ramp_limited_max_diff,
                car_config::kCurveMinDiff);
        }

        if (effective_max_diff > ramp_limited_max_diff) {
            effective_max_diff = ramp_limited_max_diff;
        }
    } else {
        effective_max_diff = 0.0f;
    }

    dbg_max_diff = effective_max_diff;
    dbg_diff_limited = 0;
    if (diff_speed > effective_max_diff) {
        diff_speed = effective_max_diff;
        dbg_diff_limited = 1;
    }
    if (diff_speed < -effective_max_diff) {
        diff_speed = -effective_max_diff;
        dbg_diff_limited = 1;
    }

    // Keep turn authority tied to the current ramped speed so startup cannot command a reverse inner wheel.

    dbg_diff_speed = diff_speed;
    last_diff_speed = diff_speed;

    // Distribute speed targets to left and right wheels using the matching effective_min_wheel
    distribute_wheel_speeds(Speed_Goal, diff_speed, effective_min_wheel);

    const bool one_wheel_released =
        (Speed_Goal_l <= 0.02f) ||
        (Speed_Goal_r <= 0.02f);

    if (simple_curve_active &&
        one_wheel_released &&
        Speed_Goal > 0.0f) {

        Speed_Goal_l = std::min(
            Speed_Goal_l,
            car_config::kCurveOuterWheelMax);

        Speed_Goal_r = std::min(
            Speed_Goal_r,
            car_config::kCurveOuterWheelMax);

        // 限速后重新记录速度误差，保证日志正确
        dbg_speed_err_l = Speed_Goal_l - speed_encoder_L;
        dbg_speed_err_r = Speed_Goal_r - speed_encoder_R;
    }

    if (car_config::kSteerDebugPrintEnable) {
        static int debug_print_count = 0;
        debug_print_count++;
        if (debug_print_count >= car_config::kSteerDebugPrintDivider) {
            debug_print_count = 0;
            printf("[steer] det:%d mid:%d raw:%.1f filt:%.1f dE:%.1f pd:%.1f tgt:%.1f cmd:%.1f gz:%.1f gf:%.1f ge:%.1f dT:%.1f d0:%.1f diff:%.1f cap:%d max:%.1f goal:%.1f L:%.1f R:%.1f encL:%.1f encR:%.1f rawEncL:%.2f rawEncR:%.2f pwmL:%.0f pwmR:%.0f eL:%.1f eR:%.1f dt:%.3f imu:%d db:%d off:%d ll:%d rl:%d curve:%d\n",
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
                   speed_encoder_raw_L,
                   speed_encoder_raw_R,
                   speed_out_l,
                   speed_out_r,
                   dbg_speed_err_l,
                   dbg_speed_err_r,
                   dbg_dt,
                   dbg_imu_ok,
                   dbg_deadband_active,
                   ImageStatus.OFFLine,
                   ImageStatus.Left_Line,
                   ImageStatus.Right_Line,
                   simple_curve_active ? 1 : 0);
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
void Car_Speed_Contrl(void)
{


    //直线加�?
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




    
    if(SystemData.Stop==1)  //停车（不再因为车�?斑马线识别停车）
    {
        Speed_Goal=0;
        Speed_Goal_l=0;
        Speed_Goal_r=0;
    }

}



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
    if (vision_age_ms > kVisionFreshTimeoutMs) {
        static int vision_guard_warn_count = 0;
        Speed_Goal = 0.0f;
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_last_erro_R = 0.0f;
        if (motor_L && motor_R) {
            motor_Duty_L(0);
            motor_Duty_R(0);
        }
        if (vision_guard_warn_count < 8) {
            ++vision_guard_warn_count;
            printf("[VISION_GUARD] stale frame age=%lldms; stop output, lock out (exit required)\n",
                   vision_age_ms);
        }
        exit_flag.store(true, std::memory_order_relaxed);
        return;
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

    static float ramped_speed_goal = 0.0f;
    static auto last_speed_ramp_time = std::chrono::steady_clock::now();
    auto speed_ramp_now = std::chrono::steady_clock::now();
    std::chrono::duration<float> speed_ramp_elapsed = speed_ramp_now - last_speed_ramp_time;
    float speed_ramp_dt = speed_ramp_elapsed.count();
    last_speed_ramp_time = speed_ramp_now;
    if (speed_ramp_dt <= 0.0001f || speed_ramp_dt > 0.05f) {
        speed_ramp_dt = 0.005f;
    }

    float inner_dt = speed_ramp_dt;
    if (inner_dt < 0.003f) inner_dt = 0.003f;
    if (inner_dt > 0.012f) inner_dt = 0.012f;

    Car_distance(inner_dt);

    if (SystemData.Stop == 1) {
        planned_speed_goal = 0.0f;
    }

    const float desired_speed_goal = planned_speed_goal;
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

    speed_dif_cascade(inner_dt);     

    // 外环弯道/丢线保护可能�?speed_dif_cascade() 内把目标速度直接压低�?
    // 同步斜坡状态，避免保护解除时从旧的高速记忆值瞬间跳回去�?
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

// 差速速度调整函数
// void Speed_Adjust_By_Offset(float offset)
// {
//     float KP = 0.3f;  // 方向调整比例系数，可调试

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

    if (motor_R && motor_L) {
        motor_Duty_R(0);    // 右电机停�?
        motor_Duty_L(0);    // 左电机停�?
    }
    // [T01修正] DIR极性反转后，停车时拉到反转方向的无效端（LOW=反转，与duty=0配对无输出）
    if (motor_R_Dir) motor_R_Dir->gpio_level_set(GPIO_LOW);
    if (motor_L_Dir) motor_L_Dir->gpio_level_set(GPIO_LOW);

    if (motor_R && motor_L) {
        motor_R->atim_pwm_set_active_low_duty(0);
        motor_L->atim_pwm_set_active_low_duty(0);
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
