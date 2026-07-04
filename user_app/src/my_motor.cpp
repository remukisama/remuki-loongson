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

    motor_R->atim_pwm_set_active_low_duty(car_config::kMotorActiveLowStopDuty);
    motor_L->atim_pwm_set_active_low_duty(car_config::kMotorActiveLowStopDuty);

    printf("R:OK!");

    motor_R_Dir->gpio_level_set(GPIO_HIGH);
    motor_L_Dir->gpio_level_set(GPIO_HIGH);
    motor_R->atim_pwm_enable();
    motor_L->atim_pwm_enable();
}

void motor_Duty_R(int duty)
{
    if (!motor_R || !motor_R_Dir) return;  // 电机未初始化时安全跳�?
    if (duty == 0)
    {
        motor_R->atim_pwm_set_active_low_duty(car_config::kMotorActiveLowStopDuty);
        motor_R_Dir->gpio_level_set(GPIO_HIGH);
    }
    else if (duty < 0)
    {
        // 反转：IN1=LOW + IN2 high-active  反转PWM占空比（对称力矩�?
        const int magnitude = std::min(-duty, car_config::kMotorPwmFullScale);
        motor_R_Dir->gpio_level_set(GPIO_LOW);
        motor_R->atim_pwm_set_duty(magnitude);
    }
    else
    {
        // 正转：IN1=HIGH + IN2 active-low PWM
        const int magnitude = std::min(duty, car_config::kMotorPwmFullScale);
        motor_R_Dir->gpio_level_set(GPIO_HIGH);
        motor_R->atim_pwm_set_active_low_duty(magnitude);
    }
}

void motor_Duty_L(int duty)
{
    if (!motor_L || !motor_L_Dir) return;  // 电机未初始化时安全跳�?
    if (duty == 0)
    {
        motor_L->atim_pwm_set_active_low_duty(car_config::kMotorActiveLowStopDuty);
        motor_L_Dir->gpio_level_set(GPIO_HIGH);
    }
    else if (duty < 0)
    {
        // 反转：IN1=LOW + IN2 high-active  反转PWM占空比（对称力矩�?
        const int magnitude = std::min(-duty, car_config::kMotorPwmFullScale);
        motor_L_Dir->gpio_level_set(GPIO_LOW);
        motor_L->atim_pwm_set_duty(magnitude);
    }
    else
    {
        // 正转：IN1=HIGH + IN2 active-low PWM
        const int magnitude = std::min(duty, car_config::kMotorPwmFullScale);
        motor_L_Dir->gpio_level_set(GPIO_HIGH);
        motor_L->atim_pwm_set_active_low_duty(magnitude);
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
    // Incremental PID wheel-speed loop gains.
    speed_P_l = car_config::kSpeedPidP;
    speed_I_l = car_config::kSpeedPidI;
    speed_D_l = car_config::kSpeedPidD;

    speed_P_r = car_config::kSpeedPidP;
    speed_I_r = car_config::kSpeedPidI;
    speed_D_r = car_config::kSpeedPidD;

    MAX_PWM = car_config::kMotorMaxPwm;

    speed_out_l = 0;
    speed_out_r = 0;

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
    static float speed_prev_erro_L = 0.0f;

    if (Speed_Goal_l <= 0.02f) {
        speed_out_l = 0.0f;
        speed_last_erro_L = 0.0f;
        speed_prev_erro_L = 0.0f;
        return;
    }

    const float delta =
        speed_P_l * (speed_erro_L - speed_last_erro_L) +
        speed_I_l * speed_erro_L +
        speed_D_l * (speed_erro_L - 2.0f * speed_last_erro_L + speed_prev_erro_L);

    speed_out_l += delta;
    speed_prev_erro_L = speed_last_erro_L;
    speed_last_erro_L = speed_erro_L;

    if (speed_out_l > MAX_PWM) speed_out_l = MAX_PWM;
    if (speed_out_l < 0.0f) speed_out_l = 0.0f;
}



void Speed_PID_R(void)
{
    speed_erro_R = Speed_Goal_r - speed_encoder_R;
    static float speed_prev_erro_R = 0.0f;

    if (Speed_Goal_r <= 0.02f) {
        speed_out_r = 0.0f;
        speed_last_erro_R = 0.0f;
        speed_prev_erro_R = 0.0f;
        return;
    }

    const float delta =
        speed_P_r * (speed_erro_R - speed_last_erro_R) +
        speed_I_r * speed_erro_R +
        speed_D_r * (speed_erro_R - 2.0f * speed_last_erro_R + speed_prev_erro_R);

    speed_out_r += delta;
    speed_prev_erro_R = speed_last_erro_R;
    speed_last_erro_R = speed_erro_R;

    if (speed_out_r > MAX_PWM) speed_out_r = MAX_PWM;
    if (speed_out_r < 0.0f) speed_out_r = 0.0f;
}



// ===============================================================================
// [5] Basic cascade control
// ===============================================================================

//差�?
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
    static auto last_visual_frame_time_ms = 0LL;
    static uint32_t last_processed_seq = 0;
    static float last_steer_error = 0.0f;
    static float steer_integral = 0.0f;
    static float target_gyro_z = 0.0f;
    static float raw_steer_error = 0.0f;

    if (SystemData.Stop == 1) {
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        last_steer_error = 0.0f;
        steer_integral = 0.0f;
        target_gyro_z = 0.0f;
        raw_steer_error = 0.0f;
        planned_speed_goal = 0.0f;
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
        return;
    }

    const uint32_t current_seq = g_control_vision_frame_seq.load(std::memory_order_acquire);
    const bool new_frame_arrived = (current_seq != last_processed_seq);

    if (new_frame_arrived) {
        VisionSnapshot local_vision;
        {
            std::lock_guard<std::mutex> lock(g_vision_mutex);
            local_vision = g_vision_snapshot;
        }
        last_processed_seq = current_seq;

        const long long current_frame_time_ms = g_control_last_vision_frame_ms.load(std::memory_order_acquire);
        float visual_dt = (last_visual_frame_time_ms > 0) ?
            (float)(current_frame_time_ms - last_visual_frame_time_ms) / 1000.0f : 0.010f;
        last_visual_frame_time_ms = current_frame_time_ms;
        if (visual_dt <= 0.0001f || visual_dt > 0.1f) {
            visual_dt = 0.010f;
        }
        dbg_dt = visual_dt;

        raw_steer_error = local_vision.MiddleLine - local_vision.Det_True;
        float steer_error = raw_steer_error;
        if (std::fabs(steer_error) <= (float)car_config::kSteerDeadBand) {
            steer_error = 0.0f;
            dbg_deadband_active = 1;
        } else {
            dbg_deadband_active = 0;
        }

        steer_integral = clamp_float(
            steer_integral + steer_error * visual_dt,
            -car_config::kSteerIntegralLimit,
            car_config::kSteerIntegralLimit);
        const float diff_steer = steer_error - last_steer_error;

        float visual_pid_out =
            car_config::kSteerKp * steer_error +
            car_config::kSteerKi * steer_integral +
            car_config::kSteerKd * diff_steer;
        visual_pid_out = clamp_float(
            visual_pid_out,
            -car_config::kSteerOutputLimitDps,
            car_config::kSteerOutputLimitDps);

        target_gyro_z = visual_pid_out;
        last_steer_error = steer_error;
        planned_speed_goal = (SystemData.Stop == 1) ? 0.0f : car_config::kBaseSpeedGoal;

        dbg_raw_steer_error = raw_steer_error;
        dbg_steer_error = steer_error;
        dbg_diff_steer = diff_steer;
        dbg_visual_pd_out = visual_pid_out;
        dbg_target_gyro_z = target_gyro_z;
        dbg_target_delta = 0.0f;
        dbg_max_diff = car_config::kGyroPidOutputLimit;
    }

    Mpu6050Data imu_data;
    if (!imu.read_processed_data(imu_data)) {
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        speed_out_l = 0.0f;
        speed_out_r = 0.0f;
        target_gyro_z = 0.0f;
        cascade_ctrl.reset_runtime_state();
        dbg_imu_ok = 0;
        dbg_diff_speed = 0.0f;
        dbg_diff_before_limit = 0.0f;
        return;
    }

    dbg_imu_ok = 1;
    const float diff_speed = cascade_ctrl.calc_diff_speed_bypass_angle(target_gyro_z, imu_data, inner_dt);

    dbg_cmd_target_gyro_z = target_gyro_z;
    dbg_real_gyro_z = imu_data.gyro_z;
    dbg_filtered_gyro_z = cascade_ctrl.last_filtered_gyro_z();
    dbg_gyro_error = cascade_ctrl.gyro_pid.err;
    dbg_diff_before_limit = diff_speed;
    dbg_diff_speed = diff_speed;
    dbg_diff_limited = 0;

    distribute_wheel_speeds(Speed_Goal, diff_speed, 0.0f);

    if (car_config::kSteerDebugPrintEnable) {
        static int debug_print_count = 0;
        debug_print_count++;
        if (debug_print_count >= car_config::kSteerDebugPrintDivider) {
            debug_print_count = 0;
            printf("[steer] det:%d mid:%d raw:%.1f err:%.1f dE:%.1f tgt:%.1f gz:%.1f ge:%.1f diff:%.2f goal:%.2f L:%.2f R:%.2f encL:%.2f encR:%.2f pwmL:%.0f pwmR:%.0f dt:%.3f imu:%d db:%d off:%d ll:%d rl:%d\n",
                   ImageStatus.Det_True,
                   ImageStatus.MiddleLine,
                   raw_steer_error,
                   dbg_steer_error,
                   dbg_diff_steer,
                   dbg_target_gyro_z,
                   dbg_real_gyro_z,
                   dbg_gyro_error,
                   dbg_diff_speed,
                   Speed_Goal,
                   Speed_Goal_l,
                   Speed_Goal_r,
                   speed_encoder_L,
                   speed_encoder_R,
                   speed_out_l,
                   speed_out_r,
                   dbg_dt,
                   dbg_imu_ok,
                   dbg_deadband_active,
                   ImageStatus.OFFLine,
                   ImageStatus.Left_Line,
                   ImageStatus.Right_Line);
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
    Speed_Goal = (planned_speed_goal > 0.0f) ? planned_speed_goal : 0.0f;
    ramped_speed_goal = Speed_Goal;

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
    // DRV8701P emergency stop: IN1=HIGH and IN2=HIGH, keep PWM enabled so the bridge is actively braked.
    if (motor_R_Dir) motor_R_Dir->gpio_level_set(GPIO_HIGH);
    if (motor_L_Dir) motor_L_Dir->gpio_level_set(GPIO_HIGH);

    if (motor_R && motor_L) {
        motor_R->atim_pwm_set_active_low_duty(car_config::kMotorActiveLowStopDuty);
        motor_L->atim_pwm_set_active_low_duty(car_config::kMotorActiveLowStopDuty);
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
