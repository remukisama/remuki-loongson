// Runtime-safety overlay for my_motor.cpp.
//
// The main CMake target compiles this translation unit instead of compiling
// my_motor.cpp directly.  It keeps the existing tuned control law intact while
// adding three fail-closed behaviors that are also needed by the tuning demos:
//   1) A control reset clears the dual-rate controller's file-local runtime state.
//   2) Speed_BiHuan refuses to run if Speed_decision()/MPU6050 init failed.
//   3) Direct motor commands used by T01/T05/T11 establish and maintain the
//      gpio_manager heartbeat before non-zero PWM is allowed.

#define Speed_BiHuan lq_speed_bihuan_impl
#define Speed_decision lq_speed_decision_impl
#define motor_Duty_L lq_motor_Duty_L_impl
#define motor_Duty_R lq_motor_Duty_R_impl
#define control_reset_vision_health lq_control_reset_vision_health_impl
#include "my_motor.cpp"
#undef control_reset_vision_health
#undef motor_Duty_R
#undef motor_Duty_L
#undef Speed_decision
#undef Speed_BiHuan

namespace {
std::atomic<bool> g_runtime_reset_requested{false};
std::atomic<bool> g_speed_controller_ready{false};
std::atomic<bool> g_direct_drive_watchdog_ready{false};

void apply_zero_motor_output()
{
    Speed_Goal = 0.0f;
    Speed_Goal_l = 0.0f;
    Speed_Goal_r = 0.0f;
    speed_out_l = 0.0f;
    speed_out_r = 0.0f;
    speed_last_erro_L = 0.0f;
    speed_last_erro_R = 0.0f;
    lq_motor_Duty_L_impl(0);
    lq_motor_Duty_R_impl(0);
}

bool direct_drive_watchdog_ready()
{
    if (!g_direct_drive_watchdog_ready.load(std::memory_order_acquire)) {
        if (!gpio_protect_heartbeat_require_ready(1500)) {
            std::printf("[DIRECT_DRIVE_GUARD] gpio_manager watchdog is not ready; non-zero PWM refused.\n");
            return false;
        }
        g_direct_drive_watchdog_ready.store(true, std::memory_order_release);
    }

    gpio_protect_heartbeat_pulse();
    if (!gpio_protect_heartbeat_is_wd_alive(car_config::kWatchdogHeartbeatStallTimeoutMs)) {
        g_direct_drive_watchdog_ready.store(false, std::memory_order_release);
        std::printf("[DIRECT_DRIVE_GUARD] watchdog heartbeat stalled; non-zero PWM refused.\n");
        return false;
    }
    return true;
}

void reset_control_runtime_once()
{
    // The original controller stores event, filter, target-rate, scene-hold,
    // IMU fallback and speed-ramp state in function-local statics.  Its existing
    // SystemData.Stop branch is the canonical reset path, so use it once with
    // zero output instead of duplicating or retuning the control law here.
    const int saved_stop = SystemData.Stop;
    planned_speed_goal = 0.0f;
    apply_zero_motor_output();

    SystemData.Stop = 1;
    speed_dif_cascade(0.005f);
    SystemData.Stop = saved_stop;

    gpio_protect_heartbeat_pulse();
}
} // namespace

bool Speed_decision(void)
{
    g_speed_controller_ready.store(false, std::memory_order_release);
    const bool ok = lq_speed_decision_impl();
    g_speed_controller_ready.store(ok, std::memory_order_release);
    if (!ok) {
        std::printf("[CONTROL_GUARD] Speed_decision/MPU6050 initialization failed; closed-loop drive is disabled.\n");
    }
    return ok;
}

void control_reset_vision_health(void)
{
    lq_control_reset_vision_health_impl();
    g_runtime_reset_requested.store(true, std::memory_order_release);
}

void Speed_BiHuan(void)
{
    if (!g_speed_controller_ready.load(std::memory_order_acquire)) {
        static int warn_count = 0;
        apply_zero_motor_output();
        gpio_protect_heartbeat_pulse();
        if (warn_count < 3) {
            ++warn_count;
            std::printf("[CONTROL_GUARD] closed-loop tick blocked because controller initialization is not ready.\n");
        }
        exit_flag.store(true, std::memory_order_relaxed);
        return;
    }

    if (g_runtime_reset_requested.exchange(false, std::memory_order_acq_rel)) {
        reset_control_runtime_once();
        return;
    }

    lq_speed_bihuan_impl();
}

void motor_Duty_L(int duty)
{
    if (duty != 0 && !direct_drive_watchdog_ready()) {
        lq_motor_Duty_L_impl(0);
        return;
    }
    if (duty != 0) gpio_protect_heartbeat_pulse();
    lq_motor_Duty_L_impl(duty);
}

void motor_Duty_R(int duty)
{
    if (duty != 0 && !direct_drive_watchdog_ready()) {
        lq_motor_Duty_R_impl(0);
        return;
    }
    if (duty != 0) gpio_protect_heartbeat_pulse();
    lq_motor_Duty_R_impl(duty);
}
