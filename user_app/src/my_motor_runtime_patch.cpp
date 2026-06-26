// Runtime-safety overlay for my_motor.cpp.
//
// This file is intentionally compiled instead of my_motor.cpp (see main/CMakeLists.txt).
// It includes the existing tuned controller once, then adds fail-closed lifecycle guards
// without retuning the vision, gyro, differential-speed or wheel-speed control law.

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
constexpr long long kControlOverrunStopMs = 50;
constexpr int kControlOverrunConsecutiveLimit = 3;

std::atomic<bool> g_runtime_reset_requested{false};
std::atomic<bool> g_speed_controller_ready{false};
std::atomic<bool> g_direct_drive_watchdog_ready{false};
std::atomic<long long> g_last_control_tick_ms{0};
std::atomic<int> g_control_overrun_streak{0};

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
    // The existing SystemData.Stop branch is the source of truth for clearing the
    // controller's function-local filter, scene-hold, fallback and PID state.
    const int saved_stop = SystemData.Stop;
    planned_speed_goal = 0.0f;
    apply_zero_motor_output();

    SystemData.Stop = 1;
    speed_dif_cascade(0.005f);
    SystemData.Stop = saved_stop;

    g_last_control_tick_ms.store(0, std::memory_order_release);
    g_control_overrun_streak.store(0, std::memory_order_release);
    gpio_protect_heartbeat_pulse();
}

bool control_timing_healthy()
{
    const long long now = control_now_ms();
    const long long previous = g_last_control_tick_ms.exchange(now, std::memory_order_acq_rel);
    if (previous <= 0) {
        return true;
    }

    const long long gap_ms = now - previous;
    if (gap_ms <= kControlOverrunStopMs) {
        g_control_overrun_streak.store(0, std::memory_order_release);
        return true;
    }

    const int streak = g_control_overrun_streak.fetch_add(1, std::memory_order_acq_rel) + 1;
    if (streak <= kControlOverrunConsecutiveLimit) {
        std::printf("[CONTROL_TIMING] delayed control tick gap=%lldms streak=%d/%d\n",
                    gap_ms, streak, kControlOverrunConsecutiveLimit);
    }

    if (streak < kControlOverrunConsecutiveLimit) {
        return true;
    }

    std::printf("[CONTROL_TIMING_GUARD] repeated control overruns; stop output and exit.\n");
    return false;
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
    // Keep the previous generation so the first frame after a reset can never be
    // mistaken for an already-consumed frame by speed_dif_cascade().
    const uint32_t previous_seq = g_control_vision_frame_seq.load(std::memory_order_acquire);
    lq_control_reset_vision_health_impl();
    g_control_vision_frame_seq.store(previous_seq + 1U, std::memory_order_release);
    g_control_last_vision_frame_ms.store(0, std::memory_order_release);

    g_runtime_reset_requested.store(true, std::memory_order_release);
    g_last_control_tick_ms.store(0, std::memory_order_release);
    g_control_overrun_streak.store(0, std::memory_order_release);
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

    if (!control_timing_healthy()) {
        apply_zero_motor_output();
        gpio_protect_heartbeat_pulse();
        exit_flag.store(true, std::memory_order_relaxed);
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
