#include "motor.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

namespace {
constexpr float kMinGoalMs = 0.30f;
constexpr float kMinDrivePwm = 4200.0f;
constexpr float kMaxStillSpeedMs = 0.05f;
constexpr float kTripTimeSec = 0.70f;
constexpr float kMaxTickDtSec = 0.05f;

std::chrono::steady_clock::time_point g_last_tick;
float g_no_motion_l_sec = 0.0f;
float g_no_motion_r_sec = 0.0f;

void reset_state() {
    g_last_tick = std::chrono::steady_clock::time_point();
    g_no_motion_l_sec = 0.0f;
    g_no_motion_r_sec = 0.0f;
}
}  // namespace

void motion_feedback_guard_reset() {
    reset_state();
}

void motion_feedback_guard_tick() {
    const auto now = std::chrono::steady_clock::now();
    float dt = 0.005f;
    if (g_last_tick != std::chrono::steady_clock::time_point()) {
        dt = std::chrono::duration<float>(now - g_last_tick).count();
        if (dt <= 0.0f || dt > kMaxTickDtSec) {
            dt = 0.005f;
        }
    }
    g_last_tick = now;

    const bool no_motion_l = Speed_Goal_l > kMinGoalMs && speed_out_l > kMinDrivePwm &&
                             std::fabs(speed_encoder_L) < kMaxStillSpeedMs;
    const bool no_motion_r = Speed_Goal_r > kMinGoalMs && speed_out_r > kMinDrivePwm &&
                             std::fabs(speed_encoder_R) < kMaxStillSpeedMs;

    g_no_motion_l_sec = no_motion_l ? g_no_motion_l_sec + dt : 0.0f;
    g_no_motion_r_sec = no_motion_r ? g_no_motion_r_sec + dt : 0.0f;

    if (g_no_motion_l_sec < kTripTimeSec && g_no_motion_r_sec < kTripTimeSec) {
        return;
    }

    std::printf("[MOTION_GUARD] commanded wheel has no encoder feedback; "
                "L{goal=%.3f,pwm=%.1f,enc=%.3f,hold=%.3f} "
                "R{goal=%.3f,pwm=%.1f,enc=%.3f,hold=%.3f}; stop and exit\n",
                Speed_Goal_l, speed_out_l, speed_encoder_L, g_no_motion_l_sec,
                Speed_Goal_r, speed_out_r, speed_encoder_R, g_no_motion_r_sec);
    reset_state();
    exit_flag.store(true, std::memory_order_relaxed);
    emergency_stop();
}
