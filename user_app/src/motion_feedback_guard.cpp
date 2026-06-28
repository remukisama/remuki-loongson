#include "motor.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

extern float Speed_Goal_l;
extern float Speed_Goal_r;
extern float speed_encoder_L;
extern float speed_encoder_R;
extern float speed_out_l;
extern float speed_out_r;
extern void emergency_stop();

namespace {
constexpr float kMinGoalMs = 0.30f;
constexpr float kMinDrivePwm = 4200.0f;
constexpr float kMaxStillSpeedMs = 0.05f;
constexpr int kPollMs = 20;
constexpr int kTripSamples = 35;  // 700 ms

void motion_feedback_guard() {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int bad_l = 0;
    int bad_r = 0;
    while (!exit_flag.load(std::memory_order_relaxed)) {
        const bool no_motion_l = Speed_Goal_l > kMinGoalMs && speed_out_l > kMinDrivePwm &&
                                 std::fabs(speed_encoder_L) < kMaxStillSpeedMs;
        const bool no_motion_r = Speed_Goal_r > kMinGoalMs && speed_out_r > kMinDrivePwm &&
                                 std::fabs(speed_encoder_R) < kMaxStillSpeedMs;
        bad_l = no_motion_l ? bad_l + 1 : 0;
        bad_r = no_motion_r ? bad_r + 1 : 0;
        if (bad_l >= kTripSamples || bad_r >= kTripSamples) {
            std::printf("[MOTION_GUARD] commanded wheel has no encoder feedback; stop and exit\n");
            exit_flag.store(true, std::memory_order_relaxed);
            emergency_stop();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
}

struct MotionFeedbackGuard {
    MotionFeedbackGuard() {
        std::thread(motion_feedback_guard).detach();
    }
};

MotionFeedbackGuard g_motion_feedback_guard;
}  // namespace
