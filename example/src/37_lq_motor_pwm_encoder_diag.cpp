#include "lq_all_demo.hpp"
#include "user_app.hpp"
#include "lq_udp_client.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

namespace {
constexpr int kHoldMs = 900;
constexpr int kWarmupMs = 250;
constexpr int kSamplePeriodMs = 50;
constexpr int kSettleMs = 250;
constexpr float kMotionThresholdMS = 0.03f;
constexpr float kOtherWheelThresholdMS = 0.06f;
constexpr float kTrendMarginMS = 0.03f;

struct StepResult {
    char wheel;
    int duty;
    float avg_l;
    float avg_r;
    float peak_abs_l;
    float peak_abs_r;
    float raw_last_l;
    float raw_last_r;
    int samples;
    int invalid_l;
    int invalid_r;
};

struct EncoderSample {
    uint32_t period;
    float rps;
    float ms;
    int dir;
};

bool should_exit()
{
    return exit_flag.load(std::memory_order_relaxed);
}

void sleep_with_heartbeat(int ms)
{
    int remaining = ms;
    while (remaining > 0 && !should_exit()) {
        const int chunk = std::min(remaining, car_config::kGpioProtectHeartbeatIntervalMs);
        gpio_protect_heartbeat_pulse();
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = chunk * 1000;
        select(0, nullptr, nullptr, nullptr, &tv);
        remaining -= chunk;
    }
}

void stop_motors_and_settle()
{
    motor_Duty_L(0);
    motor_Duty_R(0);
    gpio_protect_heartbeat_pulse();
    sleep_with_heartbeat(kSettleMs);
    encoder_reset_state();
}

void apply_single_wheel_pwm(char wheel, int duty)
{
    if (wheel == 'L') {
        motor_Duty_R(0);
        motor_Duty_L(duty);
    } else {
        motor_Duty_L(0);
        motor_Duty_R(duty);
    }
}

float main_avg(const StepResult &r)
{
    return r.wheel == 'L' ? r.avg_l : r.avg_r;
}

float other_avg(const StepResult &r)
{
    return r.wheel == 'L' ? r.avg_r : r.avg_l;
}

float main_peak(const StepResult &r)
{
    return r.wheel == 'L' ? r.peak_abs_l : r.peak_abs_r;
}

const char *sign_verdict(const StepResult &r)
{
    const float avg = main_avg(r);
    if (std::fabs(avg) < kMotionThresholdMS) {
        return "NO_MOTION";
    }
    return (avg * static_cast<float>(r.duty) > 0.0f) ? "OK" : "REVERSED";
}

const char *other_wheel_verdict(const StepResult &r)
{
    return std::fabs(other_avg(r)) <= kOtherWheelThresholdMS ? "OK" : "COUPLED";
}

StepResult run_step(char wheel, int duty, lq_udp_client &udp_log)
{
    StepResult r{};
    r.wheel = wheel;
    r.duty = duty;

    stop_motors_and_settle();
    std::printf("\n[PWM_DIAG][STEP] wheel=%c duty=%d hold=%dms\n", wheel, duty, kHoldMs);
    apply_single_wheel_pwm(wheel, duty);

    float sum_l = 0.0f;
    float sum_r = 0.0f;

    for (int elapsed = 0; elapsed < kHoldMs && !should_exit(); elapsed += kSamplePeriodMs) {
        sleep_with_heartbeat(kSamplePeriodMs);
        const int now_ms = elapsed + kSamplePeriodMs;
        const float enc_l = encoder_num_L();
        const float enc_r = encoder_num_R();

        if (now_ms >= kWarmupMs) {
            sum_l += enc_l;
            sum_r += enc_r;
            r.peak_abs_l = std::max(r.peak_abs_l, std::fabs(enc_l));
            r.peak_abs_r = std::max(r.peak_abs_r, std::fabs(enc_r));
            ++r.samples;
        }

        // Send UDP: ware:wheel_duty,encL,encR,rawL,rawR
        char send_buf[128];
        int len = std::snprintf(send_buf, sizeof(send_buf),
            "ware:%c%d,%.3f,%.3f,%.3f,%.3f\n",
            wheel, duty, enc_l, enc_r,
            encoder_raw_last_L(), encoder_raw_last_R());
        if (len > 0) {
            udp_log.udp_send(send_buf, len);
        }

        if ((now_ms % 150) == 0 || now_ms == kHoldMs) {
            std::printf("[PWM_DIAG][SAMPLE] t=%3dms duty=%5d encL=% .3f encR=% .3f rawL=% .3f rawR=% .3f invalidL=%d invalidR=%d\n",
                        now_ms, duty, enc_l, enc_r,
                        encoder_raw_last_L(), encoder_raw_last_R(),
                        encoder_invalid_count_L(), encoder_invalid_count_R());
        }
    }

    r.raw_last_l = encoder_raw_last_L();
    r.raw_last_r = encoder_raw_last_R();
    r.invalid_l = encoder_invalid_count_L();
    r.invalid_r = encoder_invalid_count_R();
    if (r.samples > 0) {
        r.avg_l = sum_l / static_cast<float>(r.samples);
        r.avg_r = sum_r / static_cast<float>(r.samples);
    }

    stop_motors_and_settle();

    std::printf("[PWM_DIAG][SUMMARY] wheel=%c duty=%5d avgL=% .3f avgR=% .3f mainAvg=% .3f mainPeak=% .3f rawLastL=% .3f rawLastR=% .3f sign=%s other=%s\n",
                r.wheel, r.duty, r.avg_l, r.avg_r, main_avg(r), main_peak(r),
                r.raw_last_l, r.raw_last_r, sign_verdict(r), other_wheel_verdict(r));
    return r;
}

const StepResult *find_result(const StepResult *results, int count, char wheel, int duty)
{
    for (int i = 0; i < count; ++i) {
        if (results[i].wheel == wheel && results[i].duty == duty) {
            return &results[i];
        }
    }
    return nullptr;
}

bool check_speed_trend(const StepResult *results, int count, char wheel, int sign)
{
    const StepResult *mid = find_result(results, count, wheel, sign * 2500);
    const StepResult *high = find_result(results, count, wheel, sign * 4500);
    if (mid == nullptr || high == nullptr) {
        return false;
    }

    const float mid_abs = std::fabs(main_avg(*mid));
    const float high_abs = std::fabs(main_avg(*high));
    if (high_abs < kMotionThresholdMS) {
        std::printf("[PWM_DIAG][FAIL] wheel=%c sign=%+d high PWM has no valid encoder speed: mid=%.3f high=%.3f\n",
                    wheel, sign, mid_abs, high_abs);
        return false;
    }

    if (mid_abs < kMotionThresholdMS) {
        std::printf("[PWM_DIAG][WARN] wheel=%c sign=%+d startup dead zone: mid=%.3f high=%.3f\n",
                    wheel, sign, mid_abs, high_abs);
        return true;
    }

    if (high_abs + kTrendMarginMS < mid_abs) {
        std::printf("[PWM_DIAG][FAIL] wheel=%c sign=%+d high PWM is slower: mid=%.3f high=%.3f\n",
                    wheel, sign, mid_abs, high_abs);
        return false;
    }

    std::printf("[PWM_DIAG][TREND] wheel=%c sign=%+d mid=%.3f high=%.3f verdict=%s\n",
                wheel, sign, mid_abs, high_abs,
                high_abs >= mid_abs + kTrendMarginMS ? "OK" : "FLAT");
    return true;
}

bool inspect_step(const StepResult &r)
{
    bool ok = true;
    const float avg = main_avg(r);
    const float abs_avg = std::fabs(avg);
    if (abs_avg < kMotionThresholdMS) {
        const bool high_pwm = std::abs(r.duty) >= 4500;
        std::printf("[PWM_DIAG][%s] wheel=%c duty=%d main encoder valid speed is weak/no motion: %.3f m/s rawLastL=%.3f rawLastR=%.3f\n",
                    high_pwm ? "FAIL" : "WARN",
                    r.wheel, r.duty, avg, r.raw_last_l, r.raw_last_r);
        return !high_pwm;
    }

    if (avg * static_cast<float>(r.duty) <= 0.0f) {
        std::printf("[PWM_DIAG][FAIL] wheel=%c duty=%d encoder sign is opposite: avg=%.3f m/s\n",
                    r.wheel, r.duty, avg);
        ok = false;
    }

    if (std::fabs(other_avg(r)) > kOtherWheelThresholdMS) {
        std::printf("[PWM_DIAG][WARN] wheel=%c duty=%d other wheel also reports %.3f m/s\n",
                    r.wheel, r.duty, other_avg(r));
    }
    return ok;
}

void reset_direct_encoder(ls_encoder_pwm &encoder)
{
    encoder.encoder_reset_counter();
    usleep(20000);
    encoder.encoder_close_reset_counter();
    usleep(50000);
    encoder.encoder_clear_capture_buffer();
}

EncoderSample read_direct_encoder(ls_encoder_pwm &encoder, ls_gpio &dir)
{
    EncoderSample s{};
    s.period = encoder.encoder_get_full_period();
    s.rps = encoder.encoder_get_count();
    s.ms = s.rps * car_config::kEncoderRpsToMS;
    s.dir = static_cast<int>(dir.gpio_level_get());
    encoder.encoder_clear_capture_buffer();
    return s;
}

const char *hand_phase_for_ms(int t_ms)
{
    if (t_ms < 10000) return "idle";
    if (t_ms < 25000) return "left_forward";
    if (t_ms < 40000) return "left_reverse";
    if (t_ms < 55000) return "right_forward";
    if (t_ms < 70000) return "right_reverse";
    return "idle_end";
}
} // namespace

int lq_motor_pwm_encoder_diag(void)
{
    std::printf("\n======================================================\n");
    std::printf("   Motor PWM + Encoder Airborne Diagnostic (UDP)\n");
    std::printf("======================================================\n");
    std::printf("[PWM_DIAG] duties: +/-1200, +/-2500, +/-4500; encoder unit: m/s\n");
    std::printf("[PWM_DIAG] expected: same-sign encoder speed, and |4500| >= |2500|.\n");
    std::printf("[PWM_DIAG] UDP waveform -> 192.168.7.22:8080\n");

    if (!gpio_protect_heartbeat_require_ready(1500)) {
        std::printf("[PWM_DIAG][FATAL] gpio_manager watchdog is not ready; refuse non-zero PWM.\n");
        return 2;
    }

    lq_udp_client udp_log;
    udp_log.udp_client_init("192.168.7.22", 8080);
    std::printf("[PWM_DIAG] UDP waveform sender initialized.\n");

    motor_Init();
    encoder_reset_state();
    stop_motors_and_settle();

    const int duties[] = {800, 1500, 2500, -800, -1500, -2500};
    StepResult results[12];
    int result_count = 0;

    for (char wheel : {'L', 'R'}) {
        for (int duty : duties) {
            if (std::abs(duty) > car_config::kMotorMaxPwm) {
                std::printf("[PWM_DIAG][SKIP] duty=%d exceeds kMotorMaxPwm=%d\n",
                            duty, car_config::kMotorMaxPwm);
                continue;
            }
            results[result_count++] = run_step(wheel, duty, udp_log);
            if (should_exit()) {
                break;
            }
        }
        if (should_exit()) {
            break;
        }
    }

    stop_motors_and_settle();
    udp_log.udp_close();

    bool ok = true;
    for (int i = 0; i < result_count; ++i) {
        ok = inspect_step(results[i]) && ok;
    }
    ok = check_speed_trend(results, result_count, 'L', +1) && ok;
    ok = check_speed_trend(results, result_count, 'L', -1) && ok;
    ok = check_speed_trend(results, result_count, 'R', +1) && ok;
    ok = check_speed_trend(results, result_count, 'R', -1) && ok;

    std::printf("\n[PWM_DIAG][RESULT] %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

int lq_forward_pwm_sweep_diag(void)
{
    std::printf("\n======================================================\n");
    std::printf("   Forward-only PWM Startup Sweep\n");
    std::printf("======================================================\n");
    std::printf("[PWM_SWEEP] expected: each wheel should become positive and grow as PWM increases.\n");

    if (!gpio_protect_heartbeat_require_ready(1500)) {
        std::printf("[PWM_SWEEP][FATAL] gpio_manager watchdog is not ready; refuse non-zero PWM.\n");
        return 2;
    }

    lq_udp_client udp_log;
    udp_log.udp_client_init("192.168.7.22", 8080);

    motor_Init();
    encoder_reset_state();
    stop_motors_and_settle();

    const int duties[] = {2600, 3000, 3400, 3800, 4200, 4600, 5000};
    const int high_duties[] = {5000, 5400, 5800, 6200, 6600, 7000};
    const bool high_sweep = std::getenv("LQ_PWM_SWEEP_HIGH") != nullptr;
    const int *selected_duties = high_sweep ? high_duties : duties;
    const int selected_duty_count = high_sweep ?
        static_cast<int>(sizeof(high_duties) / sizeof(high_duties[0])) :
        static_cast<int>(sizeof(duties) / sizeof(duties[0]));
    const char *wheel_env = std::getenv("LQ_PWM_SWEEP_WHEEL");
    const bool only_left = wheel_env != nullptr && (wheel_env[0] == 'L' || wheel_env[0] == 'l');
    const bool only_right = wheel_env != nullptr && (wheel_env[0] == 'R' || wheel_env[0] == 'r');

    std::printf("[PWM_SWEEP] wheel_filter=%s high=%d duties:",
                wheel_env ? wheel_env : "both", high_sweep ? 1 : 0);
    for (int i = 0; i < selected_duty_count; ++i) {
        std::printf(" +%d", selected_duties[i]);
    }
    std::printf("; encoder unit: m/s\n");

    StepResult results[14];
    int result_count = 0;

    for (char wheel : {'L', 'R'}) {
        if ((wheel == 'L' && only_right) || (wheel == 'R' && only_left)) {
            continue;
        }
        for (int duty_index = 0; duty_index < selected_duty_count; ++duty_index) {
            const int duty = selected_duties[duty_index];
            if (duty > car_config::kMotorMaxPwm) {
                std::printf("[PWM_SWEEP][SKIP] duty=%d exceeds kMotorMaxPwm=%d\n",
                            duty, car_config::kMotorMaxPwm);
                continue;
            }
            results[result_count++] = run_step(wheel, duty, udp_log);
            if (should_exit()) {
                break;
            }
        }
        if (should_exit()) {
            break;
        }
    }

    stop_motors_and_settle();

    bool ok = true;
    for (char wheel : {'L', 'R'}) {
        if ((wheel == 'L' && only_right) || (wheel == 'R' && only_left)) {
            continue;
        }
        float previous = -1.0f;
        int first_motion_duty = 0;
        for (int i = 0; i < result_count; ++i) {
            if (results[i].wheel != wheel) {
                continue;
            }
            const float speed = main_avg(results[i]);
            const float abs_speed = std::fabs(speed);
            if (speed < -kMotionThresholdMS) {
                std::printf("[PWM_SWEEP][FAIL] wheel=%c duty=%d forward command produced negative speed %.3f\n",
                            wheel, results[i].duty, speed);
                ok = false;
            }
            if (first_motion_duty == 0 && speed >= kMotionThresholdMS) {
                first_motion_duty = results[i].duty;
            }
            if (previous >= 0.0f && abs_speed + kTrendMarginMS < previous) {
                std::printf("[PWM_SWEEP][WARN] wheel=%c duty=%d speed dropped: prev=%.3f now=%.3f\n",
                            wheel, results[i].duty, previous, abs_speed);
            }
            previous = abs_speed;
        }
        if (first_motion_duty == 0) {
            std::printf("[PWM_SWEEP][FAIL] wheel=%c no positive motion up to tested max duty.\n", wheel);
            ok = false;
        } else {
            std::printf("[PWM_SWEEP][THRESHOLD] wheel=%c first_positive_motion_duty=%d\n",
                        wheel, first_motion_duty);
        }
    }

    std::printf("\n[PWM_SWEEP][RESULT] %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

int lq_speed_loop_step_diag(void)
{
    std::printf("\n======================================================\n");
    std::printf("   Wheel-speed PI Step Diagnostic (UDP waveform)\n");
    std::printf("======================================================\n");
    std::printf("[SPEED_STEP] current PID P=%.1f I=%.1f D=%.1f max_pwm=%d\n",
                car_config::kSpeedPidP, car_config::kSpeedPidI,
                car_config::kSpeedPidD, car_config::kMotorMaxPwm);
    std::printf("[SPEED_STEP] UDP waveform -> 192.168.7.22:8080\n");
    std::printf("[SPEED_STEP] FireWater protocol: ware:target_L,actual_L,target_R,actual_R,pwm_L,pwm_R\n");

    if (!gpio_protect_heartbeat_require_ready(1500)) {
        std::printf("[SPEED_STEP][FATAL] gpio_manager watchdog is not ready; refuse non-zero PWM.\n");
        return 2;
    }

    // UDP client for waveform logging
    lq_udp_client udp_log;
    udp_log.udp_client_init("192.168.7.22", 8080);
    std::printf("[SPEED_STEP] UDP waveform sender initialized.\n");

    motor_Init();
    if (!Speed_decision()) {
        motor_Duty_L(0);
        motor_Duty_R(0);
        std::printf("[SPEED_STEP][FATAL] Speed_decision/IMU init failed.\n");
        udp_log.udp_close();
        return 3;
    }

    encoder_reset_state();
    stop_motors_and_settle();

    // Track-safe speed-loop tuning: low-speed steps only, short enough for ~5m straight.
    // const float targets[] = {0.16f, 0.30f, 0.45f};
    const float targets[] = {0.16f, 0.30f, 0.45f};
    const int step_duration_ms = 3000;
    const int settle_duration_ms = 300;
    const int summary_start_ms = 1000;
    bool ok = true;

    for (float target : targets) {
        std::printf("\n[SPEED_STEP][TARGET] target=%.2f m/s duration=%dms\n", target, step_duration_ms);
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        Speed_PID_L();
        Speed_PID_R();
        motor_Duty_L(0);
        motor_Duty_R(0);
        encoder_reset_state();
        sleep_with_heartbeat(250);

        float sum_l = 0.0f;
        float sum_r = 0.0f;
        int samples = 0;
        float peak_l = 0.0f;
        float peak_r = 0.0f;
        float last_pwm_l = 0.0f;
        float last_pwm_r = 0.0f;
        bool tripped = false;

        for (int t = 0; t <= step_duration_ms && !should_exit(); t += 5) {
            speed_encoder_L = encoder_num_L();
            speed_encoder_R = encoder_num_R();
            Speed_Goal_l = target;
            Speed_Goal_r = target;
            Speed_PID_L();
            Speed_PID_R();
            last_pwm_l = speed_out_l;
            last_pwm_r = speed_out_r;
            motor_Duty_L(static_cast<int>(speed_out_l));
            motor_Duty_R(static_cast<int>(speed_out_r));

            peak_l = std::max(peak_l, std::fabs(speed_encoder_L));
            peak_r = std::max(peak_r, std::fabs(speed_encoder_R));
            if (t >= summary_start_ms) {
                sum_l += speed_encoder_L;
                sum_r += speed_encoder_R;
                ++samples;
            }

            // Send UDP waveform every 20ms (every 4th iteration)
            if ((t % 20) == 0) {
                char send_buf[128];
                int len = std::snprintf(send_buf, sizeof(send_buf),
                    "ware:%.3f,%.3f,%.3f\n",
                    target * 100, speed_encoder_L * 100, speed_encoder_R * 100);
                if (len > 0) {
                    udp_log.udp_send(send_buf, len);
                }
            }

            if ((t % 100) == 0) {
                std::printf("[SPEED_STEP] t=%4d target=%.2f encL=% .3f encR=% .3f pwmL=% .0f pwmR=% .0f rawL=% .3f rawR=% .3f badL=%d badR=%d\n",
                            t, target, speed_encoder_L, speed_encoder_R,
                            speed_out_l, speed_out_r,
                            encoder_raw_last_L(), encoder_raw_last_R(),
                            encoder_invalid_count_L(), encoder_invalid_count_R());
            }

            if (std::fabs(speed_encoder_L) > 4.5f || std::fabs(speed_encoder_R) > 4.5f) {
                std::printf("[SPEED_STEP][WARN] speed exceeded diagnostic guard; stop this target.\n");
                tripped = true;
                break;
            }
            sleep_with_heartbeat(5);
        }

        motor_Duty_L(0);
        motor_Duty_R(0);
        Speed_Goal_l = 0.0f;
        Speed_Goal_r = 0.0f;
        Speed_PID_L();
        Speed_PID_R();
        sleep_with_heartbeat(settle_duration_ms);

        const float avg_l = samples > 0 ? sum_l / static_cast<float>(samples) : 0.0f;
        const float avg_r = samples > 0 ? sum_r / static_cast<float>(samples) : 0.0f;
        std::printf("[SPEED_STEP][SUMMARY] target=%.2f avgL=% .3f avgR=% .3f peakL=% .3f peakR=% .3f pwmL=% .0f pwmR=% .0f tripped=%d\n",
                    target, avg_l, avg_r, peak_l, peak_r,
                    last_pwm_l, last_pwm_r, tripped ? 1 : 0);
        if (tripped || avg_l < target * 0.5f || avg_r < target * 0.5f) {
            ok = false;
        }
    }

    stop_motors_and_settle();
    udp_log.udp_close();
    std::printf("\n[SPEED_STEP][RESULT] %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

int lq_encoder_hand_push_diag(void)
{
    std::printf("\n======================================================\n");
    std::printf("   Read-only Encoder Hand-push Diagnostic\n");
    std::printf("======================================================\n");
    std::printf("[HAND_ENC] motors stay stopped; do not power wheels in this test.\n");
    std::printf("[HAND_ENC] timeline: 0-10s idle, 10-25s left forward, 25-40s left reverse, 40-55s right forward, 55-70s right reverse, 70-75s idle.\n");
    std::printf("[HAND_ENC] mapped speed: mapLms=-ch3ms, mapRms=ch0ms. Forward hand-push should be positive.\n");

    motor_Init();
    motor_Duty_L(0);
    motor_Duty_R(0);
    gpio_protect_heartbeat_pulse();

    ls_encoder_pwm encoder_0(ENC_PWM0_PIN64, PIN_72);
    ls_encoder_pwm encoder_1(ENC_PWM1_PIN65, PIN_73);
    ls_encoder_pwm encoder_2(ENC_PWM2_PIN66, PIN_74);
    ls_encoder_pwm encoder_3(ENC_PWM3_PIN67, PIN_75);
    ls_gpio dir_0(PIN_72, GPIO_MODE_IN);
    ls_gpio dir_1(PIN_73, GPIO_MODE_IN);
    ls_gpio dir_2(PIN_74, GPIO_MODE_IN);
    ls_gpio dir_3(PIN_75, GPIO_MODE_IN);

    reset_direct_encoder(encoder_0);
    reset_direct_encoder(encoder_1);
    reset_direct_encoder(encoder_2);
    reset_direct_encoder(encoder_3);

    int duration_ms = 75000;
    if (const char *env = std::getenv("LQ_HAND_PUSH_DURATION_MS")) {
        const int parsed = std::atoi(env);
        if (parsed >= 15000 && parsed <= 180000) {
            duration_ms = parsed;
        }
    }

    const int sample_ms = 100;
    const char *last_phase = "";
    for (int t_ms = 0; t_ms <= duration_ms && !should_exit(); t_ms += sample_ms) {
        const char *phase = hand_phase_for_ms(t_ms);
        if (std::strcmp(phase, last_phase) != 0) {
            std::printf("[HAND_ENC][PHASE] t=%d phase=%s\n", t_ms, phase);
            last_phase = phase;
        }

        EncoderSample ch0 = read_direct_encoder(encoder_0, dir_0);
        EncoderSample ch1 = read_direct_encoder(encoder_1, dir_1);
        EncoderSample ch2 = read_direct_encoder(encoder_2, dir_2);
        EncoderSample ch3 = read_direct_encoder(encoder_3, dir_3);
        const float map_l_ms = -ch3.ms;
        const float map_r_ms = ch0.ms;

        std::printf("[HAND_ENC] t=%05d phase=%s mapLms=% .3f mapRms=% .3f ch0ms=% .3f ch0dir=%d ch0per=%u ch1ms=% .3f ch1dir=%d ch1per=%u ch2ms=% .3f ch2dir=%d ch2per=%u ch3ms=% .3f ch3dir=%d ch3per=%u\n",
                    t_ms, phase, map_l_ms, map_r_ms,
                    ch0.ms, ch0.dir, ch0.period,
                    ch1.ms, ch1.dir, ch1.period,
                    ch2.ms, ch2.dir, ch2.period,
                    ch3.ms, ch3.dir, ch3.period);
        sleep_with_heartbeat(sample_ms);
    }

    motor_Duty_L(0);
    motor_Duty_R(0);
    gpio_protect_heartbeat_pulse();
    std::printf("[HAND_ENC][RESULT] DONE\n");
    return 0;
}
