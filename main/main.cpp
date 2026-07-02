#include "main.hpp"
#include "motion_feedback_guard.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

const auto timer_interval_1 = std::chrono::milliseconds(5);

static void print_usage(const char *program)
{
    std::printf("Usage: %s --mode=<car|motor-pwm-test|encoder-hand-test|forward-pwm-sweep|speed-loop-step|udp-pid-autotune|motor-id> [--unsafe-dev-mode]\n", program);
    std::printf("  --mode=car             competition control mode\n");
    std::printf("  --mode=motor-pwm-test   airborne PWM/encoder direction diagnostic\n");
    std::printf("  --mode=encoder-hand-test read-only hand-push encoder diagnostic\n");
    std::printf("  --mode=forward-pwm-sweep forward-only PWM startup sweep\n");
    std::printf("  --mode=speed-loop-step   airborne wheel-speed PI step diagnostic\n");
    std::printf("  --mode=udp-pid-autotune  UDP wireless LLM auto PID tuning\n");
    std::printf("  --mode=motor-id          motor identification: PWM step sweep for load model\n");
    std::printf("  --unsafe-dev-mode       bypass initial gpio_manager readiness; never use on track\n");
}

static std::string parse_mode(int argc, char **argv, bool &unsafe_dev_mode)
{
    unsafe_dev_mode = false;
    std::string mode;
    for (int i = 1; i < argc; ++i) {
        const char *arg = argv[i];
        if (std::strncmp(arg, "--mode=", 7) == 0) {
            mode = std::string(arg + 7);
        } else if (std::strcmp(arg, "--mode") == 0 && i + 1 < argc) {
            mode = std::string(argv[++i]);
        } else if (std::strcmp(arg, "--unsafe-dev-mode") == 0) {
            unsafe_dev_mode = true;
        } else if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            return "help";
        } else {
            std::printf("[MAIN] unknown argument: %s\n", arg);
            return "help";
        }
    }
    return mode.empty() ? "help" : mode;
}

void timer_thread_1()
{
    while (!exit_flag.load(std::memory_order_relaxed)) {
        Speed_BiHuan();
        motion_feedback_guard_tick();
        std::this_thread::sleep_for(timer_interval_1);
    }
}

int main(int argc, char **argv)
{
    bool unsafe_dev_mode = false;
    const std::string run_mode = parse_mode(argc, argv, unsafe_dev_mode);
    if (run_mode == "help") {
        print_usage(argv[0]);
        return 0;
    }

    gpio_protect_heartbeat_init();

    if (run_mode == "motor-pwm-test") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        if (unsafe_dev_mode) {
            std::printf("[MAIN][UNSAFE] initial gpio_manager readiness check skipped. Direct-drive guard remains active.\n");
        }

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_motor_pwm_encoder_diag();
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "encoder-hand-test") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        if (unsafe_dev_mode) {
            std::printf("[MAIN][UNSAFE] initial gpio_manager readiness check skipped. Motors stay at zero PWM.\n");
        }

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_encoder_hand_push_diag();
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "forward-pwm-sweep") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        if (unsafe_dev_mode) {
            std::printf("[MAIN][UNSAFE] initial gpio_manager readiness check skipped. Direct-drive guard remains active.\n");
        }

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_forward_pwm_sweep_diag();
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "speed-loop-step") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        if (unsafe_dev_mode) {
            std::printf("[MAIN][UNSAFE] initial gpio_manager readiness check skipped. Direct-drive guard remains active.\n");
        }

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_speed_loop_step_diag();
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "udp-pid-autotune") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        if (unsafe_dev_mode) {
            std::printf("[MAIN][UNSAFE] initial gpio_manager readiness check skipped. Direct-drive guard remains active.\n");
        }

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        lq_udp_llm_pid_autotune_demo();
        gpio_protect_heartbeat_deinit();
        return 0;
    }

    if (run_mode == "motor-id") {
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        lq_motor_id_demo();
        gpio_protect_heartbeat_deinit();
        return 0;
    }

    if (run_mode == "pwm-quick-test") {
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        lq_pwm_quick_test();
        gpio_protect_heartbeat_deinit();
        return 0;
    }

    if (run_mode != "car") {
        std::printf("[MAIN] unsupported mode: %s\n", run_mode.c_str());
        print_usage(argv[0]);
        gpio_protect_heartbeat_deinit();
        return 2;
    }

    if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
        gpio_protect_heartbeat_deinit();
        return 3;
    }
    if (unsafe_dev_mode) {
        std::printf("[MAIN][UNSAFE] gpio_manager readiness check skipped. Do not drive on track.\n");
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGHUP, signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGPIPE, SIG_IGN);

    motor_Init();
    control_reset_vision_health();
    motion_feedback_guard_reset();
    if (!Speed_decision()) {
        emergency_stop();
        gpio_protect_heartbeat_deinit();
        std::printf("[MAIN][FATAL] MPU6050 init failed; car mode refused.\n");
        return 4;
    }

    Data_Settings();
    char image_flag = image_cv_Init();
    if (!image_flag) {
        emergency_stop();
        gpio_protect_heartbeat_deinit();
        std::printf("[MAIN][FATAL] camera init failed; car mode refused.\n");
        return 5;
    }

    g_enable_element_process = false;
    std::printf("[RACE_MODE] no-element race mode: element process disabled, main loop enabled.\n");

    lq_udp_client udp_debug;
    if (car_config::kRaceUdpDebugEnable) {
        udp_debug.udp_client_init("192.168.46.22", 8080);
        std::printf("[UDP_DEBUG] center-line overlay to 192.168.46.22:8080, every 5 frames\n");
    }

    // Drop stale capture/filter state before the 5 ms control loop owns encoders.
    encoder_reset_state();
    std::thread threadA(timer_thread_1);
    int udp_frame_count = 0;

    while (!exit_flag.load(std::memory_order_relaxed)) {
        image_flag = image_cv_zip();
        if (!image_flag) {
            // Do not race the control thread by writing PWM from this thread.
            // The thread exits within one 5 ms period; shutdown below owns final PWM writes.
            exit_flag.store(true, std::memory_order_relaxed);
            break;
        }
        if (exit_flag.load(std::memory_order_relaxed)) {
            break;
        }

        imageprocess();
        lq_vision_capture_maybe_save_race_frame(udp_frame_count);
        control_mark_vision_frame_ready();

        if (car_config::kRaceUdpDebugEnable && ((udp_frame_count % 5) == 0)) {
            lq_udp_center_line_send_one_frame(udp_debug, 50);
        }
        ++udp_frame_count;
    }

    exit_flag.store(true, std::memory_order_relaxed);
    threadA.join();
    control_reset_vision_health();
    motion_feedback_guard_reset();
    emergency_stop();
    gpio_protect_heartbeat_deinit();
    return 0;
}
