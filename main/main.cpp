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
    std::printf("Usage: %s --mode=<car|tune|encoder-test|encoder512|vision-debug|speed-tune> [--unsafe-dev-mode]\n", program);
    std::printf("  --mode=car           competition control mode\n");
    std::printf("  --mode=tune          interactive parameter tuning\n");
    std::printf("  --mode=encoder-test  encoder direction test\n");
    std::printf("  --mode=encoder512    encoder UDP diagnostics\n");
    std::printf("  --mode=vision-debug  vision debug mode\n");
    std::printf("  --mode=speed-tune    forward-only speed-loop tune\n");
    std::printf("  --mode=speed-return  safe-stop placeholder\n");
    std::printf("  --unsafe-dev-mode    bypass gpio_manager readiness; never use on track\n");
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
        // This reads control state only from the same control thread that writes it.
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

    if (run_mode == "tune") {
        lq_parameter_step_interactive_demo();
        gpio_protect_heartbeat_deinit();
        return 0;
    }
    if (run_mode == "encoder-test") {
        lq_car_encoder_test_demo();
        gpio_protect_heartbeat_deinit();
        return 0;
    }
    if (run_mode == "vision-debug") {
        lq_line_patrol_debug_demo();
        gpio_protect_heartbeat_deinit();
        return 0;
    }
    if (run_mode == "speed-tune") {
        lq_speed_loop_track_udp_tune_demo();
        gpio_protect_heartbeat_deinit();
        return 0;
    }
    if (run_mode == "speed-return") {
        lq_speed_loop_recovery_return_demo();
        gpio_protect_heartbeat_deinit();
        return 0;
    }
    if (run_mode == "encoder512") {
        lq_encoder512_udp_diagnose_demo();
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

    std::thread threadA(timer_thread_1);
    int udp_frame_count = 0;

    while (!exit_flag.load(std::memory_order_relaxed)) {
        image_flag = image_cv_zip();
        if (!image_flag) {
            motor_Duty_R(0);
            motor_Duty_L(0);
            emergency_stop();
            exit_flag.store(true, std::memory_order_relaxed);
            break;
        }

        imageprocess();
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
