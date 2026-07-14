#include "main.hpp"
#include "motion_feedback_guard.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

const auto timer_interval_1 = std::chrono::milliseconds(5);

static void print_usage(const char *program)
{
    std::printf("Usage: %s --mode=<car|vision-capture|element-observe|motor-pwm-test|encoder-hand-test|forward-pwm-sweep|speed-loop-step|udp-pid-autotune|motor-id|gyro-rate-tune|gyro-motor-tune> [--unsafe-dev-mode] [--sine] [--speed=<m/s>] [--display=<udp|tft|both>]\n", program);
    std::printf("  --mode=car             competition control mode\n");
    std::printf("  --mode=vision-capture  camera/vision capture diagnostic without motor drive\n");
    std::printf("  --mode=element-observe element observation (no motor): --display=udp|tft|both\n");
    std::printf("  --mode=motor-pwm-test   airborne PWM/encoder direction diagnostic\n");
    std::printf("  --mode=encoder-hand-test read-only hand-push encoder diagnostic\n");
    std::printf("  --mode=forward-pwm-sweep forward-only PWM startup sweep\n");
    std::printf("  --mode=speed-loop-step   airborne wheel-speed PI step diagnostic\n");
    std::printf("  --mode=udp-pid-autotune  UDP wireless LLM auto PID tuning\n");
    std::printf("  --mode=motor-id          motor identification: PWM step sweep for load model\n");
    std::printf("  --mode=gyro-rate-tune    gyro rate PID tuning (no motor, IMU only)\n");
    std::printf("  --mode=gyro-motor-tune   gyro rate PID tuning with motor differential drive\n");
    std::printf("    --sine                 (gyro-motor-tune only) sine sweep instead of step\n");
    std::printf("    --speed=<m/s>          (gyro-motor-tune only) base speed, default 0.20\n");
    std::printf("  --unsafe-dev-mode       bypass initial gpio_manager readiness; never use on track\n");
}

static std::string parse_mode(int argc, char **argv, bool &unsafe_dev_mode, int &display_mode)
{
    unsafe_dev_mode = false;
    display_mode = 0;  // default: udp only
    std::string mode;
    for (int i = 1; i < argc; ++i) {
        const char *arg = argv[i];
        if (std::strncmp(arg, "--mode=", 7) == 0) {
            mode = std::string(arg + 7);
        } else if (std::strcmp(arg, "--mode") == 0 && i + 1 < argc) {
            mode = std::string(argv[++i]);
        } else if (std::strcmp(arg, "--unsafe-dev-mode") == 0) {
            unsafe_dev_mode = true;
        } else if (std::strncmp(arg, "--display=", 10) == 0) {
            std::string disp(arg + 10);
            if (disp == "udp") display_mode = 0;
            else if (disp == "tft") display_mode = 1;
            else if (disp == "both") display_mode = 2;
            else std::printf("[MAIN] unknown display: %s (use udp|tft|both)\n", disp.c_str());
        } else if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            return "help";
        } else if (std::strncmp(arg, "--", 2) == 0) {
            // Pass through mode-specific flags (e.g. --sine, --speed=, --kp=, --kd=, --ff=, --step=, --hold=)
            // to be parsed by the demo function itself.
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
    int display_mode = 0;
    const std::string run_mode = parse_mode(argc, argv, unsafe_dev_mode, display_mode);
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

    if (run_mode == "gyro-rate-tune") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_gyro_rate_pid_tune_demo();
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "gyro-motor-tune") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_gyro_motor_pid_tune_demo(argc, argv);
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "diff-speed-test") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_diff_speed_test_demo();
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "channel-check") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_channel_check_demo();
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "vision-capture") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_vision_capture_demo();
        emergency_stop();
        gpio_protect_heartbeat_deinit();
        return ret;
    }

    if (run_mode == "element-observe") {
        if (!unsafe_dev_mode && !gpio_protect_heartbeat_require_ready(1500)) {
            gpio_protect_heartbeat_deinit();
            return 3;
        }
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGHUP, signal_handler);
        signal(SIGQUIT, signal_handler);
        signal(SIGPIPE, SIG_IGN);

        const int ret = lq_element_observe_demo(display_mode);
        gpio_protect_heartbeat_deinit();
        return ret;
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
    control_set_imu_async_cache_only(true);

    std::printf("[MAIN][CAR] before Data_Settings\n");
    std::fflush(stdout);
    Data_Settings();
    std::printf("[MAIN][CAR] before image_cv_Init\n");
    std::fflush(stdout);
    char image_flag = image_cv_Init();
    std::printf("[MAIN][CAR] after image_cv_Init flag=%d\n", image_flag ? 1 : 0);
    std::fflush(stdout);
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
        udp_debug.udp_client_init(car_config::kNcnnObserveUdpHost, car_config::kNcnnObserveUdpPort);
        std::printf("[UDP_DEBUG] abnormal center-line overlay to %s:%d\n",
                    car_config::kNcnnObserveUdpHost,
                    car_config::kNcnnObserveUdpPort);
    }
    const bool ncnn_observe_ready = car_config::kNcnnObserveEnable &&
        lq_ncnn_observe_start(car_config::kNcnnObserveUdpHost, car_config::kNcnnObserveUdpPort);
    const int ncnn_observe_interval = std::max(1, car_config::kNcnnObserveUdpEveryFrames);

    // Drop stale capture/filter state before the 5 ms control loop owns encoders.
    encoder_reset_state();
    std::thread threadImu(imu_thread_1);
    std::thread threadA(timer_thread_1);
    int udp_frame_count = 0;
    int udp_debug_last_frame = -1000;
    int udp_debug_sent = 0;
    int camera_fail_count = 0;
    bool udp_debug_last_full_loss = false;

    while (!exit_flag.load(std::memory_order_relaxed)) {
        image_flag = image_cv_zip();
        if (!image_flag) {
            ++camera_fail_count;
            if (camera_fail_count <= 5) {
                std::printf("[MAIN][WARN] camera frame read failed count=%d\n", camera_fail_count);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            std::printf("[MAIN][FATAL] camera frame read failed repeatedly; stopping car.\n");
            exit_flag.store(true, std::memory_order_relaxed);
            break;
        }
        camera_fail_count = 0;
        if (exit_flag.load(std::memory_order_relaxed)) {
            break;
        }

        imageprocess();
        if (!car_config::kNoElementCompetitionMode) {
            lq_vision_capture_maybe_save_race_frame(udp_frame_count);
        }
        control_mark_vision_frame_ready();

        if (ncnn_observe_ready && (udp_frame_count % ncnn_observe_interval) == 0) {
            lq_ncnn_observe_submit(image_cv_get_hires_frame(), udp_frame_count, "car");
        }

        if (car_config::kRaceUdpDebugEnable &&
            udp_debug_sent < car_config::kRaceUdpDebugMaxFrames) {
            const int raw_error = ImageStatus.MiddleLine - ImageStatus.Det_True;
            const bool strong_turn = std::abs(raw_error) >= 12;
            const bool side_loss = ImageStatus.Left_Line >= 18 || ImageStatus.Right_Line >= 18;
            const bool full_loss = ImageStatus.OFFLine >= 45;
            const bool abnormal = strong_turn || side_loss || full_loss;
            const bool entering_full_loss = full_loss && !udp_debug_last_full_loss;
            const bool sparse_enough =
                (udp_frame_count - udp_debug_last_frame) >=
                car_config::kRaceUdpDebugIntervalFrames;
            if (abnormal && (entering_full_loss || sparse_enough)) {
                std::printf("[UDP_DEBUG] trigger frame:%d det:%d mid:%d err:%d off:%d ll:%d rl:%d full:%d sent:%d\n",
                            udp_frame_count,
                            ImageStatus.Det_True,
                            ImageStatus.MiddleLine,
                            raw_error,
                            ImageStatus.OFFLine,
                            ImageStatus.Left_Line,
                            ImageStatus.Right_Line,
                            full_loss ? 1 : 0,
                            udp_debug_sent);
                lq_udp_center_line_send_one_frame(udp_debug, 55);
                udp_debug_last_frame = udp_frame_count;
                ++udp_debug_sent;
            }
            udp_debug_last_full_loss = full_loss;
        }
        ++udp_frame_count;
    }

    exit_flag.store(true, std::memory_order_relaxed);
    if (ncnn_observe_ready) {
        lq_ncnn_observe_stop();
    }
    threadA.join();
    control_reset_vision_health();
    motion_feedback_guard_reset();
    emergency_stop();
    if (threadImu.joinable()) {
        threadImu.join();
    }
    gpio_protect_heartbeat_deinit();
    return 0;
}
