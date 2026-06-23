#include "main.hpp"
#include "thread"
#include <cstdio>
#include <cstring>

// 定义定时器间隔（5ms）
const auto timer_interval_1 = std::chrono::milliseconds(5);

static void print_usage(const char *program)
{
    std::printf("用法：%s --mode=<car|tune|encoder-test|vision-debug> [--unsafe-dev-mode]\n", program);
    std::printf("  --mode=car           正式整车闭环模式，强制要求 gpio_manager 看门狗和 IMU 正常\n");
    std::printf("  --mode=tune          Demo33 分步调参入口\n");
    std::printf("  --mode=encoder-test  编码器/电机方向测试\n");
    std::printf("  --mode=vision-debug  巡线图像调试\n");
    std::printf("  --unsafe-dev-mode    仅调试使用：允许 car 模式跳过 gpio_manager 强制检查\n");
}

static std::string parse_mode(int argc, char **argv, bool &unsafe_dev_mode)
{
    unsafe_dev_mode = false;
    std::string mode;
    for (int i = 1; i < argc; ++i) {
        const char *arg = argv[i];
        if (std::strncmp(arg, "--mode=", 7) == 0) {
            mode = std::string(arg + 7);
            continue;
        }
        if (std::strcmp(arg, "--mode") == 0 && i + 1 < argc) {
            mode = std::string(argv[++i]);
            continue;
        }
        if (std::strcmp(arg, "--unsafe-dev-mode") == 0) {
            unsafe_dev_mode = true;
            continue;
        }
        if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            return "help";  // 立即返回帮助
        }
        std::printf("[MAIN] unknown argument: %s\n", arg);
        return "help";
    }
    return mode.empty() ? "help" : mode;
}

// 闭环控制定时器（极高优先级）
void timer_thread_1()
{
    while (!exit_flag)
    {
        Speed_BiHuan();

        std::this_thread::sleep_for(timer_interval_1);
    }
}


int main(int argc, char **argv)
{
    bool unsafe_dev_mode = false;
    std::string run_mode = parse_mode(argc, argv, unsafe_dev_mode);
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
        std::printf("\033[31m[MAIN][UNSAFE] --unsafe-dev-mode: gpio_manager watchdog check skipped. Do not drive on track.\033[0m\n");
    }

    signal(SIGINT, signal_handler);   // 捕获Ctrl+C
    signal(SIGTERM, signal_handler);  // 捕获系统kill命令
    signal(SIGHUP, signal_handler);   // 捕获SSH断开/终端被关掉的情况
    signal(SIGQUIT, signal_handler);  // 捕获 Ctrl+\ 退出
    // SIGABRT/SIGSEGV 保持系统默认动作：立即终止整个进程，使控制线程停止喂狗，
    // 由 gpio_manager 在心跳超时后写入硬件停车值。禁止捕获后返回到故障指令。
    signal(SIGPIPE, SIG_IGN);         // 忽略破损管道信号(当图传UDP发送失败时忽略之，不退出程序)

    motor_Init();     // 电机初始化
    control_reset_vision_health();
    if (!Speed_decision()) { // 电机PID参数初始化 + IMU 自检
        emergency_stop();
        gpio_protect_heartbeat_deinit();
        std::printf("[MAIN][FATAL] MPU6050 init failed; car mode refused.\n");
        return 4;
    }
    Data_Settings();              // 图像处理参数初始化
    char image_flag = image_cv_Init(); // 摄像头初始化
    if (!image_flag) {
        emergency_stop();
        gpio_protect_heartbeat_deinit();
        std::printf("[MAIN][FATAL] camera init failed; car mode refused.\n");
        return 5;
    }

    // 校赛无元素竞速模式：关闭元素识别/处理，只跑基础循迹闭环。
    g_enable_element_process = false;
    printf("[RACE_MODE] no-element race mode: element process disabled, main loop enabled.\n");

    // UDP 图传初始化（正式 car 模式默认关闭；图传只用于 vision-debug/低速诊断）
    lq_udp_client udp_debug;
    if (car_config::kRaceUdpDebugEnable) {
        udp_debug.udp_client_init("192.168.90.22", 8080);
        printf("[UDP_DEBUG] center-line overlay to 192.168.90.22:8080, every 5 frames\n");
    }

    // 分离线程，允许后台运行高速闭环控制代码
    std::thread threadA(timer_thread_1);

    int udp_frame_count = 0;

    while (!exit_flag)
    {
        // 图像二值化和压缩
        image_flag = image_cv_zip();
        if (!image_flag)   // 如果图像获取失败，立即停车并停止心跳
        {
            motor_Duty_R(0);
            motor_Duty_L(0);
            emergency_stop();  // 图像获取失败时停止电机
            exit_flag = true;  // 通知子线程退出
            break;
        }

        // 图像处理
        imageprocess();
        control_mark_vision_frame_ready();

        // UDP 图传：每5帧发送一次（调用 demo24 中的函数）
        if (car_config::kRaceUdpDebugEnable && ((udp_frame_count % 5) == 0)) {
            lq_udp_center_line_send_one_frame(udp_debug, 50);
        }
        udp_frame_count++;
    }
    exit_flag = true;
    threadA.join();  // 等待底层硬实时线程结束
    control_reset_vision_health();
    emergency_stop();  // 最终确认电机停止
    gpio_protect_heartbeat_deinit();
    return 0;
}
