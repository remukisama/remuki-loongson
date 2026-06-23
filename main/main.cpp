#include "main.hpp"
#include "thread"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
// HWGpio beep(61, GPIO_Mode_Out);
ls_gpio* beep = nullptr;  // 延迟初始化，避免全局构造时访问 /dev/mem 导致崩溃


bool flags = true;     // 判断第一次进入函数标志位
uint8_t fps_count = 0; // 帧数累计
double start_time = 0; // 开始时间
double end_time = 0;   // 结束时间

// 定义定时器间隔（5ms）
const auto timer_interval_1 = std::chrono::milliseconds(5);

static std::string read_autotune_stage()
{
    const char *paths[] = {
        "lq_autotune_stage.txt",
        "/tmp/lq_autotune_stage.txt",
    };
    for (const char *path : paths) {
        std::ifstream in(path);
        if (!in.good()) continue;
        std::string stage;
        std::getline(in, stage);
        while (!stage.empty() && (stage.back() == '\r' || stage.back() == '\n' || stage.back() == ' ' || stage.back() == '\t')) {
            stage.pop_back();
        }
        return stage;
    }
    return "";
}

static int run_autotune_stage_if_requested()
{
    std::string stage = read_autotune_stage();
    if (stage.empty()) return -999;

    std::printf("[AUTOTUNE] requested_stage=%s\n", stage.c_str());
    if (stage == "T10") return lq_param_test_10_gyro_static_noise();
    if (stage == "T11") return lq_param_test_11_gyro_loop_step_response();
    if (stage == "T06") return lq_param_test_06_low_speed_visual_gyro_closed_loop();
    if (stage == "T07") return lq_param_test_07_speed_up_regression();
    if (stage == "FLOW_LOW_SPEED") return lq_param_flow_low_speed();

    std::printf("[AUTOTUNE] unknown_stage=%s, expected T10/T11/T06/T07/FLOW_LOW_SPEED\n", stage.c_str());
    return 98;
}


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
    for (int i = 1; i < argc; ++i) {
        const char *arg = argv[i];
        if (std::strncmp(arg, "--mode=", 7) == 0) {
            return std::string(arg + 7);
        }
        if (std::strcmp(arg, "--mode") == 0 && i + 1 < argc) {
            return std::string(argv[++i]);
        }
        if (std::strcmp(arg, "--unsafe-dev-mode") == 0) {
            unsafe_dev_mode = true;
            continue;
        }
        if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            return "help";
        }
        std::printf("[MAIN] unknown argument: %s\n", arg);
        return "help";
    }
    return "help";
}

// 闭环控制定时器（极高优先级）
void timer_thread_1()
{
    while (!exit_flag)
    {
        Speed_BiHuan();

        std::this_thread::sleep_for(timer_interval_1);
    }
    // 线程退出前执行一次停机
    // SafeMotorStop();
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

    // ================== 新增编码器排查 Demo ==================
    // 只要把这一行的注释解开，它就会阻断后续所有代码，专心跑编码器测试：
    // lq_car_encoder_test_demo(); 
    // =========================================================
    // lq_line_patrol_debug_demo(); // 直接运行这个 Demo 就能看到巡线算法的调试图传了！如果不想看了，注释掉它就行了。
    // lq_user_encoder_demo();
    // lq_quadrature_encoder_kernel_demo();
    // lq_motor_pid_demo();
    // lq_encoder_pwm_demo();
    // MotorTest();
    // lq_pwm_demo();
    // lq_atim_pwm_demo();
    // return 0;
    // lq_encoder_pwm_demo();
    // lq_udp_img_trans_demo();
    // lq_quadrature_demo();
    // lq_icm42688_demo();
    // lq_mpu6050_demo();
    // lq_speed_pid_tuning_demo();
    // lq_tft18_demo();
    // lq_angle_pid_tuning_demo();
    // lq_mpu6050_scale_dt_diagnose_demo();
    // lq_vofa_speed_pid_demo();
    // lq_gyro_pid_tuning_demo();
    // lq_gyro_loop_verify_demo();
    // lq_camera_video_demo();  // 视频录制 Demo，切换到实车模式时注释此行
    // return 0;

    // ===== Demo 33 分步调试总入口：输入编号运行对应测试 =====
    // lq_parameter_step_interactive_demo();
    // return 0;

    char image_flag = 0;

    if (beep) beep->gpio_level_set(GPIO_LOW); // 关闭蜂鸣器

    // /**************************
    //  * 关键：注册全量异常信号捕获
    //  **************************/
    signal(SIGINT, signal_handler);   // 捕获Ctrl+C
    signal(SIGTERM, signal_handler);  // 捕获系统kill命令
    signal(SIGHUP, signal_handler);   // 捕获SSH断开/终端被关掉的情况
    signal(SIGQUIT, signal_handler);  // 捕获 Ctrl+\ 退出
    // SIGABRT/SIGSEGV 保持系统默认动作：立即终止整个进程，使控制线程停止喂狗，
    // 由 gpio_manager 在心跳超时后写入硬件停车值。禁止捕获后返回到故障指令。
    signal(SIGPIPE, SIG_IGN);         // 忽略破损管道信号(当图传UDP发送失败时忽略之，不退出程序)

    // 双路 PWM 原始波形测试已完成；整车调试必须进入下方正式闭环，禁止在此早返回。
    // lq_atim_pwm_demo();
    // gpio_protect_heartbeat_deinit();
    // return 0;
    //
    // 整车赛道模式禁用文件触发的调参分流，避免板上残留 /tmp/lq_autotune_stage.txt
    // 让程序意外进入 T06/T07 等测试流程。需要自动调参时再显式恢复此段。
    // int autotune_ret = run_autotune_stage_if_requested();
    // if (autotune_ret != -999) {
    //     printf("[AUTOTUNE] stage_ret=%d\n", autotune_ret);
    //     return autotune_ret;
    // }
    //
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
    // lq_vofa_speed_sine_demo();

    // lq_gyro_loop_verify_demo();
    // return 0;

    // demo33 T07：提速回归测试，验证提速后是否压线/出界/震荡。
    // int demo33_ret = lq_param_test_01_motor_encoder_direction();
    // printf("[MAIN] demo33 speed_up_regression ret=%d\n", demo33_ret);
    // return demo33_ret;

    // InitMH();       //  模糊pid初始化
    motor_Init();     // 电机初始化
    if (!Speed_decision()) { // 电机PID参数初始化 + IMU 自检
        emergency_stop();
        gpio_protect_heartbeat_deinit();
        std::printf("[MAIN][FATAL] MPU6050 init failed; car mode refused.\n");
        return 4;
    }
    // Stree_Init();     // 舵机初始化
    // ICM42605_Init();
    // TFTSPI_Init(0);
    // TFTSPI_CLS(u16BLACK); // 白色屏幕
    // TFTTest();
    Data_Settings();              // 图像处理参数初始化
    image_flag = image_cv_Init(); // 摄像头初始化 - （由于Demo内已经初始化了此项，主函数先屏蔽以防重复打开设备报错）
    if (image_flag == -1)
        return 0;

    // // ============================================
    // // 手推车图像调试模式：拦截进入以下Demo
    // // (该Demo内置死循环通过图传发送图像，不会跑到下面的控制线程代码)
    // // ============================================
    // lq_line_patrol_debug_demo();
    // return 0;

    // CameraTest();
    // lq_udp_img_trans_demo();
    
    // ============================================
    // [DEMO 30] 使用专项差速调试 Demo，它会接管底下的线程并周期打印
    // 想恢复正常跑时，注释这一句即可。
    // ============================================
    // lq_car_steer_debug_demo();
    // 29: 角速度环(中环) 输出差速验证演示程序
    // lq_gyro_loop_verify_demo();
    // 校赛无元素竞速模式：关闭元素识别/处理，只跑基础循迹闭环。
    // 若要重新调元素，再改回 lq_camera_tft18_demo() 或 34 号元素专项 Demo。
    g_enable_element_process = false;
    printf("[RACE_MODE] no-element race mode: element process disabled, main loop enabled.\n");
    
    // 调用摄像头拍照测试Demo并阻断后面的逻辑
    // lq_camera_capture_demo();

    // 调用巡线算法图传调试 Demo 并阻断后面的逻辑
    // lq_line_patrol_debug_demo();
    // lq_udp_img_trans_ex_demo();

    // UDP 图传初始化（192.168.90.22:8080）
    lq_udp_client udp_debug;
    udp_debug.udp_client_init("192.168.90.22", 8080);
    printf("[UDP_DEBUG] center-line overlay to 192.168.90.22:8080, every 5 frames\n");

    // ===== NCNN 推理初始化（走马观碑组） =====
    // 临时禁用：当前排查小车起步右转和速度异常，先确保主循环只跑基础循迹。
    // NcnnInference ncnn;
    // bool ncnn_ready = ncnn.init();  // 模型不存在时优雅降级，不阻塞主流程
    // if (!ncnn_ready) {
    //     printf("[NCNN] MODEL_NOT_FOUND — 推理功能已禁用，仅运行循迹。\n");
    //     printf("[NCNN] 请将 model.param 和 model.bin 放到工作目录。\n");
    // }

    // 分离线程，允许后台运行高速闭环控制代码
    std::thread threadA(timer_thread_1);

    int udp_frame_count = 0;
    // int ncnn_infer_count = 0;    // NCNN 推理帧计数器
    // std::string ncnn_last_label; // 最近一次 NCNN 推理结果

    while (!exit_flag)
    {
        // auto start2 = std::chrono::high_resolution_clock::now();

        // if (flags)
        // {
        //     start_time = getTickCount();
        //     flags = false;
        // }
        // fps_count++;

        // 图像二值化和压缩
        image_flag = image_cv_zip();
        if (image_flag == -1)   //如果图像获取失败，电机负压全部停止
        {
            motor_Duty_R(0);
            motor_Duty_L(0);
            // Fuya.SetDutyCycle(100000);
            emergency_stop();  // 图像获取失败时停止电机
            exit_flag = true;  // 通知子线程退出
            break;
        }
            
        // 按键
        // Key_Get();

        // 图像处理
        imageprocess();

        // ===== NCNN 推理（走马观碑：每 kNcnnInferInterval 帧执行一次） =====
        // 临时禁用：当前排查小车起步右转和速度异常，先不要让推理链路占用主循环。
        // if (ncnn_ready && ((ncnn_infer_count % car_config::kNcnnInferInterval) == 0)) {
        //     std::string label;
        //     if (ncnn.infer(image_cv_get_hires_frame(), label)) {
        //         if (label != ncnn_last_label) {
        //             printf("[NCNN] 推理结果: %s\n", label.c_str());
        //             ncnn_last_label = label;
        //         }
        //     }
        // }
        // ncnn_infer_count++;

        // UDP 图传：每5帧发送一次（调用 demo24 中的函数）
        if ((udp_frame_count % 5) == 0) {
            lq_udp_center_line_send_one_frame(udp_debug, 50);
        }
        udp_frame_count++;
        // image_show();
        

        // // 计算FPS
        // if ((fps_count % 10) == 0)
        // {
        //     end_time = (getTickCount() - start_time) / getTickFrequency();
        //     printf("fps:%0.3f\n", fps_count / end_time);
        //     start_time = getTickCount();
        //     fps_count = 0;
        //     flags = true;
        // }

        // auto end2 = std::chrono::high_resolution_clock::now();
        // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count();
        // std::cout << "耗时: " << duration2 << " 毫秒" << std::endl;
    }
    exit_flag = true;
    threadA.join();  // 等待底层硬实时线程结束
    emergency_stop();  // 最终确认电机停止
    gpio_protect_heartbeat_deinit();
    return 0;
}
