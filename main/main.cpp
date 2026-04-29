#include "main.hpp"
#include "thread"
// HWGpio beep(61, GPIO_Mode_Out);
ls_gpio beep(PIN_61, GPIO_MODE_OUT);


bool flags = true;     // 判断第一次进入函数标志位
uint8_t fps_count = 0; // 帧数累计
double start_time = 0; // 开始时间
double end_time = 0;   // 结束时间

// 定义定时器间隔（5ms）
const auto timer_interval_1 = std::chrono::milliseconds(5);

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


int main()
{
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
    lq_gyro_loop_verify_demo();
    return 0;
    char image_flag = 0;

    beep.gpio_level_set(GPIO_LOW); // 关闭蜂鸣器

    // /**************************
    //  * 关键：注册全量异常信号捕获
    //  **************************/
    signal(SIGINT, signal_handler);   // 捕获Ctrl+C
    signal(SIGTERM, signal_handler);  // 捕获系统kill命令
    signal(SIGHUP, signal_handler);   // 捕获SSH断开/终端被关掉的情况
    signal(SIGABRT, signal_handler);  // 捕获程序自己 abort 崩溃的情况
    signal(SIGQUIT, signal_handler);  // 捕获 Ctrl+\ 退出
    signal(SIGSEGV, signal_handler);  // 捕获段错误(访问了空指针等崩溃)
    signal(SIGPIPE, SIG_IGN);         // 忽略破损管道信号(当图传UDP发送失败时忽略之，不退出程序)

    // InitMH();       //  模糊pid初始化
    motor_Init();     // 电机初始化
    Speed_decision(); // 电机PID参数初始化
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
    // 31: 联合位置环与角速度环调参，重点观测数据输出
    // lq_pos_pid_tuning_demo();
    
    // 调用摄像头拍照测试Demo并阻断后面的逻辑
    // lq_camera_capture_demo();

    // 调用巡线算法图传调试 Demo 并阻断后面的逻辑
    // lq_line_patrol_debug_demo();
    // lq_udp_img_trans_ex_demo();
    // return 0;

    // 以下代码因为上面的 return 0 已经不会被执行到了。
    // 如果您需要更彻底，可以把这里到文件末尾的线程(timer_thread_1)等都注释起来。
    
    // 分离线程，允许后台运行高速闭环控制代码
    std::thread threadA(timer_thread_1);
    threadA.detach();

    while (1)
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
    return 0;
}
