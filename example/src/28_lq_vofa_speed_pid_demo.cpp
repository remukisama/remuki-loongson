#include "lq_all_demo.hpp"
#include "lq_udp_client.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <cstdio>
#include <csignal>
#include <cmath>
#include <sys/select.h>
#include <string>
#include <sstream>

// ==========================================
// 【基于 VOFA+ 的速度环(PID) 波形调参演示程序】
//   所有速度单位统一为 m/s（轮胎直径 64mm）
// ==========================================

// --- 轮胎 & 齿轮参数 ---
// 轮胎直径 64mm, 车轮大齿轮 68 齿, 编码器小齿轮 30 齿
// 编码器返回电机轴 RPS, 换算为车轮线速度 (m/s):
//   wheel_RPS = motor_RPS × (30/68)
//   m/s       = wheel_RPS × π × 0.064
//   ENC_TO_MS = motor_RPS × (30/68) × π × 0.064 ≈ motor_RPS × 0.08870
#define WHEEL_DIAMETER_M       ( 0.064f )
#define GEAR_TEETH_WHEEL       ( 68 )
#define GEAR_TEETH_ENCODER     ( 30 )
#define GEAR_RATIO             ( (float)GEAR_TEETH_ENCODER / (float)GEAR_TEETH_WHEEL )  // 30/68
#define WHEEL_CIRCUMFERENCE_M  ( (float)M_PI * WHEEL_DIAMETER_M )                        // ≈ 0.20106 m
#define ENC_TO_MS(rps)         ( (float)(rps) * GEAR_RATIO * WHEEL_CIRCUMFERENCE_M )     // ≈ rps × 0.08870

static constexpr float kTunedSpeedKp = 1800.0f;
static constexpr float kTunedSpeedKi = 35.0f;
static constexpr float kTargetAccelLimit = 1.8f;  // m/s^2, 0 -> 0.9m/s 约 0.5s
static constexpr float kTargetDecelLimit = 2.0f;  // m/s^2, 略强一点便于下降段收速

static inline float slew_limit(float current, float desired, float accel_limit, float decel_limit, float dt_sec)
{
    float delta = desired - current;
    float limit = (delta >= 0.0f ? accel_limit : decel_limit) * dt_sec;
    if (delta > limit) return current + limit;
    if (delta < -limit) return current - limit;
    return desired;
}

// 用于安全退出
static std::atomic<bool> exit_vofa_demo(false);

void signal_handler_vofa_tuning(int signum) {
    (void)signum;
    exit_vofa_demo = true;
}

// 可在下面修改 VOFA+ 服务器（PC端）的 IP 和 端口
#define SERVER_IP       "192.168.90.22"
#define SERVER_PORT     8080

// ==========================================
// 1. 固定目标速度 PID 调参 (m/s)
// ==========================================
void lq_vofa_speed_pid_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);

    std::cout << "========== 基于 UDP/FireWater 的速度环波形调参演示程序启动 ==========" << std::endl;
    std::cout << ">>> UDP 目标 IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> UDP 目标 Port: " << SERVER_PORT << std::endl;
    std::cout << ">>> 轮胎周长: " << WHEEL_CIRCUMFERENCE_M << " m  (直径 " << WHEEL_DIAMETER_M << " m)" << std::endl;
    std::cout << ">>> 速度单位: m/s" << std::endl;
    std::cout << "按 Ctrl+C 安全退出并停止电机。" << std::endl;

    // 1. 初始化 UDP 客户端
    lq_udp_client client;
    client.udp_client_init(SERVER_IP, SERVER_PORT);
    std::cout << ">>> UDP 波形发送已初始化。" << std::endl;

    // 2. 初始化电机 PWM 控制
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_INV);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_INV);
    
    // 3. 初始化电机方向控制引脚
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    auto stop_motors = [&]() {
        motor_L.atim_pwm_set_duty(0);
        motor_R.atim_pwm_set_duty(0);
    };

    auto shutdown_motors = [&]() {
        stop_motors();
        motor_L.atim_pwm_disable();
        motor_R.atim_pwm_disable();
    };

    // 4. 初始化编码器
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // --- PID 调参变量区 (单位: m/s) ---
    // 【请在这里修改您的 PID 参数】
    float Kp_L = kTunedSpeedKp;
    float Ki_L = kTunedSpeedKi;
    float Kd_L = 0.0f;
    float target_speed_L = 1.0f;  // 左轮目标线速度 (m/s)

    float Kp_R = kTunedSpeedKp;
    float Ki_R = kTunedSpeedKi;
    float Kd_R = 0.0f;
    float target_speed_R = 1.0f;  // 右轮目标线速度 (m/s)

    int MAX_PWM = 7000;           // PWM 最大限制

    // PID 状态变量
    float speed_out_L = 0.0f;
    float last_error_L = 0.0f;
    float speed_out_R = 0.0f;
    float last_error_R = 0.0f;

    int loop_count = 0;

    while(!exit_vofa_demo)
    {
        // ==============================================
        // 1. 获取当前速度 (RPS → m/s)
        // ==============================================
        float current_speed_L = ENC_TO_MS(-encoder_L.encoder_get_count());
        float current_speed_R = ENC_TO_MS( encoder_R.encoder_get_count());

        // ==============================================
        // 2. 计算误差
        // ==============================================
        float error_L = target_speed_L - current_speed_L;
        float error_R = target_speed_R - current_speed_R;

        // ==============================================
        // 3. 增量式(近似) PID 计算
        // ==============================================
        if (target_speed_L == 0.0f) {
            speed_out_L = 0.0f;
            last_error_L = 0.0f;
            error_L = 0.0f;
        } else {
            speed_out_L += Ki_L * error_L + Kp_L * (error_L - last_error_L)
                         + Kd_L * (error_L - 2.0f * last_error_L + current_speed_L);
            last_error_L = error_L;
        }

        if (target_speed_R == 0.0f) {
            speed_out_R = 0.0f;
            last_error_R = 0.0f;
            error_R = 0.0f;
        } else {
            speed_out_R += Ki_R * error_R + Kp_R * (error_R - last_error_R)
                         + Kd_R * (error_R - 2.0f * last_error_R + current_speed_R);
            last_error_R = error_R;
        }

        // ==============================================
        // 4. PWM 限幅
        // ==============================================
        if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        if (target_speed_L == 0.0f && target_speed_R == 0.0f) {
            stop_motors();
        } else {
            // ==============================================
            // 5. 电机方向控制及 PWM 占空比输出
            // ==============================================
            if (speed_out_L >= 0.0f) {
                motor_L_Dir.gpio_level_set(GPIO_LOW);
                motor_L.atim_pwm_set_duty((int)speed_out_L);
            } else {
                motor_L_Dir.gpio_level_set(GPIO_HIGH);
                motor_L.atim_pwm_set_duty((int)(-speed_out_L));
            }

            if (speed_out_R >= 0.0f) {
                motor_R_Dir.gpio_level_set(GPIO_LOW);
                motor_R.atim_pwm_set_duty((int)speed_out_R);
            } else {
                motor_R_Dir.gpio_level_set(GPIO_HIGH);
                motor_R.atim_pwm_set_duty((int)(-speed_out_R));
            }
        }

        // ==============================================
        // 6. 通过 VOFA+ 客户端发送波形数据
        //    FireWater 协议: ware:目标L,实际L,目标R,实际R\n
        //    单位: m/s
        // ==============================================
        char send_buf[128];
        int len = snprintf(send_buf, sizeof(send_buf), "ware:%.3f,%.3f,%.3f,%.3f\n",
                           target_speed_L, current_speed_L,
                           target_speed_R, current_speed_R);
        if (len > 0) {
            client.udp_send(send_buf, len);
        }

        // 控制台打印 (降低频率)
        loop_count++;
        if (loop_count >= 50) {
            std::cout << ">>> 正在持续发送 VOFA 波形数据..."
                      << " Target L: " << target_speed_L << " m/s"
                      << ", R: " << target_speed_R << " m/s" << std::endl;
            loop_count = 0;
        }

        // 控制周期 20ms (闭环频率 50Hz)
        usleep(20000);
    }

    std::cout << "正在关闭电机与连接..." << std::endl;
    shutdown_motors();
    client.udp_close();
}

// ==========================================
// 2. 交互式阶跃速度测试 (m/s)
//    终端输入目标速度/时长/[Kp]/[Ki] 执行单次阶跃，
//    等待期间目标速度为 0，仅通过 UDP 发送波形。
//    格式: <目标速度(m/s)> <持续时间(ms)> [Kp] [Ki]
//    示例: 1.5 3000            (1.5 m/s 跑 3 秒)
//    示例: 2.0 5000 700 320    (2.0 m/s 跑 5 秒, Kp=700 Ki=320)
// ==========================================
void lq_vofa_speed_step_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);

    std::cout << "========== 交互式阶跃速度测试程序启动 ==========" << std::endl;
    std::cout << ">>> UDP 目标 IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> UDP 目标 Port: " << SERVER_PORT << std::endl;
    std::cout << ">>> 轮胎周长: " << WHEEL_CIRCUMFERENCE_M << " m  (直径 " << WHEEL_DIAMETER_M << " m)" << std::endl;
    std::cout << ">>> 速度单位: m/s" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << ">>> 格式: <目标速度(m/s)> <持续时间(ms)> [Kp] [Ki]" << std::endl;
    std::cout << ">>> 示例: 1.5 3000          (保持当前 PI)" << std::endl;
    std::cout << ">>> 示例: 2.0 5000 700 320  (同时更新 PI)" << std::endl;
    std::cout << ">>> 输入 q 或 quit 退出" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << "按 Ctrl+C 安全退出并停止电机。" << std::endl;

    // 1. 初始化 UDP 客户端
    lq_udp_client client;
    client.udp_client_init(SERVER_IP, SERVER_PORT);

    // 2. 初始化电机 PWM 控制
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_INV);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_INV);
    
    // 3. 初始化电机方向控制引脚
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    auto stop_motors = [&]() {
        motor_L.atim_pwm_set_duty(0);
        motor_R.atim_pwm_set_duty(0);
    };

    auto shutdown_motors = [&]() {
        stop_motors();
        motor_L.atim_pwm_disable();
        motor_R.atim_pwm_disable();
    };

    // 4. 初始化编码器
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // --- PID 调参变量区 (单位: m/s) ---
    float Kp_L = kTunedSpeedKp;
    float Ki_L = kTunedSpeedKi;
    float Kd_L = 0.0f;

    float Kp_R = kTunedSpeedKp;
    float Ki_R = kTunedSpeedKi;
    float Kd_R = 0.0f;

    int MAX_PWM = 7000;

    // --- 任务状态 ---
    float target_speed_L   = 0.0f;
    float target_speed_R   = 0.0f;
    int   task_remaining_ms = 0;       // 剩余运行时间 (ms)，0 表示无任务

    // --- PID 运行时状态 ---
    float speed_out_L  = 0.0f;
    float speed_out_R  = 0.0f;
    float last_error_L = 0.0f;
    float last_error_R = 0.0f;

    // --- 终端输入缓冲区 ---
    std::string input_buf;
    bool prompt_shown = false;

    auto show_prompt = [&]() {
        if (!prompt_shown) {
            std::cout << ">>> [等待输入] 当前 Kp=" << Kp_L << " Ki=" << Ki_L
                      << " | 输入目标速度(m/s) 时长(ms) [Kp] [Ki]: " << std::flush;
            prompt_shown = true;
        }
    };

    show_prompt();

    while(!exit_vofa_demo)
    {
        // ==============================================
        // 非阻塞检查终端输入
        // ==============================================
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 0};
        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0)
        {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0)
            {
                if (c == '\n' || c == '\r')
                {
                    if (!input_buf.empty())
                    {
                        // 去掉首尾空白
                        while (!input_buf.empty() && input_buf.back() == '\r') input_buf.pop_back();

                        if (input_buf == "q" || input_buf == "quit")
                        {
                            std::cout << ">>> 用户请求退出。" << std::endl;
                            exit_vofa_demo = true;
                            input_buf.clear();
                            continue;
                        }

                        // 解析: speed duration [Kp] [Ki]
                        std::istringstream iss(input_buf);
                        float in_speed = 0.0f;
                        int   in_duration = 0;
                        float in_Kp = 0.0f;
                        float in_Ki = 0.0f;

                        if (iss >> in_speed >> in_duration)
                        {
                            task_remaining_ms = in_duration;
                            target_speed_L   = in_speed;
                            target_speed_R   = in_speed;

                            // 可选 PI 参数: 输入了 Kp 才更新
                            if (iss >> in_Kp)
                            {
                                Kp_L = in_Kp;
                                Kp_R = in_Kp;
                                if (iss >> in_Ki)
                                {
                                    Ki_L = in_Ki;
                                    Ki_R = in_Ki;
                                }
                            }

                            speed_out_L  = 0.0f;
                            speed_out_R  = 0.0f;
                            last_error_L = 0.0f;
                            last_error_R = 0.0f;

                            std::cout << ">>> [执行] 目标=" << target_speed_L << " m/s"
                                      << ", 时长=" << task_remaining_ms << " ms"
                                      << ", Kp=" << Kp_L << ", Ki=" << Ki_L << std::endl;
                            prompt_shown = false;  // 任务完成后重新显示提示
                        }
                        else
                        {
                            std::cout << ">>> [错误] 无法解析输入: " << input_buf << std::endl;
                        }

                        input_buf.clear();
                    }
                }
                else
                {
                    input_buf += c;
                }
            }
        }

        // ==============================================
        // 任务计时: 到期自动将目标速度归零
        // ==============================================
        if (task_remaining_ms > 0)
        {
            task_remaining_ms -= 20;
            if (task_remaining_ms <= 0)
            {
                task_remaining_ms = 0;
                target_speed_L = 0.0f;
                target_speed_R = 0.0f;
                speed_out_L  = 0.0f;
                speed_out_R  = 0.0f;
                last_error_L = 0.0f;
                last_error_R = 0.0f;
                std::cout << ">>> [完成] 任务结束，目标速度已归零。" << std::endl;
                show_prompt();
            }
        }

        // ==============================================
        // 1. 获取当前速度 (RPS → m/s)
        // ==============================================
        float current_speed_L = ENC_TO_MS(-encoder_L.encoder_get_count());
        float current_speed_R = ENC_TO_MS( encoder_R.encoder_get_count());

        // ==============================================
        // 2. 计算误差
        // ==============================================
        float error_L = target_speed_L - current_speed_L;
        float error_R = target_speed_R - current_speed_R;

        // ==============================================
        // 3. 增量式(近似) PID 计算（目标=0 时也运行，主动制动维持速度为零）
        // ==============================================
        speed_out_L += Ki_L * error_L + Kp_L * (error_L - last_error_L)
                     + Kd_L * (error_L - 2.0f * last_error_L + current_speed_L);
        last_error_L = error_L;

        speed_out_R += Ki_R * error_R + Kp_R * (error_R - last_error_R)
                     + Kd_R * (error_R - 2.0f * last_error_R + current_speed_R);
        last_error_R = error_R;

        // ==============================================
        // 4. PWM 限幅
        // ==============================================
        if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        // ==============================================
        // 5. 电机方向控制及 PWM 占空比输出
        // ==============================================
        if (speed_out_L >= 0.0f) {
            motor_L_Dir.gpio_level_set(GPIO_LOW);
            motor_L.atim_pwm_set_duty((int)speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_HIGH);
            motor_L.atim_pwm_set_duty((int)(-speed_out_L));
        }

        if (speed_out_R >= 0.0f) {
            motor_R_Dir.gpio_level_set(GPIO_LOW);
            motor_R.atim_pwm_set_duty((int)speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_HIGH);
            motor_R.atim_pwm_set_duty((int)(-speed_out_R));
        }

        // ==============================================
        // 6. 通过 VOFA+ 客户端发送波形数据 (m/s)
        //    (始终发送，不阻塞)
        // ==============================================
        char send_buf[128];
        int len = snprintf(send_buf, sizeof(send_buf), "ware:%.3f,%.3f,%.3f,%.3f\n",
                           target_speed_L, current_speed_L,
                           target_speed_R, current_speed_R);
        if (len > 0) {
            client.udp_send(send_buf, len);
        }

        usleep(20000);
    }

    std::cout << "正在关闭电机与连接..." << std::endl;
    shutdown_motors();
    client.udp_close();
}

// ==========================================
// 3. 交互式实地速度波形测试 (m/s)
//    终端输入最低速度/最高速度/周期/时长/[Kp]/[Ki]，
//    目标按半余弦曲线在[min,max]之间平滑往返，等待期间目标速度为 0。
//    公式: target = min + (max - min) * 0.5 * (1 - cos(2π * t / period))
//    格式: <最低速度(m/s)> <最高速度(m/s)> <周期(s)> <运行时长(ms)> [Kp] [Ki]
//    示例: 0.9 1.3 2.0 4000          (0.9~1.3 m/s, 周期 2s, 跑 4s)
//    示例: 0.9 1.3 2.0 4000 1800 35  (同时更新 PI)
// ==========================================
void lq_vofa_speed_sine_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);

    std::cout << "========== VOFA 实地速度波形测试程序启动 ==========" << std::endl;
    std::cout << ">>> UDP 目标 IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> UDP 目标 Port: " << SERVER_PORT << std::endl;
    std::cout << ">>> 轮胎周长: " << WHEEL_CIRCUMFERENCE_M << " m  (直径 " << WHEEL_DIAMETER_M << " m)" << std::endl;
    std::cout << ">>> 速度单位: m/s" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << ">>> 默认 PI: Kp=" << kTunedSpeedKp << " Ki=" << kTunedSpeedKi << std::endl;
    std::cout << ">>> 公式: target = min + (max - min) * 0.5 * (1 - cos(2π * t / period))" << std::endl;
    std::cout << ">>> 格式: <最低速度> <最高速度> <周期(s)> <运行时长(ms)> [Kp] [Ki]" << std::endl;
    std::cout << ">>> 示例: 0.9 1.3 2.0 4000          (0.9~1.3 m/s, 2s 周期, 跑 4s)" << std::endl;
    std::cout << ">>> 示例: 0.9 1.3 2.0 4000 1800 35  (同时更新 PI)" << std::endl;
    std::cout << ">>> 输入 q 或 quit 退出" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << "按 Ctrl+C 安全退出并停止电机。" << std::endl;

    // 1. 初始化 UDP 客户端
    lq_udp_client client;
    client.udp_client_init(SERVER_IP, SERVER_PORT);

    // 2. 初始化电机 PWM 控制
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_INV);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_INV);
    
    // 3. 初始化电机方向控制引脚
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();

    auto stop_motors = [&]() {
        motor_L.atim_pwm_set_duty(0);
        motor_R.atim_pwm_set_duty(0);
    };

    auto shutdown_motors = [&]() {
        stop_motors();
        motor_L.atim_pwm_disable();
        motor_R.atim_pwm_disable();
    };

    // 4. 初始化编码器
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // --- PID 调参变量区 (单位: m/s) ---
    float Kp_L = kTunedSpeedKp;
    float Ki_L = kTunedSpeedKi;
    float Kd_L = 0.0f;

    float Kp_R = kTunedSpeedKp;
    float Ki_R = kTunedSpeedKi;
    float Kd_R = 0.0f;

    int MAX_PWM = 8000;
    const float DT_SEC = 0.02f;   // 控制周期 20ms

    // --- 任务状态 ---
    float target_speed_L   = 0.0f;
    float target_speed_R   = 0.0f;
    float desired_speed_L  = 0.0f;
    float desired_speed_R  = 0.0f;
    float sine_base        = 0.9f;    // 最低速度 (m/s)
    float sine_amplitude   = 0.4f;    // 速度范围 = max - min (m/s)
    float sine_period_s    = 2.0f;    // 速度曲线周期 (s)
    int   task_total_ms     = 0;      // 本次任务总时长 (ms)，用于终端日志时间轴
    int   task_remaining_ms = 0;      // 剩余运行时间 (ms)，0 表示无任务
    uint64_t sine_tick = 0;           // 正弦相位计数器

    // --- PID 运行时状态 ---
    float speed_out_L  = 0.0f;
    float speed_out_R  = 0.0f;
    float last_error_L = 0.0f;
    float last_error_R = 0.0f;

    // --- 终端输入缓冲区 ---
    std::string input_buf;
    bool prompt_shown = false;

    auto show_prompt = [&]() {
        if (!prompt_shown) {
            std::cout << ">>> [等待输入] 当前 Kp=" << Kp_L << " Ki=" << Ki_L
                      << " | 输入 最低 最高 周期(s) 时长(ms) [Kp] [Ki]: " << std::flush;
            prompt_shown = true;
        }
    };

    show_prompt();

    while(!exit_vofa_demo)
    {
        // ==============================================
        // 非阻塞检查终端输入
        // ==============================================
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 0};
        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0)
        {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0)
            {
                if (c == '\n' || c == '\r')
                {
                    if (!input_buf.empty())
                    {
                        while (!input_buf.empty() && input_buf.back() == '\r') input_buf.pop_back();

                        if (input_buf == "q" || input_buf == "quit")
                        {
                            std::cout << ">>> 用户请求退出。" << std::endl;
                            exit_vofa_demo = true;
                            input_buf.clear();
                            continue;
                        }

                        // 解析: min max period_s duration [Kp] [Ki]
                        std::istringstream iss(input_buf);
                        float in_base     = 0.0f;
                        float in_max      = 0.0f;
                        float in_period   = 0.0f;
                        int   in_duration = 0;
                        float in_Kp = 0.0f;
                        float in_Ki = 0.0f;

                        if (iss >> in_base >> in_max >> in_period >> in_duration)
                        {
                            if (in_base < 0.2f) in_base = 0.2f;
                            if (in_max  < in_base) in_max = in_base;
                            if (in_period < 1.0f) in_period = 1.0f;

                            sine_base      = in_base;
                            sine_amplitude = in_max - in_base;
                            sine_period_s  = in_period;
                            task_total_ms = in_duration;
                            task_remaining_ms = in_duration;
                            sine_tick = 0;

                            // 可选 PI 参数
                            if (iss >> in_Kp)
                            {
                                Kp_L = in_Kp;
                                Kp_R = in_Kp;
                                if (iss >> in_Ki)
                                {
                                    Ki_L = in_Ki;
                                    Ki_R = in_Ki;
                                }
                            }

                            speed_out_L  = 0.0f;
                            speed_out_R  = 0.0f;
                            last_error_L = 0.0f;
                            last_error_R = 0.0f;
                            target_speed_L = 0.0f;
                            target_speed_R = 0.0f;
                            desired_speed_L = 0.0f;
                            desired_speed_R = 0.0f;

                            std::cout << ">>> [执行] 最低=" << sine_base << " m/s"
                                      << ", 最高=" << in_max << " m/s"
                                      << ", 周期=" << sine_period_s << " s"
                                      << ", 时长=" << task_remaining_ms << " ms"
                                      << ", Kp=" << Kp_L << ", Ki=" << Ki_L << std::endl;
                            std::printf("[T28_HEADER] period_ms=20 fields=t_ms,rem_ms,target,desired,encL,encR,errL,errR,pwmL,pwmR,kp,ki,rawL,rawR,ramp\n");
                            std::fflush(stdout);
                            prompt_shown = false;
                        }
                        else
                        {
                            std::cout << ">>> [错误] 无法解析输入: " << input_buf << std::endl;
                        }

                        input_buf.clear();
                    }
                }
                else
                {
                    input_buf += c;
                }
            }
        }

        // ==============================================
        // 任务计时: 到期自动将目标速度归零
        // ==============================================
        if (task_remaining_ms > 0)
        {
            task_remaining_ms -= 20;
            if (task_remaining_ms <= 0)
            {
                task_remaining_ms = 0;
                task_total_ms = 0;
                target_speed_L = 0.0f;
                target_speed_R = 0.0f;
                desired_speed_L = 0.0f;
                desired_speed_R = 0.0f;
                speed_out_L  = 0.0f;
                speed_out_R  = 0.0f;
                last_error_L = 0.0f;
                last_error_R = 0.0f;
                sine_tick = 0;
                std::cout << ">>> [完成] 任务结束，目标速度已归零。" << std::endl;
                show_prompt();
            }
        }

        // ==============================================
        // 计算实地速度目标 (m/s): 在最低/最高速度之间平滑往返。
        // ==============================================
        if (task_remaining_ms > 0)
        {
            float t = (float)sine_tick * DT_SEC;
            float phase = 2.0f * (float)M_PI * t / sine_period_s;
            float profile = 0.5f * (1.0f - cosf(phase));
            desired_speed_L = sine_base + sine_amplitude * profile;
            desired_speed_R = desired_speed_L;
            target_speed_L = slew_limit(target_speed_L, desired_speed_L,
                                        kTargetAccelLimit, kTargetDecelLimit, DT_SEC);
            target_speed_R = slew_limit(target_speed_R, desired_speed_R,
                                        kTargetAccelLimit, kTargetDecelLimit, DT_SEC);
        }

        // ==============================================
        // 1. 获取当前速度 (RPS → m/s)
        // ==============================================
        float raw_speed_L = ENC_TO_MS(-encoder_L.encoder_get_count());
        float raw_speed_R = ENC_TO_MS( encoder_R.encoder_get_count());
        float current_speed_L = raw_speed_L;
        float current_speed_R = raw_speed_R;

        // ==============================================
        // 2. 计算误差
        // ==============================================
        float error_L = target_speed_L - current_speed_L;
        float error_R = target_speed_R - current_speed_R;

        // ==============================================
        // 3. 增量式(近似) PID 计算（目标=0 时也运行，主动制动维持速度为零）
        // ==============================================
        speed_out_L += Ki_L * error_L + Kp_L * (error_L - last_error_L)
                     + Kd_L * (error_L - 2.0f * last_error_L + current_speed_L);
        last_error_L = error_L;

        speed_out_R += Ki_R * error_R + Kp_R * (error_R - last_error_R)
                     + Kd_R * (error_R - 2.0f * last_error_R + current_speed_R);
        last_error_R = error_R;

        // ==============================================
        // 4. PWM 限幅
        // ==============================================
        if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        // ==============================================
        // 5. 电机方向控制及 PWM 占空比输出
        // ==============================================
        if (speed_out_L >= 0.0f) {
            motor_L_Dir.gpio_level_set(GPIO_LOW);
            motor_L.atim_pwm_set_duty((int)speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_HIGH);
            motor_L.atim_pwm_set_duty((int)(-speed_out_L));
        }

        if (speed_out_R >= 0.0f) {
            motor_R_Dir.gpio_level_set(GPIO_LOW);
            motor_R.atim_pwm_set_duty((int)speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_HIGH);
            motor_R.atim_pwm_set_duty((int)(-speed_out_R));
        }

        // ==============================================
        // 6. 通过 VOFA+ 客户端发送波形数据 (m/s)
        // ==============================================
        char send_buf[128];
        int len = snprintf(send_buf, sizeof(send_buf), "ware:%.3f,%.3f,%.3f,%.3f\n",
                           target_speed_L, current_speed_L,
                           target_speed_R, current_speed_R);
        if (len > 0) {
            client.udp_send(send_buf, len);
        }

        if (task_remaining_ms > 0) {
            int elapsed_ms = task_total_ms - task_remaining_ms;
            std::printf("[T28_LOG] t_ms=%d rem_ms=%d target=%.3f desired=%.3f encL=%.3f encR=%.3f errL=%.3f errR=%.3f pwmL=%.1f pwmR=%.1f kp=%.1f ki=%.1f rawL=%.3f rawR=%.3f ramp=1\n",
                        elapsed_ms, task_remaining_ms, target_speed_L, desired_speed_L,
                        current_speed_L, current_speed_R,
                        error_L, error_R,
                        speed_out_L, speed_out_R,
                        Kp_L, Ki_L,
                        raw_speed_L, raw_speed_R);
            std::fflush(stdout);
        }

        usleep(20000);
        if (task_remaining_ms > 0) sine_tick++;
    }

    std::cout << "正在关闭电机与连接..." << std::endl;
    shutdown_motors();
    client.udp_close();
}
