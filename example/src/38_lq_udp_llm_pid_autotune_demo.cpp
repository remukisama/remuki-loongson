#include "lq_all_demo.hpp"
#include "car_config.hpp"
#include "encoder.hpp"
#include "gpio_protect_heartbeat.hpp"
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <signal.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// ==========================================
// 【UDP 版 LLM 自动PID调参演示程序】
//
// 与主程序 Speed_BiHuan() 完全一致的：
//   • 5ms (200Hz) 控制周期
//   • 增量式 PI(D) 算法（非位置式）
//   • encoder_num_L/R() 滤波编码器
//   • [0, MAX_PWM] 单向输出限幅
// 配合 tuner.py --udp 使用
// ==========================================

// ---- UDP 配置 ----
static constexpr const char *UDP_PC_HOST    = "192.168.30.22";
static constexpr uint16_t    UDP_PC_PORT    = 9999;
static constexpr uint16_t    UDP_LOCAL_PORT = 9998;

static std::atomic<bool> g_exit(false);

static void sig_handler(int) { g_exit = true; }

// ==========================================
// UDP 收发通道（非阻塞接收）
// ==========================================
class UdpChannel {
public:
    bool init(uint16_t local_port, const char *remote_ip, uint16_t remote_port) {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) { perror("socket"); return false; }

        struct sockaddr_in la{};
        la.sin_family      = AF_INET;
        la.sin_port        = htons(local_port);
        la.sin_addr.s_addr = INADDR_ANY;
        if (bind(fd_, (struct sockaddr *)&la, sizeof(la)) < 0) {
            perror("bind"); close(fd_); fd_ = -1; return false;
        }

        int fl = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, fl | O_NONBLOCK);

        memset(&ra_, 0, sizeof(ra_));
        ra_.sin_family = AF_INET;
        ra_.sin_port   = htons(remote_port);
        inet_pton(AF_INET, remote_ip, &ra_.sin_addr);

        printf("# UDP ready: listen %u -> %s:%u\n",
               local_port, remote_ip, remote_port);
        return true;
    }

    void send(const char *buf, size_t len) {
        if (fd_ >= 0)
            sendto(fd_, buf, len, 0, (struct sockaddr *)&ra_, sizeof(ra_));
    }

    // 非阻塞接收一行指令，返回长度（0=无数据）
    ssize_t recv(char *buf, size_t maxlen) {
        if (fd_ < 0) return 0;
        struct pollfd p{fd_, POLLIN, 0};
        if (poll(&p, 1, 0) <= 0) return 0;
        ssize_t n = recvfrom(fd_, buf, maxlen - 1, 0, nullptr, nullptr);
        if (n > 0) {
            buf[n] = '\0';
            while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r')) buf[--n] = '\0';
        }
        return (n > 0) ? n : 0;
    }

    void close_sock() { if (fd_ >= 0) { close(fd_); fd_ = -1; } }

private:
    int fd_ = -1;
    struct sockaddr_in ra_{};
};

// ==========================================
// 电机模型：PWM→速度 分段线性查表
// 数据来源：--mode=motor-id 在赛道上采集
// ==========================================
//  架空测试时用此模型把 PWM 输出换算成"虚拟地面速度"
//  PID 以此为反馈，行为等效于在地面跑
struct MotorModel {
    static constexpr int N = 5;
    float pwm[N]  = {0,    4500,  5000,  6000,  7000};
    float spd[N]  = {0.0f, 0.0f,  0.253f,1.775f,2.136f};

    float operator()(float pwm_out) const {
        if (pwm_out <= 0) return 0.0f;
        float ap = pwm_out < 0 ? -pwm_out : pwm_out;
        if (ap <= pwm[0]) return spd[0];
        for (int i = 1; i < N; i++) {
            if (ap <= pwm[i]) {
                float t = (ap - pwm[i-1]) / (pwm[i] - pwm[i-1]);
                return spd[i-1] + t * (spd[i] - spd[i-1]);
            }
        }
        return spd[N-1];  // 饱和
    }
};

// ==========================================
// 增量式 PID（与主程序 Speed_PID_L/R 完全一致）
// ==========================================
struct IncrementalPID {
    float kp, ki, kd;
    float out;           // 当前输出累积值
    float last_error;    // e(k-1)
    float prev_error;    // e(k-2)
    float out_min, out_max;

    void reset() {
        out = 0.0f;
        last_error = 0.0f;
        prev_error = 0.0f;
    }

    // 与主程序 Speed_PID_L 完全同构
    float tick(float setpoint, float feedback) {
        float error = setpoint - feedback;

        if (setpoint <= 0.02f) {
            out = 0.0f;
            last_error = 0.0f;
            prev_error = 0.0f;
            return out;
        }

        const float delta =
            kp * (error - last_error) +
            ki * error +
            kd * (error - 2.0f * last_error + prev_error);

        out += delta;
        prev_error = last_error;
        last_error = error;

        if (out > out_max) out = out_max;
        if (out < out_min) out = out_min;
        return out;
    }
};

// ==========================================
// 解析 "SET P:xxx I:xxx D:xxx"
// ==========================================
static bool parse_set(const char *cmd, float &p, float &i, float &d) {
    return sscanf(cmd, "SET P:%f I:%f D:%f", &p, &i, &d) == 3;
}

// ==========================================
// 主入口
// ==========================================
void lq_udp_llm_pid_autotune_demo(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    printf("=========================================================\n");
    printf("  UDP LLM Auto PID Tune (200Hz incremental PID)\n");
    printf("  Period: 5ms | Algo: incremental | Encoder: filtered\n");
    printf("  UDP listen: %u -> send %s:%u\n",
           UDP_LOCAL_PORT, UDP_PC_HOST, UDP_PC_PORT);
    printf("=========================================================\n\n");

    // ---- 1. UDP 通道 ----
    UdpChannel udp;
    if (!udp.init(UDP_LOCAL_PORT, UDP_PC_HOST, UDP_PC_PORT)) {
        fprintf(stderr, "[ERROR] UDP init failed\n");
        return;
    }

    // ---- 2. 电机 PWM ----
    ls_atim_pwm motor_L_pwm(ATIM_PWM0_PIN81,
                            car_config::kMotorPwmFrequencyHz,
                            car_config::kMotorInitialDuty,
                            ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R_pwm(ATIM_PWM1_PIN82,
                            car_config::kMotorPwmFrequencyHz,
                            car_config::kMotorInitialDuty,
                            ATIM_PWM_POL_NORMAL);
    ls_gpio motor_L_dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_dir(PIN_22, GPIO_MODE_OUT);

    motor_L_pwm.atim_pwm_enable();
    motor_R_pwm.atim_pwm_enable();
    motor_L_dir.gpio_level_set(GPIO_LOW);
    motor_R_dir.gpio_level_set(GPIO_LOW);

    auto apply_motor = [&](ls_atim_pwm &pwm, ls_gpio &dir, float v) {
        const int max_pwm = car_config::kMotorMaxPwm;
        v = std::max(0.0f, std::min((float)max_pwm, v));
        dir.gpio_level_set(GPIO_LOW);           // 正转 DIR=LOW
        pwm.atim_pwm_set_active_low_duty((uint32_t)v);
    };

    auto stop_motors = [&]() {
        motor_L_pwm.atim_pwm_set_active_low_duty(0);
        motor_R_pwm.atim_pwm_set_active_low_duty(0);
    };

    // ---- 3. 增量式 PID 初始化（与 car_config 一致） ----
    IncrementalPID pid_L, pid_R;
    auto init_pid = [&](IncrementalPID &pid) {
        pid.kp       = 800.0f;                      // 故意给低，让 LLM 调
        pid.ki       = 10.0f;
        pid.kd       = 0.0f;
        pid.out      = 0.0f;
        pid.last_error = 0.0f;
        pid.prev_error = 0.0f;
        pid.out_min  = 0.0f;
        pid.out_max  = (float)car_config::kMotorMaxPwm;  // 8000
    };
    init_pid(pid_L);
    init_pid(pid_R);

    // ---- 4. 控制变量 ----
    MotorModel motor_model;                          // 电机模型（PWM→虚拟速度）
    float target_speed   = 0.45f;                    // 赛道实际速度
    float enc_speed_L    = 0.0f;                     // 原始编码器（仅日志用）
    float enc_speed_R    = 0.0f;
    float virtual_speed_L = 0.0f;                    // 模型估算速度（PID 反馈用）
    float virtual_speed_R = 0.0f;
    float pwm_L          = 0.0f;
    float pwm_R          = 0.0f;

    const long long kControlPeriodUs = 5000;  // 5ms = 200Hz（与主程序完全一致）
    unsigned long loop_count = 0;
    unsigned long telemetry_seq = 0;

    auto start_time = std::chrono::steady_clock::now();
    auto last_time  = start_time;

    // 初始化编码器（与主程序 encoder_reset_state 类似）
    encoder_num_L();
    encoder_num_R();

    udp.send("# UDP PID Autotuner Firmware v1.0 (200Hz incremental) Ready\n", 62);

    printf("# Initial PID: P=%.1f I=%.1f D=%.1f  target=%.2f m/s\n",
           pid_L.kp, pid_L.ki, pid_L.kd, target_speed);
    printf("# Control loop: 5ms (200Hz), incremental PID, filtered encoder\n\n");

    // ---- 5. 主控制循环 ----
    while (!g_exit.load())
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                           now - start_time);
        unsigned long timestamp_ms = (unsigned long)(elapsed.count() / 1000);

        // 5.1 读取原始编码器（仅用于日志，不参与 PID）
        enc_speed_L = encoder_num_L();
        enc_speed_R = encoder_num_R();

        // 5.2 用电机模型从上一拍 PWM 推算虚拟地面速度（PID 反馈）
        virtual_speed_L = motor_model(pwm_L);
        virtual_speed_R = motor_model(pwm_R);

        // 5.3 增量式 PID（反馈用虚拟速度，行为等效地面）
        pwm_L = pid_L.tick(target_speed, virtual_speed_L);
        pwm_R = pid_R.tick(target_speed, virtual_speed_R);

        // 5.4 电机输出
        apply_motor(motor_L_pwm, motor_L_dir, pwm_L);
        apply_motor(motor_R_pwm, motor_R_dir, pwm_R);

        // 5.4 心跳（维持 gpio_manager watchdog）
        gpio_protect_heartbeat_pulse();

        // 5.5 UDP 上报遥测（每 10ms 一帧，100Hz）
        //     上报虚拟速度（PID 实际看到的反馈），而非原始编码器
        if ((loop_count % 2) == 0) {
            float error = target_speed - virtual_speed_L;
            char buf[256];
            int n = snprintf(buf, sizeof(buf),
                "%lu,%.3f,%.3f,%.0f,%.3f,%.1f,%.1f,%.1f\n",
                timestamp_ms, target_speed, virtual_speed_L,
                pwm_L, error, pid_L.kp, pid_L.ki, pid_L.kd);
            if (n > 0) udp.send(buf, (size_t)n);
            ++telemetry_seq;
        }

        // 5.6 接收上位机 PID 指令
        char cmd[128];
        if (udp.recv(cmd, sizeof(cmd)) > 0) {
            float new_p, new_i, new_d;
            if (parse_set(cmd, new_p, new_i, new_d)) {
                pid_L.kp = new_p;  pid_L.ki = new_i;  pid_L.kd = new_d;
                pid_L.reset();
                pid_R.kp = new_p;  pid_R.ki = new_i;  pid_R.kd = new_d;
                pid_R.reset();

                char ack[128];
                int alen = snprintf(ack, sizeof(ack),
                    "# PID updated: P=%.1f I=%.1f D=%.1f\n", new_p, new_i, new_d);
                udp.send(ack, (size_t)alen);
                printf("%s", ack);
                fflush(stdout);
            }
        }

        // 5.7 周期日志（stderr，每 1s 即 200 次打印一次）
        ++loop_count;
        if ((loop_count % 200) == 0) {
            fprintf(stderr,
                "# [T=%lums] tgt=%.2f encL=%.2f encR=%.2f pwmL=%.0f pwmR=%.0f P=%.1f I=%.1f D=%.1f seq=%lu\n",
                timestamp_ms, target_speed,
                enc_speed_L, enc_speed_R,
                pwm_L, pwm_R,
                pid_L.kp, pid_L.ki, pid_L.kd,
                telemetry_seq);
        }

        // 5.8 高精度 5ms 定时（与主程序 sleep_until 一致）
        auto next = start_time + std::chrono::microseconds(
                        (long long)(loop_count + 1) * kControlPeriodUs);
        std::this_thread::sleep_until(next);
    }

    // ---- 6. 清理 ----
    stop_motors();
    udp.send("# STOPPED\n", 10);
    udp.close_sock();

    printf("\n# Demo stopped. Final PID: P=%.1f I=%.1f D=%.1f\n",
           pid_L.kp, pid_L.ki, pid_L.kd);
    printf("# Total loops: %lu  Telemetry frames: %lu\n",
           loop_count, telemetry_seq);
}
