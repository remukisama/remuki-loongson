// 39_lq_motor_id_demo.cpp — 电机辨识：PWM 阶跃扫描，采集 PWM→速度映射
// 用途：在赛道上跑一次，采集摩擦力/负载模型，供架空 LLM 调参使用
// 用法：--mode=motor-id

#include "lq_all_demo.hpp"
#include "user_app.hpp"
#include "car_config.hpp"
#include "gpio_protect_heartbeat.hpp"
#include "lq_udp_client.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <sys/select.h>

extern std::atomic<bool> exit_flag;
extern void motor_Duty_L(int duty);
extern void motor_Duty_R(int duty);
extern void motor_Init(void);

// ── 配置 ────────────────────────────────────────────────────
static constexpr const char *UDP_PC_HOST = "192.168.30.22";
static constexpr uint16_t    UDP_PC_PORT = 9999;

static constexpr int PWM_STEPS[]  = {200, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 5000, 6000, 7000, 8000};
static constexpr int NUM_STEPS    = sizeof(PWM_STEPS) / sizeof(PWM_STEPS[0]);
static constexpr int WARMUP_MS    = 1500;   // 预热窗口
static constexpr int SAMPLE_MS    = 2000;   // 采样窗口
static constexpr int SETTLE_MS    = 1500;   // 步间停车稳定
static constexpr int SAMPLE_PERIOD_MS = 20; // 50Hz 采样

// ── sleep with heartbeat ───────────────────────────────────
static void safe_sleep(int ms)
{
    while (ms > 0 && !exit_flag.load()) {
        int chunk = std::min(ms, (int)car_config::kGpioProtectHeartbeatIntervalMs);
        gpio_protect_heartbeat_pulse();
        timeval tv{0, chunk * 1000};
        select(0, nullptr, nullptr, nullptr, &tv);
        ms -= chunk;
    }
}

// ── 采集一个 PWM 阶跃的稳态速度 ──────────────────────────
struct StepResult {
    int   pwm;
    float avg_speed_L;
    float avg_speed_R;
    float avg_speed;
    int   samples;
};

static StepResult measure_step(int pwm)
{
    motor_Duty_L(pwm);
    motor_Duty_R(pwm);

    // 预热
    safe_sleep(WARMUP_MS);

    // 采样
    float sum_L = 0, sum_R = 0;
    int   cnt   = 0;
    int   elapsed = 0;

    while (elapsed < SAMPLE_MS && !exit_flag.load()) {
        float sL = (float)encoder_num_L();
        float sR = (float)encoder_num_R();
        if (sL < 0) sL = -sL;
        if (sR < 0) sR = -sR;
        sum_L += sL;
        sum_R += sR;
        cnt++;
        gpio_protect_heartbeat_pulse();
        timeval tv{0, SAMPLE_PERIOD_MS * 1000};
        select(0, nullptr, nullptr, nullptr, &tv);
        elapsed += SAMPLE_PERIOD_MS;
    }

    StepResult r;
    r.pwm = pwm;
    r.avg_speed_L = cnt > 0 ? sum_L / cnt : 0;
    r.avg_speed_R = cnt > 0 ? sum_R / cnt : 0;
    r.avg_speed   = cnt > 0 ? (sum_L + sum_R) / (2.0f * cnt) : 0;
    r.samples     = cnt;
    return r;
}

// ── 主函数 ─────────────────────────────────────────────────
void lq_motor_id_demo(void)
{
    std::printf("\n============================================================\n");
    std::printf("  电机辨识 (Motor Identification)\n");
    std::printf("  PWM 阶跃扫描: %d 步, 每步 %d ms (warmup %d + sample %d)\n",
                NUM_STEPS, WARMUP_MS + SAMPLE_MS, WARMUP_MS, SAMPLE_MS);
    std::printf("  目标: 采集 PWM→速度映射，用于架空 LLM 调参模型\n");
    std::printf("============================================================\n\n");

    // 1. 初始化
    gpio_protect_heartbeat_init();
    motor_Init();
    encoder_reset_state();

    lq_udp_client udp;
    udp.udp_client_init(std::string(UDP_PC_HOST), UDP_PC_PORT);

    motor_Duty_L(0);
    motor_Duty_R(0);
    safe_sleep(500);

    if (!gpio_protect_heartbeat_require_ready(1500)) {
        motor_Duty_L(0);
        motor_Duty_R(0);
        gpio_protect_heartbeat_deinit();
        return;
    }

    // 2. 扫描
    std::printf("# motor_id: pwm,avg_speed_L,avg_speed_R,avg_speed,samples\n");
    udp.udp_send_string("# motor_id: pwm,avg_speed_L,avg_speed_R,avg_speed,samples\n");

    for (int i = 0; i < NUM_STEPS && !exit_flag.load(); i++) {
        std::printf("[Step %d/%d] PWM=%d ... ", i + 1, NUM_STEPS, PWM_STEPS[i]);
        std::fflush(stdout);

        StepResult r = measure_step(PWM_STEPS[i]);

        std::printf("L=%.3f  R=%.3f  avg=%.3f  (n=%d)\n",
                    r.avg_speed_L, r.avg_speed_R, r.avg_speed, r.samples);

        char buf[128];
        std::snprintf(buf, sizeof(buf), "%d,%.4f,%.4f,%.4f,%d\n",
                     r.pwm, r.avg_speed_L, r.avg_speed_R, r.avg_speed, r.samples);
        udp.udp_send_string(std::string(buf));

        // 停车稳定
        motor_Duty_L(0);
        motor_Duty_R(0);
        safe_sleep(SETTLE_MS);
    }

    // 3. 汇总
    std::printf("\n=== Motor ID Complete ===\n");
    udp.udp_send_string("# MOTOR_ID_DONE\n");
    std::printf("[INFO] Data sent via UDP to %s:%d\n", UDP_PC_HOST, UDP_PC_PORT);

    motor_Duty_L(0);
    motor_Duty_R(0);
    gpio_protect_heartbeat_deinit();
}
