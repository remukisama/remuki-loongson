#include "lq_all_demo.hpp"
#include "car_config.hpp"
#include "my_mpu6050.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "gpio_protect_heartbeat.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>
#include <thread>

// ============================================================
// 【Demo 42】角速度环调参 — 电机参与模式（增强版）
//
// 用途：电机驱动差速 → 实际产生角速度 → 陀螺仪闭环验证
//       在真实动力学下标定角速度环 P/I/D。
//
// 增强功能：
//   • 起步前馈补偿：速度环输出从0累积，起步时加一个前馈PWM克服静摩擦
//     消除左右轮起转时间不一致导致的角速度震荡
//   • 完整UDP通道（9通道 VOFA+ JustFloat 兼容）
//   • 命令行可调参数：P/D/speed/step_target/feedforward
//   • 支持低速/高速自动连续阶跃测试
//
// UDP 输出格式（VOFA+ JustFloat 兼容）：
//   "ware:target,gyro_raw,gyro_filt,err,diff,goal_L,goal_R,enc_L,enc_R\n"
//
// 用法：
//   ./main --mode=gyro-motor-tune                          (默认: P=0.035 D=0 speed=1.0)
//   ./main --mode=gyro-motor-tune --kp=0.05 --kd=0.002     (指定P/D)
//   ./main --mode=gyro-motor-tune --speed=0.20              (低速测试)
//   ./main --mode=gyro-motor-tune --speed=0.80              (高速测试)
//   ./main --mode=gyro-motor-tune --step=45                 (指定阶跃目标)
//   ./main --mode=gyro-motor-tune --ff=800                  (指定起步前馈PWM)
//   ./main --mode=gyro-motor-tune --sine                    (正弦模式)
//   ./main --mode=gyro-motor-tune --hold=3.0                (阶跃保持3秒)
// ============================================================

static std::atomic<bool> g_exit(false);

static void sig_handler(int) { g_exit.store(true); }

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
// 迷你角速度环（复刻 my_cascade_pid.cpp 的信号调理链）
// ============================================================
struct MiniGyroPID {
    float P, I, D;
    float err, last_err, integral;
    float int_max, out_max, output;
    float gyro_z_filtered;
    float last_gyro_z_measurement;
    float last_output;

    void reset() {
        err = last_err = integral = output = 0.0f;
        gyro_z_filtered = last_gyro_z_measurement = last_output = 0.0f;
    }

    float update(float target_gyro_z, float raw_gyro_z, float dt) {
        if (dt <= 0.0001f || dt > 0.1f) dt = 0.005f;

        float g = raw_gyro_z;
        // 1. 尖峰拒绝
        if (std::fabs(g) > car_config::kGyroRawSpikeRejectDps)
            g = last_gyro_z_measurement;
        // 2. 测量限幅
        g = clampf(g, -car_config::kGyroMeasurementLimitDps,
                       car_config::kGyroMeasurementLimitDps);
        // 3. 一阶低通滤波
        gyro_z_filtered = gyro_z_filtered * (1.0f - car_config::kGyroLpfAlpha) +
                          g * car_config::kGyroLpfAlpha;
        // 4. 死区
        if (std::fabs(gyro_z_filtered) < car_config::kGyroDeadBandDps)
            gyro_z_filtered = 0.0f;

        err = target_gyro_z - gyro_z_filtered;
        integral = clampf(integral + err * dt, -int_max, int_max);
        float derivative = (err - last_err) / dt;
        float out = P * err + I * integral + D * derivative;
        out = clampf(out, -out_max, out_max);

        last_gyro_z_measurement = g;
        last_err = err;
        last_output = out;
        output = out;
        return out;
    }
};

// ============================================================
// 位置式速度环 + 起步前馈补偿 + 输出变化率限制
// ============================================================
struct MiniSpeedPID {
    float P, I, D;
    float err, last_err;
    float integral;
    float output;
    float last_output;
    int max_pwm;
    int feedforward_pwm;  // 起步前馈：目标>0时直接叠加，克服静摩擦
    float integral_limit;
    float slew_rate;
    float brake_max_pwm;

    void reset() {
        err = last_err = integral = output = last_output = 0.0f;
    }

    float update(float target, float measured, float dt) {
        (void)dt;
        if (target <= 0.02f) {
            output = 0.0f;
            last_err = 0.0f;
            integral = 0.0f;
            last_output = 0.0f;
            return 0.0f;
        }
        err = target - measured;

        // 位置式 PID: output = P*err + Ki*integral + Kd*derivative
        integral += err;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;

        float derivative = err - last_err;
        float raw_out = P * err + I * integral + D * derivative;

        if (raw_out > max_pwm) raw_out = max_pwm;
        // 有限反拖制动：超速>阈值时允许有限负PWM（和car模式Speed_PID_L/R一致）
        if (raw_out < 0.0f) {
            float overshoot = measured - target;  // 正=超速
            if (overshoot > car_config::kSpeedBrakeThreshold) {
                if (raw_out < brake_max_pwm)
                    raw_out = brake_max_pwm;
            } else {
                raw_out = 0.0f;
            }
        }

        // 输出变化率限制
        float slew = raw_out - last_output;
        if (slew > slew_rate) raw_out = last_output + slew_rate;
        if (slew < -slew_rate) raw_out = last_output - slew_rate;

        last_err = err;
        last_output = raw_out;
        output = raw_out;

        // 起步前馈补偿
        float ff = 0.0f;
        if (feedforward_pwm > 0 && target > 0.02f) {
            float enc_abs = std::fabs(measured);
            if (enc_abs < 0.10f) {
                ff = (float)feedforward_pwm;
            } else if (enc_abs < 0.30f) {
                float decay = 1.0f - (enc_abs - 0.10f) / 0.20f;
                ff = (float)feedforward_pwm * decay;
            }
        }
        return output + ff;
    }
};

// ============================================================
// 参数解析辅助
// ============================================================
static float parse_arg(int argc, char **argv, const char *key, float default_val) {
    std::string prefix = std::string("--") + key + "=";
    for (int i = 2; i < argc; ++i) {
        if (std::strncmp(argv[i], prefix.c_str(), prefix.length()) == 0) {
            return std::atof(argv[i] + prefix.length());
        }
    }
    return default_val;
}

// ============================================================
// 入口函数
// ============================================================
int lq_gyro_motor_pid_tune_demo(int argc, char **argv)
{
    g_exit.store(false);
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGHUP,  sig_handler);
    signal(SIGQUIT, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    // ---- 解析命令行参数 ----
    bool sine_mode = false;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--sine") == 0) {
            sine_mode = true;
        }
    }

    float kp          = parse_arg(argc, argv, "kp",    car_config::kGyroPidP);       // 默认0.035
    float kd          = parse_arg(argc, argv, "kd",    car_config::kGyroPidD);       // 默认0.0
    float base_speed  = parse_arg(argc, argv, "speed", 1.0f);                        // 默认1.0 m/s
    float step_target = parse_arg(argc, argv, "step",  30.0f);                       // 阶跃目标 °/s
    float step_hold   = parse_arg(argc, argv, "hold",  2.0f);                        // 阶跃保持秒
    int   ff_pwm      = (int)parse_arg(argc, argv, "ff", 600.0f);                    // 起步前馈PWM

    float brake_pwm = parse_arg(argc, argv, "brake", car_config::kSpeedBrakeMaxPwm);
    if (brake_pwm > 0.0f) brake_pwm = -brake_pwm;
    if (brake_pwm < -1200.0f) brake_pwm = -1200.0f;

    std::printf("=== Gyro Motor PID Tune (Enhanced) ===\n");
    std::printf("Mode: %s\n", sine_mode ? "SINE" : "STEP");
    std::printf("Gyro PID: P=%.4f I=0 D=%.5f\n", kp, kd);
    std::printf("Base speed: %.2f m/s\n", base_speed);
    std::printf("Step target: %.1f dps, hold: %.1fs\n", step_target, step_hold);
    std::printf("Startup feedforward PWM: %d\n", ff_pwm);
    std::printf("Brake max PWM: %.0f\n", brake_pwm);
    std::printf("UDP target: 192.168.141.22:8080\n");
    std::printf("Format: ware:target,gyro_raw,gyro_filt,err,diff,goal_L,goal_R,enc_L,enc_R\n\n");

    // ---- 初始化 UDP ----
    lq_udp_client udp;
    udp.udp_client_init("192.168.141.22", 8080);

    // ---- 初始化硬件 ----
    motor_Init();
    if (!gpio_protect_heartbeat_require_ready(1500)) {
        std::printf("[FATAL] gpio_manager not ready\n");
        return 3;
    }

    MyMpu6050 imu;
    if (!imu.init()) {
        std::printf("[FATAL] MPU6050 init failed\n");
        emergency_stop();
        return 1;
    }

    // ---- PID 参数 ----
    MiniGyroPID gyro_pid;
    gyro_pid.P = kp;
    gyro_pid.I = 0.0f;
    gyro_pid.D = kd;
    gyro_pid.int_max = car_config::kGyroPidIntegralLimit;
    gyro_pid.out_max = car_config::kGyroPidOutputLimit;
    gyro_pid.reset();

    MiniSpeedPID speed_l, speed_r;
    speed_l.P = speed_r.P = car_config::kSpeedPidP;
    speed_l.I = speed_r.I = car_config::kSpeedPidI;
    speed_l.D = speed_r.D = car_config::kSpeedPidD;
    speed_l.max_pwm = speed_r.max_pwm = car_config::kMotorMaxPwm;
    speed_l.feedforward_pwm = speed_r.feedforward_pwm = ff_pwm;
    speed_l.integral_limit = speed_r.integral_limit = car_config::kSpeedPidIntegralLimit;
    speed_l.slew_rate = speed_r.slew_rate = car_config::kSpeedPidSlewRate;
    speed_l.brake_max_pwm = speed_r.brake_max_pwm = brake_pwm;
    speed_l.reset();
    speed_r.reset();

    std::printf("Gyro PID: P=%.4f D=%.5f out_max=%.2f\n",
                gyro_pid.P, gyro_pid.D, gyro_pid.out_max);
    std::printf("Speed PID: P=%.0f I=%.0f max_pwm=%d ff=%d\n",
                speed_l.P, speed_l.I, speed_l.max_pwm, ff_pwm);
    std::printf("Signal: spike=%d lim=%d lpf=%.2f deadband=%.1f\n",
                (int)car_config::kGyroRawSpikeRejectDps,
                (int)car_config::kGyroMeasurementLimitDps,
                car_config::kGyroLpfAlpha,
                car_config::kGyroDeadBandDps);

    // ---- 测试参数 ----
    constexpr float kSineAmp     = 60.0f;    // 正弦幅度 °/s（大角度测试）
    constexpr float kSinePeriod  = 3.0f;     // 正弦周期 s
    constexpr float kControlPeriodSec = 0.005f;

    auto last_time = std::chrono::steady_clock::now();
    auto next_flip_time = last_time + std::chrono::milliseconds(
        static_cast<int>(step_hold * 1000));
    bool positive = true;
    int print_count = 0;
    float sine_phase = 0.0f;
    int udp_count = 0;
    auto start_time = last_time;

    // ---- 先等2秒让IMU稳定 ----
    std::printf("[INIT] Waiting 2s for IMU stabilization...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::printf("[START] Beginning test. Press Ctrl+C to stop.\n");

    while (!g_exit.load(std::memory_order_relaxed)) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = now - last_time;
        float dt = elapsed.count();
        last_time = now;
        if (dt <= 0.0001f || dt > 0.1f) dt = kControlPeriodSec;

        // ---- 生成目标角速度 ----
        float target_gyro_z;
        if (sine_mode) {
            sine_phase += 2.0f * M_PI * dt / kSinePeriod;
            target_gyro_z = kSineAmp * std::sin(sine_phase);
        } else {
            if (now >= next_flip_time) {
                positive = !positive;
                next_flip_time = now + std::chrono::milliseconds(
                    static_cast<int>(step_hold * 1000));
            }
            target_gyro_z = positive ? step_target : -step_target;
        }

        // ---- 读传感器 ----
        Mpu6050Data imu_data;
        if (!imu.read_processed_data(imu_data)) {
            std::printf("[WARN] IMU read failed\n");
            motor_Duty_L(0);
            motor_Duty_R(0);
            gpio_protect_heartbeat_pulse();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        float enc_l = encoder_num_L();
        float enc_r = encoder_num_R();
        if (!encoder_health_ok()) {
            std::printf("[WARN] encoder invalid\n");
            motor_Duty_L(0);
            motor_Duty_R(0);
            gpio_protect_heartbeat_pulse();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // ---- 角速度环 ----
        float diff_speed = gyro_pid.update(target_gyro_z, imu_data.gyro_z, dt);

        // ---- 动态差速限幅 ----
        // 确保慢轮 >= 基速35%，快轮 <= 基速165%
        float dyn_limit = base_speed * 0.65f;
        if (dyn_limit > gyro_pid.out_max) dyn_limit = gyro_pid.out_max;
        if (diff_speed > dyn_limit) diff_speed = dyn_limit;
        if (diff_speed < -dyn_limit) diff_speed = -dyn_limit;

        // ---- 差速分配 ----
        float goal_l = base_speed - diff_speed;
        float goal_r = base_speed + diff_speed;
        if (goal_l < 0.0f) goal_l = 0.0f;
        if (goal_r < 0.0f) goal_r = 0.0f;
        if (goal_l > car_config::kMaxWheelSpeedGoal) goal_l = car_config::kMaxWheelSpeedGoal;
        if (goal_r > car_config::kMaxWheelSpeedGoal) goal_r = car_config::kMaxWheelSpeedGoal;

        // ---- 速度环（含起步前馈补偿）----
        float pwm_l = speed_l.update(goal_l, enc_l, dt);
        float pwm_r = speed_r.update(goal_r, enc_r, dt);
        int pwm_l_int = std::min((int)pwm_l, car_config::kMotorPwmFullScale);
        int pwm_r_int = std::min((int)pwm_r, car_config::kMotorPwmFullScale);
        motor_Duty_L(pwm_l_int);
        motor_Duty_R(pwm_r_int);

        // ---- UDP 波形输出（9通道）----
        // 每2帧发一次，约250Hz，避免UDP淹没
        if ((udp_count & 1) == 0) {
            char ubuf[256];
            int ulen = std::snprintf(ubuf, sizeof(ubuf),
                "ware:%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                target_gyro_z,            // ch1: 目标角速度
                imu_data.gyro_z,          // ch2: 原始角速度
                gyro_pid.gyro_z_filtered, // ch3: 滤波后角速度
                gyro_pid.err,             // ch4: 角速度误差
                diff_speed,               // ch5: 差速输出
                goal_l,                   // ch6: 左轮目标速度
                goal_r,                   // ch7: 右轮目标速度
                enc_l,                    // ch8: 左轮编码器速度
                enc_r);                   // ch9: 右轮编码器速度
            udp.udp_send(ubuf, ulen);
        }
        ++udp_count;

        // ---- stdout 数据输出（每帧输出，供 SSH 远程采集）----
        // 格式: [T42] t=<time> tgt=<target> raw=<gyro_raw> filt=<gyro_filt> err=<err> diff=<diff> gL=<goal_L> gR=<goal_R> eL=<enc_L> eR=<enc_R> pL=<pwm_L> pR=<pwm_R>
        auto total_sec = std::chrono::duration<float>(now - start_time).count();
        std::printf("[T42] t=%.3f tgt=%.2f raw=%.2f filt=%.3f err=%.3f diff=%.3f gL=%.3f gR=%.3f eL=%.3f eR=%.3f pL=%d pR=%d\n",
                    total_sec,
                    target_gyro_z, imu_data.gyro_z,
                    gyro_pid.gyro_z_filtered, gyro_pid.err,
                    diff_speed, goal_l, goal_r, enc_l, enc_r,
                    pwm_l_int, pwm_r_int);
        std::fflush(stdout);

        // ---- 串口低频打印（10Hz）----
        if (++print_count >= 50) {
            print_count = 0;
            auto total_sec = std::chrono::duration<float>(now - start_time).count();
            std::printf("t:%5.1f tgt:%6.1f raw:%7.1f filt:%7.1f err:%7.1f diff:%.3f "
                        "gL:%.2f gR:%.2f eL:%.2f eR:%.2f pwmL:%d pwmR:%d\n",
                        total_sec,
                        target_gyro_z, imu_data.gyro_z,
                        gyro_pid.gyro_z_filtered, gyro_pid.err,
                        diff_speed, goal_l, goal_r, enc_l, enc_r,
                        pwm_l_int, pwm_r_int);
        }

        gpio_protect_heartbeat_pulse();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // ---- 停车 ----
    motor_Duty_L(0);
    motor_Duty_R(0);
    emergency_stop();
    std::printf("Gyro motor tune demo exited.\n");
    return 0;
}
