#include "lq_all_demo.hpp"
#include "lq_udp_client.hpp"
#include "car_config.hpp"
#include "gpio_protect_heartbeat.hpp"
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <cstdio>
#include <csignal>
#include <cmath>
#include <sys/select.h>
#include <string>
#include <sstream>
#include <algorithm>
#include <cstdint>

// ==========================================
// 銆愬熀�?VOFA+ 鐨勯€熷害鐜?PID) 娉㈠舰璋冨弬婕旂ず绋嬪簭�?//   鎵€鏈夐€熷害鍗曚綅缁熶竴涓?m/s锛堣疆鑳庣洿�?64mm�?// ==========================================

// --- 杞�?& 榻胯疆鍙傛暟 ---
// 杞儙鐩村緞 64mm, 杞﹁疆澶ч娇杞?68 �? 缂栫爜鍣ㄥ皬榻胯�?30 �?// 缂栫爜鍣ㄨ繑鍥炵數鏈鸿酱 RPS, 鎹㈢畻涓鸿溅杞嚎閫熷害 (m/s):
//   wheel_RPS = motor_RPS �?(30/68)
//   m/s       = wheel_RPS �?�?�?0.064
//   ENC_TO_MS = motor_RPS �?(30/68) �?�?�?0.064 �?motor_RPS �?0.08870
#define WHEEL_DIAMETER_M       ( 0.064f )
#define GEAR_TEETH_WHEEL       ( 68 )
#define GEAR_TEETH_ENCODER     ( 30 )
#define GEAR_RATIO             ( (float)GEAR_TEETH_ENCODER / (float)GEAR_TEETH_WHEEL )  // 30/68
#define WHEEL_CIRCUMFERENCE_M  ( (float)M_PI * WHEEL_DIAMETER_M )                        // �?0.20106 m
#define ENC_TO_MS(rps)         ( (float)(rps) * GEAR_RATIO * WHEEL_CIRCUMFERENCE_M )     // �?rps �?0.08870

static constexpr float kTunedSpeedKp = 1800.0f;
static constexpr float kTunedSpeedKi = 35.0f;
static constexpr float kTargetAccelLimit = 1.8f;  // m/s^2, 0 -> 0.9m/s �?0.5s
static constexpr float kTargetDecelLimit = 2.0f;  // m/s^2, 鐣ュ己涓€鐐逛究浜庝笅闄嶆鏀堕€?
static inline float slew_limit(float current, float desired, float accel_limit, float decel_limit, float dt_sec)
{
    float delta = desired - current;
    float limit = (delta >= 0.0f ? accel_limit : decel_limit) * dt_sec;
    if (delta > limit) return current + limit;
    if (delta < -limit) return current - limit;
    return desired;
}

// Shared exit flag for all T28 speed demos.
static std::atomic<bool> exit_vofa_demo(false);

void signal_handler_vofa_tuning(int signum) {
    (void)signum;
    exit_vofa_demo = true;
}

// UDP target for legacy VOFA demos.
#define SERVER_IP       "192.168.46.22"
#define SERVER_PORT     8080

static float Kp_L = kTunedSpeedKp;
static float Ki_L = kTunedSpeedKi;
static float Kp_R = kTunedSpeedKp;
static float Ki_R = kTunedSpeedKi;
static float speed_out_L = 0.0f;
static float speed_out_R = 0.0f;
static int sine_tick = 0;
// ==========================================
// 1. 鍥哄畾鐩爣閫熷�?PID 璋冨�?(m/s)
// ==========================================
void lq_vofa_speed_pid_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);

    std::cout << "========== UDP speed PID tuning demo ==========" << std::endl;
    std::cout << ">>> UDP target IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> UDP target Port: " << SERVER_PORT << std::endl;
    std::cout << ">>> Wheel circumference: " << WHEEL_CIRCUMFERENCE_M << " m  (diameter " << WHEEL_DIAMETER_M << " m)" << std::endl;
    std::cout << ">>> Speed unit: m/s" << std::endl;
    std::cout << "Press Ctrl+C to stop motors and exit safely." << std::endl;

    // 1. Initialize UDP client.
    lq_udp_client client;
    client.udp_client_init(SERVER_IP, SERVER_PORT);
    std::cout << ">>> UDP waveform sender initialized." << std::endl;

    // 2. Initialize motor PWM control.
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_NORMAL);

    // 3. Initialize motor direction pins.
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
        motor_L.atim_pwm_enable();
        motor_R.atim_pwm_enable();
    };

        ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // --- PID 璋冨弬鍙橀噺鍖?(鍗曚�? m/s) ---
    // 銆愯鍦ㄨ繖閲屼慨鏀规偍�?PID 鍙傛暟銆?    float Kp_L = kTunedSpeedKp;
    float Ki_L = kTunedSpeedKi;
    float Kd_L = 0.0f;
    float target_speed_L = 1.0f;  // 宸﹁疆鐩爣绾块€熷�?(m/s)

    float Kp_R = kTunedSpeedKp;
    float Ki_R = kTunedSpeedKi;
    float Kd_R = 0.0f;
    float target_speed_R = 1.0f;  // 鍙宠疆鐩爣绾块€熷�?(m/s)

    int MAX_PWM = 7000;           // PWM 鏈€澶ч檺�?
    // PID 鐘舵€佸彉閲?    float speed_out_L = 0.0f;
    float last_error_L = 0.0f;
    float speed_out_R = 0.0f;
    float last_error_R = 0.0f;

    int loop_count = 0;

    while(!exit_vofa_demo)
    {
        // ==============================================
        // 1. 鑾峰彇褰撳墠閫熷�?(RPS �?m/s)
        // ==============================================
        float current_speed_L = ENC_TO_MS(-encoder_L.encoder_get_count());
        float current_speed_R = ENC_TO_MS( encoder_R.encoder_get_count());

        // ==============================================
        // 2. 璁＄畻璇樊
        // ==============================================
        float error_L = target_speed_L - current_speed_L;
        float error_R = target_speed_R - current_speed_R;

        // ==============================================
        // 3. 澧為噺寮?杩戜技) PID 璁＄�?        // ==============================================
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
        // 4. PWM 闄愬�?        // ==============================================
                            if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        if (target_speed_L == 0.0f && target_speed_R == 0.0f) {
            stop_motors();
        } else {
            // ==============================================
            // 5. 鐢垫満鏂瑰悜鎺у埗鍙?PWM 鍗犵┖姣旇緭�?            // ==============================================
                            if (speed_out_L >= 0.0f) {
                motor_L_Dir.gpio_level_set(GPIO_HIGH);
                motor_L.atim_pwm_set_duty((int)speed_out_L);
            } else {
                motor_L_Dir.gpio_level_set(GPIO_LOW);
                motor_L.atim_pwm_set_duty((int)(-speed_out_L));
            }

            if (speed_out_R >= 0.0f) {
                motor_R_Dir.gpio_level_set(GPIO_HIGH);
                motor_R.atim_pwm_set_duty((int)speed_out_R);
            } else {
                motor_R_Dir.gpio_level_set(GPIO_LOW);
                motor_R.atim_pwm_set_duty((int)(-speed_out_R));
            }
        }

        // ==============================================
        // 6. 閫氳�?VOFA+ 瀹㈡埛绔彂閫佹尝褰㈡暟�?        //    FireWater 鍗忚�? ware:鐩爣L,瀹為檯L,鐩爣R,瀹為檯R\n
        //    鍗曚�? m/s
        // ==============================================
        char send_buf[128];
        int len = snprintf(send_buf, sizeof(send_buf), "ware:%.3f,%.3f,%.3f,%.3f\n",
                           target_speed_L, current_speed_L,
                           target_speed_R, current_speed_R);
        if (len > 0) {
            client.udp_send(send_buf, len);
        }

        // 鎺у埗鍙版墦鍗?(闄嶄綆棰戠巼)
        loop_count++;
        if (loop_count >= 50) {
            std::cout << ">>> 姝ｅ湪鎸佺画鍙戦�?VOFA 娉㈠舰鏁版嵁..."
                      << " Target L: " << target_speed_L << " m/s"
                      << ", R: " << target_speed_R << " m/s" << std::endl;
            loop_count = 0;
        }

        // 鎺у埗鍛ㄦ�?20ms (闂幆棰戠巼 50Hz)
        usleep(20000);
    }

    std::cout << "姝ｅ湪鍏抽棴鐢垫満涓庤繛鎺?.." << std::endl;
    shutdown_motors();
    client.udp_close();
}

// ==========================================
// 2. 浜や簰寮忛樁璺冮€熷害娴嬭瘯 (m/s)
//    缁堢杈撳叆鐩爣閫熷害/鏃堕�?[Kp]/[Ki] 鎵ц鍗曟闃惰穬�?//    绛夊緟鏈熼棿鐩爣閫熷害�?0锛屼粎閫氳繃 UDP 鍙戦€佹尝褰€?//    鏍煎�? <鐩爣閫熷害(m/s)> <鎸佺画鏃堕棿(ms)> [Kp] [Ki]
//    绀轰緥: 1.5 3000            (1.5 m/s �?3 �?
//    绀轰緥: 2.0 5000 700 320    (2.0 m/s �?5 �? Kp=700 Ki=320)
// ==========================================
void lq_vofa_speed_step_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);

    std::cout << "========== 浜や簰寮忛樁璺冮€熷害娴嬭瘯绋嬪簭鍚姩 ==========" << std::endl;
    std::cout << ">>> UDP 鐩�?IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> UDP 鐩�?Port: " << SERVER_PORT << std::endl;
    std::cout << ">>> 杞儙鍛ㄩ暱: " << WHEEL_CIRCUMFERENCE_M << " m  (鐩村�?" << WHEEL_DIAMETER_M << " m)" << std::endl;
    std::cout << ">>> 閫熷害鍗曚綅: m/s" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << ">>> 鏍煎�? <鐩爣閫熷害(m/s)> <鎸佺画鏃堕棿(ms)> [Kp] [Ki]" << std::endl;
    std::cout << ">>> 绀轰緥: 1.5 3000          (淇濇寔褰撳墠 PI)" << std::endl;
    std::cout << ">>> 绀轰緥: 2.0 5000 700 320  (鍚屾椂鏇存柊 PI)" << std::endl;
    std::cout << ">>> Enter q or quit to exit." << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << "Press Ctrl+C to stop motors and exit safely." << std::endl;

    // 1. Initialize UDP client.
    lq_udp_client client;
    client.udp_client_init(SERVER_IP, SERVER_PORT);

    // 2. 鍒濆鍖栫數�?PWM 鎺у埗
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_NORMAL);

    // 3. Initialize motor direction pins.
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
        motor_L.atim_pwm_enable();
        motor_R.atim_pwm_enable();
    };

        ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // --- PID 璋冨弬鍙橀噺鍖?(鍗曚�? m/s) ---
    float Kp_L = kTunedSpeedKp;
    float Ki_L = kTunedSpeedKi;
    float Kd_L = 0.0f;

    float Kp_R = kTunedSpeedKp;
    float Ki_R = kTunedSpeedKi;
    float Kd_R = 0.0f;

    int MAX_PWM = 7000;

    // --- 浠诲姟鐘舵€?---
    float target_speed_L   = 0.0f;
    float target_speed_R   = 0.0f;
    int   task_remaining_ms = 0;       // 鍓╀綑杩愯鏃堕�?(ms)�? 琛ㄧず鏃犱换�?
    // --- PID 杩愯鏃剁姸�?---
    float speed_out_L  = 0.0f;
    float speed_out_R  = 0.0f;
    float last_error_L = 0.0f;
    float last_error_R = 0.0f;

    // --- 缁堢杈撳叆缂撳啿鍖?---
    std::string input_buf;
    bool prompt_shown = false;

    auto show_prompt = [&]() {
        if (!prompt_shown) {
            std::cout << ">>> [绛夊緟杈撳叆] 褰撳�?Kp=" << Kp_L << " Ki=" << Ki_L
                      << " | 杈撳叆鐩爣閫熷�?m/s) 鏃堕�?ms) [Kp] [Ki]: " << std::flush;
            prompt_shown = true;
        }
    };

    show_prompt();

    while(!exit_vofa_demo)
    {
        // ==============================================
        // 闈為樆濉炴鏌ョ粓绔緭�?        // ==============================================
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
                        // 鍘绘帀棣栧熬绌虹櫧
                        while (!input_buf.empty() && input_buf.back() == '\r') input_buf.pop_back();

                        if (input_buf == "q" || input_buf == "quit")
                        {
                            std::cout << ">>> User requested exit." << std::endl;
                            exit_vofa_demo = true;
                            input_buf.clear();
                            continue;
                        }

                        // 瑙ｆ�? speed duration [Kp] [Ki]
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

                            // 鍙�?PI 鍙傛�? 杈撳叆浜?Kp 鎵嶆洿鏂?
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

                            std::cout << ">>> [鎵ц] 鐩�?" << target_speed_L << " m/s"
                                      << ", 鏃堕�?" << task_remaining_ms << " ms"
                                      << ", Kp=" << Kp_L << ", Ki=" << Ki_L << std::endl;
                            prompt_shown = false;
                        }
                        else
                        {
                            std::cout << ">>> [閿欒] 鏃犳硶瑙ｆ瀽杈撳叆: " << input_buf << std::endl;
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
        // 浠诲姟璁℃椂: 鍒版湡鑷姩灏嗙洰鏍囬€熷害褰掗�?        // ==============================================
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
                std::cout << ">>> [DONE] Task finished; target speed reset to zero." << std::endl;
                show_prompt();
            }
        }

        // ==============================================
        // 1. 鑾峰彇褰撳墠閫熷�?(RPS �?m/s)
        // ==============================================
        float current_speed_L = ENC_TO_MS(-encoder_L.encoder_get_count());
        float current_speed_R = ENC_TO_MS( encoder_R.encoder_get_count());

        // ==============================================
        // 2. 璁＄畻璇樊
        // ==============================================
        float error_L = target_speed_L - current_speed_L;
        float error_R = target_speed_R - current_speed_R;

        // ==============================================
        // 3. 澧為噺寮?杩戜技) PID 璁＄畻锛堢洰�?0 鏃朵篃杩愯锛屼富鍔ㄥ埗鍔ㄧ淮鎸侀€熷害涓洪浂�?        // ==============================================
        speed_out_L += Ki_L * error_L + Kp_L * (error_L - last_error_L)
                     + Kd_L * (error_L - 2.0f * last_error_L + current_speed_L);
        last_error_L = error_L;

        speed_out_R += Ki_R * error_R + Kp_R * (error_R - last_error_R)
                     + Kd_R * (error_R - 2.0f * last_error_R + current_speed_R);
        last_error_R = error_R;

        // ==============================================
        // 4. PWM 闄愬�?        // ==============================================
                            if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        // ==============================================
        // 5. 鐢垫満鏂瑰悜鎺у埗鍙?PWM 鍗犵┖姣旇緭�?        // ==============================================
                            if (speed_out_L >= 0.0f) {
            motor_L_Dir.gpio_level_set(GPIO_HIGH);
            motor_L.atim_pwm_set_duty((int)speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_LOW);
            motor_L.atim_pwm_set_duty((int)(-speed_out_L));
        }

        if (speed_out_R >= 0.0f) {
            motor_R_Dir.gpio_level_set(GPIO_HIGH);
            motor_R.atim_pwm_set_duty((int)speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_LOW);
            motor_R.atim_pwm_set_duty((int)(-speed_out_R));
        }

        // ==============================================
        // 6. 閫氳�?VOFA+ 瀹㈡埛绔彂閫佹尝褰㈡暟�?(m/s)
        //    (濮嬬粓鍙戦€侊紝涓嶉樆濉?
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

    std::cout << "姝ｅ湪鍏抽棴鐢垫満涓庤繛鎺?.." << std::endl;
    shutdown_motors();
    client.udp_close();
}

// ==========================================
// 3. 浜や簰寮忓疄鍦伴€熷害娉㈠舰娴嬭�?(m/s)
//    缁堢杈撳叆鏈€浣庨€熷�?鏈€楂橀€熷�?鍛ㄦ�?鏃堕�?[Kp]/[Ki]�?//    鐩爣鎸夊崐浣欏鸡鏇茬嚎鍦╗min,max]涔嬮棿骞虫粦寰€杩旓紝绛夊緟鏈熼棿鐩爣閫熷害涓?0�?//    鍏�? target = min + (max - min) * 0.5 * (1 - cos(2�?* t / period))
//    鏍煎�? <鏈€浣庨€熷�?m/s)> <鏈€楂橀€熷�?m/s)> <鍛ㄦ�?s)> <杩愯鏃堕暱(ms)> [Kp] [Ki]
//    绀轰緥: 0.9 1.3 2.0 4000          (0.9~1.3 m/s, 鍛ㄦ�?2s, �?4s)
//    绀轰緥: 0.9 1.3 2.0 4000 1800 35  (鍚屾椂鏇存柊 PI)
// ==========================================
void lq_vofa_speed_sine_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);

    std::cout << "========== VOFA 瀹炲湴閫熷害娉㈠舰娴嬭瘯绋嬪簭鍚姩 ==========" << std::endl;
    std::cout << ">>> UDP 鐩�?IP  : " << SERVER_IP << std::endl;
    std::cout << ">>> UDP 鐩�?Port: " << SERVER_PORT << std::endl;
    std::cout << ">>> 杞儙鍛ㄩ暱: " << WHEEL_CIRCUMFERENCE_M << " m  (鐩村�?" << WHEEL_DIAMETER_M << " m)" << std::endl;
    std::cout << ">>> 閫熷害鍗曚綅: m/s" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << ">>> 榛樿�?PI: Kp=" << kTunedSpeedKp << " Ki=" << kTunedSpeedKi << std::endl;
    std::cout << ">>> 鍏�? target = min + (max - min) * 0.5 * (1 - cos(2�?* t / period))" << std::endl;
    std::cout << ">>> 鏍煎�? <鏈€浣庨€熷�? <鏈€楂橀€熷�? <鍛ㄦ�?s)> <杩愯鏃堕暱(ms)> [Kp] [Ki]" << std::endl;
    std::cout << ">>> 绀轰緥: 0.9 1.3 2.0 4000          (0.9~1.3 m/s, 2s 鍛ㄦ�? �?4s)" << std::endl;
    std::cout << ">>> 绀轰緥: 0.9 1.3 2.0 4000 1800 35  (鍚屾椂鏇存柊 PI)" << std::endl;
    std::cout << ">>> Enter q or quit to exit." << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << "Press Ctrl+C to stop motors and exit safely." << std::endl;

    // 1. Initialize UDP client.
    lq_udp_client client;
    client.udp_client_init(SERVER_IP, SERVER_PORT);

    // 2. 鍒濆鍖栫數�?PWM 鎺у埗
    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, 25000, 0, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, 25000, 0, ATIM_PWM_POL_NORMAL);

    // 3. Initialize motor direction pins.
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
        motor_L.atim_pwm_enable();
        motor_R.atim_pwm_enable();
    };

        ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    // --- PID 璋冨弬鍙橀噺鍖?(鍗曚�? m/s) ---
    float Kp_L = kTunedSpeedKp;
    float Ki_L = kTunedSpeedKi;
    float Kd_L = 0.0f;

    float Kp_R = kTunedSpeedKp;
    float Ki_R = kTunedSpeedKi;
    float Kd_R = 0.0f;

    int MAX_PWM = 8000;
    const float DT_SEC = 0.02f;   // 鎺у埗鍛ㄦ�?20ms

    // --- 浠诲姟鐘舵€?---
    float target_speed_L   = 0.0f;
    float target_speed_R   = 0.0f;
    float desired_speed_L  = 0.0f;
    float desired_speed_R  = 0.0f;
    float sine_base        = 0.9f;    // 鏈€浣庨€熷�?(m/s)
    float sine_amplitude   = 0.4f;    // 閫熷害鑼冨洿 = max - min (m/s)
    float sine_period_s    = 2.0f;    // 閫熷害鏇茬嚎鍛ㄦ�?(s)
    int   task_total_ms     = 0;      // Total active sine task duration in ms.
    int   task_remaining_ms = 0;      // Remaining active sine task duration in ms.
    uint64_t sine_tick = 0;           // 20ms sine profile tick.
    // --- PID 杩愯鏃剁姸�?---
    float speed_out_L  = 0.0f;
    float speed_out_R  = 0.0f;
    float last_error_L = 0.0f;
    float last_error_R = 0.0f;

    // --- 缁堢杈撳叆缂撳啿鍖?---
    std::string input_buf;
    bool prompt_shown = false;

    auto show_prompt = [&]() {
        if (!prompt_shown) {
            std::cout << ">>> [绛夊緟杈撳叆] 褰撳�?Kp=" << Kp_L << " Ki=" << Ki_L
                      << " | 杈撳�?鏈€�?鏈€�?鍛ㄦ�?s) 鏃堕�?ms) [Kp] [Ki]: " << std::flush;
            prompt_shown = true;
        }
    };

    show_prompt();

    while(!exit_vofa_demo)
    {
        // ==============================================
        // 闈為樆濉炴鏌ョ粓绔緭�?        // ==============================================
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
                            std::cout << ">>> User requested exit." << std::endl;
                            exit_vofa_demo = true;
                            input_buf.clear();
                            continue;
                        }

                        // 瑙ｆ�? min max period_s duration [Kp] [Ki]
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

                            // 鍙�?PI 鍙傛�?                            if (iss >> in_Kp)
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

                            std::cout << ">>> [鎵ц] 鏈€�?" << sine_base << " m/s"
                                      << ", 鏈€�?" << in_max << " m/s"
                                      << ", 鍛ㄦ�?" << sine_period_s << " s"
                                      << ", 鏃堕�?" << task_remaining_ms << " ms"
                                      << ", Kp=" << Kp_L << ", Ki=" << Ki_L << std::endl;
                            std::printf("[T28_HEADER] period_ms=20 fields=t_ms,rem_ms,target,desired,encL,encR,errL,errR,pwmL,pwmR,kp,ki,rawL,rawR,ramp\n");
                            std::fflush(stdout);
                            prompt_shown = false;
                        }
                        else
                        {
                            std::cout << ">>> [閿欒] 鏃犳硶瑙ｆ瀽杈撳叆: " << input_buf << std::endl;
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
        // 浠诲姟璁℃椂: 鍒版湡鑷姩灏嗙洰鏍囬€熷害褰掗�?        // ==============================================
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
                std::cout << ">>> [DONE] Task finished; target speed reset to zero." << std::endl;
                show_prompt();
            }
        }

        // ==============================================
        // 璁＄畻瀹炲湴閫熷害鐩�?(m/s): 鍦ㄦ渶浣?鏈€楂橀€熷害涔嬮棿骞虫粦寰€杩斻€?        // ==============================================
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
        // 1. 鑾峰彇褰撳墠閫熷�?(RPS �?m/s)
        // ==============================================
        float raw_speed_L = ENC_TO_MS(-encoder_L.encoder_get_count());
        float raw_speed_R = ENC_TO_MS( encoder_R.encoder_get_count());
        float current_speed_L = raw_speed_L;
        float current_speed_R = raw_speed_R;

        // ==============================================
        // 2. 璁＄畻璇樊
        // ==============================================
        float error_L = target_speed_L - current_speed_L;
        float error_R = target_speed_R - current_speed_R;

        // ==============================================
        // 3. 澧為噺寮?杩戜技) PID 璁＄畻锛堢洰�?0 鏃朵篃杩愯锛屼富鍔ㄥ埗鍔ㄧ淮鎸侀€熷害涓洪浂�?        // ==============================================
        speed_out_L += Ki_L * error_L + Kp_L * (error_L - last_error_L)
                     + Kd_L * (error_L - 2.0f * last_error_L + current_speed_L);
        last_error_L = error_L;

        speed_out_R += Ki_R * error_R + Kp_R * (error_R - last_error_R)
                     + Kd_R * (error_R - 2.0f * last_error_R + current_speed_R);
        last_error_R = error_R;

        // ==============================================
        // 4. PWM 闄愬�?        // ==============================================
                            if (speed_out_L > MAX_PWM) speed_out_L = MAX_PWM;
        if (speed_out_L < -MAX_PWM) speed_out_L = -MAX_PWM;
        if (speed_out_R > MAX_PWM) speed_out_R = MAX_PWM;
        if (speed_out_R < -MAX_PWM) speed_out_R = -MAX_PWM;

        // ==============================================
        // 5. 鐢垫満鏂瑰悜鎺у埗鍙?PWM 鍗犵┖姣旇緭�?        // ==============================================
                            if (speed_out_L >= 0.0f) {
            motor_L_Dir.gpio_level_set(GPIO_HIGH);
            motor_L.atim_pwm_set_duty((int)speed_out_L);
        } else {
            motor_L_Dir.gpio_level_set(GPIO_LOW);
            motor_L.atim_pwm_set_duty((int)(-speed_out_L));
        }

        if (speed_out_R >= 0.0f) {
            motor_R_Dir.gpio_level_set(GPIO_HIGH);
            motor_R.atim_pwm_set_duty((int)speed_out_R);
        } else {
            motor_R_Dir.gpio_level_set(GPIO_LOW);
            motor_R.atim_pwm_set_duty((int)(-speed_out_R));
        }

        // ==============================================
        // 6. 閫氳�?VOFA+ 瀹㈡埛绔彂閫佹尝褰㈡暟�?(m/s)
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

    std::cout << "姝ｅ湪鍏抽棴鐢垫満涓庤繛鎺?.." << std::endl;
    shutdown_motors();
    client.udp_close();
}


// ==========================================
// 4. 6m track speed-loop UDP sine tuning
//    Flow: arm stop -> forward sine sampling -> settle -> slow auto-return -> stop.
//    Safety limits run on the car, so UDP/log failure cannot keep the car moving.
// ==========================================
void lq_speed_loop_track_udp_tune_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);
    if (!gpio_protect_heartbeat_require_ready(1500)) {
        std::cout << ">>> gpio_manager watchdog not ready; speed-tune refused." << std::endl;
        return;
    }

    constexpr float DT_SEC = (float)car_config::kSpeedTuneControlPeriodMs / 1000.0f;
    const float sine_min = std::max(0.0f, car_config::kSpeedTuneSineMinSpeed);
    const float sine_max = std::min(car_config::kSpeedTuneSineMaxSpeed, car_config::kSpeedTuneHardMaxSpeed);
    const int total_budget_ms = car_config::kSpeedTuneArmMaxMs + car_config::kSpeedTuneActiveMs + 300 +
                                car_config::kSpeedTuneReturnTimeoutMs + car_config::kSpeedTuneCooldownMs;

    std::cout << "========== speed-tune UDP sine mode ==========" << std::endl;
    std::cout << ">>> UDP: " << car_config::kSpeedTuneUdpHost << ":" << car_config::kSpeedTuneUdpPort << std::endl;
    std::cout << ">>> sine range: " << sine_min << "~" << sine_max << " m/s, hard max: "
              << car_config::kSpeedTuneHardMaxSpeed << " m/s" << std::endl;
    std::cout << ">>> forward distance limit: " << car_config::kSpeedTuneDistanceLimitM
              << " m, return stop margin: " << car_config::kSpeedTuneReturnStopDistanceM << " m" << std::endl;

    lq_udp_client client;
    client.udp_client_init(car_config::kSpeedTuneUdpHost, car_config::kSpeedTuneUdpPort);

    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, car_config::kMotorPwmFrequencyHz,
                        car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, car_config::kMotorPwmFrequencyHz,
                        car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);
    ls_encoder_pwm encoder_L(ENC_PWM3_PIN67, PIN_75);
    ls_encoder_pwm encoder_R(ENC_PWM0_PIN64, PIN_72);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();
    encoder_L.encoder_reset_counter();
    encoder_R.encoder_reset_counter();
    usleep(20000);
    encoder_L.encoder_close_reset_counter();
    encoder_R.encoder_close_reset_counter();
    usleep(100000);
    encoder_L.encoder_clear_capture_buffer();
    encoder_R.encoder_clear_capture_buffer();

    auto stop_motors = [&]() {
        motor_L.atim_pwm_set_duty(0);
        motor_R.atim_pwm_set_duty(0);
        motor_L_Dir.gpio_level_set(GPIO_HIGH);
        motor_R_Dir.gpio_level_set(GPIO_HIGH);
    };
    auto apply_motor = [&](float out_l, float out_r, int forward_limit, int reverse_limit) {
        out_l = std::max(-(float)reverse_limit, std::min((float)forward_limit, out_l));
        out_r = std::max(-(float)reverse_limit, std::min((float)forward_limit, out_r));
        if (out_l >= 0.0f) { motor_L_Dir.gpio_level_set(GPIO_HIGH); motor_L.atim_pwm_set_duty((int)out_l); }
        else { motor_L_Dir.gpio_level_set(GPIO_LOW); motor_L.atim_pwm_set_duty((int)(-out_l)); }
        if (out_r >= 0.0f) { motor_R_Dir.gpio_level_set(GPIO_HIGH); motor_R.atim_pwm_set_duty((int)out_r); }
        else { motor_R_Dir.gpio_level_set(GPIO_LOW); motor_R.atim_pwm_set_duty((int)(-out_r)); }
    };
    auto send_log = [&](const char *phase, int t_ms, float target, float desired,
                        float enc_l, float enc_r, float err_l, float err_r,
                        float pwm_l, float pwm_r, float distance_m,
                        float forward_distance, float return_distance,
                        float raw_l, float raw_r, bool encoder_valid,
                        int encoder_bad_ms, const char *reason) {
        char buf[640];
        int len = std::snprintf(buf, sizeof(buf),
            "speed_tune,v=1,profile=sine,t_ms=%d,phase=%s,target=%.3f,desired=%.3f,encL=%.3f,encR=%.3f,rawL=%.3f,rawR=%.3f,valid=%d,badMs=%d,errL=%.3f,errR=%.3f,pwmL=%.1f,pwmR=%.1f,dist=%.3f,fwd=%.3f,ret=%.3f,kp=%.1f,ki=%.1f,maxV=%.2f,reason=%s\n",
            t_ms, phase, target, desired, enc_l, enc_r, raw_l, raw_r,
            encoder_valid ? 1 : 0, encoder_bad_ms, err_l, err_r, pwm_l, pwm_r,
            distance_m, forward_distance, return_distance, car_config::kSpeedPidP, car_config::kSpeedPidI,
            car_config::kSpeedTuneHardMaxSpeed, reason);
        if (len > 0) client.udp_send(buf, (size_t)len);
        std::printf("%s", buf);
        std::fflush(stdout);
    };

    stop_motors();
    client.udp_send_string("speed_tune,v=1,event=header,profile=sine,fields=t_ms phase target desired encL encR rawL rawR valid badMs errL errR pwmL pwmR dist fwd ret kp ki maxV reason\n");

    enum Phase { ARM, ENCODER_PROBE, FORWARD, SETTLE, COOLDOWN, DONE };
    Phase phase = ARM;
    const char *stop_reason = "none";
    int phase_ms = 0, t_ms = 0, mismatch_ms = 0, saturation_ms = 0, stall_ms = 0;
    int encoder_invalid_ms = 0, arm_stable_ms = 0, low_motion_ms = 0, settle_stable_ms = 0;
    bool encoder_probe_done = false;
    float target_l = 0.0f, target_r = 0.0f, speed_out_l = 0.0f, speed_out_r = 0.0f;
    float last_error_l = 0.0f, last_error_r = 0.0f;
    float forward_distance_m = 0.0f, return_distance_m = 0.0f;

    auto read_encoder_speeds = [&](float &enc_l, float &enc_r, float &raw_l, float &raw_r) {
        uint32_t period_l = encoder_L.encoder_get_full_period();
        uint32_t period_r = encoder_R.encoder_get_full_period();
        raw_l = ENC_TO_MS(-encoder_L.encoder_get_count());
        raw_r = ENC_TO_MS( encoder_R.encoder_get_count());
        encoder_L.encoder_clear_capture_buffer();
        encoder_R.encoder_clear_capture_buffer();
        bool moving_valid = period_l > 0 && period_r > 0 &&
                            period_l < car_config::kEncoderStalePeriodThreshold &&
                            period_r < car_config::kEncoderStalePeriodThreshold &&
                            std::fabs(raw_l) <= 3.0f && std::fabs(raw_r) <= 3.0f;
        bool stationary_valid = std::fabs(raw_l) < car_config::kSpeedTuneMotionStartThreshold &&
                                std::fabs(raw_r) < car_config::kSpeedTuneMotionStartThreshold;
        bool valid = moving_valid || stationary_valid;
        enc_l = valid ? raw_l : 0.0f;
        enc_r = valid ? raw_r : 0.0f;
        return valid;
    };

    while (!exit_vofa_demo && phase != DONE && t_ms <= total_budget_ms) {
        float enc_l = 0.0f;
        float enc_r = 0.0f;
        float raw_l = 0.0f;
        float raw_r = 0.0f;
        bool encoder_valid = read_encoder_speeds(enc_l, enc_r, raw_l, raw_r);
        float avg_speed = 0.5f * (enc_l + enc_r);
        if (encoder_valid) {
            if ((phase == FORWARD || phase == SETTLE) && avg_speed > 0.0f) {
                forward_distance_m += avg_speed * DT_SEC;
            }
            encoder_invalid_ms = 0;
        } else {
            encoder_invalid_ms += car_config::kSpeedTuneControlPeriodMs;
        }
        float distance_m = forward_distance_m - return_distance_m;

        const char *phase_name = "arm";
        float desired = 0.0f;
        int forward_pwm_limit = car_config::kSpeedTuneMaxPwm;
        int reverse_pwm_limit = 0;

        if (phase == ARM) {
            phase_name = "arm";
            bool still = encoder_valid && std::fabs(enc_l) < car_config::kSpeedTuneMotionStartThreshold &&
                         std::fabs(enc_r) < car_config::kSpeedTuneMotionStartThreshold;
            arm_stable_ms = still ? arm_stable_ms + car_config::kSpeedTuneControlPeriodMs : 0;
            if (arm_stable_ms >= car_config::kSpeedTuneArmStableMs) {
                phase = FORWARD;
                phase_ms = 0;
                arm_stable_ms = 0;
            } else if (phase_ms >= car_config::kSpeedTuneArmMaxMs) {
                phase = COOLDOWN;
                phase_ms = 0;
                stop_reason = "arm_not_still";
            }
        } else if (phase == ENCODER_PROBE) {
            phase_name = "encoder_probe";
            desired = 0.0f;
            if (phase_ms >= car_config::kSpeedTuneEncoderProbeMs) {
                encoder_probe_done = true;
                phase = COOLDOWN;
                phase_ms = 0;
                stop_reason = encoder_valid ? "encoder_probe_valid" : "encoder_probe_invalid";
            }
        } else if (phase == FORWARD) {
            phase_name = "forward";
            float t = (float)phase_ms / 1000.0f;
            float period = std::max(0.5f, car_config::kSpeedTuneSinePeriodSec);
            float profile = 0.5f * (1.0f - std::cos(2.0f * (float)M_PI * t / period));
            desired = sine_min + (sine_max - sine_min) * profile;
            if (phase_ms >= car_config::kSpeedTuneActiveMs) { phase = SETTLE; phase_ms = 0; stop_reason = "forward_time_done"; }
            else if (forward_distance_m >= car_config::kSpeedTuneDistanceLimitM) { phase = SETTLE; phase_ms = 0; stop_reason = "forward_distance_limit"; }
        } else if (phase == SETTLE) {
            phase_name = "settle";
            bool settled = encoder_valid && std::fabs(avg_speed) < car_config::kSpeedTuneMotionStartThreshold;
            settle_stable_ms = settled ? settle_stable_ms + car_config::kSpeedTuneControlPeriodMs : 0;
            if (forward_distance_m < car_config::kSpeedTuneMinForwardDistanceForReturnM) {
                phase = COOLDOWN;
                phase_ms = 0;
                stop_reason = "forward_too_short";
            } else if (settle_stable_ms >= car_config::kSpeedTuneSettleStableMs) {
                phase = COOLDOWN;
                phase_ms = 0;
                settle_stable_ms = 0;
                encoder_invalid_ms = 0;
                stop_reason = "auto_return_disabled";
            } else if (phase_ms >= car_config::kSpeedTuneSettleMaxMs) {
                phase = COOLDOWN;
                phase_ms = 0;
                stop_reason = "settle_timeout";
            }

        } else if (phase == COOLDOWN) {
            phase_name = "cooldown";
            if (phase_ms >= car_config::kSpeedTuneCooldownMs) { phase = DONE; break; }
        }

        desired = std::max(-car_config::kSpeedTuneHardMaxSpeed, std::min(car_config::kSpeedTuneHardMaxSpeed, desired));
        target_l = slew_limit(target_l, desired, car_config::kSpeedTargetAccelLimit, car_config::kSpeedTargetDecelLimit, DT_SEC);
        target_r = slew_limit(target_r, desired, car_config::kSpeedTargetAccelLimit, car_config::kSpeedTargetDecelLimit, DT_SEC);

        float error_l = target_l - enc_l;
        float error_r = target_r - enc_r;
        speed_out_l += car_config::kSpeedPidI * error_l + car_config::kSpeedPidP * (error_l - last_error_l);
        speed_out_r += car_config::kSpeedPidI * error_r + car_config::kSpeedPidP * (error_r - last_error_r);
        last_error_l = error_l;
        last_error_r = error_r;
        speed_out_l = std::max(-(float)reverse_pwm_limit, std::min((float)forward_pwm_limit, speed_out_l));
        speed_out_r = std::max(-(float)reverse_pwm_limit, std::min((float)forward_pwm_limit, speed_out_r));
        if (phase == FORWARD && phase_ms < car_config::kSpeedTuneLaunchMs) {
            speed_out_l = std::max(speed_out_l, (float)car_config::kSpeedTuneLaunchPwm);
            speed_out_r = std::max(speed_out_r, (float)car_config::kSpeedTuneLaunchPwm);

        }

        bool moving_phase = (phase == FORWARD);
        if (encoder_invalid_ms >= car_config::kSpeedTuneEncoderInvalidHoldMs && phase != COOLDOWN && phase != ENCODER_PROBE) {
            stop_reason = "encoder_unstable";
            phase = COOLDOWN;
            phase_ms = 0;
            target_l = target_r = speed_out_l = speed_out_r = last_error_l = last_error_r = 0.0f;
        }
        bool overspeed = moving_phase && encoder_valid && (std::fabs(enc_l) > car_config::kSpeedTuneOverspeedStop || std::fabs(enc_r) > car_config::kSpeedTuneOverspeedStop);
        bool mismatch = moving_phase && std::fabs(enc_l - enc_r) > car_config::kSpeedTuneWheelMismatchStop && std::fabs(avg_speed) > 0.20f;
        int active_pwm_limit = desired < 0.0f ? reverse_pwm_limit : forward_pwm_limit;
        bool saturated = moving_phase && (std::fabs(speed_out_l) > 0.95f * active_pwm_limit || std::fabs(speed_out_r) > 0.95f * active_pwm_limit) &&
                         (std::fabs(error_l) > 0.25f || std::fabs(error_r) > 0.25f);
        bool low_motion = moving_phase &&
                          (phase != FORWARD || phase_ms >= car_config::kSpeedTuneLaunchMs) &&
                          std::fabs(avg_speed) < car_config::kSpeedTuneMotionStartThreshold;
        bool stall = moving_phase && (std::fabs(speed_out_l) >= 0.80f * active_pwm_limit || std::fabs(speed_out_r) >= 0.80f * active_pwm_limit) &&
                     std::fabs(avg_speed) < car_config::kSpeedTuneStallSpeedThreshold;
        mismatch_ms = mismatch ? mismatch_ms + car_config::kSpeedTuneControlPeriodMs : 0;
        saturation_ms = saturated ? saturation_ms + car_config::kSpeedTuneControlPeriodMs : 0;
        low_motion_ms = low_motion ? low_motion_ms + car_config::kSpeedTuneControlPeriodMs : 0;
        stall_ms = stall ? stall_ms + car_config::kSpeedTuneControlPeriodMs : 0;
        if (overspeed || mismatch_ms >= car_config::kSpeedTuneMismatchHoldMs || saturation_ms >= car_config::kSpeedTuneSaturationHoldMs || stall_ms >= car_config::kSpeedTuneStallHoldMs || low_motion_ms >= car_config::kSpeedTuneLowMotionHoldMs) {
            if (overspeed) stop_reason = "overspeed";
            else if (mismatch_ms >= car_config::kSpeedTuneMismatchHoldMs) stop_reason = "wheel_mismatch";
            else if (saturation_ms >= car_config::kSpeedTuneSaturationHoldMs) stop_reason = "pwm_saturation";
            else if (low_motion_ms >= car_config::kSpeedTuneLowMotionHoldMs) stop_reason = "low_motion";
            else stop_reason = "stall_risk";
            if (phase != COOLDOWN) {
                phase = COOLDOWN;
                phase_ms = 0;
            }
            target_l = target_r = speed_out_l = speed_out_r = last_error_l = last_error_r = 0.0f;
        }

        if (phase == ARM || phase == SETTLE || phase == COOLDOWN || phase == DONE) {
            target_l = target_r = speed_out_l = speed_out_r = last_error_l = last_error_r = 0.0f;
            stop_motors();
        } else if (phase == ENCODER_PROBE) {
            target_l = target_r = speed_out_l = speed_out_r = last_error_l = last_error_r = 0.0f;
            speed_out_l = speed_out_r = (float)car_config::kSpeedTuneEncoderProbePwm;
            apply_motor(speed_out_l, speed_out_r,
                        car_config::kSpeedTuneEncoderProbePwm,
                        car_config::kSpeedTuneReverseBrakePwm);
        } else {
            apply_motor(speed_out_l, speed_out_r, forward_pwm_limit, reverse_pwm_limit);
        }
        gpio_protect_heartbeat_pulse();
        send_log(phase_name, t_ms, target_l, desired, enc_l, enc_r, error_l, error_r,
                 speed_out_l, speed_out_r, distance_m, forward_distance_m, return_distance_m,
                 raw_l, raw_r, encoder_valid, encoder_invalid_ms, stop_reason);
        usleep(car_config::kSpeedTuneControlPeriodMs * 1000);
        phase_ms += car_config::kSpeedTuneControlPeriodMs;
        t_ms += car_config::kSpeedTuneControlPeriodMs;
    }

    if (exit_vofa_demo) stop_reason = "signal_exit";
    stop_motors();
    send_log("done", t_ms, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, forward_distance_m - return_distance_m, forward_distance_m, return_distance_m,
             0.0f, 0.0f, true, 0, stop_reason);
    client.udp_close();
    std::cout << ">>> speed-tune finished, motors stopped. reason=" << stop_reason << std::endl;
}


// ==========================================
// 4. Low-speed recovery return mode
//    Use after an interrupted speed-tune run when the car is ahead of the start.
// ==========================================
void lq_speed_loop_recovery_return_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);

    std::cout << "========== speed-return disabled ==========" << std::endl;
    lq_udp_client client;
    client.udp_client_init(car_config::kSpeedTuneUdpHost, car_config::kSpeedTuneUdpPort);

    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, car_config::kMotorPwmFrequencyHz,
                        car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, car_config::kMotorPwmFrequencyHz,
                        car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();
    motor_L.atim_pwm_set_duty(0);
    motor_R.atim_pwm_set_duty(0);
    motor_L_Dir.gpio_level_set(GPIO_HIGH);
    motor_R_Dir.gpio_level_set(GPIO_HIGH);

    client.udp_send_string("speed_return,v=1,event=disabled,reason=auto_reverse_disabled_after_track_incident\n");
    client.udp_close();
    std::cout << ">>> speed-return disabled; motors stopped." << std::endl;
}
// ==========================================
// 5. 512-line direction encoder UDP diagnosis
//    This validates pulse pin, direction pin, and RPS->m/s conversion before PID tuning.
// ==========================================
void lq_encoder512_udp_diagnose_demo(void)
{
    exit_vofa_demo = false;
    signal(SIGINT, signal_handler_vofa_tuning);
    signal(SIGTERM, signal_handler_vofa_tuning);

    if (!gpio_protect_heartbeat_require_ready(1500)) {
        std::cout << ">>> gpio_manager watchdog not ready; encoder512 refused." << std::endl;
        return;
    }

    std::cout << "========== encoder512 UDP diagnosis ==========" << std::endl;
    std::cout << ">>> UDP: " << car_config::kSpeedTuneUdpHost << ":" << car_config::kSpeedTuneUdpPort << std::endl;
    std::cout << ">>> encoder lines: " << car_config::kEncoderLineCount
              << ", clock: " << LS_ENC_PWM_CLK << " Hz" << std::endl;

    lq_udp_client client;
    client.udp_client_init(car_config::kSpeedTuneUdpHost, car_config::kSpeedTuneUdpPort);

    ls_atim_pwm motor_L(ATIM_PWM0_PIN81, car_config::kMotorPwmFrequencyHz,
                        car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    ls_atim_pwm motor_R(ATIM_PWM1_PIN82, car_config::kMotorPwmFrequencyHz,
                        car_config::kMotorInitialDuty, ATIM_PWM_POL_NORMAL);
    ls_gpio motor_L_Dir(PIN_21, GPIO_MODE_OUT);
    ls_gpio motor_R_Dir(PIN_22, GPIO_MODE_OUT);
    ls_encoder_pwm encoder_0(ENC_PWM0_PIN64, PIN_72);
    ls_encoder_pwm encoder_1(ENC_PWM1_PIN65, PIN_73);
    ls_encoder_pwm encoder_2(ENC_PWM2_PIN66, PIN_74);
    ls_encoder_pwm encoder_3(ENC_PWM3_PIN67, PIN_75);
    ls_gpio encoder_0_Dir(PIN_72, GPIO_MODE_IN);
    ls_gpio encoder_1_Dir(PIN_73, GPIO_MODE_IN);
    ls_gpio encoder_2_Dir(PIN_74, GPIO_MODE_IN);
    ls_gpio encoder_3_Dir(PIN_75, GPIO_MODE_IN);

    motor_L.atim_pwm_enable();
    motor_R.atim_pwm_enable();
    encoder_0.encoder_reset_counter();
    encoder_1.encoder_reset_counter();
    encoder_2.encoder_reset_counter();
    encoder_3.encoder_reset_counter();
    usleep(20000);
    encoder_0.encoder_close_reset_counter();
    encoder_1.encoder_close_reset_counter();
    encoder_2.encoder_close_reset_counter();
    encoder_3.encoder_close_reset_counter();
    usleep(100000);
    encoder_0.encoder_clear_capture_buffer();
    encoder_1.encoder_clear_capture_buffer();
    encoder_2.encoder_clear_capture_buffer();
    encoder_3.encoder_clear_capture_buffer();

    auto stop_motors = [&]() {
        motor_L.atim_pwm_set_duty(0);
        motor_R.atim_pwm_set_duty(0);
        motor_L_Dir.gpio_level_set(GPIO_HIGH);
        motor_R_Dir.gpio_level_set(GPIO_HIGH);
    };
    auto set_motor_pwm = [&](int duty_l, int duty_r) {
        motor_L_Dir.gpio_level_set(GPIO_HIGH);
        motor_R_Dir.gpio_level_set(GPIO_HIGH);
        motor_L.atim_pwm_set_duty(duty_l);
        motor_R.atim_pwm_set_duty(duty_r);
    };
    auto clear_encoder_captures = [&]() {
        encoder_0.encoder_clear_capture_buffer();
        encoder_1.encoder_clear_capture_buffer();
        encoder_2.encoder_clear_capture_buffer();
        encoder_3.encoder_clear_capture_buffer();
    };
    auto send_diag = [&](const char *phase, int t_ms, int pwm_l, int pwm_r) {
        uint32_t ch0_period = encoder_0.encoder_get_full_period();
        uint32_t ch1_period = encoder_1.encoder_get_full_period();
        uint32_t ch2_period = encoder_2.encoder_get_full_period();
        uint32_t ch3_period = encoder_3.encoder_get_full_period();
        float ch0_rps = encoder_0.encoder_get_count();
        float ch1_rps = encoder_1.encoder_get_count();
        float ch2_rps = encoder_2.encoder_get_count();
        float ch3_rps = encoder_3.encoder_get_count();
        float rps_l = -ch3_rps;
        float rps_r = ch0_rps;
        float ms_l = rps_l * car_config::kEncoderRpsToMS;
        float ms_r = rps_r * car_config::kEncoderRpsToMS;
        int dir0 = (int)encoder_0_Dir.gpio_level_get();
        int dir1 = (int)encoder_1_Dir.gpio_level_get();
        int dir2 = (int)encoder_2_Dir.gpio_level_get();
        int dir3 = (int)encoder_3_Dir.gpio_level_get();
        int period_l = (int)ch3_period;
        int period_r = (int)ch0_period;
        bool plausible = std::fabs(ms_l) <= 3.0f && std::fabs(ms_r) <= 3.0f;
        char buf[768];
        int len = std::snprintf(buf, sizeof(buf),
            "encoder512,v=1,t_ms=%d,phase=%s,pwmL=%d,pwmR=%d,line=%d,clk=%d,Lch=3,Rch=0,rpsL=%.3f,msL=%.3f,periodL=%d,rpsR=%.3f,msR=%.3f,periodR=%d,ch0=%.3f,ch0ms=%.3f,ch0dir=%d,ch0per=%d,ch1=%.3f,ch1ms=%.3f,ch1dir=%d,ch1per=%d,ch2=%.3f,ch2ms=%.3f,ch2dir=%d,ch2per=%d,ch3=%.3f,ch3ms=%.3f,ch3dir=%d,ch3per=%d,plausible=%d\n",
            t_ms, phase, pwm_l, pwm_r, car_config::kEncoderLineCount, (int)LS_ENC_PWM_CLK,
            rps_l, ms_l, period_l,
            rps_r, ms_r, period_r,
            ch0_rps, ch0_rps * car_config::kEncoderRpsToMS, dir0, (int)ch0_period,
            ch1_rps, ch1_rps * car_config::kEncoderRpsToMS, dir1, (int)ch1_period,
            ch2_rps, ch2_rps * car_config::kEncoderRpsToMS, dir2, (int)ch2_period,
            ch3_rps, ch3_rps * car_config::kEncoderRpsToMS, dir3, (int)ch3_period,
            plausible ? 1 : 0);
        if (len > 0) client.udp_send(buf, (size_t)len);
        std::printf("%s", buf);
        std::fflush(stdout);
        clear_encoder_captures();
    };

    stop_motors();
    clear_encoder_captures();
    client.udp_send_string("encoder512,v=1,event=header,fields=t_ms phase pwmL pwmR line clk Lch Rch rpsL msL periodL rpsR msR periodR ch0 ch0ms ch0dir ch0per ch1 ch1ms ch1dir ch1per ch2 ch2ms ch2dir ch2per ch3 ch3ms ch3dir ch3per plausible\n");

    int t_ms = 0;
    for (; !exit_vofa_demo && t_ms < 1000; t_ms += 50) {
        stop_motors();
        gpio_protect_heartbeat_pulse();
        send_diag("rest", t_ms, 0, 0);
        usleep(50000);
    }
    for (int probe_ms = 0; !exit_vofa_demo && probe_ms < car_config::kEncoderDiagPulseMs; probe_ms += 20, t_ms += 20) {
        set_motor_pwm(car_config::kEncoderDiagPulsePwm, 0);
        gpio_protect_heartbeat_pulse();
        send_diag("probe_left", t_ms, car_config::kEncoderDiagPulsePwm, 0);
        usleep(20000);
    }
    for (int settle_ms = 0; !exit_vofa_demo && settle_ms < 300; settle_ms += 50, t_ms += 50) {
        stop_motors();
        gpio_protect_heartbeat_pulse();
        send_diag("settle", t_ms, 0, 0);
        usleep(50000);
    }
    for (int probe_ms = 0; !exit_vofa_demo && probe_ms < car_config::kEncoderDiagPulseMs; probe_ms += 20, t_ms += 20) {
        set_motor_pwm(0, car_config::kEncoderDiagPulsePwm);
        gpio_protect_heartbeat_pulse();
        send_diag("probe_right", t_ms, 0, car_config::kEncoderDiagPulsePwm);
        usleep(20000);
    }
    for (int settle_ms = 0; !exit_vofa_demo && settle_ms < 300; settle_ms += 50, t_ms += 50) {
        stop_motors();
        gpio_protect_heartbeat_pulse();
        send_diag("settle", t_ms, 0, 0);
        usleep(50000);
    }
    for (int probe_ms = 0; !exit_vofa_demo && probe_ms < car_config::kEncoderDiagPulseMs; probe_ms += 20, t_ms += 20) {
        set_motor_pwm(car_config::kEncoderDiagPulsePwm, car_config::kEncoderDiagPulsePwm);
        gpio_protect_heartbeat_pulse();
        send_diag("probe_both", t_ms, car_config::kEncoderDiagPulsePwm, car_config::kEncoderDiagPulsePwm);
        usleep(20000);
    }
    for (int cooldown_ms = 0; !exit_vofa_demo && cooldown_ms < 1000; cooldown_ms += 50, t_ms += 50) {
        stop_motors();
        gpio_protect_heartbeat_pulse();
        send_diag("cooldown", t_ms, 0, 0);
        usleep(50000);
    }

    stop_motors();
    send_diag("done", t_ms, 0, 0);
    client.udp_close();
    std::cout << ">>> encoder512 diagnosis finished, motors stopped." << std::endl;
}
