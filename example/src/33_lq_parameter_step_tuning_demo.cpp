#include "main.hpp"
#include "car_config.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <string>
#include <thread>
#include <vector>
#include <functional>
#include <ctime>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>

// =============================================================================
// @file    33_lq_parameter_step_tuning_demo.cpp
// @brief   PARAMETER_TUNING_METHODS.md 对应的一体化分步调参 Demo
// @note    只写一个 demo 文件；每一步是一个独立测试函数，主函数里按需调用。
//          适配：三轮摄像头差速循迹小车、F车模、蓝底白色PVC赛道、摄像头高度约30cm。
//          目标：尽量少人工干扰，输出可量化指标，方便把终端输出发回来继续调参。
// =============================================================================

extern std::atomic<bool> exit_flag;
extern ImageStatustypedef ImageStatus;
extern ImageDealDatatypedef ImageDeal[60];
extern ImageFlagtypedef ImageFlag;
extern bool g_enable_element_process;
extern int img1[60][80];
extern int imgdisplay[60][80];
extern int Left_RingsFlag_Point1_Ysite;
extern int Left_RingsFlag_Point2_Ysite;
extern int Right_RingsFlag_Point1_Ysite;
extern int Right_RingsFlag_Point2_Ysite;

extern int16 Speed_Goal;
extern float Speed_Goal_l;
extern float Speed_Goal_r;
extern float speed_encoder_L;
extern float speed_encoder_R;
extern float speed_out_l;
extern float speed_out_r;
extern int DEAD_BAND;

extern float dbg_steer_error;
extern float dbg_target_gyro_z;
extern float dbg_diff_speed;
extern float dbg_raw_steer_error;
extern float dbg_diff_steer;
extern float dbg_visual_pd_out;
extern float dbg_cmd_target_gyro_z;
extern float dbg_real_gyro_z;
extern float dbg_filtered_gyro_z;
extern float dbg_gyro_error;
extern float dbg_target_delta;
extern float dbg_dt;
extern float dbg_max_diff;
extern float dbg_diff_before_limit;
extern float dbg_speed_err_l;
extern float dbg_speed_err_r;
extern int dbg_diff_limited;
extern int dbg_imu_ok;
extern int dbg_deadband_active;
int dbg_frame_count = 0;
int dbg_spike_count = 0;
float dbg_dt_max = 0.0f;
float dbg_target_before_opposing = 0.0f;
int dbg_opposing_active = 0;
float dbg_raw_gyro_z = 0.0f;

extern void timer_thread_1();
extern void emergency_stop();
extern bool image_cv_Init(void);
extern bool image_cv_zip(void);
extern void imageprocess(void);
extern void motor_Init(void);
extern void Speed_decision(void);
extern void Speed_BiHuan(void);
extern void Speed_PID_L(void);
extern void Speed_PID_R(void);
extern int encoder_num_L(void);
extern int encoder_num_R(void);
extern void motor_Duty_L(int duty);
extern void motor_Duty_R(int duty);

static void image_cv_Deinit(void) {}

namespace param_step_demo {

static FILE *g_log_file = nullptr;

static std::string make_log_path(const char *test_name) {
    mkdir("logs", 0755);
    time_t now = time(nullptr);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char buf[256];
    snprintf(buf, sizeof(buf), "logs/%04d%02d%02d_%02d%02d%02d_%s.log",
             tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
             tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec, test_name);
    return std::string(buf);
}

static void log_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    if (g_log_file) {
        va_start(args, fmt);
        vfprintf(g_log_file, fmt, args);
        va_end(args);
        fflush(g_log_file);
    }
}

#define printf(...) param_step_demo::log_printf(__VA_ARGS__)

static int run_with_log(const char *test_name, const std::function<int()> &fn) {
    std::string path = make_log_path(test_name);
    FILE *old_log = g_log_file;
    g_log_file = fopen(path.c_str(), "w");
    if (!g_log_file) {
        ::printf("[LOG_ERROR] cannot open log file: %s\n", path.c_str());
        g_log_file = old_log;
        return fn();
    }

    ::printf("\n[LOG] 本次详细输出保存到: %s\n", path.c_str());
    fprintf(g_log_file, "[LOG] file=%s\n", path.c_str());
    fflush(g_log_file);

    int ret = fn();

    fprintf(g_log_file, "\n[LOG] TEST_RET=%d\n", ret);
    fflush(g_log_file);
    fclose(g_log_file);
    g_log_file = old_log;

    ::printf("[LOG] TEST_RET=%d，详细日志: %s\n", ret, path.c_str());
    return ret;
}

struct RunningStat {
    int n = 0;
    double sum = 0.0;
    double sum_abs = 0.0;
    double sum_sq = 0.0;
    double min_v = 1e30;
    double max_v = -1e30;
    void add(double v) {
        n++;
        sum += v;
        sum_abs += std::fabs(v);
        sum_sq += v * v;
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
    }
    double mean() const { return n ? sum / n : 0.0; }
    double mean_abs() const { return n ? sum_abs / n : 0.0; }
    double rms() const { return n ? std::sqrt(sum_sq / n) : 0.0; }
    double span() const { return n ? (max_v - min_v) : 0.0; }
    double stddev() const {
        if (!n) return 0.0;
        double m = mean();
        double v = sum_sq / n - m * m;
        return std::sqrt(v > 0.0 ? v : 0.0);
    }
};

static const char *road_type_name(int t) {
    switch (t) {
        case zero: return "zero";
        case Normol: return "Normol";
        case Straight: return "Straight";
        case Cross: return "Cross";
        case Ramp: return "Ramp";
        case LeftCirque: return "LeftCirque";
        case RightCirque: return "RightCirque";
        case Cross_ture: return "Cross_ture";
        case Barn_in: return "Barn_in";
        default: return "Unknown";
    }
}

static const char *vision_algorithm_name() {
    return "HSV_BLUE_INVERT";
}

static void print_config_snapshot() {
    printf("\n[CFG] camera: %dx%d fps=%d roi=(%d,%d,%d,%d) exposure=%d algo=%s HSV=[H%d-%d S%d-%d V%d-%d]\n",
           car_config::kCameraWidth, car_config::kCameraHeight, car_config::kCameraFps,
           car_config::kCameraRoiX, car_config::kCameraRoiY, car_config::kCameraRoiWidth, car_config::kCameraRoiHeight,
           car_config::kCameraManualExposure, vision_algorithm_name(),
           car_config::kBlueHueLower, car_config::kBlueHueUpper,
           car_config::kBlueSaturationLower, car_config::kBlueSaturationUpper,
           car_config::kBlueValueLower, car_config::kBlueValueUpper);
    printf("[CFG] image: middle=%d tow=%d scan=%d cross_scan=%d static_th=%d straight(var<%d off<=%d L<%d R<%d)\n",
           car_config::kImageMiddleLine, car_config::kImageTowPoint,
           car_config::kImageScanInterval, car_config::kImageScanIntervalCross,
           car_config::kImageThresholdStatic,
           car_config::kStraightVarianceAccThreshold, car_config::kStraightOfflineMax,
           car_config::kStraightLeftLineMax, car_config::kStraightRightLineMax);
    printf("[CFG] control: speed(P/I/D=%.1f/%.1f/%.1f curve=%d straight=%d maxDiff=%.2f minWheel=%d) steer(Kp=%.2f Kd=%.2f alpha=%.2f) gyro(P=%.3f D=%.4f alpha=%.2f ff=%.3f)\n\n",
           car_config::kSpeedPidP, car_config::kSpeedPidI, car_config::kSpeedPidD,
           car_config::kCurveSpeedGoal, car_config::kStraightSpeedGoal,
           car_config::kMaxDiffRatio, car_config::kMinWheelSpeedGoal,
           car_config::kSteerKp, car_config::kSteerKd, car_config::kSteerFilterAlpha,
           car_config::kGyroPidP, car_config::kGyroPidD, car_config::kGyroLpfAlpha,
           car_config::kGyroFeedforwardGain);
}

static bool g_camera_ready = false;

static bool ensure_camera_ready() {
    if (!g_camera_ready) {
        g_camera_ready = image_cv_Init();
        if (!g_camera_ready) {
            printf("[FAIL] camera init failed. 检查摄像头设备/占用/供电。\n");
            return false;
        }
    }
    return true;
}

static void release_camera_for_imu_test() {
    if (g_camera_ready) {
        printf("[FLOW] camera release before IMU test: stop_collect + close camera, 用于隔离摄像头采集线程/USB负载对I2C的影响。\n");
        image_cv_Deinit();
        g_camera_ready = false;
        usleep(300 * 1000);
    }
}

static bool grab_and_process() {
    if (!ensure_camera_ready()) return false;
    if (!image_cv_zip()) {
        printf("[FAIL] image_cv_zip failed. 摄像头取帧失败。\n");
        return false;
    }
    imageprocess();
    return true;
}

static void init_control_for_motor_tests() {
    static bool inited = false;
    if (!inited) {
        motor_Init();
        Speed_decision();
        inited = true;
    }
}

static void safe_stop_and_print(const char *tag) {
    Speed_Goal = 0;
    Speed_Goal_l = 0;
    Speed_Goal_r = 0;
    speed_out_l = 0;
    speed_out_r = 0;
    motor_Duty_L(0);
    motor_Duty_R(0);
    printf("[%s] STOP motor duty=0 goal=0\n", tag);
}

static double white_ratio_all() {
    int white = 0;
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 80; ++x) white += (img1[y][x] != 0);
    }
    return white / 4800.0;
}

static int white_ratio_warning(double white_ratio) {
    return (white_ratio < 0.03 || white_ratio > 0.55) ? 1 : 0;
}

static int row_white_count(int y) {
    if (y < 0) y = 0;
    if (y > 59) y = 59;
    int c = 0;
    for (int x = 0; x < 80; ++x) c += (img1[y][x] != 0);
    return c;
}

static int estimate_near_line_width() {
    int sum = 0, cnt = 0;
    for (int y = 48; y <= 58; ++y) {
        int left = ImageDeal[y].LeftBorder;
        int right = ImageDeal[y].RightBorder;
        if (left >= 0 && right < 80 && right > left) {
            sum += right - left;
            cnt++;
        }
    }
    return cnt ? sum / cnt : 0;
}

static double calc_center_variance() {
    int off = std::max(0, std::min(54, (int)ImageStatus.OFFLine));
    int denom = std::max(1, 54 - off);
    double sum = 0.0;
    for (int y = 55; y > off + 1; --y) {
        double d = ImageDeal[y].Center - ImageStatus.MiddleLine;
        sum += d * d;
    }
    return sum / denom;
}

static void print_image_line_once(const char *prefix) {
    printf("%s algo=%s Det=%d Mid=%d Err=%d Tow=%d Foresight=%d Road=%s straight=%d/%d OFF=%d LMiss=%d RMiss=%d widthNear=%d white=%.3f rowW[55/40/25]=%d/%d/%d var=%.2f DB=%d\n",
           prefix,
           vision_algorithm_name(),
           ImageStatus.Det_True,
           ImageStatus.MiddleLine,
           ImageStatus.MiddleLine - ImageStatus.Det_True,
           ImageStatus.TowPoint,
           ImageStatus.Foresight,
           road_type_name(ImageStatus.Road_type),
           ImageStatus.straight_acc,
           ImageStatus.straight_acc_flag,
           ImageStatus.OFFLine,
           ImageStatus.Left_Line,
           ImageStatus.Right_Line,
           estimate_near_line_width(),
           white_ratio_all(),
           row_white_count(55), row_white_count(40), row_white_count(25),
           calc_center_variance(), DEAD_BAND);
}

// -----------------------------------------------------------------------------
// Step 0: 调参前准备，打印配置快照，不动电机。
// -----------------------------------------------------------------------------
int test_00_precheck_config(int seconds = 3) {
    printf("\n========== TEST 00 precheck_config 调参前准备 ==========" "\n");
    printf("[目的] 确认当前编译进程序的关键参数；后续每次发日志先带这一段。\n");
    print_config_snapshot();
    printf("[要求] 电池电压保持同一区间；摄像头高度约30cm；蓝底白色PVC赛道光照稳定；每轮只改一个参数。\n");
    printf("[RETURN] 0=配置打印完成。\n");
    (void)seconds;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 1.1/1.2: 电机方向 + 编码器方向。需要架空小车，多档PWM自动测试。
// 返回值：负数表示方向/反馈明显异常；0表示左右方向正确且数量级接近。
// -----------------------------------------------------------------------------
int test_01_motor_encoder_direction(int pwm = 1200, int hold_ms = 700) {
    printf("\n========== TEST 01 motor_encoder_direction 电机/编码器方向 ==========" "\n");
    printf("[安全] 必须架空小车！自动输出多档PWM，测试左右轮启动死区、方向和编码器反馈。base_pwm=%d hold_ms=%d\n", pwm, hold_ms);
    init_control_for_motor_tests();
    safe_stop_and_print("T01");
    usleep(200 * 1000);

    struct WheelProbe {
        double forward = 0.0;
        double backward = 0.0;
        double forward_abs = 0.0;
        double backward_abs = 0.0;
        int first_forward_pwm = 0;
        int first_backward_pwm = 0;
        int pass_direction = 0;
    };

    auto run_one = [&](const char *name, bool left, int duty, int test_ms) {
        RunningStat enc;
        if (left) motor_Duty_L(duty); else motor_Duty_R(duty);
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < test_ms && !exit_flag) {
            int v = left ? encoder_num_L() : encoder_num_R();
            enc.add(v);
            usleep(20 * 1000);
        }
        if (left) motor_Duty_L(0); else motor_Duty_R(0);
        usleep(150 * 1000);
        printf("[T01] %-14s duty=%5d enc_mean=%7.2f enc_abs=%7.2f enc_min=%7.1f enc_max=%7.1f samples=%d\n",
               name, duty, enc.mean(), enc.mean_abs(), enc.n ? enc.min_v : 0, enc.n ? enc.max_v : 0, enc.n);
        return enc;
    };

    const int pwm_steps[] = {1200, 1800, 2400};
    const int step_count = sizeof(pwm_steps) / sizeof(pwm_steps[0]);
    const double move_threshold = 1.0;

    auto probe_wheel = [&](const char *wheel_name, bool left) {
        WheelProbe result;
        printf("[T01_STEP] wheel=%s move_threshold=%.1f\n", wheel_name, move_threshold);
        for (int i = 0; i < step_count && !exit_flag; ++i) {
            int duty = pwm_steps[i];
            char name_f[32];
            char name_b[32];
            snprintf(name_f, sizeof(name_f), "%s_F_%d", wheel_name, duty);
            snprintf(name_b, sizeof(name_b), "%s_B_%d", wheel_name, duty);

            RunningStat f = run_one(name_f, left, duty, hold_ms);
            RunningStat b = run_one(name_b, left, -duty, hold_ms);

            if (result.first_forward_pwm == 0 && f.mean_abs() >= move_threshold)
                result.first_forward_pwm = duty;
            if (result.first_backward_pwm == 0 && b.mean_abs() >= move_threshold)
                result.first_backward_pwm = duty;

            result.forward = f.mean();
            result.backward = b.mean();
            result.forward_abs = f.mean_abs();
            result.backward_abs = b.mean_abs();

            bool dir_ok = (f.mean() > move_threshold && b.mean() < -move_threshold);
            bool no_feedback = (f.mean_abs() < move_threshold && b.mean_abs() < move_threshold);
            printf("[T01_STEP_SUMMARY] wheel=%s pwm=%d dir_ok=%d no_feedback=%d f=%.2f b=%.2f f_abs=%.2f b_abs=%.2f\n",
                   wheel_name, duty, dir_ok ? 1 : 0, no_feedback ? 1 : 0,
                   f.mean(), b.mean(), f.mean_abs(), b.mean_abs());

            if (dir_ok) {
                result.pass_direction = 1;
                break;
            }
        }
        printf("[T01_WHEEL_SUMMARY] wheel=%s pass=%d firstF=%d firstB=%d lastF=%.2f lastB=%.2f lastAbs=%.2f/%.2f\n",
               wheel_name, result.pass_direction, result.first_forward_pwm, result.first_backward_pwm,
               result.forward, result.backward, result.forward_abs, result.backward_abs);
        return result;
    };

    WheelProbe left = probe_wheel("LEFT", true);
    WheelProbe right = probe_wheel("RIGHT", false);
    safe_stop_and_print("T01");

    int pass_l = left.pass_direction;
    int pass_r = right.pass_direction;
    double ratio = (right.forward_abs > 1e-3) ? std::fabs(left.forward_abs / right.forward_abs) : 999.0;
    int balance_ok = (ratio > 0.55 && ratio < 1.80) ? 1 : 0;
    printf("[T01_SUMMARY] passL=%d passR=%d balance=%d L/R_abs_ratio=%.2f L_start=%d/%d R_start=%d/%d\n",
           pass_l, pass_r, balance_ok, ratio,
           left.first_forward_pwm, left.first_backward_pwm,
           right.first_forward_pwm, right.first_backward_pwm);
    printf("[调参意义] 正转编码器应为正，反转应为负；低档不动但高档正常=机械/电机死区大；高档仍无反馈=接线/驱动/编码器通道问题。\n");
    printf("[RETURN] 0=通过；-1=左轮方向/反馈异常；-2=右轮异常；-3=左右速度差太大。\n");
    if (!pass_l) return -1;
    if (!pass_r) return -2;
    if (!balance_ok) return -3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 1.3 + 摄像头角度核心测试：静止放直道中心，自动统计视觉偏差、可视距离、宽度梯度。
// 返回值：0=中心和角度基本可用；1=偏左/右需调中心或机械；2=看得太近/角度太低；3=看得太远/噪声大。
// -----------------------------------------------------------------------------
int test_02_camera_center_angle_static(int frames = 120) {
    printf("\n========== TEST 02 camera_center_angle_static 摄像头中心/角度静态 ==========" "\n");
    printf("[摆放] 小车放在直道正中，车头正对赛道。无需推车。摄像头高度约30cm，本测试重点判断摄像头俯仰角。\n");
    g_enable_element_process = false;

    RunningStat err, det, off, width, white, variance;
    int straight_cnt = 0, bad_center_cnt = 0, lost_cnt = 0;
    for (int i = 0; i < frames && !exit_flag; ++i) {
        if (!grab_and_process()) return -1;
        int e = ImageStatus.MiddleLine - ImageStatus.Det_True;
        int w = estimate_near_line_width();
        err.add(e);
        det.add(ImageStatus.Det_True);
        off.add(ImageStatus.OFFLine);
        width.add(w);
        white.add(white_ratio_all());
        variance.add(calc_center_variance());
        if (ImageStatus.straight_acc_flag) straight_cnt++;
        if (std::fabs(e) > 2) bad_center_cnt++;
        if (ImageStatus.OFFLine > 15 || w <= 3) lost_cnt++;
        if (i % 20 == 0) print_image_line_once("[T02_FRAME]");
        usleep(20 * 1000);
    }
    double straight_rate = frames ? (double)straight_cnt / frames : 0;
    double bad_center_rate = frames ? (double)bad_center_cnt / frames : 0;
    double lost_rate = frames ? (double)lost_cnt / frames : 0;
    printf("[T02_SUMMARY] frames=%d err_mean=%.2f err_abs=%.2f err_rms=%.2f det_mean=%.2f OFF_mean=%.2f width_mean=%.2f white_mean=%.3f var_mean=%.2f straight_rate=%.2f bad_center_rate=%.2f lost_rate=%.2f\n",
           frames, err.mean(), err.mean_abs(), err.rms(), det.mean(), off.mean(), width.mean(), white.mean(), variance.mean(), straight_rate, bad_center_rate, lost_rate);
    printf("[角度判断] OFF_mean大/width小/丢线高 => 摄像头角度太低看太近或ROI太靠下；err_abs大但稳定 => 先机械居中或微调kImageMiddleLine；white异常先看曝光/ROI/OTSU二值图。\n");
    printf("[RETURN] 0=可进入OTSU质量测试；1=中心偏差>2；2=疑似角度太低/看太近/丢线；3=直道判断不稳或远端噪声大。\n");
    if (lost_rate > 0.20 || off.mean() > 15 || width.mean() < 5) return 2;
    if (err.mean_abs() > 2.0 || bad_center_rate > 0.35) return 1;
    if (straight_rate < 0.70 || variance.mean() > car_config::kStraightVarianceAccThreshold * 2.0) return 3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 2/3: 曝光 + OTSU 二值质量测试。不改参数，打印二值化稳定性指标。
// 返回值：0=mask稳定；1=白点比例异常；2=闪烁大；3=边界/丢线异常。
// -----------------------------------------------------------------------------
int test_03_exposure_otsu_quality(int frames = 160) {
    printf("\n========== TEST 03 exposure_otsu_quality 曝光/OTSU二值质量 ==========" "\n");
    printf("[摆放] 依次对准直道/弯道/阴影/反光区域均可。先固定车不动跑一段，再可轻推很慢经过问题区域。\n");
    printf("[当前] algorithm=%s。OTSU会自动选阈值，本测试重点看white比例稳定性、Det抖动和边界丢失。\n",
           vision_algorithm_name());
    g_enable_element_process = false;

    RunningStat wr, row55, row40, row25, off, left_miss, right_miss, det_err, det_delta, width;
    int flicker_cnt = 0, lost_cnt = 0, white_bad_cnt = 0, det_jump_cnt = 0;
    double last_wr = -1.0;
    int last_det = 0;
    for (int i = 0; i < frames && !exit_flag; ++i) {
        if (!grab_and_process()) return -1;
        double w = white_ratio_all();
        int ddet = (i == 0) ? 0 : ImageStatus.Det_True - last_det;
        wr.add(w);
        row55.add(row_white_count(55));
        row40.add(row_white_count(40));
        row25.add(row_white_count(25));
        off.add(ImageStatus.OFFLine);
        left_miss.add(ImageStatus.Left_Line);
        right_miss.add(ImageStatus.Right_Line);
        det_err.add(ImageStatus.MiddleLine - ImageStatus.Det_True);
        det_delta.add(ddet);
        width.add(estimate_near_line_width());
        if (last_wr >= 0 && std::fabs(w - last_wr) > 0.10) flicker_cnt++;
        if (white_ratio_warning(w)) white_bad_cnt++;
        if (std::fabs((double)ddet) > 6.0) det_jump_cnt++;
        if (ImageStatus.OFFLine > 18 || ImageStatus.Left_Line > 12 || ImageStatus.Right_Line > 12) lost_cnt++;
        last_wr = w;
        last_det = ImageStatus.Det_True;
        if (i % 20 == 0) print_image_line_once("[T03_FRAME]");
        usleep(20 * 1000);
    }
    double flicker_rate = frames ? (double)flicker_cnt / frames : 0;
    double lost_rate = frames ? (double)lost_cnt / frames : 0;
    double white_bad_rate = frames ? (double)white_bad_cnt / frames : 0;
    double det_jump_rate = frames ? (double)det_jump_cnt / frames : 0;
    int white_abnormal = (wr.mean() < 0.03 || wr.mean() > 0.55 || white_bad_rate > 0.20) ? 1 : 0;
    printf("[T03_SUMMARY] frames=%d algo=%s white_mean=%.3f white_std=%.3f white_span=%.3f white_bad=%.2f rowW55/40/25=%.1f/%.1f/%.1f OFF_mean=%.2f width=%.2f LMiss=%.2f RMiss=%.2f err_abs=%.2f dDet_std=%.2f det_jump=%.2f flicker_rate=%.2f lost_rate=%.2f\n",
           frames, vision_algorithm_name(), wr.mean(), wr.stddev(), wr.span(), white_bad_rate,
           row55.mean(), row40.mean(), row25.mean(), off.mean(), width.mean(),
           left_miss.mean(), right_miss.mean(), det_err.mean_abs(), det_delta.stddev(),
           det_jump_rate, flicker_rate, lost_rate);
    printf("[调参意义] OTSU不靠固定阈值压white；white异常多看曝光/ROI/反光，dDet大看二值毛刺和前瞻行，lost高先调摄像头角度/ROI。\n");
    printf("[RETURN] 0=曝光OTSU基本稳定；1=white比例异常；2=mask/Det闪烁；3=边界丢失。\n");
    if (white_abnormal) return 1;
    if (flicker_rate > 0.15 || wr.span() > 0.30 || det_jump_rate > 0.15 || det_delta.stddev() > 5.0) return 2;
    if (lost_rate > 0.20) return 3;
    return 0;
}

int test_03_exposure_hsv_quality(int frames = 160) {
    return test_03_exposure_otsu_quality(frames);
}

// -----------------------------------------------------------------------------
// Step 4/5/6: ROI、前瞻、扫线、直道判断综合图像算法测试。
// 返回值：0=图像算法稳定；1=ROI/前瞻可能过近；2=扫线丢边；3=直道误判/漏判。
// -----------------------------------------------------------------------------
int test_04_roi_scan_straight(int frames = 180) {
    printf("\n========== TEST 04 roi_scan_straight ROI/前瞻/扫线/直道判断 ==========" "\n");
    printf("[运行] 可放在直道中心静止，也可低速手推过普通弯。输出会显示入弯是否提前、边界是否连续、直道判断是否稳定。\n");
    g_enable_element_process = false;

    RunningStat err, derr, off, width, var, left_miss, right_miss;
    int straight_cnt = 0, cross_cnt = 0, big_jump_cnt = 0, edge_bad_cnt = 0;
    int last_err = 0;
    for (int i = 0; i < frames && !exit_flag; ++i) {
        if (!grab_and_process()) return -1;
        int e = ImageStatus.MiddleLine - ImageStatus.Det_True;
        int de = (i == 0) ? 0 : e - last_err;
        last_err = e;
        err.add(e);
        derr.add(de);
        off.add(ImageStatus.OFFLine);
        width.add(estimate_near_line_width());
        var.add(calc_center_variance());
        left_miss.add(ImageStatus.Left_Line);
        right_miss.add(ImageStatus.Right_Line);
        if (ImageStatus.straight_acc_flag) straight_cnt++;
        if (ImageStatus.Road_type == Cross || ImageStatus.Road_type == Cross_ture) cross_cnt++;
        if (std::fabs(de) > 8) big_jump_cnt++;
        if (ImageStatus.Left_Line > 10 || ImageStatus.Right_Line > 10 || estimate_near_line_width() < 5) edge_bad_cnt++;
        if (i % 20 == 0) print_image_line_once("[T04_FRAME]");
        usleep(20 * 1000);
    }
    double straight_rate = frames ? (double)straight_cnt / frames : 0;
    double jump_rate = frames ? (double)big_jump_cnt / frames : 0;
    double edge_bad_rate = frames ? (double)edge_bad_cnt / frames : 0;
    double cross_rate = frames ? (double)cross_cnt / frames : 0;
    printf("[T04_SUMMARY] frames=%d err_abs=%.2f dErr_abs=%.2f dErr_span=%.2f OFF=%.2f width=%.2f var=%.2f LMiss=%.2f RMiss=%.2f straight_rate=%.2f cross_rate=%.2f jump_rate=%.2f edge_bad_rate=%.2f\n",
           frames, err.mean_abs(), derr.mean_abs(), derr.span(), off.mean(), width.mean(), var.mean(), left_miss.mean(), right_miss.mean(), straight_rate, cross_rate, jump_rate, edge_bad_rate);
    printf("[调参意义] 入弯晚通常减小kImageTowPoint/kCameraRoiY；直道蛇形且jump高通常增大TowPoint/RoiY或减小扫线范围；edge_bad高先调扫线/ROI/二值形态学。\n");
    printf("[RETURN] 0=稳定；1=疑似看太近/入弯晚；2=扫线边界不连续；3=直道判断不稳。\n");
    if (off.mean() > 18 || width.mean() < 5) return 1;
    if (edge_bad_rate > 0.20 || jump_rate > 0.20) return 2;
    if (straight_rate < 0.65 && err.mean_abs() < 4.0 && var.mean() < car_config::kStraightVarianceAccThreshold * 1.5) return 3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 7: 速度环闭环架空测试。不会跑视觉，左右轮同目标阶跃。
// 返回值：0=跟随尚可；1=响应慢；2=振荡/过冲；3=左右不一致。
// -----------------------------------------------------------------------------
int test_05_speed_loop_airborne(int target = 50, int seconds = 6) {
    printf("\n========== TEST 05 speed_loop_airborne 速度环架空阶跃 ==========" "\n");
    printf("[安全] 必须架空！自动闭环给左右同目标速度 target=%d seconds=%d。\n", target, seconds);
    init_control_for_motor_tests();
    Speed_Goal = 0;
    Speed_Goal_l = 0;
    Speed_Goal_r = 0;
    speed_out_l = speed_out_r = 0;
    usleep(300 * 1000);

    RunningStat err_l, err_r, enc_l, enc_r, pwm_l, pwm_r, dpwm_l, dpwm_r;
    int sat_cnt = 0, sign_flip_l = 0, sign_flip_r = 0, major_flip_l = 0, major_flip_r = 0;
    double last_el = 0, last_er = 0;
    double last_pwm_l = 0, last_pwm_r = 0;
    int total = seconds * 50;
    const double major_err = std::max(3.0, std::fabs((double)target) * 0.12);
    for (int i = 0; i < total && !exit_flag; ++i) {
        Speed_Goal = target;
        Speed_Goal_l = target;
        Speed_Goal_r = target;
        speed_encoder_L = encoder_num_L();
        speed_encoder_R = encoder_num_R();
        Speed_PID_L();
        Speed_PID_R();
        motor_Duty_L((int)speed_out_l);
        motor_Duty_R((int)speed_out_r);

        double el = Speed_Goal_l - speed_encoder_L;
        double er = Speed_Goal_r - speed_encoder_R;
        err_l.add(el); err_r.add(er);
        enc_l.add(speed_encoder_L); enc_r.add(speed_encoder_R);
        pwm_l.add(speed_out_l); pwm_r.add(speed_out_r);
        if (std::fabs(speed_out_l) > car_config::kMotorMaxPwm * 0.95 || std::fabs(speed_out_r) > car_config::kMotorMaxPwm * 0.95) sat_cnt++;
        if (i > 0) {
            dpwm_l.add(speed_out_l - last_pwm_l);
            dpwm_r.add(speed_out_r - last_pwm_r);
        }
        if (i > 5 && el * last_el < 0) {
            sign_flip_l++;
            if (std::fabs(el) > major_err && std::fabs(last_el) > major_err) major_flip_l++;
        }
        if (i > 5 && er * last_er < 0) {
            sign_flip_r++;
            if (std::fabs(er) > major_err && std::fabs(last_er) > major_err) major_flip_r++;
        }
        last_el = el; last_er = er;
        last_pwm_l = speed_out_l; last_pwm_r = speed_out_r;

        if (i % 10 == 0) {
            printf("[T05_FRAME] t=%.1f target=%d Enc=%.1f/%.1f Err=%.1f/%.1f PWM=%.0f/%.0f\n",
                   i * 0.02, target, speed_encoder_L, speed_encoder_R, el, er, speed_out_l, speed_out_r);
        }
        usleep(20 * 1000);
    }
    safe_stop_and_print("T05");
    double balance = (std::fabs(enc_r.mean()) > 1e-3) ? std::fabs(enc_l.mean() / enc_r.mean()) : 999.0;
    double sat_rate = total ? (double)sat_cnt / total : 0;
    printf("[T05_SUMMARY] target=%d enc_mean=%.2f/%.2f err_abs=%.2f/%.2f err_rms=%.2f/%.2f pwm_mean=%.0f/%.0f pwm_std=%.0f/%.0f dpwm_abs=%.0f/%.0f sat_rate=%.2f flip=%d/%d major_flip=%d/%d balance=%.2f\n",
           target, enc_l.mean(), enc_r.mean(), err_l.mean_abs(), err_r.mean_abs(), err_l.rms(), err_r.rms(),
           pwm_l.mean(), pwm_r.mean(), pwm_l.stddev(), pwm_r.stddev(), dpwm_l.mean_abs(), dpwm_r.mean_abs(),
           sat_rate, sign_flip_l, sign_flip_r, major_flip_l, major_flip_r, balance);
    printf("[调参意义] err_abs长期大=速度环弱或动力不足；flip多但major_flip少=围绕目标小幅量化过零，通常可接受；major_flip/dpwm_abs大=明显振荡；balance偏离=左右轮机械/编码器/PWM差异。\n");
    printf("[RETURN] 0=可用；1=响应慢/误差大；2=振荡；3=左右不一致。\n");
    if (err_l.mean_abs() > target * 0.55 || err_r.mean_abs() > target * 0.55 || sat_rate > 0.20) return 1;
    if (major_flip_l > seconds * 2 || major_flip_r > seconds * 2) return 2;
    if ((sign_flip_l > seconds * 8 || sign_flip_r > seconds * 8) &&
        (err_l.rms() > std::fabs((double)target) * 0.18 || err_r.rms() > std::fabs((double)target) * 0.18 ||
         dpwm_l.mean_abs() > car_config::kMotorMaxPwm * 0.08 || dpwm_r.mean_abs() > car_config::kMotorMaxPwm * 0.08)) return 2;
    if (balance < 0.65 || balance > 1.55) return 3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 8/9/12: 低速视觉外环 + 陀螺仪角速度环 + 普通赛道低速闭环。
// 小车可直接放赛道低速跑，打印有效调参指标。
// 返回值：0=低速闭环可用；1=视觉PD饱和；2=陀螺仪跟不上；3=差速限幅/速度跟不上；4=图像不稳。
// -----------------------------------------------------------------------------
int test_06_low_speed_visual_gyro_closed_loop(int seconds = 10) {
    printf("\n========== TEST 06 low_speed_visual_gyro_closed_loop 低速视觉+角速度闭环 ==========" "\n");
    printf("[运行] 小车放在普通赛道起点，程序会自动低速循迹。建议先留出安全空间，必要时Ctrl+C。seconds=%d\n", seconds);
    if (!ensure_camera_ready()) return -1;
    init_control_for_motor_tests();
    g_enable_element_process = false;

    std::thread ctrl_thread(timer_thread_1);
    RunningStat raw, filt, pd, tgt, cmd, real, gf, gerr, diff, se_l, se_r, dtmax;
    int cap_cnt = 0, imu_bad_cnt = 0, db_cnt = 0, img_bad_cnt = 0, img_warn_cnt = 0, pd_sat_cnt = 0;
    int total_prints = 0;
    auto start = std::chrono::steady_clock::now();
    auto last_print = start;
    while (!exit_flag) {
        if (!grab_and_process()) { img_bad_cnt++; break; }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= seconds) break;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 100) {
            last_print = now;
            total_prints++;
            raw.add(dbg_raw_steer_error);
            filt.add(dbg_steer_error);
            pd.add(dbg_visual_pd_out);
            tgt.add(dbg_target_gyro_z);
            cmd.add(dbg_cmd_target_gyro_z);
            real.add(dbg_real_gyro_z);
            gf.add(dbg_filtered_gyro_z);
            gerr.add(dbg_gyro_error);
            diff.add(dbg_diff_speed);
            se_l.add(dbg_speed_err_l);
            se_r.add(dbg_speed_err_r);
            dtmax.add(dbg_dt_max);
            if (dbg_diff_limited) cap_cnt++;
            if (!dbg_imu_ok) imu_bad_cnt++;
            if (dbg_deadband_active) db_cnt++;
            bool edge_warn = (ImageStatus.Left_Line > 15 || ImageStatus.Right_Line > 15);
            bool severe_img_bad = (ImageStatus.OFFLine > 18 ||
                                   (ImageStatus.Left_Line > 25 && ImageStatus.Right_Line > 25) ||
                                   std::abs(ImageStatus.MiddleLine - ImageStatus.Det_True) > 26);
            if (edge_warn) img_warn_cnt++;
            if (severe_img_bad) img_bad_cnt++;
            if (std::fabs(dbg_visual_pd_out) > car_config::kVisualGyroLimitDps * 0.90f) pd_sat_cnt++;
            printf("[T06] Raw:%5.1f Filt:%5.1f dE:%5.1f | PD:%6.1f Tgt:%6.1f Cmd:%6.1f Real:%6.1f GF:%6.1f GErr:%6.1f dT:%5.1f | D0:%6.1f D:%6.1f Cap:%d Max:%4.1f | L:%5.1f R:%5.1f Enc:%4.0f/%4.0f SE:%5.1f/%5.1f | imgErr:%d Road:%s OFF:%d L/R:%d/%d dt:%5.3f IMU:%d DB:%d | fc:%d sp:%d dt_max:%.3f rg:%.1f oa:%d tbo:%.1f\n",
                   dbg_raw_steer_error, dbg_steer_error, dbg_diff_steer,
                   dbg_visual_pd_out, dbg_target_gyro_z, dbg_cmd_target_gyro_z,
                   dbg_real_gyro_z, dbg_filtered_gyro_z, dbg_gyro_error, dbg_target_delta,
                   dbg_diff_before_limit, dbg_diff_speed, dbg_diff_limited, dbg_max_diff,
                   Speed_Goal_l, Speed_Goal_r, speed_encoder_L, speed_encoder_R,
                   dbg_speed_err_l, dbg_speed_err_r,
                   ImageStatus.MiddleLine - ImageStatus.Det_True, road_type_name(ImageStatus.Road_type),
                   ImageStatus.OFFLine, ImageStatus.Left_Line, ImageStatus.Right_Line,
                   dbg_dt, dbg_imu_ok, dbg_deadband_active,
                   dbg_frame_count, dbg_spike_count, dbg_dt_max, dbg_raw_gyro_z,
                   dbg_opposing_active, dbg_target_before_opposing);
            dbg_frame_count = 0; dbg_spike_count = 0; dbg_dt_max = 0.0f;
        }
        usleep(5 * 1000);
    }
    exit_flag = true;
    if (ctrl_thread.joinable()) ctrl_thread.join();
    safe_stop_and_print("T06");
    exit_flag = false; // 方便同一次运行继续调用其他测试；若主程序要退出可不用管

    double cap_rate = total_prints ? (double)cap_cnt / total_prints : 0;
    double imu_bad_rate = total_prints ? (double)imu_bad_cnt / total_prints : 0;
    double img_bad_rate = total_prints ? (double)img_bad_cnt / total_prints : 0;
    double img_warn_rate = total_prints ? (double)img_warn_cnt / total_prints : 0;
    double pd_sat_rate = total_prints ? (double)pd_sat_cnt / total_prints : 0;
    printf("[T06_SUMMARY] samples=%d raw_abs=%.2f filt_abs=%.2f PD_abs=%.2f Tgt_abs=%.2f Real_abs=%.2f GF_abs=%.2f GErr_abs=%.2f Diff_abs=%.2f SE_abs=%.2f/%.2f cap_rate=%.2f pd_sat_rate=%.2f imu_bad=%.2f img_bad=%.2f img_warn=%.2f dtmax_mean=%.3f DB_cnt=%d\n",
           total_prints, raw.mean_abs(), filt.mean_abs(), pd.mean_abs(), tgt.mean_abs(), real.mean_abs(), gf.mean_abs(), gerr.mean_abs(), diff.mean_abs(), se_l.mean_abs(), se_r.mean_abs(), cap_rate, pd_sat_rate, imu_bad_rate, img_bad_rate, img_warn_rate, dtmax.mean(), db_cnt);
    printf("[调参意义] pd_sat高=外环/前瞻过猛；GErr_abs>20且持续=角速度环/GF跟不上；cap高=差速受限；img_bad=严重丢线/检测贴边；img_warn=弯道单侧边界警告，需结合是否能稳定回中。\n");
    printf("[RETURN] 0=低速闭环可继续；1=PD饱和；2=陀螺仪跟踪差；3=差速/速度受限；4=图像不稳。\n");
    if (img_bad_rate > 0.15) return 4;
    if (pd_sat_rate > 0.25) return 1;
    if (gerr.mean_abs() > 20.0 || imu_bad_rate > 0.05) return 2;
    if (cap_rate > 0.25 || se_l.mean_abs() > 6.0 || se_r.mean_abs() > 6.0) return 3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 10: 提速前/后整体稳定性测试。默认元素关闭，只评估速度提升后的直道/弯道表现。
// 返回值：0=可保留当前速度；1=直道蛇形；2=入弯晚/转不过；3=弯道内切/过冲；4=随机丢线。
// -----------------------------------------------------------------------------
int test_07_speed_up_regression(int seconds = 12) {
    printf("\n========== TEST 07 speed_up_regression 速度提升回归 ==========" "\n");
    printf("[运行] 用当前car_config里的速度跑完整普通赛道片段，元素关闭。把SUMMARY和固定失误位置告诉我。\n");
    int ret = test_06_low_speed_visual_gyro_closed_loop(seconds);
    if (ret == 4) return 4;
    // 基于T06已输出详细指标，这里只做语义映射，便于主函数按返回值判断。
    if (ret == 1) return 1;
    if (ret == 2) return 2;
    if (ret == 3) return 3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 11/15-19: 元素判断调试。开启元素，只识别/处理现有代码支持的元素并输出状态。
// 返回值：0=元素状态稳定；1=误判频繁；2=漏检/状态不变；3=图像不稳。
// -----------------------------------------------------------------------------
int test_08_element_debug(int seconds = 25) {
    printf("\n========== TEST 08 element_debug 元素判断/处理 ==========" "\n");
    printf("[运行] 只在普通循迹稳定后调用。开启元素判断，低速经过十字/圆环区域，输出Road_type状态变化。seconds=%d\n", seconds);
    if (!ensure_camera_ready()) return -1;
    init_control_for_motor_tests();
    g_enable_element_process = true;

    std::thread ctrl_thread(timer_thread_1);
    int last_road = -999;
    int changes = 0, special_cnt = 0, unknown_cnt = 0, img_bad_cnt = 0, img_warn_cnt = 0, samples = 0;
    auto start = std::chrono::steady_clock::now();
    auto last_print = start;
    while (!exit_flag) {
        if (!grab_and_process()) { img_bad_cnt++; break; }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= seconds) break;
        if (ImageStatus.Road_type != last_road) {
            printf("[T08_EVENT] t=%.2f RoadChange %s -> %s Det=%d Err=%d OFF=%d L/R=%d/%d straight=%d/%d\n",
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0,
                   road_type_name(last_road), road_type_name(ImageStatus.Road_type),
                   ImageStatus.Det_True, ImageStatus.MiddleLine - ImageStatus.Det_True,
                   ImageStatus.OFFLine, ImageStatus.Left_Line, ImageStatus.Right_Line,
                   ImageStatus.straight_acc, ImageStatus.straight_acc_flag);
            last_road = ImageStatus.Road_type;
            changes++;
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 200) {
            last_print = now;
            samples++;
            if (ImageStatus.Road_type == zero) unknown_cnt++;
            bool special_seen = (ImageStatus.Road_type == Cross ||
                                 ImageStatus.Road_type == Cross_ture ||
                                 ImageStatus.Road_type == Ramp ||
                                 ImageStatus.Road_type == LeftCirque ||
                                 ImageStatus.Road_type == RightCirque ||
                                 ImageStatus.Road_type == Barn_in ||
                                 ImageFlag.image_element_rings != 0 ||
                                 ImageFlag.image_element_rings_flag != 0);
            if (special_seen) special_cnt++;
            bool edge_warn = (ImageStatus.Left_Line > 15 || ImageStatus.Right_Line > 15);
            bool severe_img_bad = (ImageStatus.OFFLine > 18 ||
                                   (ImageStatus.Left_Line > 25 && ImageStatus.Right_Line > 25) ||
                                   std::abs(ImageStatus.MiddleLine - ImageStatus.Det_True) > 26);
            if (edge_warn) img_warn_cnt++;
            if (severe_img_bad) img_bad_cnt++;
            printf("[T08] Road=%s Det=%d Err=%d OFF=%d L/R=%d/%d ring=%d/%d lp=%d/%d rp=%d/%d straight=%d/%d D=%.1f Tgt=%.1f GErr=%.1f Enc=%.0f/%.0f\n",
                   road_type_name(ImageStatus.Road_type), ImageStatus.Det_True,
                   ImageStatus.MiddleLine - ImageStatus.Det_True, ImageStatus.OFFLine,
                   ImageStatus.Left_Line, ImageStatus.Right_Line,
                   ImageFlag.image_element_rings, ImageFlag.image_element_rings_flag,
                   Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite,
                   Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite,
                   ImageStatus.straight_acc, ImageStatus.straight_acc_flag,
                   dbg_diff_speed, dbg_target_gyro_z, dbg_gyro_error,
                   speed_encoder_L, speed_encoder_R);
        }
        usleep(5 * 1000);
    }
    exit_flag = true;
    if (ctrl_thread.joinable()) ctrl_thread.join();
    safe_stop_and_print("T08");
    exit_flag = false;
    double img_bad_rate = samples ? (double)img_bad_cnt / samples : 0.0;
    double img_warn_rate = samples ? (double)img_warn_cnt / samples : 0.0;
    double unknown_rate = samples ? (double)unknown_cnt / samples : 0.0;
    double special_rate = samples ? (double)special_cnt / samples : 0.0;
    printf("[T08_SUMMARY] samples=%d road_changes=%d special_rate=%.2f unknown_rate=%.2f img_bad_rate=%.2f img_warn_rate=%.2f\n", samples, changes, special_rate, unknown_rate, img_bad_rate, img_warn_rate);
    printf("[调参意义] road_changes过多=误判/抖动；special_rate=0 表示本轮没跑到元素或元素漏检；img_bad=严重丢线/检测贴边；img_warn=元素区单侧边界警告，需结合是否稳定通过。\n");
    printf("[RETURN] 0=元素状态可继续细调；1=误判频繁；2=疑似漏检；3=图像不稳。\n");
    if (img_bad_rate > 0.20) return 3;
    if (changes > seconds * 2) return 1;
    if (special_cnt == 0) return 2;
    if (changes <= 1 && unknown_rate < 0.10) return 2;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 20: 不同电压/光照回归。只采集图像和关键控制，不强制解释。
// -----------------------------------------------------------------------------
int test_09_voltage_light_regression(int frames = 200) {
    printf("\n========== TEST 09 voltage_light_regression 电压/光照回归 ==========" "\n");
    printf("[运行] 换电池电压或光照后调用。建议先静止，再低速手推问题区域。\n");
    int r2 = test_02_camera_center_angle_static(frames / 2);
    int r3 = test_03_exposure_otsu_quality(frames / 2);
    printf("[T09_SUMMARY] center_angle_ret=%d exposure_otsu_ret=%d\n", r2, r3);
    printf("[RETURN] 0=回归通过；非0=先按对应测试修图像/角度。\n");
    return (r2 == 0 && r3 == 0) ? 0 : 1;
}

// -----------------------------------------------------------------------------
// Step 10: IMU / MPU6050 静止噪声测试。无需赛道，不动电机。
// 返回值：0=静止噪声正常；1=噪声偏大；2=存在明显尖峰/I2C干扰；3=读取失败。
// -----------------------------------------------------------------------------
int test_10_gyro_static_noise(int seconds = 10) {
    printf("\n========== TEST 10 gyro_static_noise IMU静止噪声 ==========" "\n");
    printf("[摆放] 小车静止放在桌面/地面，不要碰车，不需要赛道。本测试不会输出电机PWM。seconds=%d\n", seconds);
    printf("[目的] 统计 gyro_z 静止零漂、噪声、尖峰，用于判断 kGyroDeadBandDps / kGyroLpfAlpha / I2C稳定性。\n");

    safe_stop_and_print("T10");
    MyMpu6050 local_imu;
    if (!local_imu.init()) {
        printf("[T10_SUMMARY] init_failed=1\n");
        printf("[RETURN] 3=MPU6050初始化失败。\n");
        return 3;
    }

    RunningStat gx, gy, gz, ax, ay, az, temp, dt_us_stat;
    int read_fail_cnt = 0;
    int abs_spike_cnt = 0;
    int any_abs_spike_cnt = 0;
    int moderate_spike_cnt = 0;
    int rate_spike_cnt = 0;
    int any_rate_spike_cnt = 0;
    int accel_spike_cnt = 0;
    int detail_print_cnt = 0;
    int deadband15_out_cnt = 0;
    int deadband30_out_cnt = 0;
    float last_gx = 0.0f;
    float last_gy = 0.0f;
    float last_gz = 0.0f;
    bool has_last = false;
    auto last_sample_time = std::chrono::steady_clock::now();

    const int sample_period_us = 10000; // 100Hz
    const int total = std::max(1, seconds * 100);
    for (int i = 0; i < total && !exit_flag; ++i) {
        auto sample_time = std::chrono::steady_clock::now();
        if (i > 0) {
            dt_us_stat.add((double)std::chrono::duration_cast<std::chrono::microseconds>(sample_time - last_sample_time).count());
        }
        last_sample_time = sample_time;

        Mpu6050Data data;
        if (!local_imu.read_processed_data(data)) {
            read_fail_cnt++;
            usleep(sample_period_us);
            continue;
        }

        gx.add(data.gyro_x);
        gy.add(data.gyro_y);
        gz.add(data.gyro_z);
        ax.add(data.accel_x);
        ay.add(data.accel_y);
        az.add(data.accel_z);
        temp.add(data.temp);

        bool gz_abs_spike = std::fabs(data.gyro_z) > car_config::kGyroRawSpikeRejectDps;
        bool any_abs_spike = std::fabs(data.gyro_x) > car_config::kGyroRawSpikeRejectDps ||
                             std::fabs(data.gyro_y) > car_config::kGyroRawSpikeRejectDps ||
                             gz_abs_spike;
        bool accel_spike = std::fabs(data.accel_x) > 1.2f ||
                           std::fabs(data.accel_y) > 1.2f ||
                           std::fabs(data.accel_z) > 1.2f;
        bool gz_rate_spike = false;
        bool any_rate_spike = false;
        if (gz_abs_spike) abs_spike_cnt++;
        if (any_abs_spike) any_abs_spike_cnt++;
        if (accel_spike) accel_spike_cnt++;
        if (std::fabs(data.gyro_z) > 20.0f) moderate_spike_cnt++;
        if (has_last) {
            gz_rate_spike = std::fabs(data.gyro_z - last_gz) > 200.0f;
            any_rate_spike = std::fabs(data.gyro_x - last_gx) > 200.0f ||
                             std::fabs(data.gyro_y - last_gy) > 200.0f ||
                             gz_rate_spike;
            if (gz_rate_spike) rate_spike_cnt++;
            if (any_rate_spike) any_rate_spike_cnt++;
        }
        if (std::fabs(data.gyro_z) > 1.5f) deadband15_out_cnt++;
        if (std::fabs(data.gyro_z) > 3.0f) deadband30_out_cnt++;
        if ((any_abs_spike || any_rate_spike || accel_spike) && detail_print_cnt < 24) {
            printf("[T10_SPIKE_DETAIL] i=%d t=%.2f anyAbs=%d gzAbs=%d anyRate=%d gzRate=%d accelSpike=%d "
                   "gyro=%.1f/%.1f/%.1f dgyro=%.1f/%.1f/%.1f accel=%.3f/%.3f/%.3f dt_us=%.0f\n",
                   i, i * 0.01, any_abs_spike ? 1 : 0, gz_abs_spike ? 1 : 0,
                   any_rate_spike ? 1 : 0, gz_rate_spike ? 1 : 0, accel_spike ? 1 : 0,
                   data.gyro_x, data.gyro_y, data.gyro_z,
                   has_last ? data.gyro_x - last_gx : 0.0f,
                   has_last ? data.gyro_y - last_gy : 0.0f,
                   has_last ? data.gyro_z - last_gz : 0.0f,
                   data.accel_x, data.accel_y, data.accel_z,
                   dt_us_stat.n ? dt_us_stat.max_v : 0.0);
            detail_print_cnt++;
        }
        last_gx = data.gyro_x;
        last_gy = data.gyro_y;
        last_gz = data.gyro_z;
        has_last = true;

        if (i % 20 == 0) {
            printf("[T10_FRAME] t=%.2f gyro=%.3f/%.3f/%.3f accel=%.3f/%.3f/%.3f temp=%.2f | gz_abs=%.3f dgz=%s\n",
                   i * 0.01,
                   data.gyro_x, data.gyro_y, data.gyro_z,
                   data.accel_x, data.accel_y, data.accel_z,
                   data.temp,
                   std::fabs(data.gyro_z),
                   has_last ? "tracked" : "none");
        }
        usleep(sample_period_us);
    }

    int ok_samples = gz.n;
    double fail_rate = total ? (double)read_fail_cnt / total : 0.0;
    double spike_rate = ok_samples ? (double)(abs_spike_cnt + rate_spike_cnt) / ok_samples : 0.0;
    double any_spike_rate = ok_samples ? (double)(any_abs_spike_cnt + any_rate_spike_cnt) / ok_samples : 0.0;
    double moderate_spike_rate = ok_samples ? (double)moderate_spike_cnt / ok_samples : 0.0;
    double out15_rate = ok_samples ? (double)deadband15_out_cnt / ok_samples : 0.0;
    double out30_rate = ok_samples ? (double)deadband30_out_cnt / ok_samples : 0.0;

    double suggested_deadband = 1.5;
    if (gz.mean_abs() > 2.2 || gz.stddev() > 1.2 || out15_rate > 0.30) suggested_deadband = 3.0;
    else if (gz.mean_abs() > 1.3 || gz.stddev() > 0.8 || out15_rate > 0.12) suggested_deadband = 2.0;

    printf("[T10_SUMMARY] samples=%d read_fail=%d fail_rate=%.3f dt_us_mean=%.0f dt_us_max=%.0f | gyro_x mean=%.3f abs=%.3f rms=%.3f std=%.3f min=%.3f max=%.3f | gyro_y mean=%.3f abs=%.3f rms=%.3f std=%.3f min=%.3f max=%.3f | gyro_z mean=%.3f abs=%.3f rms=%.3f std=%.3f min=%.3f max=%.3f span=%.3f | accel_mean=%.3f/%.3f/%.3f temp=%.2f\n",
           ok_samples, read_fail_cnt, fail_rate,
           dt_us_stat.mean(), dt_us_stat.n ? dt_us_stat.max_v : 0,
           gx.mean(), gx.mean_abs(), gx.rms(), gx.stddev(), gx.n ? gx.min_v : 0, gx.n ? gx.max_v : 0,
           gy.mean(), gy.mean_abs(), gy.rms(), gy.stddev(), gy.n ? gy.min_v : 0, gy.n ? gy.max_v : 0,
           gz.mean(), gz.mean_abs(), gz.rms(), gz.stddev(), gz.n ? gz.min_v : 0, gz.n ? gz.max_v : 0, gz.span(),
           ax.mean(), ay.mean(), az.mean(), temp.mean());
    printf("[T10_SPIKE] gz_abs>|%.0f|=%d gz_rate>|200dps/frame|=%d any_abs=%d any_rate=%d accel_spike=%d moderate_z>|20dps|=%d gz_spike_rate=%.3f any_spike_rate=%.3f moderate_rate=%.3f out_deadband1.5=%d(%.3f) out_deadband3.0=%d(%.3f) suggested_kGyroDeadBandDps=%.1f current_kGyroDeadBandDps=%.1f current_kGyroLpfAlpha=%.2f\n",
           car_config::kGyroRawSpikeRejectDps,
           abs_spike_cnt, rate_spike_cnt, any_abs_spike_cnt, any_rate_spike_cnt, accel_spike_cnt, moderate_spike_cnt,
           spike_rate, any_spike_rate, moderate_spike_rate,
           deadband15_out_cnt, out15_rate,
           deadband30_out_cnt, out30_rate,
           suggested_deadband, car_config::kGyroDeadBandDps, car_config::kGyroLpfAlpha);
    printf("[T10_DIAG_HINT] 当前分支使用read_processed_data采样；若gyro突跳且dt_us_max很大，先查调度阻塞/I2C；若accel也突跳，优先查供电/接地。\n");
    printf("[调参意义] 静止gyro_z应主要落在±1.5dps内；若abs/std大则死区可能要加；若spike>0，多半是I2C尖峰/线材/供电问题，不能靠增大PID解决。\n");
    printf("[RETURN] 0=静止噪声正常；1=噪声偏大；2=明显尖峰/I2C干扰；3=读取失败。\n");

    if (ok_samples < total * 0.80 || fail_rate > 0.20) return 3;
    if (abs_spike_cnt > 0 || rate_spike_cnt > 0 || any_abs_spike_cnt > 0 || any_rate_spike_cnt > 0 || moderate_spike_rate > 0.02) return 2;
    if (gz.mean_abs() > 2.0 || gz.stddev() > 1.2 || out15_rate > 0.30) return 1;
    return 0;
}

static int run_flow_step(const char *label,
                         const std::function<int()> &fn,
                         bool stop_on_fail = true) {
    printf("\n[FLOW] >>> %s\n", label);
    int ret = fn();
    printf("[FLOW] <<< %s ret=%d %s\n", label, ret, ret == 0 ? "PASS" : "CHECK");
    if (ret != 0 && stop_on_fail) {
        printf("[FLOW] STOP at %s. 先处理该步骤日志里的问题，再继续后续测试。\n", label);
    }
    return ret;
}

int run_visual_pipeline(void) {
    printf("\n========== FLOW A visual_pipeline 纯图像调参流程 ==========" "\n");
    printf("[安全] 不输出电机。按顺序检查配置、摄像头居中/俯仰、OTSU二值质量、ROI/扫线/直道判断。\n");
    int ret = 0;
    ret = run_flow_step("T00 precheck_config", [](){ return test_00_precheck_config(); });
    if (ret != 0) return ret;
    ret = run_flow_step("T02 camera_center_angle_static", [](){ return test_02_camera_center_angle_static(); });
    if (ret != 0) return 20 + ret;
    ret = run_flow_step("T03 exposure_otsu_quality", [](){ return test_03_exposure_otsu_quality(); });
    if (ret != 0) return 30 + ret;
    ret = run_flow_step("T04 roi_scan_straight", [](){ return test_04_roi_scan_straight(); });
    if (ret != 0) return 40 + ret;
    printf("[FLOW_SUMMARY] visual_pipeline PASS。可以进入架空硬件或低速闭环测试。\n");
    return 0;
}

int run_airborne_pipeline(void) {
    printf("\n========== FLOW B airborne_pipeline 架空硬件调参流程 ==========" "\n");
    printf("[安全] 必须架空小车。会短时输出电机PWM，用于确认电机/编码器方向和速度环基础响应。\n");
    int ret = 0;
    ret = run_flow_step("T00 precheck_config", [](){ return test_00_precheck_config(); });
    if (ret != 0) return ret;
    ret = run_flow_step("T01 motor_encoder_direction", [](){ return test_01_motor_encoder_direction(); });
    if (ret != 0) return 10 + std::abs(ret);
    ret = run_flow_step("T05 speed_loop_airborne", [](){ return test_05_speed_loop_airborne(); });
    if (ret != 0) return 50 + ret;
    printf("[FLOW_SUMMARY] airborne_pipeline PASS。可以把车放回赛道做低速闭环。\n");
    return 0;
}

int run_sensor_pipeline(void) {
    printf("\n========== FLOW C sensor_pipeline 传感器静态检查 ==========" "\n");
    printf("[安全] 不跑车。用于检查摄像头图像和MPU6050静止噪声。\n");
    int ret = 0;
    ret = run_flow_step("T03 exposure_otsu_quality", [](){ return test_03_exposure_otsu_quality(); });
    if (ret != 0) return 30 + ret;
    release_camera_for_imu_test();
    ret = run_flow_step("T10 gyro_static_noise", [](){ return test_10_gyro_static_noise(); });
    if (ret != 0) return 100 + ret;
    printf("[FLOW_SUMMARY] sensor_pipeline PASS。摄像头二值和IMU静态噪声可用。\n");
    return 0;
}

int run_low_speed_pipeline(void) {
    printf("\n========== FLOW D low_speed_pipeline 低速上赛道流程 ==========" "\n");
    printf("[安全] 会让小车低速循迹。建议先跑 visual_pipeline 和 airborne_pipeline，通过后再执行。\n");
    int ret = 0;
    ret = run_flow_step("T02 camera_center_angle_static", [](){ return test_02_camera_center_angle_static(); });
    if (ret != 0) return 20 + ret;
    ret = run_flow_step("T03 exposure_otsu_quality", [](){ return test_03_exposure_otsu_quality(); });
    if (ret != 0) return 30 + ret;
    ret = run_flow_step("T04 roi_scan_straight", [](){ return test_04_roi_scan_straight(); });
    if (ret != 0) return 40 + ret;
    release_camera_for_imu_test();
    ret = run_flow_step("T10 gyro_static_noise", [](){ return test_10_gyro_static_noise(); });
    if (ret != 0) return 100 + ret;
    ret = run_flow_step("T06 low_speed_visual_gyro_closed_loop", [](){ return test_06_low_speed_visual_gyro_closed_loop(); });
    if (ret != 0) return 60 + ret;
    printf("[FLOW_SUMMARY] low_speed_pipeline PASS。可以继续做速度提升或元素调试。\n");
    return 0;
}

int run_element_pipeline(void) {
    printf("\n========== FLOW E element_pipeline 元素/特殊赛道调试流程 ==========" "\n");
    printf("[安全] 会让小车低速循迹并开启元素处理。先短距离经过普通直道确认图像，再依次低速通过十字/圆环/车库等元素。\n");
    int ret = 0;
    ret = run_flow_step("T02 camera_center_angle_static", [](){ return test_02_camera_center_angle_static(); });
    if (ret != 0) return 20 + ret;
    ret = run_flow_step("T03 exposure_otsu_quality", [](){ return test_03_exposure_otsu_quality(); });
    if (ret != 0) return 30 + ret;
    ret = run_flow_step("T04 roi_scan_straight", [](){ return test_04_roi_scan_straight(); });
    if (ret != 0) return 40 + ret;
    ret = run_flow_step("T08 element_debug", [](){ return test_08_element_debug(); });
    if (ret != 0) return 80 + ret;
    printf("[FLOW_SUMMARY] element_pipeline PASS。元素状态稳定，可以按具体元素继续细调或再做速度回归。\n");
    return 0;
}

} // namespace param_step_demo

// =============================================================================
// 对外函数：你只需要在 main.cpp 里选择性调用这些函数即可。
// 建议顺序：00 -> 01(架空) -> 02 -> 03 -> 04 -> 05(架空) -> 06 -> 07 -> 08 -> 09
// =============================================================================

int lq_param_test_00_precheck_config(void) { return param_step_demo::run_with_log("T00_precheck_config", [](){ return param_step_demo::test_00_precheck_config(); }); }
int lq_param_test_01_motor_encoder_direction(void) { return param_step_demo::run_with_log("T01_motor_encoder_direction", [](){ return param_step_demo::test_01_motor_encoder_direction(); }); }
int lq_param_test_02_camera_center_angle_static(void) { return param_step_demo::run_with_log("T02_camera_center_angle_static", [](){ return param_step_demo::test_02_camera_center_angle_static(); }); }
int lq_param_test_03_exposure_otsu_quality(void) { return param_step_demo::run_with_log("T03_exposure_otsu_quality", [](){ return param_step_demo::test_03_exposure_otsu_quality(); }); }
int lq_param_test_03_exposure_hsv_quality(void) { return lq_param_test_03_exposure_otsu_quality(); }
int lq_param_test_04_roi_scan_straight(void) { return param_step_demo::run_with_log("T04_roi_scan_straight", [](){ return param_step_demo::test_04_roi_scan_straight(); }); }
int lq_param_test_05_speed_loop_airborne(void) { return param_step_demo::run_with_log("T05_speed_loop_airborne", [](){ return param_step_demo::test_05_speed_loop_airborne(); }); }
int lq_param_test_06_low_speed_visual_gyro_closed_loop(void) { return param_step_demo::run_with_log("T06_low_speed_visual_gyro", [](){ return param_step_demo::test_06_low_speed_visual_gyro_closed_loop(); }); }
int lq_param_test_07_speed_up_regression(void) { return param_step_demo::run_with_log("T07_speed_up_regression", [](){ return param_step_demo::test_07_speed_up_regression(); }); }
int lq_param_test_08_element_debug(void) { return param_step_demo::run_with_log("T08_element_debug", [](){ return param_step_demo::test_08_element_debug(); }); }
int lq_param_test_09_voltage_light_regression(void) { return param_step_demo::run_with_log("T09_voltage_light_regression", [](){ return param_step_demo::test_09_voltage_light_regression(); }); }
int lq_param_test_10_gyro_static_noise(void) { return param_step_demo::run_with_log("T10_gyro_static_noise", [](){ return param_step_demo::test_10_gyro_static_noise(); }); }
int lq_param_flow_visual(void) { return param_step_demo::run_with_log("FLOW_A_visual_pipeline", [](){ return param_step_demo::run_visual_pipeline(); }); }
int lq_param_flow_airborne(void) { return param_step_demo::run_with_log("FLOW_B_airborne_pipeline", [](){ return param_step_demo::run_airborne_pipeline(); }); }
int lq_param_flow_sensor(void) { return param_step_demo::run_with_log("FLOW_C_sensor_pipeline", [](){ return param_step_demo::run_sensor_pipeline(); }); }
int lq_param_flow_low_speed(void) { return param_step_demo::run_with_log("FLOW_D_low_speed_pipeline", [](){ return param_step_demo::run_low_speed_pipeline(); }); }
int lq_param_flow_element(void) { return param_step_demo::run_with_log("FLOW_E_element_pipeline", [](){ return param_step_demo::run_element_pipeline(); }); }

void lq_parameter_step_tuning_demo(void) {
    printf("\n========== DEMO 33 parameter_step_tuning_demo ==========" "\n");
    printf("这是一个分阶段调参向导。建议先跑流程函数，再按失败步骤单独复测。\n\n");
    printf("[推荐流程]\n");
    printf("  lq_param_flow_visual();        // 不跑电机：配置、摄像头、OTSU、ROI/扫线\n");
    printf("  lq_param_flow_airborne();      // 必须架空：电机/编码器方向、速度环基础\n");
    printf("  lq_param_flow_sensor();        // 不跑车：OTSU质量 + MPU6050静止噪声\n");
    printf("  lq_param_flow_low_speed();     // 上赛道低速：视觉 + 陀螺仪 + 轮速闭环\n\n");
    printf("  lq_param_flow_element();       // 上赛道低速：开启元素/特殊赛道判断与处理\n\n");
    printf("[单项测试]\n");
    printf("  lq_param_test_00_precheck_config();\n");
    printf("  lq_param_test_01_motor_encoder_direction();      // 架空\n");
    printf("  lq_param_test_02_camera_center_angle_static();\n");
    printf("  lq_param_test_03_exposure_otsu_quality();\n");
    printf("  lq_param_test_04_roi_scan_straight();\n");
    printf("  lq_param_test_05_speed_loop_airborne();          // 架空\n");
    printf("  lq_param_test_06_low_speed_visual_gyro_closed_loop();\n");
    printf("  lq_param_test_07_speed_up_regression();\n");
    printf("  lq_param_test_08_element_debug();\n");
    printf("  lq_param_test_09_voltage_light_regression();\n");
    printf("  lq_param_test_10_gyro_static_noise();             // 无需赛道，静止IMU噪声\n");
    printf("\n[返回码]\n");
    printf("  单项测试返回值见各自日志；流程返回 20/30/40/50/60/100 + 子错误码，表示停在哪个阶段。\n");
    printf("  图像不过不要调PID；架空不过不要上赛道；IMU静态不过不要调角速度环。\n\n");
    printf("示例：int ret = lq_param_flow_visual(); printf(\"ret=%d\\n\", ret); return 0;\n");
    param_step_demo::print_config_snapshot();
}
