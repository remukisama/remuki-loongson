#include "main.hpp"
#include "car_config.hpp"
#include "my_cascade_pid.hpp"
#include "lq_display_tft18.hpp"
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
#include <sys/select.h>
#include <sstream>

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

enum ElementScene_e {
    ElementScene_Normal = 0,
    ElementScene_LeftRing,
    ElementScene_RightRing,
    ElementScene_Zebra,
    ElementScene_Garage,
};

static inline ElementScene_e current_element_scene_from_flags() {
    if (ImageFlag.image_element_rings == 1) return ElementScene_LeftRing;
    if (ImageFlag.image_element_rings == 2) return ElementScene_RightRing;
    if (ImageFlag.Zebra_Flag != 0) return ElementScene_Zebra;
    return ElementScene_Normal;
}

#define g_element_scene current_element_scene_from_flags()
#define g_element_cooldown_frames 0

extern int img1[60][80];
extern int imgdisplay[60][80];
extern int Left_RingsFlag_Point1_Ysite;
extern int Left_RingsFlag_Point2_Ysite;
extern int Right_RingsFlag_Point1_Ysite;
extern int Right_RingsFlag_Point2_Ysite;

extern float Speed_Goal;
extern float Speed_Goal_l;
extern float Speed_Goal_r;
extern float speed_encoder_L;
extern float speed_encoder_R;
extern float speed_out_l;
extern float speed_out_r;
extern float speed_P_l, speed_P_r, speed_I_l, speed_I_r;  // T05实地速度测试可热改
extern float speed_last_erro_L;
extern float speed_last_erro_R;
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
int dbg_el_detect_open = 0;
int dbg_el_l_block = 0, dbg_el_l_geom = 0, dbg_el_l_loss = 0, dbg_el_l_susp = 0;
int dbg_el_l_confirm = 0, dbg_el_l_p1 = 0, dbg_el_l_p2 = 0, dbg_el_l_help = 0;
int dbg_el_r_block = 0, dbg_el_r_geom = 0, dbg_el_r_loss = 0, dbg_el_r_susp = 0;
int dbg_el_r_confirm = 0, dbg_el_r_p1 = 0, dbg_el_r_p2 = 0, dbg_el_r_help = 0;
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
extern void Data_Settings(void);
extern void imageprocess(void);
extern void motor_Init(void);
extern void Speed_decision(void);
extern void Speed_BiHuan(void);
extern void Speed_PID_L(void);
extern void Speed_PID_R(void);
extern float encoder_num_L(void);
extern float encoder_num_R(void);
extern void motor_Duty_L(int duty);
extern void motor_Duty_R(int duty);

bool image_cv_IsReady(void) {
    return false;
}

void image_cv_Deinit(void) {
}

namespace param_step_demo {

static const std::string kElementDebugTargetIp = "192.168.5.22";
static const uint16_t kElementDebugTargetPort = 8080;
static const uint8_t kElementDebugJpegQuality = 55;
static constexpr bool kElementDebugUdpEnable = false;
static constexpr bool kElementDebugTftEnable = true;
static constexpr int kElementDebugTftX = 1;
static constexpr int kElementDebugTftY = 6;
static FILE *g_log_file = nullptr;

static std::string make_artifact_path(const char *test_name, const char *ext) {
    mkdir("logs", 0755);
    time_t now = time(nullptr);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char buf[256];
    snprintf(buf, sizeof(buf), "logs/%04d%02d%02d_%02d%02d%02d_%s.%s",
             tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
             tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec, test_name, ext);
    return std::string(buf);
}

static std::string make_log_path(const char *test_name) {
    return make_artifact_path(test_name, "log");
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

static const char *road_type_name(int t);
static const char *element_scene_name(ElementScene_e scene);

struct ElementCsvLogger {
    FILE *file = nullptr;
    std::string path;

    bool open(const char *test_name) {
        path = make_artifact_path(test_name, "csv");
        file = fopen(path.c_str(), "w");
        if (!file) {
            printf("[CSV_ERROR] cannot open %s\n", path.c_str());
            return false;
        }
        fprintf(file,
                "t_sec,sample,motor_on,road,scene,ring_side,ring_flag,cool,"
                "det,mid,err,tow,off,left_line,right_line,straight,straight_flag,"
                "el_open,"
                "l_block,l_geom,l_loss,l_susp,l_confirm,l_p1,l_p2,l_help,"
                "r_block,r_geom,r_loss,r_susp,r_confirm,r_p1,r_p2,r_help,"
                "lp1,lp2,rp1,rp2,img_bad,img_warn\n");
        fflush(file);
        printf("[CSV] 元素特征样本保存到: %s\n", path.c_str());
        return true;
    }

    void write(double t_sec, int sample, bool motor_on, int img_bad, int img_warn) {
        if (!file) return;
        fprintf(file,
                "%.3f,%d,%d,%s,%s,%d,%d,%u,"
                "%d,%d,%d,%d,%d,%d,%d,%d,%d,"
                "%d,"
                "%d,%d,%d,%d,%d,%d,%d,%d,"
                "%d,%d,%d,%d,%d,%d,%d,%d,"
                "%d,%d,%d,%d,%d,%d\n",
                t_sec, sample, motor_on ? 1 : 0,
                road_type_name(ImageStatus.Road_type),
                element_scene_name(g_element_scene),
                ImageFlag.image_element_rings,
                ImageFlag.image_element_rings_flag,
                g_element_cooldown_frames,
                ImageStatus.Det_True,
                ImageStatus.MiddleLine,
                ImageStatus.MiddleLine - ImageStatus.Det_True,
                ImageStatus.TowPoint,
                ImageStatus.OFFLine,
                ImageStatus.Left_Line,
                ImageStatus.Right_Line,
                ImageStatus.straight_acc,
                ImageStatus.straight_acc_flag,
                dbg_el_detect_open,
                dbg_el_l_block,
                dbg_el_l_geom,
                dbg_el_l_loss,
                dbg_el_l_susp,
                dbg_el_l_confirm,
                dbg_el_l_p1,
                dbg_el_l_p2,
                dbg_el_l_help,
                dbg_el_r_block,
                dbg_el_r_geom,
                dbg_el_r_loss,
                dbg_el_r_susp,
                dbg_el_r_confirm,
                dbg_el_r_p1,
                dbg_el_r_p2,
                dbg_el_r_help,
                Left_RingsFlag_Point1_Ysite,
                Left_RingsFlag_Point2_Ysite,
                Right_RingsFlag_Point1_Ysite,
                Right_RingsFlag_Point2_Ysite,
                img_bad,
                img_warn);
    }

    void close() {
        if (!file) return;
        fflush(file);
        fclose(file);
        file = nullptr;
    }
};

struct ElementFeatureStats {
    int samples = 0;
    int left_scene = 0;
    int right_scene = 0;
    int left_ring_active = 0;
    int right_ring_active = 0;
    int left_candidate = 0;
    int right_candidate = 0;
    int both_candidate = 0;
    int left_block = 0;
    int left_susp = 0;
    int left_confirm_seen = 0;
    int right_block = 0;
    int right_susp = 0;
    int right_confirm_seen = 0;
    RunningStat off;
    RunningStat err;
    RunningStat left_line;
    RunningStat right_line;
    RunningStat left_p1;
    RunningStat left_p2;
    RunningStat right_p1;
    RunningStat right_p2;
    RunningStat left_cand_left_line;
    RunningStat left_cand_right_line;
    RunningStat left_cand_off;
    RunningStat left_cand_p2;
    RunningStat right_cand_left_line;
    RunningStat right_cand_right_line;
    RunningStat right_cand_off;
    RunningStat right_cand_p2;

    void add() {
        samples++;
        if (g_element_scene == ElementScene_LeftRing) left_scene++;
        if (g_element_scene == ElementScene_RightRing) right_scene++;
        if (ImageFlag.image_element_rings == 1) left_ring_active++;
        if (ImageFlag.image_element_rings == 2) right_ring_active++;
        if (dbg_el_l_block) left_block++;
        if (dbg_el_l_susp) left_susp++;
        if (dbg_el_l_confirm > 0) left_confirm_seen++;
        if (dbg_el_r_block) right_block++;
        if (dbg_el_r_susp) right_susp++;
        if (dbg_el_r_confirm > 0) right_confirm_seen++;

        off.add(ImageStatus.OFFLine);
        err.add(ImageStatus.MiddleLine - ImageStatus.Det_True);
        left_line.add(ImageStatus.Left_Line);
        right_line.add(ImageStatus.Right_Line);
        if (Left_RingsFlag_Point1_Ysite > 0) left_p1.add(Left_RingsFlag_Point1_Ysite);
        if (Left_RingsFlag_Point2_Ysite > 0) left_p2.add(Left_RingsFlag_Point2_Ysite);
        if (Right_RingsFlag_Point1_Ysite > 0) right_p1.add(Right_RingsFlag_Point1_Ysite);
        if (Right_RingsFlag_Point2_Ysite > 0) right_p2.add(Right_RingsFlag_Point2_Ysite);

        bool l_cand = (dbg_el_l_geom || dbg_el_l_loss);
        bool r_cand = (dbg_el_r_geom || dbg_el_r_loss);
        if (l_cand) {
            left_candidate++;
            left_cand_left_line.add(ImageStatus.Left_Line);
            left_cand_right_line.add(ImageStatus.Right_Line);
            left_cand_off.add(ImageStatus.OFFLine);
            if (Left_RingsFlag_Point2_Ysite > 0) left_cand_p2.add(Left_RingsFlag_Point2_Ysite);
        }
        if (r_cand) {
            right_candidate++;
            right_cand_left_line.add(ImageStatus.Left_Line);
            right_cand_right_line.add(ImageStatus.Right_Line);
            right_cand_off.add(ImageStatus.OFFLine);
            if (Right_RingsFlag_Point2_Ysite > 0) right_cand_p2.add(Right_RingsFlag_Point2_Ysite);
        }
        if (l_cand && r_cand) both_candidate++;
    }

    void print_recommendation(const char *tag,
                              const std::string &csv_path,
                              double special_rate,
                              double img_bad_rate,
                              double img_warn_rate) const {
        printf("[%s_AUTO_ANALYZE] csv=%s samples=%d special_rate=%.2f img_bad_rate=%.2f img_warn_rate=%.2f leftCand=%d rightCand=%d leftScene=%d rightScene=%d bothCand=%d\n",
               tag, csv_path.c_str(), samples, special_rate, img_bad_rate, img_warn_rate,
               left_candidate, right_candidate, left_scene, right_scene, both_candidate);
        printf("[%s_FEATURE_RANGE] all OFF mean/max=%.1f/%.1f Err mean/span=%.1f/%.1f LLine mean/max=%.1f/%.1f RLine mean/max=%.1f/%.1f\n",
               tag,
               off.mean(), off.n ? off.max_v : 0,
               err.mean(), err.span(),
               left_line.mean(), left_line.n ? left_line.max_v : 0,
               right_line.mean(), right_line.n ? right_line.max_v : 0);
        if (left_candidate > 0) {
            printf("[%s_LEFT_CAND] count=%d LLine mean/min/max=%.1f/%.1f/%.1f RLine mean/max=%.1f/%.1f OFF mean/max=%.1f/%.1f P2 mean/min/max=%.1f/%.1f/%.1f\n",
                   tag, left_candidate,
                   left_cand_left_line.mean(), left_cand_left_line.n ? left_cand_left_line.min_v : 0, left_cand_left_line.n ? left_cand_left_line.max_v : 0,
                   left_cand_right_line.mean(), left_cand_right_line.n ? left_cand_right_line.max_v : 0,
                   left_cand_off.mean(), left_cand_off.n ? left_cand_off.max_v : 0,
                   left_cand_p2.mean(), left_cand_p2.n ? left_cand_p2.min_v : 0, left_cand_p2.n ? left_cand_p2.max_v : 0);
        }
        if (right_candidate > 0) {
            printf("[%s_RIGHT_CAND] count=%d RLine mean/min/max=%.1f/%.1f/%.1f LLine mean/max=%.1f/%.1f OFF mean/max=%.1f/%.1f P2 mean/min/max=%.1f/%.1f/%.1f\n",
                   tag, right_candidate,
                   right_cand_right_line.mean(), right_cand_right_line.n ? right_cand_right_line.min_v : 0, right_cand_right_line.n ? right_cand_right_line.max_v : 0,
                   right_cand_left_line.mean(), right_cand_left_line.n ? right_cand_left_line.max_v : 0,
                   right_cand_off.mean(), right_cand_off.n ? right_cand_off.max_v : 0,
                   right_cand_p2.mean(), right_cand_p2.n ? right_cand_p2.min_v : 0, right_cand_p2.n ? right_cand_p2.max_v : 0);
        }

        printf("[%s_RECOMMEND] ", tag);
        if (samples < 20) {
            printf("样本太少：每个元素至少手推5-10次，或把seconds调到60以上。\n");
        } else if (img_bad_rate > 0.25) {
            printf("先回到T03/T04修图像质量/ROI/前瞻：当前img_bad过高，直接调元素阈值会误导。\n");
        } else if (special_rate < 0.05 && left_candidate == 0 && right_candidate == 0) {
            printf("本轮几乎没采到元素候选：确认手推确实经过元素；若TFT补线已明显进入环岛但候选仍为0，再放宽对应minLost/P2/OFF条件。\n");
        } else if (both_candidate > samples / 8) {
            printf("左右候选同时出现偏多：优先提高kRingLossOnlyMinLost或增加kElementRingConfirmFrames，减少左右误抢。\n");
        } else if (left_candidate > 0 && left_scene == 0 && left_ring_active == 0) {
            printf("左环候选出现但未锁定LeftRing：若left_block高，放宽kLeftRingRightLineBlockMax/kLeftRingOfflineBlockMax；若left_susp高，检查普通弯误判并提高confirm。\n");
        } else if (right_candidate > 0 && right_scene == 0 && right_ring_active == 0) {
            printf("右环候选出现但未锁定RightRing：若right_block高，放宽kRightRingLeftLineBlockMax/kRightRingOfflineBlockMax；若right_susp高，提高confirm或收紧loss-only。\n");
        } else if ((left_scene + right_scene) > 0 && img_warn_rate < 0.30) {
            printf("元素识别链路已能进入场景锁：下一步用TFT看ring=5/6/7/8中心线方向，再进入低速自动T08。\n");
        } else {
            printf("当前样本可用但结论不强：保留CSV，按TFT画面把失败窗口前后1-2秒日志发回来继续细分。\n");
        }
    }
};

static int round_to_int(double v) {
    return (int)(v >= 0.0 ? v + 0.5 : v - 0.5);
}

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
    switch (car_config::kImageAlgorithm) {
        case 0: return "HSV_BLUE_INVERT";
        case 1: return "GRAY_STATIC";
        case 2: return "GRAY_OTSU";
        default: return "UNKNOWN";
    }
}

static const char *element_scene_name(ElementScene_e scene) {
    switch (scene) {
        case ElementScene_Normal: return "Normal";
        case ElementScene_LeftRing: return "LeftRing";
        case ElementScene_RightRing: return "RightRing";
        case ElementScene_Zebra: return "Zebra";
        case ElementScene_Garage: return "Garage";
        default: return "Unknown";
    }
}

static const char *ring_state_name(int flag) {
    switch (flag) {
        case 0: return "Idle";
        case 1: return "Suspect";
        case 2: return "Entry";
        case 3: return "Check";
        case 4: return "Wait";
        case 5: return "TurnIn";
        case 6: return "Inside";
        case 7: return "SearchExit";
        case 8: return "ForceExit";
        case 9: return "ExitWait";
        case 10: return "Finish";
        default: return "Unknown";
    }
}

static std::string element_debug_state_name() {
    if (g_element_cooldown_frames > 0) return "Cooldown";
    if (g_element_scene == ElementScene_LeftRing) {
        return std::string("LeftRing.") + ring_state_name(ImageFlag.image_element_rings_flag);
    }
    if (g_element_scene == ElementScene_RightRing) {
        return std::string("RightRing.") + ring_state_name(ImageFlag.image_element_rings_flag);
    }
    if (ImageFlag.image_element_rings == 1) {
        return std::string("LFlag.") + ring_state_name(ImageFlag.image_element_rings_flag);
    }
    if (ImageFlag.image_element_rings == 2) {
        return std::string("RFlag.") + ring_state_name(ImageFlag.image_element_rings_flag);
    }
    return element_scene_name(g_element_scene);
}

static void print_config_snapshot() {
    printf("\n[CFG] camera: %dx%d fps=%d roi=(%d,%d,%d,%d) exposure=%d algo=%s HSV=[H%d-%d S%d-%d V%d-%d]\n",
           car_config::kCameraWidth, car_config::kCameraHeight, car_config::kCameraFps,
           car_config::kCameraRoiX, car_config::kCameraRoiY, car_config::kCameraRoiWidth, car_config::kCameraRoiHeight,
           car_config::kCameraManualExposure, vision_algorithm_name(),
           car_config::kBlueHueLower, car_config::kBlueHueUpper,
           car_config::kBlueSaturationLower, car_config::kBlueSaturationUpper,
           car_config::kBlueValueLower, car_config::kBlueValueUpper);
    printf("[CFG] vision_binary: algorithm=%d(0=HSV 1=GRAY_STATIC 2=OTSU) otsu_blur=%d morph=%d\n",
           car_config::kImageAlgorithm,
           car_config::kImageOtsuBlurKernel,
           car_config::kImageMorphKernel);
    printf("[CFG] image: middle=%d tow=%d scan=%d cross_scan=%d static_th=%d straight(var<%d off<=%d L<%d R<%d)\n",
           car_config::kImageMiddleLine, car_config::kImageTowPoint,
           car_config::kImageScanInterval, car_config::kImageScanIntervalCross,
           car_config::kImageThresholdStatic,
           car_config::kStraightVarianceAccThreshold, car_config::kStraightOfflineMax,
           car_config::kStraightLeftLineMax, car_config::kStraightRightLineMax);
    printf("[CFG] element: ring_confirm=%d exit_confirm=%d cooldown=%d exit_hold=%d left(Lmin=%d Rmax=%d OFFmax=%d P2<%d) right(Rmin=%d Lmax=%d OFFmax=%d P2>%d) loss_only>=%d ring_cap=%.0f enable_runtime=%d scene=%s cooldown_now=%u\n",
           car_config::kElementRingConfirmFrames,
           car_config::kElementRingExitConfirmFrames,
           car_config::kElementCooldownFrames,
           car_config::kElementRingExitStraightHoldFrames,
           car_config::kLeftRingMinLeftLost,
           car_config::kLeftRingRightLineBlockMax,
           car_config::kLeftRingOfflineBlockMax,
           car_config::kLeftRingMaxPoint2Y,
           car_config::kRightRingMinRightLost,
           car_config::kRightRingLeftLineBlockMax,
           car_config::kRightRingOfflineBlockMax,
           car_config::kRightRingMinPoint2Y,
           car_config::kRingLossOnlyMinLost,
           car_config::kRingSceneGyroLimitDps,
           g_enable_element_process,
           element_scene_name(g_element_scene),
           g_element_cooldown_frames);
    printf("[CFG] control: speed(P/I/D=%.1f/%.1f/%.1f curve=%d straight=%d maxDiff=%.2f minWheel=%d) steer(Kp=%.2f Kd=%.2f alpha=%.2f) gyro(P=%.3f D=%.4f alpha=%.2f ff=%.3f)\n\n",
           car_config::kSpeedPidP, car_config::kSpeedPidI, car_config::kSpeedPidD,
           car_config::kCurveSpeedGoal, car_config::kStraightSpeedGoal,
           car_config::kMaxDiffRatio, car_config::kMinWheelSpeedGoal,
           car_config::kSteerKp, car_config::kSteerKd, car_config::kSteerFilterAlpha,
           car_config::kGyroPidP, car_config::kGyroPidD, car_config::kGyroLpfAlpha,
           car_config::kGyroFeedforwardGain);
}

static void print_stage_task_map() {
    printf("[TASK_MAP] 本Demo按阶段锁参数：先图像质量，再硬件方向，再速度环，再IMU，再低速闭环，最后元素/提速。\n");
    printf("[TASK_MAP] T02只判断摄像头/图像重复性，不要求手动把车摆到像素级中线；单次摆放的err_mean只能作为参考。\n");
    printf("[TASK_MAP] 中线参数只在同一段直道重复摆放3次后再改：若3次det_mean同向偏离且差异小，再考虑kImageMiddleLine每次改1。\n");
    printf("[TASK_MAP] T03只改曝光/二值化/ROI质量；T04只改前瞻/扫线/直道判断；T05只改速度环；T10只改IMU死区/滤波；T11只隔离角速度环；T06以后才联调视觉外环。\n");
}

static void print_stage_card(const char *stage,
                             const char *task,
                             const char *tune_only,
                             const char *do_not_tune,
                             const char *pass_to_next) {
    printf("[STAGE_CARD] %s\n", stage);
    printf("[STAGE_TASK] %s\n", task);
    printf("[STAGE_TUNE_ONLY] %s\n", tune_only);
    printf("[STAGE_DO_NOT_TUNE] %s\n", do_not_tune);
    printf("[STAGE_PASS_TO_NEXT] %s\n", pass_to_next);
}

static void print_param_plan_line(const char *stage,
                                  const char *params,
                                  const char *watch,
                                  const char *adjust,
                                  const char *guard) {
    printf("[PARAM_PLAN] stage=%s params=%s\n", stage, params);
    printf("[PARAM_WATCH] %s\n", watch);
    printf("[PARAM_ADJUST] %s\n", adjust);
    printf("[PARAM_GUARD] %s\n", guard);
}

static void print_parameter_debug_plan() {
    printf("\n========== PARAMETER DEBUG PLAN 参数级调试地图 ==========" "\n");
    printf("[PLAN_RULE] 每次只改同一阶段的一小组参数；改完必须回到对应测试复测，不要用T07/T08失败反推所有参数一起改。\n");
    printf("[PLAN_RULE] 先判断瓶颈层级：图像(T02/T03/T04) -> 硬件/速度(T01/T05) -> IMU(T10) -> 角速度内环(T11) -> 视觉外环(T06) -> 提速/元素(T07/T08)。\n");

    print_param_plan_line(
        "T01/T05 motor_speed",
        "kMotorPwmFrequencyHz, kMotorInitialDuty, kMotorMaxPwm",
        "T01 firstF/firstB、左右启动死区；T05 out是否长期贴近MAX_PWM、enc是否仍上不去、电机/驱动是否发热。",
        "频率通常保持20k-25kHz；初始占空比保持0；只有T05确认PWM打满且电机不热时才小幅增大kMotorMaxPwm。",
        "不要用更大PID或更高速度目标掩盖电机接线、编码器反号、机械卡滞。");
    print_param_plan_line(
        "T05 speed_loop",
        "kSpeedPidP, kSpeedPidI, kSpeedPidD",
        "T05阶跃的rise/settle/overshoot、encL/encR、speed_err、左右响应比例。",
        "起步慢且不过冲先小幅加P；稳态长期差一点再小幅加I；振荡/抖动先降P或I，D默认保持0，只有明显快速振荡才极小幅尝试。",
        "速度环不过不要跑T11/T06；不要用视觉Kp或角速度P补偿轮速跟不上。");
    print_param_plan_line(
        "T05/T07 speed_goal",
        "kCurveSpeedGoal, kStraightSpeedGoal, kMaxWheelSpeedGoal, kMinWheelSpeedGoal",
        "T06/T07 Speed_Goal、goalL/R、encL/R、SE、直道蛇形、入弯出界、内轮是否停住。",
        "直道稳定后每次只加kStraightSpeedGoal 1-2；入弯推头/出界先降kCurveSpeedGoal或加入口保护；内轮停住再看kMinWheelSpeedGoal；单轮目标被截断才看kMaxWheelSpeedGoal。",
        "普通赛道低速不稳时不要提直道速度；轮速误差大时先回T05。");

    print_param_plan_line(
        "T02/T03 camera_binary",
        "kCameraWidth, kCameraHeight, kCameraFps, kCameraFormat, kCameraDevicePath",
        "摄像头打开失败、FPS adjusted、CPU负载、画面尺寸/压缩后数组是否正常。",
        "通常保持160x120和当前格式；设备号变化只改kCameraDevicePath；帧率按实际摄像头能力配置。",
        "画面质量问题不要先改PID；尺寸/采样步长改动会牵涉80x60处理链路。");
    print_param_plan_line(
        "T02/T03 camera_roi_exposure",
        "kCameraManualExposure, kCameraRoiX/Y/Width/Height, kCameraColumnSampleStep",
        "T02 det_std/det_span/lost_rate/row_bad_rate；T03 white_avg、white_min/max、dDet、lost、rowW。",
        "拖影/过曝先降曝光，暗场漏线先升曝光；远端不够看再减小RoiY或增大RoiHeight；横向偏移优先机械校正，最后才微调ROI。",
        "ROI和曝光没稳前不要碰kImageMiddleLine、速度、PID、元素阈值。");
    print_param_plan_line(
        "T03 binary_threshold",
        "kImageAlgorithm, kImageThresholdStatic, kImageThresholdDetach, kImageOtsuBlurKernel, kImageMorphKernel, kBlueHue/Saturation/Value Lower/Upper",
        "T03 white比例、边界连续性、亮斑误检、阴影漏检；HSV模式看蓝底是否被稳定提取。",
        "OTSU优先保持；静态阈值只在固定光照验证；孤点多可加blur/morph，细线被吞就减；HSV只按背景漏检/误检方向收放H/S/V。",
        "二值不稳时T04/T06/T08的所有控制结论都可能是假信号。");

    print_param_plan_line(
        "T02 center_line",
        "kImageMiddleLine",
        "T02三轮det_mean/err_mean、middle_candidate_ok、candidate_next、round_delta_mean。",
        "只有三轮趋势同向且稳定时才按candidate_next每次改1；单次err偏一边先认为是摆车误差。",
        "不要在手摆无法像素级居中的情况下按单轮err_mean大改中线。");
    print_param_plan_line(
        "T04 scan_foresight",
        "kImageTowPoint, kImageScanInterval, kImageScanIntervalCross, kSingleSideLossThreshold",
        "T04 jump/edge_bad/straight_rate、普通弯是否提前看到、十字/复杂区域是否跳错边。",
        "转弯来不及可前移TowPoint；边界找不回可小幅增scan；十字误连/跳边则收紧cross_scan或提高单侧丢线阈值。",
        "扫线还会跳时不要靠更猛转向解决，先让Det和边界稳定。");
    print_param_plan_line(
        "T04 straight_detect",
        "kImageVariance, kImageVarianceAcc, kStraightVarianceAccThreshold, kStraightOfflineMax, kStraightLeftLineMax, kStraightRightLineMax",
        "T04/T06 Road=Straight/Normol切换、straight_acc、直道加速是否乱跳、弯道是否误判直道。",
        "直道不加速可放宽variance/off/left/right；弯道误判直道导致冲弯就收紧；先用T04手推确认再让T07提速。",
        "直道判定不稳时不要直接提高kStraightSpeedGoal。");

    print_param_plan_line(
        "T10 imu_static",
        "kMpu6050AccelScale, kMpu6050GyroScale, kMpu6050GyroBiasSamples, kMpu6050BiasSampleDelayUs",
        "T10 gyro_z mean/std/span、spike、dt_us、accel均值是否接近物理预期。",
        "量程只在驱动量程变更时同步；零偏波动大可增加BiasSamples；采样间隔保持5-10ms避免相关噪声。",
        "量程/零偏错时T11/T06所有角速度结论都不可信。");
    print_param_plan_line(
        "T10/T11 gyro_filter",
        "kGyroDeadBandDps, kGyroMeasurementLimitDps, kGyroLpfAlpha, kGyroRawSpikeRejectDps",
        "T10静止噪声；T11 real/gf/err；T06 gf是否明显滞后、尖峰是否触发。",
        "静止漂移大先加deadband；GF跟随慢可加LpfAlpha；GF噪声/抖动大则降LpfAlpha；MeasurementLimit必须高于目标角速度上限；尖峰阈值只为拒绝异常值。",
        "不要用更大kGyroPidP掩盖IMU尖峰或滤波过慢。");
    print_param_plan_line(
        "T11 gyro_inner_loop",
        "kGyroPidP, kGyroPidI, kGyroPidD, kGyroFeedforwardGain, kGyroPidIntegralLimit, kGyroPidOutputLimit",
        "T11 late_err_abs、gf_abs、diff0/diff、cap_rate、左右阶跃是否对称；T06 GErr_abs和diff_limited。",
        "无明显限幅但GF追不上：先小幅加feedforward或P；过冲/振荡：加D或降P/ff；长期固定误差才极小I并降低积分限幅；diff0常被自身out限制才加OutputLimit。",
        "T11正常时不要再动角速度P/D，T06问题多半在视觉目标或速度保护。");
    print_param_plan_line(
        "T11 gyro_direction_rate_limit",
        "kDiffOutputMaxDelta, kDiffOutputReverseMaxDelta, kReverseGyroTargetThresholdDps, kReverseGyroRealThresholdDps, kGyroTargetPositiveGain, kGyroTargetNegativeGain",
        "T11正/负阶跃gf_mean、reverse_rate、diff变化速度；T06 S弯反打是否慢半拍。",
        "差速输出爬升慢可小幅加DiffOutputMaxDelta；反向救车慢加ReverseMaxDelta；正负方向一边欠转就微调对应TargetGain。",
        "目标和实际长期反向先查符号/接线；不要靠增益硬怼。");
    print_param_plan_line(
        "angle_yaw_optional",
        "kAnglePidP/I/D, kAnglePidIntegralLimit, kAnglePidOutputLimit, kYawTrackKp, kYawTrackToleranceDeg, kYawTrackSettleGyroDps, kYawIntegralDeadBandDps, kYawTrackMin/FineRate*, kYawTrackDefaultMaxRateDps",
        "主要用于角度目标/定角验证，不是当前视觉循迹主路径；看yaw到位误差、末端抖动、是否差一点不动。",
        "回正慢加YawTrackKp或最低角速度；末端抖动放大容差/settle或降Kp；通常不启用I。",
        "无元素提速转不过时优先看T11/T06，不要先改角度环。");

    print_param_plan_line(
        "T06 visual_outer_loop",
        "kSteerKp, kSteerKd, kSteerDeadBandDefault/Straight/Curve, kSteerFilterAlpha",
        "T06 raw/filt/dE/PD/Tgt、pd_sat_rate、直道蛇形、入弯迟钝、出弯回正。",
        "转向目标偏小且T11健康可加Kp；误差快速变化反应慢可加Kd；直道蛇形先降Kp/Kd或加死区；小偏差不修正则减死区；Filt滞后加Alpha，噪声放大降Alpha。",
        "T11未通过不要用视觉外环补内环；图像不稳不要调Kp/Kd。");
    print_param_plan_line(
        "T06 visual_jump_filter",
        "kSteerRawJumpThreshold, kSteerRawMaxStepPerFrame, kSteerFilterMaxStepPerFrame, kSteerFilterSignFlipMaxStepPerFrame, kSteerFastFlipAlpha, kSteerFastFlipRawThreshold, kSteerFastFlipGap",
        "T06 Raw/Filt分离、S弯换向、跳边后是否一帧反打、出弯是否拖旧方向。",
        "跳边猛打就降低max step或降低jump接纳；S弯换向迟钝就提高FastFlipAlpha或降低触发阈值；跨符号过猛则降低SignFlipMaxStep。",
        "这些是滤波/保护参数，先确认Raw本身是真的赛道趋势。");
    print_param_plan_line(
        "T06/T07 turn_protection",
        "kCurveEntryBrakeSpeedGoal/HoldFrames/RawThreshold/FiltThreshold/DerivativeThreshold, kSharpTurnSteerError, kExtremeTurnSpeedGoal, kExtremeTurnMinDiff, kExtremeTurnRaw/FiltThreshold, kExtremeTurnExitRaw/FiltThreshold, kExtremeTurnExitFilterAlpha",
        "T07入弯前是否提前降速、直角弯是否转不过、出弯是否继续旧方向、goal是否被压低。",
        "入弯晚就降低触发阈值或增加保持帧；转不过但T11健康可提高ExtremeTurnMinDiff或降低ExtremeTurnSpeedGoal；出弯拖尾就放宽Exit快收条件/提高ExitFilterAlpha。",
        "保护过强会让车一直慢；保护过弱会高速推头，必须结合T06逐项看。");
    print_param_plan_line(
        "T06 s_curve_target",
        "kSCurveSpeedGoal, kSCurveMinDiff, kSCurveHoldFrames, kSCurveRawMinError, kSCurveEarlyRawMinError, kSCurveEarlyFiltMinError, kSCurveEarlyDerivativeThreshold, kSCurveRawFiltGap, kSCurveDerivativeThreshold, kSCurveSharpRawThreshold",
        "T06/T07 S弯换向是否慢、是否二次反冲、是否保护误触发导致过慢。",
        "换向慢：提高SCurve目标变化率相关参数前，先确认Raw-Filt触发；二次反冲：降低MinDiff或延长Hold；误触发多：提高Raw/Filt/Derivative阈值。",
        "S弯参数只在普通弯和直道已稳定后调。");
    print_param_plan_line(
        "T06 target_gyro_limit",
        "kVisualGyroLimitDps, kTargetGyroMaxDeltaDps, kTargetGyroSCurveMaxDeltaDps, kTargetGyroSignFlipMaxDeltaDps, kTargetGyroDeadbandReturnDeltaDps, kExtremeTurnExitTargetDeltaDps, kOpposingGyroHoldDps, kOpposingGyroTargetLimitDps, kOpposingGyroRecoveryFrames, kOpposingGyroRecoveryLimitDps",
        "T06 PD/Tgt/Cmd/GF/GErr/dT、S弯反打、出弯目标是否长时间贴顶或回零太慢。",
        "PD够大但Tgt小：放宽VisualGyroLimit或目标变化率；出弯回正慢：加DeadbandReturn/ExitTargetDelta；车身还旧方向转时反打过猛：收紧Opposing目标限制或延长Recovery。",
        "T11弱时先修内环；T11健康但Tgt没给够，才动这些目标生成参数。");
    print_param_plan_line(
        "T06 fallback_debug",
        "kOuterLoopMinPeriodSec, kOuterLoopFallbackPeriodSec, kSteerDebugPrintEnable, kSteerDebugPrintDivider, kFallbackDiffLimit",
        "T06 dt、打印频率、IMU断联后的fallback表现。",
        "CPU/I2C卡顿可增大外环周期；日志太密可增divider；IMU偶断导致纯视觉猛打时降低fallback diff。",
        "外环周期改动会影响D项和目标变化率体感，改后必须回T06。");

    print_param_plan_line(
        "T08 ring_detect_common",
        "kElementRingConfirmFrames, kElementRingExitConfirmFrames, kElementCooldownFrames, kElementRingExitStraightHoldFrames, kElementRingMinActiveFrames",
        "T08/T08H Scene/Ring/Cool、left/right candidate、false entry、出环后反复触发。",
        "误入环加Confirm或Cooldown；漏入环降Confirm；出环抖动加ExitConfirm/ExitStraightHold；刚进环就退出则加MinActiveFrames。",
        "T08H图像补线不正确时先别调自动控制差速。");
    print_param_plan_line(
        "T08 ring_detect_left_right",
        "kLeftRingMinLeftLost, kLeftRingRightLineBlockMax, kLeftRingOfflineBlockMax, kLeftRingMaxPoint2Y, kRightRingMinRightLost, kRightRingLeftLineBlockMax, kRightRingOfflineBlockMax, kRightRingMinPoint2Y, kRingLossOnlyMinLost, kRingLossOnlyMaxEntryError",
        "T08H CSV里left/right candidate、block/geom/loss/susp、P1/P2、OFF、LLine/RLine。",
        "候选有但被block挡住就放宽对应BlockMax；完全没候选才降低lost/P2门槛；十字/U弯误入则提高loss_only或收紧entry_error。",
        "左右参数分开调；不要用左环失败去改右环阈值。");
    print_param_plan_line(
        "T08 ring_control",
        "kRingSpeedGoal, kRingEntryCenterBias, kRingInnerCenterBias, kRingForcedSteerError, kRingForcedSteerReleaseFlag, kRingExitSteerError, kRingEntryGyroLimitDps, kRingSceneGyroLimitDps, kRingEntryMaxDiff, kRingMaxDiff",
        "T08/TFT ring flag 5/6/7/8中心线方向、是否真正切入环、是否内切/绕圈/出环回不来。",
        "不进环先加EntryCenterBias或ForcedSteer；环内贴外侧加InnerBias，内切则降；绕圈/出不来降低环内gyro/maxdiff或调整ExitSteer；入口甩出则降低EntryGyro/EntryMaxDiff。",
        "必须先确认识别flag正确，再调控制参数。");
    print_param_plan_line(
        "T08 bad_vision_unclassified",
        "kUnclassifiedTurnTriggerError, kUnclassifiedTurnGyroLimitDps, kBadVisionOfflineThreshold, kBadVisionCenterHoldOfflineThreshold, kBadVisionSpeedGoal, kBadVisionGyroLimitDps",
        "T06/T08 OFFLine、bad vision、未识别急弯/十字/漏检环岛处Tgt是否被保护。",
        "漏检急弯转不过可降低Trigger或提高GyroLimit；丢线时猛打出界就降低BadVisionGyroLimit/SpeedGoal或提高CenterHold门槛。",
        "这是兜底保护，不应代替修图像和元素识别。");

    printf("[PLAN_NEXT] 若现在目标是无元素提速：T05通过 -> T10通过 -> T11通过 -> T06低速闭环 -> T07提速；T07失败时按T06/T11摘要定位，不直接盲调角速度环。\n");
    printf("[PLAN_NEXT] 若目标是元素：T08H先手推确认补线/识别，再T08自动低速，最后才回T07速度回归。\n");
}

static bool g_camera_ready = false;
static bool g_image_params_ready = false;

static void ensure_image_params_ready() {
    if (!g_image_params_ready) {
        Data_Settings();
        g_image_params_ready = true;
    }
}

static bool ensure_camera_ready() {
    ensure_image_params_ready();
    if (!g_camera_ready) {
        g_camera_ready = image_cv_IsReady() || image_cv_Init();
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

static void draw_scaled_point(cv::Mat &img, int x80, int y60, const cv::Scalar &color, int radius = 3) {
    if (x80 < 0 || x80 >= 80 || y60 < 0 || y60 >= 60) return;
    cv::circle(img, cv::Point(x80 * 4 + 2, y60 * 4 + 2), radius, color, -1);
}

static void draw_scaled_hline(cv::Mat &img, int y60, const cv::Scalar &color, int thickness = 1) {
    if (y60 < 0 || y60 >= 60) return;
    int y = y60 * 4 + 2;
    cv::line(img, cv::Point(0, y), cv::Point(img.cols - 1, y), color, thickness);
}

static cv::Mat make_element_debug_frame(int samples,
                                        int road_changes,
                                        int scene_changes,
                                        int img_bad_cnt,
                                        int img_warn_cnt) {
    cv::Mat debug_frame(60, 80, CV_8UC3);
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 80; ++x) {
            debug_frame.at<cv::Vec3b>(y, x) = img1[y][x] ? cv::Vec3b(245, 245, 245)
                                                         : cv::Vec3b(0, 0, 0);
        }

        int raw_l = ImageDeal[y].LeftBoundary_First;
        int raw_r = ImageDeal[y].RightBoundary_First;
        if (raw_l >= 0 && raw_l < 80) debug_frame.at<cv::Vec3b>(y, raw_l) = cv::Vec3b(0, 210, 210);
        if (raw_r >= 0 && raw_r < 80) debug_frame.at<cv::Vec3b>(y, raw_r) = cv::Vec3b(210, 210, 0);

        int left = ImageDeal[y].LeftBorder;
        int right = ImageDeal[y].RightBorder;
        int center = ImageDeal[y].Center;
        if (left >= 0 && left < 80) {
            debug_frame.at<cv::Vec3b>(y, left) =
                (ImageDeal[y].IsLeftFind == 'W') ? cv::Vec3b(255, 0, 255) : cv::Vec3b(0, 0, 255);
        }
        if (right >= 0 && right < 80) {
            debug_frame.at<cv::Vec3b>(y, right) =
                (ImageDeal[y].IsRightFind == 'W') ? cv::Vec3b(255, 0, 255) : cv::Vec3b(0, 255, 0);
        }
        if (center >= 0 && center < 80) debug_frame.at<cv::Vec3b>(y, center) = cv::Vec3b(255, 0, 0);
    }

    cv::Mat display;
    cv::resize(debug_frame, display, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);

    draw_scaled_hline(display, ImageStatus.TowPoint, cv::Scalar(255, 255, 0), 1);
    draw_scaled_hline(display, ImageStatus.OFFLine, cv::Scalar(0, 165, 255), 1);
    draw_scaled_point(display, ImageStatus.Det_True, ImageStatus.TowPoint, cv::Scalar(0, 255, 255), 5);
    draw_scaled_point(display, ImageStatus.MiddleLine, 55, cv::Scalar(255, 255, 255), 4);
    draw_scaled_hline(display, Left_RingsFlag_Point1_Ysite, cv::Scalar(0, 0, 255), 1);
    draw_scaled_hline(display, Left_RingsFlag_Point2_Ysite, cv::Scalar(0, 80, 255), 1);
    draw_scaled_hline(display, Right_RingsFlag_Point1_Ysite, cv::Scalar(0, 255, 0), 1);
    draw_scaled_hline(display, Right_RingsFlag_Point2_Ysite, cv::Scalar(80, 255, 0), 1);

    char text[160];
    int line_y = 18;
    auto put_line = [&](const cv::Scalar &color, double scale = 0.42, int thick = 1) {
        cv::putText(display, text, cv::Point(6, line_y), cv::FONT_HERSHEY_SIMPLEX, scale, color, thick);
        line_y += 17;
    };

    snprintf(text, sizeof(text), "T08 Element UDP %s:%u s=%d", kElementDebugTargetIp.c_str(), kElementDebugTargetPort, samples);
    put_line(cv::Scalar(0, 255, 255), 0.45, 1);
    snprintf(text, sizeof(text), "Road:%s Scene:%s State:%s", road_type_name(ImageStatus.Road_type), element_scene_name(g_element_scene), element_debug_state_name().c_str());
    put_line(cv::Scalar(0, 200, 255), 0.43, 1);
    snprintf(text, sizeof(text), "Ring:%d/%d Cool:%u EL:%d Lc:%d Rc:%d",
             ImageFlag.image_element_rings, ImageFlag.image_element_rings_flag, g_element_cooldown_frames,
             dbg_el_detect_open, dbg_el_l_confirm, dbg_el_r_confirm);
    put_line(cv::Scalar(0, 255, 0), 0.42, 1);
    snprintf(text, sizeof(text), "L b/g/l/s/c p1/p2/h:%d/%d/%d/%d/%d %d/%d/%d",
             dbg_el_l_block, dbg_el_l_geom, dbg_el_l_loss, dbg_el_l_susp, dbg_el_l_confirm,
             dbg_el_l_p1, dbg_el_l_p2, dbg_el_l_help);
    put_line(cv::Scalar(0, 128, 255), 0.38, 1);
    snprintf(text, sizeof(text), "R b/g/l/s/c p1/p2/h:%d/%d/%d/%d/%d %d/%d/%d",
             dbg_el_r_block, dbg_el_r_geom, dbg_el_r_loss, dbg_el_r_susp, dbg_el_r_confirm,
             dbg_el_r_p1, dbg_el_r_p2, dbg_el_r_help);
    put_line(cv::Scalar(80, 255, 80), 0.38, 1);
    snprintf(text, sizeof(text), "Det:%d Err:%d Tow:%d OFF:%d L/R:%d/%d raw/final lines shown",
             ImageStatus.Det_True, ImageStatus.MiddleLine - ImageStatus.Det_True,
             ImageStatus.TowPoint, ImageStatus.OFFLine, ImageStatus.Left_Line, ImageStatus.Right_Line);
    put_line(cv::Scalar(255, 255, 255), 0.39, 1);
    snprintf(text, sizeof(text), "D:%.1f Tgt:%.1f GErr:%.1f Enc:%.0f/%.0f chg:%d/%d bad:%d warn:%d",
             dbg_diff_speed, dbg_target_gyro_z, dbg_gyro_error,
             speed_encoder_L, speed_encoder_R, road_changes, scene_changes, img_bad_cnt, img_warn_cnt);
    put_line(cv::Scalar(255, 255, 0), 0.38, 1);
    snprintf(text, sizeof(text), "Color rawL=cyan rawR=yellow L=red R=green C=blue W=magenta P1/P2=hline");
    put_line(cv::Scalar(180, 180, 180), 0.34, 1);

    return display;
}

static cv::Mat make_element_tft_frame(int samples,
                                      int road_changes,
                                      int scene_changes,
                                      int img_bad_cnt,
                                      int img_warn_cnt,
                                      bool motor_on) {
    cv::Mat base(60, 80, CV_8UC3);
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 80; ++x) {
            base.at<cv::Vec3b>(y, x) = img1[y][x] ? cv::Vec3b(235, 235, 235)
                                                  : cv::Vec3b(0, 0, 0);
        }

        int raw_l = ImageDeal[y].LeftBoundary_First;
        int raw_r = ImageDeal[y].RightBoundary_First;
        if (raw_l >= 0 && raw_l < 80) base.at<cv::Vec3b>(y, raw_l) = cv::Vec3b(0, 210, 210);
        if (raw_r >= 0 && raw_r < 80) base.at<cv::Vec3b>(y, raw_r) = cv::Vec3b(210, 210, 0);

        int left = ImageDeal[y].LeftBorder;
        int right = ImageDeal[y].RightBorder;
        int center = ImageDeal[y].Center;
        if (left >= 0 && left < 80) {
            base.at<cv::Vec3b>(y, left) =
                (ImageDeal[y].IsLeftFind == 'W') ? cv::Vec3b(255, 0, 255) : cv::Vec3b(0, 0, 255);
        }
        if (right >= 0 && right < 80) {
            base.at<cv::Vec3b>(y, right) =
                (ImageDeal[y].IsRightFind == 'W') ? cv::Vec3b(255, 0, 255) : cv::Vec3b(0, 255, 0);
        }
        if (center >= 0 && center < 80) base.at<cv::Vec3b>(y, center) = cv::Vec3b(255, 0, 0);
    }

    cv::Mat display;
    cv::resize(base, display, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);

    auto draw_hline2 = [&](int y60, const cv::Scalar &color) {
        if (y60 < 0 || y60 >= 60) return;
        int y = y60 * 2 + 1;
        cv::line(display, cv::Point(0, y), cv::Point(display.cols - 1, y), color, 1);
    };
    auto draw_point2 = [&](int x80, int y60, const cv::Scalar &color) {
        if (x80 < 0 || x80 >= 80 || y60 < 0 || y60 >= 60) return;
        cv::circle(display, cv::Point(x80 * 2 + 1, y60 * 2 + 1), 2, color, -1);
    };

    draw_hline2(ImageStatus.TowPoint, cv::Scalar(255, 255, 0));
    draw_hline2(ImageStatus.OFFLine, cv::Scalar(0, 165, 255));
    draw_hline2(Left_RingsFlag_Point1_Ysite, cv::Scalar(0, 0, 255));
    draw_hline2(Left_RingsFlag_Point2_Ysite, cv::Scalar(0, 80, 255));
    draw_hline2(Right_RingsFlag_Point1_Ysite, cv::Scalar(0, 255, 0));
    draw_hline2(Right_RingsFlag_Point2_Ysite, cv::Scalar(80, 255, 0));
    draw_point2(ImageStatus.Det_True, ImageStatus.TowPoint, cv::Scalar(0, 255, 255));
    draw_point2(ImageStatus.MiddleLine, 55, cv::Scalar(255, 255, 255));

    cv::rectangle(display, cv::Point(0, 0), cv::Point(159, 35), cv::Scalar(0, 0, 0), -1);
    char text[96];
    snprintf(text, sizeof(text), "R:%s S:%s %s",
             road_type_name(ImageStatus.Road_type),
             element_scene_name(g_element_scene),
             motor_on ? "AUTO" : "PUSH");
    cv::putText(display, text, cv::Point(2, 9), cv::FONT_HERSHEY_SIMPLEX, 0.28, cv::Scalar(0, 255, 255), 1);
    snprintf(text, sizeof(text), "ring:%d/%d cool:%u err:%d off:%d",
             ImageFlag.image_element_rings,
             ImageFlag.image_element_rings_flag,
             g_element_cooldown_frames,
             ImageStatus.MiddleLine - ImageStatus.Det_True,
             ImageStatus.OFFLine);
    cv::putText(display, text, cv::Point(2, 20), cv::FONT_HERSHEY_SIMPLEX, 0.28, cv::Scalar(0, 255, 0), 1);
    snprintf(text, sizeof(text), "L:%d/%d/%d/%d R:%d/%d/%d/%d b:%d w:%d s:%d",
             dbg_el_l_block, dbg_el_l_geom, dbg_el_l_loss, dbg_el_l_confirm,
             dbg_el_r_block, dbg_el_r_geom, dbg_el_r_loss, dbg_el_r_confirm,
             img_bad_cnt, img_warn_cnt, samples);
    cv::putText(display, text, cv::Point(2, 31), cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(255, 255, 255), 1);

    (void)road_changes;
    (void)scene_changes;
    return display;
}

// -----------------------------------------------------------------------------
// 手推车 TFT 补线观察帧生成器 — 显示二值图 + 边线 + 中线 + 实时FPS
// -----------------------------------------------------------------------------
static cv::Mat make_push_tft_frame(float fps, int frame_count) {
    cv::Mat base(60, 80, CV_8UC3);
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 80; ++x) {
            base.at<cv::Vec3b>(y, x) = img1[y][x] ? cv::Vec3b(235, 235, 235)
                                                  : cv::Vec3b(0, 0, 0);
        }

        int raw_l = ImageDeal[y].LeftBoundary_First;
        int raw_r = ImageDeal[y].RightBoundary_First;
        if (raw_l >= 0 && raw_l < 80) base.at<cv::Vec3b>(y, raw_l) = cv::Vec3b(0, 210, 210);
        if (raw_r >= 0 && raw_r < 80) base.at<cv::Vec3b>(y, raw_r) = cv::Vec3b(210, 210, 0);

        int left = ImageDeal[y].LeftBorder;
        int right = ImageDeal[y].RightBorder;
        int center = ImageDeal[y].Center;
        if (left >= 0 && left < 80) {
            base.at<cv::Vec3b>(y, left) =
                (ImageDeal[y].IsLeftFind == 'W') ? cv::Vec3b(255, 0, 255) : cv::Vec3b(0, 0, 255);
        }
        if (right >= 0 && right < 80) {
            base.at<cv::Vec3b>(y, right) =
                (ImageDeal[y].IsRightFind == 'W') ? cv::Vec3b(255, 0, 255) : cv::Vec3b(0, 255, 0);
        }
        if (center >= 0 && center < 80) base.at<cv::Vec3b>(y, center) = cv::Vec3b(255, 0, 0);
    }

    cv::Mat display;
    cv::resize(base, display, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);

    auto draw_hline2 = [&](int y60, const cv::Scalar &color) {
        if (y60 < 0 || y60 >= 60) return;
        int y = y60 * 2 + 1;
        cv::line(display, cv::Point(0, y), cv::Point(display.cols - 1, y), color, 1);
    };
    auto draw_point2 = [&](int x80, int y60, const cv::Scalar &color) {
        if (x80 < 0 || x80 >= 80 || y60 < 0 || y60 >= 60) return;
        cv::circle(display, cv::Point(x80 * 2 + 1, y60 * 2 + 1), 2, color, -1);
    };

    draw_hline2(ImageStatus.TowPoint, cv::Scalar(255, 255, 0));
    draw_hline2(ImageStatus.OFFLine, cv::Scalar(0, 165, 255));
    draw_point2(ImageStatus.Det_True, ImageStatus.TowPoint, cv::Scalar(0, 255, 255));
    draw_point2(ImageStatus.MiddleLine, 55, cv::Scalar(255, 255, 255));

    // Top info bar with FPS
    cv::rectangle(display, cv::Point(0, 0), cv::Point(159, 35), cv::Scalar(0, 0, 0), -1);
    char text[96];
    snprintf(text, sizeof(text), "PUSH FPS:%.1f #%d", fps, frame_count);
    cv::putText(display, text, cv::Point(2, 9), cv::FONT_HERSHEY_SIMPLEX, 0.30,
                cv::Scalar(0, 255, 255), 1);
    snprintf(text, sizeof(text), "Road:%s Det:%d Err:%d OFF:%d",
             road_type_name(ImageStatus.Road_type),
             ImageStatus.Det_True,
             ImageStatus.MiddleLine - ImageStatus.Det_True,
             ImageStatus.OFFLine);
    cv::putText(display, text, cv::Point(2, 20), cv::FONT_HERSHEY_SIMPLEX, 0.28,
                cv::Scalar(0, 255, 0), 1);
    snprintf(text, sizeof(text), "L/R:%d/%d Tow:%d rawL=cyn rawR=yel L=red R=grn C=blu",
             ImageStatus.Left_Line, ImageStatus.Right_Line, ImageStatus.TowPoint);
    cv::putText(display, text, cv::Point(2, 31), cv::FONT_HERSHEY_SIMPLEX, 0.25,
                cv::Scalar(255, 255, 255), 1);

    return display;
}

static bool init_element_tft_once(const char *tag) {
    if (!kElementDebugTftEnable) return false;
    static bool inited = false;
    if (!inited) {
        lq_tft18_drv_init(1);
        lq_tft18_drv_cls(U16BLACK);
        inited = true;
        printf("[%s_TFT] enabled: TFT18 shows realtime binary image + raw/final borders + center + ring state.\n", tag);
        printf("[%s_TFT] color: rawL=cyan rawR=yellow L=red R=green C=blue W=magenta P1/P2=horizontal lines.\n", tag);
    }
    return true;
}

static void show_element_tft_frame(const char *tag,
                                   int samples,
                                   int road_changes,
                                   int scene_changes,
                                   int img_bad_cnt,
                                   int img_warn_cnt,
                                   bool motor_on) {
    if (!init_element_tft_once(tag)) return;
    cv::Mat tft_frame = make_element_tft_frame(samples, road_changes, scene_changes,
                                               img_bad_cnt, img_warn_cnt, motor_on);
    lq_tft18_drv_road_color(kElementDebugTftX, kElementDebugTftY, tft_frame);
}

// -----------------------------------------------------------------------------
// Step 0: 调参前准备，打印配置快照，不动电机。
// -----------------------------------------------------------------------------
int test_00_precheck_config(int seconds = 3) {
    printf("\n========== TEST 00 precheck_config 调参前准备 ==========" "\n");
    print_stage_card("T00 precheck_config",
                     "确认本次二进制实际编译进来的关键参数，避免把不同提交/不同阶段的数值混在一起。",
                     "不调参数；只记录日志、提交号、赛道环境、电池状态。",
                     "不要在没跑后续测试前凭感觉同时改速度、PID、图像和元素参数。",
                     "确认配置快照无误后，图像走T02/T03/T04，硬件走T01/T05。");
    printf("[目的] 确认当前编译进程序的关键参数；后续每次发日志先带这一段。\n");
    print_config_snapshot();
    print_stage_task_map();
    printf("[参数地图] 若要看每个参数的细化调试方案，单独运行 lq_param_test_12_parameter_debug_plan()。\n");
    printf("[要求] 电池电压保持同一区间；摄像头高度约30cm；蓝底白色PVC赛道光照稳定；每轮只改一个参数。\n");
    printf("[RETURN] 0=配置打印完成。\n");
    (void)seconds;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 12: 参数级调试方案。只打印完整调参地图，不读传感器、不跑电机。
// -----------------------------------------------------------------------------
int test_12_parameter_debug_plan(void) {
    printf("\n========== TEST 12 parameter_debug_plan 参数级调试方案 ==========" "\n");
    print_stage_card("T12 parameter_debug_plan",
                     "把car_config里的关键参数映射到对应测试阶段、观察指标、调整方向和禁忌项。",
                     "不调参数；用于决定下一轮该跑哪个测试、只改哪一组参数。",
                     "不要把T12当成实测结论；它是路线图，真正结论必须来自T02/T03/T04/T05/T10/T11/T06/T07/T08日志。",
                     "按T12定位参数后，回到对应测试阶段复测。");
    print_config_snapshot();
    print_stage_task_map();
    print_parameter_debug_plan();
    printf("[RETURN] 0=参数调试方案打印完成。\n");
    return 0;
}

// -----------------------------------------------------------------------------
// Step 1.1/1.2: 电机方向 + 编码器方向。需要架空小车，多档PWM自动测试。
// 返回值：负数表示方向/反馈明显异常；0表示左右方向正确且数量级接近。
// -----------------------------------------------------------------------------
int test_01_motor_encoder_direction(int pwm = 1200, int hold_ms = 700) {
    printf("\n========== TEST 01 motor_encoder_direction 电机/编码器方向 ==========" "\n");
    print_stage_card("T01 motor_encoder_direction",
                     "架空确认左右电机方向、编码器正负号、启动死区和左右机械一致性。",
                     "电机接线/方向GPIO、编码器A/B相或编码器取反、明显机械卡滞。",
                     "不要用速度PID、转向PID或差速限幅去掩盖方向错误和编码器反号。",
                     "方向和左右反馈都通过后再跑T05速度环阶跃。");
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
            float v = left ? encoder_num_L() : encoder_num_R();
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
// Step 1.3 + 摄像头角度核心测试：三轮重复摆放采样，统计图像重复性、可视距离、宽度梯度。
// 注意：人工摆车无法做到像素级居中，本测试不再用单次err_mean阻断流程。
// 返回值：0=图像重复性和角度基本可用；1=中心/Det抖动大；2=看得太近/角度太低；3=直道判断不稳或远端噪声大。
// -----------------------------------------------------------------------------
int test_02_camera_center_angle_static(int frames = 120) {
    printf("\n========== TEST 02 camera_alignment_repeatability 摄像头对线重复性/角度 ==========" "\n");
    print_stage_card("T02 camera_alignment_repeatability",
                     "用三轮重复采样判断摄像头画面是否稳定、边界是否连续、俯仰是否合适；不要求手摆像素级居中。",
                     "摄像头机械位置/俯仰、ROI粗位置、图像稳定性问题；kImageMiddleLine只在三轮趋势一致时每次微调1。",
                     "不要用单轮err_mean改中线；不要在图像还跳动时调PID、速度、元素或环岛参数。",
                     "Det稳定、丢线少、直道判断不过度抖动后进入T03二值质量测试。");
    printf("[摆放] 第1轮尽量放直道中间；第2/3轮可把车重新拿起再放回同一段直道，尽量正对即可，不需要像素级微调。\n");
    printf("[任务] 本环节判断重复性和角度；中线偏置只输出候选，不作为单次失败条件。\n");
    g_enable_element_process = false;

    struct AlignRound {
        RunningStat err, det, off, width, white, variance, near_center, mid_center, far_center, near_far_delta;
        int frames = 0;
        int straight_cnt = 0;
        int lost_cnt = 0;
        int det_jump_cnt = 0;
        int center_row_bad_cnt = 0;
    };

    const int rounds = 3;
    int frames_per_round = std::max(20, frames / rounds);
    int total_frames = 0;
    std::vector<AlignRound> round_stats(rounds);

    for (int r = 0; r < rounds && !exit_flag; ++r) {
        printf("[T02_ROUND_BEGIN] round=%d/%d frames=%d。若刚跑完上一轮，可在3秒内重新摆放小车；不用追求像素级居中。\n",
               r + 1, rounds, frames_per_round);
        if (r > 0) usleep(3000 * 1000);

        AlignRound &st = round_stats[r];
        int last_det = 0;
        for (int i = 0; i < frames_per_round && !exit_flag; ++i) {
            if (!grab_and_process()) return -1;
            int e = ImageStatus.MiddleLine - ImageStatus.Det_True;
            int w = estimate_near_line_width();
            int c_near = ImageDeal[55].Center;
            int c_mid = ImageDeal[45].Center;
            int c_far = ImageDeal[35].Center;
            st.err.add(e);
            st.det.add(ImageStatus.Det_True);
            st.off.add(ImageStatus.OFFLine);
            st.width.add(w);
            st.white.add(white_ratio_all());
            st.variance.add(calc_center_variance());
            st.near_center.add(c_near);
            st.mid_center.add(c_mid);
            st.far_center.add(c_far);
            st.near_far_delta.add(c_near - c_far);
            st.frames++;
            total_frames++;
            if (ImageStatus.straight_acc_flag) st.straight_cnt++;
            if (i > 0 && std::abs(ImageStatus.Det_True - last_det) > 5) st.det_jump_cnt++;
            if (std::abs(c_near - c_mid) > 8 || std::abs(c_mid - c_far) > 8) st.center_row_bad_cnt++;
            if (ImageStatus.OFFLine > 15 || w <= 3) st.lost_cnt++;
            last_det = ImageStatus.Det_True;
            if (i % 20 == 0) print_image_line_once("[T02_FRAME]");
            usleep(20 * 1000);
        }

        double straight_rate_r = st.frames ? (double)st.straight_cnt / st.frames : 0.0;
        double lost_rate_r = st.frames ? (double)st.lost_cnt / st.frames : 0.0;
        double det_jump_rate_r = st.frames ? (double)st.det_jump_cnt / st.frames : 0.0;
        double row_bad_rate_r = st.frames ? (double)st.center_row_bad_cnt / st.frames : 0.0;
        printf("[T02_ROUND_SUMMARY] round=%d frames=%d det_mean=%.2f det_std=%.2f det_span=%.2f err_mean=%.2f OFF=%.2f width=%.2f white=%.3f var=%.2f straight=%.2f lost=%.2f det_jump=%.2f row_bad=%.2f near/mid/far=%.1f/%.1f/%.1f\n",
               r + 1, st.frames, st.det.mean(), st.det.stddev(), st.det.span(), st.err.mean(),
               st.off.mean(), st.width.mean(), st.white.mean(), st.variance.mean(),
               straight_rate_r, lost_rate_r, det_jump_rate_r, row_bad_rate_r,
               st.near_center.mean(), st.mid_center.mean(), st.far_center.mean());
    }

    RunningStat all_err, all_det, all_off, all_width, all_white, all_variance;
    RunningStat all_near, all_mid, all_far, all_near_far;
    RunningStat round_det_means, round_delta_means;
    int straight_cnt = 0, lost_cnt = 0, det_jump_cnt = 0, center_row_bad_cnt = 0;
    int same_positive = 0, same_negative = 0, stable_rounds = 0;
    for (int r = 0; r < rounds; ++r) {
        AlignRound &st = round_stats[r];
        straight_cnt += st.straight_cnt;
        lost_cnt += st.lost_cnt;
        det_jump_cnt += st.det_jump_cnt;
        center_row_bad_cnt += st.center_row_bad_cnt;
        all_err.sum += st.err.sum; all_err.sum_abs += st.err.sum_abs; all_err.sum_sq += st.err.sum_sq; all_err.n += st.err.n; all_err.min_v = std::min(all_err.min_v, st.err.min_v); all_err.max_v = std::max(all_err.max_v, st.err.max_v);
        all_det.sum += st.det.sum; all_det.sum_abs += st.det.sum_abs; all_det.sum_sq += st.det.sum_sq; all_det.n += st.det.n; all_det.min_v = std::min(all_det.min_v, st.det.min_v); all_det.max_v = std::max(all_det.max_v, st.det.max_v);
        all_off.sum += st.off.sum; all_off.sum_abs += st.off.sum_abs; all_off.sum_sq += st.off.sum_sq; all_off.n += st.off.n; all_off.min_v = std::min(all_off.min_v, st.off.min_v); all_off.max_v = std::max(all_off.max_v, st.off.max_v);
        all_width.sum += st.width.sum; all_width.sum_abs += st.width.sum_abs; all_width.sum_sq += st.width.sum_sq; all_width.n += st.width.n; all_width.min_v = std::min(all_width.min_v, st.width.min_v); all_width.max_v = std::max(all_width.max_v, st.width.max_v);
        all_white.sum += st.white.sum; all_white.sum_abs += st.white.sum_abs; all_white.sum_sq += st.white.sum_sq; all_white.n += st.white.n; all_white.min_v = std::min(all_white.min_v, st.white.min_v); all_white.max_v = std::max(all_white.max_v, st.white.max_v);
        all_variance.sum += st.variance.sum; all_variance.sum_abs += st.variance.sum_abs; all_variance.sum_sq += st.variance.sum_sq; all_variance.n += st.variance.n; all_variance.min_v = std::min(all_variance.min_v, st.variance.min_v); all_variance.max_v = std::max(all_variance.max_v, st.variance.max_v);
        all_near.sum += st.near_center.sum; all_near.sum_abs += st.near_center.sum_abs; all_near.sum_sq += st.near_center.sum_sq; all_near.n += st.near_center.n; all_near.min_v = std::min(all_near.min_v, st.near_center.min_v); all_near.max_v = std::max(all_near.max_v, st.near_center.max_v);
        all_mid.sum += st.mid_center.sum; all_mid.sum_abs += st.mid_center.sum_abs; all_mid.sum_sq += st.mid_center.sum_sq; all_mid.n += st.mid_center.n; all_mid.min_v = std::min(all_mid.min_v, st.mid_center.min_v); all_mid.max_v = std::max(all_mid.max_v, st.mid_center.max_v);
        all_far.sum += st.far_center.sum; all_far.sum_abs += st.far_center.sum_abs; all_far.sum_sq += st.far_center.sum_sq; all_far.n += st.far_center.n; all_far.min_v = std::min(all_far.min_v, st.far_center.min_v); all_far.max_v = std::max(all_far.max_v, st.far_center.max_v);
        all_near_far.sum += st.near_far_delta.sum; all_near_far.sum_abs += st.near_far_delta.sum_abs; all_near_far.sum_sq += st.near_far_delta.sum_sq; all_near_far.n += st.near_far_delta.n; all_near_far.min_v = std::min(all_near_far.min_v, st.near_far_delta.min_v); all_near_far.max_v = std::max(all_near_far.max_v, st.near_far_delta.max_v);

        double delta = st.det.mean() - car_config::kImageMiddleLine;
        round_det_means.add(st.det.mean());
        round_delta_means.add(delta);
        if (delta > 2.0) same_positive++;
        if (delta < -2.0) same_negative++;
        if (st.det.stddev() <= 1.8 && st.det.span() <= 7.0) stable_rounds++;
    }

    double straight_rate = total_frames ? (double)straight_cnt / total_frames : 0.0;
    double lost_rate = total_frames ? (double)lost_cnt / total_frames : 0.0;
    double det_jump_rate = total_frames ? (double)det_jump_cnt / total_frames : 0.0;
    double center_row_bad_rate = total_frames ? (double)center_row_bad_cnt / total_frames : 0.0;
    bool middle_candidate_ok = (stable_rounds == rounds &&
                                round_det_means.stddev() <= 1.5 &&
                                (same_positive == rounds || same_negative == rounds) &&
                                std::fabs(round_delta_means.mean()) >= 2.5);
    int candidate_middle = round_to_int(car_config::kImageMiddleLine + (round_delta_means.mean() > 0.0 ? 1.0 : -1.0));
    printf("[T02_SUMMARY] frames=%d rounds=%d err_mean=%.2f err_abs=%.2f err_rms=%.2f det_mean=%.2f det_std=%.2f det_span=%.2f round_det_std=%.2f round_delta_mean=%.2f current_middle=%d middle_candidate_ok=%d candidate_next=%d OFF_mean=%.2f width_mean=%.2f white_mean=%.3f var_mean=%.2f straight_rate=%.2f lost_rate=%.2f det_jump_rate=%.2f row_bad_rate=%.2f\n",
           total_frames, rounds, all_err.mean(), all_err.mean_abs(), all_err.rms(),
           all_det.mean(), all_det.stddev(), all_det.span(), round_det_means.stddev(),
           round_delta_means.mean(), car_config::kImageMiddleLine,
           middle_candidate_ok ? 1 : 0, middle_candidate_ok ? candidate_middle : car_config::kImageMiddleLine,
           all_off.mean(), all_width.mean(), all_white.mean(), all_variance.mean(),
           straight_rate, lost_rate, det_jump_rate, center_row_bad_rate);
    printf("[T02_ROWS] center_near55=%.2f mid45=%.2f far35=%.2f near_minus_far=%.2f near_std=%.2f mid_std=%.2f far_std=%.2f\n",
           all_near.mean(), all_mid.mean(), all_far.mean(), all_near_far.mean(),
           all_near.stddev(), all_mid.stddev(), all_far.stddev());
    printf("[中线判断] middle_candidate_ok=1时也只建议kImageMiddleLine朝candidate_next改1；若为0，说明人工摆放/图像抖动/轮次差异仍占主导，先不要改中线。\n");
    printf("[角度判断] OFF_mean大/width小/丢线高 => 摄像头角度太低看太近或ROI太靠下；near_minus_far长期很大 => 车头/摄像头姿态或透视角需要先机械调整。\n");
    printf("[T02_HINT] straight_rate只作为T04直道判断参考，不再阻断T02；T02只硬判图像稳定性、丢线和角度/可视距离。\n");
    printf("[RETURN] 0=可进入二值质量测试；1=Det/中心线跳动大；2=疑似角度太低/看太近/丢线；3=远端中心线形态异常。\n");
    if (lost_rate > 0.20 || all_off.mean() > 15 || all_width.mean() < 5) return 2;
    if (all_det.stddev() > 3.0 || all_det.span() > 12.0 || det_jump_rate > 0.08 || center_row_bad_rate > 0.25) return 1;
    if (all_variance.mean() > car_config::kStraightVarianceAccThreshold * 2.5) return 3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 2/3: 曝光 + OTSU 二值质量测试。不改参数，打印二值化稳定性指标。
// 返回值：0=mask稳定；1=白点比例异常；2=闪烁大；3=边界/丢线异常。
// -----------------------------------------------------------------------------
int test_03_exposure_otsu_quality(int frames = 160) {
    printf("\n========== TEST 03 binary_quality 曝光/二值质量 ==========" "\n");
    print_stage_card("T03 binary_quality",
                     "固定或慢推经过直道/弯道/阴影/反光区域，确认二值mask比例、边界和Det不闪。",
                     "kCameraManualExposure、kImageAlgorithm、二值阈值/OTSU滤波/形态学、ROI基础区域。",
                     "不要改kImageTowPoint、速度、PID、差速、环岛和元素参数；图像质量不过时控制参数没有意义。",
                     "white比例、Det跳变和丢边都稳定后进入T04前瞻/扫线/直道判断。");
    printf("[摆放] 依次对准直道/弯道/阴影/反光区域均可。先固定车不动跑一段，再可轻推很慢经过问题区域。\n");
    printf("[任务] 只调摄像头曝光、二值化算法/阈值、ROI基础质量；不要在本环节改PID、速度、前瞻或元素参数。\n");
    printf("[当前] algorithm=%s。本测试重点看white比例稳定性、Det抖动和边界丢失。\n",
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
    printf("[调参意义] white异常先看曝光/ROI/反光/二值算法；dDet大看二值毛刺和前瞻行；lost高先调摄像头角度/ROI，不要急着改控制参数。\n");
    printf("[RETURN] 0=曝光/二值基本稳定；1=white比例异常；2=mask/Det闪烁；3=边界丢失。\n");
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
    print_stage_card("T04 roi_scan_straight",
                     "低速手推或静态覆盖普通直道/普通弯，检查入弯提前量、边界连续性和Straight/Normol稳定性。",
                     "kImageTowPoint、ROI位置、扫线范围、直道判断阈值。",
                     "不要用err_abs校准kImageMiddleLine；不要改速度环、转向环、陀螺仪环或元素参数。",
                     "jump/edge_bad/straight_rate稳定后进入T05或T10/T06。");
    printf("[运行] 可放在直道中心静止，也可低速手推过普通弯。输出会显示入弯是否提前、边界是否连续、直道判断是否稳定。\n");
    printf("[任务] 只调kImageTowPoint、ROI、扫线范围、直道判断阈值；不要用本环节的err_abs去校准kImageMiddleLine。\n");
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
    printf("[阶段边界] 若T02/T03没过，不要用T04结论调控制；若只有err_abs大但dErr/edge稳定，优先怀疑摆车偏差而不是中线参数。\n");
    printf("[RETURN] 0=稳定；1=疑似看太近/入弯晚；2=扫线边界不连续；3=直道判断不稳。\n");
    if (off.mean() > 18 || width.mean() < 5) return 1;
    if (edge_bad_rate > 0.20 || jump_rate > 0.20) return 2;
    if (straight_rate < 0.65 && err.mean_abs() < 4.0 && var.mean() < car_config::kStraightVarianceAccThreshold * 1.5) return 3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 5: 实地直线速度环测试
//   终端实时调节：最低速度 最高速度 周期(s) 运行时长(ms) [Kp] [Ki]
//   T05默认关闭速度环滤波，直接记录/控制编码器原始速度，方便判断落地负载下的真实跟随波形。
//   控制周期 10ms，日志每 20ms 输出一帧，兼顾调参分辨率和打印扰动。
//   目标速度用半余弦曲线在[min,max]之间平滑往返，适合短直道重复比较。
//   公式: target = min + (max - min) * 0.5 * (1 - cos(2π * t / period))
// -----------------------------------------------------------------------------
int test_05_speed_loop_airborne(float /*unused_target*/ = 1.0f, int /*unused_seconds*/ = 5) {
    printf("\n========== TEST 05 actual_speed_loop_straight 实地直线速度环 ==========\n");
    print_stage_card("T05 actual_speed_loop_straight",
                     "小车落地直线跑，左右同目标速度曲线。终端实时输入调节参数、即时验证PI组合。",
                     "kSpeedPidP、kSpeedPidI、PWM限幅、左右机械一致性、落地速度跟随响应。",
                     "不要在本测试调视觉/角速度/元素参数；本测试只评价直线速度环，不评价循迹好坏。",
                     "速度跟随稳定后再跑T06/T07上赛道闭环。");
    printf("[安全] 小车会直线前进！只在空旷直道/赛道直线段测试，车头对准安全方向，手边准备Ctrl+C。\n");
    printf("\n");
    printf("=== 实地速度环跟随测试 ===\n");
    printf("公式: target = min + (max - min) * 0.5 * (1 - cos(2π * t / period))\n");
    printf("格式: <最低速度(m/s)> <最高速度(m/s)> <周期(s)> <运行时长(ms)> [Kp] [Ki]\n");
    printf("短场地示例: 0.9 1.3 2.0 4000 2000 40   (0.9~1.3 m/s，2s周期，跑2周期)\n");
    printf("常规示例:   1.0 1.5 3.0 6000 2000 40   (1.0~1.5 m/s，3s周期，跑2周期)\n");
    printf("输入 q 退出\n");
    printf("========================================\n");
    printf("注意: Kp/Ki热改对主PID有效(speed_P_l/r是全局变量); 测试结束后自动恢复car_config默认值。\n\n");

    init_control_for_motor_tests();
    Speed_Goal = 0;
    Speed_Goal_l = 0;
    Speed_Goal_r = 0;
    speed_out_l = speed_out_r = 0;
    speed_last_erro_L = speed_last_erro_R = 0;
    motor_Duty_L(0);
    motor_Duty_R(0);
    usleep(300 * 1000);

    // 保存原始 PID 参数，测试结束后恢复
    float orig_Pl = speed_P_l, orig_Il = speed_I_l;
    float orig_Pr = speed_P_r, orig_Ir = speed_I_r;

    // --- 实地速度任务状态 ---
    float sine_base      = 0.9f;    // 最低速度 m/s
    float sine_max       = 1.3f;    // 最高速度 m/s
    float sine_period_s  = 3.0f;    // 速度曲线周期 s
    int   task_remaining_ms = 0;
    int   sine_tick = 0;
    const int DT_MS = 10;           // 控制周期 10ms = 100Hz
    const int LOG_DT_MS = 20;       // 打印周期 20ms = 50Hz，适合后续AI分析相位/超调/PWM饱和
    const int LOG_DIVIDER = LOG_DT_MS / DT_MS;
    const float DT_SEC = DT_MS / 1000.0f;
    int stuck_guard_ticks = 0;

    // --- 终端输入缓冲区 ---
    std::string input_buf;
    bool need_prompt = true;

    auto show_prompt = [&]() {
        if (need_prompt) {
            printf(">>> [T05] 当前 Kp=%.0f/%.0f Ki=%.0f/%.0f | 输入 最低 最高 周期 时长 [Kp] [Ki]: ",
                   speed_P_l, speed_P_r, speed_I_l, speed_I_r);
            fflush(stdout);
            need_prompt = false;
        }
    };
    show_prompt();

    while (!exit_flag) {
        // ==========================================
        // 非阻塞终端输入
        // ==========================================
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 0};
        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                if (c == '\n' || c == '\r') {
                    if (!input_buf.empty()) {
                        while (!input_buf.empty() && (input_buf.back() == '\r' || input_buf.back() == ' '))
                            input_buf.pop_back();

                        if (input_buf == "q") {
                            printf("\n>>> [T05] 退出。\n");
                            exit_flag = true;
                            input_buf.clear();
                            continue;
                        }

                        std::istringstream iss(input_buf);
                        float in_base = 0, in_max = 0, in_period = 0;
                        int in_duration = 0;
                        float in_Kp = 0, in_Ki = 0;

                        if (iss >> in_base >> in_max >> in_period >> in_duration) {
                            if (in_base < 0.2f) in_base = 0.2f;
                            if (in_max < in_base) in_max = in_base;
                            if (in_period < 1.0f) in_period = 1.0f;
                            if (in_duration < DT_MS) in_duration = DT_MS;

                            sine_base      = in_base;
                            sine_max       = in_max;
                            sine_period_s  = in_period;
                            task_remaining_ms = in_duration;
                            sine_tick = 0;
                            stuck_guard_ticks = 0;

                            // 可选 Kp/Ki 热改
                            if (iss >> in_Kp) {
                                speed_P_l = in_Kp; speed_P_r = in_Kp;
                                if (iss >> in_Ki) {
                                    speed_I_l = in_Ki; speed_I_r = in_Ki;
                                }
                            }

                            speed_out_l = speed_out_r = 0;
                            speed_last_erro_L = speed_last_erro_R = 0;
                            // 初始化速度滤波状态，避免上一轮旧值残留
                            speed_encoder_L = encoder_num_L();
                            speed_encoder_R = encoder_num_R();

                            printf("\n>>> [执行] min=%.2f max=%.2f amplitude=%.2f period=%.1fs duration=%dms Kp=%.0f Ki=%.0f\n",
                                   sine_base, sine_max, sine_max - sine_base, sine_period_s, task_remaining_ms,
                                   speed_P_l, speed_I_l);
                            printf("[T05_ACTUAL_HEADER] log_dt_ms=%d columns=t_ms,target,encL,encR,errL,errR,pwmL,pwmR\n", LOG_DT_MS);
                            need_prompt = true;
                        } else {
                            printf("\n>>> [错误] 无法解析: %s\n", input_buf.c_str());
                        }
                        input_buf.clear();
                    }
                } else {
                    input_buf += c;
                }
            }
        }

        // ==========================================
        // 任务计时
        // ==========================================
        if (task_remaining_ms > 0) {
            task_remaining_ms -= DT_MS;
            if (task_remaining_ms <= 0) {
                task_remaining_ms = 0;
                sine_tick = 0;
                Speed_Goal = Speed_Goal_l = Speed_Goal_r = 0;
                speed_out_l = speed_out_r = 0;
                speed_last_erro_L = speed_last_erro_R = 0;
                motor_Duty_L(0); motor_Duty_R(0);
                printf("\n>>> [T05] 实地速度任务完成，电机已停止。\n");
                show_prompt();
            }
        }

        // ==========================================
        // 计算实地速度目标：在最低/最高速度之间平滑往返，避免短场地里夹到0。
        // ==========================================
        float target_speed = 0.0f;
        if (task_remaining_ms > 0) {
            float t = (float)sine_tick * DT_SEC;
            float phase = 2.0f * (float)M_PI * t / sine_period_s;
            float profile = 0.5f * (1.0f - cosf(phase));
            target_speed = sine_base + (sine_max - sine_base) * profile;
        }

        // ==========================================
        // 获取实际速度 + 执行 PID
        // T05用于无速度环滤波调参：直接使用编码器原始速度，避免滤波掩盖超调/毛刺。
        // ==========================================
        Speed_Goal = Speed_Goal_l = Speed_Goal_r = target_speed;
        speed_encoder_L = encoder_num_L();
        speed_encoder_R = encoder_num_R();
        Speed_PID_L();
        Speed_PID_R();

        int pwm_l = (int)speed_out_l;
        int pwm_r = (int)speed_out_r;
        // PWM 限幅
        if (pwm_l > (int)car_config::kMotorMaxPwm)  pwm_l = (int)car_config::kMotorMaxPwm;
        if (pwm_l < -(int)car_config::kMotorMaxPwm) pwm_l = -(int)car_config::kMotorMaxPwm;
        if (pwm_r > (int)car_config::kMotorMaxPwm)  pwm_r = (int)car_config::kMotorMaxPwm;
        if (pwm_r < -(int)car_config::kMotorMaxPwm) pwm_r = -(int)car_config::kMotorMaxPwm;
        motor_Duty_L(pwm_l);
        motor_Duty_R(pwm_r);
        // 同步 speed_out 到限幅后的值，防止 I 项在限幅时继续累积
        speed_out_l = (float)pwm_l;
        speed_out_r = (float)pwm_r;

        if (task_remaining_ms > 0 && sine_tick * DT_MS > 1000) {
            float enc_abs_mean = 0.5f * (std::fabs(speed_encoder_L) + std::fabs(speed_encoder_R));
            bool pwm_saturated = (std::fabs((float)pwm_l) > car_config::kMotorMaxPwm * 0.95f ||
                                  std::fabs((float)pwm_r) > car_config::kMotorMaxPwm * 0.95f);
            bool speed_too_low = enc_abs_mean < std::max(0.25f, target_speed * 0.35f);
            if (pwm_saturated && speed_too_low) {
                stuck_guard_ticks++;
            } else {
                stuck_guard_ticks = 0;
            }
            if (stuck_guard_ticks * DT_MS >= 300) {
                printf("[T05_SAFETY_STOP] pwm_saturated=1 speed_too_low=1 t_ms=%d target=%.3f encL=%.3f encR=%.3f pwmL=%d pwmR=%d. 检查是否顶到场地边界、轮胎打滑、供电/驱动限流或机械卡滞。\n",
                       (int)(sine_tick * DT_MS), target_speed, speed_encoder_L, speed_encoder_R, pwm_l, pwm_r);
                task_remaining_ms = 0;
                sine_tick = 0;
                stuck_guard_ticks = 0;
                Speed_Goal = Speed_Goal_l = Speed_Goal_r = 0;
                speed_out_l = speed_out_r = 0;
                speed_last_erro_L = speed_last_erro_R = 0;
                motor_Duty_L(0);
                motor_Duty_R(0);
                show_prompt();
                continue;
            }
        }

        // ==========================================
        // 调参日志：20ms = 50Hz。比控制周期稀疏一半，减少终端打印对速度环的干扰。
        // 格式: T05_ACTUAL t_ms,target,encL,encR,errL,errR,pwmL,pwmR
        // ==========================================
        if (task_remaining_ms > 0 && (sine_tick % LOG_DIVIDER) == 0) {
            int t_ms = (int)(sine_tick * DT_MS);
            float enc_l = speed_encoder_L;
            float enc_r = speed_encoder_R;
            float err_l = target_speed - enc_l;
            float err_r = target_speed - enc_r;
            printf("T05_ACTUAL %d,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
                   t_ms, target_speed, enc_l, enc_r, err_l, err_r, pwm_l, pwm_r);
        }

        usleep(DT_MS * 1000);
        if (task_remaining_ms > 0) sine_tick++;
    }

    // 恢复原始 PID 参数
    speed_P_l = orig_Pl; speed_I_l = orig_Il;
    speed_P_r = orig_Pr; speed_I_r = orig_Ir;
    safe_stop_and_print("T05");
    exit_flag = false;

    printf("[T05_RETURN] 0=测试结束。T05_ACTUAL日志可复制给AI分析。\n");
    printf("[调参意义] 实地速度测试能暴露：低速段跟不跟得上=落地摩擦/死区；高速段跟不跟得上=动力/P增益；下降段拖尾/反向PWM=I累积或P过猛；左右差异=机械/编码器/地面接触差异。\n");
    return 0;
}

// -----------------------------------------------------------------------------
// Step 8/9/12: 低速视觉外环 + 陀螺仪角速度环 + 普通赛道低速闭环。
// 小车可直接放赛道低速跑，打印有效调参指标。
// 返回值：0=低速闭环可用；1=视觉PD饱和；2=陀螺仪跟不上；3=差速限幅/速度跟不上；4=图像不稳。
// -----------------------------------------------------------------------------
int test_06_low_speed_visual_gyro_closed_loop(int seconds = 10) {
    printf("\n========== TEST 06 low_speed_visual_gyro_closed_loop 低速视觉+角速度闭环 ==========" "\n");
    print_stage_card("T06 low_speed_visual_gyro_closed_loop",
                     "在普通赛道低速闭环，联合观察视觉外环、目标角速度、实际角速度、差速限幅和轮速误差。",
                     "kSteerKp/Kd、视觉滤波、目标角速度限幅/变化率、kGyroPidP/D、差速限幅。",
                     "不要回头大改曝光/二值/扫线；若img_bad高，先退回T03/T04，而不是继续加PID。",
                     "低速能稳定跑普通赛道后再做T07提速或T08元素。");
    printf("[运行] 小车放在普通赛道起点，程序会自动低速循迹。建议先留出安全空间，必要时Ctrl+C。seconds=%d\n", seconds);
    if (!ensure_camera_ready()) return -1;
    init_control_for_motor_tests();
    g_enable_element_process = false;

    std::thread ctrl_thread(timer_thread_1);
    RunningStat raw, filt, pd, tgt, cmd, real, gf, gerr, diff, se_l, se_r, dtmax;
    int cap_cnt = 0, imu_bad_cnt = 0, db_cnt = 0, img_bad_cnt = 0, img_warn_cnt = 0, pd_sat_cnt = 0;
    int large_offset_cnt = 0;
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
            bool edge_warn = (ImageStatus.Left_Line > 15 ||
                              ImageStatus.Right_Line > 15 ||
                              ImageStatus.OFFLine > 18 ||
                              std::abs(ImageStatus.MiddleLine - ImageStatus.Det_True) > 26);
            bool severe_img_bad = (ImageStatus.OFFLine > 35 ||
                                   (ImageStatus.Left_Line > 25 && ImageStatus.Right_Line > 25));
            bool unsafe_offset = (std::fabs(dbg_raw_steer_error) > 20.0f ||
                                  std::abs(ImageStatus.MiddleLine - ImageStatus.Det_True) > 20 ||
                                  ImageStatus.Left_Line > 35 ||
                                  ImageStatus.Right_Line > 35);
            if (edge_warn) img_warn_cnt++;
            if (severe_img_bad) img_bad_cnt++;
            if (unsafe_offset) large_offset_cnt++;
            if (std::fabs(dbg_visual_pd_out) > car_config::kVisualGyroLimitDps * 0.90f) pd_sat_cnt++;
            printf("[T06] Raw:%5.1f Filt:%5.1f dE:%5.1f | PD:%6.1f Tgt:%6.1f Cmd:%6.1f Real:%6.1f GF:%6.1f GErr:%6.1f dT:%5.1f | D0:%6.1f D:%6.1f Cap:%d Max:%4.1f | L:%5.1f R:%5.1f Enc:%5.1f/%5.1f SE:%5.1f/%5.1f | imgErr:%d Road:%s OFF:%d L/R:%d/%d dt:%5.3f IMU:%d DB:%d | fc:%d sp:%d dt_max:%.3f rg:%.1f oa:%d tbo:%.1f\n",
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
    double large_offset_rate = total_prints ? (double)large_offset_cnt / total_prints : 0;
    printf("[T06_SUMMARY] samples=%d raw_abs=%.2f filt_abs=%.2f PD_abs=%.2f Tgt_abs=%.2f Real_abs=%.2f GF_abs=%.2f GErr_abs=%.2f Diff_abs=%.2f SE_abs=%.2f/%.2f cap_rate=%.2f pd_sat_rate=%.2f imu_bad=%.2f img_bad=%.2f img_warn=%.2f large_offset=%.2f dtmax_mean=%.3f DB_cnt=%d\n",
           total_prints, raw.mean_abs(), filt.mean_abs(), pd.mean_abs(), tgt.mean_abs(), real.mean_abs(), gf.mean_abs(), gerr.mean_abs(), diff.mean_abs(), se_l.mean_abs(), se_r.mean_abs(), cap_rate, pd_sat_rate, imu_bad_rate, img_bad_rate, img_warn_rate, large_offset_rate, dtmax.mean(), db_cnt);
    printf("[调参意义] pd_sat高=外环/前瞻过猛；GErr_abs>20且持续=角速度环/GF跟不上；cap高=差速受限；img_bad=OFF>35或双边严重丢失；img_warn=急弯大偏差/单侧贴边/短暂OFF；large_offset=Raw/imgErr>20或单侧丢边>35，代表压线风险，不应直接进T07。\n");
    printf("[RETURN] 0=低速闭环可继续；1=PD饱和；2=陀螺仪跟踪差；3=差速/速度受限；4=图像/贴边风险。\n");
    if (img_bad_rate > 0.20 ||
        img_warn_rate > 0.70 ||
        large_offset_rate > 0.20 ||
        (img_warn_rate > 0.45 && raw.mean_abs() > 12.0)) return 4;
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
    print_stage_card("T07 speed_up_regression",
                     "在T06低速稳定基础上验证当前直道/弯道速度是否可保留。",
                     "kStraightSpeedGoal、kCurveSpeedGoal、急弯/S弯保护速度和目标角速度变化率的小幅调整。",
                     "不要在一次提速失败后同时重调图像、速度环、IMU和元素；先按T06指标定位瓶颈。",
                     "普通赛道提速稳定后再进入元素或更高速度回归。");
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
    print_stage_card("T08 element_debug",
                     "只在普通循迹稳定后开启元素，观察Road/Scene/cooldown/圆环确认条件是否抖动或漏检。",
                     "元素确认帧数、丢线门槛、P1/P2几何限制、元素内速度/角速度/差速限幅。",
                     "不要用元素问题反推基础二值、速度环和普通弯PID一起改；若img_bad高先退回T03/T04。",
                     "单个元素稳定后再逐个增加速度或串联多个元素。");
    printf("[运行] 只在普通循迹稳定后调用。开启元素判断，低速经过十字/圆环区域，输出Road_type、Scene、ring/cooldown状态。seconds=%d\n", seconds);
    printf("[CFG_T08] ring_confirm=%d exit_confirm=%d cooldown=%d exit_hold=%d left(Lmin=%d Rmax=%d OFFmax=%d P2<%d) right(Rmin=%d Lmax=%d OFFmax=%d P2>%d) loss_only>=%d ring_cap=%.0f unclassified_limit=%.0f@%.0f。误判多增大confirm/loss_only；漏检先降低对应min或放宽对侧max/OFFmax。\n",
           car_config::kElementRingConfirmFrames,
           car_config::kElementRingExitConfirmFrames,
           car_config::kElementCooldownFrames,
           car_config::kElementRingExitStraightHoldFrames,
           car_config::kLeftRingMinLeftLost,
           car_config::kLeftRingRightLineBlockMax,
           car_config::kLeftRingOfflineBlockMax,
           car_config::kLeftRingMaxPoint2Y,
           car_config::kRightRingMinRightLost,
           car_config::kRightRingLeftLineBlockMax,
           car_config::kRightRingOfflineBlockMax,
           car_config::kRightRingMinPoint2Y,
           car_config::kRingLossOnlyMinLost,
           car_config::kRingSceneGyroLimitDps,
           car_config::kUnclassifiedTurnGyroLimitDps,
           car_config::kUnclassifiedTurnTriggerError);
    init_element_tft_once("T08");
    if (!ensure_camera_ready()) return -1;
    init_control_for_motor_tests();
    g_enable_element_process = true;

    lq_udp_client udp_client;
    if (kElementDebugUdpEnable) {
        udp_client.udp_client_init(kElementDebugTargetIp, kElementDebugTargetPort);
        printf("[T08_UDP] target=%s:%u quality=%u overlay=raw/final_boundary+center+P1/P2+state\n",
               kElementDebugTargetIp.c_str(), kElementDebugTargetPort, kElementDebugJpegQuality);
    } else {
        printf("[T08_UDP] disabled. 使用 TFT18 + 终端日志观察元素补线。\n");
    }

    std::thread ctrl_thread(timer_thread_1);
    int last_road = -999;
    ElementScene_e last_scene = ElementScene_Normal;
    bool has_last_scene = false;
    int changes = 0, scene_changes = 0, special_cnt = 0, scene_special_cnt = 0, cooldown_cnt = 0;
    int unknown_cnt = 0, img_bad_cnt = 0, img_warn_cnt = 0, samples = 0;
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
        if (!has_last_scene) {
            last_scene = g_element_scene;
            has_last_scene = true;
        } else if (g_element_scene != last_scene) {
            printf("[T08_EVENT] t=%.2f SceneChange %s -> %s Road=%s ring=%d/%d Cool=%u Err=%d OFF=%d L/R=%d/%d\n",
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0,
                   element_scene_name(last_scene), element_scene_name(g_element_scene),
                   road_type_name(ImageStatus.Road_type),
                   ImageFlag.image_element_rings, ImageFlag.image_element_rings_flag,
                   g_element_cooldown_frames,
                   ImageStatus.MiddleLine - ImageStatus.Det_True,
                   ImageStatus.OFFLine, ImageStatus.Left_Line, ImageStatus.Right_Line);
            last_scene = g_element_scene;
            scene_changes++;
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 200) {
            last_print = now;
            samples++;
            if (ImageStatus.Road_type == zero) unknown_cnt++;
            if (g_element_scene != ElementScene_Normal) scene_special_cnt++;
            if (g_element_cooldown_frames > 0) cooldown_cnt++;
            bool special_seen = (ImageStatus.Road_type == Cross ||
                                 ImageStatus.Road_type == Cross_ture ||
                                 ImageStatus.Road_type == Ramp ||
                                 ImageStatus.Road_type == LeftCirque ||
                                 ImageStatus.Road_type == RightCirque ||
                                 ImageFlag.image_element_rings != 0 ||
                                 ImageFlag.image_element_rings_flag != 0);
            if (special_seen) special_cnt++;
            bool edge_warn = (ImageStatus.Left_Line > 15 || ImageStatus.Right_Line > 15);
            bool severe_img_bad = (g_element_scene == ElementScene_Normal) &&
                                  (ImageStatus.OFFLine > 18 ||
                                   (ImageStatus.Left_Line > 25 && ImageStatus.Right_Line > 25) ||
                                   std::abs(ImageStatus.MiddleLine - ImageStatus.Det_True) > 26);
            if (edge_warn) img_warn_cnt++;
            if (severe_img_bad) img_bad_cnt++;
            printf("[T08] Road=%s Scene=%s Cool=%u Det=%d Err=%d OFF=%d L/R=%d/%d ring=%d/%d EL=%d L[b/g/l/s/c p1/p2/h]=%d/%d/%d/%d/%d %d/%d/%d R[b/g/l/s/c p1/p2/h]=%d/%d/%d/%d/%d %d/%d/%d lp=%d/%d rp=%d/%d straight=%d/%d D=%.1f Tgt=%.1f GErr=%.1f Enc=%.0f/%.0f\n",
                   road_type_name(ImageStatus.Road_type),
                   element_scene_name(g_element_scene), g_element_cooldown_frames,
                   ImageStatus.Det_True,
                   ImageStatus.MiddleLine - ImageStatus.Det_True, ImageStatus.OFFLine,
                   ImageStatus.Left_Line, ImageStatus.Right_Line,
                   ImageFlag.image_element_rings, ImageFlag.image_element_rings_flag,
                   dbg_el_detect_open,
                   dbg_el_l_block,
                   dbg_el_l_geom,
                   dbg_el_l_loss,
                   dbg_el_l_susp,
                   dbg_el_l_confirm,
                   dbg_el_l_p1,
                   dbg_el_l_p2,
                   dbg_el_l_help,
                   dbg_el_r_block,
                   dbg_el_r_geom,
                   dbg_el_r_loss,
                   dbg_el_r_susp,
                   dbg_el_r_confirm,
                   dbg_el_r_p1,
                   dbg_el_r_p2,
                   dbg_el_r_help,
                   Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite,
                   Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite,
                   ImageStatus.straight_acc, ImageStatus.straight_acc_flag,
                   dbg_diff_speed, dbg_target_gyro_z, dbg_gyro_error,
                   speed_encoder_L, speed_encoder_R);
            if (kElementDebugUdpEnable) {
                cv::Mat display_frame = make_element_debug_frame(samples, changes, scene_changes, img_bad_cnt, img_warn_cnt);
                ssize_t sent = udp_client.udp_send_image(display_frame, kElementDebugJpegQuality);
                if (sent < 0) {
                    printf("[T08_UDP_WARN] send failed\n");
                }
            }
            show_element_tft_frame("T08", samples, changes, scene_changes, img_bad_cnt, img_warn_cnt, true);
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
    double scene_special_rate = samples ? (double)scene_special_cnt / samples : 0.0;
    double cooldown_rate = samples ? (double)cooldown_cnt / samples : 0.0;
    printf("[T08_SUMMARY] samples=%d road_changes=%d scene_changes=%d special_rate=%.2f scene_special_rate=%.2f cooldown_rate=%.2f unknown_rate=%.2f img_bad_rate=%.2f img_warn_rate=%.2f final_scene=%s final_cool=%u\n",
           samples, changes, scene_changes, special_rate, scene_special_rate, cooldown_rate,
           unknown_rate, img_bad_rate, img_warn_rate,
           element_scene_name(g_element_scene), g_element_cooldown_frames);
    printf("[调参意义] road_changes/scene_changes过多=误判或状态抖动；scene_special_rate能看场景锁是否进入元素；cooldown_rate能看出环后保护是否生效；special_rate=0 表示本轮没跑到元素或元素漏检；img_bad=严重丢线/检测贴边。\n");
    printf("[元素参数] kElementRingConfirmFrames 控制入环连续确认；kElementRingExitConfirmFrames 控制出环连续确认；kElementCooldownFrames 控制元素退出后多久不再识别新元素。\n");
    printf("[RETURN] 0=元素状态可继续细调；1=误判频繁；2=疑似漏检；3=图像不稳。\n");
    if (img_bad_rate > 0.20) return 3;
    if (changes > seconds * 2 || scene_changes > seconds) return 1;
    if (special_cnt == 0 && scene_special_cnt == 0) return 2;
    if (changes <= 1 && scene_changes == 0 && unknown_rate < 0.10) return 2;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 08H: 元素手推图传调试。不开闭环、不驱动电机，只进行图像处理/元素判断/补线叠加。
// 用途：手推小车经过左/右环岛，观察 ring=1/5/6/7/8 时补线和中心线是否指向正确。
// 返回值：0=采集完成；2=未看到元素；3=图像不稳/采集失败。
// -----------------------------------------------------------------------------
int test_08_element_push_image_debug(int seconds = 40) {
    printf("\n========== TEST 08H element_push_image_debug 手推元素图传/补线观察 ==========" "\n");
    print_stage_card("T08H element_push_image_debug",
                     "不开电机闭环，只开摄像头、元素判断、TFT18显示和CSV采样；手推小车观察补线/中心线。",
                     "左/右环岛识别阈值、ring状态推进条件、P1/P2特征点、补线中心偏置。",
                     "不要在本测试中调速度环/普通弯PID；本测试目标是先确认图像补线方向是否正确。",
                     "确认 ring=1/5/6/7/8 的补线都正确后，再回到T08自动低速通过。");
    printf("[运行] 手推模式：电机目标和PWM会被强制清零。请看TFT18画面，手扶小车慢慢推过左/右环岛。seconds=%d\n", seconds);
    printf("[观察] 重点截图 ring=1/5 刚入环、ring=1/6 环内、ring=1/7/8 出环阶段；看中心线是否指向环心/出口。\n");
    printf("[采样] 本测试会把每帧元素特征保存成CSV，并在结束时打印自动调参建议。\n");
    printf("[CFG_T08H] left(Lmin=%d Rmax=%d OFFmax=%d P2<%d) right(Rmin=%d Lmax=%d OFFmax=%d P2>%d) entry_bias=%d inner_bias=%d forced=%.1f exit=%.1f\n",
           car_config::kLeftRingMinLeftLost,
           car_config::kLeftRingRightLineBlockMax,
           car_config::kLeftRingOfflineBlockMax,
           car_config::kLeftRingMaxPoint2Y,
           car_config::kRightRingMinRightLost,
           car_config::kRightRingLeftLineBlockMax,
           car_config::kRightRingOfflineBlockMax,
           car_config::kRightRingMinPoint2Y,
           car_config::kRingEntryCenterBias,
           car_config::kRingInnerCenterBias,
           car_config::kRingForcedSteerError,
           car_config::kRingExitSteerError);
    init_element_tft_once("T08H");

    if (!ensure_camera_ready()) return -1;
    init_control_for_motor_tests();
    g_enable_element_process = true;
    Speed_Goal = 0;
    Speed_Goal_l = 0;
    Speed_Goal_r = 0;
    speed_out_l = 0;
    speed_out_r = 0;
    motor_Duty_L(0);
    motor_Duty_R(0);

    lq_udp_client udp_client;
    if (kElementDebugUdpEnable) {
        udp_client.udp_client_init(kElementDebugTargetIp, kElementDebugTargetPort);
        printf("[T08H_UDP] target=%s:%u quality=%u overlay=raw/final_boundary+center+P1/P2+state\n",
               kElementDebugTargetIp.c_str(), kElementDebugTargetPort, kElementDebugJpegQuality);
    } else {
        printf("[T08H_UDP] disabled. 使用 TFT18 + 终端日志观察元素补线。\n");
    }
    ElementCsvLogger csv_logger;
    csv_logger.open("T08H_element_features");
    ElementFeatureStats feature_stats;

    int last_road = -999;
    ElementScene_e last_scene = ElementScene_Normal;
    bool has_last_scene = false;
    int changes = 0, scene_changes = 0, special_cnt = 0, img_bad_cnt = 0, img_warn_cnt = 0, samples = 0;
    auto start = std::chrono::steady_clock::now();
    auto last_print = start;
    while (!exit_flag) {
        Speed_Goal = 0;
        Speed_Goal_l = 0;
        Speed_Goal_r = 0;
        speed_out_l = 0;
        speed_out_r = 0;
        motor_Duty_L(0);
        motor_Duty_R(0);

        if (!grab_and_process()) { img_bad_cnt++; break; }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= seconds) break;

        if (ImageStatus.Road_type != last_road) {
            printf("[T08H_EVENT] t=%.2f RoadChange %s -> %s Det=%d Err=%d OFF=%d L/R=%d/%d straight=%d/%d\n",
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0,
                   road_type_name(last_road), road_type_name(ImageStatus.Road_type),
                   ImageStatus.Det_True, ImageStatus.MiddleLine - ImageStatus.Det_True,
                   ImageStatus.OFFLine, ImageStatus.Left_Line, ImageStatus.Right_Line,
                   ImageStatus.straight_acc, ImageStatus.straight_acc_flag);
            last_road = ImageStatus.Road_type;
            changes++;
        }
        if (!has_last_scene) {
            last_scene = g_element_scene;
            has_last_scene = true;
        } else if (g_element_scene != last_scene) {
            printf("[T08H_EVENT] t=%.2f SceneChange %s -> %s Road=%s ring=%d/%d Cool=%u Err=%d OFF=%d L/R=%d/%d\n",
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0,
                   element_scene_name(last_scene), element_scene_name(g_element_scene),
                   road_type_name(ImageStatus.Road_type),
                   ImageFlag.image_element_rings, ImageFlag.image_element_rings_flag,
                   g_element_cooldown_frames,
                   ImageStatus.MiddleLine - ImageStatus.Det_True,
                   ImageStatus.OFFLine, ImageStatus.Left_Line, ImageStatus.Right_Line);
            last_scene = g_element_scene;
            scene_changes++;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 120) {
            last_print = now;
            samples++;
            bool special_seen = (ImageStatus.Road_type == Cross ||
                                 ImageStatus.Road_type == Cross_ture ||
                                 ImageStatus.Road_type == Ramp ||
                                 ImageStatus.Road_type == LeftCirque ||
                                 ImageStatus.Road_type == RightCirque ||
                                 ImageFlag.image_element_rings != 0 ||
                                 ImageFlag.image_element_rings_flag != 0);
            if (special_seen) special_cnt++;
            bool edge_warn = (ImageStatus.Left_Line > 15 || ImageStatus.Right_Line > 15);
            bool severe_img_bad = (g_element_scene == ElementScene_Normal) &&
                                  (ImageStatus.OFFLine > 18 ||
                                   (ImageStatus.Left_Line > 25 && ImageStatus.Right_Line > 25) ||
                                   std::abs(ImageStatus.MiddleLine - ImageStatus.Det_True) > 26);
            if (edge_warn) img_warn_cnt++;
            if (severe_img_bad) img_bad_cnt++;
            feature_stats.add();
            double t_sec = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
            csv_logger.write(t_sec, samples, false, severe_img_bad ? 1 : 0, edge_warn ? 1 : 0);
            printf("[T08H] Road=%s Scene=%s Cool=%u Det=%d Err=%d OFF=%d L/R=%d/%d ring=%d/%d EL=%d L[b/g/l/s/c p1/p2/h]=%d/%d/%d/%d/%d %d/%d/%d R[b/g/l/s/c p1/p2/h]=%d/%d/%d/%d/%d %d/%d/%d lp=%d/%d rp=%d/%d straight=%d/%d MOTOR=OFF\n",
                   road_type_name(ImageStatus.Road_type),
                   element_scene_name(g_element_scene), g_element_cooldown_frames,
                   ImageStatus.Det_True,
                   ImageStatus.MiddleLine - ImageStatus.Det_True, ImageStatus.OFFLine,
                   ImageStatus.Left_Line, ImageStatus.Right_Line,
                   ImageFlag.image_element_rings, ImageFlag.image_element_rings_flag,
                   dbg_el_detect_open,
                   dbg_el_l_block,
                   dbg_el_l_geom,
                   dbg_el_l_loss,
                   dbg_el_l_susp,
                   dbg_el_l_confirm,
                   dbg_el_l_p1,
                   dbg_el_l_p2,
                   dbg_el_l_help,
                   dbg_el_r_block,
                   dbg_el_r_geom,
                   dbg_el_r_loss,
                   dbg_el_r_susp,
                   dbg_el_r_confirm,
                   dbg_el_r_p1,
                   dbg_el_r_p2,
                   dbg_el_r_help,
                   Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite,
                   Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite,
                   ImageStatus.straight_acc, ImageStatus.straight_acc_flag);
            if (kElementDebugUdpEnable) {
                cv::Mat display_frame = make_element_debug_frame(samples, changes, scene_changes, img_bad_cnt, img_warn_cnt);
                ssize_t sent = udp_client.udp_send_image(display_frame, kElementDebugJpegQuality);
                if (sent < 0) {
                    printf("[T08H_UDP_WARN] send failed\n");
                }
            }
            show_element_tft_frame("T08H", samples, changes, scene_changes, img_bad_cnt, img_warn_cnt, false);
        }
        usleep(5 * 1000);
    }
    safe_stop_and_print("T08H");
    exit_flag = false;
    csv_logger.close();
    double img_bad_rate = samples ? (double)img_bad_cnt / samples : 0.0;
    double img_warn_rate = samples ? (double)img_warn_cnt / samples : 0.0;
    double special_rate = samples ? (double)special_cnt / samples : 0.0;
    printf("[T08H_SUMMARY] samples=%d road_changes=%d scene_changes=%d special_rate=%.2f img_bad_rate=%.2f img_warn_rate=%.2f final_scene=%s final_ring=%d/%d cool=%u\n",
           samples, changes, scene_changes, special_rate, img_bad_rate, img_warn_rate,
           element_scene_name(g_element_scene), ImageFlag.image_element_rings,
           ImageFlag.image_element_rings_flag, g_element_cooldown_frames);
    feature_stats.print_recommendation("T08H", csv_logger.path, special_rate, img_bad_rate, img_warn_rate);
    printf("[T08H调参意义] 只看图像补线：ring=1/5中心线不进环=>调入口补线/偏置；ring=1/7/8仍指环内=>调出环补线；图像正确但自动跑不进=>再调控制差速。\n");
    printf("[RETURN] 0=采集完成；2=未看到元素；3=图像不稳。\n");
    if (img_bad_rate > 0.25) return 3;
    if (special_cnt == 0) return 2;
    return 0;
}

// -----------------------------------------------------------------------------
// TEST 08P: 手推车 TFT 实时补线观察（带FPS显示）— 电机关闭，纯视觉验证
// -----------------------------------------------------------------------------
int test_08p_push_line_patrol_tft(int seconds = 120) {
    printf("\n========== TEST 08P push_line_patrol_tft 手推补线TFT观察 ==========\n");
    print_stage_card("T08P push_line_patrol_tft",
                     "不开电机闭环，只手推小车，TFT18 逐帧显示二值图+原始/最终边线+中线+实时FPS。",
                     "摄像头实际帧率、补线质量（直道/弯道中线走向）、边界稳定性。",
                     "不要在本测试中开电机；本测试目标是在零风险下手推观察补线是否正确。",
                     "手推通过直道和各曲率弯道，重点看TFT上红线(中线)是否在该行左右边界的正中间。");
    printf("[运行] 手推模式：电机强制关闭。TFT18 实时显示补线画面+FPS。seconds=%d\n", seconds);
    printf("[观察] 直道：左右边界平行、中线居中；弯道：补线边界连续、无大片OFF；FPS>=140 则正常。\n");

    init_element_tft_once("T08P");
    if (!ensure_camera_ready()) return -1;
    init_control_for_motor_tests();
    g_enable_element_process = false;
    Speed_Goal = 0;
    Speed_Goal_l = 0;
    Speed_Goal_r = 0;
    speed_out_l = 0;
    speed_out_r = 0;
    motor_Duty_L(0);
    motor_Duty_R(0);

    auto start = std::chrono::steady_clock::now();
    auto last_fps_tick = start;
    int frame_count = 0;
    int fps_sample_count = 0;
    float fps = 0.0f;
    int img_bad_cnt = 0, samples = 0;

    while (!exit_flag) {
        Speed_Goal = 0;
        Speed_Goal_l = 0;
        Speed_Goal_r = 0;
        speed_out_l = 0;
        speed_out_r = 0;
        motor_Duty_L(0);
        motor_Duty_R(0);

        if (!grab_and_process()) { img_bad_cnt++; break; }

        frame_count++;
        fps_sample_count++;

        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_tick).count();
        if (elapsed_ms >= 500) {
            fps = fps_sample_count * 1000.0f / elapsed_ms;
            fps_sample_count = 0;
            last_fps_tick = now;
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= seconds) break;

        // 每帧都刷新 TFT
        cv::Mat tft_frame = make_push_tft_frame(fps, frame_count);
        lq_tft18_drv_road_color(kElementDebugTftX, kElementDebugTftY, tft_frame);

        samples++;
        bool severe_img_bad = (ImageStatus.OFFLine > 18 ||
                               (ImageStatus.Left_Line > 25 && ImageStatus.Right_Line > 25) ||
                               std::abs(ImageStatus.MiddleLine - ImageStatus.Det_True) > 26);
        if (severe_img_bad) img_bad_cnt++;

        usleep(2 * 1000);
    }

    safe_stop_and_print("T08P");
    exit_flag = false;

    double img_bad_rate = samples ? (double)img_bad_cnt / samples : 0.0;
    printf("[T08P_SUMMARY] frames=%d fps=%.1f img_bad_rate=%.2f\n", frame_count, fps, img_bad_rate);
    printf("[T08P调参意义] 观察TFT上中线位置：若弯道中线偏外/出界→调大弯补偿或视觉Kp；若FPS偏低→检查曝光/ROI。\n");
    printf("[RETURN] 0=采集完成；3=图像质量不稳。\n");
    if (img_bad_rate > 0.25) return 3;
    return 0;
}

struct PoseWindowSnapshot {
    int valid_rows = 0;
    int total_rows = 0;
    double width_mean = 0.0;
    double width_min = 0.0;
    double left_margin_min = 0.0;
    double right_margin_min = 0.0;
    double balance_mean = 0.0;
};

static bool valid_pose_row(int y) {
    if (y < 0 || y >= 60) return false;
    int left = ImageDeal[y].LeftBorder;
    int right = ImageDeal[y].RightBorder;
    int center = ImageDeal[y].Center;
    return left >= 0 && left < 80 &&
           right >= 0 && right < 80 &&
           center >= 0 && center < 80 &&
           right > left &&
           center > left &&
           center < right;
}

static PoseWindowSnapshot calc_pose_window(int y0, int y1) {
    PoseWindowSnapshot s;
    RunningStat width;
    RunningStat left_margin;
    RunningStat right_margin;
    RunningStat balance;
    if (y0 > y1) std::swap(y0, y1);
    y0 = std::max(0, y0);
    y1 = std::min(59, y1);
    s.total_rows = y1 - y0 + 1;
    for (int y = y0; y <= y1; ++y) {
        if (!valid_pose_row(y)) continue;
        int left = ImageDeal[y].LeftBorder;
        int right = ImageDeal[y].RightBorder;
        int center = ImageDeal[y].Center;
        int lm = center - left;
        int rm = right - center;
        width.add(right - left);
        left_margin.add(lm);
        right_margin.add(rm);
        balance.add(rm - lm);
        s.valid_rows++;
    }
    s.width_mean = width.mean();
    s.width_min = width.n ? width.min_v : 0.0;
    s.left_margin_min = left_margin.n ? left_margin.min_v : 0.0;
    s.right_margin_min = right_margin.n ? right_margin.min_v : 0.0;
    s.balance_mean = balance.mean();
    return s;
}

static double pose_valid_rate(const PoseWindowSnapshot &s) {
    return s.total_rows ? (double)s.valid_rows / s.total_rows : 0.0;
}

struct PoseReferenceCsvLogger {
    FILE *file = nullptr;
    std::string path;

    bool open() {
        path = make_artifact_path("T13_pose_reference_push", "csv");
        file = fopen(path.c_str(), "w");
        if (!file) {
            printf("[CSV_ERROR] cannot open %s\n", path.c_str());
            return false;
        }
        fprintf(file,
                "t_sec,sample,road,det,mid,err,tow,off,left_miss,right_miss,"
                "straight,straight_flag,white,width_near,"
                "near_valid,near_width_mean,near_width_min,near_left_margin_min,near_right_margin_min,near_balance,"
                "mid_valid,mid_width_mean,mid_width_min,mid_left_margin_min,mid_right_margin_min,mid_balance,"
                "far_valid,far_width_mean,far_width_min,far_left_margin_min,far_right_margin_min,far_balance,"
                "det_delta,err_delta,tow_delta,img_bad,img_warn\n");
        fflush(file);
        printf("[CSV] 手推视觉基准样本保存到: %s\n", path.c_str());
        return true;
    }

    void write(double t_sec,
               int sample,
               const PoseWindowSnapshot &near_w,
               const PoseWindowSnapshot &mid_w,
               const PoseWindowSnapshot &far_w,
               int det_delta,
               int err_delta,
               int tow_delta,
               int img_bad,
               int img_warn) {
        if (!file) return;
        fprintf(file,
                "%.3f,%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%d,"
                "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                "%d,%d,%d,%d,%d\n",
                t_sec, sample, road_type_name(ImageStatus.Road_type),
                ImageStatus.Det_True, ImageStatus.MiddleLine,
                ImageStatus.MiddleLine - ImageStatus.Det_True,
                ImageStatus.TowPoint, ImageStatus.OFFLine,
                ImageStatus.Left_Line, ImageStatus.Right_Line,
                ImageStatus.straight_acc, ImageStatus.straight_acc_flag,
                white_ratio_all(), estimate_near_line_width(),
                pose_valid_rate(near_w), near_w.width_mean, near_w.width_min,
                near_w.left_margin_min, near_w.right_margin_min, near_w.balance_mean,
                pose_valid_rate(mid_w), mid_w.width_mean, mid_w.width_min,
                mid_w.left_margin_min, mid_w.right_margin_min, mid_w.balance_mean,
                pose_valid_rate(far_w), far_w.width_mean, far_w.width_min,
                far_w.left_margin_min, far_w.right_margin_min, far_w.balance_mean,
                det_delta, err_delta, tow_delta, img_bad, img_warn);
    }

    void close() {
        if (!file) return;
        fflush(file);
        fclose(file);
        file = nullptr;
    }
};

// -----------------------------------------------------------------------------
// Step 13: 正常路径手推视觉基准。不开电机，只建立正常车位下的边界/余量参考。
// 返回值：0=采样可用；1=样本太少；2=边界覆盖不足；3=安全余量偏低；4=图像质量不稳。
// -----------------------------------------------------------------------------
int test_13_pose_reference_push(int seconds = 45) {
    printf("\n========== TEST 13 pose_reference_push 手推正常路径视觉基准 ==========" "\n");
    print_stage_card("T13 pose_reference_push",
                     "不开电机闭环，只手推小车沿正常路径采集边界覆盖、道路宽度、左右安全余量和长期偏侧。",
                     "摄像头安装、中线、ROI/扫线、基础姿态安全余量；为后续T06/T07/T08判定压线风险提供基准。",
                     "不要用本测试直接调速度环/PID；det_delta、err_delta、tow_delta受手推速度影响，只作为REF_ONLY参考。",
                     "采样可用后，用CSV里的margin/coverage基准去约束实跑日志：安全余量不足时不能判为正常。");
    printf("[运行] MOTOR=OFF。请把车放在你认为正常的车身位置，沿普通赛道路段慢慢手推；尽量覆盖直道、普通弯、容易压线的位置。seconds=%d\n", seconds);
    printf("[判定] 重点看 near/mid/far 三段 valid_rate、width_min、left/right_margin_min、balance；跳变类仅标记REF_ONLY。\n");
    printf("[硬门槛] 如果 near/mid 边界覆盖不足或任一侧安全余量长期偏低，后续AI不能再把该状态判为正常可接受。\n");

    if (!ensure_camera_ready()) return -1;
    init_control_for_motor_tests();
    g_enable_element_process = false;
    safe_stop_and_print("T13");

    PoseReferenceCsvLogger csv;
    csv.open();

    RunningStat err_stat, near_valid, mid_valid, far_valid;
    RunningStat near_width, mid_width, far_width;
    RunningStat near_l_margin, near_r_margin, mid_l_margin, mid_r_margin;
    RunningStat near_balance, mid_balance, far_balance;
    RunningStat det_delta_ref, err_delta_ref, tow_delta_ref;
    int samples = 0, img_bad_cnt = 0, img_warn_cnt = 0;
    int low_margin_cnt = 0, low_valid_cnt = 0;
    int last_det = 0, last_err = 0, last_tow = 0;
    bool has_last = false;

    auto start = std::chrono::steady_clock::now();
    auto last_print = start;
    while (!exit_flag) {
        Speed_Goal = 0;
        Speed_Goal_l = 0;
        Speed_Goal_r = 0;
        speed_out_l = 0;
        speed_out_r = 0;
        motor_Duty_L(0);
        motor_Duty_R(0);
        if (!grab_and_process()) {
            img_bad_cnt++;
            break;
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= seconds) break;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() < 150) {
            usleep(5 * 1000);
            continue;
        }
        last_print = now;
        samples++;

        PoseWindowSnapshot near_w = calc_pose_window(42, 58);
        PoseWindowSnapshot mid_w = calc_pose_window(25, 41);
        PoseWindowSnapshot far_w = calc_pose_window(12, 24);
        int err = ImageStatus.MiddleLine - ImageStatus.Det_True;
        int det_delta = has_last ? ImageStatus.Det_True - last_det : 0;
        int err_delta = has_last ? err - last_err : 0;
        int tow_delta = has_last ? ImageStatus.TowPoint - last_tow : 0;
        has_last = true;
        last_det = ImageStatus.Det_True;
        last_err = err;
        last_tow = ImageStatus.TowPoint;

        double nv = pose_valid_rate(near_w);
        double mv = pose_valid_rate(mid_w);
        double fv = pose_valid_rate(far_w);
        bool edge_warn = (ImageStatus.Left_Line > 15 || ImageStatus.Right_Line > 15 || nv < 0.55 || mv < 0.45);
        bool severe_img_bad = (ImageStatus.OFFLine > 18 ||
                               (ImageStatus.Left_Line > 25 && ImageStatus.Right_Line > 25) ||
                               nv < 0.35 ||
                               mv < 0.25);
        bool low_margin = (near_w.valid_rows > 0 &&
                           (near_w.left_margin_min < 7.0 || near_w.right_margin_min < 7.0));
        bool low_valid = (nv < 0.55 || mv < 0.45);
        if (edge_warn) img_warn_cnt++;
        if (severe_img_bad) img_bad_cnt++;
        if (low_margin) low_margin_cnt++;
        if (low_valid) low_valid_cnt++;

        err_stat.add(err);
        near_valid.add(nv);
        mid_valid.add(mv);
        far_valid.add(fv);
        if (near_w.valid_rows > 0) {
            near_width.add(near_w.width_mean);
            near_l_margin.add(near_w.left_margin_min);
            near_r_margin.add(near_w.right_margin_min);
            near_balance.add(near_w.balance_mean);
        }
        if (mid_w.valid_rows > 0) {
            mid_width.add(mid_w.width_mean);
            mid_l_margin.add(mid_w.left_margin_min);
            mid_r_margin.add(mid_w.right_margin_min);
            mid_balance.add(mid_w.balance_mean);
        }
        if (far_w.valid_rows > 0) {
            far_width.add(far_w.width_mean);
            far_balance.add(far_w.balance_mean);
        }
        det_delta_ref.add(std::abs(det_delta));
        err_delta_ref.add(std::abs(err_delta));
        tow_delta_ref.add(std::abs(tow_delta));

        double t_sec = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        csv.write(t_sec, samples, near_w, mid_w, far_w,
                  det_delta, err_delta, tow_delta,
                  severe_img_bad ? 1 : 0, edge_warn ? 1 : 0);
        printf("[T13] s=%d Road=%s Det=%d Err=%d OFF=%d L/R=%d/%d near(v=%.2f w=%.1f minL/R=%.1f/%.1f bal=%.1f) mid(v=%.2f w=%.1f minL/R=%.1f/%.1f bal=%.1f) far(v=%.2f w=%.1f bal=%.1f) REF_ONLY dDet/dErr/dTow=%d/%d/%d MOTOR=OFF\n",
               samples, road_type_name(ImageStatus.Road_type),
               ImageStatus.Det_True, err, ImageStatus.OFFLine,
               ImageStatus.Left_Line, ImageStatus.Right_Line,
               nv, near_w.width_mean, near_w.left_margin_min, near_w.right_margin_min, near_w.balance_mean,
               mv, mid_w.width_mean, mid_w.left_margin_min, mid_w.right_margin_min, mid_w.balance_mean,
               fv, far_w.width_mean, far_w.balance_mean,
               det_delta, err_delta, tow_delta);
        usleep(5 * 1000);
    }

    safe_stop_and_print("T13");
    exit_flag = false;
    csv.close();

    double img_bad_rate = samples ? (double)img_bad_cnt / samples : 0.0;
    double img_warn_rate = samples ? (double)img_warn_cnt / samples : 0.0;
    double low_margin_rate = samples ? (double)low_margin_cnt / samples : 0.0;
    double low_valid_rate = samples ? (double)low_valid_cnt / samples : 0.0;
    printf("[T13_SUMMARY] csv=%s samples=%d img_bad_rate=%.2f img_warn_rate=%.2f low_valid_rate=%.2f low_margin_rate=%.2f err_mean=%.1f err_abs=%.1f err_std=%.1f\n",
           csv.path.c_str(), samples, img_bad_rate, img_warn_rate,
           low_valid_rate, low_margin_rate,
           err_stat.mean(), err_stat.mean_abs(), err_stat.stddev());
    printf("[T13_BASELINE] valid_mean near/mid/far=%.2f/%.2f/%.2f width_mean near/mid/far=%.1f/%.1f/%.1f\n",
           near_valid.mean(), mid_valid.mean(), far_valid.mean(),
           near_width.mean(), mid_width.mean(), far_width.mean());
    printf("[T13_MARGIN] near minL/minR mean=%.1f/%.1f min=%.1f/%.1f balance mean/std=%.1f/%.1f | mid minL/minR mean=%.1f/%.1f min=%.1f/%.1f balance mean/std=%.1f/%.1f\n",
           near_l_margin.mean(), near_r_margin.mean(),
           near_l_margin.n ? near_l_margin.min_v : 0.0,
           near_r_margin.n ? near_r_margin.min_v : 0.0,
           near_balance.mean(), near_balance.stddev(),
           mid_l_margin.mean(), mid_r_margin.mean(),
           mid_l_margin.n ? mid_l_margin.min_v : 0.0,
           mid_r_margin.n ? mid_r_margin.min_v : 0.0,
           mid_balance.mean(), mid_balance.stddev());
    printf("[T13_REF_ONLY] 手推不匀速，以下跳变只看异常尖峰，不作为动态调参判据：abs(dDet/dErr/dTow) mean=%.1f/%.1f/%.1f max=%.1f/%.1f/%.1f\n",
           det_delta_ref.mean(), err_delta_ref.mean(), tow_delta_ref.mean(),
           det_delta_ref.n ? det_delta_ref.max_v : 0.0,
           err_delta_ref.n ? err_delta_ref.max_v : 0.0,
           tow_delta_ref.n ? tow_delta_ref.max_v : 0.0);
    printf("[T13_RECOMMEND] ");
    if (samples < 30) {
        printf("样本太少：建议手推45-60秒，覆盖直道、普通弯和容易压线的位置。\n");
    } else if (img_bad_rate > 0.25) {
        printf("图像质量不稳：先回T03/T04修曝光/ROI/扫线，再用本测试建基准。\n");
    } else if (low_valid_rate > 0.30) {
        printf("边界覆盖不足：不要进入PID/提速判断，先查摄像头角度、ROI、扫线和二值质量。\n");
    } else if (low_margin_rate > 0.20) {
        printf("正常手推都出现低安全余量：优先检查摄像头中线/安装角度/车身摆位；后续实跑遇到该余量必须判不可接受。\n");
    } else {
        printf("手推视觉基准可用：后续T06/T07/T08若margin低于本轮near/mid低位范围，即使Err/Det正常也判为压线风险。\n");
    }
    printf("[RETURN] 0=采样可用；1=样本太少；2=边界覆盖不足；3=安全余量偏低；4=图像质量不稳。\n");
    if (samples < 30) return 1;
    if (img_bad_rate > 0.25) return 4;
    if (low_valid_rate > 0.30) return 2;
    if (low_margin_rate > 0.20) return 3;
    return 0;
}

// -----------------------------------------------------------------------------
// Step 20: 不同电压/光照回归。只采集图像和关键控制，不强制解释。
// -----------------------------------------------------------------------------
int test_09_voltage_light_regression(int frames = 200) {
    printf("\n========== TEST 09 voltage_light_regression 电压/光照回归 ==========" "\n");
    print_stage_card("T09 voltage_light_regression",
                     "换电池电压或光照后，只回归图像稳定性和摄像头角度，不直接重调控制环。",
                     "曝光、二值化、ROI基础质量；必要时只做最小幅度补偿。",
                     "不要因为一次光照变化把PID、速度、元素参数全部推倒重来。",
                     "回归通过后沿用原控制参数；不通过时按T02/T03返回码处理。");
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
    print_stage_card("T10 gyro_static_noise",
                     "静止采样MPU6050，确认零漂、噪声、尖峰、采样dt是否健康。",
                     "kGyroDeadBandDps、kGyroLpfAlpha、I2C/供电/接地/线材问题。",
                     "不要靠增大角速度PID去掩盖静止尖峰；IMU不过不要跑T06角速度闭环。",
                     "IMU静态健康后再进入T06低速视觉+角速度闭环。");
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

// -----------------------------------------------------------------------------
// Step 11: 角速度环阶跃响应测试。绕开视觉外环，直接给角速度环目标。
// 小车需要放在普通地面/赛道空旷处低速运行，用于区分“外环没给够目标”
// 和“角速度内环/差速/轮速跟不上”。
// 返回值：0=角速度环可用；1=差速限幅瓶颈；2=角速度跟踪弱；3=轮速跟不上；4=IMU读取失败；5=方向响应异常。
// -----------------------------------------------------------------------------
int test_11_gyro_loop_step_response(float base_speed = 0.5f, int hold_ms = 900) {
    printf("\n========== TEST 11 gyro_loop_step_response 角速度环阶跃响应 ==========" "\n");
    print_stage_card("T11 gyro_loop_step_response",
                     "绕开图像外环，直接给角速度环左右阶跃目标，判断角速度环、差速限幅、轮速环谁在限制转向力度。",
                     "kGyroPidP/I/D、kGyroFeedforwardGain、kDiffOutputMaxDelta、kGyroPidOutputLimit、kGyroTargetPositive/NegativeGain，以及差速限幅。",
                     "不要在本测试里改摄像头中线、视觉Kp/Kd、元素补线或提速参数；本测试只判断角速度环和差速执行能力。",
                     "T11左右响应都健康后，再回到T06/T07判断视觉外环和提速回归。");
    printf("[安全] 小车会低速左右摆动，不使用摄像头。请放在普通地面/赛道空旷处，手边准备Ctrl+C。base_speed=%.1f hold_ms=%d\n",
           base_speed, hold_ms);
    printf("[参数快照] gyro(P/I/D=%.3f/%.4f/%.4f ff=%.3f out=%.1f dOut=%.1f/%.1f pos/neg=%.2f/%.2f dead=%.1f lpf=%.2f maxDiffRatio=%.2f)\n",
           car_config::kGyroPidP, car_config::kGyroPidI, car_config::kGyroPidD,
           car_config::kGyroFeedforwardGain, car_config::kGyroPidOutputLimit,
           car_config::kDiffOutputMaxDelta, car_config::kDiffOutputReverseMaxDelta,
           car_config::kGyroTargetPositiveGain, car_config::kGyroTargetNegativeGain,
           car_config::kGyroDeadBandDps, car_config::kGyroLpfAlpha,
           car_config::kMaxDiffRatio);

    g_enable_element_process = false;
    init_control_for_motor_tests();
    safe_stop_and_print("T11");
    usleep(300 * 1000);

    MyMpu6050 local_imu;
    if (!local_imu.init()) {
        printf("[T11_SUMMARY] init_failed=1\n");
        printf("[RETURN] 4=MPU6050初始化失败。\n");
        return 4;
    }

    AttitudeCascade local_cascade;
    local_cascade.init();
    local_cascade.reset_runtime_state();

    speed_out_l = 0.0f;
    speed_out_r = 0.0f;
    speed_last_erro_L = 0.0f;
    speed_last_erro_R = 0.0f;

    struct StepSummary {
        float target = 0.0f;
        float cmd = 0.0f;
        int samples = 0;
        int read_fail = 0;
        int cap_cnt = 0;
        int reverse_cnt = 0;
        RunningStat real;
        RunningStat gf;
        RunningStat err;
        RunningStat late_err;
        RunningStat diff;
        RunningStat speed_l;
        RunningStat speed_r;
        RunningStat se_l;
        RunningStat se_r;
    };

    const float targets[] = {0.0f, 35.0f, 70.0f, 0.0f, -35.0f, -70.0f, 0.0f};
    const int target_count = sizeof(targets) / sizeof(targets[0]);
    std::vector<StepSummary> summaries(target_count);
    int total_samples = 0;
    int total_read_fail = 0;
    int total_cap = 0;
    int total_reverse = 0;
    int print_divider = 0;
    auto last_time = std::chrono::steady_clock::now();

    for (int s = 0; s < target_count && !exit_flag; ++s) {
        StepSummary &st = summaries[s];
        st.target = targets[s];
        st.cmd = st.target;
        if (st.cmd > 0.0f) st.cmd *= car_config::kGyroTargetPositiveGain;
        if (st.cmd < 0.0f) st.cmd *= car_config::kGyroTargetNegativeGain;

        printf("[T11_STEP_BEGIN] idx=%d/%d target=%.1f cmd=%.1f hold_ms=%d\n",
               s + 1, target_count, st.target, st.cmd, hold_ms);

        auto step_start = std::chrono::steady_clock::now();
        while (!exit_flag) {
            auto now = std::chrono::steady_clock::now();
            int elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - step_start).count();
            if (elapsed_ms >= hold_ms) break;

            std::chrono::duration<float> dt_span = now - last_time;
            float dt = dt_span.count();
            last_time = now;
            if (dt <= 0.0001f || dt > 0.1f) dt = car_config::kOuterLoopFallbackPeriodSec;

            Mpu6050Data data;
            if (!local_imu.read_processed_data(data)) {
                st.read_fail++;
                total_read_fail++;
                usleep(5 * 1000);
                continue;
            }

            float diff_speed = local_cascade.calc_diff_speed_bypass_angle(st.cmd, data, dt);
            float diff_before_limit = diff_speed;
            float max_diff = std::max(base_speed * car_config::kMaxDiffRatio, 6.0f);
            int cap = 0;
            if (diff_speed > max_diff) {
                diff_speed = max_diff;
                cap = 1;
            }
            if (diff_speed < -max_diff) {
                diff_speed = -max_diff;
                cap = 1;
            }

            Speed_Goal = base_speed;
            Speed_Goal_l = base_speed - diff_speed;
            Speed_Goal_r = base_speed + diff_speed;
            if (Speed_Goal_l < car_config::kMinWheelSpeedGoal) Speed_Goal_l = car_config::kMinWheelSpeedGoal;
            if (Speed_Goal_r < car_config::kMinWheelSpeedGoal) Speed_Goal_r = car_config::kMinWheelSpeedGoal;
            if (Speed_Goal_l > car_config::kMaxWheelSpeedGoal) Speed_Goal_l = car_config::kMaxWheelSpeedGoal;
            if (Speed_Goal_r > car_config::kMaxWheelSpeedGoal) Speed_Goal_r = car_config::kMaxWheelSpeedGoal;

            speed_encoder_L = encoder_num_L();
            speed_encoder_R = encoder_num_R();
            Speed_PID_L();
            Speed_PID_R();
            motor_Duty_L(speed_out_l);
            motor_Duty_R(speed_out_r);

            const float gf = local_cascade.last_filtered_gyro_z();
            const float gerr = local_cascade.gyro_pid.err;
            const bool reverse = (std::fabs(st.cmd) > 20.0f &&
                                  st.cmd * gf < -car_config::kGyroDeadBandDps);

            st.samples++;
            total_samples++;
            if (cap) {
                st.cap_cnt++;
                total_cap++;
            }
            if (reverse) {
                st.reverse_cnt++;
                total_reverse++;
            }
            st.real.add(data.gyro_z);
            st.gf.add(gf);
            st.err.add(gerr);
            st.diff.add(diff_speed);
            st.speed_l.add(speed_encoder_L);
            st.speed_r.add(speed_encoder_R);
            st.se_l.add(Speed_Goal_l - speed_encoder_L);
            st.se_r.add(Speed_Goal_r - speed_encoder_R);
            if (elapsed_ms > hold_ms / 3) st.late_err.add(gerr);

            if (++print_divider >= 12) {
                print_divider = 0;
                printf("[T11] idx=%d t=%.2f target=%6.1f cmd=%6.1f real=%6.1f gf=%6.1f err=%6.1f diff0=%6.1f diff=%6.1f cap=%d max=%4.1f goalL/R=%5.1f/%5.1f encL/R=%5.1f/%5.1f se=%5.1f/%5.1f out=%5.0f/%5.0f\n",
                       s + 1, elapsed_ms / 1000.0,
                       st.target, st.cmd, data.gyro_z, gf, gerr,
                       diff_before_limit, diff_speed, cap, max_diff,
                       Speed_Goal_l, Speed_Goal_r, speed_encoder_L, speed_encoder_R,
                       Speed_Goal_l - speed_encoder_L, Speed_Goal_r - speed_encoder_R,
                       speed_out_l, speed_out_r);
            }

            usleep(5 * 1000);
        }
    }

    safe_stop_and_print("T11");
    Speed_Goal = 0;

    double total_cap_rate = total_samples ? (double)total_cap / total_samples : 0.0;
    double total_fail_rate = (total_samples + total_read_fail) ?
                             (double)total_read_fail / (total_samples + total_read_fail) : 0.0;
    double total_reverse_rate = total_samples ? (double)total_reverse / total_samples : 0.0;
    int weak_track_steps = 0;
    int cap_steps = 0;
    int speed_weak_steps = 0;
    int reverse_steps = 0;

    for (int s = 0; s < target_count; ++s) {
        const StepSummary &st = summaries[s];
        if (st.samples == 0) continue;
        double cap_rate = (double)st.cap_cnt / st.samples;
        double reverse_rate = (double)st.reverse_cnt / st.samples;
        double late_abs_err = st.late_err.mean_abs();
        double speed_err_abs = std::max(st.se_l.mean_abs(), st.se_r.mean_abs());
        double target_abs = std::fabs(st.cmd);
        bool active_step = target_abs > 1.0;
        if (active_step && cap_rate > 0.35) cap_steps++;
        if (active_step && cap_rate <= 0.35 && late_abs_err > std::max(18.0, target_abs * 0.45)) weak_track_steps++;
        if (active_step && speed_err_abs > 6.0) speed_weak_steps++;
        if (active_step && reverse_rate > 0.20) reverse_steps++;
        printf("[T11_STEP_SUMMARY] idx=%d target=%.1f cmd=%.1f samples=%d read_fail=%d gf_mean=%.2f gf_abs=%.2f real_abs=%.2f err_abs=%.2f late_err_abs=%.2f diff_abs=%.2f cap_rate=%.2f reverse_rate=%.2f speed_err_abs=%.2f/%2.2f enc_abs=%.2f/%2.2f\n",
               s + 1, st.target, st.cmd, st.samples, st.read_fail,
               st.gf.mean(), st.gf.mean_abs(), st.real.mean_abs(),
               st.err.mean_abs(), late_abs_err, st.diff.mean_abs(),
               cap_rate, reverse_rate, st.se_l.mean_abs(), st.se_r.mean_abs(),
               st.speed_l.mean_abs(), st.speed_r.mean_abs());
    }

    printf("[T11_SUMMARY] samples=%d read_fail=%d fail_rate=%.3f cap_rate=%.2f reverse_rate=%.2f weak_track_steps=%d cap_steps=%d speed_weak_steps=%d reverse_steps=%d base_speed=%.1f hold_ms=%d\n",
           total_samples, total_read_fail, total_fail_rate, total_cap_rate, total_reverse_rate,
           weak_track_steps, cap_steps, speed_weak_steps, reverse_steps, base_speed, hold_ms);
    printf("[T11_RECOMMEND] ");
    if (total_fail_rate > 0.10) {
        printf("先修IMU读取/I2C/供电；角速度环没有稳定输入，不能继续调PID。\n");
    } else if (reverse_steps > 0) {
        printf("目标角速度与实际角速度长期反向：优先检查左右电机差速方向、陀螺仪Z轴符号或正负方向补偿，不要直接加P。\n");
    } else if (cap_steps > 0) {
        printf("差速限幅经常压住输出：这更像maxDiff/速度目标/输出限幅瓶颈；先看diff0与diff、goalL/R是否被截断，再决定是否小幅放宽kGyroPidOutputLimit或kMaxDiffRatio。\n");
    } else if (speed_weak_steps > 0) {
        printf("角速度环给了差速但轮速跟不上：优先回T05调速度环/电机输出/PWM限幅，不要把问题全推给角速度P。\n");
    } else if (weak_track_steps > 0) {
        printf("无明显限幅但GF跟目标差距大：可小幅增加kGyroFeedforwardGain或kGyroPidP；若GF抖动/过冲再增D或降低kGyroLpfAlpha。\n");
    } else {
        printf("角速度环左右阶跃基本可用；若T06/T07仍觉得转向不够，下一步重点看视觉外环目标、目标变化率和提速保护。\n");
    }
    printf("[调参意义] T11只隔离角速度内环；T11正常但T07转不过，优先查Tgt是否太小/变化率太慢/提速保护；T11也弱，才动角速度环参数。\n");
    printf("[RETURN] 0=角速度环可继续；1=差速限幅瓶颈；2=角速度跟踪弱；3=轮速跟不上；4=IMU读取失败；5=方向响应异常。\n");

    if (total_fail_rate > 0.10) return 4;
    if (reverse_steps > 0) return 5;
    if (cap_steps > 0) return 1;
    if (speed_weak_steps > 0) return 3;
    if (weak_track_steps > 0) return 2;
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
    printf("[安全] 不输出电机。按顺序检查配置、摄像头对线重复性/俯仰、二值质量、ROI/扫线/直道判断。\n");
    int ret = 0;
    ret = run_flow_step("T00 precheck_config", [](){ return test_00_precheck_config(); });
    if (ret != 0) return ret;
    ret = run_flow_step("T02 camera_alignment_repeatability", [](){ return test_02_camera_center_angle_static(); });
    if (ret != 0) return 20 + ret;
    ret = run_flow_step("T03 binary_quality", [](){ return test_03_exposure_otsu_quality(); });
    if (ret != 0) return 30 + ret;
    ret = run_flow_step("T04 roi_scan_straight", [](){ return test_04_roi_scan_straight(); });
    if (ret != 0) return 40 + ret;
    printf("[FLOW_SUMMARY] visual_pipeline PASS。可以进入T01方向检查、T05实地速度环或低速闭环测试。\n");
    return 0;
}

int run_airborne_pipeline(void) {
    printf("\n========== FLOW B motor_speed_pipeline 电机/速度环调参流程 ==========" "\n");
    printf("[安全] T01必须架空确认方向；T05会让小车落地直线前进，用于确认实际速度环基础响应。\n");
    int ret = 0;
    ret = run_flow_step("T00 precheck_config", [](){ return test_00_precheck_config(); });
    if (ret != 0) return ret;
    ret = run_flow_step("T01 motor_encoder_direction", [](){ return test_01_motor_encoder_direction(); });
    if (ret != 0) return 10 + std::abs(ret);
    ret = run_flow_step("T05 actual_speed_loop_straight", [](){ return test_05_speed_loop_airborne(); });
    if (ret != 0) return 50 + ret;
    printf("[FLOW_SUMMARY] motor_speed_pipeline PASS。可以继续做低速闭环。\n");
    return 0;
}

int run_sensor_pipeline(void) {
    printf("\n========== FLOW C sensor_pipeline 传感器静态检查 ==========" "\n");
    printf("[安全] 不跑车。用于检查摄像头图像和MPU6050静止噪声。\n");
    int ret = 0;
    ret = run_flow_step("T03 binary_quality", [](){ return test_03_exposure_otsu_quality(); });
    if (ret != 0) return 30 + ret;
    release_camera_for_imu_test();
    ret = run_flow_step("T10 gyro_static_noise", [](){ return test_10_gyro_static_noise(); });
    if (ret != 0) return 100 + ret;
    printf("[FLOW_SUMMARY] sensor_pipeline PASS。摄像头二值和IMU静态噪声可用。\n");
    return 0;
}

int run_low_speed_pipeline(void) {
    printf("\n========== FLOW D low_speed_pipeline 低速上赛道流程 ==========" "\n");
    printf("[安全] 会让小车低速循迹。建议先跑 visual_pipeline，并确认T01/T05基础正常后再执行。\n");
    int ret = 0;
    ret = run_flow_step("T02 camera_alignment_repeatability", [](){ return test_02_camera_center_angle_static(); });
    if (ret != 0) return 20 + ret;
    ret = run_flow_step("T03 binary_quality", [](){ return test_03_exposure_otsu_quality(); });
    if (ret != 0) return 30 + ret;
    ret = run_flow_step("T04 roi_scan_straight", [](){ return test_04_roi_scan_straight(); });
    if (ret != 0) return 40 + ret;
    release_camera_for_imu_test();
    ret = run_flow_step("T10 gyro_static_noise", [](){ return test_10_gyro_static_noise(); });
    if (ret != 0) return 100 + ret;
    ret = run_flow_step("T11 gyro_loop_step_response", [](){ return test_11_gyro_loop_step_response(); });
    if (ret != 0) return 110 + ret;
    ret = run_flow_step("T06 low_speed_visual_gyro_closed_loop", [](){ return test_06_low_speed_visual_gyro_closed_loop(); });
    if (ret != 0) return 60 + ret;
    printf("[FLOW_SUMMARY] low_speed_pipeline PASS。可以继续做速度提升或元素调试。\n");
    return 0;
}

int run_element_pipeline(void) {
    printf("\n========== FLOW E element_pipeline 元素/特殊赛道调试流程 ==========" "\n");
    printf("[安全] 会让小车低速循迹并开启元素处理。当前版本跳过T02/T03/T04，只跑元素识别/处理T08。\n");
    int ret = 0;
    ret = run_flow_step("T08 element_debug", [](){ return test_08_element_debug(); });
    if (ret != 0) return 80 + ret;
    printf("[FLOW_SUMMARY] element_pipeline PASS。元素状态稳定，可以按具体元素继续细调或再做速度回归。\n");
    return 0;
}

} // namespace param_step_demo

// =============================================================================
// 对外函数：你只需要在 main.cpp 里选择性调用这些函数即可。
// 建议顺序：12(参数地图) -> 00 -> 01(架空方向) -> 02 -> 03 -> 04 -> 05(实地速度) -> 10 -> 11 -> 06 -> 07 -> 08 -> 09
// =============================================================================

int lq_param_test_00_precheck_config(void) { return param_step_demo::run_with_log("T00_precheck_config", [](){ return param_step_demo::test_00_precheck_config(); }); }
int lq_param_test_01_motor_encoder_direction(void) { return param_step_demo::run_with_log("T01_motor_encoder_direction", [](){ return param_step_demo::test_01_motor_encoder_direction(); }); }
int lq_param_test_02_camera_center_angle_static(void) { return param_step_demo::run_with_log("T02_camera_center_angle_static", [](){ return param_step_demo::test_02_camera_center_angle_static(); }); }
int lq_param_test_03_exposure_otsu_quality(void) { return param_step_demo::run_with_log("T03_exposure_otsu_quality", [](){ return param_step_demo::test_03_exposure_otsu_quality(); }); }
int lq_param_test_03_exposure_hsv_quality(void) { return lq_param_test_03_exposure_otsu_quality(); }
int lq_param_test_04_roi_scan_straight(void) { return param_step_demo::run_with_log("T04_roi_scan_straight", [](){ return param_step_demo::test_04_roi_scan_straight(); }); }
int lq_param_test_05_speed_loop_airborne(void) { return param_step_demo::run_with_log("T05_actual_speed_loop_straight", [](){ return param_step_demo::test_05_speed_loop_airborne(); }); }
int lq_param_test_06_low_speed_visual_gyro_closed_loop(void) { return param_step_demo::run_with_log("T06_low_speed_visual_gyro", [](){ return param_step_demo::test_06_low_speed_visual_gyro_closed_loop(); }); }
int lq_param_test_07_speed_up_regression(void) { return param_step_demo::run_with_log("T07_speed_up_regression", [](){ return param_step_demo::test_07_speed_up_regression(); }); }
int lq_param_test_08_element_debug(void) { return param_step_demo::run_with_log("T08_element_debug", [](){ return param_step_demo::test_08_element_debug(); }); }
int lq_param_test_08h_element_push_image_debug(void) { return param_step_demo::run_with_log("T08H_element_push_image_debug", [](){ return param_step_demo::test_08_element_push_image_debug(); }); }
int lq_param_test_09_voltage_light_regression(void) { return param_step_demo::run_with_log("T09_voltage_light_regression", [](){ return param_step_demo::test_09_voltage_light_regression(); }); }
int lq_param_test_10_gyro_static_noise(void) { return param_step_demo::run_with_log("T10_gyro_static_noise", [](){ return param_step_demo::test_10_gyro_static_noise(); }); }
int lq_param_test_11_gyro_loop_step_response(void) { return param_step_demo::run_with_log("T11_gyro_loop_step_response", [](){ return param_step_demo::test_11_gyro_loop_step_response(); }); }
int lq_param_test_12_parameter_debug_plan(void) { return param_step_demo::run_with_log("T12_parameter_debug_plan", [](){ return param_step_demo::test_12_parameter_debug_plan(); }); }
int lq_param_test_13_pose_reference_push(void) { return param_step_demo::run_with_log("T13_pose_reference_push", [](){ return param_step_demo::test_13_pose_reference_push(); }); }
int lq_param_test_14_push_line_patrol_tft(void) { return param_step_demo::run_with_log("T14_push_line_patrol_tft", [](){ return param_step_demo::test_08p_push_line_patrol_tft(); }); }
int lq_param_flow_visual(void) { return param_step_demo::run_with_log("FLOW_A_visual_pipeline", [](){ return param_step_demo::run_visual_pipeline(); }); }
int lq_param_flow_airborne(void) { return param_step_demo::run_with_log("FLOW_B_motor_speed_pipeline", [](){ return param_step_demo::run_airborne_pipeline(); }); }
int lq_param_flow_sensor(void) { return param_step_demo::run_with_log("FLOW_C_sensor_pipeline", [](){ return param_step_demo::run_sensor_pipeline(); }); }
int lq_param_flow_low_speed(void) { return param_step_demo::run_with_log("FLOW_D_low_speed_pipeline", [](){ return param_step_demo::run_low_speed_pipeline(); }); }
int lq_param_flow_element(void) { return param_step_demo::run_with_log("FLOW_E_element_pipeline", [](){ return param_step_demo::run_element_pipeline(); }); }

void lq_camera_tft18_demo(void) {
    printf("\n========== DEMO TFT18 element image debug ==========" "\n");
    printf("该入口用于单独测试 TFT18 + 摄像头 + 当前元素补线显示。\n");
    printf("它复用 T08H 手推元素调试：电机强制关闭，TFT18 实时显示二值图、边线、中心线、P1/P2 和 ring 状态。\n");
    printf("若要自动低速通过元素，请运行 lq_param_test_08_element_debug() 或当前 main.cpp 的 34 号 Demo。\n");
    int ret = param_step_demo::run_with_log("TFT18_element_push_debug",
                                            [](){ return param_step_demo::test_08_element_push_image_debug(60); });
    printf("[TFT18_DEMO] ret=%d\n", ret);
}

void lq_parameter_step_tuning_demo(void) {
    printf("\n========== DEMO 33 parameter_step_tuning_demo ==========" "\n");
    printf("这是一个分阶段调参向导。建议先跑流程函数，再按失败步骤单独复测；不要跨阶段混改参数。\n\n");
    printf("[推荐流程]\n");
    printf("  lq_param_test_12_parameter_debug_plan(); // 不跑车：打印每个参数该看哪个测试、怎么改\n");
    printf("  lq_param_flow_visual();        // 不跑电机：配置、摄像头重复性、二值、ROI/扫线\n");
    printf("  lq_param_flow_airborne();      // T01架空确认方向；T05落地直线测速度环基础\n");
    printf("  lq_param_flow_sensor();        // 不跑车：二值质量 + MPU6050静止噪声\n");
    printf("  lq_param_flow_low_speed();     // 上赛道低速：视觉 + 陀螺仪 + 轮速闭环\n\n");
    printf("  lq_param_flow_element();       // 上赛道低速：开启元素/特殊赛道判断与处理，T08/T08H会同步刷新TFT18补线画面\n\n");
    printf("[每阶段只做一件事]\n");
    printf("  T00: 打印配置快照，确认当前编译进去的数值，防止把旧版本参数当成新参数。\n");
    printf("  T01: 架空测电机/编码器方向和左右机械差异，只处理硬件方向、接线、死区。\n");
    printf("  T02: 静态看摄像头对线重复性和俯仰角，不要求手摆像素级居中；中线只看3次重复日志后的趋势。\n");
    printf("  T03: 只看曝光/二值化/ROI基础质量，white、dDet、lost不过时不要调PID。\n");
    printf("  T04: 只看前瞻、扫线和直道判断，手推普通弯观察jump/edge_bad/straight_rate。\n");
    printf("  T05: 落地直线测实际速度环，只调速度环P/I/D、PWM限幅、左右机械一致性。\n");
    printf("  T10: 静止测IMU噪声，只调gyro死区/滤波/排查I2C供电，不用PID掩盖尖峰。\n");
    printf("  T11: 低速直接给角速度阶跃，只调角速度环/差速输出/正负方向补偿，判断是不是内环跟不上。\n");
    printf("  T12: 不跑车，打印所有关键参数的阶段归属、观察指标、调整方向和禁忌。\n");
    printf("  T13: 手推正常路径视觉基准，只记录边界覆盖/道路宽度/左右安全余量；跳变类只作REF_ONLY参考。\n");
    printf("  T06: 低速闭环后才调视觉Kp/Kd、目标角速度限幅和视觉到角速度目标的衔接。\n");
    printf("  T07/T08/T09: 分别用于提速回归、元素调试、电压/光照回归，不能拿回归问题反推前面所有参数一起改。\n\n");
    printf("[中线校准规则]\n");
    printf("  单次T02的err_mean包含人工摆车误差，只能参考；先看det_std/det_span/det_jump是否稳定。\n");
    printf("  若要改kImageMiddleLine：同一段直道重复摆放3轮，只有middle_candidate_ok=1时，才按candidate_next每次改1。\n\n");
    printf("[单项测试]\n");
    printf("  lq_param_test_00_precheck_config();\n");
    printf("  lq_param_test_01_motor_encoder_direction();      // 架空\n");
    printf("  lq_param_test_02_camera_center_angle_static();   // 实际为对线重复性/角度，不做像素级摆正判罚\n");
    printf("  lq_param_test_03_exposure_otsu_quality();        // 当前按kImageAlgorithm输出二值质量\n");
    printf("  lq_param_test_04_roi_scan_straight();\n");
    printf("  lq_param_test_05_speed_loop_airborne();          // 实地直线速度环\n");
    printf("  lq_param_test_06_low_speed_visual_gyro_closed_loop();\n");
    printf("  lq_param_test_07_speed_up_regression();\n");
    printf("  lq_param_test_08_element_debug();              // 自动低速元素调试 + TFT18实时补线\n");
    printf("  lq_camera_tft18_demo();                        // 单独手推测试TFT18补线/元素状态，电机关闭\n");
    printf("  lq_param_test_09_voltage_light_regression();\n");
    printf("  lq_param_test_10_gyro_static_noise();             // 无需赛道，静止IMU噪声\n");
    printf("  lq_param_test_11_gyro_loop_step_response();       // 上地面低速，隔离角速度环阶跃响应\n");
    printf("  lq_param_test_12_parameter_debug_plan();          // 不跑车，参数级调试方案\n");
    printf("  lq_param_test_13_pose_reference_push();           // 不跑电机，手推正常路径视觉/姿态基准\n");
    printf("\n[返回码]\n");
    printf("  单项测试返回值见各自日志；流程返回 20/30/40/50/60/100/110 + 子错误码，表示停在哪个阶段。\n");
    printf("  图像不过不要调PID；T01方向不过不要跑T05/T06；IMU静态不过不要调角速度环。\n\n");
    printf("示例：int ret = lq_param_flow_visual(); printf(\"ret=%d\\n\", ret); return 0;\n");
    param_step_demo::print_config_snapshot();
}

// =============================================================================
// 交互式分步调试入口
//    终端打印菜单，等待用户输入测试编号，运行后返回等待状态。
//    输入 q 或 0 退出。
// =============================================================================
void lq_parameter_step_interactive_demo(void)
{
    printf("\n========== 开始分阶段调试智能车 ==========\n");
    printf("输入测试编号运行对应测试，输入 q 或 0 退出。\n");
    printf("------------------------------------------------------------\n");
    printf(" 0 | q  — 退出\n");
    printf(" 1  — T01 电机/编码器方向检测 (架空)\n");
    printf(" 2  — T02 摄像头对线重复性/角度\n");
    printf(" 3  — T03 曝光/二值化质量检测\n");
    printf(" 4  — T04 ROI/前瞻/扫线参数检测\n");
    printf(" 5  — T05 实地直线速度环测试 (落地/终端调参)\n");
    printf(" 6  — T06 低速视觉+陀螺仪闭环循迹\n");
    printf(" 7  — T07 提速回归测试\n");
    printf(" 8  — T08 元素调试 (低速+TFT18)\n");
    printf(" 9  — T09 电压/光照回归测试\n");
    printf("10  — T10 陀螺仪静态噪声检测\n");
    printf("11  — T11 角速度环阶跃响应测试\n");
    printf("12  — T12 参数调试方案 (不跑车)\n");
    printf("13  — T13 手推视觉/姿态基准采集\n");
    printf("14  — T14 手推补线TFT观察 (FPS+二值图+边线+中线)\n");
    printf("------------------------------------------------------------\n");

    std::string input_buf;
    bool first_prompt = true;

    while (true)
    {
        if (first_prompt) {
            printf(">>> 请输入测试编号: ");
            fflush(stdout);
            first_prompt = false;
        }

        // 非阻塞检查输入
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 50000};  // 50ms 超时，降低 CPU 空转
        int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) break;
        if (ret == 0) continue;

        char c;
        if (read(STDIN_FILENO, &c, 1) <= 0) break;

        if (c == '\n' || c == '\r')
        {
            if (input_buf.empty()) continue;
            while (!input_buf.empty() && (input_buf.back() == '\r' || input_buf.back() == ' ')) input_buf.pop_back();

            if (input_buf == "q" || input_buf == "0")
            {
                printf(">>> 退出调试。\n");
                break;
            }

            int test_num = 0;
            std::istringstream iss(input_buf);
            if (!(iss >> test_num)) {
                printf(">>> 无效输入: %s\n", input_buf.c_str());
                input_buf.clear();
                first_prompt = true;
                continue;
            }

            int result = 0;
            switch (test_num)
            {
                case 0:
                    printf(">>> 退出调试。\n");
                    return;
                case 1:
                    printf("\n>>> 运行 T01 电机/编码器方向检测...\n");
                    result = lq_param_test_01_motor_encoder_direction();
                    break;
                case 2:
                    printf("\n>>> 运行 T02 摄像头对线重复性/角度...\n");
                    result = lq_param_test_02_camera_center_angle_static();
                    break;
                case 3:
                    printf("\n>>> 运行 T03 曝光/二值化质量检测...\n");
                    result = lq_param_test_03_exposure_otsu_quality();
                    break;
                case 4:
                    printf("\n>>> 运行 T04 ROI/前瞻/扫线参数检测...\n");
                    result = lq_param_test_04_roi_scan_straight();
                    break;
                case 5:
                    printf("\n>>> 运行 T05 实地直线速度环测试...\n");
                    result = lq_param_test_05_speed_loop_airborne();
                    break;
                case 6:
                    printf("\n>>> 运行 T06 低速视觉+陀螺仪闭环循迹...\n");
                    result = lq_param_test_06_low_speed_visual_gyro_closed_loop();
                    break;
                case 7:
                    printf("\n>>> 运行 T07 提速回归测试...\n");
                    result = lq_param_test_07_speed_up_regression();
                    break;
                case 8:
                    printf("\n>>> 运行 T08 元素调试...\n");
                    result = lq_param_test_08_element_debug();
                    break;
                case 9:
                    printf("\n>>> 运行 T09 电压/光照回归测试...\n");
                    result = lq_param_test_09_voltage_light_regression();
                    break;
                case 10:
                    printf("\n>>> 运行 T10 陀螺仪静态噪声检测...\n");
                    result = lq_param_test_10_gyro_static_noise();
                    break;
                case 11:
                    printf("\n>>> 运行 T11 角速度环阶跃响应测试...\n");
                    result = lq_param_test_11_gyro_loop_step_response();
                    break;
                case 12:
                    printf("\n>>> 运行 T12 参数调试方案...\n");
                    result = lq_param_test_12_parameter_debug_plan();
                    break;
                case 13:
                    printf("\n>>> 运行 T13 手推视觉/姿态基准采集...\n");
                    result = lq_param_test_13_pose_reference_push();
                    break;
                case 14:
                    printf("\n>>> 运行 T14 手推补线TFT观察 (FPS+补线)...\n");
                    result = lq_param_test_14_push_line_patrol_tft();
                    break;
                default:
                    printf(">>> 未知测试编号: %d\n", test_num);
                    input_buf.clear();
                    first_prompt = true;
                    continue;
            }

            printf(">>> 测试完成，返回值: %d\n\n", result);
            input_buf.clear();
            first_prompt = true;
        }
        else
        {
            input_buf += c;
        }
    }
}
