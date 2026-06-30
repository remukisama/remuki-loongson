#include "lq_vision_capture_demo.hpp"
#include "user_app.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <thread>
#include <utility>
#include <vector>

extern ImageDealDatatypedef ImageDeal[60];

namespace {

const char *road_type_name(RoadType_e type)
{
    switch (type) {
    case zero: return "zero";
    case Normol: return "normal";
    case Straight: return "straight";
    case Cross: return "cross";
    case Ramp: return "ramp";
    case LeftCirque: return "left_ring";
    case RightCirque: return "right_ring";
    case Cross_ture: return "cross_true";
    case Barn_in: return "barn_in";
    default: return "unknown";
    }
}

void put_pixel(cv::Mat &image, int x, int y, const cv::Vec3b &color)
{
    if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
        image.at<cv::Vec3b>(y, x) = color;
    }
}

cv::Mat make_patrol_overlay_frame_from(
    const uint8_t binary[60][80],
    const ImageDealDatatypedef deal[60],
    const ImageStatustypedef &status,
    const ImageFlagtypedef &flag)
{
    cv::Mat base(60, 80, CV_8UC3);
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 80; ++x) {
            base.at<cv::Vec3b>(y, x) = binary[y][x] ? cv::Vec3b(235, 235, 235)
                                                    : cv::Vec3b(0, 0, 0);
        }
    }

    for (int y = 0; y < 60; ++y) {
        put_pixel(base, deal[y].LeftBoundary_First, y, cv::Vec3b(0, 210, 210));
        put_pixel(base, deal[y].RightBoundary_First, y, cv::Vec3b(210, 210, 0));
        put_pixel(base, deal[y].LeftBorder, y,
                  (deal[y].IsLeftFind == 'W') ? cv::Vec3b(255, 0, 255) : cv::Vec3b(0, 0, 255));
        put_pixel(base, deal[y].RightBorder, y,
                  (deal[y].IsRightFind == 'W') ? cv::Vec3b(255, 0, 255) : cv::Vec3b(0, 255, 0));
        put_pixel(base, deal[y].Center, y, cv::Vec3b(255, 0, 0));
    }

    const int tow_y = std::max(0, std::min(59, status.TowPoint));
    const int off_y = std::max(0, std::min(59, static_cast<int>(status.OFFLine)));
    cv::line(base, cv::Point(0, tow_y), cv::Point(79, tow_y), cv::Scalar(255, 255, 0), 1);
    cv::line(base, cv::Point(0, off_y), cv::Point(79, off_y), cv::Scalar(0, 165, 255), 1);
    put_pixel(base, status.Det_True, tow_y, cv::Vec3b(0, 255, 255));
    put_pixel(base, status.MiddleLine, 55, cv::Vec3b(255, 255, 255));

    cv::Mat display;
    cv::resize(base, display, cv::Size(640, 480), 0, 0, cv::INTER_NEAREST);
    cv::rectangle(display, cv::Point(0, 0), cv::Point(display.cols - 1, 70), cv::Scalar(0, 0, 0), -1);

    char text[180];
    std::snprintf(text, sizeof(text), "road:%s det:%d raw:%d tow:%d off:%d ll:%d rl:%d ring:%d/%d",
                  road_type_name(status.Road_type),
                  status.Det_True,
                  status.MiddleLine - status.Det_True,
                  status.TowPoint,
                  status.OFFLine,
                  status.Left_Line,
                  status.Right_Line,
                  flag.image_element_rings,
                  flag.image_element_rings_flag);
    cv::putText(display, text, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    std::snprintf(text, sizeof(text), "red=L final  green=R final  blue=center  cyan/yellow=raw boundary");
    cv::putText(display, text, cv::Point(8, 50), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    return display;
}

cv::Mat make_patrol_overlay_frame()
{
    return make_patrol_overlay_frame_from(img1, ImageDeal, ImageStatus, ImageFlag);
}

cv::Mat make_raw_roi_frame_from(cv::Mat raw, const ImageStatustypedef &status)
{
    if (raw.empty()) return raw;

    const int sx = std::max(1, car_config::kCameraHiresWidth / car_config::kCameraWidth);
    const int sy = std::max(1, car_config::kCameraHiresHeight / car_config::kCameraHeight);
    cv::rectangle(raw,
                  cv::Rect(car_config::kCameraRoiX * sx,
                           car_config::kCameraRoiY * sy,
                           car_config::kCameraRoiWidth * sx,
                           car_config::kCameraRoiHeight * sy),
                  cv::Scalar(0, 255, 255), 2);

    char text[120];
    std::snprintf(text, sizeof(text), "raw:%d off:%d ll:%d rl:%d",
                  status.MiddleLine - status.Det_True,
                  status.OFFLine,
                  status.Left_Line,
                  status.Right_Line);
    cv::putText(raw, text, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    return raw;
}

cv::Mat make_raw_roi_frame()
{
    return make_raw_roi_frame_from(image_cv_get_hires_frame().clone(), ImageStatus);
}

struct RaceSnapshot {
    int index = 0;
    int frame_index = 0;
    cv::Mat raw;
    uint8_t binary[60][80];
    ImageDealDatatypedef deal[60];
    ImageStatustypedef status;
    ImageFlagtypedef flag;
};

class RaceCaptureWriter {
public:
    RaceCaptureWriter()
        : stopping_(false), worker_(&RaceCaptureWriter::run, this)
    {
        mkdir("/tmp/vision_race_capture", 0755);
    }

    ~RaceCaptureWriter()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

    bool enqueue(RaceSnapshot snapshot)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= 2) return false;
        queue_.push_back(std::move(snapshot));
        cv_.notify_one();
        return true;
    }

private:
    void run()
    {
        for (;;) {
            RaceSnapshot snapshot;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
                if (stopping_ && queue_.empty()) return;
                snapshot = std::move(queue_.front());
                queue_.pop_front();
            }
            write(snapshot);
        }
    }

    void write(const RaceSnapshot &snapshot)
    {
        const cv::Mat raw = make_raw_roi_frame_from(snapshot.raw.clone(), snapshot.status);
        const cv::Mat overlay = make_patrol_overlay_frame_from(
            snapshot.binary, snapshot.deal, snapshot.status, snapshot.flag);

        char raw_path[192];
        char overlay_path[192];
        std::snprintf(raw_path, sizeof(raw_path),
                      "/tmp/vision_race_capture/race_%03d_f%05d_raw.jpg",
                      snapshot.index, snapshot.frame_index);
        std::snprintf(overlay_path, sizeof(overlay_path),
                      "/tmp/vision_race_capture/race_%03d_f%05d_overlay.jpg",
                      snapshot.index, snapshot.frame_index);

        const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 80};
        const bool raw_ok = !raw.empty() && cv::imwrite(raw_path, raw, jpeg_params);
        const bool overlay_ok = cv::imwrite(overlay_path, overlay, jpeg_params);
        std::printf("[VISION_CAPTURE][RACE] idx:%03d frame:%d err:%d off:%d ll:%d rl:%d road:%s raw:%d overlay:%d\n",
                    snapshot.index,
                    snapshot.frame_index,
                    snapshot.status.MiddleLine - snapshot.status.Det_True,
                    snapshot.status.OFFLine,
                    snapshot.status.Left_Line,
                    snapshot.status.Right_Line,
                    road_type_name(snapshot.status.Road_type),
                    raw_ok ? 1 : 0,
                    overlay_ok ? 1 : 0);
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<RaceSnapshot> queue_;
    bool stopping_;
    std::thread worker_;
};

RaceCaptureWriter &race_capture_writer()
{
    static RaceCaptureWriter writer;
    return writer;
}

} // namespace

int lq_vision_capture_demo(void)
{
    const char *dir = "/tmp/vision_capture";
    mkdir(dir, 0755);
    Data_Settings();
    g_enable_element_process = false;
    if (!image_cv_Init()) {
        std::printf("[VISION_CAPTURE][FATAL] camera init failed\n");
        return 5;
    }

    int saved = 0;
    for (int i = 0; i < 12; ++i) {
        if (!image_cv_zip()) {
            std::printf("[VISION_CAPTURE][WARN] frame %d read failed\n", i);
            continue;
        }
        imageprocess();

        const cv::Mat raw = make_raw_roi_frame();
        const cv::Mat overlay = make_patrol_overlay_frame();
        char raw_path[160];
        char overlay_path[160];
        std::snprintf(raw_path, sizeof(raw_path), "%s/frame_%02d_raw.jpg", dir, i);
        std::snprintf(overlay_path, sizeof(overlay_path), "%s/frame_%02d_overlay.png", dir, i);

        const bool raw_ok = !raw.empty() && cv::imwrite(raw_path, raw);
        const bool overlay_ok = cv::imwrite(overlay_path, overlay);
        std::printf("[VISION_CAPTURE] frame:%02d raw:%d overlay:%d det:%d mid:%d err:%d road:%s off:%d ll:%d rl:%d ring:%d/%d\n",
                    i,
                    raw_ok ? 1 : 0,
                    overlay_ok ? 1 : 0,
                    ImageStatus.Det_True,
                    ImageStatus.MiddleLine,
                    ImageStatus.MiddleLine - ImageStatus.Det_True,
                    road_type_name(ImageStatus.Road_type),
                    ImageStatus.OFFLine,
                    ImageStatus.Left_Line,
                    ImageStatus.Right_Line,
                    ImageFlag.image_element_rings,
                    ImageFlag.image_element_rings_flag);
        if (raw_ok || overlay_ok) ++saved;
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    std::printf("[VISION_CAPTURE] saved_frames:%d dir:%s\n", saved, dir);
    return saved > 0 ? 0 : 6;
}

void lq_vision_capture_maybe_save_race_frame(int frame_index)
{
    static int enabled = -1;
    static int saved = 0;
    static int last_saved_frame = -1000;
    if (enabled < 0) {
        const char *env = std::getenv("LQ_RACE_CAPTURE");
        enabled = (env != nullptr && env[0] != '\0' && std::strcmp(env, "0") != 0) ? 1 : 0;
        if (enabled) {
            mkdir("/tmp/vision_race_capture", 0755);
            race_capture_writer();
            std::printf("[VISION_CAPTURE] race snapshot enabled: /tmp/vision_race_capture (async)\n");
        }
    }
    if (!enabled || saved >= 3) return;

    const int raw_error = ImageStatus.MiddleLine - ImageStatus.Det_True;
    const bool turn_or_loss =
        std::abs(raw_error) >= 12 ||
        ImageStatus.OFFLine >= 18 ||
        ImageStatus.Left_Line >= 18 ||
        ImageStatus.Right_Line >= 18;
    // Keep runtime snapshots sparse; frame_index advances once per camera frame.
    if (!turn_or_loss || frame_index - last_saved_frame < 220) return;

    RaceSnapshot snapshot;
    snapshot.index = saved;
    snapshot.frame_index = frame_index;
    snapshot.raw = image_cv_get_hires_frame().clone();
    std::memcpy(snapshot.binary, img1, sizeof(snapshot.binary));
    std::memcpy(snapshot.deal, ImageDeal, sizeof(snapshot.deal));
    snapshot.status = ImageStatus;
    snapshot.flag = ImageFlag;

    if (race_capture_writer().enqueue(std::move(snapshot))) {
        ++saved;
        last_saved_frame = frame_index;
    }
}
