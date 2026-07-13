#include "lq_vision_capture_demo.hpp"
#include "main.hpp"
#include "ncnn_inference.hpp"
#include "user_app.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
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

const char *ncnn_action_name(const std::string &label)
{
    if (label == car_config::kNcnnLabel0) return "right";
    if (label == car_config::kNcnnLabel1) return "straight";
    if (label == car_config::kNcnnLabel2) return "left";
    return "none";
}

float ncnn_probability_at(const NcnnResult &result, int index)
{
    if (index < 0 || index >= static_cast<int>(result.probabilities.size())) return 0.0f;
    return result.probabilities[index];
}

struct NcnnGapObserveResult {
    int gap = 0;
    NcnnResult result;
    std::string roi_path;
};

struct NcnnDecisionVote {
    bool valid = false;
    bool supplies = false;
    bool vehicles = false;
    bool weapons = false;
};

struct NcnnDecisionState {
    bool locked = false;
    std::string label;
    int locked_frame = -1;
    int window_size = 0;
    int supplies_votes = 0;
    int vehicles_votes = 0;
    int weapons_votes = 0;
};

const int *ncnn_observe_gaps(size_t &count)
{
    static const int gaps[] = {0, 2, 5, 8};
    count = sizeof(gaps) / sizeof(gaps[0]);
    return gaps;
}

constexpr int kNcnnDecisionWindowSize = 8;
constexpr int kNcnnSuppliesVotesNeeded = 2;
constexpr int kNcnnVehiclesVotesNeeded = 4;
constexpr int kNcnnWeaponsVotesNeeded = 4;
constexpr float kNcnnSuppliesGap0Threshold = 0.55f;
constexpr float kNcnnVehiclesGap2Threshold = 0.90f;
constexpr float kNcnnWeaponsGap025Threshold = 0.90f;
constexpr int kNcnnDecisionResetNoRedFrames = 12;

std::string save_ncnn_roi_debug(const NcnnResult &result, int frame_index, const std::string &tag, int sequence)
{
    if (!result.roi_valid || result.input_roi.empty()) return {};
    char path[192];
    std::snprintf(path, sizeof(path),
                  "/home/root/ncnn_debug_roi/roi_%04d_f%06d_g%02d_%s_%s_c%03d.jpg",
                  sequence,
                  frame_index,
                  result.object_bottom_gap,
                  tag.empty() ? "element" : tag.c_str(),
                  result.label.empty() ? "none" : result.label.c_str(),
                  static_cast<int>(result.confidence * 1000.0f + 0.5f));
    if (!cv::imwrite(path, result.input_roi)) return {};
    return path;
}

void draw_points(cv::Mat &frame, const std::vector<cv::Point2f> &points, const cv::Scalar &color)
{
    if (points.size() < 2) return;
    for (size_t i = 0; i < points.size(); ++i) {
        const cv::Point p0(cvRound(points[i].x), cvRound(points[i].y));
        const cv::Point p1(cvRound(points[(i + 1) % points.size()].x), cvRound(points[(i + 1) % points.size()].y));
        cv::line(frame, p0, p1, color, 2, cv::LINE_AA);
    }
}

cv::Scalar ncnn_gap_color(int index)
{
    static const cv::Scalar colors[] = {
        cv::Scalar(0, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 0, 255),
        cv::Scalar(0, 165, 255)
    };
    return colors[index % (sizeof(colors) / sizeof(colors[0]))];
}

cv::Mat make_ncnn_observe_frame(
    cv::Mat frame,
    const std::vector<NcnnGapObserveResult> &gap_results,
    const NcnnDecisionState &decision,
    int frame_index,
    const char *tag)
{
    if (frame.empty()) {
        frame = cv::Mat::zeros(car_config::kCameraHiresHeight, car_config::kCameraHiresWidth, CV_8UC3);
    }

    const NcnnResult empty_result;
    const NcnnResult &base_result = gap_results.empty() ? empty_result : gap_results.front().result;

    const int src_width = std::max(1, frame.cols);
    const int src_height = std::max(1, frame.rows);
    cv::resize(frame, frame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
    const float sx = 640.0f / src_width;
    const float sy = 480.0f / src_height;

    if (base_result.red_marker_valid) {
        cv::Rect red_box(cvRound(base_result.red_box.x * sx),
                         cvRound(base_result.red_box.y * sy),
                         cvRound(base_result.red_box.width * sx),
                         cvRound(base_result.red_box.height * sy));
        cv::rectangle(frame, red_box, cv::Scalar(0, 255, 255), 2);

        std::vector<cv::Point2f> scaled_red;
        scaled_red.reserve(base_result.red_box_points.size());
        for (const cv::Point2f &point : base_result.red_box_points) {
            scaled_red.emplace_back(point.x * sx, point.y * sy);
        }
        draw_points(frame, scaled_red, cv::Scalar(0, 200, 255));
    }

    for (size_t i = 0; i < gap_results.size(); ++i) {
        const NcnnResult &result = gap_results[i].result;
        if (!result.roi_valid) continue;
        std::vector<cv::Point2f> scaled_roi;
        scaled_roi.reserve(result.roi_points.size());
        for (const cv::Point2f &point : result.roi_points) {
            scaled_roi.emplace_back(point.x * sx, point.y * sy);
        }
        draw_points(frame, scaled_roi, ncnn_gap_color(static_cast<int>(i)));
    }

    cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.cols - 1, 182), cv::Scalar(0, 0, 0), -1);
    char text[224];
    std::snprintf(text, sizeof(text), "NCNN GAP OBS %s f:%d red:%d area:%.0f th:%.2f",
                  tag ? tag : "-",
                  frame_index,
                  base_result.red_marker_valid ? 1 : 0,
                  base_result.red_area,
                  car_config::kNcnnConfidenceThreshold);
    cv::putText(frame, text, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.58, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    int y = 50;
    for (size_t i = 0; i < gap_results.size(); ++i) {
        const NcnnResult &result = gap_results[i].result;
        const char *label = result.label.empty() ? "-" : result.label.c_str();
        std::snprintf(text, sizeof(text), "g%-2d %s v:%d c:%4.1f S/V/W:%4.1f %4.1f %4.1f",
                      gap_results[i].gap,
                      label,
                      result.valid ? 1 : 0,
                      result.confidence * 100.0f,
                      ncnn_probability_at(result, 0) * 100.0f,
                      ncnn_probability_at(result, 1) * 100.0f,
                      ncnn_probability_at(result, 2) * 100.0f);
        cv::putText(frame, text, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX,
                    0.48, result.valid ? ncnn_gap_color(static_cast<int>(i)) : cv::Scalar(190, 190, 190), 1, cv::LINE_AA);
        y += 24;
    }

    std::snprintf(text, sizeof(text), "LOCK:%s action:%s win:%d S/V/W:%d/%d/%d",
                  decision.locked ? decision.label.c_str() : "-",
                  decision.locked ? ncnn_action_name(decision.label) : "none",
                  decision.window_size,
                  decision.supplies_votes,
                  decision.vehicles_votes,
                  decision.weapons_votes);
    cv::putText(frame, text, cv::Point(8, 146), cv::FONT_HERSHEY_SIMPLEX,
                0.52, decision.locked ? cv::Scalar(0, 255, 0) : cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(text, sizeof(text), "rule: full8 g0S>=.55x2 | g2V>=.90x4 | g0+g2+g5W>=.90x4");
    cv::putText(frame, text, cv::Point(8, 170), cv::FONT_HERSHEY_SIMPLEX,
                0.52, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    return frame;
}

class NcnnObserveWorker {
public:
    bool start(const char *host, int port)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_) return true;
        if (!ncnn_.init()) {
            std::printf("[NCNN_OBS][WARN] NCNN init failed; observe disabled\n");
            return false;
        }
        mkdir("/home/root/ncnn_debug_roi", 0755);
        udp_client_.udp_client_init(host, port);
        host_ = host ? host : "";
        port_ = port;
        saved_roi_count_ = 0;
        clear_decision_state();
        stopping_ = false;
        pending_ = false;
        running_ = true;
        worker_ = std::thread(&NcnnObserveWorker::run, this);
        std::printf("[NCNN_OBS] UDP stream to %s:%d, every %d frames, confidence>=%.2f\n",
                    host_.c_str(),
                    port_,
                    car_config::kNcnnObserveUdpEveryFrames,
                    car_config::kNcnnConfidenceThreshold);
        return true;
    }

    void submit(const cv::Mat &frame, int frame_index, const char *tag)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || frame.empty()) return;
        latest_frame_ = frame.clone();
        latest_frame_index_ = frame_index;
        latest_tag_ = tag ? tag : "";
        pending_ = true;
        cv_.notify_one();
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_) return;
            stopping_ = true;
        }
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();

        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        stopping_ = false;
        pending_ = false;
        latest_frame_.release();
    }

private:
    void clear_decision_state()
    {
        decision_window_.clear();
        decision_state_ = NcnnDecisionState();
        no_red_observe_count_ = 0;
    }

    const NcnnResult *find_gap_result(const std::vector<NcnnGapObserveResult> &gap_results, int gap) const
    {
        for (const NcnnGapObserveResult &gap_result : gap_results) {
            if (gap_result.gap == gap) return &gap_result.result;
        }
        return nullptr;
    }

    NcnnDecisionVote make_decision_vote(const std::vector<NcnnGapObserveResult> &gap_results) const
    {
        NcnnDecisionVote vote;
        const NcnnResult *g0 = find_gap_result(gap_results, 0);
        const NcnnResult *g2 = find_gap_result(gap_results, 2);
        const NcnnResult *g5 = find_gap_result(gap_results, 5);
        const NcnnResult *g8 = find_gap_result(gap_results, 8);
        if (!g0 || !g2 || !g5 || !g8 ||
            !g0->roi_valid || !g2->roi_valid || !g5->roi_valid || !g8->roi_valid) {
            return vote;
        }

        vote.valid = true;
        vote.supplies =
            ncnn_probability_at(*g0, 0) >= kNcnnSuppliesGap0Threshold;
        vote.vehicles =
            ncnn_probability_at(*g2, 1) >= kNcnnVehiclesGap2Threshold;
        vote.weapons =
            ncnn_probability_at(*g0, 2) >= kNcnnWeaponsGap025Threshold &&
            ncnn_probability_at(*g2, 2) >= kNcnnWeaponsGap025Threshold &&
            ncnn_probability_at(*g5, 2) >= kNcnnWeaponsGap025Threshold;
        return vote;
    }

    NcnnDecisionState update_decision(const std::vector<NcnnGapObserveResult> &gap_results, int frame_index)
    {
        const bool red_visible =
            !gap_results.empty() && gap_results.front().result.red_marker_valid;
        if (!red_visible) {
            ++no_red_observe_count_;
            if (no_red_observe_count_ > kNcnnDecisionResetNoRedFrames) {
                clear_decision_state();
            }
            return decision_state_;
        }
        no_red_observe_count_ = 0;

        const NcnnDecisionVote vote = make_decision_vote(gap_results);
        if (vote.valid) {
            decision_window_.push_back(vote);
            while (static_cast<int>(decision_window_.size()) > kNcnnDecisionWindowSize) {
                decision_window_.pop_front();
            }
        }

        int supplies_votes = 0;
        int vehicles_votes = 0;
        int weapons_votes = 0;
        for (const NcnnDecisionVote &item : decision_window_) {
            if (item.supplies) ++supplies_votes;
            if (item.vehicles) ++vehicles_votes;
            if (item.weapons) ++weapons_votes;
        }

        decision_state_.window_size = static_cast<int>(decision_window_.size());
        decision_state_.supplies_votes = supplies_votes;
        decision_state_.vehicles_votes = vehicles_votes;
        decision_state_.weapons_votes = weapons_votes;

        if (!decision_state_.locked &&
            decision_state_.window_size >= kNcnnDecisionWindowSize) {
            if (supplies_votes >= kNcnnSuppliesVotesNeeded &&
                supplies_votes >= weapons_votes) {
                decision_state_.locked = true;
                decision_state_.label = car_config::kNcnnLabel0;
                decision_state_.locked_frame = frame_index;
            } else if (vehicles_votes >= kNcnnVehiclesVotesNeeded) {
                decision_state_.locked = true;
                decision_state_.label = car_config::kNcnnLabel1;
                decision_state_.locked_frame = frame_index;
            } else if (weapons_votes >= kNcnnWeaponsVotesNeeded) {
                decision_state_.locked = true;
                decision_state_.label = car_config::kNcnnLabel2;
                decision_state_.locked_frame = frame_index;
            }
        }

        return decision_state_;
    }

    void run()
    {
        for (;;) {
            cv::Mat frame;
            int frame_index = 0;
            std::string tag;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stopping_ || pending_; });
                if (stopping_ && !pending_) return;
                frame = latest_frame_;
                frame_index = latest_frame_index_;
                tag = latest_tag_;
                pending_ = false;
            }

            std::vector<NcnnGapObserveResult> gap_results;
            size_t gap_count = 0;
            const int *gaps = ncnn_observe_gaps(gap_count);
            gap_results.reserve(gap_count);
            for (size_t i = 0; i < gap_count; ++i) {
                NcnnGapObserveResult gap_result;
                gap_result.gap = gaps[i];
                const bool accepted = ncnn_.infer_with_result(frame, gap_result.result, gaps[i]);
                gap_result.result.valid = accepted;
                if (gap_result.result.roi_valid && saved_roi_count_ < 500) {
                    gap_result.roi_path = save_ncnn_roi_debug(gap_result.result, frame_index, tag, ++saved_roi_count_);
                }
                gap_results.push_back(std::move(gap_result));
            }

            const NcnnDecisionState decision = update_decision(gap_results, frame_index);
            const cv::Mat overlay = make_ncnn_observe_frame(frame, gap_results, decision, frame_index, tag.c_str());
            udp_client_.udp_send_image(overlay, car_config::kNcnnObserveJpegQuality);

            const bool should_print =
                (!gap_results.empty() && gap_results.front().result.red_marker_valid) ||
                (frame_index % 60) == 0;
            if (should_print) {
                for (const NcnnGapObserveResult &gap_result : gap_results) {
                    const NcnnResult &result = gap_result.result;
                    std::printf("[NCNN_GAP] f:%d tag:%s gap:%d red:%d roi:%d valid:%d label:%s conf:%.3f pS:%.3f pV:%.3f pW:%.3f action:%s area:%.0f roi_file:%s\n",
                                frame_index,
                                tag.c_str(),
                                gap_result.gap,
                                result.red_marker_valid ? 1 : 0,
                                result.roi_valid ? 1 : 0,
                                result.valid ? 1 : 0,
                                result.label.empty() ? "-" : result.label.c_str(),
                                result.confidence,
                                ncnn_probability_at(result, 0),
                                ncnn_probability_at(result, 1),
                                ncnn_probability_at(result, 2),
                                ncnn_action_name(result.label),
                                result.red_area,
                                gap_result.roi_path.empty() ? "-" : gap_result.roi_path.c_str());
                }
                std::printf("[NCNN_LOCK] f:%d tag:%s locked:%d label:%s action:%s lock_frame:%d window:%d votesS:%d votesV:%d votesW:%d\n",
                            frame_index,
                            tag.c_str(),
                            decision.locked ? 1 : 0,
                            decision.locked ? decision.label.c_str() : "-",
                            decision.locked ? ncnn_action_name(decision.label) : "none",
                            decision.locked_frame,
                            decision.window_size,
                            decision.supplies_votes,
                            decision.vehicles_votes,
                            decision.weapons_votes);
            }
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    cv::Mat latest_frame_;
    int latest_frame_index_ = 0;
    std::string latest_tag_;
    std::string host_;
    int port_ = 0;
    bool running_ = false;
    bool stopping_ = false;
    bool pending_ = false;
    int saved_roi_count_ = 0;
    int no_red_observe_count_ = 0;
    std::deque<NcnnDecisionVote> decision_window_;
    NcnnDecisionState decision_state_;
    std::thread worker_;
    NcnnInference ncnn_;
    lq_udp_client udp_client_;
};

NcnnObserveWorker &ncnn_observe_worker()
{
    static NcnnObserveWorker worker;
    return worker;
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

bool lq_ncnn_observe_start(const char *host, int port)
{
    return ncnn_observe_worker().start(host, port);
}

void lq_ncnn_observe_submit(const cv::Mat &frame, int frame_index, const char *tag)
{
    ncnn_observe_worker().submit(frame, frame_index, tag);
}

void lq_ncnn_observe_stop(void)
{
    ncnn_observe_worker().stop();
}

int lq_vision_capture_demo(void)
{
    const char *dir = "/tmp/vision_capture";
    mkdir(dir, 0755);
    int requested_frames = 12;
    if (const char *env = std::getenv("LQ_VISION_CAPTURE_FRAMES")) {
        const long parsed = std::strtol(env, nullptr, 10);
        if (parsed >= 12 && parsed <= 5000) {
            requested_frames = static_cast<int>(parsed);
        }
    }
    const bool stress_mode = requested_frames > 12;
    const int save_interval = std::max(1, requested_frames / 12);

    Data_Settings();
    g_enable_element_process = false;
    if (!image_cv_Init()) {
        std::printf("[VISION_CAPTURE][FATAL] camera init failed\n");
        return 5;
    }

    int saved = 0;
    int frames_ok = 0;
    int frames_failed = 0;
    int failure_streak = 0;
    int max_failure_streak = 0;
    int read_over_25ms = 0;
    int read_over_50ms = 0;
    int read_over_100ms = 0;
    int read_over_150ms = 0;
    std::vector<double> read_latency_ms;
    read_latency_ms.reserve(requested_frames);
    int pipeline_over_25ms = 0;
    int pipeline_over_50ms = 0;
    int pipeline_over_100ms = 0;
    int pipeline_over_150ms = 0;
    std::vector<double> pipeline_latency_ms;
    pipeline_latency_ms.reserve(requested_frames);
    const auto capture_started = std::chrono::steady_clock::now();
    for (int i = 0; i < requested_frames; ++i) {
        const auto read_started = std::chrono::steady_clock::now();
        const bool read_ok = image_cv_zip();
        const double read_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - read_started).count();
        read_latency_ms.push_back(read_ms);
        if (read_ms > 25.0) ++read_over_25ms;
        if (read_ms > 50.0) ++read_over_50ms;
        if (read_ms > 100.0) ++read_over_100ms;
        if (read_ms > 150.0) ++read_over_150ms;

        if (!read_ok) {
            ++frames_failed;
            ++failure_streak;
            max_failure_streak = std::max(max_failure_streak, failure_streak);
            if (frames_failed <= 12) {
                std::printf("[VISION_CAPTURE][WARN] frame %d read failed\n", i);
            }
            continue;
        }
        ++frames_ok;
        failure_streak = 0;
        imageprocess();
        const double pipeline_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - read_started).count();
        pipeline_latency_ms.push_back(pipeline_ms);
        if (pipeline_ms > 25.0) ++pipeline_over_25ms;
        if (pipeline_ms > 50.0) ++pipeline_over_50ms;
        if (pipeline_ms > 100.0) ++pipeline_over_100ms;
        if (pipeline_ms > 150.0) ++pipeline_over_150ms;

        const bool save_this_frame =
            !stress_mode || (saved < 12 && (i % save_interval) == 0);
        if (!save_this_frame) {
            continue;
        }

        const cv::Mat raw = make_raw_roi_frame();
        const cv::Mat overlay = make_patrol_overlay_frame();
        char raw_path[160];
        char overlay_path[160];
        const int save_index = stress_mode ? saved : i;
        std::snprintf(raw_path, sizeof(raw_path), "%s/frame_%02d_raw.jpg", dir, save_index);
        std::snprintf(overlay_path, sizeof(overlay_path), "%s/frame_%02d_overlay.png", dir, save_index);

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
        if (!stress_mode) {
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
        }
    }

    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - capture_started).count();
    const double processed_fps = elapsed_s > 0.0 ? frames_ok / elapsed_s : 0.0;
    std::sort(read_latency_ms.begin(), read_latency_ms.end());
    std::sort(pipeline_latency_ms.begin(), pipeline_latency_ms.end());
    const auto latency_percentile = [](const std::vector<double> &latencies, int percentile) {
        if (latencies.empty()) return 0.0;
        const size_t rank =
            (latencies.size() * static_cast<size_t>(percentile) + 99U) / 100U;
        return latencies[std::min(latencies.size() - 1U, rank - 1U)];
    };
    const double read_p95_ms = latency_percentile(read_latency_ms, 95);
    const double read_p99_ms = latency_percentile(read_latency_ms, 99);
    const double read_max_ms = read_latency_ms.empty() ? 0.0 : read_latency_ms.back();
    const double pipeline_p95_ms = latency_percentile(pipeline_latency_ms, 95);
    const double pipeline_p99_ms = latency_percentile(pipeline_latency_ms, 99);
    const double pipeline_max_ms = pipeline_latency_ms.empty() ? 0.0 : pipeline_latency_ms.back();
    std::printf(
        "[VISION_CAPTURE] requested:%d ok:%d failed:%d max_fail_streak:%d elapsed_s:%.3f processed_fps:%.2f read_ms_p95/p99/max:%.2f/%.2f/%.2f read_over25/50/100/150:%d/%d/%d/%d pipeline_ms_p95/p99/max:%.2f/%.2f/%.2f pipeline_over25/50/100/150:%d/%d/%d/%d saved_frames:%d dir:%s\n",
        requested_frames,
        frames_ok,
        frames_failed,
        max_failure_streak,
        elapsed_s,
        processed_fps,
        read_p95_ms,
        read_p99_ms,
        read_max_ms,
        read_over_25ms,
        read_over_50ms,
        read_over_100ms,
        read_over_150ms,
        pipeline_p95_ms,
        pipeline_p99_ms,
        pipeline_max_ms,
        pipeline_over_25ms,
        pipeline_over_50ms,
        pipeline_over_100ms,
        pipeline_over_150ms,
        saved,
        dir);
    if (saved <= 0) return 6;
    return frames_failed == 0 ? 0 : 7;
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

// =====================================================================
// NCNN/视觉观测模式：不驱动电机，普通元素处理关闭，UDP图传/TFT屏显示
// =====================================================================

// TFT 颜色 (lq_display_color_t 枚举)
static const lq_display_color_t TFT_BLACK   = U16BLACK;
static const lq_display_color_t TFT_WHITE   = U16WHITE;
static const lq_display_color_t TFT_RED     = U16RED;
static const lq_display_color_t TFT_GREEN   = U16GREEN;
static const lq_display_color_t TFT_BLUE    = U16BLUE;
static const lq_display_color_t TFT_YELLOW  = U16YELLOW;
static const lq_display_color_t TFT_CYAN    = U16CYAN;
static const lq_display_color_t TFT_MAGENTA = U16PURPLE;
static const lq_display_color_t TFT_ORANGE  = U16ORANGE;

static const char *element_road_name(RoadType_e type)
{
    switch (type) {
    case zero:        return "zero";
    case Normol:      return "Normal";
    case Straight:    return "Straight";
    case Cross:       return "Cross";
    case Ramp:        return "Ramp";
    case LeftCirque:  return "LeftRing";
    case RightCirque: return "RightRing";
    case Cross_ture:  return "CrossTrue";
    case Barn_in:     return "BarnIn";
    default:          return "Unknown";
    }
}

static void element_observe_draw_tft(void)
{
    // 清屏
    lq_tft18_drv_cls(TFT_BLACK);

    // 画二值图 (60行x80列 -> TFT 128x128，缩放到 80x60 居中)
    uint8_t xoff = 0, yoff = 0;
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 80; ++x) {
            if (img1[y][x]) {
                lq_tft18_drv_draw_dot(xoff + x, yoff + y, TFT_WHITE);
            }
        }
    }

    // 画左边线（蓝）、右边线（绿）、中线（红）、初始边界（青/黄）
    for (int y = 0; y < 60; ++y) {
        int lx = ImageDeal[y].LeftBorder;
        int rx = ImageDeal[y].RightBorder;
        int cx = ImageDeal[y].Center;
        int lxf = ImageDeal[y].LeftBoundary_First;
        int rxf = ImageDeal[y].RightBoundary_First;

        if (lxf >= 0 && lxf < 80)
            lq_tft18_drv_draw_dot(xoff + lxf, yoff + y, TFT_CYAN);
        if (rxf >= 0 && rxf < 80)
            lq_tft18_drv_draw_dot(xoff + rxf, yoff + y, TFT_YELLOW);
        if (lx >= 0 && lx < 80)
            lq_tft18_drv_draw_dot(xoff + lx, yoff + y, TFT_BLUE);
        if (rx >= 0 && rx < 80)
            lq_tft18_drv_draw_dot(xoff + rx, yoff + y, TFT_GREEN);
        if (cx >= 0 && cx < 80)
            lq_tft18_drv_draw_dot(xoff + cx, yoff + y, TFT_RED);
    }

    // 画前瞻线（黄色横线）
    int tp = ImageStatus.TowPoint;
    if (tp >= 0 && tp < 60) {
        for (int x = 0; x < 80; ++x)
            lq_tft18_drv_draw_dot(xoff + x, yoff + tp, TFT_YELLOW);
    }

    // 状态文字（底部，TFT 128宽，64行以下为文字区）
    char buf[64];
    int ty = 64;
    std::snprintf(buf, sizeof(buf), "Road:%s", element_road_name(ImageStatus.Road_type));
    lq_tft18_drv_p6x8_str(0, ty, buf, TFT_WHITE, TFT_BLACK); ty += 8;

    std::snprintf(buf, sizeof(buf), "det:%d err:%d off:%d",
                  ImageStatus.Det_True,
                  ImageStatus.MiddleLine - ImageStatus.Det_True,
                  ImageStatus.OFFLine);
    lq_tft18_drv_p6x8_str(0, ty, buf, TFT_WHITE, TFT_BLACK); ty += 8;

    std::snprintf(buf, sizeof(buf), "ll:%d rl:%d tp:%d",
                  ImageStatus.Left_Line, ImageStatus.Right_Line, ImageStatus.TowPoint);
    lq_tft18_drv_p6x8_str(0, ty, buf, TFT_WHITE, TFT_BLACK); ty += 8;

    std::snprintf(buf, sizeof(buf), "ring:%d/%d",
                  ImageFlag.image_element_rings, ImageFlag.image_element_rings_flag);
    lq_tft18_drv_p6x8_str(0, ty, buf, TFT_YELLOW, TFT_BLACK); ty += 8;

    std::snprintf(buf, sizeof(buf), "mid:%d L:%d R:%d",
                  ImageStatus.MiddleLine,
                  ImageDeal[55].LeftBorder,
                  ImageDeal[55].RightBorder);
    lq_tft18_drv_p6x8_str(0, ty, buf, TFT_CYAN, TFT_BLACK);

    lq_tft18_drv_flush();
}

int lq_element_observe_demo(int display_mode)
{
    const bool use_udp = (display_mode == 0 || display_mode == 2);
    const bool use_tft = (display_mode == 1 || display_mode == 2);

    std::printf("[ELEMENT_OBSERVE] display_mode=%d (udp=%d tft=%d)\n",
                display_mode, use_udp ? 1 : 0, use_tft ? 1 : 0);

    Data_Settings();
    g_enable_element_process = false;
    std::printf("[ELEMENT_OBSERVE] element process DISABLED for NCNN observe\n");

    if (!image_cv_Init()) {
        std::printf("[ELEMENT_OBSERVE][FATAL] camera init failed\n");
        return 5;
    }

    // TFT 初始化
    if (use_tft) {
        lq_tft18_drv_init(1);  // 1=竖屏
        std::printf("[ELEMENT_OBSERVE] TFT18 initialized\n");
    }

    // UDP 初始化
    lq_udp_client udp_client;
    if (use_udp) {
        udp_client.udp_client_init(car_config::kNcnnObserveUdpHost, car_config::kNcnnObserveUdpPort);
        std::printf("[ELEMENT_OBSERVE] UDP stream to %s:%d\n",
                    car_config::kNcnnObserveUdpHost,
                    car_config::kNcnnObserveUdpPort);
    }
    const bool ncnn_observe_ready = use_udp &&
        car_config::kNcnnObserveEnable &&
        lq_ncnn_observe_start(car_config::kNcnnObserveUdpHost, car_config::kNcnnObserveUdpPort);
    const int ncnn_observe_interval = std::max(1, car_config::kNcnnObserveUdpEveryFrames);

    std::printf("[ELEMENT_OBSERVE] running... Press Ctrl+C to stop\n");

    uint32_t frame_count = 0;
    auto last_print = std::chrono::steady_clock::now();

    while (!exit_flag.load(std::memory_order_relaxed)) {
        if (!image_cv_zip()) {
            std::printf("[ELEMENT_OBSERVE][WARN] frame read failed\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        imageprocess();

        // UDP 图传
        if (use_udp) {
            lq_udp_center_line_send_one_frame(udp_client, 55);
            if (ncnn_observe_ready && (frame_count % ncnn_observe_interval) == 0) {
                lq_ncnn_observe_submit(image_cv_get_hires_frame(),
                                       static_cast<int>(frame_count),
                                       "element");
            }
        }

        // TFT 显示
        if (use_tft) {
            element_observe_draw_tft();
        }

        // 串口日志（每10帧打印一次，避免刷屏）
        ++frame_count;
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 200) {
            std::printf("[OBS] f:%lu road:%s det:%d err:%d off:%d ll:%d rl:%d tp:%d ring:%d/%d\n",
                        (unsigned long)frame_count,
                        element_road_name(ImageStatus.Road_type),
                        ImageStatus.Det_True,
                        ImageStatus.MiddleLine - ImageStatus.Det_True,
                        ImageStatus.OFFLine,
                        ImageStatus.Left_Line,
                        ImageStatus.Right_Line,
                        ImageStatus.TowPoint,
                        ImageFlag.image_element_rings,
                        ImageFlag.image_element_rings_flag);
            last_print = now;
        }
    }

    if (ncnn_observe_ready) {
        lq_ncnn_observe_stop();
    }
    return 0;
}
