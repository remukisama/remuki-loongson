#ifndef ROUNDABOUT_CONTROLLER_HPP
#define ROUNDABOUT_CONTROLLER_HPP

#include <cstdint>

namespace roundabout {

enum class Side : std::uint8_t {
    None = 0,
    Left = 1,
    Right = 2,
};

// 数值继续兼容原有 UDP/TFT 的 ring stage，避免调试工具同时失效。
enum class Phase : std::uint8_t {
    Idle = 0,
    Approach = 1,
    PassOpening = 2,
    SeekEntrance = 3,
    Enter = 5,
    EnterRecover = 6,
    Orbit = 7,
    Exit = 8,
    Align = 9,
};

struct Observation {
    long long now_ms = 0;
    int inner_lower_missing = 0;
    int inner_line_loss = 0;
    int outer_line_loss = 0;
    int offline = 0;
    int center_error = 0;
    float turn_yaw_deg = 0.0f;
    bool inner_tow_found = false;
    bool orbit_exit_corner = false;
    bool exit_lane_visible = false;
};

struct Update {
    Side side = Side::None;
    Phase phase = Phase::Idle;
    bool changed = false;
    bool finished = false;
    bool aborted = false;
    int phase_frames = 0;
    int phase_age_ms = 0;
    int confirm_frames = 0;
    const char* reason = "idle";
};

class Controller {
public:
    void reset();
    void begin(Side side, long long now_ms);
    Update update(const Observation& observation);

    Side side() const { return side_; }
    Phase phase() const { return phase_; }
    bool active() const { return side_ != Side::None && phase_ != Phase::Idle; }

private:
    bool confirm(bool candidate, int frames_needed);
    void transition(Phase next, long long now_ms, const char* reason);
    Update snapshot(bool changed, bool finished, const char* reason) const;

    Side side_ = Side::None;
    Phase phase_ = Phase::Idle;
    long long phase_started_ms_ = 0;
    int phase_frames_ = 0;
    int phase_age_ms_ = 0;
    int confirm_frames_ = 0;
    bool opening_wide_seen_ = false;
};

const char* phase_name(Phase phase);

}  // namespace roundabout

#endif
