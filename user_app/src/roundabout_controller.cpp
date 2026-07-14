#include "roundabout_controller.hpp"
#include "car_config.hpp"

#include <cstdlib>

namespace roundabout {

void Controller::reset()
{
    side_ = Side::None;
    phase_ = Phase::Idle;
    phase_started_ms_ = 0;
    phase_frames_ = 0;
    phase_age_ms_ = 0;
    confirm_frames_ = 0;
    opening_wide_seen_ = false;
}

void Controller::begin(Side side, long long now_ms)
{
    side_ = side;
    phase_ = Phase::Approach;
    phase_started_ms_ = now_ms;
    phase_frames_ = 0;
    phase_age_ms_ = 0;
    confirm_frames_ = 0;
    opening_wide_seen_ = false;
}

bool Controller::confirm(bool candidate, int frames_needed)
{
    if (!candidate) {
        confirm_frames_ = 0;
        return false;
    }
    if (confirm_frames_ < frames_needed) ++confirm_frames_;
    return confirm_frames_ >= frames_needed;
}

void Controller::transition(Phase next, long long now_ms, const char*)
{
    phase_ = next;
    phase_started_ms_ = now_ms;
    phase_frames_ = 0;
    phase_age_ms_ = 0;
    confirm_frames_ = 0;
}

Update Controller::snapshot(bool changed, bool finished, const char* reason) const
{
    Update result;
    result.side = side_;
    result.phase = phase_;
    result.changed = changed;
    result.finished = finished;
    result.phase_frames = phase_frames_;
    result.phase_age_ms = phase_age_ms_;
    result.confirm_frames = confirm_frames_;
    result.reason = reason;
    return result;
}

Update Controller::update(const Observation& observation)
{
    if (!active()) return snapshot(false, false, "idle");

    ++phase_frames_;
    long long age = observation.now_ms - phase_started_ms_;
    if (age < 0) age = 0;
    if (age > 100000) age = 100000;
    phase_age_ms_ = static_cast<int>(age);

    bool changed = false;
    bool finished = false;
    bool aborted = false;
    const char* reason = "hold";

    switch (phase_) {
    case Phase::Approach: {
        const bool lower_edge_arrived =
            observation.inner_lower_missing >= car_config::kRingApproachLowerLostMin;
        const bool approach_timeout =
            phase_age_ms_ >= car_config::kRingApproachForcePassMs;
        const bool ready =
            phase_age_ms_ >= car_config::kRingApproachMinMs &&
            (lower_edge_arrived || approach_timeout);
        if (confirm(ready, car_config::kRingPhaseConfirmFrames)) {
            reason = lower_edge_arrived ? "approach-edge" : "approach-time";
            transition(Phase::PassOpening, observation.now_ms, reason);
            changed = true;
        }
        break;
    }
    case Phase::PassOpening: {
        if (observation.inner_lower_missing >=
            car_config::kRingPassOpeningPeakMissingMin) {
            opening_wide_seen_ = true;
        }
        const bool opening_cleared =
            opening_wide_seen_ &&
            observation.inner_lower_missing <=
                car_config::kRingPassOpeningRecoveredMax &&
            observation.inner_tow_found;
        const bool timed_out =
            phase_age_ms_ >= car_config::kRingPassOpeningForceSeekMs;
        const bool ready =
            phase_age_ms_ >= car_config::kRingPassOpeningMinMs &&
            (opening_cleared || timed_out);
        if (confirm(ready, car_config::kRingPhaseConfirmFrames)) {
            reason = opening_cleared ?
                "first-opening-cleared" : "first-opening-timeout";
            transition(Phase::SeekEntrance, observation.now_ms, reason);
            changed = true;
        }
        break;
    }
    case Phase::SeekEntrance: {
        const bool entrance_reached_control_area =
            observation.inner_lower_missing >=
                car_config::kRingEntranceLowerLostMin ||
            observation.inner_line_loss >=
                car_config::kRingEntranceLineLostMin;
        const bool entrance_open =
            phase_age_ms_ >= car_config::kRingSeekEntranceMinMs &&
            !observation.inner_tow_found &&
            entrance_reached_control_area;
        const bool timed_out =
            phase_age_ms_ >= car_config::kRingSeekEntranceAbortMs;
        if (confirm(entrance_open || timed_out,
                    car_config::kRingPhaseConfirmFrames)) {
            if (timed_out && !entrance_open) {
                reason = "entrance-evidence-timeout";
                aborted = true;
                break;
            }
            reason = "entrance-opening";
            transition(Phase::Enter, observation.now_ms, reason);
            changed = true;
        }
        break;
    }
    case Phase::Enter: {
        const bool outer_lost =
            observation.outer_line_loss > car_config::kRingEnterOuterLostHigh;
        const bool timed_out =
            phase_age_ms_ >= car_config::kRingEnterLossWaitMaxMs;
        if (confirm(outer_lost || timed_out, car_config::kRingPhaseConfirmFrames)) {
            reason = outer_lost ? "outer-lost" : "enter-loss-timeout";
            transition(Phase::EnterRecover, observation.now_ms, reason);
            changed = true;
        }
        break;
    }
    case Phase::EnterRecover: {
        const bool outer_recovered =
            observation.outer_line_loss < car_config::kRingEnterOuterLostLow;
        const bool timed_out =
            phase_age_ms_ >= car_config::kRingEnterRecoverMaxMs;
        if (confirm(outer_recovered || timed_out, car_config::kRingPhaseConfirmFrames)) {
            reason = outer_recovered ? "outer-recovered" : "enter-recover-timeout";
            transition(Phase::Orbit, observation.now_ms, reason);
            changed = true;
        }
        break;
    }
    case Phase::Orbit: {
        const bool exit_yaw_ready =
            phase_age_ms_ >= car_config::kRingInnerMinStageMs &&
            observation.turn_yaw_deg >= car_config::kRingOrbitExitMinYawDeg;
        const bool exit_geometry_ready =
            exit_yaw_ready ||
            phase_age_ms_ >= car_config::kRingOrbitExitGeometryMinMs;
        const bool exit_corner =
            exit_geometry_ready &&
            observation.orbit_exit_corner &&
            observation.outer_line_loss > car_config::kRingEnterOuterLostHigh;
        const bool exit_lane_open =
            exit_geometry_ready &&
            observation.exit_lane_visible &&
            observation.inner_line_loss >=
                car_config::kRingOrbitExitInnerLostMin;
        // 153312图传证明：约294度时外侧边界虽已恢复，但画面仍是完整左弯；
        // 此时降到出口横摆上限会直接冲出弧线。边界恢复不能单独作为出口。
        const bool yaw_fallback =
            observation.turn_yaw_deg >= car_config::kRingOrbitForceExitYawDeg;
        if (confirm(exit_corner || exit_lane_open || yaw_fallback,
                    car_config::kRingPhaseConfirmFrames)) {
            reason = exit_corner ? "exit-corner-yaw" :
                (exit_lane_open ? "exit-lane-opening-yaw" :
                                  "exit-yaw-fallback");
            transition(Phase::Exit, observation.now_ms, reason);
            changed = true;
        }
        break;
    }
    case Phase::Exit: {
        const bool exit_lane_ready =
            phase_age_ms_ >= car_config::kRingExitSearchMinMs &&
            observation.exit_lane_visible;
        const bool timed_out =
            phase_age_ms_ >= car_config::kRingExitSearchMaxMs;
        if (confirm(exit_lane_ready || timed_out,
                    car_config::kRingPhaseConfirmFrames)) {
            reason = exit_lane_ready ? "exit-lane" : "exit-search-timeout";
            transition(Phase::Align, observation.now_ms, reason);
            changed = true;
        }
        break;
    }
    case Phase::Align: {
        const bool centered =
            phase_age_ms_ >= car_config::kRingExitAlignMinMs &&
            observation.exit_lane_visible &&
            observation.inner_line_loss <=
                car_config::kRingAlignInnerLossMax &&
            std::abs(observation.center_error) <= car_config::kRingExitCenterErrorMax;
        const bool recovered_lane_timeout =
            phase_age_ms_ >= car_config::kRingExitAlignMaxMs &&
            observation.exit_lane_visible &&
            observation.inner_line_loss <=
                car_config::kRingAlignInnerLossMax;
        if (confirm(centered || recovered_lane_timeout,
                    car_config::kRingPhaseConfirmFrames)) {
            reason = centered ? "exit-centered" : "exit-lane-recovered";
            finished = true;
        } else if (phase_age_ms_ >= car_config::kRingExitAlignMaxMs) {
            // 152004实车在OFF=55、两侧边界均丢失时走到这里。超时只能说明
            // 对正失败，不能把失败伪装成完成并把控制权交回普通巡线。
            reason = "exit-align-wait-lane";
        }
        break;
    }
    case Phase::Idle:
        break;
    }

    Update result = snapshot(changed, finished, reason);
    result.aborted = aborted;
    if (aborted) reset();
    return result;
}

const char* phase_name(Phase phase)
{
    switch (phase) {
    case Phase::Idle: return "idle";
    case Phase::Approach: return "approach";
    case Phase::PassOpening: return "pass-opening";
    case Phase::SeekEntrance: return "seek-entrance";
    case Phase::Enter: return "enter";
    case Phase::EnterRecover: return "enter-recover";
    case Phase::Orbit: return "orbit";
    case Phase::Exit: return "exit";
    case Phase::Align: return "align";
    }
    return "unknown";
}

}  // namespace roundabout
