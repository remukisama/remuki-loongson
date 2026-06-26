#pragma once

// Competition-only runtime tuning knobs.  These values are intentionally kept
// separate from the baseline car_config.hpp constants so T05/T11 PID tuning
// remains isolated from T06 curve-authority tuning.
namespace competition_tuning {

// Ordinary straight running keeps car_config::kMinWheelSpeedGoal (0.36 m/s).
// When a verified low-speed curve is already being entered, allow the inner
// wheel to roll down to this value.  This prevents the final physical clamp
// from reducing a 0.44 m/s curve to only 0.08 m/s of usable differential.
constexpr float kCurveMinWheelSpeedGoal = 0.20f;

// Do not release the inner wheel while still travelling at full straight speed.
// Normal T06 braking must first bring the active goal below this guard.
constexpr float kTurnAuthorityMaxSpeedGoal = 0.75f;

// Small but deliberate vision error is enough to release authority once the
// vehicle is already in the low-speed turn envelope.  These are not steering
// gains and should only be changed after inspecting T06 logs.
constexpr float kTurnAuthorityRawErrorThreshold = 2.0f;
constexpr float kTurnAuthorityFilteredErrorThreshold = 1.0f;
constexpr float kTurnAuthorityDerivativeThreshold = 1.8f;

}  // namespace competition_tuning
