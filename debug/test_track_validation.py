#!/usr/bin/env python3
"""Small offline tests for the feature-test analyzer; never contacts the board."""

from __future__ import annotations

import pathlib
import re
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import run_track_validation as validation  # noqa: E402


ROOT = pathlib.Path(__file__).resolve().parents[1]


def sample(host_t: float, target: float) -> dict[str, float]:
    left_inner = target > 0
    left_target, right_target = (0.20, 0.40) if left_inner else (0.40, 0.20)
    return {
        "host_t": host_t,
        "tgt": target,
        "raw": target * 0.4,
        "ge": 1.0,
        "goal": 0.30,
        "L": left_target,
        "R": right_target,
        "encL": left_target,
        "encR": right_target,
        "seL": 0.0,
        "seR": 0.0,
        "pwmL": -200.0 if left_inner else 100.0,
        "pwmR": 100.0 if left_inner else -200.0,
        "dt": 0.01,
        "db": 0.0,
        "off": 2.0,
        "rt": 1.0,
    }


class TrackValidationTests(unittest.TestCase):
    def test_live_capture_targets_board_and_times_active_telemetry(self) -> None:
        capture = (ROOT / "debug" / "run_car_live_capture.py").read_text(encoding="utf-8")
        self.assertIn('BOARD = "root@192.168.145.100"', capture)
        self.assertIn("board_hard_limit = args.duration + 12", capture)
        self.assertIn("active_duration >= args.duration", capture)
        self.assertIn('"active_duration_complete": int(active_duration_complete)', capture)
        self.assertIn('"active_telemetry_duration_s":', capture)

    def test_parse_steer(self) -> None:
        parsed = validation.parse_steer("steer,tgt=-20.0,encL=0.20,host_t=0.05")
        self.assertEqual(parsed, {"tgt": -20.0, "encL": 0.20, "host_t": 0.05})

    def test_small_s_two_reversals_passes_motion_gates(self) -> None:
        samples = [
            sample(index * 0.05, target)
            for index, target in enumerate([-20.0] * 9 + [20.0] * 9 + [-20.0] * 9 + [20.0] * 9)
        ]
        thermal = {"status": "PASS", "points": 5, "values": {}, "violations": []}
        result = validation.analyze(validation.CASES["small_s"], samples, {}, thermal)
        gates = {gate["name"]: gate["status"] for gate in result["gates"]}
        self.assertEqual(result["verdict"], "PASS")
        self.assertEqual(gates["s_reversals"], "PASS")
        self.assertEqual(result["metrics"]["target_turn_blocks"], [-1, 1, -1, 1])

    def test_small_s_one_turn_is_review(self) -> None:
        samples = [sample(index * 0.05, 20.0) for index in range(12)]
        thermal = {"status": "PASS", "points": 5, "values": {}, "violations": []}
        result = validation.analyze(validation.CASES["small_s"], samples, {}, thermal)
        gates = {gate["name"]: gate["status"] for gate in result["gates"]}
        self.assertEqual(result["verdict"], "REVIEW")
        self.assertEqual(gates["s_reversals"], "REVIEW")

    def test_cross_uses_pure_centerline_without_element_state(self) -> None:
        samples = [sample(index * 0.05, 0.0) for index in range(40)]
        thermal = {"status": "PASS", "points": 5, "values": {}, "violations": []}
        result = validation.analyze(validation.CASES["cross"], samples, {}, thermal)
        gates = {gate["name"]: gate["status"] for gate in result["gates"]}
        self.assertEqual(gates["cross_element_disabled"], "PASS")
        self.assertEqual(gates["line_retention"], "PASS")

        samples[20]["rt"] = 3.0
        result = validation.analyze(validation.CASES["cross"], samples, {}, thermal)
        gates = {gate["name"]: gate["status"] for gate in result["gates"]}
        self.assertEqual(gates["cross_element_disabled"], "FAIL")

    def test_race_source_disables_cross_special_case_and_enables_curve_center(self) -> None:
        config = (ROOT / "user_app" / "include" / "car_config.hpp").read_text(encoding="utf-8")
        image = (ROOT / "user_app" / "src" / "image.cpp").read_text(encoding="utf-8")
        self.assertIn("constexpr bool kBasicLineBaselineMode = false;", config)
        self.assertIn("constexpr bool kBasicCrossRepairEnable = false;", config)
        self.assertIn(
            "!g_enable_element_process && car_config::kBasicCrossRepairEnable && !repaired_cross",
            image,
        )
        motor = (ROOT / "user_app" / "src" / "my_motor.cpp").read_text(encoding="utf-8")
        self.assertIn("} else if (both_sides_weak) {", motor)
        self.assertIn("speed_to_plan = car_config::kDegradedSpeedGoal;", motor)

    def test_single_side_reconstruction_keeps_the_geometric_center_unbiased(self) -> None:
        config = (ROOT / "user_app" / "include" / "car_config.hpp").read_text(encoding="utf-8")
        image = (ROOT / "user_app" / "src" / "image.cpp").read_text(encoding="utf-8")
        self.assertIn("constexpr int kCurveCenterSingleSideBias = 0;", config)
        self.assertIn(
            "row, ImageDeal[row].RightBorder, -car_config::kCurveCenterSingleSideBias",
            image,
        )
        self.assertIn(
            "row, ImageDeal[row].LeftBorder, car_config::kCurveCenterSingleSideBias",
            image,
        )

    def test_gyro_tuning_keeps_diff_in_visual_turn_direction_on_logged_overshoot(self) -> None:
        config = (ROOT / "user_app" / "include" / "car_config.hpp").read_text(encoding="utf-8")

        def config_float(name: str) -> float:
            match = re.search(rf"constexpr float {name} = ([0-9.]+)f;", config)
            self.assertIsNotNone(match, name)
            return float(match.group(1))

        gyro_p = config_float("kGyroPidP")
        feedforward = config_float("kGyroFeedforwardGain")
        # Representative same-direction overshoots from car_live_text_20260713_194215.log.
        for target, filtered_gyro in ((-30.0, -41.2), (-42.0, -55.5), (34.0, 47.3)):
            diff = feedforward * target + gyro_p * (target - filtered_gyro)
            self.assertGreater(diff * target, 0.0)

    def test_drv8701e_active_reverse_is_limited_to_turn_inside_wheel(self) -> None:
        config = (ROOT / "user_app" / "include" / "car_config.hpp").read_text(encoding="utf-8")
        motor = (ROOT / "user_app" / "src" / "my_motor.cpp").read_text(encoding="utf-8")
        self.assertIn("constexpr float kSpeedBrakeMaxPwm = 0.0f;", config)
        self.assertIn("constexpr float kSpeedTurnBrakeMaxPwm = -1200.0f;", config)
        self.assertEqual(motor.count("const bool turn_inside_wheel ="), 2)
        self.assertEqual(motor.count("raw_out = car_config::kSpeedBrakeMaxPwm;"), 2)

    def test_new_driver_keeps_loaded_turn_feedforward(self) -> None:
        config = (ROOT / "user_app" / "include" / "car_config.hpp").read_text(encoding="utf-8")
        motor = (ROOT / "user_app" / "src" / "my_motor.cpp").read_text(encoding="utf-8")
        self.assertIn("constexpr float kSpeedPidFeedForwardPerMS = 4500.0f;", config)
        self.assertEqual(motor.count("kSpeedPidFeedForwardPerMS * Speed_Goal_"), 2)
        self.assertIn("if (speed_erro_L > 0.0f)", motor)
        self.assertIn("if (speed_erro_R > 0.0f)", motor)

    def test_near_center_exit_correction_requires_trustworthy_edges(self) -> None:
        config = (ROOT / "user_app" / "include" / "car_config.hpp").read_text(encoding="utf-8")
        motor = (ROOT / "user_app" / "src" / "my_motor.cpp").read_text(encoding="utf-8")
        self.assertIn("constexpr float kNearCenterSteerGainDpsPerPx = 1.10f;", config)
        self.assertIn("constexpr float kNearCenterSteerLargeErrorThresholdPx = 7.0f;", config)
        self.assertIn("constexpr float kNearCenterSteerLargeErrorGainDpsPerPx = 1.80f;", config)
        self.assertIn("constexpr float kNearCenterPersistentEnterErrorPx = 6.0f;", config)
        self.assertIn("constexpr float kNearCenterPersistentExitErrorPx = 4.0f;", config)
        self.assertIn("constexpr int kNearCenterPersistentConfirmMs = 250;", config)
        self.assertIn("constexpr float kNearCenterPersistentRiseDpsPerSec = 8.0f;", config)
        self.assertIn("constexpr float kNearCenterPersistentDecayDpsPerSec = 32.0f;", config)
        self.assertIn("constexpr float kNearCenterPersistentLimitDps = 10.0f;", config)
        self.assertIn("local_vision.NearLeftFound && local_vision.NearRightFound", motor)
        self.assertIn("local_vision.OFFLine <= car_config::kNearCenterSteerMaxOffline", motor)
        self.assertIn("near_center_error * steer_error < 0.0f", motor)
        self.assertNotIn("near_opposes_far", motor)
        self.assertIn("near_center_far_not_opposing", motor)
        self.assertIn("near_center_large_correction", motor)
        self.assertIn("const bool persistent_geometry_valid =", motor)
        self.assertIn("near_far_protection_active || near_far_recovery_guard_active", motor)
        self.assertIn("(visual_dt < 0.020f) ? visual_dt : 0.020f", motor)
        self.assertIn("persistent_direction_conflict", motor)
        self.assertIn("persistent_base_near_saturation", motor)
        self.assertIn("visual_without_persistent + near_center_persistent_add_dps", motor)
        self.assertIn("nBias=%.1f", motor)
        self.assertIn("far_confidence * far_pid + near_center_add", motor)

        def gain(near_error: float, far_error: float, protected: bool = False) -> float:
            far_not_opposing = abs(far_error) <= 2 or near_error * far_error >= 0
            large = protected or (abs(near_error) >= 7 and far_not_opposing)
            return 1.80 if large else 1.10

        self.assertEqual(gain(9.0, 10.0), 1.80)
        self.assertEqual(gain(-9.0, -10.0), 1.80)
        self.assertEqual(gain(5.0, 0.0), 1.10)
        self.assertEqual(gain(-8.0, 10.0), 1.10)
        self.assertEqual(gain(6.0, 0.0, protected=True), 1.80)

        # 50ms离线步进：250ms前无补偿，随后以8dps/s爬升并限幅10dps；
        # S弯换向或NF必须立即清零，不能把旧弯向带入下一段。
        persistent = 0.0
        sign = 0.0
        confirmed_ms = 0

        def persistent_step(candidate_sign: float, candidate: bool, hard_reset: bool) -> float:
            nonlocal persistent, sign, confirmed_ms
            if hard_reset:
                persistent = 0.0
                sign = 0.0
                confirmed_ms = 0
            elif candidate:
                if sign * candidate_sign <= 0:
                    persistent = 0.0
                    sign = candidate_sign
                    confirmed_ms = 0
                confirmed_ms += 50
                if confirmed_ms > 250:
                    persistent = max(-10.0, min(10.0, persistent + sign * 8.0 * 0.02))
            else:
                confirmed_ms = 0
                decay = 32.0 * 0.02
                persistent = max(0.0, persistent - decay) if persistent > 0 else min(0.0, persistent + decay)
                if persistent == 0:
                    sign = 0.0
            return persistent

        self.assertEqual([persistent_step(1.0, True, False) for _ in range(5)][-1], 0.0)
        self.assertGreater(persistent_step(1.0, True, False), 0.0)
        for _ in range(100):
            persistent_step(1.0, True, False)
        self.assertEqual(persistent, 10.0)
        self.assertEqual(persistent_step(-1.0, False, True), 0.0)
        self.assertEqual(sign, 0.0)

    def test_near_far_conflict_holds_near_center_until_corridor_reconnects(self) -> None:
        config = (ROOT / "user_app" / "include" / "car_config.hpp").read_text(encoding="utf-8")
        motor = (ROOT / "user_app" / "src" / "my_motor.cpp").read_text(encoding="utf-8")
        self.assertIn("constexpr float kNearFarConflictMinFarErrorPx = 6.0f;", config)
        self.assertIn("constexpr int kNearFarConflictSideLossMin = 12;", config)
        self.assertIn("constexpr float kNearFarConflictFarScale = 0.0f;", config)
        self.assertIn("constexpr int kNearFarConflictMaxHoldMs = 2300;", config)
        self.assertIn("constexpr int kNearFarConflictReleaseCenterGapPx = 5;", config)
        self.assertIn("constexpr int kNearFarConflictOutletGapMaxPx = 18;", config)
        self.assertIn("constexpr int kNearFarConflictReleaseFrames = 3;", config)
        self.assertIn("constexpr float kNearFarConflictEntryHoldRatio = 0.80f;", config)
        self.assertIn("constexpr int kNearFarConflictRecoveryBlendMs = 200;", config)
        self.assertIn("constexpr int kNearFarConflictRecoveryGuardMs = 600;", config)
        self.assertIn(
            "(steer_error < 0.0f) ? local_vision.Left_Line : local_vision.Right_Line",
            motor,
        )
        self.assertIn(
            "far_conflict_side_loss > car_config::kNearFarConflictSideLossMin",
            motor,
        )
        self.assertIn("std::abs(local_vision.TowCenter - local_vision.NearCenter)", motor)
        self.assertIn("const bool release_far_centered =", motor)
        self.assertIn(
            "std::fabs(raw_steer_error) < car_config::kNearFarConflictMinFarErrorPx",
            motor,
        )
        self.assertIn("steer_error * near_far_conflicting_sign < 0.0f", motor)
        self.assertIn(
            "local_vision.TowWidth <= car_config::kNearFarConflictReleaseTowWidthMaxPx",
            motor,
        )
        self.assertIn(
            "near_far_release_confirm_frames >= car_config::kNearFarConflictReleaseFrames",
            motor,
        )
        self.assertIn("far_confidence = car_config::kNearFarConflictFarScale;", motor)
        self.assertIn("near_far_recovery_blend_active", motor)
        self.assertIn("near_far_recovery_corridor_invalid", motor)
        self.assertIn(
            "near_far_recovery_guard_active && !tow_corridor_valid",
            motor,
        )
        self.assertIn("near_far_entry_near_error = previous_near_error;", motor)
        self.assertIn("near_control_error = entry_hold_error;", motor)
        self.assertIn("near_far_protection_active", motor)
        self.assertIn(
            "speed_to_plan > car_config::kCrossExitSpeedGoal",
            motor,
        )
        self.assertEqual(
            motor.count("speed_to_plan > car_config::kCrossExitSpeedGoal"),
            2,
        )
        self.assertNotIn("!conflict_geometry_persists", motor)
        self.assertIn("guarded_turn_sign = 0.0f;", motor)
        self.assertIn("fSc=%.2f,nfAmb=%d,nfRel=%d", motor)
        self.assertIn("constexpr bool kBasicCrossRepairEnable = false;", config)

        # Eleven logged ambiguity starts: the first ten lose the boundary that
        # explains the suspect far-center shift; the fatal 212358 bend loses
        # only the same-side boundary and must not arm the ambiguity mask.
        conflict_starts = [
            (15, 11, 31), (15, 27, 21), (15, 23, 14), (7, 0, 38), (-7, 26, 3),
            (-6, 25, 0), (-7, 26, 0), (-6, 37, 7), (-11, 27, 0), (-10, 37, 8),
            (-5, 0, 15),
        ]

        def directional_loss(far_error: int, left_loss: int, right_loss: int) -> int:
            return left_loss if far_error < 0 else right_loss

        selected_losses = [directional_loss(*start) for start in conflict_starts]
        self.assertTrue(all(loss > 12 for loss in selected_losses[:-1]))
        self.assertEqual(selected_losses[-1], 0)

        # 214730 两次真实十字在入口前都已右偏；进入横向白区后，近端误差
        # 大量变为假 0。锁存前一可信误差的较大值，并在当前误差缺失或同向
        # 但更弱时保留 60%，既持续回中，也不覆盖真实的反向近端修正。
        def held_near_error(previous: float, current: float) -> float:
            entry = previous if previous * current > 0 and abs(previous) > abs(current) else current
            hold = entry * 0.80
            if abs(current) <= 2 or (current * hold > 0 and abs(current) < abs(hold)):
                return hold
            return current

        self.assertAlmostEqual(held_near_error(10.0, 0.0), 0.0)
        self.assertAlmostEqual(held_near_error(10.0, 6.0), 8.0)

        def hold_after_trigger(entry: float, current: float) -> float:
            hold = entry * 0.80
            if abs(current) <= 2 or (current * hold > 0 and abs(current) < abs(hold)):
                return hold
            return current

        self.assertAlmostEqual(hold_after_trigger(10.0, 0.0), 8.0)
        self.assertAlmostEqual(hold_after_trigger(10.0, 4.0), 8.0)
        self.assertAlmostEqual(hold_after_trigger(10.0, -5.0), -5.0)

        def nf_speed_cap(
            requested: float,
            ambiguous: bool,
            blending: bool,
            recovery_corridor_invalid: bool,
        ) -> float:
            if ambiguous or blending or recovery_corridor_invalid:
                return min(requested, 0.26)
            return requested

        self.assertEqual(nf_speed_cap(0.74, True, False, False), 0.26)
        self.assertEqual(nf_speed_cap(0.50, False, True, False), 0.26)
        self.assertEqual(nf_speed_cap(0.50, False, False, True), 0.26)
        self.assertEqual(nf_speed_cap(0.50, False, False, False), 0.50)

        # 105453首次出口在200ms渐变结束后，前瞻走廊于260~415ms再次失效。
        # 600ms核验窗只屏蔽这些无效帧；467ms时有效出口仍可立即全量接管。
        def recovery_far_scale(age_ms: int, corridor_valid: bool) -> float:
            guard_active = 0 <= age_ms < 600
            blend_active = guard_active and age_ms < 200
            corridor_invalid = guard_active and not corridor_valid
            if corridor_invalid:
                return 0.0
            if blend_active:
                return age_ms / 200.0
            return 1.0

        self.assertEqual(recovery_far_scale(260, False), 0.0)
        self.assertEqual(recovery_far_scale(415, False), 0.0)
        self.assertEqual(recovery_far_scale(467, True), 1.0)
        self.assertEqual(recovery_far_scale(601, False), 1.0)

        # 204403 的错误释放帧：loss 已降到 11，但前/近中心仍差 21px，
        # 且前瞻宽度达 67px（横向支路），不得据此释放。只有真正出口走廊
        # 连续三帧重新对齐后才允许交还远端。
        logged_false_recovery = [(21, 67, -18), (18, 67, -18), (19, 68, -18)]
        aligned_recovery = [(5, 31, 2), (4, 30, 1), (3, 29, 0)]
        premature_112404_recovery = [(5, 26, -18)] * 3

        def confirms(frames: list[tuple[int, int, int]]) -> bool:
            count = 0
            for center_gap, tow_width, raw_error in frames:
                valid = center_gap <= 5 and 10 <= tow_width <= 42 and abs(raw_error) < 6
                count = count + 1 if valid else 0
            return count >= 3

        self.assertFalse(confirms(logged_false_recovery))
        self.assertFalse(confirms(premature_112404_recovery))
        self.assertTrue(confirms(aligned_recovery))

        # 211414 的真实出口在硬超时前约0.8s已翻到原错误支路的反方向：
        # 只有方向翻转、前瞻走廊有效且远近差不超过18px连续三帧才提前释放。
        wrong_far_sign = -1
        outlet_sequence = [
            (-20, 21, 67),
            (-15, 12, 33),
            (7, 9, 26),
            (9, 9, 26),
            (11, 10, 26),
        ]
        confirm_count = 0
        release_index = None
        for index, (far_error, center_gap, tow_width) in enumerate(outlet_sequence):
            corridor_valid = 10 <= tow_width <= 42
            reversed_outlet = (
                corridor_valid and abs(far_error) >= 6
                and far_error * wrong_far_sign < 0 and center_gap <= 18
            )
            confirm_count = confirm_count + 1 if reversed_outlet else 0
            if confirm_count >= 3:
                release_index = index
                break

        self.assertEqual(release_index, 4)


if __name__ == "__main__":
    unittest.main()
