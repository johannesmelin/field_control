import unittest

from field_control.odometry import DriveGeometry
from field_control.state_machine import SafetyConfig
from field_control.turn import DifferentialTurnPlan, in_row_turn_plan, new_row_turn_targets
from field_control.turn_controller import TurnController, TurnObservation


class TurnFoundationTests(unittest.TestCase):
    def test_turn_safety_configuration_rejects_invalid_values(self):
        with self.assertRaises(ValueError): SafetyConfig(new_row_turn_direction="up").validate()
        with self.assertRaises(ValueError): SafetyConfig(in_row_turn_wheel_degrees=0).validate()
        with self.assertRaises(ValueError): SafetyConfig(turn_heading_confirm_frames=0).validate()
        with self.assertRaises(ValueError): SafetyConfig(turn_heading_max_age_s=0).validate()

    def geometry(self): return DriveGeometry(left_wheel_circumference_m=1, right_wheel_circumference_m=2, wheel_track_m=1)

    def test_in_row_720_plan_mirrors_and_uses_wheel_geometry_only(self):
        left = in_row_turn_plan(self.geometry(), 720, "left")
        right = in_row_turn_plan(self.geometry(), 720, "right")
        self.assertEqual((left.left_distance_m, left.right_distance_m), (-2, 4))
        self.assertEqual((right.left_distance_m, right.right_distance_m), (2, -4))
        self.assertEqual((left.left_ratio, left.right_ratio), (-1, 1))

    def test_new_row_ratios_are_forward_and_not_gear_scaled(self):
        plan = new_row_turn_targets(self.geometry(), 2, 20, "right", .1)
        self.assertGreater(plan.left_ratio, 0); self.assertGreater(plan.right_ratio, 0)
        self.assertLessEqual(max(plan.left_ratio, plan.right_ratio), 1)
        # Wheel turns, not metres, determine the simultaneous-finish ratio.
        self.assertAlmostEqual(plan.left_ratio, 1.0)
        self.assertAlmostEqual(plan.right_ratio, 1 / 6)
        other_gear_ratio = DriveGeometry(left_wheel_circumference_m=1, right_wheel_circumference_m=2,
                                         wheel_track_m=1, motor_turns_per_wheel_turn=16)
        self.assertEqual((plan.left_ratio, plan.right_ratio),
                         (new_row_turn_targets(other_gear_ratio, 2, 20, "right", .1).left_ratio,
                          new_row_turn_targets(other_gear_ratio, 2, 20, "right", .1).right_ratio))

    def controller(self, **changes):
        plan = in_row_turn_plan(DriveGeometry(left_wheel_circumference_m=1, right_wheel_circumference_m=1), 720, "left")
        values = dict(initial_heading_deg=170, start_s=0, turn_speed_motor_rpm=20,
                      max_motor_rpm=15, timeout_s=5, distance_tolerance_m=.1,
                      heading_tolerance_deg=5, heading_confirm_frames=2, heading_max_age_s=.2)
        values.update(changes); return TurnController(plan, **values)

    def test_malformed_plan_is_rejected_before_any_tick_or_command(self):
        malformed = (
            DifferentialTurnPlan(float("nan"), 1, 1, 1, "left"),
            DifferentialTurnPlan(float("inf"), 1, 1, 1, "left"),
            DifferentialTurnPlan(0, 1, 0, 1, "left"),
            DifferentialTurnPlan(1, 1, 2, 1, "left"),
            DifferentialTurnPlan(1, -1, 1, 1, "left"),
            DifferentialTurnPlan(1, 1, .5, .5, "left"),
            DifferentialTurnPlan(1, 1, 1, 1, "up"),
        )
        for plan in malformed:
            with self.subTest(plan=plan), self.assertRaises(ValueError):
                TurnController(plan, initial_heading_deg=0, start_s=0, turn_speed_motor_rpm=1,
                               max_motor_rpm=1, timeout_s=1, distance_tolerance_m=0,
                               heading_tolerance_deg=0, heading_confirm_frames=1, heading_max_age_s=.2)

    def observation(self, **changes):
        values = dict(now_s=1, left_distance_m=-2, right_distance_m=2, heading_deg=-10,
                      heading_fresh=True, heading_timestamp_s=1, heading_sequence=1)
        values.update(changes); return TurnObservation(**values)

    def test_heading_wrap_success_requires_confirmations_and_motor_side_cap(self):
        controller = self.controller()
        first = controller.tick(self.observation())
        self.assertFalse(first.terminal); self.assertEqual((first.command.left_rpm, first.command.right_rpm), (-15, 15))
        repeated = controller.tick(self.observation(now_s=1.1))
        self.assertFalse(repeated.terminal)
        same_capture = controller.tick(self.observation(now_s=1.1, heading_sequence=2))
        self.assertFalse(same_capture.terminal)
        second = controller.tick(self.observation(now_s=1.1, heading_timestamp_s=1.1, heading_sequence=2))
        self.assertTrue(second.terminal); self.assertTrue(second.succeeded)

    def test_terminal_faults_are_fail_closed(self):
        cases = {
            "TURN_TIMEOUT": self.observation(now_s=5),
            "TURN_HEADING_STALE": self.observation(heading_fresh=False),
            "TURN_REVERSE": self.observation(left_distance_m=1),
            "TURN_OVERSHOOT": self.observation(left_distance_m=-2.2),
            "TURN_INVALID_OBSERVATION": self.observation(now_s=-1),
        }
        for expected, observation in cases.items():
            with self.subTest(expected=expected):
                decision = self.controller().tick(observation)
                self.assertTrue(decision.terminal); self.assertFalse(decision.succeeded)
                self.assertEqual(decision.fault, expected); self.assertIsNone(decision.command)

    def test_reverse_detection_uses_linear_tolerance_at_small_and_large_targets(self):
        for tolerance, actual in ((.1, .11), (10, 10.1)):
            decision = self.controller(distance_tolerance_m=tolerance).tick(self.observation(left_distance_m=actual))
            self.assertEqual(decision.fault, "TURN_REVERSE")

    def test_heading_timestamp_stale_faults(self):
        decision = self.controller().tick(self.observation(now_s=1, heading_timestamp_s=.7))
        self.assertEqual(decision.fault, "TURN_HEADING_STALE")
