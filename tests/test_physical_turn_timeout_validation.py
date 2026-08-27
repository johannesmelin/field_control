"""Fail-closed timeout validation for physical A4 position turns."""
from __future__ import annotations

import math
import unittest

from field_control.config import PhysicalCanConfig, RuntimeConfig
from field_control.odometry import DriveGeometry
from field_control.state_machine import SafetyConfig


def physical_can() -> PhysicalCanConfig:
    return PhysicalCanConfig(
        True, "can0", "observed-rmdx-same-id", "/dev/serial/by-id/test-canable", True, True,
    )


class PhysicalA4TurnTimeoutTests(unittest.TestCase):
    def test_physical_output_requires_exactly_one_explicit_operating_context(self):
        base = dict(enabled=True, channel="can0", reply_profile="observed-rmdx-same-id",
                    slcan_device="/dev/serial/by-id/test-canable",
                    confirm_physical_stop_tested=True)
        PhysicalCanConfig(**base, confirm_wheels_raised=True).validate()
        PhysicalCanConfig(**base, confirm_ground_test=True,
                          confirm_ground_clear=True,
                          confirm_emergency_stop_ready=True).validate()
        for changes in (
            {},
            {"confirm_ground_test": True},
            {"confirm_ground_test": True, "confirm_ground_clear": True},
            {"confirm_wheels_raised": True, "confirm_ground_test": True,
             "confirm_ground_clear": True, "confirm_emergency_stop_ready": True},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                PhysicalCanConfig(**base, **changes).validate()

    def test_720_wheel_degree_turn_at_10_motor_rpm_requires_96_seconds_plus_margin(self):
        config = RuntimeConfig(
            stream_enabled=False, max_rpm=10, turn_speed_rpm=10,
            physical_can=physical_can(),
            safety=SafetyConfig(in_row_turn_enabled=True, turn_timeout_s=106),
        )
        self.assertEqual(config.minimum_physical_a4_turn_timeout_s(), 106.0)
        config.validate()
        with self.assertRaisesRegex(ValueError, "turn_timeout_s är för kort"):
            RuntimeConfig(
                stream_enabled=False, max_rpm=10, turn_speed_rpm=10,
                physical_can=physical_can(),
                safety=SafetyConfig(in_row_turn_enabled=True, turn_timeout_s=105.999),
            ).validate()

    def test_motor_rpm_cap_is_part_of_the_deadline(self):
        config = RuntimeConfig(
            stream_enabled=False, max_rpm=5, turn_speed_rpm=10,
            physical_can=physical_can(),
            safety=SafetyConfig(in_row_turn_enabled=True, turn_timeout_s=202),
        )
        self.assertEqual(config.minimum_physical_a4_turn_timeout_s(), 202.0)
        config.validate()

    def test_new_row_uses_the_largest_directional_wheel_target(self):
        geometry = DriveGeometry(
            left_wheel_circumference_m=2, right_wheel_circumference_m=1,
            wheel_track_m=1, motor_turns_per_wheel_turn=1,
        )
        left = RuntimeConfig(
            stream_enabled=False, max_rpm=60, turn_speed_rpm=60, row_spacing_m=3,
            odometry_geometry=geometry, physical_can=physical_can(),
            safety=SafetyConfig(number_of_rows=2, new_row_turn_direction="left", turn_timeout_s=math.pi + 10),
        )
        right = RuntimeConfig(
            stream_enabled=False, max_rpm=60, turn_speed_rpm=60, row_spacing_m=3,
            odometry_geometry=geometry, physical_can=physical_can(),
            safety=SafetyConfig(number_of_rows=2, new_row_turn_direction="right", turn_timeout_s=math.pi + 10),
        )
        self.assertAlmostEqual(left.minimum_physical_a4_turn_timeout_s(), 2 * math.pi + 10)
        self.assertAlmostEqual(right.minimum_physical_a4_turn_timeout_s(), math.pi + 10)
        right.validate()
        with self.assertRaisesRegex(ValueError, "turn_timeout_s är för kort"):
            left.validate()

    def test_nonphysical_configuration_keeps_existing_pure_turn_timeout_semantics(self):
        config = RuntimeConfig(
            stream_enabled=False, max_rpm=10, turn_speed_rpm=10,
            safety=SafetyConfig(in_row_turn_enabled=True, turn_timeout_s=8),
        )
        self.assertIsNone(config.minimum_physical_a4_turn_timeout_s())
        config.validate()


if __name__ == "__main__":
    unittest.main()
