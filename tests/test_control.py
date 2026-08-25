import unittest

from field_control.control import heading_command, vision_command
from field_control.motor_boundary import DisabledMotorBoundary, PhysicalOutputDisabled


class ControlTests(unittest.TestCase):
    def test_vision_is_bounded_by_both_correction_and_max_rpm(self):
        command = vision_command(1000, 0, 78, 1, 0, 10, 80)
        self.assertEqual((command.left_rpm, command.right_rpm), (80, 68))

    def test_heading_uses_shortest_direction_across_north(self):
        command = heading_command(1, 359, 10, 1, 0, 5, 80)
        self.assertEqual((command.left_rpm, command.right_rpm), (12, 8))

    def test_default_boundary_stops_and_refuses_output(self):
        boundary = DisabledMotorBoundary()
        with self.assertRaises(PhysicalOutputDisabled):
            boundary.command(vision_command(1, 0, 5, 1, 0, 5, 80))
        self.assertEqual(boundary.events[-1][0], "stop")


if __name__ == "__main__":
    unittest.main()
