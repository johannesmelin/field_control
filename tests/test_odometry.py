import unittest

from field_control.odometry import DriveGeometry, from_motor_angles


class OdometryTests(unittest.TestCase):
    def test_eight_motor_turns_is_one_wheel_turn_for_both_sides(self):
        geometry = DriveGeometry()
        sample = from_motor_angles(0, 0, 2880, -2880, geometry)
        self.assertAlmostEqual(sample.left_distance_m, .805)
        self.assertAlmostEqual(sample.right_distance_m, .805)
        self.assertAlmostEqual(sample.forward_distance_m, .805)
        self.assertAlmostEqual(sample.yaw_change_deg, 0)

    def test_separate_wheel_circumferences_and_track_are_used(self):
        geometry = DriveGeometry(left_wheel_circumference_m=.8, right_wheel_circumference_m=.82,
                                 wheel_track_m=1, motor_turns_per_wheel_turn=1,
                                 left_forward_sign=1, right_forward_sign=1)
        sample = from_motor_angles(0, 0, 360, 720, geometry)
        self.assertAlmostEqual(sample.forward_distance_m, 1.22)
        self.assertAlmostEqual(sample.yaw_change_deg, 48.128455, places=5)


if __name__ == "__main__":
    unittest.main()
