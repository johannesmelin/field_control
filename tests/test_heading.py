import unittest

from field_control.heading import RowHeadingReference, circular_low_pass


class HeadingTests(unittest.TestCase):
    def test_filter_crosses_north_without_large_jump(self):
        self.assertAlmostEqual(circular_low_pass(359, 1, .5), 0)

    def test_reference_uses_circular_mean_and_minimum_valid_distance(self):
        reference = RowHeadingReference(window_m=2, minimum_distance_m=1)
        reference.add_visual_heading(359, 0)
        reference.add_visual_heading(1, .5)
        self.assertFalse(reference.reliable)
        reference.add_visual_heading(0, 1)
        self.assertTrue(reference.reliable)
        self.assertAlmostEqual(reference.reference_deg, 0, places=5)

    def test_successful_turn_derives_opposite_reliable_reference(self):
        reference = RowHeadingReference(window_m=2, minimum_distance_m=1)
        reference.add_visual_heading(45, 0); reference.add_visual_heading(45, 1)
        self.assertEqual(reference.apply_successful_180_turn(), 225)
        self.assertTrue(reference.reliable)

    def test_nonmonotonic_visual_odometry_remains_invalid(self):
        reference = RowHeadingReference(window_m=2, minimum_distance_m=1)
        reference.add_visual_heading(10, 1.0)
        with self.assertRaisesRegex(ValueError, "odometristräckan får inte minska"):
            reference.add_visual_heading(20, .99)

    def test_invalid_heading_or_distance_still_fails_closed(self):
        reference = RowHeadingReference(window_m=2, minimum_distance_m=1)
        with self.assertRaises(ValueError):
            reference.add_visual_heading(float("nan"), 1.0)
        with self.assertRaises(ValueError):
            reference.add_visual_heading(0.0, float("nan"))


if __name__ == "__main__":
    unittest.main()
