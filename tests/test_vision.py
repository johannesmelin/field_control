import unittest

import cv2
import numpy as np

from field_control.config import HsvFilter, VisionConfig, Zone
from field_control.vision import VisionProcessor


class VisionTests(unittest.TestCase):
    def config(self):
        red = HsvFilter((0, 200, 200), (5, 255, 255), 4)
        green = HsvFilter((55, 200, 200), (65, 255, 255), 4)
        none = HsvFilter((100, 200, 200), (110, 255, 255), 4)
        return VisionConfig("buds_and_leaves", red, green, none,
                            Zone(0, 1, 0, 1), Zone(.5, 1, .5, 1), Zone(.5, 1, .5, 1), Zone(0, 1, 0, 1), .5, 3)

    def test_target_and_zones_are_calculated_from_normalised_coordinates(self):
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[12:16, 14:18] = (0, 255, 255)  # red bud in trigger/pick zone
        hsv[4:8, 2:6] = (60, 255, 255)     # green leaf
        result = VisionProcessor().process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1.0, self.config())
        self.assertTrue(result.bud_in_trigger_zone)
        self.assertTrue(result.bud_in_pick_zone)
        self.assertAlmostEqual(result.target_x, 9.5)

    def test_outlier_can_be_ignored_without_losing_existing_filtered_target(self):
        processor = VisionProcessor(); cfg = self.config()
        hsv = np.zeros((20, 20, 3), dtype=np.uint8); hsv[4:8, 2:6] = (0, 255, 255)
        processor.process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1, cfg)
        cfg = VisionConfig(**{**cfg.__dict__, "x_outlier_threshold_px": 2})
        hsv[:] = 0; hsv[4:8, 15:19] = (0, 255, 255)
        result = processor.process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 2, cfg)
        self.assertLess(result.target_x, 6)
