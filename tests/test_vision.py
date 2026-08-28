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

    def test_raw_zone_render_uses_overlay_zone_pixels_without_detection_or_goal_lines(self):
        cfg = VisionConfig(navigation_zone=Zone(.1, .3, .1, .3),
                           trigger_zone=Zone(.4, .6, .4, .6),
                           pick_zone=Zone(.7, .9, .7, .9),
                           turn_marker_zone=Zone(.1, .2, .7, .8), x_goal=.5)
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        result = VisionProcessor().process(frame, 1.0, cfg)
        raw = VisionProcessor.draw_zones(frame, cfg)
        # navigation-zone top edge: exact shared BGR colour and one-pixel line
        self.assertTrue(np.array_equal(raw[2, 3], np.array((255, 180, 0), dtype=np.uint8)))
        self.assertTrue(np.array_equal(raw[2, 3], result.overlay[2, 3]))
        # Overlay's red x-goal is intentionally absent from Original/raw.
        self.assertTrue(np.array_equal(raw[1, 10], frame[1, 10]))
        self.assertTrue(np.array_equal(result.overlay[1, 10], np.array((0, 0, 255), dtype=np.uint8)))

    def test_original_navigation_guides_share_overlay_zone_and_x_goal_pixels(self):
        cfg = VisionConfig(navigation_zone=Zone(.1, .3, .1, .3), x_goal=.5)
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        result = VisionProcessor().process(frame, 1.0, cfg)
        original = VisionProcessor.draw_navigation_guides(frame, cfg)
        self.assertTrue(np.array_equal(original[2, 3], result.overlay[2, 3]))
        self.assertTrue(np.array_equal(original[1, 10], result.overlay[1, 10]))
