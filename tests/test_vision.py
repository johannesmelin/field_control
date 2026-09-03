import unittest

import cv2
import numpy as np

from field_control.config import (FirstCrop, GoalRelativeZone, HsvFilter,
                                  TrapezoidZone, VisionConfig, Zone,
                                  project_zone_to_trapezoid)
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

    def test_disabled_camera_rows_hide_all_original_guides(self):
        cfg = VisionConfig(navigation_zone=Zone(.1, .3, .1, .3),
                           trigger_zone=Zone(.4, .6, .4, .6),
                           pick_zone=Zone(.7, .9, .7, .9),
                           turn_marker_zone=Zone(.1, .2, .7, .8),
                           x_goal=.5, row_1_enabled=False, row_2_enabled=False)
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        guide = VisionProcessor.draw_navigation_guides(frame, cfg, rows=(1, 2))
        self.assertTrue(np.array_equal(guide, frame))

    def test_first_crop_is_the_processed_and_raw_frame_coordinate_system(self):
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[8:12, 12:16] = (0, 255, 255)
        cfg = VisionConfig(buds=HsvFilter((0, 200, 200), (5, 255, 255), 4), navigation_mode="buds_only",
                           navigation_zone=Zone(0, 1, 0, 1),
                           first_crop=FirstCrop(.25, .75, .25, .75))
        result = VisionProcessor().process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1.0, cfg)
        self.assertEqual(result.raw_frame.shape[:2], (10, 10))
        self.assertEqual(result.overlay.shape[:2], (10, 10))
        self.assertAlmostEqual(result.target_x, 8.0)
        self.assertAlmostEqual(result.target_y, 4.5)

    def test_trapezoid_zone_membership_and_sloped_goal_guide(self):
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[4:8, 2:6] = (0, 255, 255)       # inside narrow top half
        hsv[13:17, 2:6] = (0, 255, 255)     # outside the right-shifted lower half
        trapezoid = TrapezoidZone(0, .5, .1, .5, 1, .9)
        cfg = VisionConfig(buds=HsvFilter((0, 200, 200), (5, 255, 255), 4),
                           navigation_mode="buds_only", navigation_zone=trapezoid,
                           x_goal=.8, x_goal_top=.2)
        result = VisionProcessor().process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1.0, cfg)
        self.assertLess(result.target_x, 6)
        guide = VisionProcessor.draw_navigation_guides(np.zeros((20, 20, 3), dtype=np.uint8), cfg)
        self.assertTrue(np.array_equal(guide[0, 4], np.array((0, 0, 255), dtype=np.uint8)))
        self.assertTrue(np.array_equal(guide[19, 15], np.array((0, 0, 255), dtype=np.uint8)))

    def test_ground_width_calibration_slopes_left_goal_right_at_wider_top(self):
        cfg = VisionConfig(x_goal=.25, ground_width_bottom_m=.36, ground_width_top_m=.91)
        # The same left-of-centre physical track is nearer image centre at
        # the wider (far/upper) edge, i.e. it leans right in the image.
        self.assertGreater(cfg.goal_x_normalized(0, 20), cfg.x_goal)
        self.assertEqual(cfg.goal_x_normalized(19, 20), cfg.x_goal)
        projected = project_zone_to_trapezoid(
            Zone(.2, .4, .2, .8), ground_width_bottom_m=.36, ground_width_top_m=.91)
        self.assertGreater(projected.x_max_top - projected.x_min_top, .2)

    def test_unequal_ground_widths_automatically_make_legacy_zone_a_lower_limited_trapezoid(self):
        zone = Zone(.2, .4, .6, .9)
        cfg = VisionConfig(navigation_mode="buds_only", navigation_zone=zone,
                           trigger_zone=Zone(0, .05, 0, .05),
                           pick_zone=Zone(0, .05, 0, .05),
                           turn_marker_zone=Zone(0, .05, 0, .05),
                           ground_width_bottom_m=.36, ground_width_top_m=.91)
        effective = cfg.effective_zone(zone)
        self.assertIsInstance(effective, TrapezoidZone)
        # Width is interpolated at the zone's own y boundaries: 0.58 m at
        # y=.6 and 0.415 m at y=.9, not the full-image 0.36/0.91 ratio.
        self.assertAlmostEqual(effective.x_min_top, .5 + (.2 - .5) * (.58 / .415))
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        guide = VisionProcessor.draw_zones(frame, cfg)
        self.assertTrue(np.array_equal(guide[12, 4], np.array((255, 180, 0), dtype=np.uint8)))
        self.assertTrue(np.array_equal(guide[12, 1], frame[12, 1]))

        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[12:16, 4:8] = (0, 255, 255)
        processed = VisionProcessor().process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1.0,
                                               VisionConfig(navigation_mode="buds_only",
                                                            buds=HsvFilter((0, 200, 200), (5, 255, 255), 4),
                                                            navigation_zone=zone,
                                                            ground_width_bottom_m=.36,
                                                            ground_width_top_m=.91))
        self.assertGreater(cv2.countNonZero(processed.masks["buds"]), 0)
        self.assertEqual(cv2.countNonZero(processed.masks["buds"]), 16)

    def test_width_calibration_maps_goal_through_vertical_first_crop(self):
        cfg = VisionConfig(x_goal=.25, first_crop=FirstCrop(0, 1, .25, .75),
                           ground_width_bottom_m=.36, ground_width_top_m=.91)
        # Full-frame widths at the processed bottom/top are respectively
        # 0.4975 m (y=.75) and 0.7725 m (y=.25), rather than .36/.91.
        expected_top = .5 + (.25 - .5) * (.4975 / .7725)
        self.assertAlmostEqual(cfg.goal_x_normalized(0, 100), expected_top)
        self.assertEqual(cfg.goal_x_normalized(99, 100), .25)
        cropped_zone = cfg.effective_zone(Zone(.3, .7, .2, .8))
        self.assertGreater(cropped_zone.x_max_top - cropped_zone.x_min_top, .4)

    def test_asymmetric_crop_clips_projected_zone_instead_of_failing_vision(self):
        cfg = VisionConfig(navigation_mode="buds_only",
                           buds=HsvFilter((0, 200, 200), (5, 255, 255), 4),
                           navigation_zone=Zone(.1, .9, .2, .8),
                           first_crop=FirstCrop(0, .4, 0, 1),
                           ground_width_bottom_m=.36, ground_width_top_m=.91)
        effective = cfg.effective_zone(cfg.navigation_zone)
        self.assertIsInstance(effective, TrapezoidZone)
        self.assertEqual(effective.x_min_top, 0.0)
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[12:16, 3:7] = (0, 255, 255)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        result = VisionProcessor().process(frame, 1.0, cfg)
        self.assertGreater(cv2.countNonZero(result.masks["buds"]), 0)
        self.assertEqual(VisionProcessor.draw_zones(frame[:, :8], cfg).shape[:2], (20, 8))

    def test_trapezoid_validation_rejects_folded_quad_but_accepts_clipped_triangle(self):
        with self.assertRaises(ValueError):
            TrapezoidZone(.9, 1.0, .2, 0.0, .1, .8).validate()
        clipped = TrapezoidZone(1.0, 1.0, .2, .1, .9, .8)
        self.assertIs(clipped.validate(), clipped)

    def test_goal_relative_zone_defaults_preserve_old_centred_rectangles(self):
        cfg = VisionConfig()
        self.assertEqual(cfg.navigation_zone, GoalRelativeZone(.3, .3, 1.0))
        self.assertEqual(cfg.effective_zone(cfg.navigation_zone), Zone(.2, .8, .3, 1.0))

    def test_goal_relative_zone_preserves_physical_width_and_tracks_goal(self):
        zone = GoalRelativeZone(.2, .2, .8)
        cfg = VisionConfig(x_goal=.25, ground_width_bottom_m=.36, ground_width_top_m=.91,
                           navigation_zone=zone)
        effective = cfg.effective_zone(zone)
        self.assertIsInstance(effective, TrapezoidZone)
        top_width = .91 + (.36 - .91) * .2
        bottom_width = .91 + (.36 - .91) * .8
        self.assertAlmostEqual((effective.x_min_bottom + effective.x_max_bottom) / 2,
                               cfg.goal_x_normalized_fraction(.8))
        self.assertAlmostEqual((effective.x_min_top + effective.x_max_top) / 2,
                               cfg.goal_x_normalized_fraction(.2))
        self.assertAlmostEqual((effective.x_max_top - effective.x_min_top) / 2,
                               .2 * .36 / top_width)
        self.assertAlmostEqual((effective.x_max_bottom - effective.x_min_bottom) / 2,
                               .2 * .36 / bottom_width)
        self.assertLess(effective.x_max_top - effective.x_min_top,
                        effective.x_max_bottom - effective.x_min_bottom)

    def test_goal_relative_zone_uses_crop_rows_and_validates_distance(self):
        zone = GoalRelativeZone(.2, .2, .8)
        cfg = VisionConfig(first_crop=FirstCrop(0, 1, .25, .75),
                           ground_width_bottom_m=.36, ground_width_top_m=.91,
                           navigation_zone=zone)
        effective = cfg.effective_zone(zone)
        top_width = .91 + (.36 - .91) * (.25 + .5 * .2)
        reference_width = .91 + (.36 - .91) * .75
        self.assertAlmostEqual((effective.x_max_top - effective.x_min_top) / 2,
                               .2 * reference_width / top_width)
        with self.assertRaises(ValueError): GoalRelativeZone(.5001, .2, .8).validate()
        with self.assertRaises(ValueError): VisionConfig(navigation_zone=GoalRelativeZone(0, .2, .8)).validate()
        with self.assertRaises(ValueError): VisionConfig(turn_marker_zone=GoalRelativeZone(.2, .2, .8)).validate()

    def test_dual_rows_prioritise_row_one_then_immediately_fall_back_to_row_two(self):
        red = HsvFilter((0, 200, 200), (5, 255, 255), 4)
        cfg = VisionConfig(navigation_mode="buds_only", buds=red,
                           navigation_zone=Zone(0, .45, 0, 1), navigation_zone_2=Zone(.55, 1, 0, 1),
                           trigger_zone=Zone(0, .45, .5, 1), trigger_zone_2=Zone(.55, 1, .5, 1),
                           pick_zone=Zone(0, .45, .5, 1), pick_zone_2=Zone(.55, 1, .5, 1),
                           x_goal=.25, x_goal_2=.75, x_filter_window_frames=2)
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[10:14, 2:6] = (0, 255, 255); hsv[10:14, 15:19] = (0, 255, 255)
        processor = VisionProcessor()
        both = processor.process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1, cfg)
        self.assertEqual(both.master_row, 1); self.assertIsNotNone(both.row_2_target_x)
        hsv[:] = 0; hsv[10:14, 15:19] = (0, 255, 255)
        fallback = processor.process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 2, cfg)
        self.assertEqual(fallback.master_row, 2)
        self.assertGreater(fallback.target_x, 14)
        self.assertTrue(fallback.bud_in_trigger_zone); self.assertTrue(fallback.bud_in_pick_zone)

    def test_disabled_rows_cannot_supply_visual_target_or_master(self):
        red = HsvFilter((0, 200, 200), (5, 255, 255), 4)
        cfg = VisionConfig(navigation_mode="buds_only", buds=red,
                           navigation_zone=Zone(0, .45, 0, 1), navigation_zone_2=Zone(.55, 1, 0, 1),
                           trigger_zone=Zone(0, .45, .5, 1), trigger_zone_2=Zone(.55, 1, .5, 1),
                           pick_zone=Zone(0, .45, .5, 1), pick_zone_2=Zone(.55, 1, .5, 1),
                           row_1_enabled=False, row_2_enabled=True)
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[10:14, 2:6] = (0, 255, 255); hsv[10:14, 15:19] = (0, 255, 255)
        processor = VisionProcessor()
        row_two_master = processor.process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1, cfg)
        self.assertEqual(row_two_master.master_row, 2)
        self.assertIsNone(row_two_master.row_1_target_x)
        self.assertIsNotNone(row_two_master.row_2_target_x)
        # Both disabled intentionally leaves AUTO with no visual target, so
        # the existing runtime fallback may use IMU-only navigation.
        none = processor.process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 2,
                                 VisionConfig(**{**cfg.__dict__, "row_2_enabled": False}))
        self.assertIsNone(none.master_row)
        self.assertIsNone(none.target_x)
        # A disabled row supplies neither navigation nor harvest evidence:
        # its camera can then be intentionally unavailable without a hidden
        # trigger path.
        self.assertFalse(none.bud_in_trigger_zone)
        self.assertFalse(none.bud_in_pick_zone)

    def test_rows_three_and_four_are_independent_and_are_processed_on_cam_two(self):
        red = HsvFilter((0, 200, 200), (5, 255, 255), 4)
        cfg = VisionConfig(navigation_mode="buds_only", buds=red,
                           navigation_zone_3=Zone(0, .45, 0, 1), navigation_zone_4=Zone(.55, 1, 0, 1),
                           trigger_zone_3=Zone(0, .45, .5, 1), trigger_zone_4=Zone(.55, 1, .5, 1),
                           row_3_enabled=True, row_4_enabled=True)
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[10:14, 2:6] = (0, 255, 255); hsv[10:14, 15:19] = (0, 255, 255)
        result = VisionProcessor().process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1, cfg, rows=(3, 4))
        self.assertEqual(result.master_row, 3)
        self.assertIsNotNone(result.row_3_target_x); self.assertIsNotNone(result.row_4_target_x)
        self.assertTrue(result.row_triggered[3]); self.assertTrue(result.row_triggered[4])

    def test_leaf_in_either_trigger_zone_never_triggers_pick(self):
        """Trigger membership is deliberately evaluated from the bud mask only."""
        green = HsvFilter((55, 200, 200), (65, 255, 255), 4)
        no_buds = HsvFilter((100, 200, 200), (110, 255, 255), 4)
        cfg = VisionConfig(
            navigation_mode="buds_and_leaves", buds=no_buds, leaves=green,
            navigation_zone=Zone(0, .45, 0, 1), navigation_zone_2=Zone(.55, 1, 0, 1),
            trigger_zone=Zone(0, .45, .5, 1), trigger_zone_2=Zone(.55, 1, .5, 1),
            pick_zone=Zone(0, .45, .5, 1), pick_zone_2=Zone(.55, 1, .5, 1),
        )
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        # A qualifying leaf in row 2 is a valid navigation target in this
        # mode, but must never produce a harvest trigger or pick-zone hold.
        hsv[12:16, 15:19] = (60, 255, 255)
        result = VisionProcessor().process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1, cfg)
        self.assertEqual(result.master_row, 2)
        self.assertIsNotNone(result.target_x)
        self.assertFalse(result.bud_in_trigger_zone)
        self.assertFalse(result.bud_in_pick_zone)

    def test_dual_rows_keep_filter_histories_independent_and_draw_both_guides(self):
        red = HsvFilter((0, 200, 200), (5, 255, 255), 4)
        cfg = VisionConfig(navigation_mode="buds_only", buds=red,
                           navigation_zone=Zone(0, .45, 0, 1), navigation_zone_2=Zone(.55, 1, 0, 1),
                           x_goal=.2, x_goal_2=.8, x_filter_window_frames=3)
        processor = VisionProcessor(); hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[4:8, 2:6] = (0, 255, 255)
        processor.process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1, cfg)
        hsv[:] = 0; hsv[4:8, 15:19] = (0, 255, 255)
        result = processor.process(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 2, cfg)
        self.assertEqual(result.master_row, 2)
        self.assertGreater(result.target_x, 14)  # no row-1 history contamination
        guide = VisionProcessor.draw_navigation_guides(np.zeros((20, 20, 3), dtype=np.uint8), cfg)
        self.assertTrue(np.any(guide[:, 4] == (0, 0, 255)))
        self.assertTrue(np.any(guide[:, 15] == (0, 80, 255)))

    def test_validate_rejects_row_two_goal_relative_zone_outside_its_own_projection(self):
        # Row 1's centre projection is valid in this crop.  Validating row 2
        # against row 1 (the historical defect) would therefore pass, while
        # row 2's left goal projects this small far zone wholly outside.
        zone = GoalRelativeZone(.05, 0, .1)
        cfg = VisionConfig(first_crop=FirstCrop(.3, .5, 0, 1), x_goal=.5, x_goal_2=0,
                           ground_width_bottom_m=.36, ground_width_top_m=.91,
                           navigation_zone=zone, trigger_zone=zone, pick_zone=zone,
                           navigation_zone_2=zone, trigger_zone_2=zone, pick_zone_2=zone)
        self.assertIsNotNone(cfg._effective_goal_relative_zone(zone, 1))
        with self.assertRaises(ValueError):
            cfg.validate()
