import unittest

import cv2
import numpy as np

from field_control.config import HsvFilter, VisionConfig, Zone
from field_control.state_machine import FieldStateMachine, Observation, SafetyConfig, State
from field_control.vision import VisionProcessor


def obs(now=0.0, **changes):
    values = dict(now_s=now, frame_fresh=True, imu_fresh=True, odometry_fresh=True,
                  can_healthy=True, visual_target=True, distance_m=0.0,
                  row_heading_reliable=True)
    values.update(changes)
    return Observation(**values)


class FieldStateMachineTests(unittest.TestCase):
    def started(self, config=SafetyConfig(auto_start_delay_s=0.0)):
        machine = FieldStateMachine(config)
        machine.select_auto()
        machine.request_start_auto(obs())
        machine.tick(obs(0.0))
        self.assertEqual(machine.state, State.AUTO_ROW_FOLLOW)
        return machine

    def test_start_delay_can_be_stopped_without_motion(self):
        machine = FieldStateMachine(SafetyConfig(auto_start_delay_s=5.0))
        machine.select_auto(); machine.request_start_auto(obs())
        self.assertEqual(machine.state, State.AUTO_START_DELAY)
        machine.stop()
        machine.tick(obs(10.0))
        self.assertEqual(machine.state, State.MANUAL)

    def test_start_without_visual_target_enters_bounded_search(self):
        machine = FieldStateMachine(SafetyConfig(auto_start_delay_s=0, search_length_m=1.0))
        machine.select_auto()
        machine.request_start_auto(obs(0, visual_target=False, distance_m=4.0))

        machine.tick(obs(0, visual_target=False, distance_m=4.0))
        self.assertEqual(machine.state, State.AUTO_SEARCH)
        self.assertEqual(machine.snapshot(0).search_distance_m, 0.0)

        machine.tick(obs(1, visual_target=False, distance_m=5.0))
        self.assertEqual(machine.state, State.FAULT)
        self.assertEqual(machine.fault, "ROW_LOST")

    def test_start_without_target_enters_bounded_search_without_visual_row_heading(self):
        machine = FieldStateMachine(SafetyConfig(auto_start_delay_s=0))
        machine.select_auto()
        machine.request_start_auto(obs(0, visual_target=False, row_heading_reliable=False))

        machine.tick(obs(0, visual_target=False, row_heading_reliable=False))
        self.assertEqual(machine.state, State.AUTO_SEARCH)

    def test_start_without_target_at_zero_search_limit_faults_before_motion(self):
        machine = FieldStateMachine(SafetyConfig(auto_start_delay_s=0, search_length_m=0))
        machine.select_auto()
        machine.request_start_auto(obs(0, visual_target=False, distance_m=4.0))

        machine.tick(obs(0, visual_target=False, distance_m=4.0))
        self.assertEqual(machine.state, State.FAULT)
        self.assertEqual(machine.fault, "ROW_LOST")

    def test_confirmed_marker_on_initial_no_target_tick_wins_before_search(self):
        machine = FieldStateMachine(SafetyConfig(
            auto_start_delay_s=0,
            turn_marker_confirm_frames=1,
            in_row_turn_enabled=False,
            number_of_rows=2,
        ))
        machine.select_auto()
        machine.request_start_auto(obs(0, visual_target=False))

        machine.tick(obs(0, visual_target=False, marker_seen=True))
        self.assertEqual(machine.state, State.AUTO_NEW_ROW_TURN)

    def test_initial_marker_wins_even_with_zero_imu_search_length(self):
        machine = FieldStateMachine(SafetyConfig(
            auto_start_delay_s=0,
            search_length_m=0,
            turn_marker_confirm_frames=1,
            in_row_turn_enabled=False,
            number_of_rows=2,
        ))
        machine.select_auto()
        machine.request_start_auto(obs(0, visual_target=False))

        machine.tick(obs(0, visual_target=False, marker_seen=True))
        self.assertEqual(machine.state, State.AUTO_NEW_ROW_TURN)

    def test_start_delay_still_faults_on_stale_sensor_before_search(self):
        machine = FieldStateMachine(SafetyConfig(auto_start_delay_s=1))
        machine.select_auto()
        machine.request_start_auto(obs(0, visual_target=False))

        machine.tick(obs(1, visual_target=False, imu_fresh=False))
        self.assertEqual(machine.state, State.FAULT)
        self.assertEqual(machine.fault, "IMU_TIMEOUT")

    def test_critical_sensor_failure_faults_active_auto_mode(self):
        machine = self.started()
        machine.tick(obs(1.0, frame_fresh=False))
        self.assertEqual(machine.state, State.FAULT)
        self.assertEqual(machine.fault, "CAMERA_TIMEOUT")

    def test_search_is_bounded_without_visual_row_heading(self):
        machine = self.started(SafetyConfig(auto_start_delay_s=0, navigation_lost_timeout_s=0,
                                            search_length_m=.5))
        machine.tick(obs(1, visual_target=False, distance_m=1, row_heading_reliable=False))
        self.assertEqual(machine.state, State.AUTO_SEARCH)
        machine.tick(obs(2, visual_target=False, distance_m=1.5))
        self.assertEqual(machine.state, State.FAULT)
        self.assertEqual(machine.fault, "ROW_LOST")

    def test_pick_clear_then_lockout_ignores_new_trigger(self):
        machine = self.started(SafetyConfig(auto_start_delay_s=0, pick_clear_time_s=1,
                                            post_pick_lockout_distance_m=.5))
        machine.tick(obs(1, bud_in_trigger_zone=True, bud_in_pick_zone=True))
        self.assertEqual(machine.state, State.AUTO_PICK)
        machine.tick(obs(2, bud_in_pick_zone=False, distance_m=1))
        machine.tick(obs(3.1, bud_in_pick_zone=False, distance_m=1))
        self.assertEqual(machine.state, State.AUTO_POST_PICK)
        machine.tick(obs(4, bud_in_trigger_zone=True, distance_m=1.3))
        self.assertEqual(machine.state, State.AUTO_POST_PICK)
        machine.tick(obs(5, distance_m=1.5))
        self.assertEqual(machine.state, State.AUTO_ROW_FOLLOW)

    def test_row_two_bud_trigger_stops_while_row_one_remains_visual_master_after_pick(self):
        """A trigger row and the selected navigation row are independent."""
        red = HsvFilter((0, 200, 200), (5, 255, 255), 4)
        green = HsvFilter((55, 200, 200), (65, 255, 255), 4)
        cfg = VisionConfig(
            navigation_mode="buds_and_leaves", buds=red, leaves=green,
            navigation_zone=Zone(0, .45, 0, 1), navigation_zone_2=Zone(.55, 1, 0, 1),
            trigger_zone=Zone(0, .05, .5, 1), trigger_zone_2=Zone(.55, 1, .5, 1),
            pick_zone=Zone(0, .05, .5, 1), pick_zone_2=Zone(.55, 1, .5, 1),
        )
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        frame[10:14, 2:6] = (60, 255, 255)   # row-1 leaf: visual master
        frame[10:14, 15:19] = (0, 255, 255)  # row-2 bud: trigger and pick
        result = VisionProcessor().process(cv2.cvtColor(frame, cv2.COLOR_HSV2BGR), 1, cfg)
        self.assertEqual(result.master_row, 1)
        self.assertTrue(result.bud_in_trigger_zone)
        self.assertTrue(result.bud_in_pick_zone)

        machine = self.started(SafetyConfig(auto_start_delay_s=0, pick_clear_time_s=1,
                                             post_pick_lockout_distance_m=.5))
        machine.tick(obs(1, visual_target=result.target_x is not None,
                         bud_in_trigger_zone=result.bud_in_trigger_zone,
                         bud_in_pick_zone=result.bud_in_pick_zone, distance_m=1))
        self.assertEqual(machine.state, State.AUTO_PICK)

        # The bud is harvested/cleared, but row 1 continues to provide the
        # master target.  A renewed row-2 trigger cannot interrupt lockout.
        machine.tick(obs(2, visual_target=True, bud_in_pick_zone=False, distance_m=1))
        machine.tick(obs(3.1, visual_target=True, bud_in_pick_zone=False, distance_m=1))
        self.assertEqual(machine.state, State.AUTO_POST_PICK)
        machine.tick(obs(4, visual_target=True, bud_in_trigger_zone=True, distance_m=1.49))
        self.assertEqual(machine.state, State.AUTO_POST_PICK)
        machine.tick(obs(5, visual_target=True, bud_in_trigger_zone=True, distance_m=1.5))
        self.assertEqual(machine.state, State.AUTO_ROW_FOLLOW)
        machine.tick(obs(6, visual_target=True, bud_in_trigger_zone=True, distance_m=1.51))
        self.assertEqual(machine.state, State.AUTO_PICK)

    def test_post_pick_lockout_exits_to_search_without_visual_target(self):
        machine = self.started(SafetyConfig(auto_start_delay_s=0, pick_clear_time_s=1,
                                            post_pick_lockout_distance_m=.5))
        machine.tick(obs(1, bud_in_trigger_zone=True, bud_in_pick_zone=True))
        machine.tick(obs(2, bud_in_pick_zone=False, distance_m=1))
        machine.tick(obs(3.1, bud_in_pick_zone=False, distance_m=1))
        self.assertEqual(machine.state, State.AUTO_POST_PICK)
        machine.tick(obs(4, bud_in_trigger_zone=True, visual_target=False, distance_m=1.5))
        self.assertEqual(machine.state, State.AUTO_SEARCH)

    def test_marker_requires_consecutive_frames_and_counts_unique_rows(self):
        config = SafetyConfig(auto_start_delay_s=0, turn_marker_confirm_frames=2,
                              in_row_turn_enabled=False, number_of_rows=1)
        machine = self.started(config)
        machine.tick(obs(1, marker_seen=True)); self.assertEqual(machine.state, State.AUTO_ROW_FOLLOW)
        machine.tick(obs(2, marker_seen=True)); self.assertEqual(machine.state, State.AUTO_NEW_ROW_TURN)
        machine.complete_turn(obs(3, distance_m=1), succeeded=True)
        self.assertEqual(machine.state, State.AUTO_COMPLETE)

    def test_in_row_turn_reverses_pass_before_next_row_turn(self):
        config = SafetyConfig(auto_start_delay_s=0, turn_marker_confirm_frames=1,
                              in_row_turn_enabled=True, number_of_rows=2)
        machine = self.started(config)
        machine.tick(obs(1, marker_seen=True)); self.assertEqual(machine.state, State.AUTO_IN_ROW_TURN)
        machine.complete_turn(obs(2, distance_m=1), succeeded=True)
        self.assertEqual(machine.pass_number, 2)
        machine.tick(obs(3, marker_seen=True, distance_m=2))
        self.assertEqual(machine.state, State.AUTO_NEW_ROW_TURN)


if __name__ == "__main__":
    unittest.main()
