"""Mock-only runtime integration for the pure in-row/new-row turn controller."""
from __future__ import annotations

import unittest

from field_control.config import RuntimeConfig
from field_control.observation import ImuReading
from field_control.odometry import OdometrySample
from field_control.runtime import FieldControlRuntime
from field_control.sources import LatestValue
from field_control.state_machine import SafetyConfig, State


class Source:
    def __init__(self): self.latest = LatestValue()
    def start(self): pass
    def stop(self): pass
    def snapshot(self): return self.latest.snapshot()


class RecordingMotor:
    armed = True
    fault_reason = None
    def __init__(self): self.commands = []; self.stops = []
    def command(self, command, _token=None): self.commands.append(command)
    def stop_all(self, reason): self.stops.append(reason)
    def hold_stopped(self, reason, _token=None): self.stops.append(reason)


class RuntimeTurnIntegrationTests(unittest.TestCase):
    def make_runtime(self, state=State.AUTO_IN_ROW_TURN, *, confirmations=2, turn_speed=10):
        self.now = [0.0]
        safety = SafetyConfig(in_row_turn_enabled=True, new_row_turn_direction="left",
                              turn_heading_confirm_frames=confirmations,
                              turn_heading_tolerance_deg=2, turn_distance_tolerance_m=.01,
                              turn_timeout_s=2, turn_heading_max_age_s=.2)
        config = RuntimeConfig(stream_enabled=False, max_rpm=20, turn_speed_rpm=turn_speed,
                               heading_filter_alpha=1, camera_timeout_s=1, imu_timeout_s=1,
                               odometry_timeout_s=1, safety=safety)
        camera, imu, odometry = Source(), Source(), Source()
        camera.latest.publish(None, 0.0)
        imu.latest.publish(ImuReading(10, 0.0), 0.0)
        odometry.latest.publish(OdometrySample(0, 0, 0, 0), 0.0)
        motor = RecordingMotor()
        runtime = FieldControlRuntime(config, camera, imu, motor=motor, odometry=odometry,
                                      clock=lambda: self.now[0])
        runtime.heading.reference.reference_deg = 10
        runtime.heading.reference.reliable = True
        with runtime._state_lock:
            runtime.machine._transition(state, "test turn")
        return runtime, imu, odometry, motor

    def publish(self, imu, odometry, heading, sample):
        imu.latest.publish(ImuReading(heading, self.now[0]), self.now[0])
        odometry.latest.publish(sample, self.now[0])

    def test_in_row_turn_creates_one_controller_and_admits_motor_side_command(self):
        runtime, imu, odometry, motor = self.make_runtime()
        runtime.tick()
        controller = runtime._turn_controller
        self.assertIsNotNone(controller)
        self.assertEqual((motor.commands[-1].left_rpm, motor.commands[-1].right_rpm), (-10, 10))
        self.assertEqual(motor.commands[-1].source, "turn")
        self.now[0] = .05; self.publish(imu, odometry, 10, OdometrySample(0, 0, 0, 0))
        runtime.tick()
        self.assertIs(runtime._turn_controller, controller)

    def test_new_row_turn_uses_same_single_controller_path(self):
        runtime, _imu, _odometry, motor = self.make_runtime(State.AUTO_NEW_ROW_TURN)
        runtime.tick()
        self.assertIsNotNone(runtime._turn_controller)
        self.assertEqual(motor.commands[-1].source, "turn")
        self.assertGreater(motor.commands[-1].left_rpm, 0)
        self.assertGreater(motor.commands[-1].right_rpm, 0)

    def test_confirmed_success_holds_stopped_updates_heading_once_and_completes(self):
        runtime, imu, odometry, motor = self.make_runtime(confirmations=2)
        runtime.tick()
        # In-row left plan is -/+ two wheel turns from its immutable baseline.
        target = runtime._turn_controller.plan
        self.now[0] = .1
        self.publish(imu, odometry, 190, OdometrySample(target.left_distance_m, target.right_distance_m, 0, 0))
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.AUTO_IN_ROW_TURN)
        self.now[0] = .2
        self.publish(imu, odometry, 190, OdometrySample(target.left_distance_m, target.right_distance_m, 0, 0))
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.AUTO_SEARCH)
        self.assertEqual(runtime.machine.pass_number, 2)
        self.assertEqual(runtime.heading.reference.reference_deg, 190)
        self.assertIsNone(runtime._turn_controller)
        self.assertEqual(motor.stops.count("turn completed"), 1)
        self.assertEqual(len(motor.commands), 2)
        kinds = [event["kind"] for event in runtime.events.recent()]
        self.assertIn("turn_started", kinds)
        self.assertIn("heading_reference_180", kinds)
        self.assertIn("turn_completed", kinds)

    def test_turn_freezes_visual_row_heading_until_the_single_successful_180_update(self):
        runtime, imu, odometry, _motor = self.make_runtime(confirmations=2)
        runtime._vision = type("Vision", (), {
            "target_x": 1.0, "bud_in_trigger_zone": False,
            "bud_in_pick_zone": False, "marker_found": False,
        })()
        runtime.tick()
        target = runtime._turn_controller.plan
        self.now[0] = .1
        self.publish(imu, odometry, 190, OdometrySample(target.left_distance_m, target.right_distance_m, 0, 0))
        runtime.tick()
        # A fresh visual target and heading must not re-learn the new row
        # heading during the physical turn.
        self.assertEqual(runtime.heading.reference.reference_deg, 10)
        self.now[0] = .2
        self.publish(imu, odometry, 190, OdometrySample(target.left_distance_m, target.right_distance_m, 0, 0))
        runtime.tick()
        self.assertEqual(runtime.heading.reference.reference_deg, 190)

    def test_stale_heading_and_missing_per_wheel_sample_fail_closed_without_drive(self):
        runtime, _imu, odometry, motor = self.make_runtime()
        odometry.latest.publish(0.0, 0.0)  # Legacy float remains valid outside turns, never for a turn.
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.FAULT)
        self.assertEqual(runtime.status().fault, "TURN_ODOMETRY_SAMPLE_MISSING")
        self.assertEqual(motor.commands, [])
        self.assertIn("fault", [event["kind"] for event in runtime.events.recent()])

        runtime, _imu, _odometry, motor = self.make_runtime()
        runtime.tick()
        self.now[0] = .3  # Source remains fresh by RuntimeConfig, but controller age limit expires.
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.FAULT)
        self.assertEqual(runtime.status().fault, "TURN_HEADING_STALE")
        self.assertEqual(len(motor.commands), 1)

    def test_invalid_turn_configuration_fails_before_any_command_or_heading_change(self):
        runtime, _imu, _odometry, motor = self.make_runtime(turn_speed=0)
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.FAULT)
        self.assertEqual(runtime.status().fault, "TURN_CONFIGURATION_INVALID")
        self.assertEqual(motor.commands, [])
        self.assertEqual(runtime.heading.reference.reference_deg, 10)

    def test_manual_stop_clears_controller_and_prevents_stale_turn_resume(self):
        runtime, imu, odometry, motor = self.make_runtime()
        runtime.tick(); commands = len(motor.commands)
        runtime.select_manual()
        self.assertIsNone(runtime._turn_controller)
        self.now[0] = .1; self.publish(imu, odometry, 190, OdometrySample(-2, 2, 0, 0))
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.MANUAL)
        self.assertEqual(len(motor.commands), commands)

        with runtime._state_lock:
            runtime.machine._transition(State.AUTO_IN_ROW_TURN, "stale test")
        runtime.close()
        runtime.tick()
        self.assertIsNone(runtime._turn_controller)
        self.assertEqual(len(motor.commands), commands)


if __name__ == "__main__":
    unittest.main()
