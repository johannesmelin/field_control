from __future__ import annotations

import io
from contextlib import redirect_stderr
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

from field_control.control import WheelCommand
from field_control.config import Zone
from field_control.odometry import OdometrySample
from field_control.turn_phase_a_hil import (MARKER_READY_TIMEOUT_S, PHASE_A_MARKER,
                                            TURN_SPEED_MOTOR_RPM, TurnPhaseARequest, main,
                                            phase_a_config, run_turn_phase_a)
from field_control.vision import VisionProcessor


class Clock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class FakeRuntime:
    def __init__(self, *, marker=True, fault="TURN_TIMEOUT", command=(-2.0, 2.0),
                 delta=(-.02, .02), events=None):
        self.marker, self.fault, self.command, self.delta = marker, fault, command, delta
        self.started_auto, self.armed, self.selected_auto, self.turn_ticks = False, False, False, 0
        self.events = SimpleNamespace(recent=lambda: events if events is not None else [
            {"kind": "turn_started", "data": {}}, {"kind": "fault", "data": {"reason": "TURN_TIMEOUT"}},
        ])
        self.calls = []
    def status(self):
        if self.started_auto:
            self.turn_ticks += 1
        sample = OdometrySample(1.0 if not self.started_auto else 1.0 + self.delta[0],
                                -1.0 if not self.started_auto else -1.0 + self.delta[1], 0, 0)
        vision = SimpleNamespace(marker_found=self.marker, target_x=10)
        observation = SimpleNamespace(camera_fresh=True, imu_fresh=True, odometry_fresh=True,
                                      vision=vision, visual_target=self.marker, odometry_sample=sample)
        active_fault = self.fault if self.started_auto and self.turn_ticks >= 3 else None
        command = (WheelCommand(*self.command, "turn") if self.started_auto and self.turn_ticks >= 3 else None)
        return SimpleNamespace(observation=observation, fault=active_fault,
                               motor_output_armed=self.armed and active_fault is None,
                               last_command=command)
    def arm_motor_output(self):
        self.calls.append("arm")
        if not self.selected_auto: raise AssertionError("must select AUTO while disarmed before arm")
        self.armed = True
    def select_auto(self):
        self.calls.append("select_auto")
        if self.armed: raise AssertionError("select_auto must be disarmed")
        self.selected_auto = True
    def start_auto(self): self.calls.append("start_auto"); self.started_auto = True


class FakeApp:
    def __init__(self, config, **runtime_options):
        self.config, self.runtime, self.closed, self.started = config, FakeRuntime(**runtime_options), False, False
    def start(self): self.started = True
    def close(self): self.closed = True


class TurnPhaseAHilTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True,
                      confirm_turn_not_calibrated=True)
        values.update(changes)
        return TurnPhaseARequest(**values)

    def test_gates_precede_application_construction(self):
        created = []
        def factory(_config): created.append(True); return FakeApp(_config)
        for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                        {"confirm_wheels_raised": False}, {"confirm_turn_not_calibrated": False},
                        {"slcan_device": "/dev/ttyACM0"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                run_turn_phase_a(self.request(**changes), app_factory=factory)
        self.assertEqual(created, [])

    def test_fixed_config_has_zero_preturn_navigation_and_low_turn_profile(self):
        config = phase_a_config(self.request())
        self.assertTrue(config.safety.in_row_turn_enabled)
        self.assertEqual(config.auto_base_rpm, 0.0)
        self.assertEqual(config.vision_kp, 0.0)
        self.assertEqual(config.max_vision_correction_rpm, 0.0)
        self.assertEqual(config.turn_speed_rpm, TURN_SPEED_MOTOR_RPM)
        self.assertEqual(config.safety.turn_timeout_s, 2.0)
        self.assertEqual(config.safety.turn_marker_confirm_frames, 3)
        self.assertEqual(config.vision.marker, PHASE_A_MARKER)
        self.assertEqual(config.vision.turn_marker_zone, Zone(.2, .8, .3, 1.0))

    def test_temporary_marker_profile_matches_observed_yellow_bounds_and_retains_area(self):
        self.assertEqual(PHASE_A_MARKER.low, (26, 20, 150))
        self.assertEqual(PHASE_A_MARKER.high, (36, 255, 255))
        self.assertEqual(PHASE_A_MARKER.min_area, 100)
        self.assert_marker((30, 20, 150), 100, True)

    def assert_marker(self, hsv_value, area, expected):
        hsv = np.zeros((30, 30, 3), dtype=np.uint8)
        height, width = divmod(area, 10)
        self.assertEqual(width, 0)
        # Entirely inside the fixed lower-middle Phase-A marker zone.
        hsv[12:12 + height, 10:20] = hsv_value
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        result = VisionProcessor().process(frame, 1.0, phase_a_config(self.request()).vision)
        self.assertEqual(result.marker_found, expected)

    def test_marker_hsv_boundaries_and_minimum_area(self):
        processor = VisionProcessor()
        for value, expected in (
            ((26, 20, 150), True),
            ((36, 255, 255), True),
            ((25, 20, 150), False),
            ((37, 20, 150), False),
            ((26, 19, 150), False),
            ((26, 20, 149), False),
        ):
            with self.subTest(value=value):
                hsv = np.full((1, 1, 3), value, dtype=np.uint8)
                self.assertEqual(bool(processor._mask(hsv, PHASE_A_MARKER)[0, 0]), expected)
        # BGR camera input is quantized on conversion; this interior sample
        # still verifies the ordinary processor's 100 px component floor.
        self.assert_marker((30, 20, 150), 90, False)

    def test_valid_marker_outside_fixed_lower_middle_zone_is_rejected(self):
        hsv = np.zeros((30, 30, 3), dtype=np.uint8)
        # The zone starts at y=9; this valid blob is entirely above it.
        hsv[0:8, 0:10] = (30, 20, 150)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        result = VisionProcessor().process(frame, 1.0, phase_a_config(self.request()).vision)
        self.assertFalse(result.marker_found)

    def test_normal_public_lifecycle_requires_timeout_stop_and_records_data(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        result = run_turn_phase_a(self.request(), app_factory=factory)
        self.assertEqual(result.fault, "TURN_TIMEOUT")
        self.assertEqual(result.command_sign, (-1, 1))
        self.assertAlmostEqual(result.encoder_delta_m[0], -.02)
        self.assertAlmostEqual(result.encoder_delta_m[1], .02)
        self.assertEqual(result.plan.direction, "left")
        self.assertEqual(holder[0].runtime.calls, ["select_auto", "arm", "start_auto"])
        self.assertEqual(holder[0].runtime.turn_ticks, 3)  # normal 3-frame debounce retained
        self.assertTrue(holder[0].closed)
        self.assertEqual([entry["kind"] for entry in result.events], ["turn_started", "fault"])

    def test_marker_readiness_timeout_never_arms_or_starts_auto(self):
        clock = Clock(); holder = []
        def factory(config):
            app = FakeApp(config, marker=False); holder.append(app); return app
        with self.assertRaisesRegex(TimeoutError, "PHASE_A_MARKER_NOT_READY"):
            run_turn_phase_a(self.request(), app_factory=factory, monotonic=clock, sleep=clock.sleep)
        self.assertGreaterEqual(clock.now, MARKER_READY_TIMEOUT_S)
        self.assertEqual(holder[0].runtime.calls, [])
        self.assertTrue(holder[0].closed)

    def test_other_terminal_fault_is_rejected(self):
        def factory(config): return FakeApp(config, fault="TURN_HEADING_STALE")
        with self.assertRaisesRegex(RuntimeError, "TURN_HEADING_STALE"):
            run_turn_phase_a(self.request(), app_factory=factory)

    def test_wrong_or_zero_command_sign_is_rejected(self):
        for command in ((2.0, -2.0), (0.0, 2.0)):
            with self.subTest(command=command), self.assertRaisesRegex(RuntimeError, "turn-kommandotecken|icke-noll"):
                run_turn_phase_a(self.request(), app_factory=lambda config: FakeApp(config, command=command))

    def test_stale_or_wrong_sign_encoder_delta_is_rejected(self):
        for delta in ((0.0, .02), (.02, -.02)):
            with self.subTest(delta=delta), self.assertRaisesRegex(RuntimeError, "encoderdeltan"):
                run_turn_phase_a(self.request(), app_factory=lambda config: FakeApp(config, delta=delta))

    def test_missing_or_unordered_timeout_events_are_rejected(self):
        invalid = (
            [],
            [{"kind": "fault", "data": {"reason": "TURN_TIMEOUT"}}, {"kind": "turn_started", "data": {}}],
            [{"kind": "turn_started", "data": {}}, {"kind": "fault", "data": {"reason": "TURN_HEADING_STALE"}}],
        )
        for events in invalid:
            with self.subTest(events=events), self.assertRaisesRegex(RuntimeError, "runtime-event|fault-event"):
                run_turn_phase_a(self.request(), app_factory=lambda config: FakeApp(config, events=events))

    def test_cli_has_only_explicit_physical_gates(self):
        allowed = ["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--enable-motors",
                   "--confirm-physical-stop-tested", "--confirm-wheels-raised", "--confirm-turn-not-calibrated"]
        for option in ("--speed", "--direction", "--duration", "--marker-timeout"):
            with self.subTest(option=option), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(allowed + [option, "1"])
            self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
