from __future__ import annotations

import io
from contextlib import redirect_stderr, redirect_stdout
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from field_control.control import WheelCommand
from field_control.odometry import OdometrySample
from field_control.turn_phase_a_hil import PHASE_A_MARKER
from field_control.turn_phase_a_visible_hil import (
    MAX_NOMINAL_TRAVEL_RATIO, MIN_NOMINAL_TRAVEL_RATIO, TURN_SPEED_MOTOR_RPM,
    TURN_TIMEOUT_S, TurnPhaseAVisibleRequest, main, nominal_wheel_travel_m,
    phase_a_visible_config, run_turn_phase_a_visible,
)


class FakeRuntime:
    def __init__(self, *, fault="TURN_TIMEOUT", delta=(-.100625, .100625)):
        self.fault, self.delta = fault, delta
        self.started, self.armed, self.auto, self.ticks = False, False, False, 0
        self.calls = []
        self.events = SimpleNamespace(recent=lambda: [
            {"kind": "turn_started", "data": {}},
            {"kind": "fault", "data": {"reason": self.fault}},
        ])

    def status(self):
        if self.started:
            self.ticks += 1
        terminal = self.started and self.ticks >= 3
        sample = OdometrySample(1.0 + (self.delta[0] if self.started else 0),
                                -1.0 + (self.delta[1] if self.started else 0), 0, 0)
        observation = SimpleNamespace(camera_fresh=True, imu_fresh=True, odometry_fresh=True,
                                      vision=SimpleNamespace(marker_found=True), visual_target=True,
                                      odometry_sample=sample)
        return SimpleNamespace(observation=observation, fault=self.fault if terminal else None,
                               motor_output_armed=self.armed and not terminal,
                               last_command=WheelCommand(-10.0, 10.0, "turn") if terminal else None)

    def select_auto(self):
        self.calls.append("select_auto")
        self.auto = True

    def arm_motor_output(self):
        self.calls.append("arm")
        assert self.auto
        self.armed = True

    def start_auto(self):
        self.calls.append("start_auto")
        self.started = True


class FakeApp:
    def __init__(self, config, *, close_error=False, **options):
        self.config, self.runtime = config, FakeRuntime(**options)
        self.closed, self.close_error = False, close_error

    def start(self):
        pass

    def close(self):
        self.closed = True
        if self.close_error:
            raise RuntimeError("close failed")


class TurnPhaseAVisibleHilTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True,
                      confirm_turn_not_calibrated=True)
        values.update(changes)
        return TurnPhaseAVisibleRequest(**values)

    def test_fixed_visible_profile_is_10_motor_rpm_for_6_seconds_and_100625m(self):
        config = phase_a_visible_config(self.request())
        self.assertEqual(TURN_SPEED_MOTOR_RPM, 10.0)
        self.assertEqual(TURN_TIMEOUT_S, 6.0)
        self.assertEqual(config.max_rpm, 10.0)
        self.assertEqual(config.turn_speed_rpm, 10.0)
        self.assertEqual(config.safety.turn_timeout_s, 6.0)
        self.assertEqual(config.vision.marker, PHASE_A_MARKER)
        self.assertEqual(nominal_wheel_travel_m(config), (.100625, .100625))
        self.assertEqual((MIN_NOMINAL_TRAVEL_RATIO, MAX_NOMINAL_TRAVEL_RATIO), (.80, 1.20))

    def test_normal_lifecycle_accepts_only_timeout_stop_and_signed_80_to_120_percent_delta(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        result = run_turn_phase_a_visible(self.request(), app_factory=factory)
        self.assertEqual(result.fault, "TURN_TIMEOUT")
        self.assertEqual(result.command_sign, (-1, 1))
        self.assertAlmostEqual(result.encoder_delta_m[0], -.100625)
        self.assertAlmostEqual(result.encoder_delta_m[1], .100625)
        self.assertEqual(holder[0].runtime.calls, ["select_auto", "arm", "start_auto"])
        self.assertTrue(holder[0].closed)

    def test_out_of_range_wrong_sign_or_non_timeout_is_rejected(self):
        for options in ({"delta": (-.0804, .0804)}, {"delta": (-.1208, .1208)},
                        {"delta": (-.100625, -.100625)}, {"fault": "ODOMETRY_TIMEOUT"}):
            with self.subTest(options=options):
                with self.assertRaises(RuntimeError):
                    run_turn_phase_a_visible(self.request(), app_factory=lambda config: FakeApp(config, **options))

    def test_successful_terminal_evidence_is_not_returned_when_close_fails(self):
        with self.assertRaisesRegex(RuntimeError, "close failed"):
            run_turn_phase_a_visible(self.request(), app_factory=lambda config: FakeApp(config, close_error=True))

    def test_all_gates_precede_application_construction(self):
        created = []
        for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                        {"confirm_wheels_raised": False}, {"confirm_turn_not_calibrated": False},
                        {"slcan_device": "/dev/ttyACM0"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                run_turn_phase_a_visible(self.request(**changes), app_factory=lambda config: created.append(config))
        self.assertEqual(created, [])

    def test_cli_has_no_motion_or_timing_knobs(self):
        allowed = ["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--enable-motors",
                   "--confirm-physical-stop-tested", "--confirm-wheels-raised", "--confirm-turn-not-calibrated"]
        for option in ("--speed", "--direction", "--duration", "--turn-timeout", "--marker-timeout"):
            with self.subTest(option=option), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(allowed + [option, "1"])
            self.assertEqual(raised.exception.code, 2)

    def test_cli_reports_ok_false_when_runner_fails(self):
        output = io.StringIO()
        with patch("field_control.turn_phase_a_visible_hil.run_turn_phase_a_visible",
                   side_effect=RuntimeError("close failed")), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test"]), 2)
        self.assertFalse(json.loads(output.getvalue())["ok"])


if __name__ == "__main__":
    unittest.main()
