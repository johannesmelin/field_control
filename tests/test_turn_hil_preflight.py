import inspect
import io
import unittest
from contextlib import redirect_stderr

from field_control.config import RuntimeConfig
from field_control.state_machine import SafetyConfig
from field_control.turn_hil_preflight import TurnHilPreflightRequest, main, run_turn_preflight


class TurnHilPreflightTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test",
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True,
                      confirm_turn_not_calibrated=True)
        values.update(changes)
        return TurnHilPreflightRequest(**values)

    def test_gates_fail_without_any_hardware_entrypoint(self):
        for changes in ({"confirm_physical_stop_tested": False},
                        {"confirm_wheels_raised": False},
                        {"confirm_turn_not_calibrated": False},
                        {"slcan_device": "/dev/ttyUSB0"},
                        {"slcan_device": "/dev/serial/by-id/nested/device"}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    run_turn_preflight(self.request(**changes), self.config())

    @staticmethod
    def config():
        return RuntimeConfig(stream_enabled=False, max_rpm=10, turn_speed_rpm=5,
                             safety=SafetyConfig(in_row_turn_enabled=True,
                                                 new_row_turn_direction="left"))

    def test_fixed_in_row_profile_is_non_actuating_and_not_calibration(self):
        result = run_turn_preflight(self.request(), self.config())
        self.assertEqual(result.state, "AUTO_IN_ROW_TURN")
        self.assertEqual(result.direction, "left")
        self.assertEqual(result.inherited_wheel_degrees, 720.0)
        self.assertEqual((result.plan.left_ratio, result.plan.right_ratio), (-1.0, 1.0))
        self.assertFalse(result.motion_enabled)
        self.assertIn("Ingen fysisk", result.blocker)

    def test_explicit_zero_speed_config_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "turn_speed_rpm"):
            run_turn_preflight(self.request(), RuntimeConfig(
                stream_enabled=False, safety=SafetyConfig(in_row_turn_enabled=True)))

    def test_new_row_configuration_cannot_be_labeled_as_in_row_turn(self):
        config = RuntimeConfig(stream_enabled=False, max_rpm=10, turn_speed_rpm=5,
                               safety=SafetyConfig(in_row_turn_enabled=False))
        with self.assertRaisesRegex(ValueError, "in_row_turn_enabled"):
            run_turn_preflight(self.request(), config)

    def test_request_and_cli_have_no_motion_controls(self):
        self.assertEqual(tuple(inspect.signature(TurnHilPreflightRequest).parameters), (
            "slcan_device", "confirm_physical_stop_tested", "confirm_wheels_raised",
            "confirm_turn_not_calibrated",
        ))

    def test_cli_accepts_only_gates_and_rejects_motion_options(self):
        allowed = ["--slcan-device", "/dev/serial/by-id/usb-CANable_test",
                   "--confirm-physical-stop-tested", "--confirm-wheels-raised",
                   "--confirm-turn-not-calibrated"]
        self.assertEqual(main(allowed), 0)
        for option, value in (("--speed", "1"), ("--direction", "left"),
                              ("--duration", "1"), ("--enable-motors", None)):
            argv = list(allowed) + [option] + ([] if value is None else [value])
            with self.subTest(option=option), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    main(argv)
            self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
