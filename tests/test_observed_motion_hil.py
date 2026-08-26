import inspect
import os
import subprocess
import sys
import unittest
from unittest.mock import patch

from field_control.runtime import FieldControlRuntime
import field_control.observed_motion_hil as observed_motion_hil
from field_control.observed_motion_hil import ObservedMotionRequest, main, run_observed_motion


class Clock:
    def __init__(self): self.now = 0.0; self.waits = []
    def __call__(self): return self.now
    def sleep(self, seconds): self.waits.append(seconds); self.now += seconds


class FakeBoundary:
    def __init__(self, lease, *, command_error=None):
        self.control_lease = lease; self.armed = False; self.command_error = command_error
        self.commands = []; self.stops = []; self.closed = 0; self.events = []
        lease.set_revoke_callback(self._revoked)
    def _revoked(self): self.armed = False; self.stops.append("revoke")
    def arm(self, token): self.armed = True; self.events.append("arm-settle")
    def command(self, command, token):
        self.commands.append(command)
        if self.command_error: raise self.command_error
    def stop_all(self, reason): self.armed = False; self.stops.append(reason)
    def close(self): self.closed += 1; self.armed = False; self.events.append("close")


class ObservedMotionHilTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", side="left",
                      enable_motors=True, confirm_physical_stop_tested=True, confirm_wheels_raised=True)
        values.update(changes); return ObservedMotionRequest(**values)

    def test_gating_happens_before_open(self):
        with patch("field_control.observed_motion_hil.open_verified_boundary") as opened:
            for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                            {"confirm_wheels_raised": False}, {"slcan_device": "/dev/serial/by-id/nested/x"}):
                with self.subTest(changes=changes), self.assertRaises(ValueError): run_observed_motion(self.request(**changes))
            opened.assert_not_called()

    def test_fixed_left_mapping_refresh_deadline_stop_and_close(self):
        clock = Clock(); created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.observed_motion_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.observed_motion_hil.time.monotonic", clock), \
             patch("field_control.observed_motion_hil.time.sleep", clock.sleep), \
             patch.object(FieldControlRuntime, "start_auto", side_effect=AssertionError("AUTO forbidden")):
            result = run_observed_motion(self.request())
        fake = created[0]
        self.assertEqual({(c.left_rpm, c.right_rpm) for c in fake.commands}, {(10.0, 0.0)})
        self.assertEqual(len(fake.commands), 100)
        self.assertTrue(all(wait <= .101 for wait in clock.waits))
        self.assertAlmostEqual(result.measured_program_window_s, 10.0, places=9)
        self.assertEqual(result.command_motor_rpm, (10.0, 0.0))
        self.assertEqual(result.expected_wheel_rpm, (1.25, 0.0))
        self.assertTrue(fake.stops); self.assertEqual(fake.closed, 1)

    def test_right_mapping_keeps_left_zero(self):
        clock = Clock(); created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.observed_motion_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.observed_motion_hil.time.monotonic", clock), \
             patch("field_control.observed_motion_hil.time.sleep", clock.sleep):
            run_observed_motion(self.request(side="right"))
        self.assertEqual({(c.left_rpm, c.right_rpm) for c in created[0].commands}, {(0.0, 10.0)})

    def test_under_duration_watchdog_fault_fails_and_closes_without_later_drive(self):
        clock = Clock(); created = []
        class FaultingRuntime(FieldControlRuntime):
            def manual_command(self, command):
                super().manual_command(command); self._trip_independent_watchdog("CONTROL_LOOP_STALL")
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.observed_motion_hil.FieldControlRuntime", FaultingRuntime), \
             patch("field_control.observed_motion_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.observed_motion_hil.time.monotonic", clock), \
             patch("field_control.observed_motion_hil.time.sleep", clock.sleep):
            with self.assertRaisesRegex(RuntimeError, "CONTROL_LOOP_STALL"): run_observed_motion(self.request())
        self.assertEqual(len(created[0].commands), 1); self.assertEqual(created[0].closed, 1)

    def test_refresh_error_fails_safe_without_later_drive(self):
        clock = Clock(); created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"], command_error=RuntimeError("queue failed")); created.append(fake); return fake
        with patch("field_control.observed_motion_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.observed_motion_hil.time.monotonic", clock), \
             patch("field_control.observed_motion_hil.time.sleep", clock.sleep):
            with self.assertRaisesRegex(RuntimeError, "queue failed"): run_observed_motion(self.request())
        self.assertEqual(len(created[0].commands), 1); self.assertEqual(created[0].closed, 1)

    def test_cli_and_public_api_have_no_arbitrary_knobs(self):
        self.assertEqual(tuple(inspect.signature(run_observed_motion).parameters), ("request",))
        self.assertFalse(hasattr(observed_motion_hil, "_test_clock"))
        self.assertFalse(hasattr(observed_motion_hil, "_test_sleep"))
        with self.assertRaises(SystemExit) as rejected:
            main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--side", "left", "--speed", "1"])
        self.assertEqual(rejected.exception.code, 2)

    def test_help_imports_without_cv2_or_depthai(self):
        code = """
import builtins
real_import = builtins.__import__
def blocked(name, *args, **kwargs):
    if name in ('cv2', 'depthai'): raise ImportError(name)
    return real_import(name, *args, **kwargs)
builtins.__import__ = blocked
from field_control.observed_motion_hil import main
try: main(['--help'])
except SystemExit as exc: raise SystemExit(exc.code)
"""
        result = subprocess.run([sys.executable, "-c", code], cwd=os.getcwd(), capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
