import unittest
from unittest.mock import patch
import os
import subprocess
import sys
import threading

from field_control.first_motion_hil import FirstMotionRequest, run_first_motion
from field_control.lease import ControlLease
from field_control.runtime import CONTROL_LEASE_EXPIRED, FieldControlRuntime


class FakeBoundary:
    def __init__(self, lease, *, revoke=True):
        self.control_lease = lease; self.revoke = revoke
        self.armed = False; self.commands = []; self.stops = []; self.closed = 0; self.events = []
        lease.set_revoke_callback(self._revoked)
    def _revoked(self):
        self.armed = False; self.stops.append("revoke")
    def arm(self, token): self.armed = True; self.events.append("arm-settle")
    def command(self, command, token): self.commands.append((command, token))
    def stop_all(self, reason): self.armed = False; self.stops.append(reason)
    def close(self): self.closed += 1; self.armed = False; self.events.append("close")


class FirstMotionHilTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", side="left",
                      enable_motors=True, confirm_physical_stop_tested=True,
                      confirm_wheels_raised=True)
        values.update(changes)
        return FirstMotionRequest(**values)

    def test_gating_happens_before_any_physical_open(self):
        with patch("field_control.first_motion_hil.open_verified_boundary") as opened:
            for changes in (
                {"enable_motors": False},
                {"confirm_physical_stop_tested": False},
                {"confirm_wheels_raised": False},
                {"slcan_device": "/dev/ttyUSB0"},
                {"slcan_device": "/dev/serial/by-id/nested/device"},
                {"slcan_device": "/dev/serial/by-id/.."},
            ):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_first_motion(self.request(**changes))
            opened.assert_not_called()

    def test_single_left_command_is_fixed_positive_and_watchdog_then_cleanup(self):
        created = []
        def open_fake(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.first_motion_hil.open_verified_boundary", side_effect=open_fake), \
             patch.object(ControlLease, "refresh", side_effect=AssertionError("lease refresh is forbidden")), \
             patch.object(FieldControlRuntime, "start_auto", side_effect=AssertionError("AUTO is forbidden")):
            result = run_first_motion(self.request(side="left"))
        fake = created[0]
        self.assertEqual([(c.left_rpm, c.right_rpm, c.source) for c, _ in fake.commands],
                         [(2.0, 0.0, "hil-first-motion-left")])
        self.assertEqual(result.command_motor_rpm, (2.0, 0.0))
        self.assertEqual(result.expected_wheel_rpm, (.25, 0.0))
        self.assertEqual(result.fault, CONTROL_LEASE_EXPIRED)
        self.assertEqual(fake.closed, 1)

    def test_right_command_keeps_left_at_zero(self):
        created = []
        def open_fake(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.first_motion_hil.open_verified_boundary", side_effect=open_fake):
            result = run_first_motion(self.request(side="right"))
        command, _token = created[0].commands[0]
        self.assertEqual((command.left_rpm, command.right_rpm), (0.0, 2.0))
        self.assertEqual(result.side, "right")

    def test_watchdog_timeout_fails_and_still_closes(self):
        created = []
        def open_fake(**kwargs):
            fake = FakeBoundary(kwargs["lease"])
            # Prevent callback installation to model a boundary that never
            # disarms; the runner must time out and close it.
            kwargs["lease"].set_revoke_callback(lambda: None)
            created.append(fake); return fake
        with patch("field_control.first_motion_hil.LEASE_TIMEOUT_S", .03), \
             patch("field_control.first_motion_hil.WAIT_MARGIN_S", .03), \
             patch("field_control.first_motion_hil.open_verified_boundary", side_effect=open_fake):
            with self.assertRaises(TimeoutError):
                run_first_motion(self.request())
        self.assertEqual(created[0].closed, 1)

    def test_observer_wait_does_not_revoke_a_paused_watchdog_lease(self):
        created = []; watchdog_entered = threading.Event(); release_watchdog = threading.Event()
        class PausedWatchdogRuntime(FieldControlRuntime):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self._before_watchdog_revoke = lambda: (watchdog_entered.set(), release_watchdog.wait(.8))
        def open_fake(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        result = []; errors = []
        with patch("field_control.first_motion_hil.LEASE_TIMEOUT_S", .03), \
             patch("field_control.first_motion_hil.WAIT_MARGIN_S", .50), \
             patch("field_control.first_motion_hil.FieldControlRuntime", PausedWatchdogRuntime), \
             patch("field_control.first_motion_hil.open_verified_boundary", side_effect=open_fake):
            runner = threading.Thread(target=lambda: self._capture(lambda: result.append(run_first_motion(self.request())), errors))
            runner.start(); self.assertTrue(watchdog_entered.wait(.5))
            # The observer has waited beyond expiry, but must not revoke the
            # lease itself while the watchdog is paused before its final gate.
            created[0].stops.clear()  # Ignore the unarmed MANUAL startup hold.
            self.assertFalse(threading.Event().wait(.05))  # > patched lease timeout
            self.assertFalse(created[0].stops)
            self.assertTrue(runner.is_alive())
            release_watchdog.set(); runner.join(.8)
        self.assertFalse(errors)
        self.assertFalse(runner.is_alive())
        self.assertEqual(result[0].fault, CONTROL_LEASE_EXPIRED)
        self.assertFalse(created[0].armed)

    def test_alternate_fault_and_disarm_is_not_a_successful_first_motion(self):
        created = []
        class AlternateFaultRuntime(FieldControlRuntime):
            def arm_motor_output(self):
                super().arm_motor_output()
                self._record_fault("CAN_FAILURE")
                self.motor.armed = False
        def open_fake(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.first_motion_hil.FieldControlRuntime", AlternateFaultRuntime), \
             patch("field_control.first_motion_hil.open_verified_boundary", side_effect=open_fake):
            with self.assertRaisesRegex(RuntimeError, "oväntat first-motion-fel: CAN_FAILURE"):
                run_first_motion(self.request())
        self.assertEqual(created[0].closed, 1)

    def test_help_imports_without_cv2(self):
        code = """
import builtins
real_import = builtins.__import__
def blocked(name, *args, **kwargs):
    if name == 'cv2': raise ImportError('cv2 deliberately unavailable')
    return real_import(name, *args, **kwargs)
builtins.__import__ = blocked
from field_control.first_motion_hil import main
try:
    main(['--help'])
except SystemExit as exc:
    raise SystemExit(exc.code)
"""
        completed = subprocess.run([sys.executable, "-c", code], cwd=os.getcwd(), capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--confirm-wheels-raised", completed.stdout)

    @staticmethod
    def _capture(operation, errors):
        try: operation()
        except BaseException as exc: errors.append(exc)
