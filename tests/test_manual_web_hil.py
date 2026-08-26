import inspect
import os
import subprocess
import sys
import unittest
from unittest.mock import patch

from field_control.control import WheelCommand
from field_control.manual_web_hil import (
    MANUAL_LEFT_DURATION_S,
    MANUAL_LEFT_REFRESH_S,
    MANUAL_RIGHT_DURATION_S,
    MANUAL_RIGHT_REFRESH_S,
    MANUAL_REVERSE_DURATION_S,
    MANUAL_REVERSE_REFRESH_S,
    MANUAL_STOP_RPM,
    ManualWebForwardRequest,
    _post_left,
    _post_right,
    _post_reverse,
    _post_stop,
    run_manual_web_forward,
    run_manual_web_left,
    run_manual_web_right,
    run_manual_web_reverse,
    run_manual_web_stop,
)
from field_control.runtime import CONTROL_LEASE_EXPIRED, FieldControlRuntime


class FakeBoundary:
    def __init__(self, lease):
        self.control_lease = lease; self.armed = False; self.commands = []; self.stops = []; self.closed = 0; self.events = []
        lease.set_revoke_callback(self._revoked)
    def _revoked(self): self.armed = False; self.stops.append("revoke")
    def arm(self, token): self.armed = True; self.events.append("arm-settle")
    def command(self, command, token): self.commands.append((command, token))
    def stop_all(self, reason): self.armed = False; self.stops.append(reason)
    def close(self): self.closed += 1; self.armed = False; self.events.append("close")


class FakeDiagnosticsServer:
    def __init__(self, runtime, **_kwargs): self.runtime = runtime
    def start(self): pass
    def close(self): pass


def mocked_forward(server):
    # The production runner uses the actual DiagnosticsServer HTTP endpoint;
    # this transport seam avoids opening sockets in ordinary unit tests.
    server.runtime.manual_command(WheelCommand(10.0, 10.0, "web-manual-forward"))


def mocked_reverse(server):
    # Production uses the existing /api/manual/reverse HTTP endpoint.  The
    # unit seam deliberately retains runtime admission and lease refresh.
    server.runtime.manual_command(WheelCommand(-10.0, -10.0, "web-manual-reverse"))


def mocked_left(server):
    # Production uses the existing /api/manual/left endpoint. The unit seam
    # still exercises runtime admission and its shared lease.
    server.runtime.manual_command(WheelCommand(-10.0, 10.0, "web-manual-left"))


def mocked_right(server):
    # Production uses the existing /api/manual/right endpoint. The unit seam
    # still exercises runtime admission and its shared lease.
    server.runtime.manual_command(WheelCommand(10.0, -10.0, "web-manual-right"))


def mocked_stop(server):
    # Production uses /api/stop.  Retain runtime STOP semantics in the unit
    # seam rather than directly mutating fake boundary state.
    server.runtime.stop()


class FastEvent:
    def __init__(self): self.set_value = False
    def set(self): self.set_value = True
    def clear(self): self.set_value = False
    def is_set(self): return self.set_value
    def wait(self, _timeout=None): return self.set_value


class ControlledTimer:
    instances = []
    def __init__(self, _interval, callback):
        self.callback = callback; self.daemon = False; self.cancelled = False
        type(self).instances.append(self)
    def start(self): pass
    def cancel(self): self.cancelled = True
    def join(self, timeout=None): pass
    def fire(self): self.callback()


class _HttpOk:
    status = 200
    def __enter__(self): return self
    def __exit__(self, *_args): return False
    def read(self): return b"{}"


class ManualWebHilTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True)
        values.update(changes); return ManualWebForwardRequest(**values)

    def test_gating_happens_before_any_physical_open(self):
        with patch("field_control.manual_web_hil.open_verified_boundary") as opened:
            for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                            {"confirm_wheels_raised": False}, {"slcan_device": "/dev/ttyUSB0"},
                            {"slcan_device": "/dev/serial/by-id/nested/device"},
                            {"slcan_device": "/dev/serial/by-id/.."}):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_manual_web_forward(self.request(**changes))
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_manual_web_reverse(self.request(**changes))
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_manual_web_left(self.request(**changes))
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_manual_web_right(self.request(**changes))
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_manual_web_stop(self.request(**changes))
            opened.assert_not_called()

    def test_fixed_active_forward_then_existing_web_stop_disarms_without_fault(self):
        created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_forward", side_effect=mocked_forward), \
             patch("field_control.manual_web_hil._post_stop", side_effect=mocked_stop), \
             patch("field_control.manual_web_hil.MANUAL_STOP_ACTIVE_DURATION_S", .010), \
             patch("field_control.manual_web_hil.MANUAL_STOP_REFRESH_S", .005), \
             patch.object(FieldControlRuntime, "start_auto", side_effect=AssertionError("AUTO forbidden")):
            result = run_manual_web_stop(self.request())
        fake = created[0]
        self.assertGreaterEqual(len(fake.commands), 1)
        self.assertTrue(all((command.left_rpm, command.right_rpm, command.source) ==
                            (MANUAL_STOP_RPM, MANUAL_STOP_RPM, "web-manual-forward")
                            for command, _token in fake.commands))
        self.assertIn("STOP", fake.stops)
        self.assertFalse(fake.armed)
        self.assertEqual(result.action, "stop")
        self.assertEqual(result.active_command_motor_rpm, (10.0, 10.0))
        self.assertEqual(result.expected_wheel_rpm, (1.25, 1.25))
        self.assertGreater(result.active_program_window_s, 0)
        self.assertEqual(fake.closed, 1)

    def test_web_stop_rejection_closes_active_output_and_cannot_succeed(self):
        created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_forward", side_effect=mocked_forward), \
             patch("field_control.manual_web_hil._post_stop", side_effect=RuntimeError("webb-STOP avvisades med HTTP 409")), \
             patch("field_control.manual_web_hil.MANUAL_STOP_ACTIVE_DURATION_S", .001), \
             patch("field_control.manual_web_hil.MANUAL_STOP_REFRESH_S", .001):
            with self.assertRaisesRegex(RuntimeError, "webb-STOP avvisades med HTTP 409"):
                run_manual_web_stop(self.request())
        self.assertGreaterEqual(len(created[0].commands), 1)
        self.assertEqual(created[0].closed, 1)
        self.assertFalse(created[0].armed)

    def test_one_existing_web_forward_command_then_watchdog_and_close(self):
        created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_forward", side_effect=mocked_forward), \
             patch.object(FieldControlRuntime, "start_auto", side_effect=AssertionError("AUTO forbidden")):
            result = run_manual_web_forward(self.request())
        fake = created[0]
        self.assertEqual([(c.left_rpm, c.right_rpm, c.source) for c, _ in fake.commands],
                         [(10.0, 10.0, "web-manual-forward")])
        self.assertEqual(result.action, "forward")
        self.assertEqual(result.command_motor_rpm, (10.0, 10.0))
        self.assertEqual(result.expected_wheel_rpm, (1.25, 1.25))
        self.assertEqual(result.lease_fault, CONTROL_LEASE_EXPIRED)
        self.assertEqual(result.can_path, "/dev/serial/by-id/usb-CANable_test")
        self.assertEqual(fake.closed, 1)

    def test_fixed_reverse_refreshes_existing_web_route_for_five_seconds_then_stops(self):
        created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        ControlledTimer.instances = []
        def fifty_reverse_requests(server):
            mocked_reverse(server)
            if len(server.runtime.motor.commands) == 50:
                ControlledTimer.instances[0].fire()
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_reverse", side_effect=fifty_reverse_requests), \
             patch("field_control.manual_web_hil._deadline_timer_factory", ControlledTimer), \
             patch("field_control.manual_web_hil._deadline_event_factory", FastEvent), \
             patch.object(FieldControlRuntime, "start_auto", side_effect=AssertionError("AUTO forbidden")):
            result = run_manual_web_reverse(self.request())
        fake = created[0]
        self.assertEqual(MANUAL_REVERSE_DURATION_S, 5.0)
        self.assertEqual(MANUAL_REVERSE_REFRESH_S, 0.100)
        self.assertGreaterEqual(len(fake.commands), 45)
        self.assertTrue(all((command.left_rpm, command.right_rpm, command.source) ==
                            (-10.0, -10.0, "web-manual-reverse")
                            for command, _token in fake.commands))
        self.assertIn("STOP", fake.stops)
        self.assertEqual(result.action, "reverse")
        self.assertEqual(result.command_motor_rpm, (-10.0, -10.0))
        self.assertEqual(result.expected_wheel_rpm, (-1.25, -1.25))
        self.assertEqual(fake.closed, 1)

    def test_blocked_final_reverse_request_cannot_admit_after_deadline_stop(self):
        created = []; calls = 0
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        def blocked_final_request(server):
            nonlocal calls
            calls += 1
            if calls == 1:
                mocked_reverse(server)
                return
            ControlledTimer.instances[0].fire()  # Deadline fires while this request is in flight.
            with self.assertRaisesRegex(ValueError, "motorutgången är avstängd"):
                mocked_reverse(server)
        ControlledTimer.instances = []
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_reverse", side_effect=blocked_final_request), \
             patch("field_control.manual_web_hil._deadline_timer_factory", ControlledTimer), \
             patch("field_control.manual_web_hil._deadline_event_factory", FastEvent):
            result = run_manual_web_reverse(self.request())
        fake = created[0]
        self.assertEqual(result.command_motor_rpm, (-10.0, -10.0))
        self.assertEqual([(command.left_rpm, command.right_rpm) for command, _token in fake.commands],
                         [(-10.0, -10.0)])
        self.assertIn("STOP", fake.stops)

    def test_fixed_left_refreshes_existing_web_route_for_five_seconds_then_stops(self):
        created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        ControlledTimer.instances = []
        def fifty_left_requests(server):
            mocked_left(server)
            if len(server.runtime.motor.commands) == 50:
                ControlledTimer.instances[0].fire()
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_left", side_effect=fifty_left_requests), \
             patch("field_control.manual_web_hil._deadline_timer_factory", ControlledTimer), \
             patch("field_control.manual_web_hil._deadline_event_factory", FastEvent), \
             patch.object(FieldControlRuntime, "start_auto", side_effect=AssertionError("AUTO forbidden")):
            result = run_manual_web_left(self.request())
        fake = created[0]
        self.assertEqual(MANUAL_LEFT_DURATION_S, 5.0)
        self.assertEqual(MANUAL_LEFT_REFRESH_S, 0.100)
        self.assertGreaterEqual(len(fake.commands), 45)
        self.assertTrue(all((command.left_rpm, command.right_rpm, command.source) ==
                            (-10.0, 10.0, "web-manual-left")
                            for command, _token in fake.commands))
        self.assertIn("STOP", fake.stops)
        self.assertEqual(result.action, "left")
        self.assertEqual(result.command_motor_rpm, (-10.0, 10.0))
        self.assertEqual(result.expected_wheel_rpm, (-1.25, 1.25))
        self.assertEqual(fake.closed, 1)

    def test_blocked_final_left_request_cannot_admit_after_deadline_stop(self):
        created = []; calls = 0
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        def blocked_final_request(server):
            nonlocal calls
            calls += 1
            if calls == 1:
                mocked_left(server)
                return
            ControlledTimer.instances[0].fire()
            with self.assertRaisesRegex(ValueError, "motorutgången är avstängd"):
                mocked_left(server)
        ControlledTimer.instances = []
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_left", side_effect=blocked_final_request), \
             patch("field_control.manual_web_hil._deadline_timer_factory", ControlledTimer), \
             patch("field_control.manual_web_hil._deadline_event_factory", FastEvent):
            result = run_manual_web_left(self.request())
        fake = created[0]
        self.assertEqual(result.command_motor_rpm, (-10.0, 10.0))
        self.assertEqual([(command.left_rpm, command.right_rpm) for command, _token in fake.commands],
                         [(-10.0, 10.0)])
        self.assertIn("STOP", fake.stops)

    def test_fixed_right_refreshes_existing_web_route_for_five_seconds_then_stops(self):
        created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        ControlledTimer.instances = []
        def fifty_right_requests(server):
            mocked_right(server)
            if len(server.runtime.motor.commands) == 50:
                ControlledTimer.instances[0].fire()
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_right", side_effect=fifty_right_requests), \
             patch("field_control.manual_web_hil._deadline_timer_factory", ControlledTimer), \
             patch("field_control.manual_web_hil._deadline_event_factory", FastEvent), \
             patch.object(FieldControlRuntime, "start_auto", side_effect=AssertionError("AUTO forbidden")):
            result = run_manual_web_right(self.request())
        fake = created[0]
        self.assertEqual(MANUAL_RIGHT_DURATION_S, 5.0)
        self.assertEqual(MANUAL_RIGHT_REFRESH_S, 0.100)
        self.assertGreaterEqual(len(fake.commands), 45)
        self.assertTrue(all((command.left_rpm, command.right_rpm, command.source) ==
                            (10.0, -10.0, "web-manual-right")
                            for command, _token in fake.commands))
        self.assertIn("STOP", fake.stops)
        self.assertEqual(result.action, "right")
        self.assertEqual(result.command_motor_rpm, (10.0, -10.0))
        self.assertEqual(result.expected_wheel_rpm, (1.25, -1.25))
        self.assertEqual(fake.closed, 1)

    def test_blocked_final_right_request_cannot_admit_after_deadline_stop(self):
        created = []; calls = 0
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        def blocked_final_request(server):
            nonlocal calls
            calls += 1
            if calls == 1:
                mocked_right(server)
                return
            ControlledTimer.instances[0].fire()
            with self.assertRaisesRegex(ValueError, "motorutgången är avstängd"):
                mocked_right(server)
        ControlledTimer.instances = []
        with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_right", side_effect=blocked_final_request), \
             patch("field_control.manual_web_hil._deadline_timer_factory", ControlledTimer), \
             patch("field_control.manual_web_hil._deadline_event_factory", FastEvent):
            result = run_manual_web_right(self.request())
        fake = created[0]
        self.assertEqual(result.command_motor_rpm, (10.0, -10.0))
        self.assertEqual([(command.left_rpm, command.right_rpm) for command, _token in fake.commands],
                         [(10.0, -10.0)])
        self.assertIn("STOP", fake.stops)

    def test_reverse_transport_targets_only_existing_reverse_route(self):
        server = type("Server", (), {"address": ("127.0.0.1", 12345)})()
        with patch("field_control.manual_web_hil.urlopen", return_value=_HttpOk()) as opened:
            _post_reverse(server)
        request = opened.call_args.args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:12345/api/manual/reverse")
        self.assertEqual(request.get_method(), "POST")

    def test_left_transport_targets_only_existing_left_route(self):
        server = type("Server", (), {"address": ("127.0.0.1", 12345)})()
        with patch("field_control.manual_web_hil.urlopen", return_value=_HttpOk()) as opened:
            _post_left(server)
        request = opened.call_args.args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:12345/api/manual/left")
        self.assertEqual(request.get_method(), "POST")

    def test_right_transport_targets_only_existing_right_route(self):
        server = type("Server", (), {"address": ("127.0.0.1", 12345)})()
        with patch("field_control.manual_web_hil.urlopen", return_value=_HttpOk()) as opened:
            _post_right(server)
        request = opened.call_args.args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:12345/api/manual/right")
        self.assertEqual(request.get_method(), "POST")

    def test_stop_transport_targets_only_existing_stop_route(self):
        server = type("Server", (), {"address": ("127.0.0.1", 12345)})()
        with patch("field_control.manual_web_hil.urlopen", return_value=_HttpOk()) as opened:
            _post_stop(server)
        request = opened.call_args.args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:12345/api/stop")
        self.assertEqual(request.get_method(), "POST")

    def test_reverse_web_rejection_or_timeout_closes_output(self):
        for message in ("webb-back avvisades med HTTP 409", "webb-back kunde inte nås: timed out"):
            with self.subTest(message=message):
                created = []
                def opened(**kwargs):
                    fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
                with patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
                     patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
                     patch("field_control.manual_web_hil._post_reverse", side_effect=RuntimeError(message)):
                    with self.assertRaisesRegex(RuntimeError, message):
                        run_manual_web_reverse(self.request())
                self.assertEqual(created[0].closed, 1)
                self.assertEqual(created[0].commands, [])

    def test_unexpected_web_rejection_closes_output(self):
        created = []
        class FaultingRuntime(FieldControlRuntime):
            def arm_motor_output(self):
                super().arm_motor_output(); self._record_fault("CAN_FAILURE"); self.motor.armed = False
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.manual_web_hil.FieldControlRuntime", FaultingRuntime), \
             patch("field_control.manual_web_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.manual_web_hil.DiagnosticsServer", FakeDiagnosticsServer), \
             patch("field_control.manual_web_hil._post_forward", side_effect=mocked_forward):
            with self.assertRaisesRegex(ValueError, "manuellt kommando kräver MANUAL"):
                run_manual_web_forward(self.request())
        self.assertEqual(created[0].closed, 1)

    def test_public_api_and_cli_offer_no_direction_speed_or_duration_knob(self):
        self.assertEqual(tuple(inspect.signature(run_manual_web_forward).parameters), ("request",))
        self.assertEqual(tuple(inspect.signature(run_manual_web_reverse).parameters), ("request",))
        self.assertEqual(tuple(inspect.signature(run_manual_web_left).parameters), ("request",))
        self.assertEqual(tuple(inspect.signature(run_manual_web_right).parameters), ("request",))
        self.assertEqual(tuple(inspect.signature(run_manual_web_stop).parameters), ("request",))
        from field_control.manual_web_hil import main
        with self.assertRaises(SystemExit) as rejected:
            main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--direction", "reverse"])
        self.assertEqual(rejected.exception.code, 2)
        from field_control.manual_web_stop_hil import main as stop_main
        with self.assertRaises(SystemExit) as rejected:
            stop_main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--duration", "1"])
        self.assertEqual(rejected.exception.code, 2)
        from field_control.manual_web_reverse_hil import main as reverse_main
        with self.assertRaises(SystemExit) as rejected:
            reverse_main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--action", "reverse"])
        self.assertEqual(rejected.exception.code, 2)
        from field_control.manual_web_left_hil import main as left_main
        with self.assertRaises(SystemExit) as rejected:
            left_main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--duration", "1"])
        self.assertEqual(rejected.exception.code, 2)
        from field_control.manual_web_right_hil import main as right_main
        with self.assertRaises(SystemExit) as rejected:
            right_main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--duration", "1"])
        self.assertEqual(rejected.exception.code, 2)

    def test_help_imports_without_cv2_or_depthai(self):
        code = """
import builtins
real_import = builtins.__import__
def blocked(name, *args, **kwargs):
    if name in ('cv2', 'depthai'): raise ImportError(name)
    return real_import(name, *args, **kwargs)
builtins.__import__ = blocked
from field_control.manual_web_hil import main
try: main(['--help'])
except SystemExit as exc: raise SystemExit(exc.code)
"""
        completed = subprocess.run([sys.executable, "-c", code], cwd=os.getcwd(), capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--confirm-wheels-raised", completed.stdout)

    def test_reverse_module_entrypoint_help_imports_without_cv2_or_depthai(self):
        code = """
import builtins
real_import = builtins.__import__
def blocked(name, *args, **kwargs):
    if name in ('cv2', 'depthai'): raise ImportError(name)
    return real_import(name, *args, **kwargs)
builtins.__import__ = blocked
from field_control.manual_web_reverse_hil import main
try: main(['--help'])
except SystemExit as exc: raise SystemExit(exc.code)
"""
        completed = subprocess.run([sys.executable, "-c", code], cwd=os.getcwd(), capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--confirm-wheels-raised", completed.stdout)

    def test_reverse_module_entrypoint_rejects_action_knob_before_hardware_open(self):
        completed = subprocess.run(
            [sys.executable, "-m", "field_control.manual_web_reverse_hil",
             "--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--action", "reverse"],
            cwd=os.getcwd(), capture_output=True, text=True,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("unrecognized arguments: --action reverse", completed.stderr)

    def test_left_module_entrypoint_help_imports_without_cv2_or_depthai(self):
        code = """
import builtins
real_import = builtins.__import__
def blocked(name, *args, **kwargs):
    if name in ('cv2', 'depthai'): raise ImportError(name)
    return real_import(name, *args, **kwargs)
builtins.__import__ = blocked
from field_control.manual_web_left_hil import main
try: main(['--help'])
except SystemExit as exc: raise SystemExit(exc.code)
"""
        completed = subprocess.run([sys.executable, "-c", code], cwd=os.getcwd(), capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--confirm-wheels-raised", completed.stdout)

    def test_left_module_entrypoint_rejects_action_knob_before_hardware_open(self):
        completed = subprocess.run(
            [sys.executable, "-m", "field_control.manual_web_left_hil",
             "--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--action", "left"],
            cwd=os.getcwd(), capture_output=True, text=True,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("unrecognized arguments: --action left", completed.stderr)

    def test_right_module_entrypoint_help_imports_without_cv2_or_depthai(self):
        code = """
import builtins
real_import = builtins.__import__
def blocked(name, *args, **kwargs):
    if name in ('cv2', 'depthai'): raise ImportError(name)
    return real_import(name, *args, **kwargs)
builtins.__import__ = blocked
from field_control.manual_web_right_hil import main
try: main(['--help'])
except SystemExit as exc: raise SystemExit(exc.code)
"""
        completed = subprocess.run([sys.executable, "-c", code], cwd=os.getcwd(), capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--confirm-wheels-raised", completed.stdout)

    def test_right_module_entrypoint_rejects_action_knob_before_hardware_open(self):
        completed = subprocess.run(
            [sys.executable, "-m", "field_control.manual_web_right_hil",
             "--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--action", "right"],
            cwd=os.getcwd(), capture_output=True, text=True,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("unrecognized arguments: --action right", completed.stderr)

    def test_stop_module_entrypoint_help_imports_without_cv2_or_depthai(self):
        code = """
import builtins
real_import = builtins.__import__
def blocked(name, *args, **kwargs):
    if name in ('cv2', 'depthai'): raise ImportError(name)
    return real_import(name, *args, **kwargs)
builtins.__import__ = blocked
from field_control.manual_web_stop_hil import main
try: main(['--help'])
except SystemExit as exc: raise SystemExit(exc.code)
"""
        completed = subprocess.run([sys.executable, "-c", code], cwd=os.getcwd(), capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--confirm-wheels-raised", completed.stdout)

    def test_stop_module_entrypoint_rejects_duration_knob_before_hardware_open(self):
        completed = subprocess.run(
            [sys.executable, "-m", "field_control.manual_web_stop_hil",
             "--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--duration", "1"],
            cwd=os.getcwd(), capture_output=True, text=True,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("unrecognized arguments: --duration 1", completed.stderr)


if __name__ == "__main__":
    unittest.main()
