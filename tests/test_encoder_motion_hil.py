import inspect
import unittest
from unittest.mock import patch

import field_control.encoder_motion_hil as encoder_motion_hil
from field_control.encoder_motion_hil import EncoderMotionRequest, run_encoder_motion
from field_control.odometry import OdometrySample
from field_control.sources import SourceSnapshot


class Clock:
    def __init__(self): self.now = 0.0; self.waits = []
    def __call__(self): return self.now
    def sleep(self, seconds): self.waits.append(seconds); self.now += seconds


class FakeBoundary:
    def __init__(self, lease):
        self.control_lease = lease; self.events = []; self.closed = 0
    def encoder_backend(self): return object()
    def close(self): self.closed += 1


class FakeOdometry:
    instances = []
    samples = []
    ready = True
    def __init__(self, _backend, _geometry):
        self.index = 0; self.started = 0; self.stopped = 0; self.waits = 0
        type(self).instances.append(self)
    def start(self): self.started += 1
    def stop(self): self.stopped += 1
    def wait_until_ready(self, _timeout): self.waits += 1; return type(self).ready
    def snapshot(self):
        item = type(self).samples[min(self.index, len(type(self).samples) - 1)]
        self.index += 1
        if isinstance(item, Exception): raise item
        return item


class FakeRuntime:
    instances = []
    command_error = None
    def __init__(self, config, _camera, _imu, *, motor, odometry, lease, clock):
        self.config = config; self.motor = motor; self.odometry = odometry; self.lease = lease
        self.clock = clock; self.armed = False; self.fault = None; self.commands = []; self.stops = 0; self.closed = 0
        type(self).instances.append(self)
    def start(self): self.odometry.start()
    def arm_motor_output(self): self.armed = True
    def manual_command(self, command):
        self.commands.append(command)
        if type(self).command_error: raise type(self).command_error
    def stop(self): self.stops += 1; self.armed = False
    def status(self):
        return type("Status", (), {"fault": self.fault, "motor_output_armed": self.armed})()
    def close(self): self.closed += 1; self.armed = False; self.odometry.stop(); self.motor.close()


def sample(left, right, timestamp=0.0, *, connected=True):
    return SourceSnapshot(OdometrySample(left, right, (left + right) / 2.0, 0.0), timestamp, connected)


class EncoderMotionHilTests(unittest.TestCase):
    def setUp(self):
        FakeOdometry.instances = []; FakeOdometry.ready = True
        FakeRuntime.instances = []; FakeRuntime.command_error = None

    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", side="left",
                      enable_motors=True, confirm_physical_stop_tested=True,
                      confirm_wheels_raised=True)
        values.update(changes); return EncoderMotionRequest(**values)

    def runner(self, request):
        clock = Clock(); created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.encoder_motion_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.encoder_motion_hil.OdometrySource", FakeOdometry), \
             patch("field_control.encoder_motion_hil.FieldControlRuntime", FakeRuntime), \
             patch("field_control.encoder_motion_hil._monotonic", clock), \
             patch("field_control.encoder_motion_hil._bounded_sleep", clock.sleep):
            result = run_encoder_motion(request)
        return result, clock, created[0], FakeRuntime.instances[0], FakeOdometry.instances[0]

    def test_gates_fail_before_open(self):
        with patch("field_control.encoder_motion_hil.open_verified_boundary") as opened:
            for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                            {"confirm_wheels_raised": False}, {"side": "both"},
                            {"slcan_device": "/dev/ttyUSB0"},
                            {"slcan_device": "/dev/serial/by-id/nested/device"}):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_encoder_motion(self.request(**changes))
            opened.assert_not_called()

    def test_fixed_left_profile_waits_for_readiness_stops_then_validates(self):
        FakeOdometry.samples = [sample(0.0, 0.0), sample(.0034, .0001, 1.0)]
        result, clock, boundary, runtime, odometry = self.runner(self.request())
        self.assertEqual({(command.left_rpm, command.right_rpm) for command in runtime.commands}, {(2.0, 0.0)})
        self.assertEqual(len(runtime.commands), 10)
        self.assertEqual(odometry.waits, 1)
        self.assertEqual(runtime.stops, 1); self.assertEqual(runtime.closed, 1); self.assertEqual(boundary.closed, 1)
        self.assertTrue(all(wait <= .101 for wait in clock.waits))
        self.assertEqual(result.expected_wheel_rpm, (.25, 0.0))
        self.assertAlmostEqual(result.active_side_delta_m, .0034)
        self.assertAlmostEqual(result.inactive_side_delta_m, .0001)

    def test_fixed_right_profile_only_commands_right_and_uses_absolute_change(self):
        FakeOdometry.samples = [sample(.0, .0), sample(.0, -.0034, 1.0)]
        result, _clock, _boundary, runtime, _odometry = self.runner(self.request(side="right"))
        self.assertEqual({(command.left_rpm, command.right_rpm) for command in runtime.commands}, {(0.0, 2.0)})
        self.assertEqual(result.expected_wheel_rpm, (0.0, .25))
        self.assertAlmostEqual(result.active_side_delta_m, -.0034)

    def test_not_ready_fails_closed_before_arm_or_drive(self):
        FakeOdometry.ready = False; FakeOdometry.samples = [sample(.0, .0)]
        clock = Clock(); created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.encoder_motion_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.encoder_motion_hil.OdometrySource", FakeOdometry), \
             patch("field_control.encoder_motion_hil.FieldControlRuntime", FakeRuntime), \
             patch("field_control.encoder_motion_hil._monotonic", clock):
            with self.assertRaisesRegex(RuntimeError, "inte redo"):
                run_encoder_motion(self.request())
        self.assertFalse(FakeRuntime.instances[0].armed)
        self.assertEqual(FakeRuntime.instances[0].commands, [])
        self.assertEqual(created[0].closed, 1)

    def test_invalid_or_stale_odometry_fails_closed_without_drive(self):
        FakeOdometry.samples = [SourceSnapshot(0.0, 0.0, True)]
        clock = Clock(); created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.encoder_motion_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.encoder_motion_hil.OdometrySource", FakeOdometry), \
             patch("field_control.encoder_motion_hil.FieldControlRuntime", FakeRuntime), \
             patch("field_control.encoder_motion_hil._monotonic", clock):
            with self.assertRaisesRegex(RuntimeError, "typed fysisk odometri"):
                run_encoder_motion(self.request())
        self.assertEqual(FakeRuntime.instances[0].commands, [])
        self.assertEqual(created[0].closed, 1)

    def test_command_error_fails_closed_without_later_drive(self):
        FakeOdometry.samples = [sample(.0, .0), sample(.0034, .0, 1.0)]
        FakeRuntime.command_error = RuntimeError("queue failed")
        clock = Clock(); created = []
        def opened(**kwargs):
            fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
        with patch("field_control.encoder_motion_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.encoder_motion_hil.OdometrySource", FakeOdometry), \
             patch("field_control.encoder_motion_hil.FieldControlRuntime", FakeRuntime), \
             patch("field_control.encoder_motion_hil._monotonic", clock), \
             patch("field_control.encoder_motion_hil._bounded_sleep", clock.sleep):
            with self.assertRaisesRegex(RuntimeError, "queue failed"):
                run_encoder_motion(self.request())
        self.assertEqual(len(FakeRuntime.instances[0].commands), 1)
        self.assertEqual(created[0].closed, 1)

    def test_inactive_or_active_delta_limit_fails_closed(self):
        for final, message in ((sample(.0002, .0, 1.0), "miniminivån"),
                               (sample(.0034, .0011, 1.0), "okommanderad")):
            with self.subTest(message=message):
                self.setUp(); FakeOdometry.samples = [sample(.0, .0), final]
                clock = Clock(); created = []
                def opened(**kwargs):
                    fake = FakeBoundary(kwargs["lease"]); created.append(fake); return fake
                with patch("field_control.encoder_motion_hil.open_verified_boundary", side_effect=opened), \
                     patch("field_control.encoder_motion_hil.OdometrySource", FakeOdometry), \
                     patch("field_control.encoder_motion_hil.FieldControlRuntime", FakeRuntime), \
                     patch("field_control.encoder_motion_hil._monotonic", clock), \
                     patch("field_control.encoder_motion_hil._bounded_sleep", clock.sleep):
                    with self.assertRaisesRegex(RuntimeError, message): run_encoder_motion(self.request())
                self.assertEqual(created[0].closed, 1)

    def test_public_api_and_cli_expose_no_arbitrary_motion_knobs(self):
        self.assertEqual(tuple(inspect.signature(EncoderMotionRequest).parameters), (
            "slcan_device", "side", "enable_motors", "confirm_physical_stop_tested", "confirm_wheels_raised",
        ))
        with self.assertRaises(SystemExit) as rejected:
            encoder_motion_hil.main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--side", "left", "--speed", "1"])
        self.assertEqual(rejected.exception.code, 2)
