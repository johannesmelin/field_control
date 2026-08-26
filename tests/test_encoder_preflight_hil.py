import unittest
from unittest.mock import patch
import inspect

from field_control.encoder_preflight_hil import (
    ENCODER_SAMPLE_COUNT,
    EncoderPreflightRequest,
    STATIONARY_DELTA_TOLERANCE_MOTOR_DEG,
    run_encoder_preflight,
)


class FakeBackend:
    def __init__(self, values): self.values = iter(values); self.reads = 0
    def angles(self):
        self.reads += 1
        return next(self.values)


class FakeBoundary:
    def __init__(self, values):
        self.backend = FakeBackend(values); self.closed = 0; self.arm_calls = 0; self.command_calls = 0
    def encoder_backend(self): return self.backend
    def arm(self, *_args): self.arm_calls += 1; raise AssertionError("armering är förbjuden")
    def command(self, *_args): self.command_calls += 1; raise AssertionError("drive är förbjudet")
    def close(self): self.closed += 1


class EncoderPreflightHilTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_can=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True)
        values.update(changes)
        return EncoderPreflightRequest(**values)

    def test_gates_all_fail_before_any_physical_open(self):
        with patch("field_control.encoder_preflight_hil.open_verified_boundary") as opened:
            for changes in (
                {"enable_can": False}, {"confirm_physical_stop_tested": False},
                {"confirm_wheels_raised": False}, {"slcan_device": "/dev/ttyUSB0"},
                {"slcan_device": "/dev/serial/by-id/nested/device"},
                {"slcan_device": "/dev/serial/by-id/.."},
            ):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_encoder_preflight(self.request(**changes))
            opened.assert_not_called()

    def test_reads_exactly_five_atomic_pairs_without_arm_or_drive_and_closes(self):
        created = []
        def opened(**_kwargs):
            fake = FakeBoundary([(10.00, -10.00), (10.01, -10.01), (10.00, -10.00),
                                 (10.02, -10.02), (10.01, -10.01)])
            created.append(fake); return fake
        times = iter((0.0, 0.0, .01, .10, .11, .20, .21, .30, .31, .40, .41))
        with patch("field_control.encoder_preflight_hil.open_verified_boundary", side_effect=opened), \
             patch("field_control.encoder_preflight_hil.time.monotonic", side_effect=lambda: next(times)), \
             patch("field_control.encoder_preflight_hil.time.sleep"):
            result = run_encoder_preflight(self.request())
        fake = created[0]
        self.assertEqual(fake.backend.reads, ENCODER_SAMPLE_COUNT)
        self.assertEqual(fake.arm_calls, 0); self.assertEqual(fake.command_calls, 0)
        self.assertEqual(fake.closed, 1)
        self.assertEqual(result.raw_motor_angles_deg[0], (10.0, -10.0))
        self.assertEqual(result.deltas_from_first_motor_deg[-1], (.009999999999999787, -.009999999999999787))
        for interval in result.sample_intervals_s:
            self.assertAlmostEqual(interval, .1)

    def test_malformed_angle_fails_closed_and_closes(self):
        fake = FakeBoundary([(1.0, 2.0), (float("nan"), 2.0)])
        with patch("field_control.encoder_preflight_hil.open_verified_boundary", return_value=fake), \
             patch("field_control.encoder_preflight_hil.time.sleep"):
            with self.assertRaisesRegex(ValueError, "ogiltiga motorvinklar"):
                run_encoder_preflight(self.request())
        self.assertEqual(fake.closed, 1)
        self.assertEqual(fake.arm_calls, 0); self.assertEqual(fake.command_calls, 0)

    def test_read_timeout_fails_closed_and_closes(self):
        fake = FakeBoundary([(1.0, 2.0)])
        def timeout(): raise TimeoutError("0x92 timeout")
        fake.backend.angles = timeout
        with patch("field_control.encoder_preflight_hil.open_verified_boundary", return_value=fake):
            with self.assertRaisesRegex(TimeoutError, "0x92 timeout"):
                run_encoder_preflight(self.request())
        self.assertEqual(fake.closed, 1)

    def test_stationary_limit_is_fixed_and_close_still_happens(self):
        fake = FakeBoundary([(1.0, 2.0)] + [(1.0 + STATIONARY_DELTA_TOLERANCE_MOTOR_DEG + .01, 2.0)] * 4)
        with patch("field_control.encoder_preflight_hil.open_verified_boundary", return_value=fake), \
             patch("field_control.encoder_preflight_hil.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "stillaståendegränsen"):
                run_encoder_preflight(self.request())
        self.assertEqual(fake.closed, 1)

    def test_public_request_and_cli_expose_no_motor_control_knobs(self):
        self.assertEqual(tuple(inspect.signature(EncoderPreflightRequest).parameters), (
            "slcan_device", "enable_can", "confirm_physical_stop_tested", "confirm_wheels_raised",
        ))
