from __future__ import annotations

import io
from contextlib import redirect_stderr
from types import SimpleNamespace
import unittest

from field_control.odometry import OdometrySample
from field_control.turn import in_row_turn_plan
from field_control.turn_ground_hil import (
    GROUND_SPEED_PROFILES_RPM, GROUND_TIMEOUT_MARGIN_S, GroundTurnRequest,
    ground_turn_config, main, run_ground_turn,
)


class Clock:
    def __init__(self): self.value = 0.0
    def __call__(self): return self.value
    def sleep(self, delay): self.value += delay


class FakeRuntime:
    def __init__(self, config, *, heading=190.0, imu_fresh=True, stale_after_start=False, complete=True):
        self.config, self.target_heading, self.imu_fresh, self.stale_after_start, self.complete = config, heading, imu_fresh, stale_after_start, complete
        self.started = self.armed = self.manual = False
        self.calls: list[str] = []
        self._timestamp = 1.0
        self.events = SimpleNamespace(recent=self._events)
        self.imu = SimpleNamespace(snapshot=self._imu_snapshot)

    def _imu_snapshot(self):
        if self.started and self.complete:
            self._timestamp += .01
        return SimpleNamespace(connected=self.imu_fresh and not (self.started and self.stale_after_start), updated_at_s=self._timestamp)

    def _events(self):
        if not self.started: return []
        result = [{"kind": "turn_started", "data": {"state": "AUTO_IN_ROW_TURN"}}]
        if self.complete: result.append({"kind": "turn_completed", "data": {"state": "AUTO_IN_ROW_TURN"}})
        return result

    def status(self):
        done = self.started and self.complete and not self.manual
        plan = in_row_turn_plan(self.config.odometry_geometry,
                                self.config.safety.in_row_turn_wheel_degrees, "left")
        sample = OdometrySample(plan.left_distance_m if done else 0.0,
                                plan.right_distance_m if done else 0.0, 0.0, 0.0)
        heading = self.target_heading if done else 10.0
        observation = SimpleNamespace(camera_fresh=True, imu_fresh=self.imu_fresh and not (self.started and self.stale_after_start),
                                      odometry_fresh=True, now_s=self._timestamp,
                                      heading_deg=heading,
                                      vision=SimpleNamespace(marker_found=True),
                                      visual_target=True, odometry_sample=sample)
        return SimpleNamespace(observation=observation, fault=None,
                               state="MANUAL" if self.manual else ("AUTO_ROW_FOLLOW" if done else "AUTO_IN_ROW_TURN"),
                               motor_output_armed=self.armed and not self.manual)

    def select_auto(self): self.calls.append("select_auto")
    def arm_motor_output(self): self.calls.append("arm"); self.armed = True
    def start_auto(self): self.calls.append("start_auto"); self.started = True
    def select_manual(self): self.calls.append("select_manual"); self.manual = True; self.armed = False


class FakeApp:
    def __init__(self, config, **options): self.runtime = FakeRuntime(config, **options); self.closed = False
    def start(self): pass
    def close(self): self.closed = True


class GroundTurnHilTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable-test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_ground_clear=True,
                      confirm_emergency_stop_ready=True)
        values.update(changes)
        return GroundTurnRequest(**values)

    def test_ground_profile_has_fixed_20_default_and_explicit_40_profile(self):
        low, high = ground_turn_config(self.request()), ground_turn_config(self.request(speed_profile=40.0))
        self.assertEqual(GROUND_SPEED_PROFILES_RPM, (20.0, 40.0))
        self.assertEqual(low.turn_speed_rpm, 20.0)
        self.assertEqual(high.turn_speed_rpm, 40.0)
        self.assertEqual(low.safety.turn_timeout_s, 48.0 + GROUND_TIMEOUT_MARGIN_S)
        self.assertEqual(high.safety.turn_timeout_s, 24.0 + GROUND_TIMEOUT_MARGIN_S)
        self.assertFalse(low.physical_can.confirm_wheels_raised)
        self.assertTrue(low.physical_can.confirm_ground_test)

    def test_gates_and_unsupported_speed_precede_app_construction(self):
        created = []
        for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                        {"confirm_ground_clear": False}, {"confirm_emergency_stop_ready": False},
                        {"speed_profile": 30.0}, {"slcan_device": "/dev/ttyACM0"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                run_ground_turn(self.request(**changes), app_factory=lambda config: created.append(config))
        self.assertEqual(created, [])

    def test_proves_fresh_180_heading_and_signed_encoder_targets_before_disarm(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        result = run_ground_turn(self.request(), app_factory=factory)
        self.assertEqual(result.completed_state, "AUTO_ROW_FOLLOW")
        self.assertAlmostEqual(abs(result.heading_delta_deg), 180.0)
        self.assertLess(result.encoder_delta_m[0], 0)
        self.assertGreater(result.encoder_delta_m[1], 0)
        self.assertEqual(holder[0].runtime.calls, ["select_auto", "arm", "start_auto", "select_manual", "select_manual"])
        self.assertTrue(holder[0].closed)

    def test_out_of_tolerance_or_stale_heading_fails_closed(self):
        for options, message in (({"heading": 170.0}, "OUT_OF_TOLERANCE"),
                                 ({"stale_after_start": True}, "HEADING_STALE")):
            holder = []
            def factory(config, options=options):
                app = FakeApp(config, **options); holder.append(app); return app
            with self.subTest(options=options), self.assertRaisesRegex(RuntimeError, message):
                run_ground_turn(self.request(), app_factory=factory)
            self.assertIn("select_manual", holder[0].runtime.calls)
            self.assertTrue(holder[0].closed)

    def test_out_of_tolerance_heading_reports_calibration_measurement(self):
        with self.assertRaisesRegex(
                RuntimeError,
                r"initial=10\.00, expected=190\.00, actual=170\.00, delta=160\.00, error=20\.00 deg"):
            run_ground_turn(self.request(), app_factory=lambda config: FakeApp(config, heading=170.0))

    def test_missing_a4_completion_is_bounded_and_cleaned_up(self):
        clock, holder = Clock(), []
        def factory(config):
            app = FakeApp(config, complete=False); holder.append(app); return app
        with self.assertRaisesRegex(TimeoutError, "A4-måldeadline"):
            run_ground_turn(self.request(), app_factory=factory, monotonic=clock, sleep=clock.sleep)
        self.assertIn("select_manual", holder[0].runtime.calls)
        self.assertTrue(holder[0].closed)

    def test_cli_rejects_raised_and_arbitrary_motion_knobs(self):
        base = ["--slcan-device", "/dev/serial/by-id/usb-CANable-test", "--enable-motors",
                "--confirm-physical-stop-tested", "--confirm-ground-clear", "--confirm-emergency-stop-ready"]
        for option in ("--confirm-wheels-raised", "--speed", "--duration", "--angle"):
            with self.subTest(option=option), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(base + [option, "1"])
            self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
