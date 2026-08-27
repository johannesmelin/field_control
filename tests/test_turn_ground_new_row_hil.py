from __future__ import annotations

import io
from contextlib import redirect_stderr, redirect_stdout
import json
import os
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from field_control.odometry import OdometrySample
from field_control.turn import new_row_turn_targets
from field_control.turn_ground_new_row_hil import (
    GROUND_NEW_ROW_TIMEOUT_MARGIN_S, GROUND_SPEED_PROFILES_RPM,
    NEW_ROW_DIRECTION, ROW_SPACING_M, ROW_SPACING_PROFILES_M,
    GroundNewRowRequest, GroundNewRowResult,
    _odometry_snapshot_needs_settle, ground_new_row_config, main, run_ground_new_row,
)
from field_control.turn_phase_a_hil import a4_target_timeout_s


class Clock:
    def __init__(self): self.value = 0.0
    def __call__(self): return self.value
    def sleep(self, delay): self.value += delay


class FakeRuntime:
    def __init__(self, config, *, heading=190.0, imu_fresh=True,
                 stale_after_start=False, complete=True, bad_event_state=False):
        self.config, self.target_heading = config, heading
        self.imu_fresh, self.stale_after_start, self.complete = imu_fresh, stale_after_start, complete
        self.bad_event_state = bad_event_state
        self.started = self.armed = self.manual = False
        self.calls: list[str] = []
        self._timestamp = 1.0
        self.events = SimpleNamespace(recent=self._events)
        self.imu = SimpleNamespace(snapshot=self._imu_snapshot)

    def _imu_snapshot(self):
        if self.started and self.complete: self._timestamp += .01
        return SimpleNamespace(connected=self.imu_fresh and not (self.started and self.stale_after_start),
                               updated_at_s=self._timestamp)

    def _events(self):
        if not self.started: return []
        state = "AUTO_IN_ROW_TURN" if self.bad_event_state else "AUTO_NEW_ROW_TURN"
        events = [{"kind": "turn_started", "data": {"state": state}}]
        if self.complete: events.append({"kind": "turn_completed", "data": {"state": state}})
        return events

    def status(self):
        plan = new_row_turn_targets(self.config.odometry_geometry, self.config.row_spacing_m,
                                    self.config.turn_speed_rpm, self.config.safety.new_row_turn_direction)
        done = self.started and self.complete and not self.manual
        sample = OdometrySample(plan.left_distance_m if done else 0.0,
                                plan.right_distance_m if done else 0.0, 0.0, 0.0)
        heading = self.target_heading if done else 10.0
        observation = SimpleNamespace(camera_fresh=True,
                                      imu_fresh=self.imu_fresh and not (self.started and self.stale_after_start),
                                      odometry_fresh=True, now_s=self._timestamp, heading_deg=heading,
                                      vision=SimpleNamespace(marker_found=True), visual_target=True,
                                      odometry_sample=sample)
        return SimpleNamespace(observation=observation, fault=None,
                               state="MANUAL" if self.manual else ("AUTO_ROW_FOLLOW" if done else "AUTO_NEW_ROW_TURN"),
                               motor_output_armed=self.armed and not self.manual)

    def select_auto(self): self.calls.append("select_auto")
    def arm_motor_output(self): self.calls.append("arm"); self.armed = True
    def start_auto(self): self.calls.append("start_auto"); self.started = True
    def select_manual(self): self.calls.append("select_manual"); self.manual = True; self.armed = False


class FakeApp:
    def __init__(self, config, **options): self.runtime = FakeRuntime(config, **options); self.closed = False
    def start(self): pass
    def close(self): self.closed = True


class GroundNewRowHilTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.report_path = os.path.join(self.temporary_directory.name, "last-report.json")
        self.report_patch = patch("field_control.turn_ground_new_row_hil.LAST_REPORT_PATH", self.report_path)
        self.report_patch.start()

    def tearDown(self):
        self.report_patch.stop()
        self.temporary_directory.cleanup()

    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable-test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_ground_clear=True,
                      confirm_emergency_stop_ready=True)
        values.update(changes)
        return GroundNewRowRequest(**values)

    def test_fixed_profiles_ground_context_and_asymmetric_plan(self):
        low, high = ground_new_row_config(self.request()), ground_new_row_config(self.request(speed_profile=40.0))
        self.assertEqual(GROUND_SPEED_PROFILES_RPM, (20.0, 30.0, 40.0))
        self.assertEqual((low.turn_speed_rpm, high.turn_speed_rpm), (20.0, 40.0))
        self.assertFalse(low.safety.in_row_turn_enabled)
        self.assertEqual(low.safety.new_row_turn_direction, NEW_ROW_DIRECTION)
        self.assertFalse(low.physical_can.confirm_wheels_raised)
        self.assertTrue(low.physical_can.confirm_ground_test)
        plan = new_row_turn_targets(low.odometry_geometry, ROW_SPACING_M, low.turn_speed_rpm, NEW_ROW_DIRECTION)
        self.assertGreater(plan.left_distance_m, 0.0)
        self.assertGreater(plan.right_distance_m, plan.left_distance_m)
        self.assertAlmostEqual(plan.left_distance_m, .306305, places=5)
        self.assertAlmostEqual(plan.right_distance_m, 3.463606, places=5)
        self.assertGreater(low.safety.turn_timeout_s, high.safety.turn_timeout_s)
        self.assertEqual(GROUND_NEW_ROW_TIMEOUT_MARGIN_S, 20.0)

    def test_row_spacing_profiles_are_fixed_and_150_cm_derives_exact_asymmetric_plan_and_deadline(self):
        default = ground_new_row_config(self.request())
        profile_150 = ground_new_row_config(self.request(row_spacing_profile=1.50))
        self.assertEqual(ROW_SPACING_PROFILES_M, (1.20, 1.50))
        self.assertEqual(default.row_spacing_m, 1.20)
        self.assertEqual(profile_150.row_spacing_m, 1.50)
        plan_150 = new_row_turn_targets(profile_150.odometry_geometry, 1.50,
                                        profile_150.turn_speed_rpm, NEW_ROW_DIRECTION)
        self.assertGreater(plan_150.left_distance_m, 0.0)
        self.assertGreater(plan_150.right_distance_m, plan_150.left_distance_m)
        self.assertAlmostEqual(plan_150.left_distance_m, .777544, places=5)
        self.assertAlmostEqual(plan_150.right_distance_m, 3.934845, places=5)
        largest_wheel_degrees = max(
            abs(plan_150.left_distance_m / profile_150.odometry_geometry.left_wheel_circumference_m * 360.0),
            abs(plan_150.right_distance_m / profile_150.odometry_geometry.right_wheel_circumference_m * 360.0),
        )
        self.assertEqual(profile_150.safety.turn_timeout_s, a4_target_timeout_s(
            largest_wheel_degrees, 20.0, profile_150.odometry_geometry.motor_turns_per_wheel_turn,
            timeout_margin_s=GROUND_NEW_ROW_TIMEOUT_MARGIN_S))
        self.assertGreater(profile_150.safety.turn_timeout_s, default.safety.turn_timeout_s)

    def test_right_150_cm_30_rpm_has_fixed_asymmetric_plan_and_geometry_derived_deadline(self):
        config = ground_new_row_config(self.request(
            row_spacing_profile=1.50, speed_profile=30.0, direction="right"))
        self.assertEqual(config.safety.new_row_turn_direction, "right")
        self.assertEqual(config.turn_speed_rpm, 30.0)
        plan = new_row_turn_targets(config.odometry_geometry, 1.50, 30.0, "right")
        self.assertGreater(plan.left_distance_m, plan.right_distance_m)
        self.assertAlmostEqual(plan.left_distance_m, 3.934845, places=5)
        self.assertAlmostEqual(plan.right_distance_m, .777544, places=5)
        largest_wheel_degrees = max(
            abs(plan.left_distance_m / config.odometry_geometry.left_wheel_circumference_m * 360.0),
            abs(plan.right_distance_m / config.odometry_geometry.right_wheel_circumference_m * 360.0),
        )
        self.assertEqual(config.safety.turn_timeout_s, a4_target_timeout_s(
            largest_wheel_degrees, 30.0, config.odometry_geometry.motor_turns_per_wheel_turn,
            timeout_margin_s=GROUND_NEW_ROW_TIMEOUT_MARGIN_S))

    def test_gates_and_unsupported_speed_precede_app_construction(self):
        created = []
        for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                        {"confirm_ground_clear": False}, {"confirm_emergency_stop_ready": False},
                        {"speed_profile": 25.0}, {"row_spacing_profile": 1.35}, {"direction": "forward"},
                        {"slcan_device": "/dev/ttyACM0"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                run_ground_new_row(self.request(**changes), app_factory=lambda config: created.append(config))
        self.assertEqual(created, [])

    def test_post_turn_encoder_settle_wait_is_only_for_an_aged_physical_snapshot(self):
        self.assertTrue(_odometry_snapshot_needs_settle(SimpleNamespace(
            observation=SimpleNamespace(odometry_age_s=.041))))
        for age_s in (None, .0, .04):
            with self.subTest(age_s=age_s):
                self.assertFalse(_odometry_snapshot_needs_settle(SimpleNamespace(
                    observation=SimpleNamespace(odometry_age_s=age_s))))

    def test_proves_three_fresh_180_headings_and_signed_asymmetric_targets_before_disarm(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        result = run_ground_new_row(self.request(), app_factory=factory)
        self.assertEqual(result.completed_state, "AUTO_ROW_FOLLOW")
        self.assertAlmostEqual(abs(result.heading_delta_deg), 180.0)
        self.assertGreater(result.encoder_delta_m[0], 0)
        self.assertGreater(result.encoder_delta_m[1], result.encoder_delta_m[0])
        self.assertEqual(holder[0].runtime.calls,
                         ["select_auto", "arm", "start_auto", "select_manual", "select_manual"])
        self.assertTrue(holder[0].closed)

    def test_hil_success_uses_the_explicit_150_cm_profile_plan(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        result = run_ground_new_row(self.request(row_spacing_profile=1.50), app_factory=factory)
        expected = new_row_turn_targets(holder[0].runtime.config.odometry_geometry, 1.50,
                                        20.0, NEW_ROW_DIRECTION)
        self.assertEqual(result.plan, expected)
        self.assertEqual(result.encoder_delta_m,
                         (expected.left_distance_m, expected.right_distance_m))
        self.assertGreater(holder[0].runtime.config.safety.turn_timeout_s,
                           ground_new_row_config(self.request()).safety.turn_timeout_s)

    def test_hil_success_uses_explicit_right_150_cm_30_rpm_profile(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        request = self.request(row_spacing_profile=1.50, speed_profile=30.0, direction="right")
        result = run_ground_new_row(request, app_factory=factory)
        expected = new_row_turn_targets(holder[0].runtime.config.odometry_geometry, 1.50, 30.0, "right")
        self.assertEqual(result.plan, expected)
        self.assertEqual(result.encoder_delta_m,
                         (expected.left_distance_m, expected.right_distance_m))
        self.assertGreater(result.encoder_delta_m[0], result.encoder_delta_m[1])

    def test_out_of_tolerance_or_stale_heading_fails_closed_with_measurement(self):
        for options, message in (({"heading": 170.0}, "OUT_OF_TOLERANCE"),
                                 ({"stale_after_start": True}, "HEADING_STALE")):
            holder = []
            def factory(config, options=options):
                app = FakeApp(config, **options); holder.append(app); return app
            with self.subTest(options=options), self.assertRaisesRegex(RuntimeError, message):
                run_ground_new_row(self.request(), app_factory=factory)
            self.assertIn("select_manual", holder[0].runtime.calls)
            self.assertTrue(holder[0].closed)
        with self.assertRaisesRegex(
                RuntimeError,
                r"initial=10\.00, expected=190\.00, actual=170\.00, delta=160\.00, error=20\.00 deg"):
            run_ground_new_row(self.request(), app_factory=lambda config: FakeApp(config, heading=170.0))

    def test_missing_a4_completion_is_bounded_and_cleaned_up(self):
        clock, holder = Clock(), []
        def factory(config):
            app = FakeApp(config, complete=False); holder.append(app); return app
        with self.assertRaisesRegex(TimeoutError, "A4-måldeadline"):
            run_ground_new_row(self.request(), app_factory=factory, monotonic=clock, sleep=clock.sleep)
        self.assertIn("select_manual", holder[0].runtime.calls)
        self.assertTrue(holder[0].closed)

    def test_refuses_a_completion_event_from_any_route_except_auto_new_row_turn(self):
        holder = []
        def factory(config):
            app = FakeApp(config, bad_event_state=True); holder.append(app); return app
        with self.assertRaisesRegex(RuntimeError, "AUTO_NEW_ROW_TURN"):
            run_ground_new_row(self.request(), app_factory=factory)
        self.assertIn("select_manual", holder[0].runtime.calls)
        self.assertTrue(holder[0].closed)

    def test_cli_rejects_raised_and_arbitrary_motion_knobs(self):
        base = ["--slcan-device", "/dev/serial/by-id/usb-CANable-test", "--enable-motors",
                "--confirm-physical-stop-tested", "--confirm-ground-clear", "--confirm-emergency-stop-ready"]
        for option in ("--confirm-wheels-raised", "--speed", "--duration", "--angle", "--row-spacing"):
            with self.subTest(option=option), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(base + [option, "1"])
            self.assertEqual(raised.exception.code, 2)

    def test_cli_persists_private_heading_report(self):
        config = ground_new_row_config(self.request())
        plan = new_row_turn_targets(config.odometry_geometry, ROW_SPACING_M, 20.0, NEW_ROW_DIRECTION)
        result = GroundNewRowResult(plan, (.3, 3.4), 10.0, 190.0, 180.0, "AUTO_ROW_FOLLOW")
        output = io.StringIO()
        with patch("field_control.turn_ground_new_row_hil.run_ground_new_row", return_value=result), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable-test"]), 0)
        reported = json.loads(output.getvalue())
        self.assertTrue(reported["ok"])
        self.assertEqual(reported["heading"]["delta_deg"], 180.0)
        self.assertEqual(reported["row_spacing_profile_m"], ROW_SPACING_M)
        with open(self.report_path, encoding="utf-8") as report:
            self.assertEqual(json.load(report), reported)
        self.assertEqual(stat.S_IMODE(os.stat(self.report_path).st_mode), 0o600)

    def test_cli_explicit_150_cm_profile_is_reported_and_help_exposes_no_generic_row_spacing(self):
        config = ground_new_row_config(self.request(row_spacing_profile=1.50))
        plan = new_row_turn_targets(config.odometry_geometry, 1.50, 20.0, NEW_ROW_DIRECTION)
        result = GroundNewRowResult(plan, (.7, 3.9), 10.0, 190.0, 180.0, "AUTO_ROW_FOLLOW")
        output = io.StringIO()
        arguments = ["--slcan-device", "/dev/serial/by-id/usb-CANable-test",
                     "--row-spacing-profile", "1.50"]
        with patch("field_control.turn_ground_new_row_hil.run_ground_new_row", return_value=result), redirect_stdout(output):
            self.assertEqual(main(arguments), 0)
        self.assertEqual(json.loads(output.getvalue())["row_spacing_profile_m"], 1.50)
        help_output, errors = io.StringIO(), io.StringIO()
        with redirect_stdout(help_output), redirect_stderr(errors), self.assertRaises(SystemExit) as raised:
            main(["--help"])
        self.assertEqual(raised.exception.code, 0)
        self.assertIn("--row-spacing-profile", help_output.getvalue())
        self.assertNotIn("--row-spacing M", help_output.getvalue())

    def test_cli_reports_right_150_cm_30_rpm_profile(self):
        config = ground_new_row_config(self.request(row_spacing_profile=1.50, speed_profile=30.0,
                                                    direction="right"))
        plan = new_row_turn_targets(config.odometry_geometry, 1.50, 30.0, "right")
        result = GroundNewRowResult(plan, (3.9, .7), 10.0, 190.0, 180.0, "AUTO_ROW_FOLLOW")
        output = io.StringIO()
        arguments = ["--slcan-device", "/dev/serial/by-id/usb-CANable-test",
                     "--row-spacing-profile", "1.50", "--speed-profile", "30", "--direction", "right"]
        with patch("field_control.turn_ground_new_row_hil.run_ground_new_row", return_value=result), redirect_stdout(output):
            self.assertEqual(main(arguments), 0)
        reported = json.loads(output.getvalue())
        self.assertEqual(reported["row_spacing_profile_m"], 1.50)
        self.assertEqual(reported["speed_profile_motor_rpm"], 30.0)
        self.assertEqual(reported["direction"], "right")


if __name__ == "__main__":
    unittest.main()
