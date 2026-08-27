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
from field_control.turn_phase_a_hil import (
    MarkerNotReadyError,
    PHASE_A_MARKER, POLL_S, TURN_SPEED_MOTOR_RPM, TURN_TIMEOUT_MARGIN_S, TURN_TIMEOUT_S,
    TurnPhaseARequest, TurnPhaseAResult, _persist_last_report, a4_target_timeout_s, main,
    phase_a_config, run_turn_phase_a,
)
from field_control.turn import DifferentialTurnPlan


class FakeRuntime:
    def __init__(self, *, fault=None, delta=(-1.61, 1.61), completed=True, marker_ready=True):
        self.fault, self.delta, self.completed, self.marker_ready = fault, delta, completed, marker_ready
        self.auto = self.armed = self.started = self.manual = False
        self.calls: list[str] = []
        self.events = SimpleNamespace(recent=self._events)

    def _events(self):
        if not self.started:
            return []
        if self.completed:
            return [
                {"kind": "turn_started", "data": {"state": "AUTO_IN_ROW_TURN"}},
                {"kind": "turn_completed", "data": {"state": "AUTO_IN_ROW_TURN"}},
            ]
        return [{"kind": "turn_started", "data": {"state": "AUTO_IN_ROW_TURN"}}]

    def status(self):
        complete = self.started and self.completed and not self.manual
        sample = OdometrySample(1.0 + (self.delta[0] if complete else 0.0),
                                -1.0 + (self.delta[1] if complete else 0.0), 0.0, 0.0)
        observation = SimpleNamespace(camera_fresh=True, imu_fresh=True, odometry_fresh=True,
                                      camera_age_s=.01, imu_age_s=.02, odometry_age_s=.03,
                                      camera_error=None, imu_error=None,
                                      vision=SimpleNamespace(marker_found=self.marker_ready, target_x=123.0,
                                                             bud_in_trigger_zone=False), visual_target=self.marker_ready,
                                      odometry_sample=sample)
        return SimpleNamespace(observation=observation, fault=self.fault if self.started else None,
                               state="MANUAL" if self.manual else ("AUTO_ROW_FOLLOW" if complete else "AUTO_IN_ROW_TURN"),
                               motor_output_armed=self.armed and not self.manual)

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

    def select_manual(self):
        self.calls.append("select_manual")
        self.manual = True
        self.armed = False


class FakeApp:
    def __init__(self, config, *, close_error=False, **options):
        self.config, self.runtime = config, FakeRuntime(**options)
        self.closed = False
        self.close_error = close_error

    def start(self):
        pass

    def close(self):
        self.closed = True
        sink = getattr(getattr(self.runtime, "motor", None), "_sink", None)
        if sink is not None:
            sink.closed = True
        if self.close_error:
            raise RuntimeError("STOP+0x9C settle failed")


class Clock:
    def __init__(self): self.value = 0.0
    def __call__(self): return self.value
    def sleep(self, delay): self.value += delay


class TurnPhaseAHilTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.report_path = os.path.join(self.temporary_directory.name, "last-report.json")
        self.report_patch = patch("field_control.turn_phase_a_hil.LAST_REPORT_PATH", self.report_path)
        self.report_patch.start()

    def tearDown(self):
        self.report_patch.stop()
        self.temporary_directory.cleanup()

    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True,
                      confirm_turn_not_calibrated=True)
        values.update(changes)
        return TurnPhaseARequest(**values)

    def test_gates_precede_application_construction(self):
        created = []
        for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                        {"confirm_wheels_raised": False}, {"confirm_turn_not_calibrated": False},
                        {"slcan_device": "/dev/ttyACM0"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                run_turn_phase_a(self.request(**changes), app_factory=lambda config: created.append(config))
        self.assertEqual(created, [])

    def test_fixed_a4_profile_derives_deadline_from_full_target_without_motion_cli_knobs(self):
        config = phase_a_config(self.request())
        self.assertEqual(TURN_SPEED_MOTOR_RPM, 40.0)
        self.assertEqual(a4_target_timeout_s(720.0, 40.0, 8.0), 24.0 + TURN_TIMEOUT_MARGIN_S)
        self.assertEqual(TURN_TIMEOUT_S, 34.0)
        self.assertGreaterEqual(TURN_TIMEOUT_S, 24.0 + TURN_TIMEOUT_MARGIN_S)
        self.assertEqual(config.turn_speed_rpm, TURN_SPEED_MOTOR_RPM)
        self.assertEqual(config.safety.turn_timeout_s, TURN_TIMEOUT_S)
        self.assertEqual(config.vision.marker, PHASE_A_MARKER)
        self.assertTrue(config.safety.in_row_turn_enabled)
        # 40 motor-RPM remains inside the verified physical output bound and
        # the derived 34 s deadline satisfies the physical A4 validation.
        self.assertIs(config.validate(), config)

    def test_normal_target_completion_is_captured_before_public_stop_disarm(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        result = run_turn_phase_a(self.request(), app_factory=factory)
        self.assertEqual(result.completed_state, "AUTO_ROW_FOLLOW")
        self.assertAlmostEqual(result.encoder_delta_m[0], -1.61)
        self.assertAlmostEqual(result.encoder_delta_m[1], 1.61)
        self.assertEqual(holder[0].runtime.calls, ["select_auto", "arm", "start_auto", "select_manual"])
        self.assertTrue(holder[0].closed)

    def test_success_is_rejected_when_final_close_stop_settle_fails(self):
        holder = []
        def factory(config):
            app = FakeApp(config, close_error=True)
            holder.append(app)
            return app
        with self.assertRaisesRegex(RuntimeError, r"STOP\+0x9C settle failed"):
            run_turn_phase_a(self.request(), app_factory=factory)
        self.assertTrue(holder[0].closed)
        # The runner has already called public MANUAL before close; a close
        # failure must never leave this test fake reporting armed output.
        self.assertFalse(holder[0].runtime.status().motor_output_armed)

    def test_wrong_sign_or_distance_outside_geometry_tolerance_is_rejected(self):
        for delta in ((1.61, -1.61), (-1.50, 1.61)):
            with self.subTest(delta=delta), self.assertRaisesRegex(RuntimeError, "teckenkonsistent|turn-tolerans"):
                run_turn_phase_a(self.request(), app_factory=lambda config: FakeApp(config, delta=delta))

    def test_fault_and_missing_completion_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "ODOMETRY_TIMEOUT"):
            run_turn_phase_a(self.request(), app_factory=lambda config: FakeApp(config, fault="ODOMETRY_TIMEOUT"))
        clock = Clock()
        with self.assertRaisesRegex(TimeoutError, "exakt A4-måldeadline"):
            run_turn_phase_a(self.request(), app_factory=lambda config: FakeApp(config, completed=False),
                             monotonic=clock, sleep=clock.sleep)
        self.assertGreaterEqual(clock.value, TURN_TIMEOUT_S)
        self.assertLessEqual(clock.value, TURN_TIMEOUT_S + POLL_S)

    def test_marker_timeout_has_json_safe_pre_close_diagnostic_without_motor_commands(self):
        holder = []
        def factory(config):
            app = FakeApp(config, marker_ready=False)
            app.runtime.camera = SimpleNamespace(snapshot=lambda: SimpleNamespace(connected=False, error="camera disconnected"))
            app.runtime.imu = SimpleNamespace(snapshot=lambda: SimpleNamespace(connected=True, error=None))
            app.runtime._odometry = SimpleNamespace(snapshot=lambda: SimpleNamespace(connected=True, error="retrying"))
            app.runtime.events = SimpleNamespace(recent=lambda: [
                {"timestamp_s": 1.0, "level": "WARNING", "kind": "camera_retry",
                 "data": {"attempt": 2, "detail": "bounded"}},
            ])
            holder.append(app)
            return app
        clock = Clock()
        with self.assertRaises(MarkerNotReadyError) as raised:
            run_turn_phase_a(self.request(), app_factory=factory, monotonic=clock, sleep=clock.sleep)
        diagnostic = raised.exception.diagnostic
        self.assertEqual(diagnostic["state"], "AUTO_IN_ROW_TURN")
        self.assertFalse(diagnostic["camera"]["connected"])
        self.assertEqual(diagnostic["odometry"]["error"], "retrying")
        self.assertFalse(diagnostic["vision"]["marker_found"])
        self.assertEqual(diagnostic["recent_events"][0]["kind"], "camera_retry")
        self.assertEqual(holder[0].runtime.calls, [])
        self.assertTrue(holder[0].closed)
        self.assertGreaterEqual(clock.value, 30.0)
        json.dumps(diagnostic, allow_nan=False)

    def test_admitted_fault_keeps_pre_close_sources_events_and_post_close_can_evidence(self):
        class Worker:
            def __init__(self): self.closed = False
            def status(self):
                return SimpleNamespace(mode="physical-test", ready=not self.closed, error=None)
            def diagnostic_snapshot(self):
                if not self.closed:
                    raise RuntimeError("must close first")
                return (SimpleNamespace(timestamp_s=1.25, sequence=7, phase="runtime A4",
                                        direction="rx", can_id=0x141, dlc=8,
                                        expected_reply_ids=(0x141,), pending_reply_ids=(),
                                        detail="position poll"),)

        holder = []
        def factory(config):
            app = FakeApp(config, fault="ODOMETRY_TIMEOUT")
            app.runtime.camera = SimpleNamespace(snapshot=lambda: SimpleNamespace(connected=True, error=None))
            app.runtime.imu = SimpleNamespace(snapshot=lambda: SimpleNamespace(connected=True, error=None))
            app.runtime._odometry = SimpleNamespace(snapshot=lambda: SimpleNamespace(connected=False, error="read deadline"))
            app.runtime.events = SimpleNamespace(recent=lambda: [
                {"timestamp_s": 2.0, "level": "ERROR", "kind": "turn_fault",
                 "data": {"reason": "ODOMETRY_TIMEOUT"}},
            ])
            app.runtime.motor = SimpleNamespace(_sink=Worker(), events=[("position", -720.0, 720.0, "turn A4")])
            holder.append(app)
            return app

        with self.assertRaisesRegex(RuntimeError, "ODOMETRY_TIMEOUT") as raised:
            run_turn_phase_a(self.request(), app_factory=factory)
        diagnostic = raised.exception.diagnostic
        self.assertEqual(diagnostic["odometry"]["error"], "read deadline")
        self.assertEqual(diagnostic["recent_events"][0]["kind"], "turn_fault")
        self.assertTrue(diagnostic["can_worker_pre_close"]["status"]["ready"])
        post_close = diagnostic["can_worker_post_close"]
        self.assertEqual(post_close["entries"][0]["phase"], "runtime A4")
        self.assertEqual(post_close["entries"][0]["can_id"], 0x141)
        self.assertTrue(holder[0].closed)
        json.dumps(diagnostic, allow_nan=False)

    def test_close_stop_failure_is_terminal_after_admitted_fault_and_preserves_prior_diagnostic(self):
        holder = []

        def factory(config):
            app = FakeApp(config, fault="ODOMETRY_TIMEOUT", close_error=True)
            app.runtime._odometry = SimpleNamespace(
                snapshot=lambda: SimpleNamespace(connected=False, error="read deadline")
            )
            holder.append(app)
            return app

        with self.assertRaisesRegex(RuntimeError, r"STOP\+0x9C settle failed") as raised:
            run_turn_phase_a(self.request(), app_factory=factory)
        self.assertIsNotNone(raised.exception.__cause__)
        self.assertRegex(str(raised.exception.__cause__), "ODOMETRY_TIMEOUT")
        prior = raised.exception.diagnostic["prior_failure"]
        self.assertEqual(prior["error"], "RuntimeError: A4-vändningen avslutades med fel: ODOMETRY_TIMEOUT")
        self.assertEqual(prior["diagnostic"]["odometry"]["error"], "read deadline")
        self.assertIn("can_worker_post_close", raised.exception.diagnostic)
        self.assertTrue(holder[0].closed)

    def test_cli_has_only_explicit_physical_gates(self):
        allowed = ["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--enable-motors",
                   "--confirm-physical-stop-tested", "--confirm-wheels-raised", "--confirm-turn-not-calibrated"]
        for option in ("--speed", "--direction", "--duration", "--turn-timeout", "--marker-timeout"):
            with self.subTest(option=option), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(allowed + [option, "1"])
            self.assertEqual(raised.exception.code, 2)

    def test_cli_reports_failure(self):
        output = io.StringIO()
        with patch("field_control.turn_phase_a_hil.run_turn_phase_a", side_effect=RuntimeError("bad")), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test"]), 2)
        reported = json.loads(output.getvalue())
        self.assertFalse(reported["ok"])
        with open(self.report_path, encoding="utf-8") as report:
            self.assertEqual(json.load(report), reported)
        self.assertEqual(stat.S_IMODE(os.stat(self.report_path).st_mode), 0o600)

    def test_cli_includes_marker_timeout_diagnostic(self):
        output = io.StringIO()
        diagnostic = {"camera": {"fresh": False, "connected": False}, "recent_events": []}
        with patch("field_control.turn_phase_a_hil.run_turn_phase_a", side_effect=MarkerNotReadyError(diagnostic)), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test"]), 2)
        reported = json.loads(output.getvalue())
        self.assertEqual(reported["diagnostic"], diagnostic)

    def test_cli_persists_bounded_json_safe_success_without_device_paths_or_tokens(self):
        result = TurnPhaseAResult(
            DifferentialTurnPlan(-1.61, 1.61, -1.0, 1.0, "left"), (-1.61, 1.61), "AUTO_ROW_FOLLOW",
            ({"kind": "turn_completed", "data": {"device_path": "/dev/serial/by-id/private",
                                                   "access_token": "private", "nan": float("nan")}},),
        )
        output = io.StringIO()
        with patch("field_control.turn_phase_a_hil.run_turn_phase_a", return_value=result), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test"]), 0)
        reported = json.loads(output.getvalue())
        with open(self.report_path, encoding="utf-8") as report:
            persisted = json.load(report)
        self.assertEqual(persisted, reported)
        self.assertTrue(reported["ok"])
        data = reported["events"][0]["data"]
        self.assertEqual(data["device_path"], "[redacted]")
        self.assertEqual(data["access_token"], "[redacted]")
        self.assertIsNone(data["nan"])
        self.assertEqual(stat.S_IMODE(os.stat(self.report_path).st_mode), 0o600)

    def test_report_write_failure_does_not_mask_terminal_error(self):
        output = io.StringIO()
        with patch("field_control.turn_phase_a_hil.run_turn_phase_a", side_effect=RuntimeError("original failure")), \
             patch("field_control.turn_phase_a_hil.os.replace", side_effect=OSError("full")), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test"]), 2)
        reported = json.loads(output.getvalue())
        self.assertEqual(reported["error"], "RuntimeError: original failure")
        self.assertEqual(reported["report_persistence_error"], "OSError")

    def test_report_fsyncs_file_then_directory_after_atomic_replace(self):
        actual_replace, actual_fsync = os.replace, os.fsync
        order: list[str] = []

        def tracked_replace(source, destination):
            order.append("replace")
            return actual_replace(source, destination)

        def tracked_fsync(fd):
            order.append("fsync")
            return actual_fsync(fd)

        with patch("field_control.turn_phase_a_hil.os.replace", side_effect=tracked_replace), \
             patch("field_control.turn_phase_a_hil.os.fsync", side_effect=tracked_fsync):
            self.assertIsNone(_persist_last_report({"ok": True}))
        self.assertEqual(order, ["fsync", "replace", "fsync"])
        with open(self.report_path, encoding="utf-8") as report:
            self.assertEqual(json.load(report), {"ok": True})
        self.assertEqual(stat.S_IMODE(os.stat(self.report_path).st_mode), 0o600)

    def test_directory_fsync_failure_is_reported_after_atomic_replace(self):
        actual_fsync = os.fsync
        calls = 0

        def fail_directory_fsync(fd):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError("directory sync failed")
            return actual_fsync(fd)

        with patch("field_control.turn_phase_a_hil.os.fsync", side_effect=fail_directory_fsync):
            self.assertEqual(_persist_last_report({"ok": False, "error": "terminal"}), "OSError")
        # The rename has already occurred; reporting the durability uncertainty
        # must not discard the safe terminal record.
        with open(self.report_path, encoding="utf-8") as report:
            self.assertEqual(json.load(report), {"ok": False, "error": "terminal"})


if __name__ == "__main__":
    unittest.main()
