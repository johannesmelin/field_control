from __future__ import annotations

import io
from contextlib import redirect_stdout
import json
import os
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from field_control.odometry import DriveGeometry, OdometrySample
from field_control.turn import new_row_turn_targets
from field_control.turn_new_row_hil import (
    A4_ACTIVE_ADMISSION_TIMEOUT_S, NEW_ROW_DIRECTION, NUMBER_OF_ROWS,
    NEW_ROW_TIMEOUT_MARGIN_S, ROW_SPACING_M, TURN_SPEED_MOTOR_RPM,
    NewRowTurnRequest, NewRowTurnResult, main, new_row_config,
    run_new_row_stop, run_new_row_turn,
)


class Clock:
    def __init__(self): self.value = 0.0
    def __call__(self): return self.value
    def sleep(self, delay): self.value += delay


class FakeMotor:
    def __init__(self, *, active=False):
        self.events = []
        self.active = active
    def position_move_status(self, request):
        return (False, False, None, self.active)
    def position_move_stage(self, request):
        return (self.active, self.active)


class FakeRuntime:
    def __init__(self, config, *, complete=True, active=False, bad_delta=False, settle_error=False):
        self.config, self.complete, self.active, self.bad_delta = config, complete, active, bad_delta
        self.motor = FakeMotor(active=active)
        self.started = self.armed = self.manual = False
        self.settle_error = settle_error
        self._position_turn_request = None
        self.calls = []
        self.events = SimpleNamespace(recent=self._events)

    def _plan(self):
        return new_row_turn_targets(self.config.odometry_geometry, self.config.row_spacing_m,
                                    self.config.turn_speed_rpm, NEW_ROW_DIRECTION)

    def _events(self):
        if not self.started:
            return []
        output = [{"kind": "turn_started", "data": {"state": "AUTO_NEW_ROW_TURN"}}]
        if self.complete:
            output.append({"kind": "turn_completed", "data": {"state": "AUTO_NEW_ROW_TURN"}})
        return output

    def status(self):
        plan = self._plan()
        done = self.started and self.complete and not self.manual
        left = plan.left_distance_m if done else 0.0
        right = plan.right_distance_m if done else 0.0
        if self.bad_delta and done: left = -left
        sample = OdometrySample(left, right, 0.0, 0.0)
        observation = SimpleNamespace(camera_fresh=True, imu_fresh=True, odometry_fresh=True,
                                      vision=SimpleNamespace(marker_found=True), visual_target=True,
                                      odometry_sample=sample)
        state = "MANUAL" if self.manual else ("AUTO_SEARCH" if done else "AUTO_NEW_ROW_TURN")
        return SimpleNamespace(observation=observation, fault=None, state=state,
                               motor_output_armed=self.armed and not self.manual)

    def select_auto(self): self.calls.append("select_auto")
    def arm_motor_output(self): self.calls.append("arm"); self.armed = True
    def start_auto(self):
        self.calls.append("start_auto"); self.started = True
        if self.active:
            self._position_turn_request = object()
            self.motor.events.append(("position", 137.0, 1549.0, "turn A4"))
    def stop(self): self.calls.append("stop"); self.manual = True; self.armed = False
    def stop_and_settle(self):
        self.stop()
        self.calls.append("settle")
        if self.settle_error:
            raise RuntimeError("STOP+0x9C settle failed")
    def select_manual(self): self.calls.append("select_manual"); self.manual = True; self.armed = False


class FakeApp:
    def __init__(self, config, *, complete=True, active=False, bad_delta=False, settle_error=False):
        self.runtime = FakeRuntime(config, complete=complete, active=active, bad_delta=bad_delta,
                                   settle_error=settle_error)
        self.closed = False
    def start(self): pass
    def close(self): self.closed = True


class DelayedPostArmOdometry:
    """Raw source seam: status may look fresh while source recovery lags."""
    def __init__(self, clock, *, ready_after_s):
        self.clock, self.ready_after_s = clock, ready_after_s
        self.sample = OdometrySample(0.0, 0.0, 0.0, 0.0)

    def snapshot(self):
        timestamp = .0 if self.clock.value < self.ready_after_s else self.clock.value
        return SimpleNamespace(
            connected=self.clock.value >= self.ready_after_s,
            value=self.sample if self.clock.value >= self.ready_after_s else None,
            updated_at_s=timestamp if self.clock.value >= self.ready_after_s else None,
        )


class NewRowHilTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.report_path = os.path.join(self.temporary_directory.name, "last-report.json")
        self.report_patch = patch("field_control.turn_new_row_hil.LAST_REPORT_PATH", self.report_path)
        self.report_patch.start()

    def tearDown(self):
        self.report_patch.stop()
        self.temporary_directory.cleanup()

    def request(self, **changes):
        fields = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True,
                      confirm_turn_not_calibrated=True)
        fields.update(changes)
        return NewRowTurnRequest(**fields)

    def test_fixed_profile_is_40_motor_rpm_and_geometry_asymmetric(self):
        config = new_row_config(self.request())
        plan = new_row_turn_targets(config.odometry_geometry, ROW_SPACING_M, TURN_SPEED_MOTOR_RPM,
                                    NEW_ROW_DIRECTION)
        self.assertEqual(config.turn_speed_rpm, 40.0)
        self.assertFalse(config.safety.in_row_turn_enabled)
        self.assertEqual(config.safety.number_of_rows, NUMBER_OF_ROWS)
        self.assertGreater(plan.right_distance_m, plan.left_distance_m)
        self.assertAlmostEqual(plan.left_distance_m, .306305, places=5)
        self.assertAlmostEqual(plan.right_distance_m, 3.463606, places=5)
        self.assertEqual(NEW_ROW_TIMEOUT_MARGIN_S, 30.0)
        self.assertAlmostEqual(config.safety.turn_timeout_s, 81.6314, places=3)

    def test_all_physical_gates_precede_application_construction(self):
        created = []
        for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                        {"confirm_wheels_raised": False}, {"confirm_turn_not_calibrated": False},
                        {"slcan_device": "/dev/ttyACM0"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                run_new_row_turn(self.request(**changes), app_factory=lambda config: created.append(config))
        self.assertEqual(created, [])

    def test_normal_new_row_requires_asymmetric_signed_encoder_target_and_public_disarm(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        result = run_new_row_turn(self.request(), app_factory=factory)
        self.assertEqual(result.completed_state, "AUTO_SEARCH")
        self.assertGreater(result.encoder_delta_m[0], 0)
        self.assertGreater(result.encoder_delta_m[1], result.encoder_delta_m[0])
        self.assertEqual(holder[0].runtime.calls, ["select_auto", "arm", "start_auto", "select_manual"])
        self.assertTrue(holder[0].closed)

    def test_wrong_encoder_sign_is_rejected_and_closed(self):
        holder = []
        def factory(config):
            app = FakeApp(config, bad_delta=True); holder.append(app); return app
        with self.assertRaisesRegex(RuntimeError, "teckenkonsistent"):
            run_new_row_turn(self.request(), app_factory=factory)
        self.assertTrue(holder[0].closed)

    def test_failure_retains_bounded_pre_and_post_close_worker_diagnostics(self):
        class Worker:
            def status(self):
                return SimpleNamespace(mode="physical-test", ready=True, error=None)
            def diagnostic_snapshot(self):
                return (SimpleNamespace(timestamp_s=1.0, sequence=2, phase="runtime A4",
                                        direction="rx", can_id=0x142, dlc=8,
                                        data=b"\x92\0\0\0\0\0\0\0",
                                        expected_reply_ids=(0x142,), pending_reply_ids=(),
                                        detail="position poll"),)

        holder = []
        def factory(config):
            app = FakeApp(config, bad_delta=True)
            app.runtime.motor._sink = Worker()
            holder.append(app)
            return app

        with self.assertRaisesRegex(RuntimeError, "teckenkonsistent") as raised:
            run_new_row_turn(self.request(), app_factory=factory)
        diagnostic = raised.exception.diagnostic
        self.assertEqual(diagnostic["can_worker_pre_close"]["status"]["mode"], "physical-test")
        self.assertEqual(diagnostic["can_worker_post_close"]["entries"][0]["can_id"], 0x142)
        json.dumps(diagnostic, allow_nan=False)
        self.assertTrue(holder[0].closed)

    def test_stop_waits_for_active_a4_then_stays_manual_disarmed_without_re_admission(self):
        holder = []
        def factory(config):
            app = FakeApp(config, complete=False, active=True); holder.append(app); return app
        clock = Clock()
        result = run_new_row_stop(self.request(), app_factory=factory, monotonic=clock, sleep=clock.sleep)
        self.assertEqual(result.position_events_before_stop, 1)
        self.assertEqual(result.position_events_after_stop, 1)
        self.assertEqual(result.completed_state, "MANUAL")
        self.assertEqual(holder[0].runtime.calls, ["select_auto", "arm", "start_auto", "stop", "settle", "select_manual"])
        self.assertTrue(holder[0].closed)
        self.assertLess(clock.value, A4_ACTIVE_ADMISSION_TIMEOUT_S)

    def test_stop_start_waits_for_new_connected_odometry_after_arm_stop(self):
        """Do not enter AUTO in arm's intentional STOP-preemption window."""
        holder = []
        clock = Clock()

        def factory(config):
            app = FakeApp(config, complete=False, active=True)
            # This models the real source after arm's STOP has invalidated its
            # cached pair: the RuntimeStatus fixture remains marker-ready, but
            # raw physical odometry reconnects only on the next 100 ms sample.
            app.runtime._odometry = DelayedPostArmOdometry(clock, ready_after_s=.100)
            holder.append(app)
            return app

        result = run_new_row_stop(self.request(), app_factory=factory,
                                  monotonic=clock, sleep=clock.sleep)
        self.assertEqual(result.completed_state, "MANUAL")
        self.assertGreaterEqual(clock.value, .100)
        self.assertLess(holder[0].runtime.calls.index("start_auto"),
                        holder[0].runtime.calls.index("stop"))

    def test_stop_start_fails_closed_when_no_post_arm_odometry_sample_arrives(self):
        holder = []
        clock = Clock()

        def factory(config):
            app = FakeApp(config, complete=False, active=True)
            app.runtime._odometry = DelayedPostArmOdometry(clock, ready_after_s=99.0)
            holder.append(app)
            return app

        with self.assertRaisesRegex(TimeoutError, "POST_ARM_ODOMETRY_NOT_READY"):
            run_new_row_stop(self.request(), app_factory=factory,
                             monotonic=clock, sleep=clock.sleep)
        self.assertNotIn("start_auto", holder[0].runtime.calls)
        self.assertIn("select_manual", holder[0].runtime.calls)
        self.assertTrue(holder[0].closed)

    def test_stop_refuses_to_claim_success_without_active_a4(self):
        clock = Clock()
        with self.assertRaisesRegex(TimeoutError, "blev inte aktiv"):
            run_new_row_stop(self.request(), app_factory=lambda config: FakeApp(config, complete=False),
                             monotonic=clock, sleep=clock.sleep)
        self.assertGreaterEqual(clock.value, A4_ACTIVE_ADMISSION_TIMEOUT_S)

    def test_stop_settle_failure_is_terminal_and_app_closes(self):
        holder = []
        def factory(config):
            app = FakeApp(config, complete=False, active=True, settle_error=True); holder.append(app); return app
        with self.assertRaisesRegex(RuntimeError, "settle failed"):
            run_new_row_stop(self.request(), app_factory=factory)
        self.assertTrue(holder[0].closed)

    def test_cli_persists_atomic_private_success_report(self):
        result = NewRowTurnResult(
            new_row_turn_targets(DriveGeometry(), ROW_SPACING_M, TURN_SPEED_MOTOR_RPM, NEW_ROW_DIRECTION),
            (.3, 3.4), "AUTO_SEARCH", ({"kind": "turn_completed"},),
        )
        output = io.StringIO()
        with patch("field_control.turn_new_row_hil.run_new_row_turn", return_value=result), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test"]), 0)
        reported = json.loads(output.getvalue())
        self.assertTrue(reported["ok"])
        self.assertEqual(reported["result"]["direction"], NEW_ROW_DIRECTION)
        self.assertEqual(reported["result"]["events"][0]["kind"], "turn_completed")
        with open(self.report_path, encoding="utf-8") as report:
            self.assertEqual(json.load(report), reported)
        self.assertEqual(stat.S_IMODE(os.stat(self.report_path).st_mode), 0o600)

    def test_cli_persists_bounded_error_report(self):
        error = RuntimeError("bounded failure")
        error.diagnostic = {"can_worker_post_close": {"entries": [{"can_id": 0x141}]}}
        output = io.StringIO()
        with patch("field_control.turn_new_row_hil.run_new_row_turn", side_effect=error), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test"]), 2)
        reported = json.loads(output.getvalue())
        self.assertFalse(reported["ok"])
        self.assertEqual(reported["diagnostic"]["can_worker_post_close"]["entries"][0]["can_id"], 0x141)
        with open(self.report_path, encoding="utf-8") as report:
            self.assertEqual(json.load(report), reported)


if __name__ == "__main__":
    unittest.main()
