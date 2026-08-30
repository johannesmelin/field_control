from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from field_control.auto_stop_ground_diagnostic import (
    AUTO_SEARCH_TIMEOUT_S, DISABLED_MIN_AREA, IMU_ONLY_DISTANCE_M, MOTOR_RPM, SENSOR_READY_TIMEOUT_S,
    AutoStopGroundDiagnosticRequest, ground_auto_stop_config, run_auto_stop_ground_diagnostic,
)


class Entry:
    timestamp_s = 1.0; sequence = 1; phase = "runtime stop settle"; direction = "rx"
    can_id = 0x142; dlc = 8; data = bytes((0x9C, 0, 0, 0, 0, 0, 0, 0))
    expected_reply_ids = (0x142,); pending_reply_ids = (); detail = "0x9C sample 0 dps after 20 ms"


class Runtime:
    def __init__(self, *, fault: str | None, ready_after: int = 0):
        self.fault, self.calls, self.ready_after, self.status_calls = fault, [], ready_after, 0
        self.motor = SimpleNamespace(_sink=SimpleNamespace(diagnostic_snapshot=lambda: (Entry(),)))
        self.events = SimpleNamespace(recent=lambda: [{"kind": "fault", "data": {"reason": fault}}])
    def select_auto(self): self.calls.append("select_auto")
    def arm_motor_output(self): self.calls.append("arm")
    def start_auto(self): self.calls.append("start_auto")
    def stop(self): self.calls.append("stop")
    def select_manual(self): self.calls.append("select_manual")
    def status(self):
        self.status_calls += 1
        ready = self.status_calls > self.ready_after
        observation = SimpleNamespace(camera_fresh=ready, imu_fresh=ready, odometry_fresh=ready)
        return SimpleNamespace(state="FAULT" if self.fault else "AUTO_SEARCH", fault=self.fault,
                               observation=observation)


class App:
    def __init__(self, config, *, fault: str | None, ready_after: int = 0):
        self.config, self.runtime, self.closed = config, Runtime(fault=fault, ready_after=ready_after), False
    def start(self): pass
    def close(self): self.closed = True


class AutoStopGroundDiagnosticTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable-test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_ground_clear=True,
                      confirm_emergency_stop_ready=True)
        values.update(changes)
        return AutoStopGroundDiagnosticRequest(**values)

    def test_fixed_config_disables_detections_and_turns_without_deployment_shortcuts(self):
        config = ground_auto_stop_config(self.request())
        self.assertEqual((config.max_rpm, config.auto_base_rpm, config.search_speed_rpm), (MOTOR_RPM,) * 3)
        self.assertEqual(config.safety.search_length_m, IMU_ONLY_DISTANCE_M)
        self.assertFalse(config.safety.in_row_turn_enabled)
        self.assertEqual((config.vision.buds.min_area, config.vision.leaves.min_area, config.vision.marker.min_area),
                         (DISABLED_MIN_AREA,) * 3)
        self.assertTrue(config.physical_can.confirm_ground_test)
        self.assertFalse(config.physical_can.confirm_wheels_raised)

    def test_gates_fail_before_application_construction(self):
        created = []
        for changes in ({"enable_motors": False}, {"confirm_ground_clear": False},
                        {"confirm_emergency_stop_ready": False}, {"slcan_device": "/dev/ttyUSB0"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                run_auto_stop_ground_diagnostic(self.request(**changes), app_factory=lambda config: created.append(config))
        self.assertEqual(created, [])

    def test_row_lost_captures_report_then_always_stops_manual_and_closes(self):
        holder = []
        def factory(config):
            app = App(config, fault="ROW_LOST"); holder.append(app); return app
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.auto_stop_ground_diagnostic._write_report", return_value=Path(directory) / "report.json") as write:
            result = run_auto_stop_ground_diagnostic(self.request(), app_factory=factory)
        self.assertEqual(result.terminal_outcome, "row_lost")
        self.assertEqual(holder[0].runtime.calls, ["select_auto", "arm", "start_auto", "stop", "select_manual"])
        self.assertTrue(holder[0].closed)
        self.assertEqual(result.worker_diagnostics[0]["can_id"], 0x142)
        self.assertTrue(write.call_args.args[0]["ok"])

    def test_worker_fault_is_reported_and_cleanup_still_runs(self):
        holder = []
        def factory(config):
            app = App(config, fault="CAN-worker timeout"); holder.append(app); return app
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.auto_stop_ground_diagnostic._write_report", return_value=Path(directory) / "report.json") as write:
            result = run_auto_stop_ground_diagnostic(self.request(), app_factory=factory)
        self.assertEqual(result.terminal_outcome, "fault")
        self.assertEqual(holder[0].runtime.calls[-2:], ["stop", "select_manual"])
        self.assertTrue(holder[0].closed)
        self.assertFalse(write.call_args.args[0]["ok"])

    def test_delayed_sensor_readiness_precedes_auto_arm_and_start(self):
        now, holder = [0.0], []
        def clock(): return now[0]
        def sleep(delay): now[0] += delay
        def factory(config):
            app = App(config, fault="ROW_LOST", ready_after=2); holder.append(app); return app
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.auto_stop_ground_diagnostic._write_report", return_value=Path(directory) / "report.json"):
            result = run_auto_stop_ground_diagnostic(self.request(), app_factory=factory,
                                                      monotonic=clock, sleep=sleep)
        self.assertEqual(result.terminal_outcome, "row_lost")
        self.assertGreaterEqual(holder[0].runtime.status_calls, 4)
        self.assertEqual(holder[0].runtime.calls[:3], ["select_auto", "arm", "start_auto"])

    def test_readiness_timeout_admits_no_auto_or_arm(self):
        now, holder = [0.0], []
        def clock(): return now[0]
        def sleep(delay): now[0] += delay
        def factory(config):
            app = App(config, fault=None, ready_after=10_000); holder.append(app); return app
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.auto_stop_ground_diagnostic._write_report", return_value=Path(directory) / "report.json"):
            result = run_auto_stop_ground_diagnostic(self.request(), app_factory=factory,
                                                      monotonic=clock, sleep=sleep)
        self.assertEqual(result.terminal_outcome, "sensor_readiness_timeout")
        self.assertEqual(result.terminal_fault, "SENSOR_READINESS_TIMEOUT")
        self.assertGreaterEqual(now[0], SENSOR_READY_TIMEOUT_S)
        self.assertEqual(holder[0].runtime.calls, ["stop", "select_manual"])
        self.assertTrue(holder[0].closed)

    def test_fresh_snapshot_at_or_after_readiness_deadline_is_rejected_without_arm(self):
        for offset in (0.0, 0.001):
            with self.subTest(offset=offset):
                clock_calls, holder = [0], []
                def clock():
                    clock_calls[0] += 1
                    return 0.0 if clock_calls[0] == 1 else SENSOR_READY_TIMEOUT_S + offset
                def factory(config):
                    app = App(config, fault="ROW_LOST"); holder.append(app); return app
                with tempfile.TemporaryDirectory() as directory, \
                     patch("field_control.auto_stop_ground_diagnostic._write_report", return_value=Path(directory) / "report.json"):
                    result = run_auto_stop_ground_diagnostic(self.request(), app_factory=factory,
                                                              monotonic=clock, sleep=lambda _delay: None)
                self.assertEqual(result.terminal_outcome, "sensor_readiness_timeout")
                self.assertEqual(holder[0].runtime.calls, ["stop", "select_manual"])

    def test_status_snapshot_crossing_deadline_cannot_grant_auto_or_arm(self):
        now, holder = [0.0], []
        def clock(): return now[0]
        def factory(config):
            app = App(config, fault="ROW_LOST")
            def status():
                now[0] = SENSOR_READY_TIMEOUT_S
                observation = SimpleNamespace(camera_fresh=True, imu_fresh=True, odometry_fresh=True)
                return SimpleNamespace(state="MANUAL", fault=None, observation=observation)
            app.runtime.status = status
            holder.append(app)
            return app
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.auto_stop_ground_diagnostic._write_report", return_value=Path(directory) / "report.json"):
            result = run_auto_stop_ground_diagnostic(self.request(), app_factory=factory,
                                                      monotonic=clock, sleep=lambda _delay: None)
        self.assertEqual(result.terminal_outcome, "sensor_readiness_timeout")
        self.assertEqual(holder[0].runtime.calls, ["stop", "select_manual"])


if __name__ == "__main__":
    unittest.main()
