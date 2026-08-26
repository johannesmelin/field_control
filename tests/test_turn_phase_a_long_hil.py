from __future__ import annotations

import io
from contextlib import redirect_stderr, redirect_stdout
import json
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

from field_control.control import WheelCommand
from field_control.odometry import OdometrySample
from field_control.sources import SourceSnapshot
from field_control.turn_phase_a_hil import PHASE_A_MARKER
from field_control.turn_phase_a_long_hil import (
    MAX_NOMINAL_TRAVEL_RATIO, MIN_NOMINAL_TRAVEL_RATIO, TURN_SPEED_MOTOR_RPM,
    TURN_TIMEOUT_S, TurnPhaseALongRequest, main, nominal_wheel_travel_m,
    phase_a_long_config, run_turn_phase_a_long, _bounded_events, _runtime_diagnostics,
)


class FakeRuntime:
    def __init__(self, *, marker=True, fault="TURN_TIMEOUT", command=(-2.0, 2.0),
                 delta=(-.100625, .100625), events=None, odometry_snapshot=None):
        self.marker, self.fault, self.command, self.delta = marker, fault, command, delta
        self.started_auto, self.armed, self.selected_auto, self.turn_ticks = False, False, False, 0
        self.events = SimpleNamespace(recent=lambda: events if events is not None else [
            {"kind": "turn_started", "data": {}}, {"kind": "fault", "data": {"reason": "TURN_TIMEOUT"}},
        ])
        self.calls = []
        self.motor = FakeMotor()
        self._odometry = FakeOdometry(odometry_snapshot)

    def status(self):
        if self.started_auto:
            self.turn_ticks += 1
        sample = OdometrySample(1.0 if not self.started_auto else 1.0 + self.delta[0],
                                -1.0 if not self.started_auto else -1.0 + self.delta[1], 0, 0)
        observation = SimpleNamespace(camera_fresh=True, imu_fresh=True, odometry_fresh=True,
                                      vision=SimpleNamespace(marker_found=self.marker),
                                      visual_target=self.marker, odometry_sample=sample)
        active_fault = self.fault if self.started_auto and self.turn_ticks >= 3 else None
        command = WheelCommand(*self.command, "turn") if active_fault is not None else None
        return SimpleNamespace(observation=observation, fault=active_fault,
                               motor_output_armed=self.armed and active_fault is None,
                               last_command=command)

    def select_auto(self):
        self.calls.append("select_auto")
        if self.armed:
            raise AssertionError("select_auto must precede arm")
        self.selected_auto = True

    def arm_motor_output(self):
        self.calls.append("arm")
        if not self.selected_auto:
            raise AssertionError("AUTO must be selected while disarmed")
        self.armed = True

    def start_auto(self):
        self.calls.append("start_auto")
        self.started_auto = True


class FakeOdometry:
    def __init__(self, snapshot):
        self._snapshot = snapshot or SourceSnapshot(
            OdometrySample(1.0, -1.0, 0.0, 0.0), time.monotonic(), True, None,
        )

    def snapshot(self):
        return self._snapshot


class FakeSink:
    def __init__(self):
        self.released, self.diagnostic_calls, self.send_calls = False, 0, 0

    def diagnostic_snapshot(self):
        self.diagnostic_calls += 1
        if not self.released:
            raise AssertionError("worker diagnostics must be read only after close")
        return ("released-worker-ring",)


class FakeMotor:
    def __init__(self):
        self._sink = FakeSink()


class FakeApp:
    def __init__(self, config, *, close_error=False, **runtime_options):
        self.config, self.runtime = config, FakeRuntime(**runtime_options)
        self.started = self.closed = False
        self.close_error = close_error

    def start(self): self.started = True
    def close(self):
        self.runtime.motor._sink.released = True
        self.closed = True
        if self.close_error:
            raise RuntimeError("close failed")


class TurnPhaseALongHilTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_motors=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True,
                      confirm_turn_not_calibrated=True)
        values.update(changes)
        return TurnPhaseALongRequest(**values)

    def test_gates_precede_application_construction(self):
        created = []
        def factory(config):
            created.append(config)
            return FakeApp(config)
        for changes in ({"enable_motors": False}, {"confirm_physical_stop_tested": False},
                        {"confirm_wheels_raised": False}, {"confirm_turn_not_calibrated": False},
                        {"slcan_device": "/dev/ttyACM0"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                run_turn_phase_a_long(self.request(**changes), app_factory=factory)
        self.assertEqual(created, [])

    def test_fixed_profile_is_30_seconds_at_2_motor_rpm_with_nominal_10_cm_travel(self):
        config = phase_a_long_config(self.request())
        self.assertEqual(TURN_SPEED_MOTOR_RPM, 2.0)
        self.assertEqual(TURN_TIMEOUT_S, 30.0)
        self.assertEqual(config.turn_speed_rpm, 2.0)
        self.assertEqual(config.max_rpm, 2.0)
        self.assertEqual(config.safety.turn_timeout_s, 30.0)
        self.assertEqual(config.auto_base_rpm, 0.0)
        self.assertEqual(config.vision.marker, PHASE_A_MARKER)
        self.assertEqual(config.safety.turn_marker_confirm_frames, 3)
        self.assertEqual(nominal_wheel_travel_m(config), (.100625, .100625))
        self.assertEqual((MIN_NOMINAL_TRAVEL_RATIO, MAX_NOMINAL_TRAVEL_RATIO), (.80, 1.20))

    def test_normal_public_lifecycle_requires_timeout_stop_and_10cm_encoder_evidence(self):
        holder = []
        def factory(config):
            app = FakeApp(config); holder.append(app); return app
        result = run_turn_phase_a_long(self.request(), app_factory=factory)
        self.assertEqual(result.fault, "TURN_TIMEOUT")
        self.assertEqual(result.command_sign, (-1, 1))
        self.assertAlmostEqual(result.encoder_delta_m[0], -.100625)
        self.assertAlmostEqual(result.encoder_delta_m[1], .100625)
        self.assertEqual(result.nominal_wheel_travel_m, (.100625, .100625))
        self.assertEqual(holder[0].runtime.calls, ["select_auto", "arm", "start_auto"])
        self.assertEqual(holder[0].runtime.turn_ticks, 3)
        self.assertTrue(holder[0].closed)

    def test_encoder_distance_outside_fixed_observation_interval_is_rejected(self):
        for delta in ((-.0804, .0804), (-.1208, .1208), (-.100625, -.100625)):
            with self.subTest(delta=delta), self.assertRaisesRegex(RuntimeError, "80--120 procent|teckenkonsistent"):
                run_turn_phase_a_long(self.request(), app_factory=lambda config: FakeApp(config, delta=delta))

    def test_non_timeout_terminal_fault_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "TURN_HEADING_STALE"):
            run_turn_phase_a_long(self.request(), app_factory=lambda config: FakeApp(config, fault="TURN_HEADING_STALE"))

    def test_successful_terminal_evidence_is_not_returned_when_close_fails(self):
        with self.assertRaisesRegex(RuntimeError, "close failed"):
            run_turn_phase_a_long(self.request(), app_factory=lambda config: FakeApp(config, close_error=True))

    def test_odometry_timeout_attaches_bounded_terminal_diagnostics_after_safe_close(self):
        holder = []
        snapshot = SourceSnapshot(
            OdometrySample(1.25, -1.25, 0.0, 0.0), time.monotonic() - .5,
            False, "PhysicalCanError: angle reply missing for 0x142",
        )
        events = [
            {"timestamp_s": 1.0, "level": "INFO", "kind": "turn_started", "data": {}},
            {"timestamp_s": 1.1, "level": "ERROR", "kind": "fault", "data": {"reason": "ODOMETRY_TIMEOUT"}},
        ]
        def factory(config):
            app = FakeApp(config, fault="ODOMETRY_TIMEOUT", events=events, odometry_snapshot=snapshot)
            holder.append(app)
            return app
        with self.assertRaisesRegex(RuntimeError, "ODOMETRY_TIMEOUT") as raised:
            run_turn_phase_a_long(self.request(), app_factory=factory)
        diagnostics = raised.exception.diagnostics
        self.assertEqual(diagnostics["runtime"]["fault"], "ODOMETRY_TIMEOUT")
        self.assertEqual(diagnostics["runtime"]["last_command"]["source"], "turn")
        self.assertEqual(diagnostics["odometry"]["connected"], False)
        self.assertEqual(diagnostics["odometry"]["error"], "PhysicalCanError: angle reply missing for 0x142")
        self.assertIsInstance(diagnostics["odometry"]["age_s"], float)
        self.assertEqual(diagnostics["odometry"]["value"]["left_distance_m"], 1.25)
        self.assertEqual([event["kind"] for event in diagnostics["events"]], ["turn_started", "fault"])
        self.assertIn("released-worker-ring", diagnostics["can"]["worker"])
        sink = holder[0].runtime.motor._sink
        self.assertTrue(holder[0].closed)
        self.assertEqual(sink.diagnostic_calls, 1)
        self.assertEqual(sink.send_calls, 0)

    def test_event_diagnostics_bound_large_snapshot_and_untrusted_data_before_json(self):
        huge_text = "x" * 10000
        data = {f"{item}-" + ("very-long-key-" * 20): huge_text for item in range(40)}
        events = [
            {"timestamp_s": index, "level": "INFO", "kind": f"event-{index}", "data": data}
            for index in range(1000)
        ]
        runtime = SimpleNamespace(events=SimpleNamespace(recent=lambda: events))
        diagnostics = _bounded_events(runtime)
        self.assertEqual(len(diagnostics), 16)
        self.assertEqual(diagnostics[0]["kind"], "event-984")
        self.assertEqual(len(diagnostics[0]["data"]), 8)
        key, value = next(iter(diagnostics[0]["data"].items()))
        self.assertLessEqual(len(key), 128)
        self.assertLessEqual(len(value), 320)
        self.assertLess(len(json.dumps(diagnostics)), 60000)

    def test_event_diagnostics_refuse_infinite_generator_without_iteration(self):
        def infinite_events():
            while True:
                yield {"kind": "never-consumed", "data": {}}
        runtime = SimpleNamespace(events=SimpleNamespace(recent=infinite_events))
        self.assertEqual(_bounded_events(runtime), [{
            "kind": "diagnostics_error", "data": "runtime events har ogiltig icke-sekvens-typ",
        }])

    def test_runtime_diagnostics_sanitize_huge_terminal_fields_before_json(self):
        huge = "x" * 10000
        status = SimpleNamespace(
            mode=huge, state=huge, fault=huge, motor_output_armed=False,
            last_command=SimpleNamespace(left_rpm=huge, right_rpm=10 ** 2000, source=huge),
        )
        snapshot = SourceSnapshot(OdometrySample(float("inf"), 0.0, 0.0, 0.0), 1.0, False, huge)
        runtime = SimpleNamespace(
            status=lambda: status,
            _odometry=SimpleNamespace(snapshot=lambda: snapshot),
            events=SimpleNamespace(recent=lambda: []),
        )
        diagnostics = _runtime_diagnostics(runtime, 2.0)
        terminal = diagnostics["runtime"]
        self.assertEqual(len(terminal["mode"]), 1000)
        self.assertEqual(len(terminal["state"]), 1000)
        self.assertEqual(len(terminal["fault"]), 1000)
        self.assertEqual(len(terminal["last_command"]["source"]), 1000)
        self.assertEqual(len(terminal["last_command"]["left_rpm"]), 1000)
        self.assertIsInstance(terminal["last_command"]["right_rpm"], str)
        self.assertEqual(len(terminal["last_command"]["right_rpm"]), 1000)
        self.assertEqual(len(diagnostics["odometry"]["error"]), 1000)
        self.assertEqual(diagnostics["odometry"]["value"]["left_distance_m"], "inf")
        json.dumps(diagnostics)

    def test_cli_has_no_time_speed_or_direction_knobs(self):
        allowed = ["--slcan-device", "/dev/serial/by-id/usb-CANable_test", "--enable-motors",
                   "--confirm-physical-stop-tested", "--confirm-wheels-raised", "--confirm-turn-not-calibrated"]
        for option in ("--speed", "--direction", "--duration", "--turn-timeout", "--marker-timeout"):
            with self.subTest(option=option), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(allowed + [option, "1"])
            self.assertEqual(raised.exception.code, 2)

    def test_cli_reports_ok_false_when_runner_fails(self):
        output = io.StringIO()
        with patch("field_control.turn_phase_a_long_hil.run_turn_phase_a_long",
                   side_effect=RuntimeError("close failed")), redirect_stdout(output):
            self.assertEqual(main(["--slcan-device", "/dev/serial/by-id/usb-CANable_test"]), 2)
        self.assertFalse(json.loads(output.getvalue())["ok"])


if __name__ == "__main__":
    unittest.main()
