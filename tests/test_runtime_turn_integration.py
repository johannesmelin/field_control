"""Mock-only runtime integration for the pure in-row/new-row turn controller."""
from __future__ import annotations

import threading
import unittest

from field_control.config import RuntimeConfig
from field_control.observation import ImuReading
from field_control.observation import Observation as SensorObservation
from field_control.odometry import DriveGeometry, OdometrySample
from field_control.runtime import FieldControlRuntime, _Lifecycle
from field_control.lease import ControlLease
from field_control.sources import EncoderReadPreempted, LatestValue, OdometrySource
from field_control.state_machine import Observation as MachineObservation, SafetyConfig, State


class Source:
    def __init__(self): self.latest = LatestValue()
    def start(self): pass
    def stop(self): pass
    def snapshot(self): return self.latest.snapshot()


class RecordingMotor:
    armed = True
    fault_reason = None
    def __init__(self): self.commands = []; self.stops = []
    def command(self, command, _token=None): self.commands.append(command)
    def stop_all(self, reason): self.stops.append(reason)
    def hold_stopped(self, reason, _token=None): self.stops.append(reason)


class PositionMotor(RecordingMotor):
    armed = True
    def __init__(self):
        super().__init__()
        self.requests = []
        # Represents the A4 pair accepted by the physical boundary.  The
        # boundary owns actual CAN I/O; a runtime test must prove it admits
        # this transaction even when the periodic encoder cache is stale.
        self.a4_transactions = []
        self.status = (False, False, None, False)
    def arm(self, token): pass
    def begin_wheel_position_move(self, **kwargs):
        self.requests.append(kwargs)
        self.a4_transactions.append((kwargs["left_wheel_degrees"], kwargs["right_wheel_degrees"]))
        return object()
    def position_move_status(self, request): return self.status


class RuntimeTurnIntegrationTests(unittest.TestCase):
    def make_runtime(self, state=State.AUTO_IN_ROW_TURN, *, confirmations=2, turn_speed=10):
        self.now = [0.0]
        safety = SafetyConfig(in_row_turn_enabled=True, new_row_turn_direction="left",
                              turn_heading_confirm_frames=confirmations,
                              turn_heading_tolerance_deg=2, turn_distance_tolerance_m=.01,
                              turn_timeout_s=2, turn_heading_max_age_s=.2)
        config = RuntimeConfig(stream_enabled=False, max_rpm=20, turn_speed_rpm=turn_speed,
                               heading_filter_alpha=1, camera_timeout_s=1, imu_timeout_s=1,
                               odometry_timeout_s=1, safety=safety)
        camera, imu, odometry = Source(), Source(), Source()
        camera.latest.publish(None, 0.0)
        imu.latest.publish(ImuReading(10, 0.0), 0.0)
        odometry.latest.publish(OdometrySample(0, 0, 0, 0), 0.0)
        motor = RecordingMotor()
        runtime = FieldControlRuntime(config, camera, imu, motor=motor, odometry=odometry,
                                      clock=lambda: self.now[0])
        runtime.heading.reference.reference_deg = 10
        runtime.heading.reference.reliable = True
        with runtime._state_lock:
            runtime.machine._transition(state, "test turn")
        return runtime, imu, odometry, motor

    def publish(self, imu, odometry, heading, sample):
        imu.latest.publish(ImuReading(heading, self.now[0]), self.now[0])
        odometry.latest.publish(sample, self.now[0])

    def test_in_row_turn_creates_one_controller_and_admits_motor_side_command(self):
        runtime, imu, odometry, motor = self.make_runtime()
        runtime.tick()
        controller = runtime._turn_controller
        self.assertIsNotNone(controller)
        self.assertEqual((motor.commands[-1].left_rpm, motor.commands[-1].right_rpm), (-10, 10))
        self.assertEqual(motor.commands[-1].source, "turn")
        self.now[0] = .05; self.publish(imu, odometry, 10, OdometrySample(0, 0, 0, 0))
        runtime.tick()
        self.assertIs(runtime._turn_controller, controller)

    def test_configuration_restart_blocks_a4_after_pre_admission_pause(self):
        runtime, _imu, _odometry, _motor = self.make_runtime()
        motor = PositionMotor()
        runtime.motor = motor
        runtime._lifecycle = _Lifecycle.RUNNING
        runtime._lease_token = runtime.lease.acquire()
        motor.stop_and_settle_for_restart = lambda _reason: setattr(motor, "armed", False)
        entered, release = threading.Event(), threading.Event()
        runtime._before_position_command_admission = lambda: (entered.set(), release.wait(.5))
        tick = threading.Thread(target=runtime.tick, daemon=True)
        tick.start(); self.assertTrue(entered.wait(.2))
        self.assertTrue(runtime.reserve_configuration_restart())
        release.set(); tick.join(.5)
        self.assertFalse(tick.is_alive())
        self.assertEqual(motor.requests, [])
        self.assertEqual(motor.a4_transactions, [])

    def test_new_row_turn_uses_same_single_controller_path(self):
        runtime, _imu, _odometry, motor = self.make_runtime(State.AUTO_NEW_ROW_TURN)
        runtime.tick()
        self.assertIsNotNone(runtime._turn_controller)
        self.assertEqual(motor.commands[-1].source, "turn")
        self.assertGreater(motor.commands[-1].left_rpm, 0)
        self.assertGreater(motor.commands[-1].right_rpm, 0)

    def test_confirmed_success_holds_stopped_updates_heading_once_and_completes(self):
        runtime, imu, odometry, motor = self.make_runtime(confirmations=2)
        runtime.tick()
        # In-row left plan is -/+ two wheel turns from its immutable baseline.
        target = runtime._turn_controller.plan
        self.now[0] = .1
        self.publish(imu, odometry, 190, OdometrySample(target.left_distance_m, target.right_distance_m, 0, 0))
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.AUTO_IN_ROW_TURN)
        self.now[0] = .2
        self.publish(imu, odometry, 190, OdometrySample(target.left_distance_m, target.right_distance_m, 0, 0))
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.AUTO_SEARCH)
        self.assertEqual(runtime.machine.pass_number, 2)
        self.assertEqual(runtime.heading.reference.reference_deg, 190)
        self.assertIsNone(runtime._turn_controller)
        self.assertEqual(motor.stops.count("turn completed"), 1)
        self.assertEqual(len(motor.commands), 2)
        kinds = [event["kind"] for event in runtime.events.recent()]
        self.assertIn("turn_started", kinds)
        self.assertIn("heading_reference_180", kinds)
        self.assertIn("turn_completed", kinds)

    def test_turn_freezes_visual_row_heading_until_the_single_successful_180_update(self):
        runtime, imu, odometry, _motor = self.make_runtime(confirmations=2)
        runtime._vision = type("Vision", (), {
            "target_x": 1.0, "bud_in_trigger_zone": False,
            "bud_in_pick_zone": False, "marker_found": False,
        })()
        runtime.tick()
        target = runtime._turn_controller.plan
        self.now[0] = .1
        self.publish(imu, odometry, 190, OdometrySample(target.left_distance_m, target.right_distance_m, 0, 0))
        runtime.tick()
        # A fresh visual target and heading must not re-learn the new row
        # heading during the physical turn.
        self.assertEqual(runtime.heading.reference.reference_deg, 10)
        self.now[0] = .2
        self.publish(imu, odometry, 190, OdometrySample(target.left_distance_m, target.right_distance_m, 0, 0))
        runtime.tick()
        self.assertEqual(runtime.heading.reference.reference_deg, 190)

    def test_stale_heading_and_missing_per_wheel_sample_fail_closed_without_drive(self):
        runtime, _imu, odometry, motor = self.make_runtime()
        odometry.latest.publish(0.0, 0.0)  # Legacy float remains valid outside turns, never for a turn.
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.FAULT)
        self.assertEqual(runtime.status().fault, "TURN_ODOMETRY_SAMPLE_MISSING")
        self.assertEqual(motor.commands, [])
        self.assertIn("fault", [event["kind"] for event in runtime.events.recent()])

        runtime, _imu, _odometry, motor = self.make_runtime()
        runtime.tick()
        self.now[0] = .3  # Source remains fresh by RuntimeConfig, but controller age limit expires.
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.FAULT)
        self.assertEqual(runtime.status().fault, "TURN_HEADING_STALE")
        self.assertEqual(len(motor.commands), 1)

    def test_invalid_turn_configuration_fails_before_any_command_or_heading_change(self):
        runtime, _imu, _odometry, motor = self.make_runtime(turn_speed=0)
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.FAULT)
        self.assertEqual(runtime.status().fault, "TURN_CONFIGURATION_INVALID")
        self.assertEqual(motor.commands, [])
        self.assertEqual(runtime.heading.reference.reference_deg, 10)

    def test_manual_stop_clears_controller_and_prevents_stale_turn_resume(self):
        runtime, imu, odometry, motor = self.make_runtime()
        runtime.tick(); commands = len(motor.commands)
        runtime.select_manual()
        self.assertIsNone(runtime._turn_controller)
        self.now[0] = .1; self.publish(imu, odometry, 190, OdometrySample(-2, 2, 0, 0))
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.MANUAL)
        self.assertEqual(len(motor.commands), commands)

    def test_physical_a4_turn_is_nonblocking_and_completes_without_imu_gate(self):
        runtime, _imu, _odometry, _old = self.make_runtime()
        motor = PositionMotor(); runtime.motor = motor
        runtime._lifecycle = _Lifecycle.RUNNING
        sensor = SensorObservation(0, None, None, 10, True, None, None, None, 0,
                                   True, False, True, OdometrySample(0, 0, 0, 0))
        machine = MachineObservation(0, True, False, True, True, False, row_heading_reliable=True)
        runtime._tick_turn_controller(0, sensor, runtime.imu.latest.snapshot(), machine, State.AUTO_IN_ROW_TURN)
        self.assertEqual(len(motor.requests), 1)
        self.assertEqual((motor.requests[0]["left_wheel_degrees"], motor.requests[0]["right_wheel_degrees"]), (-720, 720))
        motor.status = (True, True, None, False)
        runtime._tick_turn_controller(.1, sensor, runtime.imu.latest.snapshot(), machine, State.AUTO_IN_ROW_TURN)
        self.assertEqual(runtime.machine.state, State.AUTO_SEARCH)
        self.assertEqual(runtime.heading.reference.reference_deg, 190)

    def test_physical_a4_error_status_fails_closed(self):
        runtime, _imu, _odometry, _old = self.make_runtime()
        motor = PositionMotor(); runtime.motor = motor
        sensor = SensorObservation(0, None, None, 10, True, None, None, None, 0,
                                   True, False, True, OdometrySample(0, 0, 0, 0))
        machine = MachineObservation(0, True, False, True, True, False, row_heading_reliable=True)
        runtime._tick_turn_controller(0, sensor, runtime.imu.latest.snapshot(), machine, State.AUTO_IN_ROW_TURN)
        motor.status = (True, False, "A4 timeout", False)
        runtime._tick_turn_controller(.1, sensor, runtime.imu.latest.snapshot(), machine, State.AUTO_IN_ROW_TURN)
        self.assertEqual(runtime.machine.state, State.FAULT)
        self.assertEqual(runtime.status().fault, "A4 timeout")

    def test_live_a4_worker_owns_encoder_freshness_until_its_bounded_status_returns(self):
        """A4 polling must not be preempted by the source's stale cache.

        The physical worker is doing fresh 0x92 target confirmation during
        this interval.  Its pending request therefore substitutes for the
        ordinary asynchronous odometry-source deadline; a failed request is
        still observed and faulted by the following turn tick.
        """
        runtime, imu, odometry, _old = self.make_runtime()
        motor = PositionMotor(); runtime.motor = motor
        runtime.lease = ControlLease(runtime.config.control_lease_timeout_s, clock=lambda: self.now[0])
        # Camera and IMU remain healthy, while the source's last 0x92 sample
        # deliberately ages out as it would while the worker owns CAN.
        self.now[0] = 2.0
        runtime._lease_token = runtime.lease.acquire()
        runtime._position_turn_request = object()
        motor.status = (False, False, None, True)
        imu.latest.publish(ImuReading(10, self.now[0]), self.now[0])
        runtime.camera.latest.publish(None, self.now[0])
        runtime.tick()
        self.assertNotEqual(runtime.machine.state, State.FAULT)
        self.assertIsNone(runtime.status().fault)

    def test_marker_decision_persists_until_queued_a4_is_claimed_despite_stale_odometry(self):
        runtime, imu, odometry, _old = self.make_runtime()
        motor = PositionMotor(); runtime.motor = motor
        runtime.lease = ControlLease(runtime.config.control_lease_timeout_s, clock=lambda: self.now[0])
        self.now[0] = 2.0
        runtime._lease_token = runtime.lease.acquire()
        runtime._position_turn_request = object()
        runtime._a4_admission_pending = True
        runtime._position_turn_admission_deadline_s = self.now[0] + .250
        # The worker has not yet claimed its fresh 0x92 baseline.  The
        # marker-derived turn decision nevertheless owns encoder authority
        # during this bounded interval, so the stale source cannot preempt it.
        motor.status = (False, False, None, False)
        imu.latest.publish(ImuReading(10, self.now[0]), self.now[0])
        runtime.camera.latest.publish(None, self.now[0])
        runtime.tick()
        self.assertNotEqual(runtime.machine.state, State.FAULT)
        self.assertIsNone(runtime.status().fault)

    def test_unclaimed_a4_queue_expires_with_stop_and_turn_specific_fault(self):
        runtime, imu, _odometry, _old = self.make_runtime()
        motor = PositionMotor(); runtime.motor = motor
        runtime.lease = ControlLease(runtime.config.control_lease_timeout_s, clock=lambda: self.now[0])
        self.now[0] = 2.0
        runtime._lease_token = runtime.lease.acquire()
        runtime._lifecycle = _Lifecycle.RUNNING
        runtime._position_turn_request = object()
        runtime._position_turn_admission_deadline_s = self.now[0] - .001
        motor.status = (False, False, None, False)
        imu.latest.publish(ImuReading(10, self.now[0]), self.now[0])
        runtime.camera.latest.publish(None, self.now[0])
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.FAULT)
        self.assertEqual(runtime.status().fault, "TURN_A4_ADMISSION_TIMEOUT")
        self.assertIn("TURN_A4_ADMISSION_TIMEOUT", motor.stops)

    def test_confirmed_marker_hands_stale_source_to_bounded_a4_admission_only(self):
        runtime, imu, _odometry, _old = self.make_runtime(state=State.AUTO_ROW_FOLLOW)
        motor = PositionMotor(); runtime.motor = motor
        runtime.lease = ControlLease(runtime.config.control_lease_timeout_s, clock=lambda: self.now[0])
        self.now[0] = 2.0
        runtime._lease_token = runtime.lease.acquire()
        imu.latest.publish(ImuReading(10, self.now[0]), self.now[0])
        runtime.camera.latest.publish(None, self.now[0])
        runtime._vision = type("Vision", (), {
            "target_x": 1.0, "bud_in_trigger_zone": False, "bud_in_pick_zone": False,
            "marker_found": True,
        })()
        # The marker was already debounced on preceding fresh observations;
        # this exact observation triggers the physical turn hand-off.
        runtime.machine._marker_frames = runtime.config.safety.turn_marker_confirm_frames - 1
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.AUTO_IN_ROW_TURN)
        self.assertEqual(len(motor.requests), 1)
        self.assertEqual(motor.a4_transactions, [(-720, 720)])
        self.assertIsNotNone(runtime._position_turn_admission_deadline_s)

    def test_select_auto_stop_preempted_initial_encoder_read_recovers_before_marker_a4_admission(self):
        """Only the typed STOP race retries; a later fresh pair drives A4 admission.

        This models the real startup race: the periodic source has already
        requested its first 0x92 pair when ``select_auto()`` queues STOP.  No
        stale/late pair is accepted.  At the next source period a new pair is
        acquired, after which the normal marker transition admits exactly one
        A4 request.
        """
        class StopPreemptedBackend:
            def __init__(self):
                self.calls = 0
                self.read_started = threading.Event()
                self.stop_requested = threading.Event()
                self.closed = False

            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    self.read_started.set()
                    if not self.stop_requested.wait(.200):
                        raise RuntimeError("test STOP was not queued")
                    raise EncoderReadPreempted("intentional AUTO STOP")
                return 0.0, 0.0

            def close(self):
                self.closed = True

        class StopSignalsPositionMotor(PositionMotor):
            def __init__(self, backend):
                super().__init__()
                self._backend = backend

            def stop_all(self, reason):
                super().stop_all(reason)
                if reason == "AUTO valt":
                    self._backend.stop_requested.set()

        self.now = [0.0]
        safety = SafetyConfig(in_row_turn_enabled=True, new_row_turn_direction="left",
                              turn_distance_tolerance_m=.01, turn_timeout_s=2)
        config = RuntimeConfig(stream_enabled=False, max_rpm=20, turn_speed_rpm=10,
                               heading_filter_alpha=1, camera_timeout_s=1, imu_timeout_s=1,
                               odometry_timeout_s=1, safety=safety)
        camera, imu = Source(), Source()
        camera.latest.publish(None, 0.0)
        imu.latest.publish(ImuReading(10, 0.0), 0.0)
        backend = StopPreemptedBackend()
        odometry = OdometrySource(backend, DriveGeometry())
        motor = StopSignalsPositionMotor(backend)
        runtime = FieldControlRuntime(config, camera, imu, motor=motor, odometry=odometry,
                                      clock=lambda: self.now[0])
        runtime.heading.reference.reference_deg = 10
        runtime.heading.reference.reliable = True
        runtime._lifecycle = _Lifecycle.RUNNING
        odometry.start()
        try:
            self.assertTrue(backend.read_started.wait(.100))
            runtime.select_auto()
            self.assertTrue(odometry.wait_until_ready(.350))
            self.assertEqual(backend.calls, 2)
            self.assertTrue(odometry.snapshot().connected)

            runtime._lease_token = runtime.lease.acquire()
            with runtime._state_lock:
                runtime.machine._transition(State.AUTO_ROW_FOLLOW, "test marker after recovered 0x92")
                runtime.machine._marker_frames = runtime.config.safety.turn_marker_confirm_frames - 1
            runtime._vision = type("Vision", (), {
                "target_x": 1.0, "bud_in_trigger_zone": False, "bud_in_pick_zone": False,
                "marker_found": True,
            })()
            runtime.tick()
            self.assertEqual(runtime.machine.state, State.AUTO_IN_ROW_TURN)
            self.assertEqual(motor.a4_transactions, [(-720, 720)])
            self.assertIsNone(runtime.status().fault)
        finally:
            odometry.stop()
        self.assertTrue(backend.closed)

    def test_select_auto_preemption_holds_stopped_without_premature_odometry_fault(self):
        """The select-AUTO STOP race may not fault before its replacement 0x92.

        This is the physical-HIL scheduling window: the first pair loses to
        STOP and the replacement pair is deliberately held past several
        runtime ticks.  Those ticks must neither enter AUTO nor issue motion
        based on the invalidated cache.
        """
        class StopPreemptedBackend:
            def __init__(self):
                self.calls = 0
                self.read_started = threading.Event()
                self.stop_requested = threading.Event()
                self.replacement_started = threading.Event()
                self.release_replacement = threading.Event()

            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    self.read_started.set()
                    self.stop_requested.wait(.200)
                    raise EncoderReadPreempted("intentional AUTO STOP")
                self.replacement_started.set()
                self.release_replacement.wait(.250)
                return 0.0, 0.0

            def close(self):
                self.release_replacement.set()

        class StopSignalsPositionMotor(PositionMotor):
            def __init__(self, backend):
                super().__init__()
                self._backend = backend

            def stop_all(self, reason):
                super().stop_all(reason)
                if reason == "AUTO valt":
                    self._backend.stop_requested.set()

        self.now = [0.0]
        config = RuntimeConfig(stream_enabled=False, heading_filter_alpha=1,
                               camera_timeout_s=1, imu_timeout_s=1,
                               odometry_timeout_s=1)
        camera, imu = Source(), Source()
        camera.latest.publish(None, 0.0)
        imu.latest.publish(ImuReading(10, 0.0), 0.0)
        backend = StopPreemptedBackend()
        odometry = OdometrySource(backend, DriveGeometry())
        motor = StopSignalsPositionMotor(backend)
        runtime = FieldControlRuntime(config, camera, imu, motor=motor, odometry=odometry,
                                      clock=lambda: self.now[0])
        runtime._lifecycle = _Lifecycle.RUNNING
        odometry.start()
        try:
            self.assertTrue(backend.read_started.wait(.100))
            runtime.select_auto()
            self.assertTrue(backend.replacement_started.wait(.250))
            for now in (.02, .04, .06):
                self.now[0] = now
                runtime.tick()
                self.assertEqual(runtime.machine.state, State.MANUAL)
                self.assertIsNone(runtime.status().fault)
                self.assertEqual(motor.commands, [])
            backend.release_replacement.set()
            self.assertTrue(odometry.wait_until_ready(.150))
            self.now[0] = .12
            runtime.tick()
            self.assertIsNone(runtime.status().fault)
        finally:
            odometry.stop()

    def test_auto_pick_hold_does_not_gate_new_row_a4_admission(self):
        """A one-time AUTO_PICK hold may preempt 0x92 without losing the turn.

        This is the ground-HIL ordering: row following reaches PICK while the
        periodic encoder source already owns a request.  The hold STOP wins,
        then a replacement sample must arrive before the confirmed marker can
        admit its worker-owned new-row A4 transaction.
        """
        class HoldPreemptedBackend:
            def __init__(self):
                self.calls = 0
                self.second_started = threading.Event()
                self.hold_queued = threading.Event()
                self.replacement_started = threading.Event()
                self.release_replacement = threading.Event()

            def angles(self):
                self.calls += 1
                if self.calls == 2:
                    self.second_started.set()
                    if not self.hold_queued.wait(.200):
                        raise RuntimeError("test AUTO_PICK hold was not queued")
                    raise EncoderReadPreempted("intentional AUTO_PICK hold")
                if self.calls == 3:
                    self.replacement_started.set()
                    self.release_replacement.wait(.250)
                return 0.0, 0.0

            def close(self):
                self.release_replacement.set()

        class HoldSignalsPositionMotor(PositionMotor):
            def __init__(self, backend):
                super().__init__()
                self._backend = backend

            def hold_stopped(self, reason, _token=None):
                super().hold_stopped(reason, _token)
                if reason == "state AUTO_PICK":
                    self._backend.hold_queued.set()

        self.now = [0.0]
        safety = SafetyConfig(in_row_turn_enabled=True, new_row_turn_direction="left",
                              turn_distance_tolerance_m=.01, turn_timeout_s=2)
        config = RuntimeConfig(stream_enabled=False, max_rpm=20, turn_speed_rpm=10,
                               heading_filter_alpha=1, camera_timeout_s=1, imu_timeout_s=1,
                               odometry_timeout_s=1, safety=safety)
        camera, imu = Source(), Source()
        camera.latest.publish(None, 0.0)
        imu.latest.publish(ImuReading(10, 0.0), 0.0)
        backend = HoldPreemptedBackend()
        odometry = OdometrySource(backend, DriveGeometry())
        motor = HoldSignalsPositionMotor(backend)
        runtime = FieldControlRuntime(config, camera, imu, motor=motor, odometry=odometry,
                                      clock=lambda: self.now[0])
        runtime.heading.reference.reference_deg = 10
        runtime.heading.reference.reliable = True
        runtime._lifecycle = _Lifecycle.RUNNING
        runtime._lease_token = runtime.lease.acquire()
        with runtime._state_lock:
            runtime.machine._transition(State.AUTO_ROW_FOLLOW, "test AUTO_PICK handoff")
            runtime.machine.pass_number = 2
        odometry.start()
        try:
            self.assertTrue(odometry.wait_until_ready(.150))
            # The next periodic 0x92 is deliberately held in flight.
            self.assertTrue(backend.second_started.wait(.300))
            runtime._vision = type("Vision", (), {
                "target_x": 1.0, "bud_in_trigger_zone": True, "bud_in_pick_zone": True,
                "marker_found": False,
            })()
            runtime.tick()
            self.assertEqual(runtime.machine.state, State.AUTO_PICK)
            self.assertTrue(backend.hold_queued.is_set())
            self.assertFalse(runtime._stationary_hold_odometry_recovery_pending)
            self.assertIsNone(runtime.status().fault)

            for now in (.02, .04, .06):
                self.now[0] = now
                runtime.tick()
                self.assertEqual(runtime.machine.state, State.AUTO_PICK)
                self.assertIsNone(runtime.status().fault)
                self.assertEqual(motor.requests, [])

            self.now[0] = .12
            runtime._vision = type("Vision", (), {
                "target_x": 1.0, "bud_in_trigger_zone": False, "bud_in_pick_zone": False,
                "marker_found": True,
            })()
            with runtime._state_lock:
                runtime.machine._marker_frames = runtime.config.safety.turn_marker_confirm_frames - 1
            runtime.tick()
            self.assertEqual(runtime.machine.state, State.AUTO_NEW_ROW_TURN)
            self.assertEqual(len(motor.requests), 1)
            self.assertIsNone(runtime.status().fault)
        finally:
            odometry.stop()

    def test_manual_stop_preempts_queued_a4_transaction(self):
        runtime, _imu, _odometry, _old = self.make_runtime()
        motor = PositionMotor(); runtime.motor = motor
        runtime._lifecycle = _Lifecycle.RUNNING
        sensor = SensorObservation(0, None, None, 10, True, None, None, None, 0,
                                   True, False, True, OdometrySample(0, 0, 0, 0))
        machine = MachineObservation(0, True, False, True, True, False, row_heading_reliable=True)
        runtime._tick_turn_controller(0, sensor, runtime.imu.latest.snapshot(), machine, State.AUTO_IN_ROW_TURN)
        self.assertEqual(len(motor.a4_transactions), 1)
        runtime.select_manual()
        self.assertEqual(runtime.machine.state, State.MANUAL)
        self.assertIsNone(runtime._position_turn_request)
        self.assertIn("MANUAL vald", motor.stops)

    def test_stale_odometry_before_marker_turn_transition_admits_a4_worker(self):
        runtime, imu, _odometry, _old = self.make_runtime(state=State.AUTO_ROW_FOLLOW)
        motor = PositionMotor(); runtime.motor = motor
        runtime.lease = ControlLease(runtime.config.control_lease_timeout_s, clock=lambda: self.now[0])
        self.now[0] = 2.0
        runtime._lease_token = runtime.lease.acquire()
        imu.latest.publish(ImuReading(10, self.now[0]), self.now[0])
        runtime.camera.latest.publish(None, self.now[0])
        runtime._vision = type("Vision", (), {
            "target_x": 1.0, "bud_in_trigger_zone": False, "bud_in_pick_zone": False,
            "marker_found": True,
        })()
        runtime.machine._marker_frames = runtime.config.safety.turn_marker_confirm_frames - 1
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.AUTO_IN_ROW_TURN)
        self.assertIsNone(runtime.status().fault)
        self.assertEqual(len(motor.requests), 1)

    def test_physical_new_row_a4_is_asymmetric_and_completes_without_imu_gate(self):
        runtime, _imu, _odometry, _old = self.make_runtime(State.AUTO_NEW_ROW_TURN)
        motor = PositionMotor(); runtime.motor = motor
        sensor = SensorObservation(0, None, None, 10, True, None, None, None, 0,
                                   True, False, True, OdometrySample(0, 0, 0, 0))
        machine = MachineObservation(0, True, False, True, True, False, row_heading_reliable=True)
        runtime._tick_turn_controller(0, sensor, runtime.imu.latest.snapshot(), machine, State.AUTO_NEW_ROW_TURN)
        self.assertEqual(len(motor.requests), 1)
        left, right = motor.requests[0]["left_wheel_degrees"], motor.requests[0]["right_wheel_degrees"]
        self.assertGreater(right, left)
        self.assertGreater(left, 0)
        motor.status = (True, True, None, False)
        runtime._tick_turn_controller(.1, sensor, runtime.imu.latest.snapshot(), machine, State.AUTO_NEW_ROW_TURN)
        self.assertEqual(runtime.heading.reference.reference_deg, 190)

if __name__ == "__main__":
    unittest.main()
