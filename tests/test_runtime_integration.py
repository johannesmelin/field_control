import unittest
import threading
import time
import io

from field_control.config import PhysicalCanConfig, RuntimeConfig
from field_control.control import WheelCommand
from field_control.heading import RowHeadingReference
from field_control.observation import (HeadingProcessor, ImuReading, Observation, build_observation,
                                       forward_distance_from_odometry)
from field_control.sources import LatestValue
from field_control.sources import heading_from_imu_quaternion
from field_control.sources import OdometrySource, RightEncoderReplyTimeout
from field_control.odometry import DriveGeometry, OdometrySample
from field_control.turn import new_row_turn_plan
from field_control.lease import ControlLease
from field_control.runtime import FieldControlRuntime, _Lifecycle
from field_control.state_machine import Observation as MachineObservation, State
from field_control.web import DiagnosticsServer
from unittest.mock import patch


class RuntimeIntegrationTests(unittest.TestCase):
    @staticmethod
    def _physical_config(*, odometry_timeout_s=.2):
        return RuntimeConfig(
            stream_enabled=False, max_rpm=10, manual_rpm=10,
            odometry_timeout_s=odometry_timeout_s,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                            "/dev/serial/by-id/test", True, True),
        )

    def test_configuration_restart_predicate_rejects_runtime_lease_or_arming(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False), PassiveSource(), PassiveSource())
        self.assertTrue(runtime.configuration_restart_safe())
        # It is the runtime-held token, not a best-effort lease-validity
        # query, that controls this configuration safety decision.
        with runtime._lifecycle_lock:
            runtime._lease_token = runtime.lease.acquire()
        self.assertFalse(runtime.configuration_restart_safe())
        with runtime._lifecycle_lock:
            runtime._lease_token = None
            runtime._arming_in_progress = True
        self.assertFalse(runtime.configuration_restart_safe())

    def test_row_progress_reset_requires_idle_manual_and_never_claims_output(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False), PassiveSource(), PassiveSource())
        runtime._lifecycle = _Lifecycle.RUNNING
        runtime.machine.row_number = 4; runtime.machine.pass_number = 2
        runtime.reset_row_progress()
        self.assertEqual((runtime.machine.row_number, runtime.machine.pass_number), (1, 1))
        self.assertIsNone(runtime._lease_token)

        runtime.machine.row_number = 3; runtime.machine.pass_number = 2
        runtime._lease_token = runtime.lease.acquire()
        with self.assertRaisesRegex(ValueError, "inaktiv MANUAL"):
            runtime.reset_row_progress()
        self.assertEqual((runtime.machine.row_number, runtime.machine.pass_number), (3, 2))
        runtime._lease_token = None
        runtime.machine._transition(State.AUTO_ROW_FOLLOW, "test AUTO")
        with self.assertRaisesRegex(ValueError, "kräver MANUAL"):
            runtime.reset_row_progress()
        self.assertEqual((runtime.machine.row_number, runtime.machine.pass_number), (3, 2))
        runtime.machine.select_manual()
        with runtime._state_lock:
            runtime._auto_selected = True
        with self.assertRaisesRegex(ValueError, "kräver MANUAL"):
            runtime.reset_row_progress()
        self.assertEqual((runtime.machine.row_number, runtime.machine.pass_number), (3, 2))
        with runtime._state_lock:
            runtime._auto_selected = False
        with runtime._lifecycle_lock:
            runtime._arming_in_progress = True
        with self.assertRaisesRegex(ValueError, "inaktiv MANUAL"):
            runtime.reset_row_progress()
        self.assertEqual((runtime.machine.row_number, runtime.machine.pass_number), (3, 2))

    def test_configuration_restart_reservation_blocks_new_motor_authority(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False), PassiveSource(), PassiveSource())
        self.assertTrue(runtime.reserve_configuration_restart())
        self.assertFalse(runtime.configuration_restart_safe())
        # The reservation is checked before the ordinary state, source, or
        # physical-boundary gates: no concurrent local action can progress
        # from an accepted restart into a new output authority transaction.
        with self.assertRaisesRegex(ValueError, "konfigurationsomstart"):
            runtime.arm_motor_output()
        with self.assertRaisesRegex(ValueError, "konfigurationsomstart"):
            runtime.manual_command(WheelCommand(1.0, 1.0, "test"))
        with self.assertRaisesRegex(ValueError, "konfigurationsomstart"):
            runtime.start_auto()
        runtime.cancel_configuration_restart()
        self.assertTrue(runtime.configuration_restart_safe())

    def test_application_restart_fence_stops_and_rejects_new_manual_auto_or_a2(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Physical:
            def __init__(self): self.armed = True; self.stops = []; self.commands = []
            def arm(self, _token): self.armed = True
            def recoverable_stop_to_web_standby(self, _reason): return False
            def stop_all(self, reason): self.stops.append(reason)
            def command(self, command, _token): self.commands.append(command)

        motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor)
        runtime._lifecycle = _Lifecycle.RUNNING; runtime._lease_token = runtime.lease.acquire()
        runtime.begin_application_restart()

        self.assertTrue(runtime._application_restart_pending)
        self.assertEqual(motor.stops, ["APPLICATION_RESTART"])
        with self.assertRaisesRegex(ValueError, "programomstart väntar"):
            runtime.manual_command(WheelCommand(1, 1, "must not admit"))
        with self.assertRaisesRegex(ValueError, "programomstart väntar"):
            runtime.select_auto()
        with self.assertRaisesRegex(ValueError, "programomstart väntar"):
            runtime.start_auto()
        runtime._admit_command(WheelCommand(1, 1, "must not admit A2"))
        self.assertEqual(motor.commands, [])

    def test_failed_configuration_restart_releases_only_verified_disarmed_fault_boundary(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class LatchedBoundary:
            armed = False
            fault_reason = None
            def arm(self, _token): pass
            def stop_and_settle_for_configuration_restart(self, _reason):
                self.fault_reason = "verified STOP settle failed"
                raise RuntimeError(self.fault_reason)

        class UnknownBoundary:
            armed = True
            fault_reason = None
            def arm(self, _token): pass
            def stop_and_settle_for_configuration_restart(self, _reason):
                raise RuntimeError("unknown output state")

        safe = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=LatchedBoundary())
        self.assertFalse(safe.reserve_configuration_restart())
        self.assertFalse(safe._configuration_restart_pending)
        self.assertEqual(safe.machine.state, State.FAULT)

        unsafe = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=UnknownBoundary())
        self.assertFalse(unsafe.reserve_configuration_restart())
        self.assertTrue(unsafe._configuration_restart_pending)
        self.assertEqual(unsafe.machine.state, State.FAULT)

    def test_configuration_restart_blocks_a2_after_pre_admission_pause(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Physical:
            def __init__(self): self.armed = True; self.commands = []; self.settles = []
            def arm(self, _token): self.armed = True
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, _reason): self.armed = False
            def stop_and_settle_for_restart(self, reason): self.settles.append(reason); self.armed = False

        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=Physical())
        runtime._lifecycle = _Lifecycle.RUNNING
        entered, release = threading.Event(), threading.Event()
        runtime._before_auto_command_admission = lambda: (entered.set(), release.wait(.5))
        command = threading.Thread(target=lambda: runtime._admit_command(WheelCommand(2, 2, "race-a2")), daemon=True)
        command.start(); self.assertTrue(entered.wait(.2))
        self.assertTrue(runtime.reserve_configuration_restart())
        release.set(); command.join(.5)
        self.assertFalse(command.is_alive())
        self.assertEqual(runtime.motor.commands, [])
        self.assertEqual(runtime.motor.settles, ["CONFIGURATION_RESTART"])

    def test_auto_to_manual_blocks_manual_claim_until_encoder_pause_ack(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass
        class Angles:
            def angles(self): return 0.0, 0.0
            def close(self): pass
        class Physical:
            def __init__(self): self.armed = True; self.commands = []; self.stops = []
            def arm(self, _token): pass
            def recoverable_stop_to_web_standby(self, reason): self.stops.append(reason); return True
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        entered, release, errors = threading.Event(), threading.Event(), []
        odometry = OdometrySource(Angles(), DriveGeometry())
        odometry.pause_for_manual = lambda _timeout: (entered.set(), release.wait(.5), True)[2]
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=Physical(), odometry=odometry)
        runtime._lifecycle = _Lifecycle.RUNNING; runtime._lease_token = runtime.lease.acquire()
        selecting = threading.Thread(target=lambda: self._capture(runtime.select_manual, errors), daemon=True)
        selecting.start(); self.assertTrue(entered.wait(.2))
        with self.assertRaisesRegex(ValueError, "encoderläsaren pausas"):
            runtime.manual_command(WheelCommand(1, 1, "must wait"))
        with self.assertRaisesRegex(ValueError, "encoderläsaren pausas"):
            runtime.select_auto()
        with self.assertRaisesRegex(ValueError, "encoderläsaren pausas"):
            runtime.start_auto()
        self.assertEqual(runtime.motor.commands, [])
        release.set(); selecting.join(.5)
        self.assertEqual(errors, [])

    def test_auto_to_manual_pause_timeout_disarms_and_rejects_manual(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass
        class Angles:
            def angles(self): return 0.0, 0.0
            def close(self): pass
        class Physical:
            def __init__(self): self.armed = True
            def arm(self, _token): pass
            def recoverable_stop_to_web_standby(self, _reason): return True
            def stop_all(self, _reason): self.armed = False

        odometry = OdometrySource(Angles(), DriveGeometry()); odometry.pause_for_manual = lambda _timeout: False
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=Physical(), odometry=odometry)
        runtime._lifecycle = _Lifecycle.RUNNING; runtime._lease_token = runtime.lease.acquire()
        with self.assertRaisesRegex(ValueError, "kunde inte pausas"):
            runtime.select_manual()
        self.assertFalse(runtime.motor.armed)
        with self.assertRaises(ValueError):
            runtime.manual_command(WheelCommand(1, 1, "must reject"))

    def test_imu_only_search_freezes_current_heading_separately_from_visual_reference(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False), PassiveSource(), PassiveSource())
        sensor = Observation(0.0, None, 123.0, None, False, None, None, None, 0.0,
                             True, True, True)
        self.assertTrue(runtime._synchronize_navigation_reference(
            State.AUTO_START_DELAY, State.AUTO_SEARCH, sensor))
        self.assertEqual(runtime._active_navigation_reference(sensor), 123.0)
        self.assertFalse(runtime.heading.reference.reliable)
        self.assertIsNone(runtime.heading.reference.reference_deg)

    def test_visual_reacquire_discards_temporary_imu_search_reference(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False), PassiveSource(), PassiveSource())
        sensor = Observation(0.0, None, 123.0, None, False, None, None, None, 0.0,
                             True, True, True)
        self.assertTrue(runtime._synchronize_navigation_reference(
            State.AUTO_START_DELAY, State.AUTO_SEARCH, sensor))
        self.assertTrue(runtime._synchronize_navigation_reference(
            State.AUTO_SEARCH, State.AUTO_ROW_FOLLOW, sensor))
        self.assertIsNone(runtime._active_navigation_reference(sensor))

    def test_temporary_imu_reference_rotates_after_turn_without_visual_reliability(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False), PassiveSource(), PassiveSource())
        sensor = Observation(0.0, None, 270.0, None, False, None, None, None, 0.0,
                             True, True, True)
        self.assertTrue(runtime._synchronize_navigation_reference(
            State.AUTO_SEARCH, State.AUTO_NEW_ROW_TURN, sensor))
        self.assertTrue(runtime._apply_navigation_reference_180_after_turn(1.0))
        self.assertEqual(runtime._active_navigation_reference(sensor), 90.0)
        self.assertFalse(runtime.heading.reference.reliable)

    def test_imu_only_search_without_fresh_filtered_heading_fails_reference_capture(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False), PassiveSource(), PassiveSource())
        sensor = Observation(0.0, None, None, None, False, None, None, None, 0.0,
                             True, True, True)
        self.assertFalse(runtime._synchronize_navigation_reference(
            State.AUTO_START_DELAY, State.AUTO_SEARCH, sensor))

    def test_auto_search_dispatches_heading_command_from_temporary_imu_reference(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class RecordingMotor:
            armed = True
            def __init__(self): self.commands = []
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, _reason): pass
            def hold_stopped(self, _reason, _token=None): pass

        motor = RecordingMotor()
        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False, search_speed_rpm=5.0, max_rpm=10.0),
                                      PassiveSource(), PassiveSource(), motor=motor)
        runtime._lifecycle = _Lifecycle.RUNNING
        sensor = Observation(0.0, None, 85.0, None, False, None, None, None, 0.0,
                             True, True, True)
        self.assertTrue(runtime._synchronize_navigation_reference(
            State.AUTO_START_DELAY, State.AUTO_SEARCH, sensor))
        runtime._dispatch_command(sensor, State.AUTO_SEARCH)
        self.assertEqual(len(motor.commands), 1)
        self.assertGreater(motor.commands[0].left_rpm, 0.0)
        self.assertGreater(motor.commands[0].right_rpm, 0.0)

    def test_stationary_auto_hold_is_idempotent_without_masking_stop_or_motion(self):
        """AUTO_PICK must not repeatedly preempt the shared encoder worker."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Physical:
            def __init__(self):
                self.armed = True
                self.holds = []
                self.stops = []
                self.commands = []
            def arm(self, _token): self.armed = True
            def hold_stopped(self, reason, _token): self.holds.append(reason)
            def stop_all(self, reason):
                self.stops.append(reason)
                self.armed = False
            def command(self, command, _token): self.commands.append(command)

        motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor)
        runtime._lifecycle = _Lifecycle.RUNNING
        stationary = Observation(0.0, None, None, None, False, None, None, None, 0.0,
                                 True, True, True)

        # Repeated 30 Hz AUTO_PICK ticks establish exactly one hold; each
        # additional hold used to preempt a pending shared 0x92 read.
        for _ in range(4):
            runtime._dispatch_command(stationary, State.AUTO_PICK)
        self.assertEqual(motor.holds, ["state AUTO_PICK"])
        self.assertIsNone(runtime.status().last_command)

        # A different stationary AUTO state establishes its own hold.
        runtime._dispatch_command(stationary, State.AUTO_START_DELAY)
        self.assertEqual(motor.holds, ["state AUTO_PICK", "state AUTO_START_DELAY"])

        # An admitted nonzero command clears the marker, so returning to the
        # same stopped state cannot accidentally suppress its safety hold.
        vision = type("Vision", (), {
            "target_x": 10.0,
            "overlay": type("Overlay", (), {"shape": (20, 20)})(),
        })()
        following = Observation(0.0, vision, None, None, False, None, None, None, 0.0,
                                True, True, True)
        runtime._dispatch_command(following, State.AUTO_ROW_FOLLOW)
        self.assertEqual(len(motor.commands), 1)
        self.assertIsNotNone(runtime.status().last_command)
        runtime._dispatch_command(stationary, State.AUTO_PICK)
        self.assertEqual(motor.holds[-1], "state AUTO_PICK")
        self.assertEqual(len(motor.holds), 3)
        self.assertIsNone(runtime.status().last_command)

        # Explicit STOP is never coalesced: it queues immediately, clears the
        # marker and a later AUTO entry issues a new hold.
        runtime.stop()
        self.assertEqual(motor.stops, ["STOP"])
        self.assertIsNone(runtime.status().last_command)
        self.assertFalse(runtime.lease.valid(None))
        runtime._dispatch_command(stationary, State.AUTO_PICK)
        self.assertEqual(len(motor.holds), 4)

    def test_stop_clears_live_command_but_retains_read_only_nonzero_admission_history(self):
        """Historical HIL evidence cannot become an input to motor control."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Physical:
            def __init__(self):
                self.armed = True
                self.commands = []
                self.stops = []
            def arm(self, _token): self.armed = True
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason):
                self.stops.append(reason)
                self.armed = False

        motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor)
        runtime._lifecycle = _Lifecycle.RUNNING
        command = WheelCommand(3.0, -2.0, "test")
        runtime._admit_command(command)
        self.assertEqual(runtime.status().last_command, command)
        self.assertEqual(runtime.status().last_admitted_nonzero_command, command)

        runtime.stop()

        status = runtime.status()
        self.assertIsNone(status.last_command)
        self.assertEqual(status.last_admitted_nonzero_command, command)
        self.assertFalse(status.motor_output_armed)
        self.assertFalse(runtime.lease.valid(None))
        self.assertEqual(motor.stops, ["STOP"])

    def test_physical_arm_allows_cold_unresponsive_encoder_before_drive(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class DelayedAngles:
            def __init__(self): self.read_started = threading.Event(); self.release = threading.Event()
            def angles(self):
                self.read_started.set()
                if not self.release.wait(.5): raise RuntimeError("test encoder remained blocked")
                return 0.0, 0.0
            def close(self): self.release.set()

        class Physical:
            def __init__(self): self.armed = False; self.arm_calls = 0; self.stops = []
            def arm(self, _token): self.arm_calls += 1; self.armed = True
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        backend = DelayedAngles(); source = OdometrySource(backend, DriveGeometry()); motor = Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(),
                                  motor=motor, odometry=source)
        errors = []
        runtime.start()
        try:
            arm = threading.Thread(target=lambda: self._capture(runtime.arm_motor_output, errors))
            arm.start()
            self.assertTrue(backend.read_started.wait(.2))
            arm.join(.4)
            self.assertFalse(arm.is_alive())
            self.assertEqual(errors, [])
            self.assertTrue(motor.armed)
            self.assertEqual(motor.arm_calls, 1)
            backend.release.set()
        finally:
            runtime.close()

    def test_manual_arm_pauses_encoder_before_exposing_lease(self):
        """MANUAL arm exposes no A2 lease until the 0x92 reader is quiescent."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self):
                self.calls = 0
                self.replacement_started = threading.Event()
                self.release_replacement = threading.Event()
            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    return 0.0, 0.0
                self.replacement_started.set()
                self.release_replacement.wait(.5)
                return 0.0, 0.0
            def close(self): self.release_replacement.set()

        class Physical:
            def __init__(self): self.armed = False; self.stops = []
            def arm(self, _token): self.armed = True
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        backend = Angles(); motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=OdometrySource(backend, DriveGeometry()))
        errors = []
        runtime.start()
        try:
            # A prior navigation sample exists, but MANUAL must pause rather
            # than schedule a replacement 0x92 around its verified arm STOP.
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            arm = threading.Thread(target=lambda: self._capture(runtime.arm_motor_output, errors))
            arm.start()
            arm.join(.5)
            self.assertFalse(arm.is_alive())
            self.assertEqual(errors, [])
            self.assertIsNotNone(runtime._lease_token)
            self.assertTrue(runtime._odometry.manual_paused)
            self.assertFalse(backend.replacement_started.is_set())
            self.assertIsNone(runtime.status().fault)
        finally:
            runtime.close()

    def test_manual_arm_does_not_charge_or_request_navigation_odometry(self):
        """A long verified STOP settle does not start a MANUAL 0x92 read.

        MANUAL uses the acknowledged pause rather than the AUTO recovery
        fence, so its STOP settle cannot compete with an encoder request.
        """
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self):
                self.calls = 0
                self.replacement_started = threading.Event()
            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    return 0.0, 0.0
                self.replacement_started.set()
                time.sleep(.2)
                return 0.0, 0.0
            def close(self): pass

        class Physical:
            def __init__(self): self.armed = False; self.settle_started = threading.Event()
            def arm(self, _token):
                self.settle_started.set()
                time.sleep(.35)
                self.armed = True
            def stop_all(self, _reason): self.armed = False

        backend = Angles(); motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(odometry_timeout_s=.3), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=OdometrySource(backend, DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            started = time.monotonic()
            runtime.arm_motor_output()
            elapsed = time.monotonic() - started
            self.assertTrue(motor.settle_started.is_set())
            self.assertFalse(backend.replacement_started.is_set())
            self.assertGreater(elapsed, .3)
            self.assertTrue(motor.armed)
            self.assertIsNotNone(runtime._lease_token)
            self.assertIsNone(runtime.status().fault)
        finally:
            runtime.close()

    def test_manual_arm_accepts_without_post_settle_navigation_sample(self):
        """MANUAL arming requires STOP+0x9C, not a post-stop 0x92 pair."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self):
                self.calls = 0
                self.replacement_started = threading.Event()
                self.release = threading.Event()
            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    return 0.0, 0.0
                self.replacement_started.set()
                self.release.wait(.5)
                return 0.0, 0.0
            def close(self): self.release.set()

        class Physical:
            def __init__(self): self.armed = False
            def arm(self, _token):
                time.sleep(.35)
                self.armed = True
            def stop_all(self, _reason): self.armed = False

        backend = Angles(); motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(odometry_timeout_s=.2), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=OdometrySource(backend, DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime.arm_motor_output()
            self.assertFalse(backend.replacement_started.is_set())
            self.assertIsNone(runtime.status().fault)
            self.assertTrue(motor.armed)
            self.assertIsNotNone(runtime._lease_token)
        finally:
            runtime.close()

    def test_start_auto_does_not_wait_for_post_stop_odometry(self):
        """Start Auto's own hold STOP cannot reuse the preceding 0x92 sample."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self):
                self.calls = 0
                self.replacement_started = threading.Event()
                self.release_replacement = threading.Event()
            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    return 0.0, 0.0
                self.replacement_started.set()
                self.release_replacement.wait(.5)
                return 0.0, 0.0
            def close(self): self.release_replacement.set()

        class Physical:
            def __init__(self): self.armed = True; self.holds = []; self.stops = []; self.commands = []
            def arm(self, _token): self.armed = True
            def hold_stopped(self, reason, _token): self.holds.append(reason)
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        # The required replacement 0x92 can legitimately take longer than
        # the ordinary drive lease.  During this phase the verified STOP hold
        # is already admitted, so Start Auto must retain only that no-motion
        # lease until the bounded encoder recovery completes.
        lease = ControlLease(.05)
        backend, motor = Angles(), Physical()
        runtime = BlockingRuntime(self._physical_config(odometry_timeout_s=.5), PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(backend, DriveGeometry()), lease=lease)
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime._lease_token = runtime.lease.acquire()
            target = type("Vision", (), {
                "target_x": 10.0,
                "bud_in_trigger_zone": False,
                "bud_in_pick_zone": False,
                "marker_found": False,
            })()
            runtime._observation = Observation(0.0, target, None, None, False, 0.0, 0.0, 0.0, 0.0,
                                               True, True, True, OdometrySample(0, 0, 0, 0))
            errors = []
            starter = threading.Thread(target=lambda: self._capture(runtime.start_auto, errors))
            starter.start()
            starter.join(.3)
            self.assertFalse(starter.is_alive(), errors)
            self.assertEqual(runtime.machine.state, State.AUTO_START_DELAY)
            self.assertEqual(motor.holds, ["AUTO startförberedelse"])
            self.assertIsNone(runtime.status().fault)
            self.assertTrue(runtime.lease.valid(runtime._lease_token))
            with self.assertRaisesRegex(ValueError, "manuellt kommando kräver MANUAL"):
                runtime.manual_command(WheelCommand(4.0, 4.0, "racing-manual"))
            self.assertEqual(motor.commands, [])

            self.assertEqual(errors, [])
            self.assertEqual(runtime.machine.state, State.AUTO_START_DELAY)
            self.assertFalse(runtime._auto_start_odometry_recovery_pending)
        finally:
            runtime.close()

    def test_start_auto_reservation_rejects_manual_before_standby_or_stop_fence(self):
        """MANUAL cannot overtake Start Auto's pre-fence transition window."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Physical:
            def __init__(self): self.armed = True; self.commands = []; self.holds = []
            def arm(self, _token): self.armed = True
            def hold_stopped(self, reason, _token): self.holds.append(reason)
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, _reason): self.armed = False

        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=Physical())
        runtime._lifecycle = _Lifecycle.RUNNING
        runtime._lease_token = runtime.lease.acquire()
        target = type("Vision", (), {"target_x": 10.0, "bud_in_trigger_zone": False,
                                      "bud_in_pick_zone": False, "marker_found": False})()
        runtime._observation = Observation(0.0, target, None, None, False, None, None, None, 0.0,
                                           True, True, True)
        entered, release, errors = threading.Event(), threading.Event(), []
        runtime._before_auto_start_transition = lambda: (entered.set(), release.wait(.5))
        starter = threading.Thread(target=lambda: self._capture(runtime.start_auto, errors))
        starter.start()
        self.assertTrue(entered.wait(.2))
        with self.assertRaisesRegex(ValueError, "AUTO-start väntar"):
            runtime.manual_command(WheelCommand(2.0, 2.0, "pre-fence-manual"))
        self.assertEqual(runtime.motor.commands, [])
        release.set()
        starter.join(.5)
        self.assertEqual(errors, [])
        self.assertEqual(runtime.machine.state, State.AUTO_START_DELAY)

    def test_auto_to_manual_cancels_pre_reserved_start_before_standby_claim(self):
        """A cancelled Start Auto cannot consume MANUAL's new web standby."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def angles(self): return 0.0, 0.0
            def close(self): pass

        class Physical:
            def __init__(self):
                self.armed = True; self.commands = []; self.claims = 0; self.recoveries = []
            def arm(self, _token): self.armed = True
            def recoverable_stop_to_web_standby(self, reason):
                self.recoveries.append(reason); return True
            def claim_web_standby(self, _token): self.claims += 1
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, _reason): self.armed = False

        odometry = OdometrySource(Angles(), DriveGeometry())
        pause_entered, release_pause = threading.Event(), threading.Event()
        odometry.pause_for_manual = lambda _timeout: (pause_entered.set(), release_pause.wait(.5), True)[2]
        motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=odometry)
        runtime._lifecycle = _Lifecycle.RUNNING
        runtime._lease_token = runtime.lease.acquire()
        target = type("Vision", (), {"target_x": 10.0, "bud_in_trigger_zone": False,
                                      "bud_in_pick_zone": False, "marker_found": False})()
        observation = Observation(0.0, target, None, None, False, None, None, None, 0.0,
                                  True, True, True)
        runtime._observation = observation
        # Establish active AUTO without invoking the unrelated sensor-gate
        # conversion; this regression exercises only the lifecycle race.
        runtime.machine._transition(State.AUTO_ROW_FOLLOW, "test AUTO")

        start_entered, release_start, start_errors, manual_errors = (
            threading.Event(), threading.Event(), [], [])
        runtime._before_auto_start_transition = lambda: (start_entered.set(), release_start.wait(.5))
        starter = threading.Thread(target=lambda: self._capture(runtime.start_auto, start_errors), daemon=True)
        starter.start(); self.assertTrue(start_entered.wait(.2))
        manual = threading.Thread(target=lambda: self._capture(runtime.select_manual, manual_errors), daemon=True)
        manual.start(); self.assertTrue(pause_entered.wait(.2))

        release_start.set(); starter.join(.5)
        self.assertFalse(starter.is_alive())
        self.assertEqual(len(start_errors), 1)
        self.assertIsInstance(start_errors[0], ValueError)
        self.assertEqual(motor.claims, 0)
        self.assertEqual(motor.commands, [])
        self.assertIsNone(runtime._lease_token)

        release_pause.set(); manual.join(.5)
        self.assertFalse(manual.is_alive())
        self.assertEqual(manual_errors, [])

    def test_stop_keeps_manual_behind_cancel_and_stop_admission(self):
        """A MANUAL command cannot enter after STOP cancels pending AUTO first."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Physical:
            def __init__(self):
                self.armed = True; self.commands = []; self.stop_entered = threading.Event(); self.release_stop = threading.Event()
            def arm(self, _token): self.armed = True
            def hold_stopped(self, _reason, _token): pass
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, _reason):
                self.stop_entered.set()
                self.release_stop.wait(.5)
                self.armed = False

        motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor)
        runtime._lifecycle = _Lifecycle.RUNNING
        runtime._lease_token = runtime.lease.acquire()
        target = type("Vision", (), {"target_x": 10.0, "bud_in_trigger_zone": False,
                                      "bud_in_pick_zone": False, "marker_found": False})()
        runtime._observation = Observation(0.0, target, None, None, False, None, None, None, 0.0,
                                           True, True, True)
        start_entered, release_start, start_errors = threading.Event(), threading.Event(), []
        runtime._before_auto_start_transition = lambda: (start_entered.set(), release_start.wait(.5))
        starter = threading.Thread(target=lambda: self._capture(runtime.start_auto, start_errors))
        starter.start()
        self.assertTrue(start_entered.wait(.2))
        stop_errors, manual_errors = [], []
        stopper = threading.Thread(target=lambda: self._capture(runtime.stop, stop_errors))
        stopper.start()
        self.assertTrue(motor.stop_entered.wait(.2))
        manual = threading.Thread(target=lambda: self._capture(
            lambda: runtime.manual_command(WheelCommand(2.0, 2.0, "racing-manual")), manual_errors))
        manual.start()
        time.sleep(.03)
        self.assertTrue(manual.is_alive())
        motor.release_stop.set()
        stopper.join(.5); manual.join(.5)
        release_start.set(); starter.join(.5)
        self.assertEqual(stop_errors, [])
        self.assertEqual(motor.commands, [])
        self.assertEqual(len(manual_errors), 1)
        self.assertIsInstance(manual_errors[0], ValueError)
        self.assertEqual(len(start_errors), 1)
        self.assertIsInstance(start_errors[0], ValueError)

    def test_locally_armed_physical_web_auto_then_start_preserves_arm_through_stop_barriers(self):
        """Browser mode/start actions never arm, but may use a local arm.

        Both mode selection and Start Auto queue a stopped physical hold and
        require a replacement encoder sample.  The former must not turn a
        locally acquired lease into browser-controlled arming.
        """
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def angles(self): return 0.0, 0.0
            def close(self): pass

        class Physical:
            def __init__(self): self.armed = False; self.holds = []; self.stops = []; self.arm_calls = 0
            def arm(self, _token): self.arm_calls += 1; self.armed = True
            def hold_stopped(self, reason, _token): self.holds.append(reason)
            def stop_all(self, reason): self.stops.append(reason); self.armed = False

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        def post(path: str) -> int:
            with patch("field_control.web.status_payload", return_value={"ok": True}):
                server = object.__new__(DiagnosticsServer); server.runtime = runtime
                handler = object.__new__(server._handler())
                result: list[int] = []; handler.path = path; handler.wfile = io.BytesIO()
                handler.send_response = lambda status, *_args: result.append(status)
                handler.send_header = lambda *_args: None; handler.end_headers = lambda: None
                handler.send_error = lambda status, *_args: result.append(status)
                handler.do_POST()
                return result[0]

        motor = Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(Angles(), DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime.arm_motor_output()
            local_token = runtime._lease_token
            self.assertTrue(motor.armed)
            self.assertIsNotNone(local_token)
            target = type("Vision", (), {
                "target_x": 10.0, "bud_in_trigger_zone": False,
                "bud_in_pick_zone": False, "marker_found": False,
            })()
            runtime._observation = Observation(0.0, target, None, None, False, 0.0, 0.0, 0.0, 0.0,
                                               True, True, True, OdometrySample(0, 0, 0, 0))

            self.assertEqual(post("/api/auto"), 200)
            self.assertTrue(motor.armed)
            self.assertEqual(runtime._lease_token, local_token)
            self.assertEqual(motor.holds, ["AUTO valt"])
            self.assertEqual(motor.stops, [])
            # The select-AUTO hold invalidates the preceding shared sample.
            # The next web action is only admitted after its replacement.
            self.assertTrue(runtime._odometry.wait_until_ready(.2))

            self.assertEqual(post("/api/start-auto"), 200)
            self.assertEqual(runtime.machine.state, State.AUTO_START_DELAY)
            self.assertTrue(motor.armed)
            self.assertEqual(runtime._lease_token, local_token)
            self.assertEqual(motor.holds, ["AUTO valt", "AUTO startförberedelse"])
            self.assertEqual(motor.stops, [])
            # There is intentionally no HTTP route that could acquire arm.
            self.assertEqual(post("/api/arm-motor-output"), 404)
            self.assertEqual(motor.arm_calls, 1)
        finally:
            runtime.close()

    def test_web_standby_has_no_drive_lease_until_start_auto_claims_it(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def angles(self): return 0.0, 0.0
            def close(self): pass

        class Physical:
            def __init__(self):
                self.armed = False; self.standby = False; self.holds = []; self.stops = []
                self.claim_calls = 0; self.commands = []
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token): self.standby = True
            def claim_web_standby(self, _token):
                if not self.standby: raise RuntimeError("standby saknas")
                self.claim_calls += 1
                self.standby = False
            def hold_stopped(self, reason, _token): self.holds.append(reason)
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.standby = False; self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        lease = ControlLease(.05)
        config = RuntimeConfig(**{
            **self._physical_config().__dict__,
            "physical_web_standby_timeout_s": .45,
        })
        motor = Physical()
        runtime = BlockingRuntime(config, PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(Angles(), DriveGeometry()), lease=lease)
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime.arm_motor_output_for_web_standby()
            self.assertTrue(motor.standby)
            self.assertIsNone(runtime._lease_token)
            self.assertEqual(motor.holds, [])
            runtime.select_auto()
            # Mode selection retains no-motion standby.  In particular it
            # must remain safe beyond an ordinary drive lease timeout, since
            # no drive authority or motor transaction was admitted.
            time.sleep(.12)
            self.assertTrue(motor.standby)
            self.assertTrue(motor.armed)
            self.assertIsNone(runtime._lease_token)
            self.assertEqual(motor.holds, [])
            self.assertEqual(motor.stops, [])
            self.assertEqual(runtime.status().mode, "AUTO")
            with self.assertRaisesRegex(ValueError, "AUTO har valts"):
                runtime.manual_command(WheelCommand(4.0, 4.0, "manual-must-not-claim"))
            self.assertTrue(motor.standby)
            self.assertTrue(motor.armed)
            self.assertIsNone(runtime._lease_token)
            self.assertEqual(motor.claim_calls, 0)
            self.assertEqual(motor.commands, [])
            target = type("Vision", (), {
                "target_x": 10.0, "bud_in_trigger_zone": False,
                "bud_in_pick_zone": False, "marker_found": False,
            })()
            runtime._observation = Observation(0.0, target, None, None, False, 0.0, 0.0, 0.0, 0.0,
                                               True, True, True, OdometrySample(0, 0, 0, 0))
            runtime.start_auto()
            self.assertEqual(runtime.machine.state, State.AUTO_START_DELAY)
            self.assertFalse(motor.standby)
            self.assertIsNotNone(runtime._lease_token)
            self.assertEqual(motor.claim_calls, 1)
            self.assertEqual(motor.holds, ["AUTO startförberedelse"])
            self.assertEqual(motor.stops, [])
        finally:
            runtime.close()

    def test_manual_claim_linearizes_before_auto_selection_stopped_handoff(self):
        """A manual command already admitted at the lifecycle gate wins once."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def angles(self): return 0.0, 0.0
            def close(self): pass

        class Physical:
            def __init__(self):
                self.armed = False; self.standby = False; self.claim_calls = 0
                self.commands = []; self.holds = []; self.stops = []
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token): self.standby = True
            def claim_web_standby(self, _token):
                if not self.standby: raise RuntimeError("standby saknas")
                self.claim_calls += 1; self.standby = False
            def command(self, command, _token): self.commands.append(command)
            def hold_stopped(self, reason, _token): self.holds.append(reason)
            def stop_all(self, reason): self.armed = False; self.standby = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        entered, release, errors = threading.Event(), threading.Event(), []
        motor = Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(Angles(), DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime.arm_motor_output_for_web_standby()
            runtime._before_manual_standby_claim = lambda: (entered.set(), release.wait(.5))
            manual = threading.Thread(
                target=lambda: self._capture(
                    lambda: runtime.manual_command(WheelCommand(4.0, 4.0, "manual-first")), errors),
                daemon=True,
            )
            manual.start()
            self.assertTrue(entered.wait(.2))
            selecting = threading.Thread(target=lambda: self._capture(runtime.select_auto, errors), daemon=True)
            selecting.start()
            self.assertTrue(selecting.is_alive())
            release.set()
            manual.join(.5); selecting.join(.5)
            self.assertFalse(manual.is_alive())
            self.assertFalse(selecting.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual([(command.left_rpm, command.right_rpm) for command in motor.commands], [(4.0, 4.0)])
            self.assertEqual(motor.claim_calls, 1)
            # Selection observed that MANUAL had already claimed authority,
            # so it follows the existing stopped local-arm handoff.
            self.assertEqual(motor.holds, ["AUTO valt"])
            self.assertEqual(motor.stops, [])
            self.assertEqual(runtime.status().mode, "AUTO")
        finally:
            runtime._before_manual_standby_claim = None
            release.set()
            runtime.close()

    def test_paused_manual_source_stops_reads_and_accepts_held_command(self):
        """A web MANUAL lease uses A2 while its 0x92 producer is quiescent."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self): self.calls = 0
            def angles(self): self.calls += 1; return 0.0, 0.0
            def close(self): pass

        class Physical:
            def __init__(self): self.armed = False; self.standby = False; self.commands = []; self.stops = []
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token): self.standby = True
            def claim_web_standby(self, _token): self.standby = False
            def hold_stopped(self, _reason, _token): pass
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.armed = False; self.standby = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        backend, motor = Angles(), Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(backend, DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime.arm_motor_output_for_web_standby()
            paused_calls = backend.calls
            time.sleep(.15)
            self.assertEqual(backend.calls, paused_calls)
            runtime.manual_command(WheelCommand(4, 4, "held-web"))
            runtime.manual_command(WheelCommand(0, 0, "web-manual-hold"))
            self.assertEqual([(item.left_rpm, item.right_rpm) for item in motor.commands], [(4, 4), (0, 0)])
            self.assertTrue(motor.armed)
            self.assertTrue(runtime.lease.valid(runtime._lease_token))
            self.assertTrue(runtime._odometry.manual_paused)
            runtime.stop()
            self.assertEqual(motor.stops[-1], "STOP")
        finally:
            runtime.close()

    def test_start_auto_resumes_paused_source_after_stopped_hold(self):
        """Selecting AUTO preserves standby; Start Auto owns the stopped handoff."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self): self.calls = 0; self.second = threading.Event(); self.release = threading.Event()
            def angles(self):
                self.calls += 1
                if self.calls > 1:
                    self.second.set(); self.release.wait(.5)
                return 0.0, 0.0
            def close(self): self.release.set()

        class Physical:
            def __init__(self): self.armed = False; self.standby = False; self.holds = []; self.stops = []
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token): self.standby = True
            def claim_web_standby(self, _token): self.standby = False
            def hold_stopped(self, reason, _token): self.holds.append(reason)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        backend, motor = Angles(), Physical()
        runtime = BlockingRuntime(self._physical_config(odometry_timeout_s=.05), PassiveSource(), PassiveSource(),
                                  motor=motor, odometry=OdometrySource(backend, DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime.arm_motor_output_for_web_standby()
            runtime.select_auto()
            self.assertTrue(runtime._odometry.manual_paused)
            self.assertTrue(motor.standby)
            self.assertEqual(motor.holds, [])
            target = type("Vision", (), {
                "target_x": 10.0, "bud_in_trigger_zone": False,
                "bud_in_pick_zone": False, "marker_found": False,
            })()
            runtime._observation = Observation(0.0, target, None, None, False, 0.0, 0.0, 0.0, 0.0,
                                               True, True, True, OdometrySample(0, 0, 0, 0))
            runtime.start_auto()
            self.assertTrue(backend.second.wait(.2))
            self.assertFalse(runtime._odometry.manual_paused)
            self.assertEqual(motor.holds, ["AUTO startförberedelse"])
            self.assertEqual(motor.stops, [])
        finally:
            backend.release.set()
            runtime.close()

    def test_failed_auto_hold_never_reopens_paused_encoder_source(self):
        """A failed AUTO STOP leaves MANUAL's 0x92 pause acknowledged."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self): self.calls = 0; self.second_started = threading.Event()
            def angles(self):
                self.calls += 1
                if self.calls > 1: self.second_started.set()
                return 0.0, 0.0
            def close(self): pass

        class Physical:
            def __init__(self): self.armed = True; self.holds = 0; self.stops = []
            def arm(self, _token): self.armed = True
            def hold_stopped(self, _reason, _token):
                self.holds += 1
                raise RuntimeError("injected hold admission failure")
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        backend, motor = Angles(), Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(backend, DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            self.assertTrue(runtime._odometry.pause_for_manual(.2))
            runtime._lease_token = runtime.lease.acquire()
            with self.assertRaisesRegex(RuntimeError, "injected hold"):
                runtime.select_auto()
            time.sleep(.15)
            self.assertTrue(runtime._odometry.manual_paused)
            self.assertFalse(backend.second_started.is_set())
            self.assertEqual(runtime.machine.state, State.FAULT)
            self.assertTrue(motor.stops)
        finally:
            runtime.close()

    def test_web_standby_handoff_gates_expired_drive_lease_until_no_motion_exchange(self):
        """A watchdog waking during the handoff cannot preempt the 0x92 reader.

        The simulated watchdog is deliberately invoked from
        ``enter_web_standby`` after the ordinary arm lease has expired.  It
        models the narrow race between ``arm_motor_output`` returning and the
        standby exchange; no motor command is ever admitted in that interval.
        """
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        now = [0.0]
        lease = ControlLease(.05, clock=lambda: now[0])

        class Physical:
            control_lease = lease
            def __init__(self): self.armed = False; self.standby = False; self.stops = []
            @property
            def fault_reason(self): return None
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token):
                # The independent watchdog is now due, but it must remain
                # gated until this no-motion exchange has completed.
                now[0] = .10
                runtime._watchdog_revoke_if_running("CONTROL_LEASE_EXPIRED")
                self.standby = True
            def claim_web_standby(self, _token): self.standby = False
            def stop_all(self, reason):
                self.armed = False; self.standby = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        motor = Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                  clock=lambda: now[0], lease=lease)
        runtime.start()
        try:
            runtime.arm_motor_output_for_web_standby()
            # Also model a watchdog which observed non-standby just before
            # publication, then won the lifecycle lock afterwards.
            runtime._watchdog_revoke_if_running("CONTROL_LEASE_EXPIRED")
            self.assertTrue(motor.armed)
            self.assertTrue(motor.standby)
            self.assertEqual(motor.stops, [])
            self.assertIsNone(runtime._lease_token)
            self.assertTrue(runtime.web_standby_status()[0])
            self.assertIsNone(runtime.status().fault)
        finally:
            runtime.close()

    def test_web_standby_handoff_failure_stops_and_clears_arming_gate(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Physical:
            def __init__(self): self.armed = False; self.stops = []
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token): raise RuntimeError("standby exchange failed")
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        motor = Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor)
        runtime.start()
        try:
            with self.assertRaisesRegex(RuntimeError, "standby exchange failed"):
                runtime.arm_motor_output_for_web_standby()
            self.assertFalse(motor.armed)
            self.assertEqual(motor.stops, ["fysisk webbstandby misslyckades"])
            self.assertFalse(runtime._arming_in_progress)
            self.assertIsNone(runtime._lease_token)
            self.assertFalse(runtime.web_standby_status()[0])
        finally:
            runtime.close()

    def test_web_standby_timeout_fails_closed_and_disarms(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Physical:
            def __init__(self): self.armed = True; self.stops = []
            def arm(self, _token): pass
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        now = [10.0]
        motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                      clock=lambda: now[0])
        runtime._lifecycle = _Lifecycle.RUNNING
        runtime._web_standby_deadline_s = 10.0
        self.assertTrue(runtime._expire_web_standby_if_due(now[0]))
        self.assertFalse(motor.armed)
        self.assertEqual(motor.stops, ["WEB_STANDBY_TIMEOUT"])
        self.assertEqual(runtime.status().fault, "WEB_STANDBY_TIMEOUT")
        self.assertFalse(runtime.web_standby_status()[0])

    def test_threaded_watchdog_keeps_valid_no_motion_web_standby_alive_until_its_deadline(self):
        """Standby is not a stalled control loop or an expired drive lease."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def angles(self): return 0.0, 0.0
            def close(self): pass

        lease = ControlLease(.08)
        class Physical:
            control_lease = lease
            def __init__(self): self.armed = False; self.standby = False; self.stops = []
            @property
            def fault_reason(self): return None
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, token):
                self.assert_token = token; self.standby = lease.release(token)
            def claim_web_standby(self, _token): self.standby = False
            def stop_all(self, reason): self.armed = False; self.standby = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        config = self._physical_config(odometry_timeout_s=.3)
        config = RuntimeConfig(**{**config.__dict__, "physical_web_standby_timeout_s": .45})
        motor = Physical()
        runtime = BlockingRuntime(config, PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(Angles(), DriveGeometry()), lease=lease)
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime.arm_motor_output_for_web_standby()
            self.assertTrue(runtime.web_standby_status()[0])
            # This exceeds both the normal 80 ms drive lease and the 120 ms
            # control-stall bound across several watchdog periods.
            time.sleep(.20)
            self.assertTrue(runtime.web_standby_status()[0])
            self.assertTrue(motor.armed)
            self.assertEqual(motor.stops, [])
            self.assertIsNone(runtime.status().fault)
            # Explicit STOP remains dominant while standby is alive.
            runtime.stop()
            self.assertFalse(runtime.web_standby_status()[0])
            self.assertFalse(motor.armed)
            self.assertEqual(motor.stops, ["STOP"])
        finally:
            runtime.close()

    def test_stop_cancels_start_auto_without_post_stop_odometry_wait(self):
        """A delayed Start Auto caller cannot revive AUTO after operator STOP."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self):
                self.calls = 0
                self.replacement_started = threading.Event()
                self.release_replacement = threading.Event()
            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    return 0.0, 0.0
                self.replacement_started.set()
                self.release_replacement.wait(.5)
                return 0.0, 0.0
            def close(self): self.release_replacement.set()

        class Physical:
            def __init__(self): self.armed = True; self.holds = []; self.stops = []
            def arm(self, _token): self.armed = True
            def hold_stopped(self, reason, _token): self.holds.append(reason)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        backend, motor = Angles(), Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(backend, DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(runtime._odometry.wait_until_ready(.2))
            runtime._lease_token = runtime.lease.acquire()
            target = type("Vision", (), {
                "target_x": 10.0,
                "bud_in_trigger_zone": False,
                "bud_in_pick_zone": False,
                "marker_found": False,
            })()
            runtime._observation = Observation(0.0, target, None, None, False, 0.0, 0.0, 0.0, 0.0,
                                               True, True, True, OdometrySample(0, 0, 0, 0))
            errors = []
            starter = threading.Thread(target=lambda: self._capture(runtime.start_auto, errors))
            starter.start()
            runtime.stop()
            self.assertEqual(runtime.machine.state, State.MANUAL)
            starter.join(.5)

            self.assertFalse(starter.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(runtime.machine.state, State.MANUAL)
            # Later control ticks must not resume the cancelled request.
            runtime.tick()
            self.assertEqual(runtime.machine.state, State.MANUAL)
        finally:
            runtime.close()

    def test_physical_unarmed_manual_ticks_do_not_queue_stop_before_encoder_ready(self):
        """An unarmed physical MANUAL tick leaves the shared encoder worker alone."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Angles:
            def __init__(self): self.read_started = threading.Event()
            def angles(self):
                self.read_started.set()
                return 0.0, 0.0
            def close(self): pass

        class Physical:
            def __init__(self): self.armed = False; self.stops = []
            def arm(self, _token): self.armed = True
            def stop_all(self, reason): self.stops.append(reason)

        backend = Angles(); motor = Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=OdometrySource(backend, DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(backend.read_started.wait(.2))
            # Let a MANUAL control tick run after the first 0x92-equivalent
            # source read. It must not admit a redundant physical STOP.
            time.sleep(.12)
            self.assertEqual(motor.stops, [])
            self.assertFalse(motor.armed)
            runtime.stop()
            self.assertEqual(motor.stops, ["STOP"])
        finally:
            runtime.close()

    def test_physical_arm_allows_first_odometry_sample_timeout(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class BlockedAngles:
            def __init__(self): self.read_started = threading.Event(); self.release = threading.Event()
            def angles(self):
                self.read_started.set(); self.release.wait(.5)
                return 0.0, 0.0
            def close(self): self.release.set()

        class Physical:
            def __init__(self): self.armed = False; self.arm_calls = 0; self.stops = []
            def arm(self, _token): self.arm_calls += 1; self.armed = True
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        backend = BlockedAngles(); motor = Physical()
        runtime = BlockingRuntime(self._physical_config(odometry_timeout_s=.05), PassiveSource(), PassiveSource(),
                                  motor=motor, odometry=OdometrySource(backend, DriveGeometry()))
        runtime.start()
        try:
            self.assertTrue(backend.read_started.wait(.2))
            runtime.arm_motor_output()
            self.assertTrue(motor.armed)
            self.assertEqual(motor.arm_calls, 1)
            self.assertIsNone(runtime.status().fault)
        finally:
            runtime.close()

    def test_manual_arm_allows_malformed_encoder_sample(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class MalformedAngles:
            def angles(self): return "not-a-number", 0.0
            def close(self): pass

        class Physical:
            def __init__(self): self.armed = False; self.arm_calls = 0; self.stops = []
            def arm(self, _token): self.arm_calls += 1; self.armed = True
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        motor = Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(MalformedAngles(), DriveGeometry()))
        runtime.start()
        try:
            runtime.arm_motor_output()
            self.assertTrue(motor.armed)
            self.assertEqual(motor.arm_calls, 1)
            self.assertIsNone(runtime.status().fault)
        finally:
            runtime.close()

    def test_close_cancels_physical_odometry_readiness_wait_without_arming(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class BlockedAngles:
            def __init__(self): self.read_started = threading.Event(); self.release = threading.Event()
            def angles(self):
                self.read_started.set(); self.release.wait(.5)
                return 0.0, 0.0
            def close(self): self.release.set()

        class Physical:
            def __init__(self): self.armed = False; self.arm_calls = 0; self.stops = []
            def arm(self, _token): self.arm_calls += 1; self.armed = True
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        backend = BlockedAngles(); motor = Physical()
        runtime = BlockingRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=OdometrySource(backend, DriveGeometry()))
        errors = []
        runtime.start()
        arm = threading.Thread(target=lambda: self._capture(runtime.arm_motor_output, errors))
        arm.start()
        self.assertTrue(backend.read_started.wait(.2))
        closer = threading.Thread(target=runtime.close)
        closer.start(); closer.join(.5); arm.join(.5)
        self.assertFalse(closer.is_alive())
        self.assertFalse(arm.is_alive())
        self.assertFalse(motor.armed)
        self.assertEqual(motor.arm_calls, 0)
        self.assertEqual(len(errors), 1)

    def test_physical_close_signals_shared_odometry_before_can_close_claim(self):
        """The source retry barrier is linearized before CAN shutdown owns 0x92."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Backend:
            def __init__(self): self.shutdown = threading.Event()
            def angles(self): return 0.0, 0.0
            def begin_shutdown(self): self.shutdown.set()
            def close(self): self.shutdown.set()

        class Physical:
            def __init__(self, backend):
                self.backend = backend; self.armed = False; self.claim_saw_shutdown = False; self.closed = False
            def arm(self, _token): self.armed = True
            def stop_all(self, _reason): self.armed = False
            def _begin_close(self):
                self.claim_saw_shutdown = self.backend.shutdown.is_set()
                return True
            def _finish_close(self): self.closed = True
            def close(self): self.closed = True

        backend = Backend(); motor = Physical(backend)
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(), motor=motor,
                                      odometry=OdometrySource(backend, DriveGeometry()))
        runtime.close()

        self.assertTrue(motor.claim_saw_shutdown)
        self.assertTrue(motor.closed)

    @staticmethod
    def _capture(operation, errors):
        try:
            operation()
        except Exception as exc:
            errors.append(exc)

    def test_missing_encoder_pair_allows_auto_and_uses_bounded_distance(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class RightTimeoutAngles:
            def angles(self):
                raise RightEncoderReplyTimeout(
                    "0x92 fick giltigt svar från 0x141 men timeout från 0x142"
                )
            def close(self): pass

        class Physical:
            def __init__(self):
                self.armed = False; self.standby = False; self.stops = []; self.commands = []
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token): self.standby = True
            def claim_web_standby(self, _token): self.standby = False
            def hold_stopped(self, _reason, _token): pass
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason):
                self.armed = False; self.standby = False; self.stops.append(reason)

        config = RuntimeConfig(
            stream_enabled=False, max_rpm=10, manual_rpm=10,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                            "/dev/serial/by-id/test", True, True),
        )
        motor = Physical(); odometry = OdometrySource(RightTimeoutAngles(), DriveGeometry())
        runtime = FieldControlRuntime(config, PassiveSource(), PassiveSource(), motor=motor, odometry=odometry)
        runtime.start()
        try:
            deadline = time.monotonic() + .2
            while not odometry.right_encoder_timeout_after_left_reply and time.monotonic() < deadline:
                time.sleep(.002)
            self.assertTrue(odometry.right_encoder_timeout_after_left_reply)
            # The verified boundary's arm operation is a STOP+0x9C settle.
            runtime.arm_motor_output_for_web_standby()
            self.assertTrue(motor.armed)
            self.assertTrue(motor.standby)
            self.assertIsNone(runtime.status().fault)
            runtime.manual_command(WheelCommand(3.0, 3.0, "degraded-manual"))
            self.assertEqual([(command.left_rpm, command.right_rpm) for command in motor.commands], [(3.0, 3.0)])

            # The exact 0x141 reply / 0x142 timeout keeps the stopped handoff
            # and lease protocol, but may enter ordinary AUTO navigation with
            # a conservative command-integrated distance bound.
            runtime.select_auto()
            self.assertEqual([(command.left_rpm, command.right_rpm) for command in motor.commands], [(3.0, 3.0)])
            self.assertTrue(motor.armed)
            self.assertTrue(runtime._degraded_auto_odometry_active())
            runtime._observation = Observation(
                0.0, None, 0.0, None, False, None, None, None, 0.0,
                True, True, True,
            )
            runtime.start_auto()
            self.assertEqual([(command.left_rpm, command.right_rpm) for command in motor.commands], [(3.0, 3.0)])
            self.assertTrue(motor.armed)
            self.assertEqual(runtime.machine.state, State.AUTO_START_DELAY)
            runtime._reset_degraded_auto_distance(0.0)
            runtime._last_command = WheelCommand(1.0, 1.0, "heading")
            self.assertGreater(runtime._degraded_auto_distance(1.0), 0.0)

            # Without a physical A4 worker/baseline, no encoder angle is
            # invented by the fallback path.
            runtime.machine.state = State.AUTO_IN_ROW_TURN
            result = runtime._tick_turn_controller(
                0.1, runtime._observation, LatestValue().snapshot(),
                MachineObservation(0.1, True, True, True, True, False),
                State.AUTO_IN_ROW_TURN,
            )
            self.assertEqual(result.fault, "TURN_ODOMETRY_SAMPLE_MISSING")
            self.assertIn("TURN_ODOMETRY_SAMPLE_MISSING", motor.stops)
        finally:
            runtime.close()

    def test_right_timeout_is_classified_before_manual_pause_can_arm(self):
        """A concurrent MANUAL arm observes the authorized 0x142 case atomically."""
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class BlockingRightTimeout:
            def __init__(self):
                self.read_started = threading.Event()
                self.release = threading.Event()
            def angles(self):
                self.read_started.set()
                self.release.wait(.5)
                raise RightEncoderReplyTimeout(
                    "0x92 fick giltigt svar från 0x141 men timeout från 0x142"
                )
            def close(self): self.release.set()

        class Physical:
            def __init__(self):
                self.armed = False; self.standby = False; self.commands = []; self.stops = []
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token): self.standby = True
            def claim_web_standby(self, _token): self.standby = False
            def hold_stopped(self, _reason, _token): pass
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason):
                self.armed = False; self.standby = False; self.stops.append(reason)

        backend, motor = BlockingRightTimeout(), Physical()
        odometry = OdometrySource(backend, DriveGeometry())
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=odometry)
        runtime.start()
        errors = []
        try:
            self.assertTrue(backend.read_started.wait(.2))
            armer = threading.Thread(
                target=lambda: self._capture(runtime.arm_motor_output_for_web_standby, errors)
            )
            armer.start()
            self.assertTrue(armer.is_alive())
            backend.release.set()
            armer.join(.5)
            self.assertFalse(armer.is_alive())
            self.assertEqual(errors, [])
            self.assertTrue(odometry.right_encoder_timeout_after_left_reply)
            self.assertTrue(motor.armed)
            self.assertTrue(motor.standby)
            self.assertEqual(motor.commands, [])
        finally:
            runtime.close()

    def test_manual_arm_allows_delayed_left_or_generic_encoder_error(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class BlockingLeftTimeout:
            def __init__(self):
                self.read_started = threading.Event()
                self.release = threading.Event()
            def angles(self):
                self.read_started.set()
                self.release.wait(.5)
                raise RuntimeError("timeout från 0x141")
            def close(self): self.release.set()

        class Physical:
            def __init__(self): self.armed = False; self.arm_calls = 0; self.stops = []
            def arm(self, _token): self.arm_calls += 1; self.armed = True
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        backend, motor = BlockingLeftTimeout(), Physical()
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=OdometrySource(backend, DriveGeometry()))
        errors = []
        runtime.start()
        try:
            self.assertTrue(backend.read_started.wait(.2))
            armer = threading.Thread(target=lambda: self._capture(runtime.arm_motor_output, errors))
            armer.start()
            self.assertTrue(armer.is_alive())
            backend.release.set()
            armer.join(.5)
            self.assertFalse(armer.is_alive())
            self.assertEqual(errors, [])
            self.assertTrue(motor.armed)
            self.assertEqual(motor.arm_calls, 1)
        finally:
            runtime.close()

    def test_known_right_timeout_auto_selection_preserves_web_standby(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class RecoveringAngles:
            def __init__(self):
                self.timeout_seen = threading.Event(); self.recover = threading.Event()
            def angles(self):
                if not self.recover.is_set():
                    self.timeout_seen.set()
                    raise RightEncoderReplyTimeout(
                        "0x92 fick giltigt svar från 0x141 men timeout från 0x142"
                    )
                return 0.0, 0.0
            def close(self): self.recover.set()

        class Physical:
            def __init__(self):
                self.armed = False; self.standby = False; self.commands = []; self.stops = []
            def arm(self, _token): self.armed = True
            def enter_web_standby(self, _token): self.standby = True
            def claim_web_standby(self, _token): self.standby = False
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason):
                self.armed = False; self.standby = False; self.stops.append(reason)

        backend, motor = RecoveringAngles(), Physical()
        odometry = OdometrySource(backend, DriveGeometry())
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=odometry)
        runtime.start()
        try:
            self.assertTrue(backend.timeout_seen.wait(.2))
            runtime.arm_motor_output_for_web_standby()
            runtime.select_auto()
            self.assertEqual(motor.commands, [])
            # AUTO selection remains the no-motion web-standby state even
            # while the right encoder is temporarily unavailable.
            self.assertTrue(motor.armed)
            self.assertTrue(motor.standby)
            self.assertEqual(motor.stops, [])

        finally:
            runtime.close()

    def test_later_generic_encoder_error_does_not_block_manual_arm(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class FailingAfterRightTimeout:
            def __init__(self):
                self.right_timeout_seen = threading.Event(); self.generic_failure = threading.Event()
            def angles(self):
                if not self.generic_failure.is_set():
                    self.right_timeout_seen.set()
                    raise RightEncoderReplyTimeout(
                        "0x92 fick giltigt svar från 0x141 men timeout från 0x142"
                    )
                raise RuntimeError("left 0x141 timeout")
            def close(self): self.generic_failure.set()

        class Physical:
            def __init__(self): self.armed = False; self.arm_calls = 0; self.commands = []; self.stops = []
            def arm(self, _token): self.arm_calls += 1; self.armed = True
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        backend, motor = FailingAfterRightTimeout(), Physical()
        odometry = OdometrySource(backend, DriveGeometry())
        runtime = FieldControlRuntime(self._physical_config(), PassiveSource(), PassiveSource(),
                                      motor=motor, odometry=odometry)
        runtime.start()
        try:
            self.assertTrue(backend.right_timeout_seen.wait(.2))
            deadline = time.monotonic() + .2
            while not odometry.right_encoder_timeout_after_left_reply and time.monotonic() < deadline:
                time.sleep(.002)
            self.assertTrue(odometry.right_encoder_timeout_after_left_reply)
            backend.generic_failure.set()
            deadline = time.monotonic() + .3
            while odometry.right_encoder_timeout_after_left_reply and time.monotonic() < deadline:
                time.sleep(.002)
            self.assertFalse(odometry.right_encoder_timeout_after_left_reply)

            runtime.arm_motor_output()
            self.assertTrue(motor.armed)
            self.assertEqual(motor.arm_calls, 1)
            self.assertEqual(motor.commands, [])
        finally:
            runtime.close()

    def test_physical_manual_rejects_fresh_numeric_odometry_after_arming(self):
        now = [1.0]

        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Odometry(PassiveSource):
            def start(self): self.latest.publish(OdometrySample(0, 0, 0, 0), now[0])
            def snapshot(self): return self.latest.snapshot()

        class Physical:
            def __init__(self): self.armed = False; self.stops = []; self.commands = []
            def arm(self, _token): self.armed = True
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        config = RuntimeConfig(
            stream_enabled=False, max_rpm=10, manual_rpm=10,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                            "/dev/serial/by-id/test", True, True),
        )
        odometry, motor = Odometry(), Physical()
        runtime = BlockingRuntime(config, PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=odometry, clock=lambda: now[0])
        runtime.lease.set_revoke_callback(lambda: motor.stop_all("lease revoke"))
        runtime.start(); runtime.arm_motor_output()
        try:
            now[0] = 1.1
            odometry.latest.publish(1.25, now[0])  # Fresh legacy value is forbidden on physical output.
            command = WheelCommand(1.0, 1.0, "manual")
            runtime.manual_command(command)
            self.assertEqual(motor.commands, [command])
            self.assertIsNone(runtime.status().fault)
        finally:
            runtime.close()

    def test_physical_auto_rejects_fresh_numeric_odometry_before_dispatch(self):
        now = [1.0]

        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Odometry(PassiveSource):
            def start(self): self.latest.publish(OdometrySample(0, 0, 0, 0), now[0])
            def snapshot(self): return self.latest.snapshot()

        class Physical:
            def __init__(self): self.armed = False; self.stops = []; self.commands = []
            def arm(self, _token): self.armed = True
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        config = RuntimeConfig(
            stream_enabled=False, max_rpm=10, auto_base_rpm=5, vision_kp=1,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                            "/dev/serial/by-id/test", True, True),
        )
        odometry, motor = Odometry(), Physical()
        runtime = BlockingRuntime(config, PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=odometry, clock=lambda: now[0])
        runtime.start(); runtime.arm_motor_output()
        try:
            now[0] = 1.1
            odometry.latest.publish(1.25, now[0])
            with runtime._state_lock:
                runtime.machine._transition(State.AUTO_ROW_FOLLOW, "test")
            runtime.tick()
            self.assertEqual(motor.commands, [])
            # Camera and IMU remain independent safety inputs; numeric
            # physical odometry itself is no longer an output gate.
            self.assertEqual(runtime.status().fault, "CAMERA_TIMEOUT/IMU_TIMEOUT")
        finally:
            runtime.close()

    def test_valid_physical_odometry_does_not_own_manual_deadline(self):
        now = [1.0]

        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class Odometry(PassiveSource):
            def start(self): self.latest.publish(OdometrySample(0, 0, 0, 0), now[0])
            def snapshot(self): return self.latest.snapshot()

        class Physical:
            def __init__(self): self.armed = False; self.stops = []; self.commands = []
            def arm(self, _token): self.armed = True
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        class BlockingRuntime(FieldControlRuntime):
            def _run(self): self._stop.wait()

        config = RuntimeConfig(
            stream_enabled=False, max_rpm=10, manual_rpm=10, odometry_timeout_s=.1,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                            "/dev/serial/by-id/test", True, True),
        )
        odometry, motor = Odometry(), Physical()
        runtime = BlockingRuntime(config, PassiveSource(), PassiveSource(), motor=motor,
                                  odometry=odometry, clock=lambda: now[0])
        runtime.lease.set_revoke_callback(lambda: motor.stop_all("lease revoke"))
        runtime.start(); runtime.arm_motor_output()
        try:
            command = WheelCommand(1.0, 1.0, "manual")
            runtime.manual_command(command)
            self.assertEqual(motor.commands, [command])
            now[0] = 1.101
            runtime._watchdog_revoke_if_running("CONTROL_LEASE_EXPIRED")
            self.assertIsNone(runtime.status().fault)
        finally:
            runtime.close()

    def test_runtime_owns_optional_odometry_source_lifecycle(self):
        class PassiveSource:
            def __init__(self):
                self.latest = LatestValue(); self.started = 0; self.stopped = 0
            def start(self): self.started += 1
            def stop(self): self.stopped += 1

        class Backend:
            def __init__(self): self.closed = 0
            def close(self): self.closed += 1

        class ControlledOdometry(PassiveSource):
            def __init__(self, backend, fail=False):
                super().__init__(); self.backend, self.fail = backend, fail
            def start(self):
                super().start()
                self.latest.publish(OdometrySample(1, 3, 2, 45), time.monotonic())
                if self.fail: raise RuntimeError("encoder start failure")
            def snapshot(self): return self.latest.snapshot()
            def stop(self):
                super().stop(); self.backend.close()

        camera, imu = PassiveSource(), PassiveSource()
        backend = Backend(); odometry = ControlledOdometry(backend)
        runtime = FieldControlRuntime(RuntimeConfig(), camera, imu, odometry=odometry)
        runtime.start()
        snapshot = odometry.snapshot()
        self.assertIsInstance(snapshot.value, OdometrySample)
        self.assertIsNotNone(snapshot.updated_at_s)
        self.assertLess(snapshot.age_s(time.monotonic()), RuntimeConfig().odometry_timeout_s)
        runtime.close(); runtime.close()
        self.assertEqual((odometry.started, odometry.stopped, backend.closed), (1, 1, 1))

        camera, imu = PassiveSource(), PassiveSource()
        failed_backend = Backend(); failing = ControlledOdometry(failed_backend, fail=True)
        failed_runtime = FieldControlRuntime(RuntimeConfig(), camera, imu, odometry=failing)
        with self.assertRaisesRegex(RuntimeError, "encoder start failure"):
            failed_runtime.start()
        self.assertEqual((camera.stopped, imu.stopped, failing.stopped, failed_backend.closed), (1, 1, 1, 1))
        failed_runtime.close()
        self.assertEqual(failing.stopped, 1)

    def test_new_row_turn_plan_uses_track_and_row_spacing(self):
        plan = new_row_turn_plan(DriveGeometry(wheel_track_m=1.0), 2.0, 12.0, "right")
        self.assertAlmostEqual(plan.left_distance_m, 1.5 * 3.141592653589793)
        self.assertAlmostEqual(plan.right_distance_m, .5 * 3.141592653589793)
        self.assertEqual(plan.speed_rpm, 12.0)

    def test_odometry_source_publishes_immutable_per_wheel_sample(self):
        class Angles:
            value = (0.0, 0.0)
            def angles(self): return self.value
            def close(self): pass

        backend = Angles(); source = OdometrySource(backend, DriveGeometry())
        initial = source._read_sample()
        self.assertEqual(initial, OdometrySample(0.0, 0.0, 0.0, 0.0))
        backend.value = (360.0, -360.0)
        sample = source._read_sample()
        self.assertAlmostEqual(sample.left_distance_m, DriveGeometry().left_wheel_circumference_m / 8.0)
        self.assertAlmostEqual(sample.right_distance_m, DriveGeometry().right_wheel_circumference_m / 8.0)
        self.assertAlmostEqual(sample.forward_distance_m, DriveGeometry().left_wheel_circumference_m / 8.0)
        with self.assertRaises(AttributeError): sample.left_distance_m = 0.0
        source.latest.publish(sample, 3.0)
        snapshot = source.snapshot()
        self.assertIs(snapshot.value, sample)
        self.assertEqual(snapshot.updated_at_s, 3.0)
        source.stop()

    def test_odometry_source_rate_limits_shared_encoder_sampling(self):
        class Angles:
            def __init__(self): self.calls = 0
            def angles(self): self.calls += 1; return (0.0, 0.0)
            def close(self): pass

        backend = Angles(); source = OdometrySource(backend, DriveGeometry())
        source.start()
        try:
            # 10 Hz: immediate first sample then at most two more over 250 ms.
            time.sleep(.250)
            self.assertGreaterEqual(backend.calls, 2)
            self.assertLessEqual(backend.calls, 3)
        finally:
            source.stop()

    def test_observation_accepts_sample_and_legacy_numeric_odometry(self):
        camera = LatestValue(); camera.publish(object(), 1.0)
        imu = LatestValue(); imu.publish(ImuReading(10.0, 1.0), 1.0)
        heading = HeadingProcessor(.5, RowHeadingReference(2.0, 1.0))
        sample = OdometrySample(1.0, 3.0, 2.0, 45.0)
        samples = LatestValue(); samples.publish(sample, 1.0)
        observation = build_observation(1.0, camera.snapshot(), imu.snapshot(), samples.snapshot(),
                                        None, heading, 1.0, 1.0, 1.0)
        self.assertIs(observation.odometry_sample, sample)
        self.assertEqual(observation.distance_m, sample.forward_distance_m)
        self.assertEqual(forward_distance_from_odometry(2.5), 2.5)

    def test_sample_odometry_stale_and_failed_states_remain_fail_closed(self):
        camera = LatestValue(); camera.publish(object(), 1.0)
        imu = LatestValue(); imu.publish(ImuReading(10.0, 1.0), 1.0)
        heading = HeadingProcessor(.5, RowHeadingReference(2.0, 1.0))
        odometry = LatestValue(); odometry.publish(OdometrySample(1, 1, 1, 0), 1.0)
        stale = build_observation(2.0, camera.snapshot(), imu.snapshot(), odometry.snapshot(),
                                  None, heading, 2.0, 2.0, .5)
        self.assertFalse(stale.odometry_fresh)
        self.assertEqual(stale.fault, "ODOMETRY_TIMEOUT")
        odometry.fail("encoder lost")
        failed = build_observation(1.1, camera.snapshot(), imu.snapshot(), odometry.snapshot(),
                                   None, heading, 2.0, 2.0, 2.0)
        self.assertFalse(failed.odometry_fresh)
        self.assertEqual(failed.fault, "ODOMETRY_TIMEOUT")

    def test_control_lease_expires_using_injected_monotonic_clock(self):
        now = [10.0]
        lease = ControlLease(.5, lambda: now[0])
        token = lease.acquire()
        self.assertTrue(lease.valid(token))
        now[0] = 10.5
        self.assertFalse(lease.valid(token))
        self.assertFalse(lease.run_if_valid(token, lambda: self.fail("expired lease ran operation")))

    def test_control_lease_refresh_and_revoke_callback_are_fail_closed(self):
        now = [1.0]; revoked = []
        lease = ControlLease(1.0, lambda: now[0]); lease.set_revoke_callback(lambda: revoked.append(True))
        token = lease.acquire(); now[0] = 1.5; lease.refresh(token)
        self.assertTrue(lease.valid(token)); self.assertTrue(lease.revoke_any())
        self.assertEqual(revoked, [True]); self.assertFalse(lease.valid(token))

    def test_latest_value_replaces_old_data_and_keeps_timestamp(self):
        latest = LatestValue()
        latest.publish("old", 10.0)
        latest.publish("new", 12.0)
        snapshot = latest.snapshot()
        self.assertEqual(snapshot.value, "new")
        self.assertEqual(snapshot.updated_at_s, 12.0)
        self.assertEqual(snapshot.age_s(13.5), 1.5)

    def test_latest_value_rejects_non_monotonic_timestamps(self):
        latest = LatestValue()
        latest.publish("value", 10.0)
        with self.assertRaises(ValueError):
            latest.publish("older", 9.0)

    def test_observation_marks_camera_stale_without_using_old_vision(self):
        camera = LatestValue(); camera.publish(object(), 1.0)
        imu = LatestValue(); imu.publish(ImuReading(10.0, 2.0), 2.0)
        odometry = LatestValue(); odometry.publish(3.0, 2.0)
        heading = HeadingProcessor(.5, RowHeadingReference(2.0, 1.0))
        heading.update(ImuReading(10.0, 2.0), visual_following=False, distance_m=3.0)
        observation = build_observation(
            3.0, camera.snapshot(), imu.snapshot(), odometry.snapshot(),
            object(), heading, .5, 2.0, 2.0,
        )
        self.assertFalse(observation.camera_fresh)
        self.assertIsNone(observation.vision)
        self.assertEqual(observation.fault, "CAMERA_TIMEOUT")

    def test_backward_odometry_in_auto_pick_does_not_turn_valid_imu_into_timeout(self):
        """A stationary marker/pick state does not collect row-heading samples."""
        class Source:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass
            def snapshot(self): return self.latest.snapshot()

        now = [1.0]
        camera, imu, odometry = Source(), Source(), Source()
        camera.latest.publish(object(), now[0])
        imu.latest.publish(ImuReading(10.0, now[0]), now[0])
        odometry.latest.publish(1.0, now[0])
        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False), camera, imu,
                                      odometry=odometry, clock=lambda: now[0])
        runtime._vision = type("Vision", (), {
            "target_x": 160.0,
            "bud_in_trigger_zone": False,
            "bud_in_pick_zone": False,
            "marker_found": False,
            "overlay": type("Overlay", (), {"shape": (240, 320)})(),
        })()
        runtime._last_frame_timestamp = now[0]
        runtime.machine.state = State.AUTO_PICK

        runtime.tick()
        now[0] = 1.1
        imu.latest.publish(ImuReading(20.0, now[0]), now[0])
        odometry.latest.publish(.99, now[0])
        status = runtime.tick()

        self.assertTrue(status.observation.imu_fresh)
        self.assertIsNone(status.observation.imu_error)
        self.assertNotEqual(status.snapshot.fault, "IMU_TIMEOUT")
        self.assertIsNone(status.fault)
        self.assertIsNone(runtime.heading.reference.reference_deg)
        self.assertEqual(runtime.heading.reference.reliable_distance_m, 0.0)

    def test_backward_odometry_in_auto_row_follow_remains_strict_reference_failure(self):
        """Unexpected reverse progress is not silently hidden while following a row."""
        class Source:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass
            def snapshot(self): return self.latest.snapshot()

        class Motor:
            armed = False
            def __init__(self): self.stops = []
            def stop_all(self, reason): self.stops.append(reason)

        now = [1.0]
        camera, imu, odometry = Source(), Source(), Source()
        camera.latest.publish(object(), now[0])
        imu.latest.publish(ImuReading(10.0, now[0]), now[0])
        odometry.latest.publish(1.0, now[0])
        motor = Motor()
        runtime = FieldControlRuntime(RuntimeConfig(stream_enabled=False, max_rpm=10, auto_base_rpm=1), camera, imu,
                                      motor=motor, odometry=odometry, clock=lambda: now[0])
        runtime._vision = type("Vision", (), {
            "target_x": 160.0,
            "bud_in_trigger_zone": False,
            "bud_in_pick_zone": False,
            "marker_found": False,
            "overlay": type("Overlay", (), {"shape": (240, 320)})(),
        })()
        runtime._last_frame_timestamp = now[0]
        runtime.machine.state = State.AUTO_ROW_FOLLOW

        runtime.tick()
        self.assertEqual(runtime.heading.reference.reference_deg, 10.0)
        self.assertEqual(runtime.machine.state, State.AUTO_ROW_FOLLOW, runtime.machine.reason)
        now[0] = 1.1
        imu.latest.publish(ImuReading(20.0, now[0]), now[0])
        odometry.latest.publish(.99, now[0])
        status = runtime.tick()

        self.assertTrue(status.observation.imu_fresh)
        self.assertIsNone(status.observation.imu_error)
        self.assertEqual(status.observation.fault, "ROW_REFERENCE_ODOMETRY_NONMONOTONIC")
        self.assertEqual(status.snapshot.fault, "ROW_REFERENCE_ODOMETRY_NONMONOTONIC")
        self.assertEqual(status.fault, "ROW_REFERENCE_ODOMETRY_NONMONOTONIC")
        self.assertEqual(motor.stops, ["ROW_REFERENCE_ODOMETRY_NONMONOTONIC"])

    def test_actual_imu_source_failure_remains_imu_timeout(self):
        camera = LatestValue(); camera.publish(object(), 1.0)
        imu = LatestValue(); imu.fail("BNO086 lost")
        odometry = LatestValue(); odometry.publish(1.0, 1.0)
        observation = build_observation(
            1.1, camera.snapshot(), imu.snapshot(), odometry.snapshot(), None,
            HeadingProcessor(.5, RowHeadingReference(2.0, 1.0)), 1.0, 1.0, 1.0,
        )
        self.assertFalse(observation.imu_fresh)
        self.assertEqual(observation.fault, "IMU_TIMEOUT")

    def test_runtime_configuration_rejects_nonpositive_sensor_timeout(self):
        with self.assertRaises(ValueError):
            RuntimeConfig(camera_timeout_s=0).validate()

    def test_runtime_defaults_to_320_by_240_for_processing_and_streaming(self):
        config = RuntimeConfig()
        self.assertEqual((config.processing_width, config.processing_height), (320, 240))
        self.assertEqual((config.stream_width, config.stream_height), (320, 240))

    def test_tilt_compensated_quaternion_heading_is_circular(self):
        self.assertAlmostEqual(heading_from_imu_quaternion((0, 0.70710678, 0, 0.70710678)), 0.0, places=4)
        calibrated = ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(heading_from_imu_quaternion((0, 0, 0, 1), calibrated), 0.0, places=4)
        with self.assertRaises(ValueError):
            heading_from_imu_quaternion((0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
