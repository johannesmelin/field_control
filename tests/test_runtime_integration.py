import unittest
import threading
import time

from field_control.config import PhysicalCanConfig, RuntimeConfig
from field_control.control import WheelCommand
from field_control.heading import RowHeadingReference
from field_control.observation import (HeadingProcessor, ImuReading, build_observation,
                                       forward_distance_from_odometry)
from field_control.sources import LatestValue
from field_control.sources import heading_from_imu_quaternion
from field_control.sources import OdometrySource
from field_control.odometry import DriveGeometry, OdometrySample
from field_control.turn import new_row_turn_plan
from field_control.lease import ControlLease
from field_control.runtime import FieldControlRuntime
from field_control.state_machine import State


class RuntimeIntegrationTests(unittest.TestCase):
    @staticmethod
    def _physical_config(*, odometry_timeout_s=.2):
        return RuntimeConfig(
            stream_enabled=False, max_rpm=10, manual_rpm=10,
            odometry_timeout_s=odometry_timeout_s,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                            "/dev/serial/by-id/test", True, True),
        )

    def test_physical_arm_waits_for_delayed_first_odometry_sample(self):
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
            self.assertFalse(motor.armed)
            self.assertEqual(motor.arm_calls, 0)
            backend.release.set()
            arm.join(.5)
            self.assertFalse(arm.is_alive())
            self.assertEqual(errors, [])
            self.assertTrue(motor.armed)
            self.assertEqual(motor.arm_calls, 1)
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

    def test_physical_arm_fails_closed_when_first_odometry_sample_times_out(self):
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
            with self.assertRaisesRegex(ValueError, "odometri blev inte redo"):
                runtime.arm_motor_output()
            self.assertFalse(motor.armed)
            self.assertEqual(motor.arm_calls, 0)
            self.assertEqual(runtime.status().fault, "ODOMETRY_TIMEOUT")
            self.assertTrue(motor.stops)
        finally:
            runtime.close()

    def test_physical_arm_rejects_malformed_first_odometry_sample(self):
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
            with self.assertRaisesRegex(ValueError, "odometri blev inte redo"):
                runtime.arm_motor_output()
            self.assertFalse(motor.armed)
            self.assertEqual(motor.arm_calls, 0)
            self.assertEqual(runtime.status().fault, "ODOMETRY_TIMEOUT")
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

    def test_failed_shared_physical_odometry_rejects_arm_before_drive(self):
        class PassiveSource:
            def __init__(self): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class FailedOdometry(PassiveSource):
            def snapshot(self): return self.latest.snapshot()
            def start(self): self.latest.fail("0x92 timeout")

        class Physical:
            def __init__(self): self.armed = False; self.stops = []; self.commands = []
            def arm(self, _token): self.armed = True
            def command(self, command, _token): self.commands.append(command)
            def stop_all(self, reason): self.armed = False; self.stops.append(reason)

        config = RuntimeConfig(
            stream_enabled=False, max_rpm=10, manual_rpm=10,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                            "/dev/serial/by-id/test", True, True),
        )
        motor = Physical(); runtime = FieldControlRuntime(
            config, PassiveSource(), PassiveSource(), motor=motor, odometry=FailedOdometry(),
        )
        runtime.start()
        try:
            with self.assertRaisesRegex(ValueError, "odometri"):
                runtime.arm_motor_output()
            self.assertEqual(motor.commands, [])
            self.assertTrue(motor.stops)
            self.assertEqual(runtime.status().fault, "ODOMETRY_TIMEOUT")
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
            with self.assertRaisesRegex(ValueError, "odometri"):
                runtime.manual_command(WheelCommand(1.0, 1.0, "manual"))
            self.assertEqual(motor.commands, [])
            self.assertTrue(motor.stops)
            self.assertEqual(runtime.status().fault, "ODOMETRY_TIMEOUT")
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
            self.assertTrue(motor.stops)
            self.assertEqual(runtime.status().fault, "ODOMETRY_TIMEOUT")
        finally:
            runtime.close()

    def test_valid_physical_odometry_allows_manual_then_owns_deadline(self):
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
            self.assertEqual(runtime.status().fault, "ODOMETRY_TIMEOUT")
            self.assertTrue(motor.stops)
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
