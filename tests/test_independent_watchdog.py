import threading
import time
import unittest

from field_control.config import PhysicalCanConfig, RuntimeConfig
from field_control.lease import ControlLease
from field_control.runtime import CONTROL_LEASE_EXPIRED, FieldControlRuntime
from field_control.sources import LatestValue
from field_control.state_machine import State
from field_control.verified_motor_boundary import _VerifiedPhysicalMotorBoundary
from field_control.motor_boundary import MotorOutputFault
from field_control.control import WheelCommand
from field_control.observation import Observation as SensorObservation
from field_control.odometry import OdometrySample


class Source:
    def __init__(self): self.latest = LatestValue()
    def start(self): pass
    def stop(self): pass


class Sink:
    def __init__(self):
        self.stops = []; self.commands = []; self.callback = None; self.closed = False
        self.drive_admitted = threading.Event()
        self.stop_queued = threading.Event()
        self.events = []; self.fail_settle = None
    def set_fault_callback(self, callback): self.callback = callback
    def command(self, left, right, reason):
        self.commands.append((left, right, reason)); self.drive_admitted.set()
    def stop_all(self, reason): self.stops.append(reason); self.stop_queued.set()
    def stop_and_settle_for_restart(self):
        self.events.append("settle")
        if self.fail_settle is not None: raise self.fail_settle
    def stop_and_settle_and_close(self):
        self.events.append("shutdown-settle")
        try:
            if self.fail_settle is not None: raise self.fail_settle
        finally:
            self.close()
    def close(self): self.closed = True; self.events.append("close")


class BlockingRuntime(FieldControlRuntime):
    def _run(self):
        # Deliberately no tick/heartbeat: the independent watchdog must act.
        self._stop.wait()


class IndependentWatchdogTests(unittest.TestCase):
    def make_runtime(self, *, lease_timeout=.3):
        now = [0.0]
        config = RuntimeConfig(
            stream_enabled=False, max_rpm=20, auto_base_rpm=5, vision_kp=1, control_lease_timeout_s=lease_timeout,
            watchdog_period_s=.01, max_control_stall_s=.12,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id", "/dev/serial/by-id/test", True, True),
        )
        lease = ControlLease(lease_timeout, clock=lambda: now[0]); sink = Sink()
        motor = _VerifiedPhysicalMotorBoundary(sink, lease, max_rpm=20)
        runtime = BlockingRuntime(config, Source(), Source(), motor=motor, lease=lease, clock=lambda: now[0])
        return runtime, sink, now

    def test_arm_requires_running_control_and_independent_watchdog(self):
        runtime, _sink, _now = self.make_runtime()
        with self.assertRaises(ValueError): runtime.arm_motor_output()
        runtime.close()

    def test_stall_queues_stop_then_retains_armed_manual_standby(self):
        runtime, sink, now = self.make_runtime()
        runtime.start(); runtime.arm_motor_output()
        now[0] = .13
        deadline = time.monotonic() + .5
        while runtime.machine.state is not State.FAULT and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertEqual(runtime.machine.state, State.MANUAL)
        self.assertEqual(runtime.status().fault, "CONTROL_LOOP_STALL")
        self.assertTrue(sink.stops)
        self.assertTrue(runtime.motor.armed)
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.MANUAL)
        runtime.select_manual()
        runtime.manual_command(WheelCommand(1, 1, "after watchdog"))
        self.assertEqual(sink.commands[-1], (1, 1, "after watchdog"))
        runtime.close()

    def test_close_gate_rejects_an_arm_that_finishes_after_closing_begins(self):
        runtime, sink, _now = self.make_runtime()
        entered = threading.Event(); release = threading.Event()
        def settle():
            entered.set(); release.wait(.5)
        sink.stop_and_settle_for_restart = settle
        runtime.start()
        errors = []
        arm_thread = threading.Thread(target=lambda: self._capture(runtime.arm_motor_output, errors))
        arm_thread.start(); self.assertTrue(entered.wait(.5))
        close_thread = threading.Thread(target=lambda: self._capture(runtime.close, errors))
        close_thread.start(); time.sleep(.02); release.set()
        arm_thread.join(.5); close_thread.join(.5)
        self.assertFalse(runtime.motor.armed)
        self.assertEqual(sink.commands, [])
        self.assertTrue(errors)
        with self.assertRaises(ValueError):
            runtime.manual_command(WheelCommand(1, 1, "after close"))
        with self.assertRaises(ValueError):
            runtime.start_auto()

    def test_close_linearization_blocks_paused_auto_dispatch_admission(self):
        runtime, sink, _now = self.make_runtime()
        token_seen = threading.Event(); release_dispatch = threading.Event()
        close_entered = threading.Event(); release_close = threading.Event()
        runtime._before_auto_command_admission = lambda: (token_seen.set(), release_dispatch.wait(.5))
        runtime._after_closing_before_revoke = lambda: (close_entered.set(), release_close.wait(.5))
        runtime.start(); runtime.arm_motor_output()
        vision = type("Vision", (), {"target_x": 10.0, "overlay": type("Overlay", (), {"shape": (20, 20)})()})()
        observation = SensorObservation(0, vision, None, None, False, 0, 0, 0, 0,
                                        True, True, True)
        dispatch = threading.Thread(target=lambda: runtime._dispatch_command(observation, State.AUTO_ROW_FOLLOW))
        dispatch.start(); self.assertTrue(token_seen.wait(.5))
        closer = threading.Thread(target=runtime.close); closer.start()
        self.assertTrue(close_entered.wait(.5))
        self.assertEqual(runtime._lifecycle.value, "CLOSING")
        self.assertTrue(runtime.lease.valid(runtime._lease_token))
        release_dispatch.set()
        # Dispatch has the token but remains blocked on lifecycle admission.
        self.assertFalse(sink.drive_admitted.wait(.05))
        release_close.set(); dispatch.join(.5); closer.join(.5)
        self.assertEqual(sink.commands, [])
        self.assertEqual(sink.stops, [])

    def test_verified_runtime_close_has_single_settle_then_close_without_stop_queue(self):
        runtime, sink, _now = self.make_runtime()
        runtime.start(); runtime.arm_motor_output()
        sink.events.clear()

        runtime.close()

        self.assertEqual(sink.events, ["shutdown-settle", "close"])
        self.assertEqual(sink.stops, [])
        self.assertEqual(sink.commands, [])
        self.assertFalse(runtime._thread and runtime._thread.is_alive())
        self.assertFalse(runtime._watchdog_thread and runtime._watchdog_thread.is_alive())

    def test_verified_runtime_close_settle_failure_still_closes_and_faults(self):
        runtime, sink, _now = self.make_runtime()
        runtime.start(); runtime.arm_motor_output()
        sink.events.clear(); sink.fail_settle = RuntimeError("settle timeout")

        with self.assertRaises(MotorOutputFault):
            runtime.close()

        self.assertEqual(sink.events, ["shutdown-settle", "close"])
        self.assertEqual(sink.stops, [])
        self.assertTrue(sink.closed)
        self.assertIn("SHUTDOWN_STOP_FAILURE", runtime.status().fault or "")
        self.assertFalse(runtime._thread and runtime._thread.is_alive())
        self.assertFalse(runtime._watchdog_thread and runtime._watchdog_thread.is_alive())

    def test_close_wins_over_an_already_woken_watchdog_before_revoke(self):
        runtime, sink, now = self.make_runtime()
        watchdog_ready = threading.Event(); release_watchdog = threading.Event()
        close_entered = threading.Event()
        runtime._before_watchdog_revoke = lambda: (watchdog_ready.set(), release_watchdog.wait(.5))
        runtime._after_closing_before_revoke = close_entered.set
        runtime.start(); runtime.arm_motor_output()
        sink.events.clear(); now[0] = .13
        self.assertTrue(watchdog_ready.wait(.5))

        closer = threading.Thread(target=runtime.close)
        closer.start(); self.assertTrue(close_entered.wait(.5))
        release_watchdog.set(); closer.join(.5)

        self.assertFalse(closer.is_alive())
        self.assertEqual(sink.stops, [])
        self.assertEqual(sink.events, ["shutdown-settle", "close"])

    def test_close_wins_over_an_inflight_sensor_fault_tick_stop(self):
        runtime, sink, _now = self.make_runtime()
        fault_recorded = threading.Event(); release_tick = threading.Event()
        close_entered = threading.Event(); release_close = threading.Event()
        runtime._before_tick_fault_stop = lambda: (fault_recorded.set(), release_tick.wait(.5))
        runtime._after_closing_before_revoke = lambda: (close_entered.set(), release_close.wait(.5))
        runtime.start(); runtime.arm_motor_output()
        sink.events.clear()
        with runtime._state_lock:
            runtime.machine._transition(State.AUTO_ROW_FOLLOW, "test sensor fault")

        tick = threading.Thread(target=runtime.tick)
        tick.start(); self.assertTrue(fault_recorded.wait(.5))
        self.assertIn("CAMERA_TIMEOUT", runtime.status().fault or "")
        closer = threading.Thread(target=runtime.close)
        closer.start(); self.assertTrue(close_entered.wait(.5))
        release_tick.set()
        # Tick is now blocked at its lifecycle-gated output action; close owns
        # the adapter and no best-effort STOP can be queued.
        self.assertFalse(sink.stop_queued.wait(.05))
        release_close.set(); tick.join(.5); closer.join(.5)

        self.assertFalse(tick.is_alive())
        self.assertFalse(closer.is_alive())
        self.assertEqual(sink.stops, [])
        self.assertEqual(sink.events, ["shutdown-settle", "close"])

    @staticmethod
    def _capture(operation, errors):
        try: operation()
        except Exception as exc: errors.append(exc)

    def test_lease_expiry_stops_without_any_control_tick(self):
        runtime, sink, now = self.make_runtime(lease_timeout=.1)
        runtime.start(); runtime.arm_motor_output()
        now[0] = .11
        deadline = time.monotonic() + .5
        while runtime.machine.state is not State.FAULT and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertEqual(runtime.status().fault, "CONTROL_LEASE_EXPIRED")
        self.assertTrue(sink.stops)
        self.assertTrue(runtime.motor.armed)
        self.assertEqual(runtime.machine.state, State.MANUAL)
        runtime.manual_command(WheelCommand(1, 1, "after lease expiry"))
        self.assertEqual(sink.commands[-1], (1, 1, "after lease expiry"))
        runtime.close()

    def test_row_lost_stops_to_armed_manual_standby_then_manual_claims_fresh_lease(self):
        runtime, sink, _now = self.make_runtime()
        runtime.start(); runtime.arm_motor_output()
        try:
            runtime._fail_closed("ROW_LOST")
            self.assertEqual(runtime.machine.state, State.MANUAL)
            self.assertTrue(runtime.motor.armed)
            self.assertTrue(sink.stops)
            self.assertIsNone(runtime._lease_token)
            runtime.select_manual()
            runtime.manual_command(WheelCommand(2, 2, "after row lost"))
            self.assertEqual(sink.commands[-1], (2, 2, "after row lost"))
        finally:
            runtime.close()

    def test_recovery_publication_blocks_manual_claim_until_standby_is_owned(self):
        runtime, sink, _now = self.make_runtime()
        entered, release, recovery_errors, manual_errors = (
            threading.Event(), threading.Event(), [], [],
        )
        runtime.start(); runtime.arm_motor_output()
        runtime._after_recoverable_boundary_standby = lambda: (entered.set(), release.wait(.5))
        try:
            recovery = threading.Thread(
                target=lambda: self._capture(lambda: runtime._fail_closed("ROW_LOST"), recovery_errors),
                daemon=True,
            )
            recovery.start()
            self.assertTrue(entered.wait(.2))
            # Boundary STOP is already queued, but lifecycle ownership has
            # not yet published runtime's standby deadline/state/token.
            manual = threading.Thread(
                target=lambda: self._capture(
                    lambda: runtime.manual_command(WheelCommand(4, 4, "must wait")), manual_errors),
                daemon=True,
            )
            manual.start()
            self.assertTrue(manual.is_alive())
            self.assertEqual(sink.commands, [])
            release.set()
            recovery.join(.5); manual.join(.5)
            self.assertFalse(recovery.is_alive())
            self.assertFalse(manual.is_alive())
            self.assertEqual(recovery_errors, [])
            self.assertEqual(manual_errors, [])
            self.assertEqual(sink.commands[-1], (4, 4, "must wait"))
            self.assertIsNotNone(runtime._lease_token)
            self.assertTrue(runtime.lease.valid(runtime._lease_token))
        finally:
            runtime._after_recoverable_boundary_standby = None
            release.set()
            runtime.close()

    def test_explicit_stop_retains_armed_manual_standby(self):
        runtime, sink, _now = self.make_runtime()
        runtime.start(); runtime.arm_motor_output()
        try:
            runtime.stop()
            self.assertEqual(runtime.machine.state, State.MANUAL)
            self.assertTrue(runtime.motor.armed)
            self.assertTrue(sink.stops)
            # STOP is idempotent while already armed/tokenless in web
            # standby; it must not degrade into stop_all/disarm.
            runtime.stop()
            self.assertTrue(runtime.motor.armed)
            runtime.manual_command(WheelCommand(3, 3, "after explicit stop"))
            self.assertEqual(sink.commands[-1], (3, 3, "after explicit stop"))
            runtime.stop()
            self.assertTrue(runtime.motor.armed)
            runtime.manual_command(WheelCommand(4, 4, "after active manual stop"))
            self.assertEqual(sink.commands[-1], (4, 4, "after active manual stop"))
        finally:
            runtime.close()

    def test_select_manual_from_active_lease_keeps_armed_and_requires_fresh_claim(self):
        runtime, sink, _now = self.make_runtime()
        runtime.start(); runtime.arm_motor_output()
        try:
            old_token = runtime._lease_token
            runtime.manual_command(WheelCommand(5, 5, "active before toggle"))
            runtime.select_manual()
            self.assertTrue(runtime.motor.armed)
            self.assertEqual(runtime.machine.state, State.MANUAL)
            self.assertIsNone(runtime._lease_token)
            self.assertFalse(runtime.lease.valid(old_token))
            runtime.manual_command(WheelCommand(2, 2, "fresh after toggle"))
            self.assertNotEqual(runtime._lease_token, old_token)
            self.assertEqual(sink.commands[-1], (2, 2, "fresh after toggle"))
        finally:
            runtime.close()

    def test_verified_boundary_fault_still_disarms_instead_of_recovering(self):
        runtime, sink, _now = self.make_runtime()
        runtime.start(); runtime.arm_motor_output()
        try:
            sink.callback("injected CAN worker failure")
            self.assertFalse(runtime.motor.armed)
            runtime._fail_closed("MOTOR_OUTPUT_ERROR: injected CAN failure")
            self.assertFalse(runtime.motor.armed)
            with self.assertRaises(ValueError):
                runtime.manual_command(WheelCommand(1, 1, "must remain blocked"))
        finally:
            runtime.close()

    def test_nonrecoverable_sensor_fault_still_disarms(self):
        runtime, sink, _now = self.make_runtime()
        runtime.start(); runtime.arm_motor_output()
        try:
            runtime._fail_closed("CAMERA_TIMEOUT")
            self.assertFalse(runtime.motor.armed)
            self.assertTrue(sink.stops)
        finally:
            runtime.close()

    def test_previously_fresh_physical_odometry_does_not_gate_manual_without_a_command(self):
        """A missing encoder reply is diagnostic while the lease remains valid."""
        now = [0.0]

        class Odometry:
            def __init__(self): self.latest = LatestValue()
            def start(self): self.latest.publish(OdometrySample(0, 0, 0, 0), now[0])
            def stop(self): pass
            def snapshot(self): return self.latest.snapshot()

        config = RuntimeConfig(
            stream_enabled=False, max_rpm=20, auto_base_rpm=5, vision_kp=1,
            odometry_timeout_s=.1, control_lease_timeout_s=.3,
            watchdog_period_s=.01, max_control_stall_s=.12,
            physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id", "/dev/serial/by-id/test", True, True),
        )
        lease = ControlLease(.3, clock=lambda: now[0]); sink = Sink()
        motor = _VerifiedPhysicalMotorBoundary(sink, lease, max_rpm=20)
        runtime = BlockingRuntime(config, Source(), Source(), motor=motor, odometry=Odometry(), clock=lambda: now[0], lease=lease)
        runtime.start(); runtime.arm_motor_output()
        try:
            # Cross only the former encoder deadline, not the lease timeout.
            runtime._watchdog_revoke_if_running(CONTROL_LEASE_EXPIRED)
            now[0] = .101
            runtime._watchdog_revoke_if_running(CONTROL_LEASE_EXPIRED)

            self.assertIsNone(runtime.status().fault)
            self.assertEqual(runtime.machine.state, State.MANUAL)
            self.assertEqual(sink.stops, [])
        finally:
            runtime.close()


if __name__ == "__main__":
    unittest.main()
