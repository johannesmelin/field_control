"""Top-level hardware-independent field-control runtime."""
from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
import threading
import time
from typing import TYPE_CHECKING, Callable

from .config import RuntimeConfig
from .control import WheelCommand, heading_command, vision_command
from .motor_boundary import DisabledMotorBoundary, MotorBoundary, MotorOutputFault
from .heading import RowHeadingReference, RowHeadingReferenceDistanceError
from .lease import ControlLease
from .observation import (HeadingProcessor, ImuReading, Observation as SensorObservation,
                          build_observation, forward_distance_from_odometry)
from .sources import CameraSource, ImuSource, OdometrySource, SourceSnapshot
from .odometry import OdometrySample
from .state_machine import FieldStateMachine, Observation, Snapshot, State
from .turn import absolute_position_turn, in_row_turn_plan, new_row_turn_targets
from .turn_controller import TurnController, TurnObservation
from .event_log import EventLog

if TYPE_CHECKING:
    from .vision import VisionProcessor, VisionResult


# Stable fault value consumed by the bounded first-motion HIL runner.
CONTROL_LEASE_EXPIRED = "CONTROL_LEASE_EXPIRED"
# A queued A4 may briefly wait for the sole CAN worker to claim its fresh
# baseline. This matches the worker's bounded 0x92 admission contract.
A4_WORKER_ADMISSION_BOUND_S = .250


@dataclass(frozen=True)
class RuntimeStatus:
    running: bool
    mode: str
    state: str
    snapshot: Snapshot
    observation: SensorObservation | None
    last_command: WheelCommand | None
    # Historical diagnostic only.  This is never consulted for command
    # admission, lease renewal, arming, or output control.
    last_admitted_nonzero_command: WheelCommand | None
    motor_output_armed: bool
    fault: str | None


class _Lifecycle(str, Enum):
    NEW = "NEW"
    RUNNING = "RUNNING"
    CLOSING = "CLOSING"
    CLOSED = "CLOSED"


class FieldControlRuntime:
    """Owns lifecycle and joins latest sensor values without blocking control."""

    def __init__(self, config: RuntimeConfig, camera: CameraSource, imu: ImuSource,
                 *, motor: MotorBoundary | None = None, odometry: object | None = None,
                 clock=time.monotonic, lease: ControlLease | None = None) -> None:
        self.config = config.validate()
        self.camera, self.imu = camera, imu
        self.motor = motor or DisabledMotorBoundary()
        self.lease = lease or ControlLease(self.config.control_lease_timeout_s, clock=clock)
        motor_lease = getattr(self.motor, "control_lease", None)
        if motor_lease is not None and motor_lease is not self.lease:
            raise ValueError("runtime och fysisk motorgräns måste dela samma ControlLease")
        self._clock = clock
        self._odometry = odometry
        self.machine = FieldStateMachine(self.config.safety)
        self.heading = HeadingProcessor(
            self.config.heading_filter_alpha,
            RowHeadingReference(self.config.row_heading_window_m, self.config.heading_reference_min_distance_m),
        )
        # Vision (and therefore cv2) stays lazy: MANUAL diagnostics/HIL must
        # be able to start their watchdog without an image-processing stack.
        self.vision_processor: "VisionProcessor | None" = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._watchdog_thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._lifecycle = _Lifecycle.NEW
        self._last_frame_timestamp: float | None = None
        self._vision: VisionResult | None = None
        self._frame: object | None = None
        self._observation: SensorObservation | None = None
        self._last_snapshot = self.machine.snapshot(self._clock())
        self._last_command: WheelCommand | None = None
        self._last_admitted_nonzero_command: WheelCommand | None = None
        # A stationary AUTO state must establish a physical stopped hold, but
        # it must not enqueue that same hold on every control tick.  In
        # particular, repeated queued STOPs preempt the shared 0x92 encoder
        # reader and can make an otherwise healthy AUTO_PICK fault.  This is
        # a state-scoped idempotence marker only: it never replaces explicit
        # STOP, watchdog, fault, or lifecycle shutdown behaviour.
        self._stopped_hold_state: State | None = None
        self._fault: str | None = None
        self._last_imu_timestamp: float | None = None
        self._imu_sequence = 0
        self._lease_token: str | None = None
        self._turn_controller: TurnController | None = None
        self._turn_state: State | None = None
        self._turn_baseline: OdometrySample | None = None
        self._position_turn_request: object | None = None
        self._position_turn_admission_deadline_s: float | None = None
        # A confirmed marker decides the physical turn before the CAN worker
        # can claim its fresh 0x92 baseline.  Keep that decision explicitly
        # across the state-machine transition: periodic source odometry is
        # not an authority for this worker-owned transaction.
        self._a4_admission_pending = False
        self.events = EventLog(level=self.config.log_level, clock=clock)
        self._last_logged_state = self.machine.state
        self._last_heading_reference_reliable = self.heading.reference.reliable
        # Test-only seam. Production leaves this unset; it is reached after
        # capturing an admitted AUTO lease token and before queue admission.
        self._before_auto_command_admission: Callable[[], None] | None = None
        # Test-only close linearization seam; unset in production.
        self._after_closing_before_revoke: Callable[[], None] | None = None
        # Test-only watchdog seam immediately before its final lifecycle gate.
        self._before_watchdog_revoke: Callable[[], None] | None = None
        # Test-only seam after a control tick records a fault and before it
        # attempts output shutdown.  Unset in production.
        self._before_tick_fault_stop: Callable[[], None] | None = None
        self._last_control_heartbeat_s = self._clock()
        # The physical CAN encoder reader shares the verified motor worker.
        # Once it has yielded a valid sample, loss of subsequent samples is a
        # motor-boundary health failure even while MANUAL has no navigation
        # tick to consume odometry.  Keep only a monotonic deadline, not a
        # queued reading, so this remains independent of control requests.
        self._physical_odometry_deadline_s: float | None = None
        # ``select_auto`` deliberately queues a STOP before AUTO may be
        # started.  With the shared CAN worker that STOP can preempt a 0x92
        # source read.  Keep the runtime in stopped MANUAL until a *new*
        # post-STOP sample is available; this is not permission to use the
        # preceding cached sample.
        self._auto_select_odometry_recovery_pending = False
        self._auto_select_odometry_recovery_deadline_s: float | None = None
        # ``start_auto`` also issues a mandatory physical STOP immediately
        # before changing state.  It has the same shared-worker preemption
        # hazard as mode selection, but must not reuse that mode fence: a
        # successful AUTO selection may already have recovered before this
        # separate STOP is admitted.
        self._auto_start_odometry_recovery_pending = False
        self._auto_start_odometry_recovery_deadline_s: float | None = None
        # Entering a stationary AUTO state is also an intentional physical
        # STOP.  It can preempt one in-flight shared 0x92 request just like
        # the explicit mode/start STOPs above.  Keep that expected gap
        # bounded and non-admissive instead of treating its invalidated cache
        # as an immediate encoder failure on the next control tick.
        self._stationary_hold_odometry_recovery_pending = False
        self._stationary_hold_odometry_recovery_deadline_s: float | None = None
        # Every Start Auto transaction captures this generation before its
        # mandatory STOP and bounded post-STOP encoder wait.  STOP, a mode
        # selection, or close invalidates it so a delayed caller can never
        # revive AUTO after an operator cancellation.
        self._auto_start_generation = 0
        # ``motor.arm`` itself performs a verified STOP+settle.  That STOP can
        # preempt the source's preceding 0x92 pair, so an armed boundary is
        # held non-admissive until a replacement pair has completed.
        self._arm_odometry_recovery_pending = False
        self._arm_odometry_recovery_deadline_s: float | None = None
        self._arming_in_progress = False
        # Physical web deployment releases the ordinary 300 ms drive lease
        # after local arm and retains only this bounded, no-motion handoff.
        # It is never a command admission token.
        self._web_standby_deadline_s: float | None = None

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._lifecycle in (_Lifecycle.CLOSING, _Lifecycle.CLOSED):
                raise RuntimeError("runtime är stängd")
            self._lifecycle = _Lifecycle.RUNNING
        if self._thread and self._thread.is_alive():
            if not (self._watchdog_thread and self._watchdog_thread.is_alive()):
                self._watchdog_thread = threading.Thread(
                    target=self._watchdog_run, name="field-control-watchdog", daemon=True,
                )
                self._watchdog_thread.start()
            return
        self._stop.clear()
        sources = [self.camera, self.imu]
        if self._odometry is not None:
            sources.append(self._odometry)
        try:
            for source in sources:
                start = getattr(source, "start", None)
                if not callable(start):
                    raise TypeError("odometrikälla saknar start")
                start()
            with self._lock:
                self._last_control_heartbeat_s = self._clock()
            self._thread = threading.Thread(target=self._run, name="field-control", daemon=True)
            self._thread.start()
            self._watchdog_thread = threading.Thread(target=self._watchdog_run, name="field-control-watchdog", daemon=True)
            self._watchdog_thread.start()
        except Exception as exc:
            self._record_fault(f"SENSOR_START_FAILURE: {type(exc).__name__}: {exc}")
            try:
                # Use the normal shutdown ownership path. In particular, a
                # verified adapter must perform its bounded settle and close;
                # marking CLOSED here would make app.start() cleanup skip it.
                self.close()
            except Exception as cleanup_exc:
                self._record_fault(f"SENSOR_START_CLEANUP_FAILURE: {type(cleanup_exc).__name__}: {cleanup_exc}")
            raise

    def close(self) -> None:
        """Close deterministically; verified output owns its final STOP settle."""
        failure: RuntimeError | None = None
        verified_output = self._is_physical_output()
        close_output = getattr(self.motor, "close", None)
        finish_verified_close: Callable[[], None] | None = None
        with self._lifecycle_lock:
            if self._lifecycle is _Lifecycle.CLOSED:
                return
            self._lifecycle = _Lifecycle.CLOSING
            self._clear_stopped_hold()
            self._clear_web_standby()
            self._cancel_pending_auto_start()
            self._clear_turn_controller()
            # Prevent an already-woken watchdog from admitting a lease
            # revocation after shutdown ownership has transferred to the
            # verified adapter below.
            self._stop.set()
            # The shared encoder source may be in its bounded retry wait
            # after a restart STOP preempted 0x92 read.  Cancel it before the
            # verified CAN worker claims shutdown, otherwise it could admit a
            # new read in that narrow transfer window.  This is non-owning:
            # final socket/STOP ownership remains with the motor boundary.
            begin_odometry_shutdown = getattr(self._odometry, "begin_shutdown", None)
            if verified_output and callable(begin_odometry_shutdown):
                begin_odometry_shutdown()
            close_hook = self._after_closing_before_revoke
            if close_hook is not None:
                close_hook()
            self._lease_token = None
            if verified_output and callable(close_output):
                # Claim adapter shutdown ownership before releasing the gate.
                # The bounded worker settle itself occurs below, outside the
                # lifecycle lock; runtime must not queue a competing STOP.
                claim_close = getattr(self.motor, "_begin_close", None)
                finish_close = getattr(self.motor, "_finish_close", None)
                if callable(claim_close) and callable(finish_close) and claim_close():
                    finish_verified_close = finish_close
            else:
                try:
                    self.lease.revoke_any()
                except RuntimeError as exc:
                    failure = exc
                    self._record_fault(f"SHUTDOWN_STOP_FAILURE: {type(exc).__name__}: {exc}")
        if verified_output and callable(close_output):
            try:
                if finish_verified_close is not None:
                    finish_verified_close()
                else:
                    close_output()
            except RuntimeError as exc:
                failure = exc
                self._record_fault(f"SHUTDOWN_STOP_FAILURE: {type(exc).__name__}: {exc}")
        watchdog = self._watchdog_thread
        if watchdog is not None and watchdog is not threading.current_thread():
            watchdog.join(timeout=max(.1, self.config.watchdog_period_s * 4))
        if not verified_output:
            try:
                self._stop_motor("FieldControl shutdown")
            except RuntimeError as exc:
                failure = exc
                self._record_fault(f"SHUTDOWN_STOP_FAILURE: {type(exc).__name__}: {exc}")
        for source in (self.camera, self.imu, self._odometry):
            stop = getattr(source, "stop", None)
            if not callable(stop):
                continue
            try:
                stop()
            except Exception as exc:
                source_failure = RuntimeError(f"sensor shutdown failure: {type(exc).__name__}: {exc}")
                if failure is None:
                    failure = source_failure
                self._record_fault(f"SENSOR_SHUTDOWN_FAILURE: {type(exc).__name__}: {exc}")
        if self._thread: self._thread.join(timeout=5.0)
        # Disabled output has no adapter-owned verified settle, so retain its
        # existing best-effort stop semantics.
        if not verified_output and failure is None:
            try:
                self._stop_motor("FieldControl shutdown complete")
            except RuntimeError as exc:
                failure = exc
                self._record_fault(f"SHUTDOWN_STOP_FAILURE: {type(exc).__name__}: {exc}")
        if not verified_output and callable(close_output):
            try:
                close_output()
            except RuntimeError as exc:
                if failure is None:
                    failure = exc
                self._record_fault(f"SHUTDOWN_STOP_FAILURE: {type(exc).__name__}: {exc}")
        with self._lifecycle_lock:
            self._lifecycle = _Lifecycle.CLOSED
        if failure is not None: raise failure

    def tick(self) -> RuntimeStatus:
        with self._lifecycle_lock:
            if self._lifecycle in (_Lifecycle.CLOSING, _Lifecycle.CLOSED):
                self._clear_turn_controller()
                return self.status()
        now = self._clock()
        if self._expire_web_standby_if_due(now):
            return self.status()
        with self._lock:
            self._last_control_heartbeat_s = now
        if self._arming_in_progress:
            # The arm caller alone completes the lease hand-off after its
            # post-STOP 0x92. No state tick may admit output in between.
            return self.status()
        if self._arm_odometry_recovery_pending:
            if not self._complete_arm_odometry_recovery_if_ready(now):
                return self.status()
        if self._auto_select_odometry_recovery_pending:
            if not self._complete_auto_select_odometry_recovery_if_ready(now):
                # The output was explicitly STOPped before this gate.  Do not
                # evaluate AUTO state or fault on the intentionally invalid
                # cache while the bounded post-STOP 0x92 recovery is pending.
                # A terminal encoder failure or deadline still fails closed.
                return self.status()
        if self._auto_start_odometry_recovery_pending:
            if not self._complete_auto_start_odometry_recovery_if_ready(now):
                # Start Auto's mandatory STOP invalidates the shared 0x92
                # cache.  Remaining in MANUAL is deliberate until a new
                # source sample is available or the bounded recovery faults.
                return self.status()
        if self._stationary_hold_odometry_recovery_pending:
            if not self._complete_stationary_hold_odometry_recovery_if_ready(now):
                # A stationary AUTO hold has already queued STOP.  Do not
                # inspect the intentionally invalidated shared source or
                # admit the next automatic action until a fresh pair arrives.
                return self.status()
        # Physical lease expiry is owned by the independent watchdog.  It has
        # a final lifecycle-gated revocation path, unlike an in-flight control
        # tick that may overlap CLOSING.  Dry-run/default boundaries retain
        # their synchronous tick watchdog behavior.
        if not self._is_physical_output():
            try:
                if self.lease.watchdog_tick():
                    # The lease callback has already stopped output; avoid a
                    # recursive second stop if its acknowledgement failed.
                    self._fail_closed(CONTROL_LEASE_EXPIRED, output_already_stopped=True)
                    return self.status()
            except RuntimeError as exc:
                self._record_fault(f"{CONTROL_LEASE_EXPIRED}; STOP_FAILURE: {type(exc).__name__}: {exc}")
                return self.status()
        if self._lease_token is not None and self._is_physical_output() and not getattr(self.motor, "armed", False):
            self._fail_closed(getattr(self.motor, "fault_reason", None) or "MOTOR_OUTPUT_DISARMED")
            return self.status()
        camera = self.camera.latest.snapshot()
        imu = self.imu.latest.snapshot()
        row_reference_fault: str | None = None
        if imu.value is not None and imu.updated_at_s != self._last_imu_timestamp:
            with self._state_lock:
                # The row reference is a stable-following estimate, never a
                # generic visual-frame history.  In particular AUTO_PICK can
                # be stationary at a marker while encoder quantization moves
                # slightly backwards; that must not be treated as an IMU
                # acquisition failure.  Preserve the strict distance check
                # inside RowHeadingReference for genuine row-following.
                row_follow_active = self.machine.state is State.AUTO_ROW_FOLLOW
            try:
                self.heading.update(imu.value, visual_following=(row_follow_active and camera.connected
                                                                and camera.age_s(now) is not None
                                                                and camera.age_s(now) <= self.config.camera_timeout_s
                                                                and self._vision is not None and self._vision.target_x is not None),
                                    distance_m=forward_distance_from_odometry(self._odometry_snapshot(now).value))
                self._imu_sequence += 1
                if self.heading.reference.reliable != self._last_heading_reference_reliable:
                    self.events.record("heading_reference_reliable", timestamp_s=now,
                                       data={"reliable": self.heading.reference.reliable})
                    self._last_heading_reference_reliable = self.heading.reference.reliable
            except RowHeadingReferenceDistanceError:
                # The IMU sample and verified filter are valid.  Only the
                # forward-distance invariant for stable row-reference
                # collection failed, so fail closed with its own diagnostic
                # rather than fabricating an IMU source failure.
                row_reference_fault = "ROW_REFERENCE_ODOMETRY_NONMONOTONIC"
            except ValueError as exc:
                imu = SourceSnapshot(imu.value, imu.updated_at_s, False, str(exc))
            self._last_imu_timestamp = imu.updated_at_s
        frame = camera.value
        if frame is not None and camera.updated_at_s != self._last_frame_timestamp:
            if self.vision_processor is None:
                from .vision import VisionProcessor
                self.vision_processor = VisionProcessor()
            self._vision = self.vision_processor.process(frame, camera.updated_at_s or now, self.config.vision)
            self._frame = frame
            self._last_frame_timestamp = camera.updated_at_s
        odometry_snapshot = self._odometry_snapshot(now)
        sensor = build_observation(
            now, camera, imu, odometry_snapshot, self._vision, self.heading,
            self.config.camera_timeout_s, self.config.imu_timeout_s, self.config.odometry_timeout_s,
        )
        if row_reference_fault is not None:
            sensor = replace(sensor, fault=row_reference_fault)
            with self._lock:
                self._observation = sensor
            self._fail_closed(row_reference_fault)
            return self.status()
        # A marker-confirmed physical turn is the narrow hand-off where the
        # CAN worker, rather than the periodic source, becomes the encoder
        # authority.  Persist it over the state-machine transition and queue
        # admission; otherwise the next tick can fault stale external
        # odometry before the worker has sent its first 0x92.
        preview = Observation(now, sensor.camera_fresh, sensor.imu_fresh, sensor.odometry_fresh, True,
                              sensor.visual_target,
                              False if sensor.vision is None else sensor.vision.bud_in_trigger_zone,
                              False if sensor.vision is None else sensor.vision.bud_in_pick_zone,
                              False if sensor.vision is None else sensor.vision.marker_found,
                              sensor.distance_m, sensor.row_heading_reliable)
        with self._state_lock:
            a4_transition_due = (self._is_physical_output()
                                 and self.machine.turn_transition_due(preview))
            if a4_transition_due:
                self._a4_admission_pending = True
                self._position_turn_admission_deadline_s = now + A4_WORKER_ADMISSION_BOUND_S
        if self._a4_admission_timed_out(now):
            self._fail_turn("TURN_A4_ADMISSION_TIMEOUT", now)
            return self.status()
        a4_worker_owns_odometry = self._a4_worker_owns_odometry()
        a4_transaction_owns_odometry = self._a4_admission_pending or a4_worker_owns_odometry
        if not a4_transaction_owns_odometry:
            odometry_fault = self._physical_odometry_fault_if_due(now)
            if odometry_fault is not None:
                self._fail_closed(odometry_fault)
                return self.status()
        if a4_transaction_owns_odometry and not sensor.odometry_fresh:
            # StateMachine still requires an odometry-ready observation during
            # AUTO_*_TURN.  This is safe only for an already admitted physical
            # A4 request: its worker, not this stale latest-value source, is
            # verifying fresh encoder samples before it can report success.
            sensor = replace(sensor, odometry_fresh=True, fault=None)
        machine_observation = Observation(now, sensor.camera_fresh, sensor.imu_fresh, sensor.odometry_fresh, True,
                                          sensor.visual_target,
                                          False if sensor.vision is None else sensor.vision.bud_in_trigger_zone,
                                          False if sensor.vision is None else sensor.vision.bud_in_pick_zone,
                                          False if sensor.vision is None else sensor.vision.marker_found,
                                          sensor.distance_m, sensor.row_heading_reliable)
        with self._state_lock:
            snapshot = self.machine.tick(machine_observation)
        self._log_state_transition(snapshot, now)
        if snapshot.state is State.FAULT:
            self._clear_turn_controller()
            self._fail_closed(snapshot.fault or snapshot.reason)
            with self._state_lock: snapshot = self.machine.snapshot(now)
        elif snapshot.state in (State.AUTO_IN_ROW_TURN, State.AUTO_NEW_ROW_TURN):
            try:
                self._refresh_active_lease(snapshot.state)
                snapshot = self._tick_turn_controller(now, sensor, imu, machine_observation, snapshot.state)
            except (RuntimeError, ValueError) as exc:
                self._clear_turn_controller()
                self._fail_closed(f"TURN_RUNTIME_ERROR: {type(exc).__name__}: {exc}")
                with self._state_lock: snapshot = self.machine.snapshot(now)
        else:
            self._clear_turn_controller()
            try:
                self._refresh_active_lease(snapshot.state)
                self._dispatch_command(sensor, snapshot.state)
            except (RuntimeError, ValueError) as exc:
                self._fail_closed(f"MOTOR_OUTPUT_ERROR: {type(exc).__name__}: {exc}")
                with self._state_lock: snapshot = self.machine.snapshot(now)
        with self._lock:
            self._observation, self._last_snapshot = sensor, snapshot
        return self.status()

    def _run(self) -> None:
        period = 1.0 / max(1.0, self.config.navigation_frame_rate_hz)
        while not self._stop.is_set():
            started = self._clock()
            try: self.tick()
            except Exception as exc:
                self._fail_closed(f"RUNTIME_ERROR: {type(exc).__name__}: {exc}")
            self._stop.wait(max(0.0, period - (self._clock() - started)))

    def _watchdog_run(self) -> None:
        """Independent monotonic watchdog; it only revokes/queues STOP."""
        while not self._stop.wait(self.config.watchdog_period_s):
            now = self._clock()
            if self._expire_web_standby_if_due(now):
                continue
            if self._web_standby_active():
                # Standby intentionally has no ordinary drive lease and no
                # control-loop command heartbeat.  Its own deadline above is
                # the authority, not CONTROL_LEASE_EXPIRED/stall.  Continue
                # checking physical health while it remains no-motion.
                motor_fault = getattr(self.motor, "fault_reason", None)
                odometry_fault = self._physical_odometry_fault_if_due(now)
                if motor_fault is not None:
                    self._trip_independent_watchdog(str(motor_fault))
                elif odometry_fault is not None:
                    self._trip_independent_watchdog(odometry_fault)
                continue
            if not self._is_physical_output() or not getattr(self.motor, "armed", False):
                continue
            with self._lock:
                stalled = now - self._last_control_heartbeat_s >= self.config.max_control_stall_s
            self._watchdog_revoke_if_running("CONTROL_LOOP_STALL" if stalled else CONTROL_LEASE_EXPIRED)

    def _watchdog_revoke_if_running(self, reason: str) -> None:
        """Final watchdog check and lease revocation, linear with close()."""
        hook = self._before_watchdog_revoke
        if hook is not None:
            hook()
        # The adapter callback is explicitly queue-only; no CAN/socket I/O is
        # performed under this lifecycle lock.  Holding it through the final
        # check-and-revoke means CLOSING wins over any already-woken watchdog.
        with self._lifecycle_lock:
            if self._lifecycle is not _Lifecycle.RUNNING or self._stop.is_set():
                return
            if not self._is_physical_output() or not getattr(self.motor, "armed", False):
                return
            # The watchdog may have observed non-standby immediately before
            # the arming thread published the no-motion handoff and then
            # waited on this lifecycle gate.  Recheck under the same final
            # gate so that stale observation cannot revoke a released lease.
            if self._web_standby_active():
                return
            if self._arming_in_progress:
                return
            if self._arm_odometry_recovery_pending:
                self._complete_arm_odometry_recovery_if_ready(self._clock())
                if self._arm_odometry_recovery_pending:
                    return
            if self._auto_select_odometry_recovery_pending:
                # The queued mode-change STOP has already made output safe.
                # Avoid turning the expected <one sample-period> invalidation
                # into a false watchdog/odometry fault; this gate is bounded
                # by ``odometry_timeout_s`` and permits no AUTO admission.
                self._complete_auto_select_odometry_recovery_if_ready(self._clock())
                if self._auto_select_odometry_recovery_pending:
                    return
            if self._auto_start_odometry_recovery_pending:
                self._complete_auto_start_odometry_recovery_if_ready(self._clock())
                if self._auto_start_odometry_recovery_pending:
                    return
            if self._stationary_hold_odometry_recovery_pending:
                self._complete_stationary_hold_odometry_recovery_if_ready(self._clock())
                if self._stationary_hold_odometry_recovery_pending:
                    return
            if self._a4_admission_timed_out(self._clock()):
                self._trip_independent_watchdog("TURN_A4_ADMISSION_TIMEOUT")
                return
            if not self._a4_worker_owns_odometry():
                odometry_fault = self._physical_odometry_fault_if_due(self._clock())
                if odometry_fault is not None:
                    self._trip_independent_watchdog(odometry_fault)
                    return
            if reason == "CONTROL_LOOP_STALL":
                now = self._clock()
                with self._lock:
                    if now - self._last_control_heartbeat_s < self.config.max_control_stall_s:
                        return
                self._trip_independent_watchdog(reason)
                return
            try:
                if self.lease.watchdog_tick():
                    self._trip_independent_watchdog(reason)
            except RuntimeError as exc:
                self._trip_independent_watchdog(f"WATCHDOG_STOP_FAILURE: {type(exc).__name__}: {exc}")

    @staticmethod
    def _valid_physical_odometry_sample(value: object) -> bool:
        """Accept only the immutable, finite sample emitted by OdometrySource."""
        if type(value) is not OdometrySample:
            return False
        return all(isinstance(component, (int, float)) and not isinstance(component, bool)
                   and math.isfinite(component)
                   for component in (value.left_distance_m, value.right_distance_m,
                                     value.forward_distance_m, value.yaw_change_deg))

    def _physical_odometry_fault_if_due(self, now_s: float, *, require_armed: bool = True) -> str | None:
        """Validate the shared encoder source and maintain its hard deadline.

        Physical operation with an installed encoder source accepts only a
        connected, fresh immutable :class:`OdometrySample`. Numeric legacy
        snapshots remain valid for disabled/simulated outputs, but never for a
        physical motor boundary. Raised-wheel HIL runners have no odometry
        source and retain their lease-only behavior.
        """
        if not self._is_physical_output() or self._odometry is None:
            return None
        if require_armed and not getattr(self.motor, "armed", False):
            return None
        # Physical MANUAL deliberately pauses the shared 0x92 producer before
        # a held A2 command may be admitted.  A paused source has an explicit
        # no-active/no-future-read acknowledgement, so it is neither stale
        # navigation data nor a CAN health failure.  AUTO always resumes it
        # behind a fresh post-STOP recovery fence.
        if (isinstance(self._odometry, OdometrySource) and self._odometry.manual_paused
                and self.machine.state is State.MANUAL):
            return None
        try:
            snapshot = self._odometry_snapshot(now_s)
        except Exception as exc:
            return f"ODOMETRY_SOURCE_ERROR: {type(exc).__name__}: {exc}"

        updated_at_s = snapshot.updated_at_s
        fresh = (snapshot.connected and self._valid_physical_odometry_sample(snapshot.value)
                 and updated_at_s is not None and snapshot.age_s(now_s) is not None
                 and snapshot.age_s(now_s) <= self.config.odometry_timeout_s)
        if not fresh:
            # Invalid, missing, disconnected, or stale data must stop an
            # armed physical boundary now, not after an older deadline or a
            # later manual command happens to inspect it.
            return "ODOMETRY_TIMEOUT"
        deadline = float(updated_at_s) + self.config.odometry_timeout_s
        with self._lock:
            self._physical_odometry_deadline_s = max(
                deadline, self._physical_odometry_deadline_s or deadline,
            )
        return None

    def _trip_independent_watchdog(self, reason: str) -> None:
        """Fail closed without socket I/O or waiting on the CAN worker."""
        if self._web_standby_active():
            self._record_fault(reason)
            try:
                self._stop_motor(reason)
            except RuntimeError:
                pass
            return
        with self._lock:
            self._lease_token = None
            self._record_fault(reason)
        try:
            # Lease callback reaches the verified sink's nonblocking STOP queue.
            self.lease.revoke_any()
        except RuntimeError:
            # The physical worker already attempted its fail-closed path.
            pass

    def status(self) -> RuntimeStatus:
        with self._lock:
            armed = bool(getattr(self.motor, "armed", False))
            with self._state_lock: state = self.machine.state
            return RuntimeStatus(bool(self._thread and self._thread.is_alive()), "AUTO" if state.value.startswith("AUTO") else "MANUAL",
                                 state.value, self._last_snapshot, self._observation,
                                 self._last_command, self._last_admitted_nonzero_command,
                                 armed, self._fault)

    def web_standby_status(self) -> tuple[bool, float | None]:
        """Return non-secret physical-web standby state for diagnostics."""
        with self._lock:
            deadline = self._web_standby_deadline_s
        return deadline is not None, deadline

    def _web_standby_active(self) -> bool:
        with self._lock:
            return self._web_standby_deadline_s is not None

    def _clear_web_standby(self) -> None:
        with self._lock:
            self._web_standby_deadline_s = None

    def _expire_web_standby_if_due(self, now_s: float) -> bool:
        with self._lock:
            deadline = self._web_standby_deadline_s
        if deadline is None or now_s < deadline:
            return False
        self._fail_closed("WEB_STANDBY_TIMEOUT")
        return True

    def _odometry_snapshot(self, now_s: float) -> SourceSnapshot[OdometrySample | float | int]:
        if self._odometry is None:
            return SourceSnapshot(0.0, None, False, "ODOMETRY_SOURCE_MISSING")
        return self._odometry.snapshot()

    def _begin_auto_select_odometry_recovery(self) -> OdometrySource | None:
        """Fence shared encoder reads around the mandatory AUTO-select STOP."""
        if not self._is_physical_output() or not isinstance(self._odometry, OdometrySource):
            return None
        self._odometry.begin_stop_recovery()
        self._auto_select_odometry_recovery_pending = True
        self._auto_select_odometry_recovery_deadline_s = self._clock() + self.config.odometry_timeout_s
        return self._odometry

    def _complete_auto_select_odometry_recovery_if_ready(self, now_s: float) -> bool:
        """Clear AUTO's STOP fence only for one valid post-STOP sample.

        The caller may then continue normal observation/state handling.  Until
        then this method is fail-closed: AUTO cannot be started and the
        runtime emits no motion command.  A true source error remains terminal
        rather than being relabelled as a transient preemption.
        """
        if not self._auto_select_odometry_recovery_pending:
            return True
        try:
            snapshot = self._odometry_snapshot(now_s)
        except Exception as exc:
            self._fail_closed(f"ODOMETRY_SOURCE_ERROR: {type(exc).__name__}: {exc}")
            return False
        if snapshot.error is not None:
            self._fail_closed(f"ODOMETRY_SOURCE_ERROR: {snapshot.error}")
            return False
        fresh = (snapshot.connected and self._valid_physical_odometry_sample(snapshot.value)
                 and snapshot.updated_at_s is not None
                 and snapshot.age_s(now_s) is not None
                 and snapshot.age_s(now_s) <= self.config.odometry_timeout_s)
        if fresh:
            self._auto_select_odometry_recovery_pending = False
            self._auto_select_odometry_recovery_deadline_s = None
            return True
        deadline = self._auto_select_odometry_recovery_deadline_s
        if deadline is not None and now_s >= deadline:
            self._fail_closed("ODOMETRY_TIMEOUT")
        return False

    def _begin_auto_start_odometry_recovery(self) -> OdometrySource | None:
        """Fence the mandatory Start-Auto STOP before AUTO state admission."""
        if not self._is_physical_output() or not isinstance(self._odometry, OdometrySource):
            return None
        self._odometry.begin_stop_recovery()
        self._auto_start_odometry_recovery_pending = True
        self._auto_start_odometry_recovery_deadline_s = self._clock() + self.config.odometry_timeout_s
        return self._odometry

    def _cancel_pending_auto_start(self) -> None:
        """Invalidate any Start Auto caller waiting outside the state lock."""
        with self._state_lock:
            self._auto_start_generation += 1

    def _complete_auto_start_odometry_recovery_if_ready(self, now_s: float) -> bool:
        """Accept only a valid 0x92 sample obtained after Start-Auto STOP."""
        if not self._auto_start_odometry_recovery_pending:
            return True
        try:
            snapshot = self._odometry_snapshot(now_s)
        except Exception as exc:
            self._fail_closed(f"ODOMETRY_SOURCE_ERROR: {type(exc).__name__}: {exc}")
            return False
        if snapshot.error is not None:
            self._fail_closed(f"ODOMETRY_SOURCE_ERROR: {snapshot.error}")
            return False
        fresh = (snapshot.connected and self._valid_physical_odometry_sample(snapshot.value)
                 and snapshot.updated_at_s is not None
                 and snapshot.age_s(now_s) is not None
                 and snapshot.age_s(now_s) <= self.config.odometry_timeout_s)
        if fresh:
            self._auto_start_odometry_recovery_pending = False
            self._auto_start_odometry_recovery_deadline_s = None
            return True
        deadline = self._auto_start_odometry_recovery_deadline_s
        if deadline is not None and now_s >= deadline:
            self._fail_closed("ODOMETRY_TIMEOUT")
        return False

    def _begin_arm_odometry_recovery(self, source: OdometrySource) -> None:
        """Fence the mandatory arm STOP before a replacement encoder pair.

        The replacement request is deliberately still blocked here.  The
        bounded recovery deadline starts only after ``motor.arm()`` has
        completed its mandatory STOP+settle and just before this fence is
        released; otherwise the required settle time could consume all of
        the encoder-recovery budget before a replacement read is permitted.
        """
        source.begin_stop_recovery()
        self._arm_odometry_recovery_pending = True
        self._arm_odometry_recovery_deadline_s = None

    def _start_arm_odometry_recovery_deadline(self) -> None:
        """Start the bounded post-STOP encoder recovery interval."""
        if self._arm_odometry_recovery_pending:
            self._arm_odometry_recovery_deadline_s = self._clock() + self.config.odometry_timeout_s

    def _complete_arm_odometry_recovery_if_ready(self, now_s: float) -> bool:
        """Accept only post-arm-STOP odometry before exposing the lease."""
        if not self._arm_odometry_recovery_pending:
            return True
        try:
            snapshot = self._odometry_snapshot(now_s)
        except Exception as exc:
            self._fail_closed(f"ODOMETRY_SOURCE_ERROR: {type(exc).__name__}: {exc}")
            return False
        if snapshot.error is not None:
            self._fail_closed(f"ODOMETRY_SOURCE_ERROR: {snapshot.error}")
            return False
        fresh = (snapshot.connected and self._valid_physical_odometry_sample(snapshot.value)
                 and snapshot.updated_at_s is not None
                 and snapshot.age_s(now_s) is not None
                 and snapshot.age_s(now_s) <= self.config.odometry_timeout_s)
        if fresh:
            self._arm_odometry_recovery_pending = False
            self._arm_odometry_recovery_deadline_s = None
            return True
        deadline = self._arm_odometry_recovery_deadline_s
        if deadline is not None and now_s >= deadline:
            self._fail_closed("ODOMETRY_TIMEOUT")
        return False

    def _begin_stationary_hold_odometry_recovery(self) -> OdometrySource | None:
        """Fence a stationary AUTO hold STOP from the shared 0x92 reader."""
        if not self._is_physical_output() or not isinstance(self._odometry, OdometrySource):
            return None
        self._odometry.begin_stop_recovery()
        self._stationary_hold_odometry_recovery_pending = True
        self._stationary_hold_odometry_recovery_deadline_s = self._clock() + self.config.odometry_timeout_s
        return self._odometry

    def _complete_stationary_hold_odometry_recovery_if_ready(self, now_s: float) -> bool:
        """Permit AUTO only after the stationary hold's replacement sample."""
        if not self._stationary_hold_odometry_recovery_pending:
            return True
        try:
            snapshot = self._odometry_snapshot(now_s)
        except Exception as exc:
            self._fail_closed(f"ODOMETRY_SOURCE_ERROR: {type(exc).__name__}: {exc}")
            return False
        if snapshot.error is not None:
            self._fail_closed(f"ODOMETRY_SOURCE_ERROR: {snapshot.error}")
            return False
        fresh = (snapshot.connected and self._valid_physical_odometry_sample(snapshot.value)
                 and snapshot.updated_at_s is not None
                 and snapshot.age_s(now_s) is not None
                 and snapshot.age_s(now_s) <= self.config.odometry_timeout_s)
        if fresh:
            self._stationary_hold_odometry_recovery_pending = False
            self._stationary_hold_odometry_recovery_deadline_s = None
            return True
        deadline = self._stationary_hold_odometry_recovery_deadline_s
        if deadline is not None and now_s >= deadline:
            self._fail_closed("ODOMETRY_TIMEOUT")
        return False

    def _clear_turn_controller(self) -> None:
        self._turn_controller = None
        self._turn_state = None
        self._turn_baseline = None
        self._position_turn_request = None
        self._position_turn_admission_deadline_s = None
        self._a4_admission_pending = False

    def _log_state_transition(self, snapshot: Snapshot, now_s: float) -> None:
        """Emit one event per state transition, never once per control tick."""
        previous = self._last_logged_state
        if snapshot.state is previous:
            return
        self._clear_stopped_hold()
        self.events.record("state_transition", timestamp_s=now_s,
                           data={"from": previous.value, "to": snapshot.state.value, "reason": snapshot.reason})
        if snapshot.state is State.AUTO_START_DELAY:
            self.events.record("auto_delay_started", timestamp_s=now_s)
        if snapshot.state in (State.AUTO_IN_ROW_TURN, State.AUTO_NEW_ROW_TURN):
            self.events.record("turn_started", timestamp_s=now_s, data={"state": snapshot.state.value})
            self.events.record("marker_turn_triggered", timestamp_s=now_s)
        if previous in (State.AUTO_IN_ROW_TURN, State.AUTO_NEW_ROW_TURN):
            self.events.record("turn_ended", timestamp_s=now_s, data={"to": snapshot.state.value})
        if snapshot.state is State.AUTO_PICK:
            self.events.record("pick_started", timestamp_s=now_s)
        if snapshot.state is State.AUTO_SEARCH:
            self.events.record("visual_loss_search_started", timestamp_s=now_s)
        if previous is State.AUTO_SEARCH and snapshot.state is State.AUTO_ROW_FOLLOW:
            self.events.record("visual_reacquired", timestamp_s=now_s)
        self._last_logged_state = snapshot.state

    def _tick_turn_controller(self, now_s: float, sensor: SensorObservation,
                              imu: SourceSnapshot[ImuReading], machine_observation: Observation,
                              state: State) -> Snapshot:
        """Run one pure turn tick, admitting any command through the normal boundary."""
        # Physical A4 turns are autonomous motor targets.  Runtime only
        # admits/polls the sole CAN-worker request; it never blocks or opens
        # a socket. Completion is target-confirmed by fresh 0x92, not IMU.
        position_begin = getattr(self.motor, "begin_wheel_position_move", None)
        position_status = getattr(self.motor, "position_move_status", None)
        if self._is_physical_output() and callable(position_begin) and callable(position_status):
            if self._position_turn_request is None or self._turn_state is not state:
                if state is State.AUTO_IN_ROW_TURN:
                    plan = in_row_turn_plan(self.config.odometry_geometry,
                                            self.config.safety.in_row_turn_wheel_degrees,
                                            self.config.safety.new_row_turn_direction)
                else:
                    plan = new_row_turn_targets(self.config.odometry_geometry, self.config.row_spacing_m,
                                                self.config.turn_speed_rpm,
                                                self.config.safety.new_row_turn_direction,
                                                self.config.safety.inner_wheel_min_ratio)
                target = absolute_position_turn(plan, self.config.odometry_geometry)
                self._position_turn_request = position_begin(
                    left_wheel_degrees=target.left_wheel_degrees,
                    right_wheel_degrees=target.right_wheel_degrees,
                    max_motor_rpm=min(self.config.turn_speed_rpm, self.config.max_rpm),
                    motor_turns_per_wheel_turn=self.config.odometry_geometry.motor_turns_per_wheel_turn,
                    tolerance_wheel_degrees=self.config.safety.turn_distance_tolerance_m
                    / min(self.config.odometry_geometry.left_wheel_circumference_m,
                          self.config.odometry_geometry.right_wheel_circumference_m) * 360.0,
                    timeout_s=self.config.safety.turn_timeout_s,
                    deadline_s=now_s + self.config.safety.turn_timeout_s,
                    token=self._lease_token,
                )
                self._position_turn_admission_deadline_s = now_s + A4_WORKER_ADMISSION_BOUND_S
                self._turn_state = state
                return self.machine.snapshot(now_s)
            done, succeeded, error, _active = position_status(self._position_turn_request)
            if not done:
                return self.machine.snapshot(now_s)
            if not succeeded:
                return self._fail_turn(error or "TURN_POSITION_FAILED", now_s)
            self._hold_motor_stopped("turn completed")
            with self._state_lock:
                if self.machine.state is not state:
                    return self.machine.snapshot(now_s)
                try:
                    reference_before = self.heading.reference.reference_deg
                    self.heading.reference.apply_successful_180_turn()
                    self.events.record("heading_reference_180", timestamp_s=now_s,
                                       data={"before_deg": reference_before, "after_deg": self.heading.reference.reference_deg})
                except ValueError:
                    return self._fail_turn("TURN_ROW_HEADING_UNAVAILABLE", now_s)
                self.machine.complete_turn(machine_observation, succeeded=True)
                self.events.record("turn_completed", timestamp_s=now_s, data={"state": state.value})
                self._clear_turn_controller()
                return self.machine.snapshot(now_s)

        if self._turn_controller is None or self._turn_state is not state:
            sample = sensor.odometry_sample
            if sample is None:
                return self._fail_turn("TURN_ODOMETRY_SAMPLE_MISSING", now_s)
            if sensor.heading_deg is None or not sensor.imu_fresh or not isinstance(imu.value, ImuReading):
                return self._fail_turn("TURN_HEADING_STALE", now_s)
            if state is State.AUTO_IN_ROW_TURN:
                plan = in_row_turn_plan(self.config.odometry_geometry,
                                        self.config.safety.in_row_turn_wheel_degrees,
                                        self.config.safety.new_row_turn_direction)
            else:
                plan = new_row_turn_targets(self.config.odometry_geometry, self.config.row_spacing_m,
                                            self.config.turn_speed_rpm,
                                            self.config.safety.new_row_turn_direction,
                                            self.config.safety.inner_wheel_min_ratio)
            try:
                self._turn_controller = TurnController(
                    plan, initial_heading_deg=sensor.heading_deg, start_s=now_s,
                    turn_speed_motor_rpm=self.config.turn_speed_rpm, max_motor_rpm=self.config.max_rpm,
                    timeout_s=self.config.safety.turn_timeout_s,
                    distance_tolerance_m=self.config.safety.turn_distance_tolerance_m,
                    heading_tolerance_deg=self.config.safety.turn_heading_tolerance_deg,
                    heading_confirm_frames=self.config.safety.turn_heading_confirm_frames,
                    heading_max_age_s=self.config.safety.turn_heading_max_age_s,
                )
            except ValueError:
                return self._fail_turn("TURN_CONFIGURATION_INVALID", now_s)
            self._turn_state, self._turn_baseline = state, sample

        controller, baseline = self._turn_controller, self._turn_baseline
        sample = sensor.odometry_sample
        if controller is None or baseline is None or sample is None:
            return self._fail_turn("TURN_ODOMETRY_SAMPLE_MISSING", now_s)
        heading_timestamp = imu.value.timestamp_s if isinstance(imu.value, ImuReading) else None
        decision = controller.tick(TurnObservation(
            now_s, sample.left_distance_m - baseline.left_distance_m,
            sample.right_distance_m - baseline.right_distance_m,
            sensor.heading_deg, sensor.imu_fresh and isinstance(imu.value, ImuReading),
            heading_timestamp, self._imu_sequence,
        ))
        if not decision.terminal:
            if decision.command is None:
                return self._fail_turn("TURN_COMMAND_MISSING", now_s)
            self._admit_command(decision.command)
            with self._state_lock:
                return self.machine.snapshot(now_s)
        if not decision.succeeded:
            return self._fail_turn(decision.fault or "TURN_FAILURE", now_s)

        # The output is held stopped before the one-time reference/state update.
        self._hold_motor_stopped("turn completed")
        with self._state_lock:
            if self._turn_controller is not controller or self._turn_state is not state or self.machine.state is not state:
                return self.machine.snapshot(now_s)
            try:
                reference_before = self.heading.reference.reference_deg
                self.heading.reference.apply_successful_180_turn()
                self.events.record("heading_reference_180", timestamp_s=now_s,
                                   data={"before_deg": reference_before, "after_deg": self.heading.reference.reference_deg})
            except ValueError:
                return self._fail_turn("TURN_ROW_HEADING_UNAVAILABLE", now_s)
            self.machine.complete_turn(machine_observation, succeeded=True)
            self.events.record("turn_completed", timestamp_s=now_s, data={"state": state.value})
            self._clear_turn_controller()
            return self.machine.snapshot(now_s)

    def _fail_turn(self, reason: str, now_s: float) -> Snapshot:
        self._clear_turn_controller()
        self._fail_closed(reason)
        with self._state_lock:
            return self.machine.snapshot(now_s)

    def _dispatch_command(self, observation: SensorObservation, state: State) -> None:
        active = state in FieldStateMachine._ACTIVE
        command = None
        if active and state not in (State.AUTO_START_DELAY, State.AUTO_PICK,
                                    State.AUTO_IN_ROW_TURN, State.AUTO_NEW_ROW_TURN):
            if observation.visual_target and observation.vision is not None:
                command = vision_command(
                    observation.vision.target_x or 0.0,
                    self.config.vision.x_goal * observation.vision.overlay.shape[1],
                    self.config.auto_base_rpm, self.config.vision_kp,
                    self.config.vision_deadband_px, self.config.max_vision_correction_rpm,
                    self.config.max_rpm,
                )
            elif observation.heading_deg is not None and observation.row_heading_reference_deg is not None and observation.row_heading_reliable:
                command = heading_command(
                    observation.row_heading_reference_deg, observation.heading_deg,
                    self.config.search_speed_rpm, self.config.heading_kp,
                    self.config.heading_deadband_deg, self.config.max_heading_correction_rpm,
                    self.config.max_rpm,
                )
        if command is None:
            # Manual commands are refreshed only by fresh operator input.
            # Background navigation ticks must neither overwrite them nor
            # keep them alive: lack of input expires the bounded lease.
            if state is State.MANUAL and self._is_physical_output():
                # Before explicit arming, the verified boundary has already
                # completed its own STOP+settle at open.  A background MANUAL
                # tick must remain inert: queuing another STOP can preempt the
                # first shared 0x92 encoder read needed by arm_motor_output().
                # Once armed, retain the existing lease-only MANUAL behavior.
                return
            if state in FieldStateMachine._ACTIVE and self._is_physical_output():
                self._hold_motor_stopped_once(state, f"state {state.value}")
            else:
                self._stop_motor(f"state {state.value}")
            self._last_command = None
            return
        self._admit_command(command)

    def _admit_command(self, command: WheelCommand) -> None:
        # A subsequently stationary state needs to issue one new hold even
        # when it is the same state that was previously held stopped.
        self._clear_stopped_hold()
        self._last_command = command
        if getattr(self.motor, "armed", False):
            token = self._lease_token
            hook = self._before_auto_command_admission
            if hook is not None:
                hook()
            # Sink admission is queue-only. Holding the lifecycle gate makes
            # this linear with CLOSING without holding any CAN/I/O lock.
            with self._lifecycle_lock:
                if self._lifecycle in (_Lifecycle.CLOSING, _Lifecycle.CLOSED):
                    return
                if self._is_physical_output() and self._lifecycle is not _Lifecycle.RUNNING:
                    return
                self.motor.command(command, token)
                self._record_admitted_nonzero_command(command)

    def arm_motor_output(self, *, _retain_arming_gate: bool = False) -> None:
        """Explicitly acquire and arm the configured physical output.

        The verified physical adapter performs the required STOP+0x9C settle
        before accepting this lease. This method is intentionally not
        exposed through the diagnostics web API.
        """
        with self._lifecycle_lock:
            if self._lifecycle is not _Lifecycle.RUNNING:
                raise ValueError("runtime är inte igång för armering")
            if not self._is_physical_output(): raise ValueError("ingen fysisk motorutgång är konfigurerad")
            if not (self._thread and self._thread.is_alive() and self._watchdog_thread and self._watchdog_thread.is_alive()):
                raise ValueError("runtime och oberoende watchdog måste vara igång före armering")
            if getattr(self.motor, "armed", False): raise ValueError("motorutgången är redan armerad")
        with self._state_lock:
            arming_manual = self.machine.state is State.MANUAL
        manual_source = self._odometry if (self._is_physical_output() and arming_manual
                                           and isinstance(self._odometry, OdometrySource)) else None
        # MANUAL is intentionally independent of navigation odometry.  Pause
        # the producer, and wait for its linearized acknowledgement, before a
        # lease can expose A2 output.  This prevents 0x92 traffic from
        # competing with held web commands rather than merely tolerating a
        # timeout after the fact.
        if manual_source is not None:
            if not manual_source.pause_for_manual(self.config.odometry_timeout_s):
                self._fail_closed("ODOMETRY_PAUSE_TIMEOUT")
                raise ValueError("fysisk odometri kunde inte pausas före MANUAL-armering")
        # AUTO arming retains the established fresh-odometry gate.  Do not
        # hold the lifecycle gate while waiting: close() must be able to stop
        # the source and wake this condition immediately.
        if isinstance(self._odometry, OdometrySource) and manual_source is None:
            if not self._odometry.wait_until_ready(self.config.odometry_timeout_s):
                self._fail_closed("ODOMETRY_TIMEOUT")
                raise ValueError("fysisk odometri blev inte redo före timeout")
        with self._lifecycle_lock:
            if self._lifecycle is not _Lifecycle.RUNNING:
                raise RuntimeError("runtime stängs under odometriförberedelse")
            if self._fault is not None:
                raise ValueError("runtime har fel före armering")
            if manual_source is None:
                odometry_fault = self._physical_odometry_fault_if_due(self._clock(), require_armed=False)
                if odometry_fault is not None:
                    self._fail_closed(odometry_fault)
                    raise ValueError("fysisk odometri är felaktig eller för gammal")
            token = self.lease.acquire()
            self._arming_in_progress = True
        recovery_source = (self._odometry if isinstance(self._odometry, OdometrySource)
                           and manual_source is None else None)
        if recovery_source is not None:
            # Fence before arm() queues its mandatory STOP.  A pre-existing
            # read is discarded and no next 0x92 reaches the shared worker
            # until that STOP admission has happened.
            self._begin_arm_odometry_recovery(recovery_source)
        try:
            self.motor.arm(token)  # type: ignore[attr-defined]
            if recovery_source is not None:
                # ``begin_stop_recovery`` deliberately prevents a replacement
                # 0x92 until the verified arm STOP+settle has completed.  Give
                # that now-permitted replacement its full configured bounded
                # interval, rather than charging the STOP settle against it.
                self._start_arm_odometry_recovery_deadline()
        except Exception:
            self._arm_odometry_recovery_pending = False
            self._arm_odometry_recovery_deadline_s = None
            self._arming_in_progress = False
            self.lease.revoke_any()
            raise
        finally:
            if recovery_source is not None:
                recovery_source.finish_stop_recovery()
        if recovery_source is not None:
            # Never expose the lease or permit AUTO while the arm STOP's
            # replacement 0x92 is missing.  The concurrent watchdog has the
            # same bounded pending gate and will fail closed on real error or
            # deadline instead of treating this expected preemption as stale.
            if not recovery_source.wait_until_ready(self.config.odometry_timeout_s):
                self._arm_odometry_recovery_pending = False
                self._arm_odometry_recovery_deadline_s = None
                self._arming_in_progress = False
                self._fail_closed("ODOMETRY_TIMEOUT")
                raise ValueError("fysisk odometri återhämtade sig inte efter armerings-STOP")
            if not self._complete_arm_odometry_recovery_if_ready(self._clock()):
                self._arming_in_progress = False
                raise ValueError("fysisk odometri är felaktig efter armerings-STOP")
        with self._lifecycle_lock:
            if self._lifecycle is not _Lifecycle.RUNNING:
                self._arming_in_progress = False
                self.lease.revoke_any()
                raise RuntimeError("runtime stängs under armering")
            self._lease_token = token
            # Physical web standby must exchange this just-acquired ordinary
            # lease for a no-motion standby token before an independent
            # watchdog may observe it.  Retaining this gate covers only that
            # synchronous handoff; explicit STOP and close still bypass it.
            if not _retain_arming_gate:
                self._arming_in_progress = False

    def arm_motor_output_for_web_standby(self) -> None:
        """Locally arm, then relinquish drive authority into bounded standby.

        The regular arm path supplies its verified STOP+0x9C and fresh 0x92
        gates.  The resulting ordinary lease is released, never extended;
        only a later valid web manual/Start Auto action can claim a new one.
        """
        with self._state_lock:
            if self.machine.state is not State.MANUAL:
                raise ValueError("fysisk webbstandby kräver MANUAL efter uppstart")
        # Keep the arm gate continuously from lease acquisition through the
        # no-motion standby exchange.  Without it, a just-woken watchdog can
        # revoke the ordinary lease in the few instructions between the arm
        # return and ``enter_web_standby()``, preempting the shared 0x92
        # reader even though no wheel command has been admitted.
        self.arm_motor_output(_retain_arming_gate=True)
        enter = getattr(self.motor, "enter_web_standby", None)
        try:
            with self._lifecycle_lock:
                with self._state_lock:
                    manual = self.machine.state is State.MANUAL
                token = self._lease_token
                if self._lifecycle is not _Lifecycle.RUNNING or not manual or not callable(enter):
                    raise ValueError("fysisk webbstandby kan inte etableras")
                enter(token)
                self._lease_token = None
                with self._lock:
                    self._web_standby_deadline_s = self._clock() + self.config.physical_web_standby_timeout_s
        except Exception:
            self._stop_motor("fysisk webbstandby misslyckades")
            raise
        finally:
            # The standby deadline has been published (or a fail-closed STOP
            # has been queued) before ordinary watchdog/tick admission may
            # resume.  This is deliberately not a general watchdog bypass.
            self._arming_in_progress = False

    def _claim_web_standby_if_needed(self) -> None:
        """Atomically exchange no-motion standby for one fresh drive lease."""
        if not self._web_standby_active():
            return
        if self._expire_web_standby_if_due(self._clock()):
            raise ValueError("fysisk webbstandby har löpt ut")
        with self._state_lock:
            if self.machine.state is not State.MANUAL:
                raise ValueError("fysisk webbstandby kräver MANUAL")
        claim = getattr(self.motor, "claim_web_standby", None)
        if not callable(claim):
            self._fail_closed("WEB_STANDBY_INTERFACE_ERROR")
            raise ValueError("motorgränsen saknar webbstandby")
        expired = False
        with self._lifecycle_lock:
            with self._lock:
                deadline = self._web_standby_deadline_s
            if deadline is None:
                raise ValueError("fysisk webbstandby är inte längre tillgänglig")
            if self._clock() >= deadline:
                expired = True
            elif self._lifecycle is not _Lifecycle.RUNNING:
                raise ValueError("fysisk webbstandby är inte längre tillgänglig")
            else:
                token = self.lease.acquire()
                try:
                    claim(token)
                except Exception:
                    self.lease.revoke_any()
                    raise
                self._lease_token = token
                self._clear_web_standby()
        if expired:
            self._fail_closed("WEB_STANDBY_TIMEOUT")
            raise ValueError("fysisk webbstandby har löpt ut")

    def _refresh_active_lease(self, state: State) -> None:
        if not self._is_physical_output() or state not in FieldStateMachine._ACTIVE:
            return
        if not getattr(self.motor, "armed", False) or self._lease_token is None:
            raise MotorOutputFault("fysisk motorutgång måste vara explicit armerad före AUTO")
        self.lease.refresh(self._lease_token)

    def _is_physical_output(self) -> bool:
        return callable(getattr(self.motor, "arm", None))

    def _a4_worker_owns_odometry(self) -> bool:
        """Whether a live worker-owned A4 transaction is confirming 0x92.

        This intentionally keys on the runtime's private request handle, not
        on a loose motor capability: before admission and after cleanup the
        normal independent physical-odometry watchdog remains mandatory.
        """
        if not self._is_physical_output() or self._position_turn_request is None:
            return False
        with self._state_lock:
            if self.machine.state not in (State.AUTO_IN_ROW_TURN, State.AUTO_NEW_ROW_TURN):
                return False
        # The marker decision remains authoritative through the one bounded
        # queue-admission interval.  There is no external-odometry fallback
        # in that interval: worker-owned fresh 0x92 either starts the A4
        # transaction or its deadline faults and STOPs it.
        if self._a4_admission_pending and self._position_turn_request is None:
            deadline = self._position_turn_admission_deadline_s
            return deadline is not None and self._clock() <= deadline
        status = getattr(self.motor, "position_move_status", None)
        if not callable(status):
            return False
        try:
            done, _succeeded, error, active = status(self._position_turn_request)
        except Exception:
            # A malformed or unavailable status may never extend actuator
            # operation; retain the normal independent odometry fail-safe.
            return False
        if active is True:
            return True
        # The one transition hand-off is bounded and only applies to this
        # exact, nonterminal request. A generic queued A4 never gets this
        # exception; it is created only after marker-confirmed state entry.
        deadline = self._position_turn_admission_deadline_s
        return (not done and error is None and deadline is not None
                and self._clock() <= deadline)

    def _a4_admission_timed_out(self, now_s: float) -> bool:
        """Whether an exact marker hand-off missed the worker-claim bound."""
        request = self._position_turn_request
        deadline = self._position_turn_admission_deadline_s
        if deadline is None or now_s <= deadline:
            return False
        if request is None:
            return self._a4_admission_pending
        status = getattr(self.motor, "position_move_status", None)
        if not callable(status):
            return True
        try:
            done, _succeeded, _error, active = status(request)
        except Exception:
            return True
        return not done and active is not True

    def _stop_motor(self, reason: str) -> None:
        # Explicit STOP, watchdog/fault stop, mode selection and shutdown
        # must always remain immediate and must invalidate a prior AUTO hold.
        self._clear_stopped_hold()
        self._clear_web_standby()
        self._clear_stationary_hold_odometry_recovery()
        self._last_command = None
        self._lease_token = None
        if self._is_physical_output():
            # A close that has claimed adapter shutdown ownership must be the
            # only physical STOP issuer.  This action only admits queue work;
            # no CAN I/O occurs while the lifecycle gate is held.
            with self._lifecycle_lock:
                if self._lifecycle is not _Lifecycle.RUNNING:
                    return
        self.motor.stop_all(reason)

    def _record_admitted_nonzero_command(self, command: WheelCommand) -> None:
        """Retain immutable evidence of a successful nonzero sink admission.

        This deliberately has no control role.  In particular STOP clears the
        live ``_last_command`` and the lease token, while this field remains
        available solely to explain the command that preceded a safe stop.
        """
        if command.left_rpm == 0.0 and command.right_rpm == 0.0:
            return
        with self._lock:
            self._last_admitted_nonzero_command = command

    def _hold_motor_stopped(self, reason: str) -> None:
        hold = getattr(self.motor, "hold_stopped", None)
        if not callable(hold):
            self._stop_motor(reason)
            return
        if self._is_physical_output():
            with self._lifecycle_lock:
                if self._lifecycle is not _Lifecycle.RUNNING:
                    return
                hold(reason, self._lease_token)
            return
        hold(reason, self._lease_token)

    def _hold_motor_stopped_once(self, state: State, reason: str) -> None:
        """Queue one physical hold for a stationary AUTO state.

        The physical boundary remains stopped between ticks.  The marker is
        reset by a state transition, any command admission, and every
        explicit/fault/lifecycle stop, so it cannot suppress a required STOP.
        """
        with self._lock:
            if self._stopped_hold_state is state:
                return
        source = self._begin_stationary_hold_odometry_recovery()
        try:
            self._hold_motor_stopped(reason)
        except Exception:
            # Do not remember a hold that failed to enter the boundary.
            self._clear_stopped_hold()
            raise
        finally:
            if source is not None:
                source.finish_stop_recovery()
        with self._lock:
            self._stopped_hold_state = state

    def _clear_stopped_hold(self) -> None:
        with self._lock:
            self._stopped_hold_state = None

    def _clear_stationary_hold_odometry_recovery(self) -> None:
        self._stationary_hold_odometry_recovery_pending = False
        self._stationary_hold_odometry_recovery_deadline_s = None

    def _record_fault(self, reason: str) -> None:
        # A first fault is the causal diagnostic. Later safety ticks must not
        # overwrite it or trigger a new output-stop sequence.
        with self._state_lock:
            if self._fault is None:
                self._fault = reason
                self.machine._fault(reason)
                self.events.record("fault", level="ERROR", timestamp_s=self._clock(), data={"reason": reason})
            self._last_snapshot = self.machine.snapshot(self._clock())

    def _fail_closed(self, reason: str, *, output_already_stopped: bool = False) -> None:
        self._record_fault(reason)
        if output_already_stopped:
            return
        hook = self._before_tick_fault_stop
        if hook is not None:
            hook()
        try:
            self._stop_motor(reason)
        except RuntimeError as exc:
            self._record_fault(f"{reason}; STOP_FAILURE: {type(exc).__name__}: {exc}")

    def select_manual(self) -> None:
        self._cancel_pending_auto_start()
        self._clear_turn_controller()
        try:
            self._stop_motor("MANUAL vald")
        except RuntimeError as exc:
            self._record_fault(f"MODE_CHANGE_STOP_FAILURE: {type(exc).__name__}: {exc}")
            raise
        if self._is_physical_output() and isinstance(self._odometry, OdometrySource):
            if not self._odometry.pause_for_manual(self.config.odometry_timeout_s):
                self._fail_closed("ODOMETRY_PAUSE_TIMEOUT")
                raise ValueError("fysisk odometri kunde inte pausas efter MANUAL-STOP")
        with self._state_lock: self.machine.select_manual()
        self.events.record("mode_manual_selected", timestamp_s=self._clock())

    def select_auto(self) -> None:
        self._cancel_pending_auto_start()
        self._clear_turn_controller()
        # A locally armed physical web deployment must be able to select AUTO
        # without a browser gaining (or needing) arming authority.  Queue a
        # verified stopped hold in that one case: it preserves the existing
        # local lease token while still fencing the shared 0x92 reader until
        # a replacement sample arrives.  Explicit STOP, faults, watchdog and
        # shutdown deliberately retain their disarming ``_stop_motor`` path.
        preserve_local_arm = (self._is_physical_output()
                              and bool(getattr(self.motor, "armed", False))
                              and self._lease_token is not None)
        source = self._begin_auto_select_odometry_recovery()
        stop_admitted = False
        try:
            if preserve_local_arm:
                self._hold_motor_stopped("AUTO valt")
            else:
                self._stop_motor("AUTO valt")
            stop_admitted = True
        except RuntimeError as exc:
            reason = f"MODE_CHANGE_STOP_FAILURE: {type(exc).__name__}: {exc}"
            # A failed STOP admission may not reopen the shared encoder
            # reader. Keep MANUAL's acknowledged pause in force while the
            # independent fail-closed path attempts a safe STOP/disarm.
            self._fail_closed(reason)
            raise
        finally:
            if source is not None and stop_admitted:
                source.resume_from_manual()
                source.finish_stop_recovery()
        with self._state_lock: self.machine.select_auto()
        self.events.record("mode_auto_selected", timestamp_s=self._clock())

    def start_auto(self) -> None:
        with self._lifecycle_lock:
            if self._is_physical_output() and self._lifecycle is not _Lifecycle.RUNNING:
                raise ValueError("runtime stängs")
        with self._lock:
            observation = self._observation
        if observation is None:
            raise ValueError("sensorobservation saknas")
        self._claim_web_standby_if_needed()
        if self._arming_in_progress:
            raise ValueError("AUTO väntar på att fysisk armering ska slutföras")
        if self._auto_select_odometry_recovery_pending:
            if not self._complete_auto_select_odometry_recovery_if_ready(self._clock()):
                raise ValueError("AUTO väntar på en ny encoderavläsning efter STOP")
        if self._is_physical_output() and not getattr(self.motor, "armed", False):
            self._stop_motor("AUTO nekad: motorutgång ej armerad")
            raise ValueError("fysisk motorutgång måste armeras explicit före Start Auto")
        with self._state_lock:
            start_generation = self._auto_start_generation
        source = self._begin_auto_start_odometry_recovery()
        stop_admitted = False
        try:
            if self._is_physical_output():
                self._hold_motor_stopped("AUTO startförberedelse")
            else:
                self.motor.stop_all("AUTO startförberedelse")
            stop_admitted = True
        except RuntimeError as exc:
            self._fail_closed(f"MOTOR_OUTPUT_ERROR: {type(exc).__name__}: {exc}")
            raise
        finally:
            if source is not None and stop_admitted:
                source.resume_from_manual()
                source.finish_stop_recovery()
        if source is not None:
            # ``hold_stopped`` admits the STOP without blocking the control
            # path.  This API call is deliberately bounded like arming: no
            # AUTO state transition, lease refresh or motion admission can
            # occur until the source has published a post-STOP 0x92 sample.
            if not source.wait_until_ready(self.config.odometry_timeout_s):
                with self._state_lock:
                    cancelled = start_generation != self._auto_start_generation
                if cancelled:
                    raise ValueError("AUTO-start avbröts")
                self._auto_start_odometry_recovery_pending = False
                self._auto_start_odometry_recovery_deadline_s = None
                self._fail_closed("ODOMETRY_TIMEOUT")
                raise ValueError("AUTO saknar ny encoderavläsning efter start-STOP")
            if not self._complete_auto_start_odometry_recovery_if_ready(self._clock()):
                with self._state_lock:
                    cancelled = start_generation != self._auto_start_generation
                if cancelled:
                    raise ValueError("AUTO-start avbröts")
                raise ValueError("AUTO saknar giltig encoderavläsning efter start-STOP")
        # Linearize the final state transition with shutdown and cancellation.
        # A STOP/select action can run while the bounded wait above releases
        # locks; it must win instead of allowing that stale caller to start.
        with self._lifecycle_lock:
            if self._is_physical_output() and self._lifecycle is not _Lifecycle.RUNNING:
                raise ValueError("AUTO-start avbröts")
            with self._state_lock:
                if start_generation != self._auto_start_generation:
                    raise ValueError("AUTO-start avbröts")
                self.machine.request_start_auto(Observation(
            observation.now_s, observation.camera_fresh, observation.imu_fresh,
            observation.odometry_fresh, True, observation.visual_target,
            False if observation.vision is None else observation.vision.bud_in_trigger_zone,
            False if observation.vision is None else observation.vision.bud_in_pick_zone,
            False if observation.vision is None else observation.vision.marker_found,
            observation.distance_m, observation.row_heading_reliable,
                ))
        self.events.record("auto_start_requested", timestamp_s=self._clock())

    def stop(self) -> None:
        self._cancel_pending_auto_start()
        self._clear_turn_controller()
        try:
            self._stop_motor("STOP")
        except RuntimeError as exc:
            self._record_fault(f"STOP_FAILURE: {type(exc).__name__}: {exc}")
            raise
        with self._state_lock: self.machine.stop()
        self.events.record("stop_requested", timestamp_s=self._clock())

    def stop_and_settle(self) -> None:
        """Public STOP followed by the bounded verified physical settle.

        Normal STOP remains nonblocking.  This explicit operation exists for
        controlled HIL verification, where completion must include the
        worker's STOP+0x9C acknowledgement before the application is closed.
        """
        self.stop()
        settle = getattr(self.motor, "stop_and_settle_for_restart", None)
        if not callable(settle):
            raise RuntimeError("motorgränsen saknar verifierad STOP+0x9C-settle")
        try:
            settle("STOP HIL settle")
        except Exception as exc:
            self._record_fault(f"STOP_SETTLE_FAILURE: {type(exc).__name__}: {exc}")
            raise
        terminal = self.status()
        if terminal.state != State.MANUAL.value or terminal.motor_output_armed or terminal.fault is not None:
            raise RuntimeError("STOP+0x9C-settle nådde inte disarmerad MANUAL")
        self.events.record("stop_settled", timestamp_s=self._clock())

    def manual_command(self, command: WheelCommand) -> None:
        with self._lifecycle_lock:
            if self._is_physical_output() and self._lifecycle is not _Lifecycle.RUNNING:
                raise ValueError("runtime stängs")
            if self._arming_in_progress:
                raise ValueError("manuell styrning väntar på att fysisk armering ska slutföras")
            self._claim_web_standby_if_needed()
            self._manual_command_admitted(command)

    def _manual_command_admitted(self, command: WheelCommand) -> None:
        with self._state_lock: manual = self.machine.state is State.MANUAL
        if not manual:
            raise ValueError("manuellt kommando kräver MANUAL")
        if not (isinstance(self._odometry, OdometrySource) and self._odometry.manual_paused):
            odometry_fault = self._physical_odometry_fault_if_due(self._clock())
            if odometry_fault is not None:
                self._fail_closed(odometry_fault)
                raise ValueError("fysisk odometri är felaktig eller för gammal")
        if not getattr(self.motor, "armed", False):
            self._stop_motor("manuell output är avstängd")
            raise ValueError("motorutgången är avstängd")
        if self._lease_token is None:
            self._stop_motor("manuell control-lease saknas")
            raise ValueError("manuell control-lease saknas")
        try:
            self.lease.refresh(self._lease_token)
            self.motor.command(command, self._lease_token)
        except (RuntimeError, ValueError) as exc:
            self._fail_closed(f"MOTOR_OUTPUT_ERROR: {type(exc).__name__}: {exc}")
            raise
        self._last_command = command
        self._record_admitted_nonzero_command(command)

    def latest_image(self, view: str) -> bytes | None:
        with self._lock:
            frame, result = self._frame, self._vision
            if frame is None:
                return None
            if view == "raw": image = frame
            elif view == "overlay" and result is not None: image = result.overlay
            elif result is not None: image = result.masks.get(view)
            else: image = None
        if image is None:
            return None
        import cv2
        if image.shape[1] != self.config.stream_width or image.shape[0] != self.config.stream_height:
            image = cv2.resize(image, (self.config.stream_width, self.config.stream_height))
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality])
        return encoded.tobytes() if ok else None
