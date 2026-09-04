"""Lease-gated adapter for the verified remote_control physical CAN worker.

This module deliberately contains no CAN framing, socket handling, or worker.
Those safety-critical functions remain owned by ``remote_control.physical``.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Protocol

from .control import WheelCommand
from .lease import ControlLease
from .motor_boundary import (LEFT_ID, MotorBoundary, MotorOutputFault,
                             PhysicalOutputDisabled, VERIFIED_MAX_MOTOR_RPM)
from .sources import EncoderReadPreempted, RightEncoderReplyTimeout


class VerifiedMotorSink(Protocol):
    def command(self, left_rpm: float, right_rpm: float, reason: str) -> None: ...
    def stop_all(self, reason: str) -> None: ...
    def stop_and_settle_for_restart(self) -> None: ...
    def stop_and_settle_and_close(self) -> None: ...
    def close(self) -> None: ...
    def set_fault_callback(self, callback: Callable[[str], None]) -> None: ...
    def read_multi_turn_angles(self) -> tuple[float, float]: ...
    def begin_wheel_position_move(self, *, left_wheel_degrees: float, right_wheel_degrees: float,
                                  max_motor_rpm: float, motor_turns_per_wheel_turn: float,
                                  tolerance_wheel_degrees: float, timeout_s: float, deadline_s: float) -> object: ...
    def position_move_status(self, request: object) -> object: ...


class SharedCanEncoderBackend:
    """Non-owning encoder adapter over the verified CAN worker's sole socket.

    Closing the odometry source must never close the motor sink; application
    lifecycle ownership remains with :class:`_VerifiedPhysicalMotorBoundary`.
    """
    def __init__(self, sink: VerifiedMotorSink) -> None:
        self._sink = sink
        # This is deliberately adapter-local and non-owning. It prevents any
        # source from admitting a later 0x92 request while FieldControl is
        # transferring shutdown ownership to the CAN worker.
        self._shutdown = threading.Event()
        # ``read_multi_turn_angles`` is the sink's admission point for a 0x92
        # pair.  Keep that call and the shutdown barrier under one lock: an
        # Event check followed by an unlocked sink call leaves a TOCTOU where
        # shutdown may return and a newly scheduled reader can still admit
        # 0x92.  The verified sink bounds every read, so shutdown can wait for
        # at most that existing bounded transaction; it never owns its socket.
        self._admission_lock = threading.Lock()

    def angles(self) -> tuple[float, float]:
        return self.angles_with_timestamp()[0]

    def angles_with_timestamp(self) -> tuple[tuple[float, float], float]:
        # Do not release this gate until the sink has either accepted and
        # completed the bounded request or rejected it.  ``begin_shutdown``
        # shares this exact gate, making its return the linearization point
        # after which no new sink read can be admitted.
        with self._admission_lock:
            if self._shutdown.is_set():
                raise RuntimeError("delad encoderadapter är stängd")
            try:
                sample_reader = getattr(self._sink, "read_multi_turn_angles_sample", None)
                if callable(sample_reader):
                    angles, acquired_at_s = sample_reader()
                else:
                    angles, acquired_at_s = self._sink.read_multi_turn_angles(), time.monotonic()
            except Exception as exc:
                # Once this adapter has been cancelled that result is terminal;
                # never queue another uncorrelatable 0x92 read.
                if self._shutdown.is_set():
                    raise RuntimeError("delad encoderadapter stängdes under avläsning") from exc
                # Keep field_control importable without remote_control.  The
                # optional dependency is consulted only while mapping its explicit
                # transient STOP/restart preemption signal.
                if _is_remote_angle_read_preempted(exc):
                    raise EncoderReadPreempted("0x92-avläsning preempterades av säkert STOP") from exc
                if _is_exact_right_encoder_timeout_after_left_reply(exc):
                    raise RightEncoderReplyTimeout(
                        "0x92 fick giltigt svar från 0x141 men timeout från 0x142"
                    ) from exc
                raise
        if (not isinstance(angles, tuple) or len(angles) != 2
                or not all(isinstance(item, (int, float)) and not isinstance(item, bool)
                           and math.isfinite(item) for item in angles)):
            raise MotorOutputFault("verifierad CAN-worker returnerade ogiltig flervarvsvinkel")
        if (not isinstance(acquired_at_s, (int, float)) or isinstance(acquired_at_s, bool)
                or not math.isfinite(acquired_at_s) or acquired_at_s < 0):
            raise MotorOutputFault("verifierad CAN-worker returnerade ogiltig 0x92-tid")
        return (float(angles[0]), float(angles[1])), float(acquired_at_s)

    def close(self) -> None:
        # Deliberately non-owning: no socket or motor sink is closed here.
        # It only cancels future adapter reads so OdometrySource.stop() may
        # safely linearize before the motor boundary claims CAN shutdown.
        self._shutdown.set()

    def begin_shutdown(self) -> None:
        """Atomically prohibit further reads without taking CAN ownership."""
        # Wait only for one already-admitted, protocol-bounded sink read.  Do
        # not call into the sink here: this adapter remains non-owning and
        # cannot deadlock against the CAN worker's final close.
        with self._admission_lock:
            self._shutdown.set()


def _is_remote_angle_read_preempted(exc: Exception) -> bool:
    """Recognize only remote_control's public terminal preemption type.

    All import/protocol/timeout errors remain untouched and therefore retain
    the existing fail-closed path.
    """
    try:
        from remote_control.physical import AngleReadPreempted
    except ImportError:
        return False
    return isinstance(exc, AngleReadPreempted)


def _is_exact_right_encoder_timeout_after_left_reply(exc: Exception) -> bool:
    """Recognize only the user-authorized 0x141-replied/0x142-timeout case.

    ``remote_control`` exposes this one outcome as a public typed exception.
    Do not treat a text-only CAN error, a left timeout, malformed reply, or
    any other exception as this narrow degradation.
    """
    try:
        from remote_control.physical import PartialLeftAngleReadTimeout
    except ImportError:
        return False
    return isinstance(exc, PartialLeftAngleReadTimeout) and exc.received_reply_ids == (LEFT_ID,)


class _VerifiedPhysicalMotorBoundary:
    """Small non-I/O adapter over remote_control.PhysicalCanMotors.

    The lease is held only long enough to admit a queue operation; the verified
    worker is the sole owner of socket I/O and performs paired A2/STOP handling.
    """
    def __init__(self, sink: VerifiedMotorSink, lease: ControlLease, *, max_rpm: float) -> None:
        if not 0 < max_rpm <= VERIFIED_MAX_MOTOR_RPM:
            raise ValueError("max_rpm måste ligga inom verifierad gräns")
        self._sink, self._lease, self._max_rpm = sink, lease, float(max_rpm)
        self._lock = threading.RLock()
        self._armed_token: str | None = None
        self._web_standby = False
        # Set only while a runtime-authorized recovery revokes an ordinary
        # lease.  The lease callback then queues zero output and publishes
        # no-motion standby instead of disarming.  It never grants a token.
        self._recoverable_revoke_reason: str | None = None
        self._fault_reason: str | None = None
        self._closing = False
        self._closed = False
        self.events: list[tuple[str, float, float, str]] = []
        sink.set_fault_callback(self._sink_fault)
        lease.set_revoke_callback(self.lease_revoked)

    @property
    def control_lease(self) -> ControlLease: return self._lease

    @property
    def armed(self) -> bool:
        with self._lock:
            return (self._armed_token is not None or self._web_standby) and self._fault_reason is None

    @property
    def web_standby_active(self) -> bool:
        with self._lock:
            return self._web_standby and not self._closing and not self._closed and self._fault_reason is None

    @property
    def fault_reason(self) -> str | None:
        with self._lock: return self._fault_reason

    def encoder_backend(self) -> SharedCanEncoderBackend:
        """Return a non-owning fresh-read adapter sharing this CAN worker."""
        return SharedCanEncoderBackend(self._sink)

    def arm(self, token: str | None) -> None:
        if not self._lease.valid(token):
            raise PhysicalOutputDisabled("giltig control-lease krävs före armering")
        # The verified worker sends 0x81 plus bounded 0x9C zero-speed settle.
        # No adapter/lease lock is held while waiting for its bounded result.
        try:
            self._sink.stop_and_settle_for_restart()
        except Exception as exc:
            self._latch_fault(f"startstopp före armering misslyckades: {exc}")
            raise MotorOutputFault(self._fault_reason or "startstopp misslyckades") from exc
        with self._lock:
            if self._closing or self._closed or self._fault_reason is not None or not self._lease.valid(token):
                raise PhysicalOutputDisabled("control-lease löpte ut under armering")
            self._armed_token = token
            self._web_standby = False
            self.events.append(("armed", 0.0, 0.0, "verifierat STOP+0x9C-settle"))

    def command(self, command: WheelCommand, token: str | None = None) -> None:
        if not isinstance(command, WheelCommand) or not all(math.isfinite(v) for v in (command.left_rpm, command.right_rpm)):
            self.stop_all("ogiltigt motorcommando")
            raise ValueError("ogiltigt motorcommando")
        active = token
        def admit() -> None:
            with self._lock:
                if self._closing or self._closed or self._fault_reason is not None or self._armed_token is None or active != self._armed_token:
                    raise PhysicalOutputDisabled("motorutgång är inte armerad för denna control-lease")
                left = max(-self._max_rpm, min(self._max_rpm, command.left_rpm))
                right = max(-self._max_rpm, min(self._max_rpm, command.right_rpm))
                self._sink.command(left, right, command.source)
                self.events.append(("drive", left, right, command.source))
        admitted = self._lease.run_if_valid_or_revoke(
                active, admit,
                lambda: self._prepare_recoverable_expiry(active, "CONTROL_LEASE_EXPIRED"))
        if admitted is None:
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")
        if not admitted:
            # A stale bearer token is not an expiry race.  Preserve the
            # established fail-closed treatment of an unauthorised command.
            self.stop_all("control-lease saknas eller har löpt ut")
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")

    def hold_stopped(self, reason: str, token: str | None = None) -> None:
        def admit() -> None:
            self._sink.stop_all(reason)
            self.events.append(("hold-stop", 0.0, 0.0, reason))
        admitted = self._lease.run_if_valid_or_revoke(
                token, admit,
                lambda: self._prepare_recoverable_expiry(token, "CONTROL_LEASE_EXPIRED"))
        if admitted is None:
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")
        if not admitted:
            self.stop_all(reason)
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")

    def refresh_lease_or_recover_expired(self, token: str | None,
                                          reason: str = "CONTROL_LEASE_EXPIRED") -> None:
        """Refresh active command authority without an ordinary-disarm gap.

        Lease expiry is checked and the recoverable boundary hand-off is
        prepared under the same lease lock.  A periodic control tick can
        therefore not revoke normally between expiry detection and runtime
        recovery publication.
        """
        if not self._lease.refresh_or_revoke(
                token, lambda: self._prepare_recoverable_expiry(token, reason)):
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")

    def enter_web_standby(self, token: str | None) -> None:
        """Atomically exchange a just-armed drive lease for no-motion standby."""
        with self._lock:
            if (self._closing or self._closed or self._fault_reason is not None
                    or token is None or token != self._armed_token):
                raise PhysicalOutputDisabled("motorutgången kan inte övergå till webb-standby")
        # Keep the global lock order lease -> boundary, matching command().
        # A STOP that wins after this release has already made the output safe;
        # the second boundary check below then rejects standby establishment.
        if not self._lease.release(token):
            raise PhysicalOutputDisabled("control-lease löpte ut före webb-standby")
        with self._lock:
            if (self._closing or self._closed or self._fault_reason is not None
                    or token != self._armed_token):
                raise PhysicalOutputDisabled("webbstandby avbröts av STOP eller fel")
            self._armed_token = None
            self._web_standby = True
            self.events.append(("web-standby", 0.0, 0.0, "lokal no-motion handoff"))

    def claim_web_standby(self, token: str | None) -> None:
        """Claim stopped standby using a newly acquired ordinary lease."""
        if not self._lease.valid(token):
            raise PhysicalOutputDisabled("webbstandby saknar ny control-lease")
        with self._lock:
            if (self._closing or self._closed or self._fault_reason is not None
                    or not self._web_standby):
                raise PhysicalOutputDisabled("webb-standby kan inte tas över")
            self._web_standby = False
            self._armed_token = token
            self.events.append(("web-standby-claimed", 0.0, 0.0, "ny control-lease"))

    def begin_wheel_position_move(self, *, left_wheel_degrees: float, right_wheel_degrees: float,
                                  max_motor_rpm: float, motor_turns_per_wheel_turn: float,
                                  tolerance_wheel_degrees: float, timeout_s: float,
                                  deadline_s: float,
                                  token: str | None = None) -> object:
        if (not isinstance(max_motor_rpm, (int, float)) or isinstance(max_motor_rpm, bool)
                or not math.isfinite(max_motor_rpm) or max_motor_rpm <= 0):
            raise ValueError("A4 max-rpm måste vara positiv och ändlig")
        max_motor_rpm = min(float(max_motor_rpm), self._max_rpm)
        request: object | None = None
        def admit() -> None:
            nonlocal request
            with self._lock:
                if self._closing or self._closed or self._fault_reason is not None or token != self._armed_token:
                    raise PhysicalOutputDisabled("motorutgång är inte armerad för A4-positionering")
                request = self._sink.begin_wheel_position_move(
                    left_wheel_degrees=left_wheel_degrees, right_wheel_degrees=right_wheel_degrees,
                    max_motor_rpm=max_motor_rpm, motor_turns_per_wheel_turn=motor_turns_per_wheel_turn,
                    tolerance_wheel_degrees=tolerance_wheel_degrees, timeout_s=timeout_s, deadline_s=deadline_s,
                )
                self.events.append(("position", left_wheel_degrees, right_wheel_degrees, "turn A4"))
        admitted = self._lease.run_if_valid_or_revoke(
                token, admit,
                lambda: self._prepare_recoverable_expiry(token, "CONTROL_LEASE_EXPIRED"))
        if admitted is None:
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")
        if not admitted:
            self.stop_all("A4-positionering saknar control-lease")
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")
        assert request is not None
        return request

    def position_move_status(self, request: object) -> tuple[bool, bool, str | None, bool]:
        status = self._sink.position_move_status(request)
        done, succeeded, error, active = (getattr(status, "done", None), getattr(status, "succeeded", None),
                                          getattr(status, "error", None), getattr(status, "active", None))
        if (not isinstance(done, bool) or not isinstance(succeeded, bool) or not isinstance(active, bool)
                or (error is not None and not isinstance(error, str))):
            raise MotorOutputFault("verifierad CAN-worker returnerade ogiltig A4-status")
        return done, succeeded, error, active

    def position_move_stage(self, request: object) -> tuple[bool, bool]:
        """Return worker-authored A4 acknowledgement/running evidence only.

        A queue claim is not sufficient for a STOP-under-A4 HIL assertion:
        this evidence becomes true only after both sequential A4 replies.
        """
        status = self._sink.position_move_status(request)
        acknowledged = getattr(status, "a4_targets_acknowledged", None)
        running = getattr(status, "target_running", None)
        if not isinstance(acknowledged, bool) or not isinstance(running, bool):
            raise MotorOutputFault("verifierad CAN-worker saknar A4-målstatus")
        return acknowledged, running

    def stop_and_settle_for_restart(self, reason: str = "STOP") -> None:
        """Public bounded STOP followed by verified 0x9C zero-speed settle."""
        self.stop_all(reason)
        try:
            self._sink.stop_and_settle_for_restart()
        except Exception as exc:
            self._latch_fault(f"STOP+0x9C-settle misslyckades: {exc}")
            raise MotorOutputFault(self._fault_reason or "STOP+0x9C-settle misslyckades") from exc

    def stop_and_settle_for_configuration_restart(self, reason: str = "CONFIGURATION_RESTART") -> None:
        """One bounded delayed re-check before restart is declared unsafe."""
        self.stop_all(reason)
        last_error: Exception | None = None
        for attempt in range(2):
            try:
                self._sink.stop_and_settle_for_restart()
                return
            except Exception as exc:
                last_error = exc
                if attempt == 0:
                    time.sleep(.075)
        self._latch_fault(f"konfigurationsomstart STOP+0x9C-settle misslyckades: {last_error}")
        raise MotorOutputFault(self._fault_reason or "konfigurationsomstart STOP+0x9C-settle misslyckades") from last_error

    def stop_all(self, reason: str) -> None:
        with self._lock:
            self._armed_token = None
            self._web_standby = False
            if self._closing or self._closed:
                return
        if not self._lease.revoke_any():
            self._sink.stop_all(reason)
        self.events.append(("stop", 0.0, 0.0, reason))

    def lease_revoked(self) -> None:
        with self._lock:
            recovery_reason = self._recoverable_revoke_reason
            self._recoverable_revoke_reason = None
            if (recovery_reason is not None and not self._closing and not self._closed
                    and self._fault_reason is None):
                self._armed_token = None
                self._web_standby = True
                recoverable = True
            else:
                recoverable = False
                self._armed_token = None
            closing = self._closing or self._closed
        if recoverable:
            # Lease revocation linearizes after any admitted A2 command.  This
            # queue-only STOP is therefore the next physical operation and no
            # fresh command can be admitted without a new token.
            self._sink.stop_all(recovery_reason)
            self.events.append(("recoverable-stop", 0.0, 0.0, recovery_reason))
            return
        # close() issues the stronger verified STOP+0x9C settle itself.  Do not
        # enqueue a second best-effort stop for a revocation it initiated.
        if not closing:
            self._sink.stop_all("control-lease återkallades")

    def recoverable_stop_to_web_standby(self, reason: str) -> bool:
        """Revoke authority into stopped, armed web standby.

        The runtime invokes this only for operator-recoverable conditions.
        Revocation remains the linearization point: it waits behind any
        already-admitted command, then ``lease_revoked`` queues zero output
        and leaves only no-motion standby.  A worker/boundary fault, close,
        or a competing STOP rejects this path and retains fail-closed output.
        """
        if not isinstance(reason, str) or not reason:
            raise ValueError("återställningsstopp kräver orsak")
        with self._lock:
            if self._closing or self._closed or self._fault_reason is not None:
                return False
            if self._web_standby and self._armed_token is None:
                # Already no-motion and tokenless: an explicit repeated STOP
                # remains a physical zero request but must not fall through
                # to stop_all/disarm merely because there is no lease left to
                # revoke.
                already_standby = True
            elif self._armed_token is None:
                return False
            else:
                already_standby = False
            if already_standby:
                self._sink.stop_all(reason)
                self.events.append(("recoverable-stop", 0.0, 0.0, reason))
                return True
            self._recoverable_revoke_reason = reason
        if self._lease.revoke_any():
            with self._lock:
                return self._web_standby and self._fault_reason is None
        with self._lock:
            self._recoverable_revoke_reason = None
        return False

    def _prepare_recoverable_expiry(self, token: str | None, reason: str) -> None:
        """Mark the *currently expiring* active lease as recoverable.

        This executes under ``ControlLease``'s lock.  The lock ordering is
        lease then boundary, matching command admission.  A fault or close
        deliberately leaves the callback on its ordinary fail-closed path.
        """
        with self._lock:
            if (self._closing or self._closed or self._fault_reason is not None
                    or token is None or token != self._armed_token):
                return
            self._recoverable_revoke_reason = reason

    def close(self) -> None:
        if not self._begin_close():
            return
        self._finish_close()

    def _begin_close(self) -> bool:
        """Claim shutdown ownership without waiting on the CAN worker.

        FieldControlRuntime calls this private handshake while holding its
        lifecycle gate, then invokes _finish_close after releasing that gate.
        """
        with self._lock:
            if self._closing or self._closed:
                return False
            self._closing = True
            self._armed_token = None
            self._web_standby = False
            return True

    def _finish_close(self) -> None:
        # Revoke admission before waiting for the verified worker.  Its
        # callback observes _closing and leaves the single shutdown settle
        # below responsible for the physical STOP sequence.
        self._lease.revoke_any()
        failure: Exception | None = None
        try:
            # The verified worker claims shutdown *before* cancelling any
            # 0x92 pair, then performs bounded STOP+0x9C settle and releases
            # its socket/lock.  Do not compose restart-settle plus close here:
            # that would expose a retryable encoder preemption during close.
            self._sink.stop_and_settle_and_close()
        except Exception as exc:
            failure = exc
            self._latch_fault(f"shutdown-STOP+0x9C-settle misslyckades: {exc}")
        finally:
            with self._lock:
                self._closed = True

        if failure is not None:
            raise MotorOutputFault(self.fault_reason or "fysisk motorstängning misslyckades") from failure

    def _sink_fault(self, reason: str) -> None:
        self._latch_fault(f"verifierad CAN-worker: {reason}")
        self._lease.revoke_any()

    def _latch_fault(self, reason: str) -> None:
        with self._lock:
            if self._fault_reason is None:
                self._fault_reason = reason
            self._armed_token = None
            self._web_standby = False


def open_verified_boundary(*, channel: str, slcan_device: str, max_rpm: float,
                          lease: ControlLease) -> MotorBoundary:
    """Construct the verified worker only after deployment config was validated."""
    if not isinstance(max_rpm, (int, float)) or isinstance(max_rpm, bool) or not math.isfinite(max_rpm) or not 0 < max_rpm <= VERIFIED_MAX_MOTOR_RPM:
        raise ValueError("max_rpm måste vara ändligt och inom verifierad gräns")
    try:
        from remote_control.config import ControlConfig
        from remote_control.physical import PhysicalCanMotors
    except ImportError as exc:
        raise RuntimeError("remote_control måste installeras för fysisk CAN-output") from exc
    config = ControlConfig(can_interface=channel, slcan_device=slcan_device, max_rpm=max_rpm)
    sink = PhysicalCanMotors.open_for_raised_wheel_test(config)
    if not isinstance(sink, PhysicalCanMotors):
        try: sink.close()
        except Exception: pass
        raise RuntimeError("verifierad CAN-öppning returnerade fel sink-typ")
    try:
        return _VerifiedPhysicalMotorBoundary(sink, lease, max_rpm=max_rpm)
    except Exception:
        try:
            sink.close()
        except Exception:
            pass
        raise
