"""Lease-gated adapter for the verified remote_control physical CAN worker.

This module deliberately contains no CAN framing, socket handling, or worker.
Those safety-critical functions remain owned by ``remote_control.physical``.
"""
from __future__ import annotations

import math
import threading
from typing import Callable, Protocol

from .control import WheelCommand
from .lease import ControlLease
from .motor_boundary import MotorBoundary, MotorOutputFault, PhysicalOutputDisabled, VERIFIED_MAX_MOTOR_RPM
from .sources import EncoderReadPreempted


class VerifiedMotorSink(Protocol):
    def command(self, left_rpm: float, right_rpm: float, reason: str) -> None: ...
    def stop_all(self, reason: str) -> None: ...
    def stop_and_settle_for_restart(self) -> None: ...
    def stop_and_settle_and_close(self) -> None: ...
    def close(self) -> None: ...
    def set_fault_callback(self, callback: Callable[[str], None]) -> None: ...
    def read_multi_turn_angles(self) -> tuple[float, float]: ...


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
        # Do not release this gate until the sink has either accepted and
        # completed the bounded request or rejected it.  ``begin_shutdown``
        # shares this exact gate, making its return the linearization point
        # after which no new sink read can be admitted.
        with self._admission_lock:
            if self._shutdown.is_set():
                raise RuntimeError("delad encoderadapter är stängd")
            try:
                angles = self._sink.read_multi_turn_angles()
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
                raise
        if (not isinstance(angles, tuple) or len(angles) != 2
                or not all(isinstance(item, (int, float)) and not isinstance(item, bool)
                           and math.isfinite(item) for item in angles)):
            raise MotorOutputFault("verifierad CAN-worker returnerade ogiltig flervarvsvinkel")
        return float(angles[0]), float(angles[1])

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
        with self._lock: return self._armed_token is not None and self._fault_reason is None

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
        if not self._lease.run_if_valid(active, admit):
            self.stop_all("control-lease saknas eller har löpt ut")
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")

    def hold_stopped(self, reason: str, token: str | None = None) -> None:
        if not self._lease.valid(token):
            self.stop_all(reason)
            raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")
        self._sink.stop_all(reason)
        self.events.append(("hold-stop", 0.0, 0.0, reason))

    def stop_all(self, reason: str) -> None:
        with self._lock:
            self._armed_token = None
            if self._closing or self._closed:
                return
        if not self._lease.revoke_any():
            self._sink.stop_all(reason)
        self.events.append(("stop", 0.0, 0.0, reason))

    def lease_revoked(self) -> None:
        with self._lock:
            self._armed_token = None
            closing = self._closing or self._closed
        # close() issues the stronger verified STOP+0x9C settle itself.  Do not
        # enqueue a second best-effort stop for a revocation it initiated.
        if not closing:
            self._sink.stop_all("control-lease återkallades")

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
