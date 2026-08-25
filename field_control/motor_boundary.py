"""Fail-closed, explicitly armed RMD-X V3.8 motor boundary.

Importing this module and constructing :class:`PhysicalMotorBoundary` never
opens SocketCAN. A caller must separately construct a transport through the
explicit ``SocketCanV38Transport.open`` factory, and must successfully acquire
and arm a control lease before any non-zero frame can be sent.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import fcntl
import math
from pathlib import Path
import threading
import time
from typing import Callable, Protocol

from .control import WheelCommand

LEFT_ID = 0x141
RIGHT_ID = 0x142
CAN_BITRATE = 1_000_000
DEFAULT_REPLY_TIMEOUT_S = 0.10
DEFAULT_COMMAND_DEADLINE_S = 0.25
# This is deliberately shared with the already deployed motor applications.
# A different lock name would permit two independent processes to command can0.
CAN_LOCK_PATH = Path("/run/lock/can0-motor-control.lock")
VERIFIED_MAX_WHEEL_RPM = 80.0


class MotorTransportError(RuntimeError):
    """A bounded CAN transaction did not complete safely."""


class PhysicalOutputDisabled(RuntimeError):
    pass


class MotorOutputFault(RuntimeError):
    """Output was disabled after a failed or unauthorised command."""


class MotorBoundary(Protocol):
    def stop_all(self, reason: str) -> None: ...
    def command(self, command: WheelCommand, token: str | None = None) -> None: ...


class LeasedControl(Protocol):
    """Small subset of the verified expiring control lease used here."""
    def valid(self, token: str | None) -> bool: ...
    def run_if_valid(self, token: str | None, operation: Callable[[], object]) -> bool: ...
    def set_revoke_callback(self, callback: Callable[[], None]) -> None: ...
    def revoke_any(self) -> bool: ...


class V38Transport(Protocol):
    def set_speed_dps(self, motor_id: int, speed_dps: float, deadline_s: float) -> None: ...
    def stop_pair_acknowledged(self, deadline_s: float) -> None: ...
    def best_effort_stop_pair(self) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class MotorDirections:
    """Verified robot-forward signs: left +, right -."""
    left_forward_sign: int = 1
    right_forward_sign: int = -1

    def validate(self) -> "MotorDirections":
        if self.left_forward_sign not in (-1, 1) or self.right_forward_sign not in (-1, 1):
            raise ValueError("motorriktning måste vara -1 eller +1")
        return self


def _validated_max_rpm(value: float) -> float:
    """Return a per-deployment limit that cannot exceed the approved bound."""
    if not math.isfinite(value) or not 0 < value <= VERIFIED_MAX_WHEEL_RPM:
        raise ValueError(
            f"max_rpm måste vara ändligt och inom 0–{VERIFIED_MAX_WHEEL_RPM:g} rpm",
        )
    return float(value)


@dataclass
class DisabledMotorBoundary:
    """Default boundary: records stops and refuses every command."""
    events: list[tuple[str, float, float, str]] = field(default_factory=list)

    def stop_all(self, reason: str) -> None:
        self.events.append(("stop", 0.0, 0.0, reason))

    def command(self, command: WheelCommand, token: str | None = None) -> None:
        del token
        self.stop_all("fysisk motorutgång är avstängd")
        raise PhysicalOutputDisabled("field_control har ingen armerad fysisk CAN-motorutgång")


class PhysicalMotorBoundary:
    """Lease-gated wheel-RPM adapter with fail-closed fault handling.

    The boundary never opens CAN itself: ``transport`` must have been made by
    an explicit, deployment-level action. Commands are linearised through the
    lease, so a lease revocation cannot race a new A2 transmission.
    """
    def __init__(self, transport: V38Transport, lease: LeasedControl, *, max_rpm: float,
                 directions: MotorDirections = MotorDirections(),
                 command_deadline_s: float = DEFAULT_COMMAND_DEADLINE_S) -> None:
        if transport is None or lease is None:
            raise ValueError("transport och control lease krävs")
        if not math.isfinite(command_deadline_s) or not 0 < command_deadline_s <= 1.0:
            raise ValueError("kommandodeadline måste vara 0–1 sekund")
        self._transport, self._lease = transport, lease
        self._max_rpm = _validated_max_rpm(max_rpm)
        self._directions = directions.validate()
        self._command_deadline_s = float(command_deadline_s)
        self._lock = threading.RLock()
        self._armed_token: str | None = None
        self._fault_reason: str | None = None
        self.events: list[tuple[str, float, float, str]] = []
        self._lease.set_revoke_callback(self.lease_revoked)

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed_token is not None and self._fault_reason is None

    @property
    def fault_reason(self) -> str | None:
        with self._lock:
            return self._fault_reason

    def arm(self, token: str | None) -> None:
        """Stop both motors with acknowledgement before accepting drive output.

        This makes an explicit arm a safe transition after process startup or
        recovery.  A failed pre-arm stop is latched and cannot be bypassed by
        a later arm request.
        """
        with self._lock:
            if self._fault_reason is not None:
                raise MotorOutputFault(f"motorutgång är låst: {self._fault_reason}")
            if not self._lease.valid(token):
                raise PhysicalOutputDisabled("giltig control-lease krävs före armering")
            try:
                deadline = time.monotonic() + self._command_deadline_s
                if not self._lease.run_if_valid(
                    token, lambda: self._transport.stop_pair_acknowledged(deadline),
                ):
                    raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut före armering")
            except PhysicalOutputDisabled:
                self._stop_locked("control-lease saknas eller har löpt ut före armering")
                raise
            except Exception as exc:
                self._fault_reason = f"startstopp före armering misslyckades: {exc}"
                try:
                    self._transport.best_effort_stop_pair()
                except Exception:
                    pass
                raise MotorOutputFault(self._fault_reason) from exc
            self._armed_token = token
            self.events.append(("armed", 0.0, 0.0, "kvitterat startstopp och explicit armering"))

    def command(self, command: WheelCommand, token: str | None = None) -> None:
        """Send one bounded A2 pair only while the exact armed lease is live."""
        if not isinstance(command, WheelCommand):
            self.stop_all("ogiltigt motorcommando")
            raise ValueError("command måste vara WheelCommand")
        if not all(math.isfinite(value) for value in (command.left_rpm, command.right_rpm)):
            self.stop_all("icke-ändligt motorcommando")
            raise ValueError("hjulhastigheter måste vara ändliga")
        with self._lock:
            active_token = token if token is not None else self._armed_token
            if self._fault_reason is not None:
                raise MotorOutputFault(f"motorutgång är låst: {self._fault_reason}")
            if self._armed_token is None or active_token != self._armed_token:
                self._stop_locked("saknad eller felaktig control-lease")
                raise PhysicalOutputDisabled("motorutgång är inte armerad för denna control-lease")
            if not self._lease.run_if_valid(active_token, lambda: self._send_locked(command)):
                self._stop_locked("control-lease saknas eller har löpt ut")
                raise PhysicalOutputDisabled("control-lease saknas eller har löpt ut")

    def stop_all(self, reason: str) -> None:
        """Emergency stop: revoke control first, then stop both motors.

        The verified lease invokes ``lease_revoked`` synchronously when active,
        which prevents a later command from being admitted with this bearer.
        The fallback keeps the boundary usable with a minimal test/dry-run lease.
        """
        revoke_any = getattr(self._lease, "revoke_any", None)
        if callable(revoke_any):
            try:
                if revoke_any():
                    return
            except Exception:
                # A failed lease implementation must not prevent a physical stop.
                pass
        with self._lock:
            self._stop_locked(reason)

    def lease_revoked(self) -> None:
        """Lease watchdog callback. Safe to call repeatedly."""
        with self._lock:
            self._stop_locked("control-lease återkallades")

    def close(self) -> None:
        with self._lock:
            try:
                self._stop_locked("motorgränsen stängs")
            finally:
                self._transport.close()

    def _send_locked(self, command: WheelCommand) -> None:
        left_rpm = max(-self._max_rpm, min(self._max_rpm, float(command.left_rpm)))
        right_rpm = max(-self._max_rpm, min(self._max_rpm, float(command.right_rpm)))
        deadline = time.monotonic() + self._command_deadline_s
        try:
            # A2 uses motor-side degree/s. The 8:1 ratio belongs to odometry,
            # never to the motor command. Directions are physical CAN signs.
            self._transport.set_speed_dps(LEFT_ID, self._directions.left_forward_sign * left_rpm * 6.0, deadline)
            self._transport.set_speed_dps(RIGHT_ID, self._directions.right_forward_sign * right_rpm * 6.0, deadline)
            self.events.append(("drive", left_rpm, right_rpm, command.source))
        except Exception as exc:
            self._fault_reason = f"CAN-motorcommando misslyckades: {exc}"
            self._stop_locked(self._fault_reason)
            raise MotorOutputFault(self._fault_reason) from exc

    def _stop_locked(self, reason: str) -> None:
        self._armed_token = None
        deadline = time.monotonic() + self._command_deadline_s
        try:
            self._transport.stop_pair_acknowledged(deadline)
            self.events.append(("stop", 0.0, 0.0, reason))
        except Exception as exc:
            self._fault_reason = f"stopp misslyckades: {exc}"
            self.events.append(("stop-fault", 0.0, 0.0, self._fault_reason))
            try:
                self._transport.best_effort_stop_pair()
            except Exception:
                pass


@dataclass(frozen=True)
class MotorReplyProfile:
    """Explicit reply identity; it is never inferred from received frames."""
    name: str
    reply_id_by_request_id: dict[int, int]

    def reply_id(self, motor_id: int) -> int:
        try:
            return self.reply_id_by_request_id[motor_id]
        except KeyError as exc:
            raise ValueError("okänt motor-ID") from exc


V38_STANDARD_REPLY_PROFILE = MotorReplyProfile(
    "v3.8-standard-plus-0x100", {LEFT_ID: LEFT_ID + 0x100, RIGHT_ID: RIGHT_ID + 0x100},
)
# This profile is a named observation from the installed RMD-X motors.  It is
# intentionally not a fallback from the documented V3.8 profile.
OBSERVED_RMDX_SAME_ID_REPLY_PROFILE = MotorReplyProfile(
    "observed-rmdx-same-id", {LEFT_ID: LEFT_ID, RIGHT_ID: RIGHT_ID},
)
MOTOR_REPLY_PROFILES = {
    V38_STANDARD_REPLY_PROFILE.name: V38_STANDARD_REPLY_PROFILE,
    OBSERVED_RMDX_SAME_ID_REPLY_PROFILE.name: OBSERVED_RMDX_SAME_ID_REPLY_PROFILE,
}


def get_motor_reply_profile(name: str | None) -> MotorReplyProfile:
    if name is None:
        raise ValueError("en explicit motorprofil krävs före CAN-öppning")
    try:
        return MOTOR_REPLY_PROFILES[name]
    except KeyError as exc:
        raise ValueError("okänd motorprofil") from exc


def v38_speed_frame(motor_id: int, speed_dps: float) -> bytes:
    """Make strict V3.8 A2 frame: signed int32 in 0.01 degree/s."""
    if motor_id not in (LEFT_ID, RIGHT_ID):
        raise ValueError("okänt motor-ID")
    if not isinstance(speed_dps, (int, float)) or not math.isfinite(speed_dps):
        raise ValueError("ogiltig motorhastighet")
    if not -21_474_836.48 <= speed_dps <= 21_474_836.47:
        raise ValueError("motorhastighet utanför V3.8 int32-intervall")
    return bytes((0xA2, 0, 0, 0)) + int(round(speed_dps * 100)).to_bytes(4, "little", signed=True)


class SocketCanV38Transport:
    """Strict bounded SocketCAN. It opens only through explicit ``open``."""
    def __init__(self, bus: object, *, profile: MotorReplyProfile,
                 reply_timeout_s: float = DEFAULT_REPLY_TIMEOUT_S,
                 can_module: object | None = None, lock_file: object | None = None) -> None:
        if not 0 < reply_timeout_s <= 1.0:
            raise ValueError("CAN-timeout måste vara 0–1 sekund")
        if MOTOR_REPLY_PROFILES.get(profile.name) != profile:
            raise ValueError("en namngiven, godkänd motorprofil krävs")
        self._bus, self._profile = bus, profile
        self._timeout_s, self._can, self._lock_file = reply_timeout_s, can_module, lock_file
        self._lock = threading.RLock()

    @classmethod
    def open(cls, *, channel: str, profile: MotorReplyProfile,
             reply_timeout_s: float = DEFAULT_REPLY_TIMEOUT_S) -> "SocketCanV38Transport":
        """Explicit production-only CAN opening; there is no automatic fallback."""
        if not channel:
            raise ValueError("CAN-kanal krävs")
        lock_file = None
        try:
            CAN_LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
            lock_file = open(CAN_LOCK_PATH, "a+")
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            import can  # lazy: dry-run/UI imports need no python-can
            bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=CAN_BITRATE,
                                    receive_own_messages=False)
            return cls(bus, profile=profile, reply_timeout_s=reply_timeout_s, can_module=can, lock_file=lock_file)
        except Exception as exc:
            if lock_file is not None:
                try:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
                    lock_file.close()
                except Exception:
                    pass
            raise MotorTransportError(f"kunde inte öppna {channel}: {exc}") from exc

    def close(self) -> None:
        with self._lock:
            if self._bus is not None:
                try: self._bus.shutdown()
                except Exception: pass
                self._bus = None
            if self._lock_file is not None:
                try:
                    fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
                    self._lock_file.close()
                finally:
                    self._lock_file = None

    def set_speed_dps(self, motor_id: int, speed_dps: float, deadline_s: float) -> None:
        self._request(motor_id, v38_speed_frame(motor_id, speed_dps), 0xA2, deadline_s)

    def stop_pair_acknowledged(self, deadline_s: float) -> None:
        """Transmit both STOPs before waiting for either acknowledgement.

        Sending the second STOP must never be delayed by a missing response to
        the first motor.  Replies may arrive in either order, so they are
        collected as a pair under one serialized bus transaction.
        """
        with self._lock:
            deadline = self._bounded_deadline(deadline_s)
            self._drain_stale_replies_unlocked()
            sent_at = {
                LEFT_ID: self._send_unlocked(LEFT_ID, bytes((0x81,)) + bytes(7), deadline),
                RIGHT_ID: self._send_unlocked(RIGHT_ID, bytes((0x81,)) + bytes(7), deadline),
            }
            self._wait_for_replies_unlocked(
                {LEFT_ID: 0x81, RIGHT_ID: 0x81}, deadline, sent_at,
            )

    def best_effort_stop_pair(self) -> None:
        # Best effort must preserve the same "both frames first" property.
        with self._lock:
            if self._bus is None:
                return
            deadline = time.monotonic() + self._timeout_s
            try:
                self._drain_stale_replies_unlocked()
                self._send_unlocked(LEFT_ID, bytes((0x81,)) + bytes(7), deadline)
                self._send_unlocked(RIGHT_ID, bytes((0x81,)) + bytes(7), deadline)
            except MotorTransportError:
                pass

    def _request(self, motor_id: int, payload: bytes, command: int, deadline_s: float) -> bytes:
        with self._lock:
            deadline = self._bounded_deadline(deadline_s)
            # Responses do not contain a transaction nonce.  Discard every
            # queued frame before transmission so an old response can never
            # authorise this new request.  The SocketCAN channel is also held
            # by the shared inter-process lock and this object lock.
            self._drain_stale_replies_unlocked()
            sent_at = self._send_unlocked(motor_id, payload, deadline)
            return self._wait_for_replies_unlocked(
                {motor_id: command}, deadline, {motor_id: sent_at},
            )[motor_id]

    def _bounded_deadline(self, deadline_s: float) -> float:
        if self._bus is None:
            raise MotorTransportError("CAN är stängd")
        deadline = min(deadline_s, time.monotonic() + self._timeout_s)
        if deadline <= time.monotonic():
            raise MotorTransportError("CAN-tidsbudget har löpt ut")
        return deadline

    def _make_message(self, motor_id: int, payload: bytes) -> object:
        return (self._can.Message(arbitration_id=motor_id, data=payload, is_extended_id=False)
                if self._can is not None else type("Message", (), {
                    "arbitration_id": motor_id, "data": payload,
                })())

    def _send_unlocked(self, motor_id: int, payload: bytes, deadline: float) -> float:
        if time.monotonic() >= deadline:
            raise MotorTransportError("CAN-tidsbudget har löpt ut före sändning")
        try:
            # SocketCAN timestamps use the wall clock, unlike all deadlines in
            # this module.  Record it only to reject replies received before
            # this individual frame was submitted.
            sent_at = time.time()
            self._bus.send(self._make_message(motor_id, payload), timeout=max(.001, deadline - time.monotonic()))
            return sent_at
        except MotorTransportError:
            raise
        except Exception as exc:
            raise MotorTransportError(f"CAN-sändning för 0x{motor_id:03X} misslyckades: {exc}") from exc

    def _drain_stale_replies_unlocked(self) -> None:
        """Discard frames already queued before a new request is transmitted."""
        try:
            while self._bus.recv(timeout=0) is not None:
                pass
        except Exception as exc:
            raise MotorTransportError(f"kunde inte tömma gamla CAN-svar: {exc}") from exc

    def _wait_for_replies_unlocked(self, expected: dict[int, int], deadline: float,
                                   sent_at: dict[int, float]) -> dict[int, bytes]:
        pending = dict(expected)
        received: dict[int, bytes] = {}
        while pending and (remaining := deadline - time.monotonic()) > 0:
            try:
                reply = self._bus.recv(timeout=remaining)
            except Exception as exc:
                raise MotorTransportError(f"CAN-mottagning misslyckades: {exc}") from exc
            if reply is None:
                break
            data = bytes(reply.data)
            for motor_id, command in tuple(pending.items()):
                if (reply.arbitration_id == self._profile.reply_id(motor_id)
                        and self._valid_reply(data, command)
                        and self._reply_is_not_stale(reply, sent_at[motor_id])):
                    received[motor_id] = data
                    del pending[motor_id]
                    break
        if pending:
            ids = ", ".join(f"0x{motor_id:03X}" for motor_id in pending)
            raise MotorTransportError(f"CAN-svarstimeout för motor {ids}")
        return received

    def _reply_is_not_stale(self, reply: object, sent_at: float) -> bool:
        """Use SocketCAN's receive timestamp as a transaction freshness fence.

        V3.8 replies have no request nonce.  A pre-send non-blocking drain
        removes queued frames; on a real python-can SocketCAN bus the receive
        timestamp provides the remaining fence against a late old reply.  If
        that timestamp is unavailable in a test/dry-run transport, only the
        drain is available and no physical transport can be opened through it.
        """
        timestamp = getattr(reply, "timestamp", None)
        if timestamp is None:
            return self._can is None
        return isinstance(timestamp, (int, float)) and math.isfinite(timestamp) and timestamp >= sent_at

    @staticmethod
    def _valid_reply(data: bytes, command: int) -> bool:
        return (len(data) == 8 and data[0] == command
                and (command != 0x81 or data == bytes((0x81,)) + bytes(7)))
