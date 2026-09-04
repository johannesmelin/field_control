"""Non-operational motor boundary primitives and verified protocol constants.

Physical CAN I/O intentionally does not exist in this module. The only
production physical path is the private adapter in ``verified_motor_boundary``
which delegates to ``remote_control.physical.PhysicalCanMotors``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Protocol

from .control import WheelCommand

LEFT_ID = 0x141
RIGHT_ID = 0x142
CAN_BITRATE = 1_000_000
# Motor-side command cap, before the configured gearbox ratio.  Raw A2 values
# remain motor RPM; no gearbox conversion belongs in this boundary.
VERIFIED_MAX_MOTOR_RPM = 80.0


class PhysicalOutputDisabled(RuntimeError): pass
class MotorOutputFault(RuntimeError): pass


class MotorBoundary(Protocol):
    def stop_all(self, reason: str) -> None: ...
    def hold_stopped(self, reason: str, token: str | None = None) -> None: ...
    def command(self, command: WheelCommand, token: str | None = None) -> None: ...


@dataclass
class DisabledMotorBoundary:
    events: list[tuple[str, float, float, str]] = field(default_factory=list)

    def stop_all(self, reason: str) -> None:
        self.events.append(("stop", 0.0, 0.0, reason))

    def hold_stopped(self, reason: str, token: str | None = None) -> None:
        del token
        self.stop_all(reason)

    def command(self, command: WheelCommand, token: str | None = None) -> None:
        del command, token
        self.stop_all("fysisk motorutgång är avstängd")
        raise PhysicalOutputDisabled("field_control har ingen armerad fysisk CAN-motorutgång")


@dataclass(frozen=True)
class MotorReplyProfile:
    name: str
    reply_id_by_request_id: dict[int, int]

    def reply_id(self, motor_id: int) -> int:
        try: return self.reply_id_by_request_id[motor_id]
        except KeyError as exc: raise ValueError("okänt motor-ID") from exc


V38_STANDARD_REPLY_PROFILE = MotorReplyProfile(
    "v3.8-standard-plus-0x100", {LEFT_ID: LEFT_ID + 0x100, RIGHT_ID: RIGHT_ID + 0x100},
)
OBSERVED_RMDX_SAME_ID_REPLY_PROFILE = MotorReplyProfile(
    "observed-rmdx-same-id", {LEFT_ID: LEFT_ID, RIGHT_ID: RIGHT_ID},
)
MOTOR_REPLY_PROFILES = {
    V38_STANDARD_REPLY_PROFILE.name: V38_STANDARD_REPLY_PROFILE,
    OBSERVED_RMDX_SAME_ID_REPLY_PROFILE.name: OBSERVED_RMDX_SAME_ID_REPLY_PROFILE,
}


def get_motor_reply_profile(name: str | None) -> MotorReplyProfile:
    if name is None: raise ValueError("en explicit motorprofil krävs")
    try: return MOTOR_REPLY_PROFILES[name]
    except KeyError as exc: raise ValueError("okänd motorprofil") from exc


def v38_speed_frame(motor_id: int, speed_dps: float) -> bytes:
    """Pure V3.8 A2 encoder: signed int32 in 0.01 degree/s."""
    if motor_id not in (LEFT_ID, RIGHT_ID): raise ValueError("okänt motor-ID")
    if not isinstance(speed_dps, (int, float)) or not math.isfinite(speed_dps):
        raise ValueError("ogiltig motorhastighet")
    if not -21_474_836.48 <= speed_dps <= 21_474_836.47:
        raise ValueError("motorhastighet utanför V3.8 int32-intervall")
    return bytes((0xA2, 0, 0, 0)) + int(round(speed_dps * 100)).to_bytes(4, "little", signed=True)
