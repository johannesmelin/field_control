"""Pure differential-drive odometry using verified RMD-X6 8:1 semantics."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DriveGeometry:
    left_wheel_circumference_m: float = .805
    right_wheel_circumference_m: float = .805
    wheel_track_m: float = 1.01
    motor_turns_per_wheel_turn: float = 8.0
    left_forward_sign: int = 1
    right_forward_sign: int = -1

    def validate(self) -> "DriveGeometry":
        if self.left_forward_sign not in (-1, 1) or self.right_forward_sign not in (-1, 1):
            raise ValueError("hjulens framåttecken måste vara -1 eller +1")
        if not all(math.isfinite(value) and value > 0 for value in (
            self.left_wheel_circumference_m, self.right_wheel_circumference_m,
            self.wheel_track_m, self.motor_turns_per_wheel_turn,
        )):
            raise ValueError("robotgeometrin måste vara ändlig och positiv")
        return self


def wheel_distance_m(initial_deg: float, current_deg: float, circumference_m: float,
                     motor_turns_per_wheel_turn: float, forward_sign: int) -> float:
    if forward_sign not in (-1, 1) or not all(math.isfinite(value) for value in
                                               (initial_deg, current_deg, circumference_m, motor_turns_per_wheel_turn)):
        raise ValueError("ogiltig odometridata")
    return forward_sign * (current_deg - initial_deg) / 360.0 / motor_turns_per_wheel_turn * circumference_m


@dataclass(frozen=True)
class OdometrySample:
    left_distance_m: float
    right_distance_m: float
    forward_distance_m: float
    yaw_change_deg: float


def from_motor_angles(initial_left_deg: float, initial_right_deg: float,
                      current_left_deg: float, current_right_deg: float,
                      geometry: DriveGeometry) -> OdometrySample:
    geometry.validate()
    left = wheel_distance_m(initial_left_deg, current_left_deg, geometry.left_wheel_circumference_m,
                            geometry.motor_turns_per_wheel_turn, geometry.left_forward_sign)
    right = wheel_distance_m(initial_right_deg, current_right_deg, geometry.right_wheel_circumference_m,
                             geometry.motor_turns_per_wheel_turn, geometry.right_forward_sign)
    return OdometrySample(left, right, (left + right) / 2.0,
                           math.degrees((right - left) / geometry.wheel_track_m))
