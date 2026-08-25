"""Pure turn geometry derived from the verified OAK navigation implementation."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from .odometry import DriveGeometry


@dataclass(frozen=True)
class TurnPlan:
    left_distance_m: float
    right_distance_m: float
    speed_rpm: float
    direction: Literal["left", "right"]


def new_row_turn_plan(
    geometry: DriveGeometry, row_spacing_m: float, speed_rpm: float,
    direction: Literal["left", "right"] = "right", inner_wheel_min_ratio: float = 0.0,
) -> TurnPlan:
    """Return signed wheel travel for a 180-degree turn to the next row."""
    geometry.validate()
    if not math.isfinite(row_spacing_m) or row_spacing_m <= geometry.wheel_track_m:
        raise ValueError("row_spacing_m måste vara större än wheel_track_m")
    if not math.isfinite(speed_rpm) or speed_rpm <= 0:
        raise ValueError("turn_speed_rpm måste vara positiv")
    if direction not in ("left", "right"):
        raise ValueError("turn direction måste vara left eller right")
    if not math.isfinite(inner_wheel_min_ratio) or not 0 <= inner_wheel_min_ratio <= 1:
        raise ValueError("inner_wheel_min_ratio måste ligga mellan 0 och 1")
    inner_radius = (row_spacing_m - geometry.wheel_track_m) / 2.0
    outer_radius = (row_spacing_m + geometry.wheel_track_m) / 2.0
    inner = math.pi * inner_radius
    outer = math.pi * outer_radius
    if inner < outer * inner_wheel_min_ratio:
        raise ValueError("inner-wheel turn är under minsta tillåtna kvot")
    inner_distance = inner if direction == "left" else outer
    outer_distance = outer if direction == "left" else inner
    return TurnPlan(inner_distance, outer_distance, speed_rpm, direction)