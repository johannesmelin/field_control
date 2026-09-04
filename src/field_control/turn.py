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


@dataclass(frozen=True)
class DifferentialTurnPlan:
    """Signed wheel-distance targets and motor ratios normalized from wheel turns."""
    left_distance_m: float
    right_distance_m: float
    left_ratio: float
    right_ratio: float
    direction: Literal["left", "right"]


@dataclass(frozen=True)
class AbsolutePositionTurn:
    """Wheel-angle increments for an encoder-targeted differential turn.

    Values are logical wheel degrees.  The verified CAN boundary converts
    them to motor-shaft angles using ``DriveGeometry`` and applies hardware
    signs once, after it has read fresh 0x92 starting angles.
    """
    left_wheel_degrees: float
    right_wheel_degrees: float
    direction: Literal["left", "right"]


def absolute_position_turn(plan: DifferentialTurnPlan, geometry: DriveGeometry) -> AbsolutePositionTurn:
    """Convert geometry-derived signed travel to exact logical wheel angles."""
    geometry.validate()
    values = (plan.left_distance_m, plan.right_distance_m)
    if not all(math.isfinite(value) and value != 0 for value in values):
        raise ValueError("turn-plan targets måste vara ändliga och icke-noll")
    return AbsolutePositionTurn(
        left_wheel_degrees=plan.left_distance_m / geometry.left_wheel_circumference_m * 360.0,
        right_wheel_degrees=plan.right_distance_m / geometry.right_wheel_circumference_m * 360.0,
        direction=plan.direction,
    )


def _ratios(left_wheel_turns: float, right_wheel_turns: float) -> tuple[float, float]:
    """Normalize signed wheel turns; motor-side common 8:1 cancels out."""
    maximum = max(abs(left_wheel_turns), abs(right_wheel_turns))
    if maximum <= 0: raise ValueError("turn targets måste vara icke-noll")
    return left_wheel_turns / maximum, right_wheel_turns / maximum


def in_row_turn_plan(geometry: DriveGeometry, wheel_degrees: float = 720.0,
                     direction: Literal["left", "right"] = "right") -> DifferentialTurnPlan:
    """Contra-wheel in-row target inherited from OAK's 720 wheel-degree setting."""
    geometry.validate()
    if not math.isfinite(wheel_degrees) or wheel_degrees <= 0:
        raise ValueError("in_row_turn_wheel_degrees måste vara positiv")
    if direction not in ("left", "right"): raise ValueError("turn direction måste vara left eller right")
    left = geometry.left_wheel_circumference_m * wheel_degrees / 360.0
    right = geometry.right_wheel_circumference_m * wheel_degrees / 360.0
    if direction == "left": left = -left
    else: right = -right
    left_ratio, right_ratio = _ratios(left / geometry.left_wheel_circumference_m,
                                      right / geometry.right_wheel_circumference_m)
    return DifferentialTurnPlan(left, right, left_ratio, right_ratio, direction)


def new_row_turn_targets(geometry: DriveGeometry, row_spacing_m: float, speed_rpm: float,
                         direction: Literal["left", "right"] = "right",
                         inner_wheel_min_ratio: float = 0.0) -> DifferentialTurnPlan:
    """Forward arc targets; ratios remain motor-side and have no gearbox scale."""
    plan = new_row_turn_plan(geometry, row_spacing_m, speed_rpm, direction, inner_wheel_min_ratio)
    left_ratio, right_ratio = _ratios(plan.left_distance_m / geometry.left_wheel_circumference_m,
                                      plan.right_distance_m / geometry.right_wheel_circumference_m)
    return DifferentialTurnPlan(plan.left_distance_m, plan.right_distance_m,
                                left_ratio, right_ratio, direction)


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
