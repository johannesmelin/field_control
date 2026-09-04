"""Bounded differential controllers; no hardware transport is opened here."""
from __future__ import annotations

from dataclasses import dataclass
import math

from .heading import signed_angle_delta


@dataclass(frozen=True)
class WheelCommand:
    """Per-side motor RPM commands, before ``DriveGeometry`` gearbox ratio."""
    left_rpm: float
    right_rpm: float
    source: str


def _validate(base_rpm: float, kp: float, deadband: float, max_correction: float, max_rpm: float) -> None:
    if not all(math.isfinite(value) for value in (base_rpm, kp, deadband, max_correction, max_rpm)):
        raise ValueError("styrparametrar måste vara ändliga")
    if base_rpm < 0 or kp < 0 or deadband < 0 or max_correction < 0 or max_rpm <= 0:
        raise ValueError("styrparametrar utanför tillåtet intervall")


def bounded_differential(base_rpm: float, error: float, kp: float, deadband: float,
                         max_correction_rpm: float, max_rpm: float, source: str) -> WheelCommand:
    """Apply bounded P steering without either motor command exceeding ``max_rpm``.

    Positive error makes the left wheel faster and the right wheel slower. The
    physical motor adapter additionally clamps and signs commands at its own
    boundary as defence in depth.
    """
    _validate(base_rpm, kp, deadband, max_correction_rpm, max_rpm)
    correction = 0.0 if abs(error) <= deadband else min(max_correction_rpm, abs(error) * kp)
    correction = math.copysign(correction, error)
    left = min(max_rpm, max(-max_rpm, base_rpm + correction))
    right = min(max_rpm, max(-max_rpm, base_rpm - correction))
    return WheelCommand(left, right, source)


def vision_command(target_x: float, x_goal: float, base_rpm: float, kp: float,
                   deadband_px: float, max_correction_rpm: float, max_rpm: float) -> WheelCommand:
    return bounded_differential(base_rpm, target_x - x_goal, kp, deadband_px,
                                max_correction_rpm, max_rpm, "vision")


def heading_command(row_heading_reference_deg: float, filtered_heading_deg: float, base_rpm: float,
                    kp: float, deadband_deg: float, max_correction_rpm: float,
                    max_rpm: float) -> WheelCommand:
    return bounded_differential(base_rpm, signed_angle_delta(row_heading_reference_deg, filtered_heading_deg),
                                kp, deadband_deg, max_correction_rpm, max_rpm, "heading")
