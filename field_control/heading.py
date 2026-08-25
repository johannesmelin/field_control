"""Heading filtering and row-reference estimation.

The circular low-pass equations are retained from the verified get_heading
implementation.  This module adds only the row-distance history required by
field_control; it does not read IMU hardware itself.
"""
from __future__ import annotations

from collections import deque
import math


def wrap_degrees(value: float) -> float:
    wrapped = value % 360.0
    return 0.0 if math.isclose(wrapped, 360.0, abs_tol=1e-9) else wrapped


def signed_angle_delta(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def circular_low_pass(previous: float | None, sample: float, alpha: float) -> float:
    if not 0.0 < alpha <= 1.0:
        raise ValueError("heading_filter_alpha måste vara > 0 och <= 1")
    if not math.isfinite(sample):
        raise ValueError("heading måste vara ändlig")
    sample = wrap_degrees(sample)
    return sample if previous is None else wrap_degrees(previous + alpha * signed_angle_delta(sample, previous))


class RowHeadingReference:
    """Cicular row-heading mean over valid visual-following distance only."""

    def __init__(self, window_m: float, minimum_distance_m: float) -> None:
        if not math.isfinite(window_m) or window_m <= 0 or not math.isfinite(minimum_distance_m) or minimum_distance_m < 0:
            raise ValueError("headingreferensens sträckor är ogiltiga")
        self.window_m, self.minimum_distance_m = window_m, minimum_distance_m
        self._samples: deque[tuple[float, float]] = deque()
        self._reliable_distance_m = 0.0
        self.reference_deg: float | None = None
        self.reliable = False

    @property
    def reliable_distance_m(self) -> float:
        return self._reliable_distance_m

    def add_visual_heading(self, heading_deg: float, distance_m: float) -> None:
        if not math.isfinite(heading_deg) or not math.isfinite(distance_m):
            raise ValueError("heading och sträcka måste vara ändliga")
        heading = wrap_degrees(heading_deg)
        if self._samples and distance_m < self._samples[-1][0]:
            raise ValueError("odometristräckan får inte minska")
        previous_distance = self._samples[-1][0] if self._samples else distance_m
        self._reliable_distance_m += max(0.0, distance_m - previous_distance)
        self._samples.append((distance_m, heading))
        while self._samples and distance_m - self._samples[0][0] > self.window_m:
            self._samples.popleft()
        x = sum(math.cos(math.radians(value)) for _d, value in self._samples)
        y = sum(math.sin(math.radians(value)) for _d, value in self._samples)
        if math.hypot(x, y) > 1e-9:
            self.reference_deg = wrap_degrees(math.degrees(math.atan2(y, x)))
        self.reliable = self.reference_deg is not None and self._reliable_distance_m >= self.minimum_distance_m

    def apply_successful_180_turn(self) -> float:
        if self.reference_deg is None:
            raise ValueError("kan inte härleda heading efter vändning utan tidigare radreferens")
        self.reference_deg = wrap_degrees(self.reference_deg + 180.0)
        self.reliable = True
        self._samples.clear()
        self._reliable_distance_m = self.minimum_distance_m
        return self.reference_deg
