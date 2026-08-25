"""Coherent, non-blocking sensor observation for navigation and diagnostics."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

from .heading import RowHeadingReference, circular_low_pass
from .sources import SourceSnapshot

if TYPE_CHECKING:
    from .vision import VisionResult


@dataclass(frozen=True)
class ImuReading:
    heading_deg: float
    timestamp_s: float


@dataclass(frozen=True)
class Observation:
    now_s: float
    vision: "VisionResult | None"
    heading_deg: float | None
    row_heading_reference_deg: float | None
    row_heading_reliable: bool
    camera_age_s: float | None
    imu_age_s: float | None
    odometry_age_s: float | None
    distance_m: float
    camera_fresh: bool
    imu_fresh: bool
    odometry_fresh: bool
    camera_error: str | None = None
    imu_error: str | None = None
    fault: str | None = None

    @property
    def visual_target(self) -> bool:
        return self.vision is not None and self.vision.target_x is not None and self.camera_fresh


class HeadingProcessor:
    """Reuse the verified circular low-pass and update row reference explicitly."""

    def __init__(self, alpha: float, reference: RowHeadingReference) -> None:
        self.alpha, self.reference = alpha, reference
        self.filtered_heading_deg: float | None = None

    def update(self, reading: ImuReading, *, visual_following: bool, distance_m: float) -> float:
        if not math.isfinite(reading.heading_deg) or not math.isfinite(reading.timestamp_s):
            raise ValueError("ogiltig IMU-heading")
        self.filtered_heading_deg = circular_low_pass(
            self.filtered_heading_deg, reading.heading_deg, self.alpha,
        )
        if visual_following:
            self.reference.add_visual_heading(self.filtered_heading_deg, distance_m)
        return self.filtered_heading_deg


def build_observation(
    now_s: float,
    camera: SourceSnapshot[object],
    imu: SourceSnapshot[ImuReading],
    odometry: SourceSnapshot[float],
    vision: "VisionResult | None",
    heading: HeadingProcessor,
    camera_timeout_s: float,
    imu_timeout_s: float,
    odometry_timeout_s: float,
    can_healthy: bool = True,
) -> Observation:
    camera_age = camera.age_s(now_s)
    imu_age = imu.age_s(now_s)
    odometry_age = odometry.age_s(now_s)
    camera_fresh = camera.connected and camera_age is not None and camera_age <= camera_timeout_s
    imu_fresh = imu.connected and imu_age is not None and imu_age <= imu_timeout_s
    odometry_fresh = odometry.connected and odometry_age is not None and odometry_age <= odometry_timeout_s
    fault = None
    if not camera_fresh: fault = "CAMERA_TIMEOUT"
    elif not imu_fresh: fault = "IMU_TIMEOUT"
    elif not odometry_fresh: fault = "ODOMETRY_TIMEOUT"
    elif not can_healthy: fault = "CAN_FAILURE"
    return Observation(
        now_s, vision if camera_fresh else None, heading.filtered_heading_deg,
        heading.reference.reference_deg, heading.reference.reliable,
        camera_age, imu_age, odometry_age,
        float(odometry.value or 0.0), camera_fresh, imu_fresh, odometry_fresh,
        camera.error, imu.error, fault,
    )