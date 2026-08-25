"""Validated configuration types shared by vision and navigation.

All image zones are normalised (0.0–1.0), keeping the field configuration
independent of camera and livestream resolution.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from .state_machine import SafetyConfig
from .odometry import DriveGeometry


@dataclass(frozen=True)
class Zone:
    x_min: float; x_max: float; y_min: float; y_max: float

    def validate(self) -> "Zone":
        if not (0 <= self.x_min < self.x_max <= 1 and 0 <= self.y_min < self.y_max <= 1):
            raise ValueError("zon måste ligga inom 0,0–1,0 och ha positiv storlek")
        return self

    def pixels(self, width: int, height: int) -> tuple[int, int, int, int]:
        self.validate()
        return (round(self.x_min * width), round(self.x_max * width),
                round(self.y_min * height), round(self.y_max * height))


@dataclass(frozen=True)
class HsvFilter:
    low: tuple[int, int, int]
    high: tuple[int, int, int]
    min_area: int

    def validate(self) -> "HsvFilter":
        if len(self.low) != 3 or len(self.high) != 3:
            raise ValueError("HSV-gränser måste ha tre värden")
        if any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= limit
               for values in (self.low, self.high) for v, limit in zip(values, (179, 255, 255))):
            raise ValueError("HSV-värden utanför tillåtet intervall")
        if any(low > high for low, high in zip(self.low, self.high)):
            raise ValueError("HSV low får inte vara större än high")
        if self.min_area < 1:
            raise ValueError("min_area måste vara minst 1")
        return self


@dataclass(frozen=True)
class VisionConfig:
    navigation_mode: Literal["buds_only", "buds_and_leaves"] = "buds_and_leaves"
    buds: HsvFilter = HsvFilter((0, 0, 0), (179, 255, 255), 100)
    leaves: HsvFilter = HsvFilter((0, 0, 0), (179, 255, 255), 100)
    marker: HsvFilter = HsvFilter((0, 0, 0), (179, 255, 255), 100)
    navigation_zone: Zone = Zone(.2, .8, .3, 1.0)
    trigger_zone: Zone = Zone(.2, .8, .8, 1.0)
    pick_zone: Zone = Zone(.2, .8, .5, 1.0)
    turn_marker_zone: Zone = Zone(.0, 1.0, .0, 1.0)
    x_goal: float = .5
    x_filter_window_frames: int = 5
    x_outlier_threshold_px: float | None = None

    def validate(self) -> "VisionConfig":
        if self.navigation_mode not in ("buds_only", "buds_and_leaves"):
            raise ValueError("navigation_mode måste vara buds_only eller buds_and_leaves")
        self.buds.validate(); self.leaves.validate(); self.marker.validate()
        self.navigation_zone.validate(); self.trigger_zone.validate(); self.pick_zone.validate(); self.turn_marker_zone.validate()
        if not 0 <= self.x_goal <= 1:
            raise ValueError("x_goal måste vara normaliserat till 0,0–1,0")
        if self.x_filter_window_frames < 1:
            raise ValueError("x_filter_window_frames måste vara minst 1")
        if self.x_outlier_threshold_px is not None and self.x_outlier_threshold_px <= 0:
            raise ValueError("x_outlier_threshold_px måste vara positiv eller null")
        return self


@dataclass(frozen=True)
class RuntimeConfig:
    vision: VisionConfig = VisionConfig()
    safety: SafetyConfig = SafetyConfig()
    heading_filter_alpha: float = .2
    row_heading_window_m: float = 2.0
    heading_reference_min_distance_m: float = 1.0
    camera_timeout_s: float = .5
    imu_timeout_s: float = .5
    odometry_timeout_s: float = .5
    control_lease_timeout_s: float = .5
    odometry_geometry: DriveGeometry = DriveGeometry()
    row_spacing_m: float = 1.2
    processing_width: int = 320
    processing_height: int = 240
    navigation_frame_rate_hz: float = 10.0
    stream_enabled: bool = True
    stream_fps: float = 5.0
    stream_width: int = 320
    stream_height: int = 240
    jpeg_quality: int = 85
    max_rpm: float = 0.0
    auto_base_rpm: float = 0.0
    search_speed_rpm: float = 0.0
    turn_speed_rpm: float = 0.0
    vision_kp: float = 0.0
    vision_deadband_px: float = 0.0
    max_vision_correction_rpm: float = 0.0
    heading_kp: float = 0.0
    heading_deadband_deg: float = 0.0
    max_heading_correction_rpm: float = 0.0

    def validate(self) -> "RuntimeConfig":
        self.vision.validate(); self.safety.validate(); self.odometry_geometry.validate()
        if not 0 < self.heading_filter_alpha <= 1:
            raise ValueError("heading_filter_alpha måste vara > 0 och <= 1")
        if self.row_heading_window_m <= 0 or self.heading_reference_min_distance_m < 0:
            raise ValueError("headingreferensens sträckor är ogiltiga")
        if self.row_spacing_m <= self.odometry_geometry.wheel_track_m:
            raise ValueError("row_spacing_m måste vara större än wheel_track_m")
        timeouts = (self.camera_timeout_s, self.imu_timeout_s, self.odometry_timeout_s, self.control_lease_timeout_s)
        if any(value <= 0 for value in timeouts):
            raise ValueError("sensortimeout måste vara positiv")
        dimensions = (self.processing_width, self.processing_height, self.stream_width, self.stream_height)
        if any(value < 1 for value in dimensions) or self.jpeg_quality not in range(1, 101):
            raise ValueError("bilddimensioner och JPEG-kvalitet är ogiltiga")
        if self.navigation_frame_rate_hz <= 0 or self.stream_fps <= 0:
            raise ValueError("bildfrekvens måste vara positiv")
        numeric = (self.max_rpm, self.auto_base_rpm, self.search_speed_rpm, self.turn_speed_rpm,
                   self.vision_kp, self.vision_deadband_px, self.max_vision_correction_rpm,
                   self.heading_kp, self.heading_deadband_deg, self.max_heading_correction_rpm)
        if any(value < 0 for value in numeric):
            raise ValueError("styrparametrar får inte vara negativa")
        return self
