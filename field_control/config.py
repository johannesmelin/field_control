"""Validated configuration types shared by vision and navigation.

All image zones are normalised (0.0–1.0), keeping the field configuration
independent of camera and livestream resolution.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from .state_machine import SafetyConfig
from .odometry import DriveGeometry


# A position-target turn must be given enough time to traverse the largest
# wheel target at the commanded *motor-side* RPM.  Keep the bounded CAN/setup
# allowance explicit and shared by every physical deployment configuration.
A4_POSITION_TURN_TIMEOUT_MARGIN_S = 10.0


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
class TrapezoidZone:
    """A normalised, top-to-bottom convex zone for perspective-aware vision.

    The point order returned by :meth:`pixels` is top-left, top-right,
    bottom-right, bottom-left.  ``Zone`` remains the default in order to keep
    existing deployments and JSON profiles byte-for-byte meaningful.
    """
    x_min_top: float; x_max_top: float; y_min: float
    x_min_bottom: float; x_max_bottom: float; y_max: float

    def validate(self) -> "TrapezoidZone":
        top_width = self.x_max_top - self.x_min_top
        bottom_width = self.x_max_bottom - self.x_min_bottom
        # Ordinary four-corner zones must have overlapping horizontal spans;
        # otherwise the requested corner order can fold through itself.  A
        # collapsed top/bottom is permitted only for a clipped, still
        # positive-area triangular intersection at an image boundary.
        clipped_triangle = top_width == 0 or bottom_width == 0
        interleaved = (self.x_min_top <= self.x_max_bottom
                       and self.x_min_bottom <= self.x_max_top)
        if not (0 <= self.x_min_top <= self.x_max_top <= 1
                and 0 <= self.x_min_bottom <= self.x_max_bottom <= 1
                and 0 <= self.y_min < self.y_max <= 1
                and top_width + bottom_width > 0
                and (clipped_triangle or interleaved)):
            raise ValueError("trapetszon måste ligga inom 0,0–1,0 och ha positiv storlek")
        return self

    def pixels(self, width: int, height: int) -> tuple[tuple[int, int], ...]:
        self.validate()
        return ((round(self.x_min_top * width), round(self.y_min * height)),
                (round(self.x_max_top * width), round(self.y_min * height)),
                (round(self.x_max_bottom * width), round(self.y_max * height)),
                (round(self.x_min_bottom * width), round(self.y_max * height)))


@dataclass(frozen=True)
class FirstCrop:
    """Normalised first-stage crop; full-frame defaults preserve legacy input."""
    x_min: float = 0.0
    x_max: float = 1.0
    y_min: float = 0.0
    y_max: float = 1.0

    def validate(self) -> "FirstCrop":
        if not (0 <= self.x_min < self.x_max <= 1 and 0 <= self.y_min < self.y_max <= 1):
            raise ValueError("first_crop måste ligga inom 0,0–1,0 och ha positiv storlek")
        return self

    def pixels(self, width: int, height: int) -> tuple[int, int, int, int]:
        self.validate()
        return (round(self.x_min * width), round(self.x_max * width),
                round(self.y_min * height), round(self.y_max * height))


@dataclass(frozen=True)
class GoalRelativeZone:
    """A navigation zone centred on the perspective-aware x-goal guide.

    ``x_distance`` is the half-width in normalised processed-image
    coordinates at ``y_max``.  It is intentionally a single operator-facing
    width rather than two independently drifting x boundaries.
    """
    x_distance: float
    y_min: float
    y_max: float

    def validate(self) -> "GoalRelativeZone":
        if not (0 <= self.x_distance <= .5 and 0 <= self.y_min < self.y_max <= 1):
            raise ValueError("målrelativ zon måste ha x_distance inom 0,0–0,5 och giltigt y-område")
        return self


def project_zone_to_trapezoid(zone: Zone, *, ground_width_bottom_m: float,
                              ground_width_top_m: float,
                              first_crop: FirstCrop = FirstCrop()) -> TrapezoidZone:
    """Project a rectangular zone's lower-edge x limits onto the upper edge.

    Widths describe the full-frame visible ground plane at its lower and upper
    edges.  A legacy rectangular zone expands around its centre toward its
    own upper/far edge according to the apparent ground-plane width.
    ``first_crop`` maps those rows and columns back to the full-frame
    calibration.  Equal widths deliberately return an equivalent rectangle.
    """
    zone.validate()
    if (not math.isfinite(ground_width_bottom_m) or not math.isfinite(ground_width_top_m)
            or ground_width_bottom_m <= 0 or ground_width_top_m <= 0):
        raise ValueError("markbredder måste vara positiva ändliga meter")
    first_crop.validate()
    def width_at_full_y(y: float) -> float:
        return ground_width_top_m + (ground_width_bottom_m - ground_width_top_m) * y
    y_span = first_crop.y_max - first_crop.y_min
    x_span = first_crop.x_max - first_crop.x_min
    top_width = width_at_full_y(first_crop.y_min + y_span * zone.y_min)
    bottom_width = width_at_full_y(first_crop.y_min + y_span * zone.y_max)
    ratio = top_width / bottom_width
    def top_x(x: float) -> float:
        full_bottom_x = first_crop.x_min + x * x_span
        full_top_x = .5 + (full_bottom_x - .5) * ratio
        # A physical boundary can legitimately leave an asymmetric crop at
        # its upper end.  Intersect it with the processed image rather than
        # rejecting the entire frame.  Equal clipped endpoints represent a
        # positive-area triangular intersection when the lower edge remains
        # visible (validated by TrapezoidZone).
        return min(1.0, max(0.0, (full_top_x - first_crop.x_min) / x_span))
    return TrapezoidZone(top_x(zone.x_min), top_x(zone.x_max), zone.y_min,
                         zone.x_min, zone.x_max, zone.y_max).validate()


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
    navigation_zone: Zone | TrapezoidZone | GoalRelativeZone = GoalRelativeZone(.3, .3, 1.0)
    trigger_zone: Zone | TrapezoidZone | GoalRelativeZone = GoalRelativeZone(.3, .8, 1.0)
    pick_zone: Zone | TrapezoidZone | GoalRelativeZone = GoalRelativeZone(.3, .5, 1.0)
    turn_marker_zone: Zone = Zone(.0, 1.0, .0, 1.0)
    x_goal: float = .5
    x_filter_window_frames: int = 5
    x_outlier_threshold_px: float | None = None
    # All fields below are trailing so legacy positional callers keep exactly
    # their old interpretation.
    first_crop: FirstCrop = FirstCrop()
    x_goal_top: float | None = None
    # Unequal widths opt in to a perspective-derived guide. Equal defaults
    # retain the legacy vertical x_goal without requiring migration.
    ground_width_bottom_m: float = 1.0
    ground_width_top_m: float = 1.0
    # Row 1 deliberately retains the original flat names for profile
    # compatibility.  These trailing row-2 values make dual-row operation an
    # additive change: old profiles are row 1 and obtain a usable row 2.
    navigation_zone_2: Zone | TrapezoidZone | GoalRelativeZone = GoalRelativeZone(.3, .3, 1.0)
    trigger_zone_2: Zone | TrapezoidZone | GoalRelativeZone = GoalRelativeZone(.3, .8, 1.0)
    pick_zone_2: Zone | TrapezoidZone | GoalRelativeZone = GoalRelativeZone(.3, .5, 1.0)
    x_goal_2: float = .75

    @property
    def x_goal_1(self) -> float:
        """Explicit dual-row spelling for the legacy row-1 goal."""
        return self.x_goal

    @property
    def navigation_zone_1(self) -> Zone | TrapezoidZone | GoalRelativeZone:
        return self.navigation_zone

    @property
    def trigger_zone_1(self) -> Zone | TrapezoidZone | GoalRelativeZone:
        return self.trigger_zone

    @property
    def pick_zone_1(self) -> Zone | TrapezoidZone | GoalRelativeZone:
        return self.pick_zone

    def validate(self) -> "VisionConfig":
        if self.navigation_mode not in ("buds_only", "buds_and_leaves"):
            raise ValueError("navigation_mode måste vara buds_only eller buds_and_leaves")
        self.buds.validate(); self.leaves.validate(); self.marker.validate()
        self.navigation_zone.validate(); self.trigger_zone.validate(); self.pick_zone.validate(); self.turn_marker_zone.validate()
        self.navigation_zone_2.validate(); self.trigger_zone_2.validate(); self.pick_zone_2.validate()
        if isinstance(self.turn_marker_zone, GoalRelativeZone):
            raise ValueError("turn_marker_zone stöder inte målrelativ zon")
        self.first_crop.validate()
        if not 0 <= self.x_goal <= 1:
            raise ValueError("x_goal måste vara normaliserat till 0,0–1,0")
        if not 0 <= self.x_goal_2 <= 1:
            raise ValueError("x_goal_2 måste vara normaliserat till 0,0–1,0")
        if self.x_goal_top is not None and not 0 <= self.x_goal_top <= 1:
            raise ValueError("x_goal_top måste vara normaliserat till 0,0–1,0 eller null")
        if (not math.isfinite(self.ground_width_bottom_m) or not math.isfinite(self.ground_width_top_m)
                or self.ground_width_bottom_m <= 0 or self.ground_width_top_m <= 0):
            raise ValueError("ground_width_*_m måste vara positiva ändliga meter")
        for zone, row in ((self.navigation_zone, 1), (self.trigger_zone, 1),
                          (self.pick_zone, 1), (self.navigation_zone_2, 2),
                          (self.trigger_zone_2, 2), (self.pick_zone_2, 2)):
            if isinstance(zone, GoalRelativeZone):
                self._effective_goal_relative_zone(zone, row).validate()
        if self.x_filter_window_frames < 1:
            raise ValueError("x_filter_window_frames måste vara minst 1")
        if self.x_outlier_threshold_px is not None and self.x_outlier_threshold_px <= 0:
            raise ValueError("x_outlier_threshold_px måste vara positiv eller null")
        return self

    def _goal_x_endpoints(self, row: int = 1) -> tuple[float, float]:
        """Return processed-image top and bottom x-goal endpoints."""
        goal = self.x_goal if row == 1 else self.x_goal_2
        if self.x_goal_top is not None:
            # Preserve the explicitly calibrated row-1 top endpoint.  Row 2
            # follows the same physical projection unless separately modelled.
            if row == 1:
                return self.x_goal_top, goal
        if self.ground_width_bottom_m != self.ground_width_top_m:
            # ``x_goal`` specifies the physical offset at the lower edge.
            # At the wider upper edge that identical offset is nearer centre.
            def width_at_full_y(y: float) -> float:
                return (self.ground_width_top_m
                        + (self.ground_width_bottom_m - self.ground_width_top_m) * y)
            x_span = self.first_crop.x_max - self.first_crop.x_min
            full_bottom_x = self.first_crop.x_min + goal * x_span
            ratio = width_at_full_y(self.first_crop.y_max) / width_at_full_y(self.first_crop.y_min)
            full_top_x = .5 + (full_bottom_x - .5) * ratio
            return (full_top_x - self.first_crop.x_min) / x_span, goal
        return goal, goal

    def goal_x_normalized_fraction(self, y_normalized: float, row: int = 1) -> float:
        """Perspective guide at a continuous processed-image y fraction."""
        top, bottom = self._goal_x_endpoints(row)
        ratio = min(1.0, max(0.0, float(y_normalized)))
        return top + (bottom - top) * ratio

    def goal_x_normalized(self, y_px: float, height: int, row: int = 1) -> float:
        """Goal at a processed-frame y coordinate; bottom is ``x_goal``.

        Keeping the old constant when no top endpoint is configured is an
        explicit backwards-compatibility contract.
        """
        goal = self.x_goal if row == 1 else self.x_goal_2
        return goal if height <= 1 else self.goal_x_normalized_fraction(float(y_px) / float(height - 1), row)

    def _effective_goal_relative_zone(self, zone: GoalRelativeZone, row: int = 1) -> Zone | TrapezoidZone:
        """Project one physical lower-edge half-width along the goal guide."""
        zone.validate()
        if zone.x_distance <= 0:
            raise ValueError("målrelativ zon måste ha positiv x_distance")
        def width_at_full_y(y: float) -> float:
            return self.ground_width_top_m + (self.ground_width_bottom_m - self.ground_width_top_m) * y
        y_span = self.first_crop.y_max - self.first_crop.y_min
        top_width = width_at_full_y(self.first_crop.y_min + y_span * zone.y_min)
        bottom_width = width_at_full_y(self.first_crop.y_min + y_span * zone.y_max)
        # The one displayed x_distance is deliberately calibrated at the
        # lower edge of the *crop*, independent of an individual zone's
        # y_max.  Every zone row then derives the same physical half-width.
        reference_width = width_at_full_y(self.first_crop.y_max)
        top_center = self.goal_x_normalized_fraction(zone.y_min, row)
        bottom_center = self.goal_x_normalized_fraction(zone.y_max, row)
        top_half_width = zone.x_distance * reference_width / top_width
        top_left, top_right = top_center - top_half_width, top_center + top_half_width
        bottom_half_width = zone.x_distance * reference_width / bottom_width
        bottom_left, bottom_right = bottom_center - bottom_half_width, bottom_center + bottom_half_width
        # This exact fast path preserves historic rectangular masks and
        # one-pixel outlines for the default centred/equal-width setup.
        if (top_width == bottom_width and top_center == bottom_center
                and 0 <= bottom_left < bottom_right <= 1):
            return Zone(bottom_left, bottom_right, zone.y_min, zone.y_max)
        # A partial intersection remains usable.  A fully out-of-frame or
        # zero-area result is rejected during config validation, before the
        # navigation loop owns the configuration.
        top_left, top_right = max(0.0, top_left), min(1.0, top_right)
        bottom_left, bottom_right = max(0.0, bottom_left), min(1.0, bottom_right)
        result = TrapezoidZone(top_left, top_right, zone.y_min,
                               bottom_left, bottom_right, zone.y_max)
        return result.validate()

    def effective_zone(self, zone: Zone | TrapezoidZone | GoalRelativeZone, row: int = 1) -> Zone | TrapezoidZone:
        """Return a literal trapezoid or derive one from a legacy rectangle."""
        if isinstance(zone, GoalRelativeZone):
            return self._effective_goal_relative_zone(zone, row)
        if isinstance(zone, Zone) and self.ground_width_bottom_m != self.ground_width_top_m:
            return project_zone_to_trapezoid(
                zone, ground_width_bottom_m=self.ground_width_bottom_m,
                ground_width_top_m=self.ground_width_top_m, first_crop=self.first_crop)
        return zone


@dataclass(frozen=True)
class PhysicalCanConfig:
    """Deployment-only opt-in for a physical SocketCAN motor boundary.

    The default deliberately has no channel or reply profile, so ordinary
    application construction remains a dry-run and cannot open CAN.
    """
    enabled: bool = False
    channel: str | None = None
    reply_profile: str | None = None
    slcan_device: str | None = None
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False
    # Ground-output is intentionally a separate, fully explicit operating
    # context.  These trailing defaults preserve existing positional callers
    # for the raised-wheel HIL profile.
    confirm_ground_test: bool = False
    confirm_ground_clear: bool = False
    confirm_emergency_stop_ready: bool = False

    def validate(self) -> "PhysicalCanConfig":
        if not isinstance(self.enabled, bool):
            raise ValueError("physical CAN enable måste vara boolesk")
        if not self.enabled:
            return self
        if not isinstance(self.channel, str) or not self.channel.strip():
            raise ValueError("explicit CAN-kanal krävs när fysisk output aktiveras")
        if self.channel != "can0":
            raise ValueError("fysisk output kräver den verifierade can0-kanalen")
        if self.reply_profile != "observed-rmdx-same-id":
            raise ValueError("fysisk output kräver installerad named same-ID motorsvarsprofil")
        by_id_prefix = "/dev/serial/by-id/"
        basename = (self.slcan_device[len(by_id_prefix):]
                    if isinstance(self.slcan_device, str) and self.slcan_device.startswith(by_id_prefix) else "")
        if not basename or basename in (".", "..") or "/" in basename:
            raise ValueError("fysisk output kräver stabil /dev/serial/by-id/-sökväg")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("explicit bekräftelse av fysiskt STOP-test krävs")
        raised = self.confirm_wheels_raised is True
        ground = (self.confirm_ground_test is True
                  and self.confirm_ground_clear is True
                  and self.confirm_emergency_stop_ready is True)
        any_ground = any((self.confirm_ground_test, self.confirm_ground_clear,
                          self.confirm_emergency_stop_ready))
        if raised:
            if any_ground:
                raise ValueError("fysisk CAN-output kräver exakt ett testläge: upphissade hjul eller fullständigt marktest")
        elif not ground:
            if any_ground:
                raise ValueError("marktest kräver --confirm-ground-test, fri yta och nödstopp")
            raise ValueError("explicit bekräftelse att hjulen är upphissade eller fullständigt marktest krävs")
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
    # Active manual control must be refreshed within this interval.  A lost
    # browser/client therefore reaches the runtime stop path deterministically.
    control_lease_timeout_s: float = 1.0
    watchdog_period_s: float = .02
    max_control_stall_s: float = .12
    # Deprecated compatibility value. Physical web standby is indefinitely
    # armed/no-motion; this value is intentionally not consulted at runtime.
    physical_web_standby_timeout_s: float = 30.0
    physical_can: PhysicalCanConfig = PhysicalCanConfig()
    odometry_geometry: DriveGeometry = DriveGeometry()
    row_spacing_m: float = 1.2
    processing_width: int = 320
    processing_height: int = 240
    navigation_frame_rate_hz: float = 10.0
    imu_sample_hz: int = 100
    stream_enabled: bool = True
    stream_fps: float = 5.0
    stream_width: int = 320
    stream_height: int = 240
    jpeg_quality: int = 85
    # All speed values below are motor-side RPM before DriveGeometry's ratio.
    max_rpm: float = 0.0
    manual_rpm: float = 0.0  # motor-side RPM; never enables physical output by itself
    auto_base_rpm: float = 0.0
    search_speed_rpm: float = 0.0
    turn_speed_rpm: float = 0.0  # motor-side RPM; turn ratios are geometry-only
    vision_kp: float = 0.0
    vision_deadband_px: float = 0.0
    max_vision_correction_rpm: float = 0.0
    heading_kp: float = 0.0
    heading_deadband_deg: float = 0.0
    max_heading_correction_rpm: float = 0.0
    log_level: str = "INFO"

    def minimum_physical_a4_turn_timeout_s(self) -> float | None:
        """Return the fail-safe deadline needed by enabled physical A4 turns.

        The verified A4 path commands motor-shaft angle while its configured
        speed is motor-side RPM.  Therefore a logical wheel target of ``D``
        degrees takes ``D * gear / (RPM * 6)`` seconds.  Only turn manoeuvres
        that this configuration can reach are included: an in-row turn needs
        ``in_row_turn_enabled`` and a new-row turn needs more than one row.
        ``None`` means this configuration cannot enter an automatic turn.
        """
        if not self.physical_can.enabled:
            return None

        target_degrees: list[float] = []
        geometry = self.odometry_geometry
        if self.safety.in_row_turn_enabled:
            target_degrees.append(float(self.safety.in_row_turn_wheel_degrees))
        if self.safety.number_of_rows > 1:
            inner_distance_m = math.pi * (self.row_spacing_m - geometry.wheel_track_m) / 2.0
            outer_distance_m = math.pi * (self.row_spacing_m + geometry.wheel_track_m) / 2.0
            if self.safety.new_row_turn_direction == "left":
                target_degrees.extend((
                    inner_distance_m / geometry.left_wheel_circumference_m * 360.0,
                    outer_distance_m / geometry.right_wheel_circumference_m * 360.0,
                ))
            else:
                target_degrees.extend((
                    outer_distance_m / geometry.left_wheel_circumference_m * 360.0,
                    inner_distance_m / geometry.right_wheel_circumference_m * 360.0,
                ))
        if not target_degrees:
            return None

        motor_rpm = min(float(self.turn_speed_rpm), float(self.max_rpm))
        if motor_rpm <= 0:
            raise ValueError("turn_speed_rpm och max_rpm måste vara positiva för fysisk A4-vändning")
        largest_wheel_degrees = max(abs(value) for value in target_degrees)
        nominal_s = (largest_wheel_degrees * geometry.motor_turns_per_wheel_turn
                     / (motor_rpm * 6.0))
        return nominal_s + A4_POSITION_TURN_TIMEOUT_MARGIN_S

    def validate(self) -> "RuntimeConfig":
        self.vision.validate(); self.safety.validate(); self.odometry_geometry.validate(); self.physical_can.validate()
        if not 0 < self.heading_filter_alpha <= 1:
            raise ValueError("heading_filter_alpha måste vara > 0 och <= 1")
        if self.row_heading_window_m <= 0 or self.heading_reference_min_distance_m < 0:
            raise ValueError("headingreferensens sträckor är ogiltiga")
        if self.row_spacing_m <= self.odometry_geometry.wheel_track_m:
            raise ValueError("row_spacing_m måste vara större än wheel_track_m")
        timeouts = (self.camera_timeout_s, self.imu_timeout_s, self.odometry_timeout_s,
                    self.control_lease_timeout_s)
        if any(value <= 0 for value in timeouts):
            raise ValueError("sensortimeout måste vara positiv")
        if not 0 < self.watchdog_period_s <= .020:
            raise ValueError("watchdogperiod måste vara positiv och högst 20 ms")
        if not 0 < self.max_control_stall_s <= .120:
            raise ValueError("maximal styrloopstall måste vara positiv och högst 120 ms")
        if self.control_lease_timeout_s > 1.0:
            raise ValueError("control-lease-timeout får vara högst 1 s")
        dimensions = (self.processing_width, self.processing_height, self.stream_width, self.stream_height)
        if any(value < 1 for value in dimensions) or self.jpeg_quality not in range(1, 101):
            raise ValueError("bilddimensioner och JPEG-kvalitet är ogiltiga")
        if self.navigation_frame_rate_hz <= 0 or self.stream_fps <= 0:
            raise ValueError("bildfrekvens måste vara positiv")
        if isinstance(self.imu_sample_hz, bool) or not isinstance(self.imu_sample_hz, int) or self.imu_sample_hz <= 0:
            raise ValueError("imu_sample_hz måste vara ett positivt heltal")
        if self.log_level not in ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"):
            raise ValueError("log_level måste vara DEBUG, INFO, WARNING, ERROR eller CRITICAL")
        numeric = (self.max_rpm, self.manual_rpm, self.auto_base_rpm, self.search_speed_rpm, self.turn_speed_rpm,
                   self.vision_kp, self.vision_deadband_px, self.max_vision_correction_rpm,
                   self.heading_kp, self.heading_deadband_deg, self.max_heading_correction_rpm)
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0
               for value in numeric):
            raise ValueError("styrparametrar får inte vara negativa")
        if self.physical_can.enabled and self.max_rpm <= 0:
            raise ValueError("max_rpm måste vara positiv när fysisk CAN-output aktiveras")
        minimum_turn_timeout_s = self.minimum_physical_a4_turn_timeout_s()
        if (minimum_turn_timeout_s is not None
                and self.safety.turn_timeout_s < minimum_turn_timeout_s):
            raise ValueError(
                "turn_timeout_s är för kort för fysisk A4-vändning: "
                f"minst {minimum_turn_timeout_s:.3f} s krävs för mål, utväxling, motor-RPM och 10 s marginal"
            )
        return self
