"""Strict standard-library JSON serialization for deployment configuration."""
from __future__ import annotations

from dataclasses import asdict, fields
import json
import math
from pathlib import Path
from typing import Any, Callable

from .config import (FirstCrop, GoalRelativeZone, HsvFilter, PhysicalCanConfig,
                     RuntimeConfig, TrapezoidZone, VisionConfig, Zone)
from .odometry import DriveGeometry
from .state_machine import SafetyConfig


def _object(value: Any, cls: type, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} måste vara ett JSON-objekt")
    unknown = set(value) - {field.name for field in fields(cls)}
    if unknown:
        raise ValueError(f"okända nycklar i {path}: {', '.join(sorted(unknown))}")
    return value


def _number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{path} måste vara ett ändligt tal")
    return float(value)


def _integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{path} måste vara ett heltal")
    return value


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{path} måste vara boolesk")
    return value


def _string(value: Any, path: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{path} måste vara en sträng")
    return value


def _optional_string(value: Any, path: str) -> str | None:
    return None if value is None else _string(value, path)


def _tuple3(value: Any, path: str) -> tuple[int, int, int]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{path} måste vara en lista med tre heltal")
    return tuple(_integer(item, f"{path}[{index}]") for index, item in enumerate(value))  # type: ignore[return-value]


def _zone(value: Any, path: str) -> Zone | TrapezoidZone | GoalRelativeZone:
    if not isinstance(value, dict):
        raise ValueError(f"{path} måste vara ett JSON-objekt")
    cls = (GoalRelativeZone if "x_distance" in value else
           TrapezoidZone if {"x_min_top", "x_max_top", "x_min_bottom", "x_max_bottom"} & set(value) else Zone)
    data = _object(value, cls, path)
    missing = {field.name for field in fields(cls)} - set(data)
    if missing:
        raise ValueError(f"saknade nycklar i {path}: {', '.join(sorted(missing))}")
    return cls(**{field.name: _number(data[field.name], f"{path}.{field.name}")
                  for field in fields(cls)})


def _first_crop(value: Any, path: str) -> FirstCrop:
    data = _object(value, FirstCrop, path)
    missing = {field.name for field in fields(FirstCrop)} - set(data)
    if missing:
        raise ValueError(f"saknade nycklar i {path}: {', '.join(sorted(missing))}")
    return FirstCrop(**{field.name: _number(data[field.name], f"{path}.{field.name}")
                        for field in fields(FirstCrop)})


def _hsv(value: Any, path: str) -> HsvFilter:
    data = _object(value, HsvFilter, path)
    missing = {field.name for field in fields(HsvFilter)} - set(data)
    if missing:
        raise ValueError(f"saknade nycklar i {path}: {', '.join(sorted(missing))}")
    return HsvFilter(low=_tuple3(data["low"], f"{path}.low"),
                     high=_tuple3(data["high"], f"{path}.high"),
                     min_area=_integer(data["min_area"], f"{path}.min_area"))


def _vision(value: Any) -> VisionConfig:
    data = _object(value, VisionConfig, "vision")
    defaults = VisionConfig()
    kwargs: dict[str, Any] = {}
    for field in fields(VisionConfig):
        item = data.get(field.name, getattr(defaults, field.name))
        path = f"vision.{field.name}"
        if field.name in ("buds", "leaves", "marker"):
            kwargs[field.name] = getattr(defaults, field.name) if field.name not in data else _hsv(item, path)
        elif field.name.endswith("_zone") or "_zone_" in field.name:
            kwargs[field.name] = getattr(defaults, field.name) if field.name not in data else _zone(item, path)
        elif field.name == "first_crop":
            kwargs[field.name] = getattr(defaults, field.name) if field.name not in data else _first_crop(item, path)
        elif field.name == "navigation_mode":
            kwargs[field.name] = _string(item, path)
        elif field.name == "camera_serial_number":
            kwargs[field.name] = _string(item, path)
        elif field.name in ("row_1_enabled", "row_2_enabled"):
            kwargs[field.name] = _boolean(item, path)
        elif field.name == "x_filter_window_frames":
            kwargs[field.name] = _integer(item, path)
        elif field.name == "x_outlier_threshold_px":
            kwargs[field.name] = None if item is None else _number(item, path)
        elif field.name == "x_goal_top":
            kwargs[field.name] = None if item is None else _number(item, path)
        else:
            kwargs[field.name] = _number(item, path)
    if isinstance(kwargs["turn_marker_zone"], GoalRelativeZone):
        raise ValueError("vision.turn_marker_zone stöder inte målrelativ zon")
    return VisionConfig(**kwargs)


def _safety(value: Any) -> SafetyConfig:
    data = _object(value, SafetyConfig, "safety")
    defaults = SafetyConfig(); kwargs: dict[str, Any] = {}
    bools = {"in_row_turn_enabled"}
    integers = {"navigation_reacquire_frames", "turn_marker_confirm_frames", "turn_heading_confirm_frames", "number_of_rows"}
    strings = {"new_row_turn_direction"}
    for field in fields(SafetyConfig):
        item, path = data.get(field.name, getattr(defaults, field.name)), f"safety.{field.name}"
        kwargs[field.name] = (_boolean(item, path) if field.name in bools else
                              _integer(item, path) if field.name in integers else
                              _string(item, path) if field.name in strings else _number(item, path))
    return SafetyConfig(**kwargs)


def _geometry(value: Any) -> DriveGeometry:
    data = _object(value, DriveGeometry, "odometry_geometry")
    defaults = DriveGeometry(); kwargs: dict[str, Any] = {}
    for field in fields(DriveGeometry):
        item, path = data.get(field.name, getattr(defaults, field.name)), f"odometry_geometry.{field.name}"
        kwargs[field.name] = _integer(item, path) if field.name.endswith("_sign") else _number(item, path)
    return DriveGeometry(**kwargs)


def _physical(value: Any) -> PhysicalCanConfig:
    data = _object(value, PhysicalCanConfig, "physical_can")
    defaults = PhysicalCanConfig(); kwargs: dict[str, Any] = {}
    for field in fields(PhysicalCanConfig):
        item, path = data.get(field.name, getattr(defaults, field.name)), f"physical_can.{field.name}"
        kwargs[field.name] = (_boolean(item, path) if field.name in ("enabled", "confirm_physical_stop_tested", "confirm_wheels_raised",
                                                                       "confirm_ground_test", "confirm_ground_clear",
                                                                       "confirm_emergency_stop_ready")
                              else _optional_string(item, path))
    return PhysicalCanConfig(**kwargs)


def runtime_config_from_dict(value: Any) -> RuntimeConfig:
    """Parse a strict JSON-shaped mapping and run the normal safety validation."""
    data = _object(value, RuntimeConfig, "runtime")
    defaults = RuntimeConfig(); kwargs: dict[str, Any] = {}
    nested: dict[str, Callable[[Any], Any]] = {
        "vision": _vision, "safety": _safety, "physical_can": _physical, "odometry_geometry": _geometry,
    }
    floats = {"heading_filter_alpha", "row_heading_window_m", "heading_reference_min_distance_m",
              "camera_timeout_s", "imu_timeout_s", "odometry_timeout_s", "control_lease_timeout_s",
              "watchdog_period_s", "max_control_stall_s", "physical_web_standby_timeout_s", "row_spacing_m", "navigation_frame_rate_hz",
              "stream_fps", "max_rpm", "manual_rpm", "auto_base_rpm", "search_speed_rpm", "turn_speed_rpm",
              "vision_kp", "vision_deadband_px", "max_vision_correction_rpm", "heading_kp",
              "heading_deadband_deg", "max_heading_correction_rpm"}
    integers = {"processing_width", "processing_height", "imu_sample_hz", "stream_width", "stream_height", "jpeg_quality"}
    for field in fields(RuntimeConfig):
        item, path = data.get(field.name, getattr(defaults, field.name)), f"runtime.{field.name}"
        if field.name in nested:
            kwargs[field.name] = getattr(defaults, field.name) if field.name not in data else nested[field.name](item)
        elif field.name in floats:
            kwargs[field.name] = _number(item, path)
        elif field.name in integers:
            kwargs[field.name] = _integer(item, path)
        elif field.name == "stream_enabled":
            kwargs[field.name] = _boolean(item, path)
        elif field.name == "log_level":
            kwargs[field.name] = _string(item, path)
        else:
            raise AssertionError(f"saknad JSON-konverterare för {field.name}")
    return RuntimeConfig(**kwargs).validate()


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"ogiltig JSON-konstant: {value}")


def load_runtime_config(path: str | Path) -> RuntimeConfig:
    with Path(path).open(encoding="utf-8") as handle:
        value = json.load(handle, parse_constant=_reject_json_constant)
    return runtime_config_from_dict(value)


def runtime_config_to_dict(config: RuntimeConfig) -> dict[str, Any]:
    config.validate()
    return asdict(config)


def dump_runtime_config(config: RuntimeConfig, path: str | Path) -> None:
    with Path(path).open("w", encoding="utf-8") as handle:
        json.dump(runtime_config_to_dict(config), handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")
