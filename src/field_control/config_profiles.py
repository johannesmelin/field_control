"""Safe, restart-staged operator configuration profiles.

Profiles deliberately exclude deployment-only physical CAN data.  A saved
profile is a candidate for a later process start; it never mutates a running
runtime or its motor boundary.
"""
from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any

from .config import RuntimeConfig
from .config_io import runtime_config_from_dict
from .project_paths import project_data_root

_NAME = re.compile(r"konfig_\d{8}_\d{6}(?:_\d{6})?\.json\Z")
_SELECTED = "selected.json"
# These govern control-loss, scheduling and physical-web lifecycle.  They are
# deployment safety boundaries, not operator navigation settings.
_DEPLOYMENT_ONLY = frozenset({"physical_can", "control_lease_timeout_s", "watchdog_period_s",
                              "max_control_stall_s", "physical_web_standby_timeout_s"})


def default_profiles_dir() -> Path:
    return project_data_root() / "konfigurationer"


def _directory(directory: Path) -> Path:
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    st = directory.lstat()
    if not directory.is_dir() or directory.is_symlink():
        raise ValueError("konfigurationsmappen måste vara en vanlig katalog")
    return directory


def _safe_child(directory: Path, name: str, *, allow_selected: bool = False) -> Path:
    if not (name == _SELECTED and allow_selected) and not _NAME.fullmatch(name):
        raise ValueError("ogiltigt konfigurationsfilnamn")
    child = directory / name
    if child.parent != directory:
        raise ValueError("ogiltigt konfigurationsfilnamn")
    # Path.exists() is false for a dangling symlink.  lstat() deliberately
    # observes the directory entry itself so both live and dangling links are
    # rejected before any read, listing, selection, or overwrite decision.
    try:
        child.lstat()
    except FileNotFoundError:
        pass
    else:
        if child.is_symlink() or not child.is_file():
            raise ValueError("konfigurationsfil måste vara en vanlig fil")
    return child


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    data = (json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    fd, temp_name = tempfile.mkstemp(prefix=".config-", dir=path.parent)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "wb") as handle:
            handle.write(data); handle.flush(); os.fsync(handle.fileno())
        os.replace(temp_name, path)
        directory_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try: os.fsync(directory_fd)
        finally: os.close(directory_fd)
    except BaseException:
        try: os.unlink(temp_name)
        except FileNotFoundError: pass
        raise


def operator_profile_dict(config: RuntimeConfig) -> dict[str, Any]:
    """Serialize all operator-facing config, explicitly excluding CAN gates."""
    data = asdict(config.validate())
    for key in _DEPLOYMENT_ONLY:
        data.pop(key, None)
    return data


_ZONE_FIELDS = frozenset({"navigation_zone", "trigger_zone", "pick_zone", "turn_marker_zone",
                          "navigation_zone_2", "trigger_zone_2", "pick_zone_2",
                          "navigation_zone_3", "trigger_zone_3", "pick_zone_3",
                          "navigation_zone_4", "trigger_zone_4", "pick_zone_4"})


def _merge(base: dict[str, Any], profile: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in profile.items():
        if key in _DEPLOYMENT_ONLY:
            raise ValueError(f"operatörsprofil får inte innehålla deployment-parametern {key}")
        if key not in merged:
            raise ValueError(f"okänd profilparameter: {key}")
        # Zone encodings are discriminated unions (legacy rectangle,
        # explicit trapezoid, or goal-relative).  They must replace as one
        # strict object: recursively merging their incompatible key sets can
        # fabricate a malformed hybrid before config_io has a chance to
        # validate the profile.
        if key in _ZONE_FIELDS:
            merged[key] = value
        elif isinstance(value, dict) and isinstance(merged[key], dict):
            merged[key] = _merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _normalize_legacy_cam_b_geometry(config: dict[str, Any]) -> dict[str, Any]:
    """Migrate only the known pre-full-FOV CAM_B 320x240 representation."""
    normalized = dict(config)
    # Before full-width CAM_B support, operator profiles used exactly 320x240
    # and the acquisition path centre-cropped the 16:10 sensor. Other aspect
    # ratios remain invalid so no arbitrary operator geometry is changed.
    for width_key, height_key in (("processing_width", "processing_height"),
                                  ("stream_width", "stream_height")):
        if (type(normalized[width_key]) is int and type(normalized[height_key]) is int
                and normalized[width_key] == 320 and normalized[height_key] == 240):
            normalized[height_key] = 200
    return normalized


def list_profiles(directory: Path | None = None) -> list[str]:
    directory = _directory(directory or default_profiles_dir())
    names: list[str] = []
    for child in directory.iterdir():
        if _NAME.fullmatch(child.name):
            _safe_child(directory, child.name)
            names.append(child.name)
    return sorted(names, reverse=True)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as handle:
            value = json.load(handle, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    except json.JSONDecodeError as exc:
        raise ValueError("ogiltig JSON i konfigurationsprofil") from exc
    if not isinstance(value, dict): raise ValueError("konfigurationsprofil måste vara ett objekt")
    return value


def load_profile(name: str, deployment: RuntimeConfig, directory: Path | None = None) -> RuntimeConfig:
    directory = _directory(directory or default_profiles_dir())
    profile = _load_json(_safe_child(directory, name))
    merged = _merge(asdict(deployment.validate()), profile)
    return runtime_config_from_dict(_normalize_legacy_cam_b_geometry(merged))


def save_profile(config: RuntimeConfig, directory: Path | None = None, *, now: datetime | None = None) -> str:
    directory = _directory(directory or default_profiles_dir())
    now = now or datetime.now()
    stem = now.strftime("konfig_%Y%m%d_%H%M%S")
    name = f"{stem}.json"
    if (directory / name).exists(): name = f"{stem}_{now.microsecond:06d}.json"
    path = _safe_child(directory, name)
    if path.exists():
        # Same microsecond is unlikely but must never overwrite a profile.
        raise FileExistsError("konfigurationsfilnamnskollision")
    _atomic_json(path, operator_profile_dict(config))
    return name


def selected_profile(directory: Path | None = None) -> str | None:
    directory = _directory(directory or default_profiles_dir()); path = _safe_child(directory, _SELECTED, allow_selected=True)
    if not path.exists(): return None
    value = _load_json(path); name = value.get("selected")
    if not isinstance(name, str): raise ValueError("ogiltigt valt konfigurationsnamn")
    _safe_child(directory, name)
    return name


def select_profile(name: str, directory: Path | None = None) -> None:
    directory = _directory(directory or default_profiles_dir()); _safe_child(directory, name)
    _atomic_json(_safe_child(directory, _SELECTED, allow_selected=True), {"selected": name})


def load_selected_or_latest(deployment: RuntimeConfig, directory: Path | None = None) -> tuple[RuntimeConfig, str | None]:
    directory = _directory(directory or default_profiles_dir())
    name = selected_profile(directory)
    if name is None:
        names = list_profiles(directory); name = names[0] if names else None
    return (load_profile(name, deployment, directory), name) if name else (deployment.validate(), None)
