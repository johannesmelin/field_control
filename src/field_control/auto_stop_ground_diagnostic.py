"""Fixed ground AUTO_SEARCH -> ROW_LOST diagnostic with post-close CAN evidence.

This is a deliberately narrow HIL runner, not a general drive command.  Its
only motion profile is 40 motor RPM, vision/pick/marker suppression, 1.5 m
IMU-only search, then the production runtime's ordinary ROW_LOST/STOP path.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from typing import Any, Callable
import time

from .app import FieldControlApplication
from .config import HsvFilter, PhysicalCanConfig, RuntimeConfig, VisionConfig
from .state_machine import SafetyConfig
from .stop_settle_diagnostic import _entry_to_json, _write_report


CAN_CHANNEL = "can0"
REPLY_PROFILE = "observed-rmdx-same-id"
MOTOR_RPM = 40.0
IMU_ONLY_DISTANCE_M = 1.5
AUTO_SEARCH_TIMEOUT_S = 35.0
SENSOR_READY_TIMEOUT_S = 5.0
POLL_S = 0.050
DISABLED_MIN_AREA = 1_000_000


@dataclass(frozen=True)
class AutoStopGroundDiagnosticRequest:
    slcan_device: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_ground_clear: bool = False
    confirm_emergency_stop_ready: bool = False

    def validate(self) -> "AutoStopGroundDiagnosticRequest":
        if self.enable_motors is not True:
            raise ValueError("--enable-motors krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_ground_clear is not True:
            raise ValueError("--confirm-ground-clear krävs")
        if self.confirm_emergency_stop_ready is not True:
            raise ValueError("--confirm-emergency-stop-ready krävs")
        prefix = "/dev/serial/by-id/"
        path = self.slcan_device
        basename = path[len(prefix):] if isinstance(path, str) and path.startswith(prefix) else ""
        if not isinstance(path, str) or not basename or basename in (".", "..") or "/" in basename:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class AutoStopGroundDiagnosticResult:
    request: AutoStopGroundDiagnosticRequest
    terminal_outcome: str
    terminal_state: str | None
    terminal_fault: str | None
    events: tuple[dict[str, Any], ...]
    cleanup_error: str | None
    worker_diagnostics: tuple[dict[str, Any], ...]
    report_path: str


def ground_auto_stop_config(request: AutoStopGroundDiagnosticRequest) -> RuntimeConfig:
    """Return the sole fixed ground AUTO_SEARCH diagnostic profile."""
    disabled = HsvFilter((0, 0, 0), (179, 255, 255), DISABLED_MIN_AREA)
    return RuntimeConfig(
        stream_enabled=False,
        max_rpm=MOTOR_RPM,
        auto_base_rpm=MOTOR_RPM,
        search_speed_rpm=MOTOR_RPM,
        turn_speed_rpm=0.0,
        vision_kp=0.0,
        max_vision_correction_rpm=0.0,
        heading_kp=0.0,
        max_heading_correction_rpm=0.0,
        vision=VisionConfig(navigation_mode="buds_and_leaves", buds=disabled, leaves=disabled, marker=disabled),
        safety=SafetyConfig(auto_start_delay_s=0.0, search_length_m=IMU_ONLY_DISTANCE_M,
                            in_row_turn_enabled=False, number_of_rows=1),
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device,
                                       True, False, True, True, True),
    ).validate()


def _events(runtime: object) -> tuple[dict[str, Any], ...]:
    recent = getattr(getattr(runtime, "events", None), "recent", None)
    if not callable(recent):
        return ()
    try:
        return tuple(item for item in recent() if isinstance(item, dict))
    except Exception as exc:
        return ({"kind": "diagnostic_events_unavailable", "data": {"error": type(exc).__name__}},)


def _post_close_snapshot(app: object | None) -> tuple[dict[str, Any], ...]:
    runtime = getattr(app, "runtime", None)
    motor = getattr(runtime, "motor", None)
    sink = getattr(motor, "_sink", motor)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if not callable(snapshot):
        return ()
    try:
        return tuple(_entry_to_json(entry) for entry in snapshot())
    except Exception as exc:
        return ({"diagnostic_snapshot_error": f"{type(exc).__name__}: {exc}"[:1000]},)


def _sensors_ready(status: object) -> bool:
    """Accept only one runtime-published, fully fresh pre-AUTO snapshot."""
    observation = getattr(status, "observation", None)
    return bool(getattr(observation, "camera_fresh", False)
                and getattr(observation, "imu_fresh", False)
                and getattr(observation, "odometry_fresh", False))


def _wait_for_sensor_readiness(runtime: object, *, monotonic: Callable[[], float],
                               sleep: Callable[[float], None]) -> bool:
    """Wait a fixed bounded interval before arming; never starts AUTO here."""
    deadline_s = monotonic() + SENSOR_READY_TIMEOUT_S
    while True:
        if monotonic() >= deadline_s:
            return False
        status = runtime.status()
        # status() may cross a scheduler boundary while obtaining its latest
        # source snapshot. Recheck before granting the one-way AUTO/arm path.
        if _sensors_ready(status) and monotonic() < deadline_s:
            return True
        sleep(min(POLL_S, max(0.0, deadline_s - monotonic())))


def run_auto_stop_ground_diagnostic(
    request: AutoStopGroundDiagnosticRequest, *,
    app_factory: Callable[[RuntimeConfig], FieldControlApplication] = FieldControlApplication,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> AutoStopGroundDiagnosticResult:
    """Run fixed AUTO_SEARCH until ROW_LOST/fault, then always STOP/MANUAL/close."""
    request.validate()
    config = ground_auto_stop_config(request)
    app: FieldControlApplication | None = None
    terminal_outcome, terminal_state, terminal_fault = "startup_failure", None, None
    terminal_events: tuple[dict[str, Any], ...] = ()
    cleanup_error: str | None = None
    try:
        app = app_factory(config)
        app.start()
        # OAK startup is asynchronous.  Do not let an initial empty source
        # snapshot become an AUTO sensor fault; wait for camera, IMU and the
        # shared encoder source before any arm/Start-Auto transition.  The
        # runtime retains ownership of all later arm and post-STOP odometry
        # recovery checks.
        if not _wait_for_sensor_readiness(app.runtime, monotonic=monotonic, sleep=sleep):
            terminal_outcome = "sensor_readiness_timeout"
            terminal_fault = "SENSOR_READINESS_TIMEOUT"
            terminal_state = str(getattr(app.runtime.status(), "state", None))
            terminal_events = _events(app.runtime)
        else:
            app.runtime.select_auto()
            app.runtime.arm_motor_output()
            app.runtime.start_auto()
            deadline_s = monotonic() + AUTO_SEARCH_TIMEOUT_S
            while True:
                status = app.runtime.status()
                terminal_state = str(getattr(status, "state", None))
                terminal_fault = getattr(status, "fault", None)
                terminal_events = _events(app.runtime)
                if terminal_fault == "ROW_LOST":
                    terminal_outcome = "row_lost"
                    break
                if terminal_fault is not None:
                    terminal_outcome = "fault"
                    break
                if monotonic() >= deadline_s:
                    terminal_outcome = "timeout"
                    break
                sleep(POLL_S)
    except BaseException as exc:
        terminal_outcome = "exception"
        terminal_fault = f"{type(exc).__name__}: {exc}"[:2000]
        if app is not None:
            terminal_state = str(getattr(app.runtime.status(), "state", None))
            terminal_events = _events(app.runtime)
    finally:
        if app is not None:
            # These are production runtime paths: no direct motor/CAN I/O is
            # performed by this diagnostic, even after a terminal worker fault.
            for cleanup in (app.runtime.stop, app.runtime.select_manual, app.close):
                try:
                    cleanup()
                except BaseException as exc:
                    detail = f"{type(exc).__name__}: {exc}"[:2000]
                    cleanup_error = detail if cleanup_error is None else f"{cleanup_error}; {detail}"
    entries = _post_close_snapshot(app)
    payload = {
        "ok": terminal_outcome == "row_lost" and cleanup_error is None,
        "terminal_outcome": terminal_outcome,
        "request": asdict(request),
        "fixed_profile": {"motor_rpm": MOTOR_RPM, "imu_only_distance_m": IMU_ONLY_DISTANCE_M,
                          "auto_search_timeout_s": AUTO_SEARCH_TIMEOUT_S, "turns_enabled": False},
        "terminal_state": terminal_state,
        "terminal_fault": terminal_fault,
        "events": list(terminal_events),
        "cleanup_error": cleanup_error,
        "worker_diagnostics": list(entries),
    }
    report_path = _write_report(payload, report_prefix="auto_stop_ground")
    return AutoStopGroundDiagnosticResult(request, terminal_outcome, terminal_state, terminal_fault,
                                           terminal_events, cleanup_error, entries, str(report_path))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Fixed ground AUTO_SEARCH -> ROW_LOST diagnostic")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-ground-clear", action="store_true")
    parser.add_argument("--confirm-emergency-stop-ready", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_auto_stop_ground_diagnostic(AutoStopGroundDiagnosticRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000]}, ensure_ascii=False))
        return 2
    output = {"ok": result.terminal_outcome == "row_lost" and result.cleanup_error is None,
              "terminal_outcome": result.terminal_outcome, "terminal_state": result.terminal_state,
              "terminal_fault": result.terminal_fault, "cleanup_error": result.cleanup_error,
              "report_path": result.report_path}
    print(json.dumps(output, ensure_ascii=False))
    return 0 if output["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
