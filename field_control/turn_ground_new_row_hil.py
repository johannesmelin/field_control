"""Fixed, marker-triggered ground HIL for one production A4 new-row turn.

This runner is deliberately separate from the raised-wheel new-row HIL.  It
uses the normal ``AUTO_NEW_ROW_TURN`` production route, checks the asymmetric
per-wheel encoder arrival, and independently measures the physical IMU
heading change.  It is a test runner, never a geometry-calibration mechanism.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import time
from typing import Callable

from .app import FieldControlApplication
from .config import PhysicalCanConfig, RuntimeConfig, VisionConfig, Zone
from .heading import signed_angle_delta, wrap_degrees
from .odometry import DriveGeometry, OdometrySample
from .state_machine import SafetyConfig
from .turn import DifferentialTurnPlan, new_row_turn_targets
from .turn_phase_a_hil import (
    MARKER_READY_TIMEOUT_S, PHASE_A_MARKER, POLL_S, REPLY_PROFILE,
    _attach_diagnostic, _attach_prior_failure, _can_worker_diagnostic,
    _persist_last_report, _ready_marker, _safe_terminal_value, _sample, _sign,
    _terminal_error_payload, a4_target_timeout_s, runtime_failure_diagnostic,
)

CAN_CHANNEL = "can0"
GROUND_SPEED_PROFILES_RPM = (20.0, 30.0, 40.0)
GROUND_NEW_ROW_TIMEOUT_MARGIN_S = 20.0
ROW_SPACING_M = 1.20
# Test profiles only: this must not alter RuntimeConfig's production default.
ROW_SPACING_PROFILES_M = (ROW_SPACING_M, 1.50)
NEW_ROW_DIRECTION = "left"
NEW_ROW_DIRECTIONS = ("left", "right")
NUMBER_OF_ROWS = 2
HEADING_CONFIRM_TIMEOUT_S = 2.0
# The runtime's physical encoder source samples at 10 Hz.  A4 completion is
# confirmed by the CAN worker first, so the status snapshot on the same tick
# can still contain the preceding periodic 0x92 sample.  This is a
# measurement-settle bound only; motors have already been held stopped.
POST_TURN_ODOMETRY_MAX_AGE_S = 2 * POLL_S
LAST_REPORT_PATH = "/tmp/field_control-ground-new-row-hil-last-report.json"


@dataclass(frozen=True)
class GroundNewRowRequest:
    slcan_device: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_ground_clear: bool = False
    confirm_emergency_stop_ready: bool = False
    speed_profile: float = 20.0
    row_spacing_profile: float = ROW_SPACING_M
    direction: str = NEW_ROW_DIRECTION

    def validate(self) -> "GroundNewRowRequest":
        if self.enable_motors is not True:
            raise ValueError("--enable-motors krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_ground_clear is not True:
            raise ValueError("--confirm-ground-clear krävs")
        if self.confirm_emergency_stop_ready is not True:
            raise ValueError("--confirm-emergency-stop-ready krävs")
        if self.speed_profile not in GROUND_SPEED_PROFILES_RPM:
            raise ValueError("endast fasta marktestprofiler 20, 30 eller 40 RPM är tillåtna")
        if self.row_spacing_profile not in ROW_SPACING_PROFILES_M:
            raise ValueError("endast fast radavståndsprofil 1,20 eller explicit 1,50 m är tillåten")
        if self.direction not in NEW_ROW_DIRECTIONS:
            raise ValueError("new-row-riktning måste vara left eller right")
        prefix = "/dev/serial/by-id/"
        name = (self.slcan_device[len(prefix):] if isinstance(self.slcan_device, str)
                and self.slcan_device.startswith(prefix) else "")
        if not name or name in (".", "..") or "/" in name:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class GroundNewRowResult:
    plan: DifferentialTurnPlan
    encoder_delta_m: tuple[float, float]
    initial_heading_deg: float
    final_heading_deg: float
    heading_delta_deg: float
    completed_state: str


def ground_new_row_config(request: GroundNewRowRequest) -> RuntimeConfig:
    """Build one fixed ground profile; the CLI exposes no free motion values."""
    geometry = DriveGeometry()
    speed = float(request.speed_profile)
    row_spacing_m = float(request.row_spacing_profile)
    plan = new_row_turn_targets(geometry, row_spacing_m, speed, request.direction)
    largest_wheel_degrees = max(
        abs(plan.left_distance_m / geometry.left_wheel_circumference_m * 360.0),
        abs(plan.right_distance_m / geometry.right_wheel_circumference_m * 360.0),
    )
    timeout_s = a4_target_timeout_s(largest_wheel_degrees, speed,
                                    geometry.motor_turns_per_wheel_turn,
                                    timeout_margin_s=GROUND_NEW_ROW_TIMEOUT_MARGIN_S)
    return RuntimeConfig(
        stream_enabled=False, max_rpm=speed, auto_base_rpm=0.0,
        max_vision_correction_rpm=0.0, vision_kp=0.0, search_speed_rpm=0.0,
        heading_kp=0.0, max_heading_correction_rpm=0.0,
        turn_speed_rpm=speed, navigation_frame_rate_hz=20.0, row_spacing_m=row_spacing_m,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device,
                                       True, False, True, True, True),
        vision=VisionConfig(navigation_mode="buds_only", buds=PHASE_A_MARKER,
                            leaves=PHASE_A_MARKER, marker=PHASE_A_MARKER,
                            turn_marker_zone=Zone(.2, .8, .3, 1.0)),
        odometry_geometry=geometry,
        safety=SafetyConfig(in_row_turn_enabled=False, new_row_turn_direction=request.direction,
                            number_of_rows=NUMBER_OF_ROWS, auto_start_delay_s=0.0,
                            turn_timeout_s=timeout_s),
    ).validate()


def _fresh_heading(runtime: object) -> tuple[float, float]:
    """Return a finite filtered heading and its source timestamp."""
    observation = getattr(runtime.status(), "observation", None)
    heading = getattr(observation, "heading_deg", None)
    if (not bool(getattr(observation, "imu_fresh", False)) or isinstance(heading, bool)
            or not isinstance(heading, (int, float)) or not math.isfinite(heading)):
        raise RuntimeError("GROUND_NEW_ROW_HEADING_STALE")
    source = getattr(runtime, "imu", None)
    snapshot = getattr(source, "snapshot", None)
    if callable(snapshot):
        value = snapshot()
        timestamp = getattr(value, "updated_at_s", None)
        if (not bool(getattr(value, "connected", False)) or isinstance(timestamp, bool)
                or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp)):
            raise RuntimeError("GROUND_NEW_ROW_HEADING_STALE")
        return float(heading), float(timestamp)
    now_s = getattr(observation, "now_s", None)
    if isinstance(now_s, bool) or not isinstance(now_s, (int, float)) or not math.isfinite(now_s):
        raise RuntimeError("GROUND_NEW_ROW_HEADING_STALE")
    return float(heading), float(now_s)


def _validate_completion(status: object, plan: DifferentialTurnPlan,
                         baseline: OdometrySample, events: tuple[dict[str, object], ...],
                         tolerance_m: float) -> tuple[float, float]:
    if getattr(status, "fault", None) is not None:
        raise RuntimeError(f"A4-mark-new-row avslutades med fel: {getattr(status, 'fault', None)}")
    if getattr(status, "state", None) not in ("AUTO_ROW_FOLLOW", "AUTO_SEARCH", "AUTO_COMPLETE"):
        raise RuntimeError("A4-mark-new-row nådde inget normalt terminalstate")
    final = _sample(status)
    delta = (final.left_distance_m - baseline.left_distance_m,
             final.right_distance_m - baseline.right_distance_m)
    expected = (plan.left_distance_m, plan.right_distance_m)
    if (not all(math.isfinite(value) for value in delta)
            or tuple(_sign(value) for value in delta) != tuple(_sign(value) for value in expected)):
        raise RuntimeError("mark-new-row encoderdelta är inte teckenkonsistent med A4-turn-planen")
    if any(abs(actual - target) > tolerance_m for actual, target in zip(delta, expected)):
        raise RuntimeError("mark-new-row encoderdelta når inte A4-målet inom konfigurerad turn-tolerans")
    kinds = [entry.get("kind") for entry in events]
    try:
        started = kinds.index("turn_started")
        completed = kinds.index("turn_completed", started + 1)
    except ValueError as exc:
        raise RuntimeError("saknar ordnade runtime-event turn_started följt av turn_completed") from exc
    for index in (started, completed):
        data = events[index].get("data")
        if not isinstance(data, dict) or data.get("state") != "AUTO_NEW_ROW_TURN":
            raise RuntimeError("runtime-event bekräftar inte AUTO_NEW_ROW_TURN")
    return delta


def _odometry_snapshot_needs_settle(status: object) -> bool:
    """Whether a physical status still exposes a pre-completion 0x92 sample."""
    observation = getattr(status, "observation", None)
    age_s = getattr(observation, "odometry_age_s", None)
    return (isinstance(age_s, (int, float)) and not isinstance(age_s, bool)
            and math.isfinite(age_s) and age_s > POST_TURN_ODOMETRY_MAX_AGE_S)


def _success_payload(result: GroundNewRowResult, *, speed_profile: float,
                     row_spacing_profile: float, direction: str) -> dict[str, object]:
    return {"ok": True, "speed_profile_motor_rpm": speed_profile,
            "row_spacing_profile_m": row_spacing_profile,
            "direction": direction,
            "completed_state": result.completed_state,
            "encoder_delta_m": result.encoder_delta_m,
            "heading": {"initial_deg": result.initial_heading_deg,
                        "final_deg": result.final_heading_deg,
                        "delta_deg": result.heading_delta_deg}}


def run_ground_new_row(request: GroundNewRowRequest, *,
                       app_factory: Callable[..., FieldControlApplication] = FieldControlApplication,
                       monotonic: Callable[[], float] = time.monotonic,
                       sleep: Callable[[float], None] = time.sleep) -> GroundNewRowResult:
    """Run one normal marker-triggered production ``AUTO_NEW_ROW_TURN``."""
    request.validate()
    config = ground_new_row_config(request)
    plan = new_row_turn_targets(config.odometry_geometry, config.row_spacing_m,
                                config.turn_speed_rpm, request.direction)
    if math.isclose(abs(plan.left_distance_m), abs(plan.right_distance_m)):
        raise RuntimeError("mark-new-row kräver asymmetriska geometrimål")
    app: FieldControlApplication | None = None
    error: BaseException | None = None
    result: GroundNewRowResult | None = None
    try:
        app = app_factory(config)
        app.start()
        marker_deadline = monotonic() + MARKER_READY_TIMEOUT_S
        while monotonic() < marker_deadline:
            if _ready_marker(app.runtime.status()):
                break
            sleep(POLL_S)
        else:
            raise TimeoutError("GROUND_NEW_ROW_MARKER_NOT_READY")
        initial_heading, heading_timestamp = _fresh_heading(app.runtime)
        app.runtime.select_auto()
        app.runtime.arm_motor_output()
        baseline = _sample(app.runtime.status())
        app.runtime.start_auto()
        admission_deadline = monotonic() + config.safety.turn_timeout_s
        turn_deadline: float | None = None
        complete = False
        completion_odometry_deadline: float | None = None
        heading_deadline: float | None = None
        confirmations = 0
        final_heading: float | None = None
        delta: tuple[float, float] | None = None
        while True:
            now_s = monotonic()
            status = app.runtime.status()
            if status.fault is not None:
                raise RuntimeError(f"A4-mark-new-row avslutades med fel: {status.fault}")
            events = tuple(app.runtime.events.recent())
            kinds = [entry.get("kind") for entry in events]
            if turn_deadline is None:
                if "turn_started" in kinds:
                    turn_deadline = now_s + config.safety.turn_timeout_s
                elif now_s >= admission_deadline:
                    raise TimeoutError("mark-new-row registrerade inte turn_started inom A4-måldeadline")
            if not complete and "turn_completed" in kinds:
                if _odometry_snapshot_needs_settle(status):
                    if completion_odometry_deadline is None:
                        completion_odometry_deadline = now_s + config.odometry_timeout_s
                    if now_s >= completion_odometry_deadline:
                        raise TimeoutError("GROUND_NEW_ROW_POST_TURN_ODOMETRY_CONFIRM_TIMEOUT")
                    sleep(POLL_S)
                    continue
                delta = _validate_completion(status, plan, baseline, events,
                                             config.safety.turn_distance_tolerance_m)
                complete, heading_deadline = True, now_s + HEADING_CONFIRM_TIMEOUT_S
            if complete:
                heading, timestamp = _fresh_heading(app.runtime)
                if timestamp > heading_timestamp:
                    expected = wrap_degrees(initial_heading + 180.0)
                    heading_error = signed_angle_delta(expected, heading)
                    if abs(heading_error) > config.safety.turn_heading_tolerance_deg:
                        raise RuntimeError(
                            "GROUND_NEW_ROW_HEADING_OUT_OF_TOLERANCE: "
                            f"initial={initial_heading:.2f}, expected={expected:.2f}, actual={heading:.2f}, "
                            f"delta={signed_angle_delta(heading, initial_heading):.2f}, "
                            f"error={heading_error:.2f} deg")
                    heading_timestamp, final_heading = timestamp, heading
                    confirmations += 1
                    if confirmations >= config.safety.turn_heading_confirm_frames:
                        app.runtime.select_manual()
                        terminal = app.runtime.status()
                        if terminal.fault is not None or terminal.motor_output_armed or terminal.state != "MANUAL":
                            raise RuntimeError("mark-new-row avslutades inte säkert disarmerad i MANUAL")
                        assert delta is not None and final_heading is not None
                        result = GroundNewRowResult(plan, delta, initial_heading, final_heading,
                                                    signed_angle_delta(final_heading, initial_heading),
                                                    status.state)
                        break
                if heading_deadline is not None and now_s >= heading_deadline:
                    raise TimeoutError("GROUND_NEW_ROW_HEADING_CONFIRM_TIMEOUT")
            if turn_deadline is not None and now_s >= turn_deadline:
                raise TimeoutError("mark-new-row nådde inte exakt A4-måldeadline")
            sleep(POLL_S)
    except BaseException as exc:
        error = exc
        if app is not None:
            try:
                _attach_diagnostic(error, runtime_failure_diagnostic(app.runtime))
            except Exception:
                pass
    finally:
        if app is not None:
            try:
                app.runtime.select_manual()
            except BaseException as stop_exc:
                if error is None:
                    error = stop_exc
                else:
                    _attach_prior_failure(stop_exc, error); error = stop_exc
            try:
                app.close()
            except BaseException as close_exc:
                if error is None:
                    error = close_exc
                else:
                    _attach_prior_failure(close_exc, error); error = close_exc
            if error is not None:
                try:
                    _attach_diagnostic(error, {"can_worker_post_close": _can_worker_diagnostic(app.runtime, after_close=True)})
                except Exception:
                    pass
    if error is not None:
        raise error
    assert result is not None
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Ground A4 AUTO_NEW_ROW_TURN HIL (fixed 20 RPM; explicit 30/40 RPM and 1.50 m C/C profiles)")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-ground-clear", action="store_true")
    parser.add_argument("--confirm-emergency-stop-ready", action="store_true")
    parser.add_argument("--speed-profile", type=float, choices=GROUND_SPEED_PROFILES_RPM, default=20.0)
    parser.add_argument("--row-spacing-profile", type=float, choices=ROW_SPACING_PROFILES_M,
                        default=ROW_SPACING_M, metavar="M")
    parser.add_argument("--direction", choices=NEW_ROW_DIRECTIONS, default=NEW_ROW_DIRECTION)
    args = parser.parse_args(argv)
    request = GroundNewRowRequest(**vars(args))
    try:
        result = run_ground_new_row(request)
    except Exception as exc:
        output = _safe_terminal_value(_terminal_error_payload(exc)); assert isinstance(output, dict)
        output["row_spacing_profile_m"] = request.row_spacing_profile
        output["speed_profile_motor_rpm"] = request.speed_profile
        output["direction"] = request.direction
        persistence_error = _persist_last_report(output, path=LAST_REPORT_PATH)
        if persistence_error is not None: output["report_persistence_error"] = persistence_error
        print(json.dumps(output, allow_nan=False)); return 2
    output = _success_payload(result, speed_profile=request.speed_profile,
                              row_spacing_profile=request.row_spacing_profile,
                              direction=request.direction)
    persistence_error = _persist_last_report(output, path=LAST_REPORT_PATH)
    if persistence_error is not None: output["report_persistence_error"] = persistence_error
    print(json.dumps(output, allow_nan=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
