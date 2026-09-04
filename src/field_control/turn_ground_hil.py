"""Fixed, marker-triggered ground HIL for one production A4 in-row turn.

This is deliberately separate from the raised-wheel routines: it requires a
fully explicit ground operating context and proves the real IMU heading change
as well as the worker-owned A4 encoder target.  It is a measurement runner,
not a calibration tool; it reports the observed heading delta.
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
from .turn import DifferentialTurnPlan, in_row_turn_plan
from .turn_phase_a_hil import (
    MARKER_READY_TIMEOUT_S, PHASE_A_MARKER, POLL_S, REPLY_PROFILE, _attach_diagnostic,
    _attach_prior_failure, _can_worker_diagnostic, _persist_last_report, _ready_marker,
    _sample, _sign, _terminal_error_payload, _safe_terminal_value, a4_target_timeout_s,
    runtime_failure_diagnostic,
)

CAN_CHANNEL = "can0"
GROUND_SPEED_PROFILES_RPM = (20.0, 40.0)
GROUND_TIMEOUT_MARGIN_S = 20.0
IN_ROW_TURN_WHEEL_DEGREES = 720.0
LAST_REPORT_PATH = "/tmp/field_control-ground-turn-hil-last-report.json"
HEADING_CONFIRM_TIMEOUT_S = 2.0


@dataclass(frozen=True)
class GroundTurnRequest:
    slcan_device: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_ground_clear: bool = False
    confirm_emergency_stop_ready: bool = False
    speed_profile: float = 20.0

    def validate(self) -> "GroundTurnRequest":
        if self.enable_motors is not True:
            raise ValueError("--enable-motors krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_ground_clear is not True:
            raise ValueError("--confirm-ground-clear krävs")
        if self.confirm_emergency_stop_ready is not True:
            raise ValueError("--confirm-emergency-stop-ready krävs")
        if self.speed_profile not in GROUND_SPEED_PROFILES_RPM:
            raise ValueError("endast fast marktestprofil 20 eller explicit 40 RPM är tillåten")
        prefix = "/dev/serial/by-id/"
        name = (self.slcan_device[len(prefix):] if isinstance(self.slcan_device, str)
                and self.slcan_device.startswith(prefix) else "")
        if not name or name in (".", "..") or "/" in name:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class GroundTurnResult:
    plan: DifferentialTurnPlan
    encoder_delta_m: tuple[float, float]
    initial_heading_deg: float
    final_heading_deg: float
    heading_delta_deg: float
    completed_state: str


def ground_turn_config(request: GroundTurnRequest) -> RuntimeConfig:
    """Build one of the two fixed ground profiles; no arbitrary motion CLI."""
    geometry = DriveGeometry()
    speed = float(request.speed_profile)
    plan = in_row_turn_plan(geometry, IN_ROW_TURN_WHEEL_DEGREES, "left")
    largest_wheel_degrees = max(abs(plan.left_distance_m / geometry.left_wheel_circumference_m * 360.0),
                                abs(plan.right_distance_m / geometry.right_wheel_circumference_m * 360.0))
    timeout_s = a4_target_timeout_s(largest_wheel_degrees, speed,
                                    geometry.motor_turns_per_wheel_turn,
                                    timeout_margin_s=GROUND_TIMEOUT_MARGIN_S)
    return RuntimeConfig(
        stream_enabled=False, max_rpm=speed, auto_base_rpm=0.0,
        max_vision_correction_rpm=0.0, vision_kp=0.0, search_speed_rpm=0.0,
        heading_kp=0.0, max_heading_correction_rpm=0.0,
        turn_speed_rpm=speed, navigation_frame_rate_hz=20.0,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device,
                                       True, False, True, True, True),
        vision=VisionConfig(navigation_mode="buds_only", buds=PHASE_A_MARKER,
                            leaves=PHASE_A_MARKER, marker=PHASE_A_MARKER,
                            turn_marker_zone=Zone(.2, .8, .3, 1.0)),
        odometry_geometry=geometry,
        safety=SafetyConfig(in_row_turn_enabled=True, new_row_turn_direction="left",
                            in_row_turn_wheel_degrees=IN_ROW_TURN_WHEEL_DEGREES,
                            auto_start_delay_s=0.0, turn_timeout_s=timeout_s),
    ).validate()


def _fresh_heading(runtime: object) -> tuple[float, float]:
    """Return the current finite filtered heading and a fresh source timestamp."""
    status = runtime.status()
    observation = getattr(status, "observation", None)
    heading = getattr(observation, "heading_deg", None)
    now_s = getattr(observation, "now_s", None)
    if (not bool(getattr(observation, "imu_fresh", False)) or isinstance(heading, bool)
            or not isinstance(heading, (int, float)) or not math.isfinite(heading)):
        raise RuntimeError("GROUND_TURN_HEADING_STALE")
    source = getattr(runtime, "imu", None)
    snapshot = getattr(source, "snapshot", None)
    if callable(snapshot):
        value = snapshot()
        timestamp = getattr(value, "updated_at_s", None)
        if (not bool(getattr(value, "connected", False)) or isinstance(timestamp, bool)
                or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp)):
            raise RuntimeError("GROUND_TURN_HEADING_STALE")
        return float(heading), float(timestamp)
    # Mock-only compatibility; production OAK source always supplies snapshot.
    if not isinstance(now_s, (int, float)) or isinstance(now_s, bool) or not math.isfinite(now_s):
        raise RuntimeError("GROUND_TURN_HEADING_STALE")
    return float(heading), float(now_s)


def _validate_completion(status: object, plan: DifferentialTurnPlan,
                         baseline: OdometrySample, tolerance_m: float) -> tuple[float, float]:
    if getattr(status, "fault", None) is not None:
        raise RuntimeError(f"A4-markvändning avslutades med fel: {getattr(status, 'fault', None)}")
    if getattr(status, "state", None) not in ("AUTO_ROW_FOLLOW", "AUTO_SEARCH", "AUTO_COMPLETE"):
        raise RuntimeError("A4-markvändning nådde inget normalt terminalstate")
    final = _sample(status)
    delta = (final.left_distance_m - baseline.left_distance_m,
             final.right_distance_m - baseline.right_distance_m)
    expected = (plan.left_distance_m, plan.right_distance_m)
    if (not all(math.isfinite(value) for value in delta)
            or tuple(_sign(value) for value in delta) != tuple(_sign(value) for value in expected)):
        raise RuntimeError("marktestets encoderdelta är inte teckenkonsistent med A4-turn-planen")
    if any(abs(actual - target) > tolerance_m for actual, target in zip(delta, expected)):
        raise RuntimeError("marktestets encoderdelta når inte A4-målet inom konfigurerad turn-tolerans")
    return delta


def _terminal_success_payload(result: GroundTurnResult, *, speed_profile: float) -> dict[str, object]:
    return {"ok": True, "speed_profile_motor_rpm": speed_profile,
            "completed_state": result.completed_state,
            "encoder_delta_m": result.encoder_delta_m,
            "heading": {"initial_deg": result.initial_heading_deg,
                        "final_deg": result.final_heading_deg,
                        "delta_deg": result.heading_delta_deg}}


def run_ground_turn(request: GroundTurnRequest, *,
                    app_factory: Callable[..., FieldControlApplication] = FieldControlApplication,
                    monotonic: Callable[[], float] = time.monotonic,
                    sleep: Callable[[float], None] = time.sleep) -> GroundTurnResult:
    """Run one marker-triggered A4 target and independently prove ground heading."""
    request.validate()
    config = ground_turn_config(request)
    plan = in_row_turn_plan(config.odometry_geometry, config.safety.in_row_turn_wheel_degrees,
                            config.safety.new_row_turn_direction)
    app: FieldControlApplication | None = None
    error: BaseException | None = None
    result: GroundTurnResult | None = None
    try:
        app = app_factory(config)
        app.start()
        marker_deadline = monotonic() + MARKER_READY_TIMEOUT_S
        while monotonic() < marker_deadline:
            if _ready_marker(app.runtime.status()):
                break
            sleep(POLL_S)
        else:
            raise TimeoutError("GROUND_TURN_MARKER_NOT_READY")
        initial_heading, initial_timestamp = _fresh_heading(app.runtime)
        app.runtime.select_auto()
        app.runtime.arm_motor_output()
        baseline = _sample(app.runtime.status())
        app.runtime.start_auto()
        start_deadline = monotonic() + config.safety.turn_timeout_s
        turn_deadline: float | None = None
        completed = False
        heading_confirmations = 0
        last_heading_timestamp = initial_timestamp
        final_heading = None
        heading_deadline = None
        while True:
            now_s = monotonic()
            status = app.runtime.status()
            if status.fault is not None:
                raise RuntimeError(f"A4-markvändning avslutades med fel: {status.fault}")
            events = tuple(app.runtime.events.recent())
            kinds = [entry.get("kind") for entry in events]
            if turn_deadline is None:
                if "turn_started" in kinds:
                    turn_deadline = now_s + config.safety.turn_timeout_s
                elif now_s >= start_deadline:
                    raise TimeoutError("marktest registrerade inte turn_started inom A4-måldeadline")
            if not completed and "turn_completed" in kinds:
                delta = _validate_completion(status, plan, baseline,
                                             config.safety.turn_distance_tolerance_m)
                completed = True
                heading_deadline = now_s + HEADING_CONFIRM_TIMEOUT_S
            if completed:
                heading, timestamp = _fresh_heading(app.runtime)
                if timestamp > last_heading_timestamp:
                    expected = wrap_degrees(initial_heading + 180.0)
                    error_deg = signed_angle_delta(expected, heading)
                    if abs(error_deg) > config.safety.turn_heading_tolerance_deg:
                        raise RuntimeError(
                            "GROUND_TURN_HEADING_OUT_OF_TOLERANCE: "
                            f"initial={initial_heading:.2f}, expected={expected:.2f}, "
                            f"actual={heading:.2f}, delta={signed_angle_delta(heading, initial_heading):.2f}, "
                            f"error={error_deg:.2f} deg"
                        )
                    last_heading_timestamp = timestamp
                    final_heading = heading
                    heading_confirmations += 1
                    if heading_confirmations >= config.safety.turn_heading_confirm_frames:
                        app.runtime.select_manual()
                        terminal = app.runtime.status()
                        if terminal.fault is not None or terminal.motor_output_armed or terminal.state != "MANUAL":
                            raise RuntimeError("marktest avslutades inte säkert disarmerad i MANUAL")
                        assert final_heading is not None
                        result = GroundTurnResult(plan, delta, initial_heading, final_heading,
                                                  signed_angle_delta(final_heading, initial_heading),
                                                  status.state)
                        break
                if heading_deadline is not None and now_s >= heading_deadline:
                    raise TimeoutError("GROUND_TURN_HEADING_CONFIRM_TIMEOUT")
            if turn_deadline is not None and now_s >= turn_deadline:
                raise TimeoutError("marktest nådde inte exakt A4-måldeadline")
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
    parser = argparse.ArgumentParser(description="Ground A4 AUTO_IN_ROW_TURN HIL (fixed 20 RPM; explicit 40 profile)")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-ground-clear", action="store_true")
    parser.add_argument("--confirm-emergency-stop-ready", action="store_true")
    parser.add_argument("--speed-profile", type=float, choices=GROUND_SPEED_PROFILES_RPM, default=20.0)
    args = parser.parse_args(argv)
    request = GroundTurnRequest(**vars(args))
    try:
        result = run_ground_turn(request)
    except Exception as exc:
        output = _safe_terminal_value(_terminal_error_payload(exc)); assert isinstance(output, dict)
        persistence_error = _persist_last_report(output, path=LAST_REPORT_PATH)
        if persistence_error is not None: output["report_persistence_error"] = persistence_error
        print(json.dumps(output, allow_nan=False)); return 2
    output = _terminal_success_payload(result, speed_profile=request.speed_profile)
    persistence_error = _persist_last_report(output, path=LAST_REPORT_PATH)
    if persistence_error is not None: output["report_persistence_error"] = persistence_error
    print(json.dumps(output, allow_nan=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
