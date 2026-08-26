"""Fixed longer raised-wheel observation of the normal in-row turn.

This is deliberately separate from :mod:`turn_phase_a_hil`: the original
two-second Phase-A timeout test remains the short safety check.  This runner
only gives an operator a longer view of the same fail-closed path.  Because
the wheels are raised, ``TURN_TIMEOUT`` (and a disarmed output) remains the
only accepted terminal outcome; it is neither a successful turn nor a
calibration.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import time
from typing import Callable, Mapping

from .app import FieldControlApplication
from .config import HsvFilter, PhysicalCanConfig, RuntimeConfig, VisionConfig, Zone
from .odometry import OdometrySample, motor_rpm_to_wheel_rpm
from .state_machine import SafetyConfig
from .turn import DifferentialTurnPlan, in_row_turn_plan
from .turn_phase_a_hil import PHASE_A_MARKER


CAN_CHANNEL = "can0"
REPLY_PROFILE = "observed-rmdx-same-id"
MARKER_READY_TIMEOUT_S = 30.0
# Fixed motor-side values.  The CLI deliberately exposes neither of them.
TURN_TIMEOUT_S = 30.0
TURN_SPEED_MOTOR_RPM = 2.0
POLL_S = .020
# At the configured 8:1 reduction and 0.805 m wheel circumference, 2 motor
# RPM for 30 s is 0.100625 m per wheel.  The acceptance interval is deliberately
# broad (80--120%) because this is an uncalibrated raised-wheel observation,
# but it still rejects a missing/near-stationary wheel and a surprising travel
# that could approach the inherited 1.61 m target.
MIN_NOMINAL_TRAVEL_RATIO = .80
MAX_NOMINAL_TRAVEL_RATIO = 1.20
_DIAGNOSTIC_EVENT_LIMIT = 16
_DIAGNOSTIC_TEXT_LIMIT = 1000
_DIAGNOSTIC_WORKER_LIMIT = 6000
_DIAGNOSTIC_EVENT_DATA_LIMIT = 8
_DIAGNOSTIC_EVENT_KEY_LIMIT = 128
_DIAGNOSTIC_EVENT_VALUE_LIMIT = 320


@dataclass(frozen=True)
class TurnPhaseALongRequest:
    slcan_device: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False
    confirm_turn_not_calibrated: bool = False

    def validate(self) -> "TurnPhaseALongRequest":
        if self.enable_motors is not True:
            raise ValueError("--enable-motors krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_wheels_raised is not True:
            raise ValueError("--confirm-wheels-raised krävs")
        if self.confirm_turn_not_calibrated is not True:
            raise ValueError("--confirm-turn-not-calibrated krävs")
        prefix = "/dev/serial/by-id/"
        basename = (self.slcan_device[len(prefix):] if isinstance(self.slcan_device, str)
                    and self.slcan_device.startswith(prefix) else "")
        if not basename or basename in (".", "..") or "/" in basename:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class TurnPhaseALongResult:
    plan: DifferentialTurnPlan
    command_sign: tuple[int, int]
    encoder_delta_m: tuple[float, float]
    nominal_wheel_travel_m: tuple[float, float]
    fault: str
    events: tuple[dict[str, object], ...]


def phase_a_long_config(request: TurnPhaseALongRequest) -> RuntimeConfig:
    """The sole longer observation profile; production settings are untouched."""
    return RuntimeConfig(
        stream_enabled=False, max_rpm=TURN_SPEED_MOTOR_RPM,
        auto_base_rpm=0.0, max_vision_correction_rpm=0.0, vision_kp=0.0,
        search_speed_rpm=0.0, heading_kp=0.0, max_heading_correction_rpm=0.0,
        turn_speed_rpm=TURN_SPEED_MOTOR_RPM, navigation_frame_rate_hz=20.0,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
        vision=VisionConfig(navigation_mode="buds_only", buds=PHASE_A_MARKER,
                            leaves=PHASE_A_MARKER, marker=PHASE_A_MARKER,
                            turn_marker_zone=Zone(.2, .8, .3, 1.0)),
        safety=SafetyConfig(in_row_turn_enabled=True, new_row_turn_direction="left",
                            auto_start_delay_s=0.0, turn_timeout_s=TURN_TIMEOUT_S),
    ).validate()


def _ready_marker(status: object) -> bool:
    observation = getattr(status, "observation", None)
    vision = getattr(observation, "vision", None)
    return bool(getattr(observation, "camera_fresh", False) and getattr(observation, "imu_fresh", False)
                and getattr(observation, "odometry_fresh", False) and getattr(vision, "marker_found", False)
                and getattr(observation, "visual_target", False))


def _sample(status: object) -> OdometrySample:
    value = getattr(getattr(status, "observation", None), "odometry_sample", None)
    if type(value) is not OdometrySample:
        raise RuntimeError("färsk per-hjulsodometri saknas")
    return value


def _sign(value: float) -> int:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value == 0:
        raise RuntimeError("turn-kommando måste ha ändliga icke-noll hjulkomponenter")
    return 1 if value > 0 else -1


def nominal_wheel_travel_m(config: RuntimeConfig) -> tuple[float, float]:
    """Fixed profile distance, calculated from the configured gearbox geometry."""
    wheel_rpm = motor_rpm_to_wheel_rpm(TURN_SPEED_MOTOR_RPM, config.odometry_geometry)
    minutes = TURN_TIMEOUT_S / 60.0
    return (wheel_rpm * minutes * config.odometry_geometry.left_wheel_circumference_m,
            wheel_rpm * minutes * config.odometry_geometry.right_wheel_circumference_m)


def _odometry_value(value: object) -> dict[str, object] | None:
    """Format only the immutable sample shape expected from the source."""
    if type(value) is not OdometrySample:
        return None
    return {
        "left_distance_m": _diagnostic_scalar(value.left_distance_m),
        "right_distance_m": _diagnostic_scalar(value.right_distance_m),
        "forward_distance_m": _diagnostic_scalar(value.forward_distance_m),
        "yaw_change_deg": _diagnostic_scalar(value.yaw_change_deg),
    }


def _diagnostic_scalar(value: object, *, text_limit: int = _DIAGNOSTIC_TEXT_LIMIT) -> object:
    """Return a JSON-safe scalar with bounded text representation."""
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value if len(repr(value)) <= text_limit else repr(value)[:text_limit]
    if isinstance(value, float):
        return value if math.isfinite(value) else repr(value)
    if isinstance(value, str):
        return value[:text_limit]
    return repr(value)[:text_limit]


def _diagnostic_event_data(data: object) -> object:
    """Bound keys and values even if a nonstandard event producer is used."""
    if not isinstance(data, Mapping):
        return _diagnostic_scalar(data)
    result: dict[str, object] = {}
    for index, (key, value) in enumerate(data.items()):
        if index >= _DIAGNOSTIC_EVENT_DATA_LIMIT:
            break
        text_key = (key if isinstance(key, str) else repr(key))[:_DIAGNOSTIC_EVENT_KEY_LIMIT]
        result[text_key] = _diagnostic_scalar(value, text_limit=_DIAGNOSTIC_EVENT_VALUE_LIMIT)
    return result


def _bounded_events(runtime: object) -> list[dict[str, object]]:
    """Copy the tail of the runtime journal without touching control state."""
    events = getattr(runtime, "events", None)
    recent = getattr(events, "recent", None)
    if not callable(recent):
        return []
    try:
        raw_events = recent()
    except Exception as exc:
        return [{"kind": "diagnostics_error", "data": f"{type(exc).__name__}: {exc}"[:_DIAGNOSTIC_TEXT_LIMIT]}]
    # EventLog.recent() is specified to return a finite list.  Refuse a
    # substitute iterable rather than consuming it: a generator could be
    # unbounded, and diagnostics must never delay a safe terminal close.
    if not isinstance(raw_events, (list, tuple)):
        return [{"kind": "diagnostics_error", "data": "runtime events har ogiltig icke-sekvens-typ"}]
    selected = raw_events[-_DIAGNOSTIC_EVENT_LIMIT:]
    result: list[dict[str, object]] = []
    for event in selected:
        if not isinstance(event, dict):
            result.append({"event": repr(event)[:_DIAGNOSTIC_TEXT_LIMIT]})
            continue
        data = event.get("data")
        result.append({
            "timestamp_s": _diagnostic_scalar(event.get("timestamp_s")),
            "level": _diagnostic_scalar(event.get("level")),
            "kind": _diagnostic_scalar(event.get("kind")),
            "data": _diagnostic_event_data(data),
        })
    return result


def _runtime_diagnostics(runtime: object, now_s: float) -> dict[str, object]:
    """Read a bounded terminal snapshot; this does not issue CAN traffic."""
    status = runtime.status()
    command = getattr(status, "last_command", None)
    source = getattr(runtime, "_odometry", None)
    snapshot_method = getattr(source, "snapshot", None)
    odometry: dict[str, object] = {"available": False}
    if callable(snapshot_method):
        try:
            snapshot = snapshot_method()
            age = getattr(snapshot, "age_s", None)
            odometry = {
                "available": True,
                "connected": bool(getattr(snapshot, "connected", False)),
                "error": _diagnostic_scalar(getattr(snapshot, "error", None)),
                "age_s": _diagnostic_scalar(age(now_s) if callable(age) else None),
                "value": _odometry_value(getattr(snapshot, "value", None)),
            }
        except Exception as exc:
            odometry = {"available": True, "snapshot_error": _diagnostic_scalar(f"{type(exc).__name__}: {exc}")}
    return {
        "runtime": {
            "mode": _diagnostic_scalar(getattr(status, "mode", None)),
            "state": _diagnostic_scalar(getattr(status, "state", None)),
            "fault": _diagnostic_scalar(getattr(status, "fault", None)),
            "motor_output_armed": bool(getattr(status, "motor_output_armed", False)),
            "last_command": None if command is None else {
                "left_rpm": _diagnostic_scalar(getattr(command, "left_rpm", None)),
                "right_rpm": _diagnostic_scalar(getattr(command, "right_rpm", None)),
                "source": _diagnostic_scalar(getattr(command, "source", None)),
            },
        },
        "odometry": odometry,
        "events": _bounded_events(runtime),
    }


def _worker_diagnostics_after_close(runtime: object) -> dict[str, object]:
    """Read the verified worker's released ring; never reopen or command CAN."""
    motor = getattr(runtime, "motor", None)
    sink = getattr(motor, "_sink", motor)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if not callable(snapshot):
        return {"available": False}
    try:
        return {"available": True, "worker": repr(snapshot())[:_DIAGNOSTIC_WORKER_LIMIT]}
    except Exception as exc:
        return {"available": True, "worker_error": f"{type(exc).__name__}: {exc}"[:_DIAGNOSTIC_TEXT_LIMIT]}


def _validate_timeout_evidence(status: object, plan: DifferentialTurnPlan, baseline: OdometrySample,
                               nominal: tuple[float, float], events: tuple[dict[str, object], ...]) -> tuple[tuple[int, int], tuple[float, float]]:
    if getattr(status, "fault", None) != "TURN_TIMEOUT" or getattr(status, "motor_output_armed", True):
        raise RuntimeError(f"lång Phase-A måste fail-closed med TURN_TIMEOUT, fick {getattr(status, 'fault', None)}")
    command = getattr(status, "last_command", None)
    if command is None or getattr(command, "source", None) != "turn":
        raise RuntimeError("normal turn-command observerades inte före timeout")
    signs = (_sign(command.left_rpm), _sign(command.right_rpm))
    expected = (_sign(plan.left_ratio), _sign(plan.right_ratio))
    if signs != expected:
        raise RuntimeError("observerade turn-kommandotecken matchar inte turn-planen")
    final = _sample(status)
    delta = (final.left_distance_m - baseline.left_distance_m,
             final.right_distance_m - baseline.right_distance_m)
    if (not all(math.isfinite(value) for value in delta)
            or tuple(_sign(value) for value in delta) != expected):
        raise RuntimeError("encoderdeltan är inte ändlig och teckenkonsistent med turn-planen")
    for value, expected_distance in zip(delta, nominal):
        if not MIN_NOMINAL_TRAVEL_RATIO * expected_distance <= abs(value) <= MAX_NOMINAL_TRAVEL_RATIO * expected_distance:
            raise RuntimeError("encoderdeltan ligger utanför långprofilens 80--120 procent av nominella 10 cm")
    kinds = [entry.get("kind") for entry in events]
    try:
        turn_index = kinds.index("turn_started")
        fault_index = kinds.index("fault", turn_index + 1)
    except ValueError as exc:
        raise RuntimeError("saknar ordnade runtime-event turn_started följt av fault") from exc
    fault_data = events[fault_index].get("data")
    if not isinstance(fault_data, dict) or fault_data.get("reason") != "TURN_TIMEOUT":
        raise RuntimeError("fault-event bekräftar inte TURN_TIMEOUT")
    return signs, delta


def run_turn_phase_a_long(request: TurnPhaseALongRequest, *, app_factory: Callable[..., FieldControlApplication] = FieldControlApplication,
                          monotonic: Callable[[], float] = time.monotonic,
                          sleep: Callable[[float], None] = time.sleep) -> TurnPhaseALongResult:
    """Run only the ordinary AUTO path and require its bounded timeout stop."""
    request.validate()
    config = phase_a_long_config(request)
    plan = in_row_turn_plan(config.odometry_geometry, config.safety.in_row_turn_wheel_degrees,
                            config.safety.new_row_turn_direction)
    nominal = nominal_wheel_travel_m(config)
    app: FieldControlApplication | None = None
    error: BaseException | None = None
    failure_diagnostics: dict[str, object] | None = None
    result: TurnPhaseALongResult | None = None
    try:
        app = app_factory(config)
        app.start()
        marker_deadline = monotonic() + MARKER_READY_TIMEOUT_S
        while monotonic() < marker_deadline:
            if _ready_marker(app.runtime.status()):
                break
            sleep(POLL_S)
        else:
            raise TimeoutError("PHASE_A_MARKER_NOT_READY: inget motorutgångskommando före markörberedskap")
        app.runtime.select_auto()
        app.runtime.arm_motor_output()
        baseline = _sample(app.runtime.status())
        app.runtime.start_auto()
        # The turn controller itself owns the fixed 30 s monotonic stop.  This
        # outer deadline is only a bounded detector for a failed controller.
        deadline = monotonic() + TURN_TIMEOUT_S + 2.0
        while monotonic() < deadline:
            status = app.runtime.status()
            if status.fault is not None:
                events = tuple(app.runtime.events.recent())
                signs, delta = _validate_timeout_evidence(status, plan, baseline, nominal, events)
                result = TurnPhaseALongResult(plan, signs, delta, nominal, status.fault, events)
                break
            sleep(POLL_S)
        if result is None:
            raise TimeoutError("lång Phase-A nådde inte bounded TURN_TIMEOUT-deadline")
    except BaseException as exc:
        error = exc
        if app is not None:
            try:
                # Capture terminal runtime/source state before close cancels
                # the source.  This is read-only and performs no CAN I/O.
                failure_diagnostics = _runtime_diagnostics(app.runtime, monotonic())
            except Exception as diagnostic_exc:
                failure_diagnostics = {
                    "runtime_diagnostics_error": f"{type(diagnostic_exc).__name__}: {diagnostic_exc}"[:_DIAGNOSTIC_TEXT_LIMIT]
                }
    finally:
        if app is not None:
            try:
                app.close()
            except BaseException as close_exc:
                if error is None:
                    error = close_exc
            if error is not None:
                if failure_diagnostics is None:
                    try:
                        failure_diagnostics = _runtime_diagnostics(app.runtime, monotonic())
                    except Exception as diagnostic_exc:
                        failure_diagnostics = {
                            "runtime_diagnostics_error": f"{type(diagnostic_exc).__name__}: {diagnostic_exc}"[:_DIAGNOSTIC_TEXT_LIMIT]
                        }
                # The verified sink permits this only after close.  It is a
                # released diagnostic ring, not a retry or a new CAN read.
                failure_diagnostics["can"] = _worker_diagnostics_after_close(app.runtime)
    if error is not None:
        if failure_diagnostics is not None:
            try:
                setattr(error, "diagnostics", failure_diagnostics)
            except Exception:
                pass
        raise error
    assert result is not None
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Längre Phase-A raised-wheel AUTO_IN_ROW_TURN timeout HIL")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    parser.add_argument("--confirm-turn-not-calibrated", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_turn_phase_a_long(TurnPhaseALongRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({"ok": True, "fault": result.fault, "direction": result.plan.direction,
                      "command_sign": result.command_sign, "encoder_delta_m": result.encoder_delta_m,
                      "nominal_wheel_travel_m": result.nominal_wheel_travel_m,
                      "events": result.events}, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
