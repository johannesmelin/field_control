"""Fixed raised-wheel HIL for one normal A4 in-row turn target.

The application owns the normal marker trigger and A4 position operation.  A
successful target is deliberately accepted even though raised wheels cannot
turn the OAK/IMU through 180 degrees: after the verified A4 target completes,
production hands control to the established heading/vision controller.  This
runner then explicitly selects MANUAL, which is its final STOP/disarm cleanup.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
import time
from typing import Callable

from .app import FieldControlApplication
from .config import HsvFilter, PhysicalCanConfig, RuntimeConfig, VisionConfig, Zone
from .odometry import DriveGeometry, OdometrySample
from .state_machine import SafetyConfig
from .turn import DifferentialTurnPlan, in_row_turn_plan

CAN_CHANNEL = "can0"
REPLY_PROFILE = "observed-rmdx-same-id"
# Temporary HIL marker only; production HSV belongs in RuntimeConfig.
PHASE_A_MARKER = HsvFilter((26, 20, 150), (36, 255, 255), 100)
MARKER_READY_TIMEOUT_S = 30.0
# Fixed raised-wheel HIL speed, explicitly selected by the operator after the
# successful 10 RPM A4 test.  This remains motor-shaft RPM before 8:1.
TURN_SPEED_MOTOR_RPM = 40.0
# Covers bounded 0x92/A4 exchanges and ordinary position-settle variation;
# it is deliberately fixed rather than exposed through the HIL CLI.
TURN_TIMEOUT_MARGIN_S = 10.0
IN_ROW_TURN_WHEEL_DEGREES = 720.0
POLL_S = .020
_DIAGNOSTIC_EVENT_LIMIT = 25
_DIAGNOSTIC_STRING_LIMIT = 512
# A fixed local hand-off record for terminals which lose the final JSON line.
# /tmp is intentional: it is writable in normal Pi deployments and is not a
# repository artifact or an operator-selectable output destination.
LAST_REPORT_PATH = "/tmp/field_control-phase-a-hil-last-report.json"


class MarkerNotReadyError(TimeoutError):
    """Marker wait expiry with a bounded, non-sensitive runtime snapshot."""

    def __init__(self, diagnostic: dict[str, object]) -> None:
        super().__init__("PHASE_A_MARKER_NOT_READY: inget motorutgångskommando före markörberedskap")
        self.diagnostic = diagnostic


def a4_target_timeout_s(max_wheel_degrees: float, motor_rpm: float,
                        motor_turns_per_wheel_turn: float, *,
                        timeout_margin_s: float = TURN_TIMEOUT_MARGIN_S) -> float:
    """Bound an A4 move from its largest wheel target and motor-side speed.

    The worker scales the shorter wheel's A4 speed, so the largest logical
    wheel angle is the limiting one.  ``motor_rpm`` is before the configured
    gearbox reduction.
    """
    values = (max_wheel_degrees, motor_rpm, motor_turns_per_wheel_turn,
              timeout_margin_s)
    if (not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value) and value > 0 for value in values)):
        raise ValueError("A4-HIL-målvinkel, motor-rpm, utväxling och tidsmarginal måste vara positiva och ändliga")
    nominal_s = abs(float(max_wheel_degrees)) / 360.0 * float(motor_turns_per_wheel_turn) / float(motor_rpm) * 60.0
    return nominal_s + float(timeout_margin_s)


# The default is derived, rather than an independently tuned short timeout:
# 720 wheel degrees at 40 motor RPM through 8:1 needs 24 s, plus 10 s margin.
TURN_TIMEOUT_S = a4_target_timeout_s(IN_ROW_TURN_WHEEL_DEGREES, TURN_SPEED_MOTOR_RPM, 8.0)


@dataclass(frozen=True)
class TurnPhaseARequest:
    slcan_device: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False
    confirm_turn_not_calibrated: bool = False

    def validate(self) -> "TurnPhaseARequest":
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
class TurnPhaseAResult:
    plan: DifferentialTurnPlan
    encoder_delta_m: tuple[float, float]
    completed_state: str
    events: tuple[dict[str, object], ...]


def phase_a_config(request: TurnPhaseARequest) -> RuntimeConfig:
    """The sole fixed A4 profile; no motion parameters are exposed in CLI."""
    geometry = DriveGeometry()
    plan = in_row_turn_plan(geometry, IN_ROW_TURN_WHEEL_DEGREES, "left")
    timeout_s = a4_target_timeout_s(
        max(abs(plan.left_distance_m / geometry.left_wheel_circumference_m * 360.0),
            abs(plan.right_distance_m / geometry.right_wheel_circumference_m * 360.0)),
        TURN_SPEED_MOTOR_RPM, geometry.motor_turns_per_wheel_turn,
    )
    return RuntimeConfig(
        stream_enabled=False, max_rpm=TURN_SPEED_MOTOR_RPM,
        auto_base_rpm=0.0, max_vision_correction_rpm=0.0, vision_kp=0.0,
        search_speed_rpm=0.0, heading_kp=0.0, max_heading_correction_rpm=0.0,
        turn_speed_rpm=TURN_SPEED_MOTOR_RPM, navigation_frame_rate_hz=20.0,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
        vision=VisionConfig(navigation_mode="buds_only", buds=PHASE_A_MARKER,
                            leaves=PHASE_A_MARKER, marker=PHASE_A_MARKER,
                            turn_marker_zone=Zone(.2, .8, .3, 1.0)),
        odometry_geometry=geometry,
        safety=SafetyConfig(in_row_turn_enabled=True, new_row_turn_direction="left",
                            in_row_turn_wheel_degrees=IN_ROW_TURN_WHEEL_DEGREES,
                            auto_start_delay_s=0.0, turn_timeout_s=timeout_s),
    ).validate()


def _ready_marker(status: object) -> bool:
    observation = getattr(status, "observation", None)
    vision = getattr(observation, "vision", None)
    return bool(getattr(observation, "camera_fresh", False) and getattr(observation, "imu_fresh", False)
                and getattr(observation, "odometry_fresh", False) and getattr(vision, "marker_found", False)
                and getattr(observation, "visual_target", False))


def _safe_diagnostic_value(value: object) -> str | int | float | bool | None:
    """Keep HIL failure output JSON-safe without serializing arbitrary objects."""
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        return value[:_DIAGNOSTIC_STRING_LIMIT]
    return type(value).__name__


def _safe_terminal_value(value: object, *, depth: int = 0) -> object:
    """Bound terminal reports to JSON data and exclude hardware path strings."""
    if isinstance(value, dict):
        if depth >= 5:
            return type(value).__name__
        output: dict[str, object] = {}
        for key, item in list(value.items())[:_DIAGNOSTIC_EVENT_LIMIT]:
            safe_key = str(key)[:128]
            key_lower = safe_key.lower()
            if any(term in key_lower for term in ("token", "secret", "password", "credential", "api_key", "device_path")):
                output[safe_key] = "[redacted]"
            else:
                output[safe_key] = _safe_terminal_value(item, depth=depth + 1)
        return output
    if isinstance(value, (list, tuple)):
        if depth >= 5:
            return type(value).__name__
        return [_safe_terminal_value(item, depth=depth + 1)
                for item in list(value)[:_DIAGNOSTIC_EVENT_LIMIT]]
    safe_value = _safe_diagnostic_value(value)
    if isinstance(safe_value, str):
        # Error/event text must not turn the fixed local report into a record
        # of the selected serial device.  The HIL CLI never needs that detail.
        return "[redacted-device-path]" if "/dev/" in safe_value else safe_value
    return safe_value


def _terminal_error_payload(exc: Exception) -> dict[str, object]:
    output: dict[str, object] = {
        "ok": False,
        "error": _safe_terminal_value(f"{type(exc).__name__}: {exc}"[:2000]),
    }
    diagnostic = getattr(exc, "diagnostic", None)
    if isinstance(diagnostic, dict):
        output["diagnostic"] = _safe_terminal_value(diagnostic)
    return output


def _attach_diagnostic(exc: BaseException, diagnostic: dict[str, object]) -> None:
    """Keep the causal error while attaching best-effort bounded evidence.

    The HIL runner must close its physical output even after a fault.  Capture
    the runtime portion before that close, then merge the worker's post-close
    record below.  Diagnostics must never replace the original safe failure.
    """
    try:
        existing = getattr(exc, "diagnostic", None)
        merged = dict(existing) if isinstance(existing, dict) else {}
        merged.update(diagnostic)
        setattr(exc, "diagnostic", merged)
    except Exception:
        # Some exceptional objects may reject attributes.  Safe cleanup and
        # the original failure remain more important than observability.
        pass


def _attach_prior_failure(close_error: BaseException, prior_error: BaseException) -> None:
    """Make a failed final STOP the terminal error without losing causality."""
    prior_diagnostic = getattr(prior_error, "diagnostic", None)
    _attach_diagnostic(close_error, {
        "prior_failure": {
            "error": _safe_terminal_value(
                f"{type(prior_error).__name__}: {prior_error}"[:2000]
            ),
            "diagnostic": (_safe_terminal_value(prior_diagnostic)
                           if isinstance(prior_diagnostic, dict) else None),
        },
    })
    try:
        # The HIL API raises the close error, while exception-aware callers
        # can still inspect the original reason as its explicit cause.
        close_error.__cause__ = prior_error
    except Exception:
        pass


def _terminal_success_payload(result: TurnPhaseAResult) -> dict[str, object]:
    return {
        "ok": True,
        "completed_state": _safe_terminal_value(result.completed_state),
        "direction": _safe_terminal_value(result.plan.direction),
        "plan": {"left_distance_m": _safe_terminal_value(result.plan.left_distance_m),
                 "right_distance_m": _safe_terminal_value(result.plan.right_distance_m)},
        "encoder_delta_m": _safe_terminal_value(result.encoder_delta_m),
        "events": _safe_terminal_value(result.events),
    }


def _persist_last_report(payload: dict[str, object], *, path: str | None = None) -> str | None:
    """Atomically replace the fixed post-close report without affecting HIL outcome."""
    path = LAST_REPORT_PATH if path is None else path
    encoded = json.dumps(_safe_terminal_value(payload), allow_nan=False,
                         separators=(",", ":")).encode("utf-8")
    temporary = f"{path}.tmp-{os.getpid()}"
    try:
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        try:
            with os.fdopen(fd, "wb") as handle:
                handle.write(encoded)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
            directory_fd = os.open(os.path.dirname(path),
                                   os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    except OSError as exc:
        return type(exc).__name__
    return None


def _source_diagnostic(source: object, observation: object, *, fresh_name: str,
                       age_name: str, error_name: str) -> dict[str, object]:
    """Read a source snapshot only for post-timeout diagnostics, never control."""
    snapshot = None
    getter = getattr(source, "snapshot", None)
    if callable(getter):
        try:
            snapshot = getter()
        except Exception as exc:  # Diagnostic capture must not mask safe cleanup.
            return {"fresh": bool(getattr(observation, fresh_name, False)), "connected": None,
                    "age_s": _safe_diagnostic_value(getattr(observation, age_name, None)),
                    "error": f"snapshot failed: {type(exc).__name__}"}
    observed_error = getattr(observation, error_name, None) if observation is not None else None
    return {
        "fresh": bool(getattr(observation, fresh_name, False)),
        "connected": (None if snapshot is None else bool(getattr(snapshot, "connected", False))),
        "age_s": _safe_diagnostic_value(getattr(observation, age_name, None)),
        "error": _safe_diagnostic_value(
            observed_error if observed_error is not None
            else (None if snapshot is None else getattr(snapshot, "error", None))
        ),
    }


def _can_worker_diagnostic(runtime: object, *, after_close: bool) -> dict[str, object]:
    """Return a bounded CAN worker record without opening or commanding it.

    ``diagnostic_snapshot`` is intentionally legal only after the worker has
    performed its own STOP+settle close.  Before close we capture only the
    public status and adapter events; after close we additionally retain the
    last bounded worker entries that explain A4/0x92 traffic.
    """
    motor = getattr(runtime, "motor", None)
    sink = getattr(motor, "_sink", motor)
    output: dict[str, object] = {
        "adapter_events": _safe_terminal_value(list(getattr(motor, "events", ()))[-_DIAGNOSTIC_EVENT_LIMIT:]),
    }
    status_getter = getattr(sink, "status", None)
    if callable(status_getter):
        try:
            status = status_getter()
            output["status"] = {
                "mode": _safe_diagnostic_value(getattr(status, "mode", None)),
                "ready": _safe_diagnostic_value(getattr(status, "ready", None)),
                "error": _safe_diagnostic_value(getattr(status, "error", None)),
            }
        except Exception as exc:
            output["status_error"] = type(exc).__name__
    if not after_close:
        return output
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if callable(snapshot):
        try:
            entries: list[dict[str, object]] = []
            for entry in list(snapshot())[-_DIAGNOSTIC_EVENT_LIMIT:]:
                raw_data = getattr(entry, "data", None)
                try:
                    data_hex = None if raw_data is None else bytes(raw_data)[:8].hex()
                except (TypeError, ValueError):
                    data_hex = None
                entries.append({
                    "timestamp_s": _safe_diagnostic_value(getattr(entry, "timestamp_s", None)),
                    "sequence": _safe_diagnostic_value(getattr(entry, "sequence", None)),
                    "phase": _safe_diagnostic_value(getattr(entry, "phase", None)),
                    "direction": _safe_diagnostic_value(getattr(entry, "direction", None)),
                    "can_id": _safe_diagnostic_value(getattr(entry, "can_id", None)),
                    "dlc": _safe_diagnostic_value(getattr(entry, "dlc", None)),
                    "data_hex": data_hex,
                    "expected_reply_ids": _safe_terminal_value(getattr(entry, "expected_reply_ids", ())),
                    "pending_reply_ids": _safe_terminal_value(getattr(entry, "pending_reply_ids", ())),
                    "detail": _safe_diagnostic_value(getattr(entry, "detail", None)),
                })
            output["entries"] = entries
        except Exception as exc:
            output["entries_error"] = type(exc).__name__
    return output


def runtime_failure_diagnostic(runtime: object) -> dict[str, object]:
    """Capture the last relevant status before the runner closes the app.

    This deliberately excludes configuration and raw source values: the HIL
    output must explain readiness without exposing device paths or image data.
    """
    status = runtime.status()
    observation = getattr(status, "observation", None)
    vision = getattr(observation, "vision", None)
    events_getter = getattr(getattr(runtime, "events", None), "recent", None)
    try:
        raw_events = events_getter() if callable(events_getter) else []
    except Exception as exc:
        raw_events = [{"kind": "diagnostic_events_unavailable", "data": {"error": type(exc).__name__}}]
    events: list[dict[str, object]] = []
    for item in list(raw_events)[-_DIAGNOSTIC_EVENT_LIMIT:]:
        if not isinstance(item, dict):
            continue
        raw_data = item.get("data")
        data = ({str(key)[:128]: _safe_diagnostic_value(value) for key, value in raw_data.items()}
                if isinstance(raw_data, dict) else {})
        events.append({
            "timestamp_s": _safe_diagnostic_value(item.get("timestamp_s")),
            "level": _safe_diagnostic_value(item.get("level")),
            "kind": _safe_diagnostic_value(item.get("kind")),
            "data": data,
        })
    return {
        "state": _safe_diagnostic_value(getattr(status, "state", None)),
        "state_reason": _safe_diagnostic_value(getattr(getattr(status, "snapshot", None), "reason", None)),
        "fault": _safe_diagnostic_value(getattr(status, "fault", None)),
        "motor_output_armed": bool(getattr(status, "motor_output_armed", False)),
        "camera": _source_diagnostic(getattr(runtime, "camera", None), observation,
                                     fresh_name="camera_fresh", age_name="camera_age_s", error_name="camera_error"),
        "imu": _source_diagnostic(getattr(runtime, "imu", None), observation,
                                  fresh_name="imu_fresh", age_name="imu_age_s", error_name="imu_error"),
        "odometry": _source_diagnostic(getattr(runtime, "_odometry", None), observation,
                                       fresh_name="odometry_fresh", age_name="odometry_age_s", error_name="odometry_error"),
        "vision": {
            "visual_target": bool(getattr(observation, "visual_target", False)),
            "target_valid": getattr(vision, "target_x", None) is not None,
            "marker_found": bool(getattr(vision, "marker_found", False)),
            "bud_in_trigger_zone": bool(getattr(vision, "bud_in_trigger_zone", False)),
        },
        "recent_events": events,
        "can_worker_pre_close": _can_worker_diagnostic(runtime, after_close=False),
    }


# Kept as a narrow compatibility name for callers/tests that originally
# requested marker-only evidence.  The same snapshot is now used for every
# runner failure, including admitted A4 and odometry deadlines.
marker_not_ready_diagnostic = runtime_failure_diagnostic


def _sample(status: object) -> OdometrySample:
    value = getattr(getattr(status, "observation", None), "odometry_sample", None)
    if type(value) is not OdometrySample:
        raise RuntimeError("färsk per-hjulsodometri saknas")
    return value


def _sign(value: float) -> int:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value == 0:
        raise RuntimeError("encoderdelta måste vara ändlig och icke-noll")
    return 1 if value > 0 else -1


def _validate_completion(status: object, plan: DifferentialTurnPlan, baseline: OdometrySample,
                         events: tuple[dict[str, object], ...], tolerance_m: float) -> tuple[float, float]:
    if getattr(status, "fault", None) is not None:
        raise RuntimeError(f"A4-vändningen avslutades med fel: {getattr(status, 'fault', None)}")
    state = getattr(status, "state", None)
    if state not in ("AUTO_ROW_FOLLOW", "AUTO_SEARCH", "AUTO_COMPLETE"):
        raise RuntimeError(f"A4-vändningen nådde inget normalt terminalstate: {state}")
    final = _sample(status)
    delta = (final.left_distance_m - baseline.left_distance_m,
             final.right_distance_m - baseline.right_distance_m)
    expected = (plan.left_distance_m, plan.right_distance_m)
    if (not all(math.isfinite(value) for value in delta)
            or tuple(_sign(value) for value in delta) != tuple(_sign(value) for value in expected)):
        raise RuntimeError("encoderdeltan är inte ändlig och teckenkonsistent med A4-turn-planen")
    if any(abs(actual - target) > tolerance_m for actual, target in zip(delta, expected)):
        raise RuntimeError("encoderdeltan når inte A4-målet inom konfigurerad turn-tolerans")
    kinds = [entry.get("kind") for entry in events]
    try:
        started = kinds.index("turn_started")
        completed = kinds.index("turn_completed", started + 1)
    except ValueError as exc:
        raise RuntimeError("saknar ordnade runtime-event turn_started följt av turn_completed") from exc
    completed_data = events[completed].get("data")
    if not isinstance(completed_data, dict) or completed_data.get("state") != "AUTO_IN_ROW_TURN":
        raise RuntimeError("turn_completed bekräftar inte AUTO_IN_ROW_TURN")
    return delta


def run_turn_phase_a(request: TurnPhaseARequest, *, app_factory: Callable[..., FieldControlApplication] = FieldControlApplication,
                     monotonic: Callable[[], float] = time.monotonic,
                     sleep: Callable[[float], None] = time.sleep) -> TurnPhaseAResult:
    """Run exactly one marker-triggered production A4 in-row target."""
    request.validate()  # All physical gates precede application construction.
    config = phase_a_config(request)
    plan = in_row_turn_plan(config.odometry_geometry, config.safety.in_row_turn_wheel_degrees,
                            config.safety.new_row_turn_direction)
    app: FieldControlApplication | None = None
    error: BaseException | None = None
    pre_close_diagnostic: dict[str, object] | None = None
    result: TurnPhaseAResult | None = None
    try:
        app = app_factory(config)
        app.start()
        marker_deadline = monotonic() + MARKER_READY_TIMEOUT_S
        while monotonic() < marker_deadline:
            if _ready_marker(app.runtime.status()):
                break
            sleep(POLL_S)
        else:
            raise MarkerNotReadyError(marker_not_ready_diagnostic(app.runtime))
        app.runtime.select_auto()
        app.runtime.arm_motor_output()
        baseline = _sample(app.runtime.status())
        app.runtime.start_auto()
        # Start the independent observer deadline only when the normal runtime
        # has recorded its turn start. This avoids charging marker debounce to
        # motor movement while retaining the exact same bounded A4 duration as
        # the production worker -- no second grace period is permitted.
        turn_start_wait_deadline = monotonic() + config.safety.turn_timeout_s
        turn_deadline: float | None = None
        while True:
            now_s = monotonic()
            status = app.runtime.status()
            if status.fault is not None:
                raise RuntimeError(f"A4-vändningen avslutades med fel: {status.fault}")
            events = tuple(app.runtime.events.recent())
            if turn_deadline is None:
                if any(entry.get("kind") == "turn_started" for entry in events):
                    turn_deadline = now_s + config.safety.turn_timeout_s
                elif now_s >= turn_start_wait_deadline:
                    raise TimeoutError("Phase-A registrerade inte turn_started inom A4-måldeadline")
            if any(entry.get("kind") == "turn_completed" for entry in events):
                delta = _validate_completion(status, plan, baseline, events,
                                             config.safety.turn_distance_tolerance_m)
                completed_state = status.state
                # Cleanup only after evidence capture: MANUAL sends STOP and
                # revokes the output lease through the public runtime path.
                app.runtime.select_manual()
                terminal = app.runtime.status()
                if terminal.fault is not None or terminal.motor_output_armed or terminal.state != "MANUAL":
                    raise RuntimeError("A4-HIL avslutades inte säkert disarmerad i MANUAL")
                result = TurnPhaseAResult(plan, delta, completed_state, events)
                break
            if turn_deadline is not None and now_s >= turn_deadline:
                raise TimeoutError("Phase-A nådde inte exakt A4-måldeadline")
            sleep(POLL_S)
    except BaseException as exc:
        error = exc
        if app is not None:
            try:
                pre_close_diagnostic = runtime_failure_diagnostic(app.runtime)
                _attach_diagnostic(error, pre_close_diagnostic)
            except Exception:
                # Snapshot collection cannot delay the mandatory safe close.
                pass
    finally:
        if app is not None:
            # This is intentionally before app.close(): source stop/worker
            # close must not erase the causal runtime evidence for a close
            # failure.  It is attached only if there is an error.
            if pre_close_diagnostic is None:
                try:
                    pre_close_diagnostic = runtime_failure_diagnostic(app.runtime)
                except Exception:
                    pre_close_diagnostic = None
            try:
                app.close()
            except BaseException as close_exc:
                if error is None:
                    error = close_exc
                    if pre_close_diagnostic is not None:
                        _attach_diagnostic(error, pre_close_diagnostic)
                else:
                    _attach_prior_failure(close_exc, error)
                    error = close_exc
            if error is not None:
                try:
                    _attach_diagnostic(error, {
                        "can_worker_post_close": _can_worker_diagnostic(app.runtime, after_close=True),
                    })
                except Exception:
                    pass
    if error is not None:
        raise error
    assert result is not None
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Phase-A raised-wheel AUTO_IN_ROW_TURN A4-mål HIL")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    parser.add_argument("--confirm-turn-not-calibrated", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_turn_phase_a(TurnPhaseARequest(**vars(args)))
    except Exception as exc:
        output = _terminal_error_payload(exc)
        persistence_error = _persist_last_report(output)
        if persistence_error is not None:
            output["report_persistence_error"] = persistence_error
        print(json.dumps(output, allow_nan=False))
        return 2
    output = _terminal_success_payload(result)
    persistence_error = _persist_last_report(output)
    if persistence_error is not None:
        output["report_persistence_error"] = persistence_error
    print(json.dumps(output, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
