"""Fixed raised-wheel HIL routines for the production A4 new-row path.

This module intentionally has no motion knobs.  Both routines start through
the ordinary marker/state-machine path; the first confirms the asymmetric
geometry target and the second sends the public STOP only after the worker has
accepted an active A4 operation.
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
from .odometry import DriveGeometry, OdometrySample
from .state_machine import SafetyConfig
from .turn import DifferentialTurnPlan, new_row_turn_targets
from .turn_phase_a_hil import (
    MARKER_READY_TIMEOUT_S, PHASE_A_MARKER, POLL_S, REPLY_PROFILE,
    TURN_SPEED_MOTOR_RPM, _ready_marker,
    _sample, _sign, a4_target_timeout_s,
    _attach_diagnostic, _attach_prior_failure, _can_worker_diagnostic,
    _persist_last_report, _safe_terminal_value, _terminal_error_payload,
    runtime_failure_diagnostic,
)

CAN_CHANNEL = "can0"
# Fixed, normal production geometry.  With 1.20 m row spacing and 1.005 m
# track, a left new-row arc has 0.306 m left/3.464 m right wheel targets.
ROW_SPACING_M = 1.20
NEW_ROW_DIRECTION = "left"
NUMBER_OF_ROWS = 2
# The worker must start A4 within a bounded time after normal turn admission.
A4_ACTIVE_ADMISSION_TIMEOUT_S = 2.0
# New-row has the longer outer wheel path.  Keep this fixed and local to this
# raised-wheel HIL profile: the A4 target itself still stops at its target.
NEW_ROW_TIMEOUT_MARGIN_S = 30.0
# Fixed local result for a terminal which drops the HIL's final JSON line.
# It deliberately differs from the in-row report so the last result from each
# physical routine remains independently inspectable.
LAST_REPORT_PATH = "/tmp/field_control-new-row-hil-last-report.json"


@dataclass(frozen=True)
class NewRowTurnRequest:
    slcan_device: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False
    confirm_turn_not_calibrated: bool = False

    def validate(self) -> "NewRowTurnRequest":
        if self.enable_motors is not True:
            raise ValueError("--enable-motors krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_wheels_raised is not True:
            raise ValueError("--confirm-wheels-raised krävs")
        if self.confirm_turn_not_calibrated is not True:
            raise ValueError("--confirm-turn-not-calibrated krävs")
        prefix = "/dev/serial/by-id/"
        name = (self.slcan_device[len(prefix):] if isinstance(self.slcan_device, str)
                and self.slcan_device.startswith(prefix) else "")
        if not name or name in (".", "..") or "/" in name:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class NewRowTurnResult:
    plan: DifferentialTurnPlan
    encoder_delta_m: tuple[float, float]
    completed_state: str
    events: tuple[dict[str, object], ...]


@dataclass(frozen=True)
class NewRowStopResult:
    plan: DifferentialTurnPlan
    position_events_before_stop: int
    position_events_after_stop: int
    completed_state: str


def _persist_new_row_last_report(payload: dict[str, object]) -> str | None:
    """Persist a bounded 0600 atomic result without affecting motor cleanup."""
    return _persist_last_report(payload, path=LAST_REPORT_PATH)


def _terminal_success_payload(result: NewRowTurnResult | NewRowStopResult,
                              *, test: str) -> dict[str, object]:
    """JSON-safe normal evidence, including the bounded runtime event record."""
    return {
        "ok": True,
        "test": test,
        "result": {
            "completed_state": _safe_terminal_value(result.completed_state),
            "direction": _safe_terminal_value(result.plan.direction),
            "plan": {
                "left_distance_m": _safe_terminal_value(result.plan.left_distance_m),
                "right_distance_m": _safe_terminal_value(result.plan.right_distance_m),
            },
            "position_events_before_stop": _safe_terminal_value(
                getattr(result, "position_events_before_stop", None)),
            "position_events_after_stop": _safe_terminal_value(
                getattr(result, "position_events_after_stop", None)),
            "encoder_delta_m": _safe_terminal_value(getattr(result, "encoder_delta_m", None)),
            "events": _safe_terminal_value(getattr(result, "events", ())),
        },
    }


def new_row_config(request: NewRowTurnRequest) -> RuntimeConfig:
    """The sole 40 motor-RPM production A4 new-row profile."""
    geometry = DriveGeometry()
    plan = new_row_turn_targets(geometry, ROW_SPACING_M, TURN_SPEED_MOTOR_RPM,
                                NEW_ROW_DIRECTION)
    wheel_degrees = (
        abs(plan.left_distance_m / geometry.left_wheel_circumference_m * 360.0),
        abs(plan.right_distance_m / geometry.right_wheel_circumference_m * 360.0),
    )
    timeout_s = a4_target_timeout_s(
        max(wheel_degrees), TURN_SPEED_MOTOR_RPM,
        geometry.motor_turns_per_wheel_turn,
        timeout_margin_s=NEW_ROW_TIMEOUT_MARGIN_S,
    )
    return RuntimeConfig(
        stream_enabled=False, max_rpm=TURN_SPEED_MOTOR_RPM,
        auto_base_rpm=0.0, max_vision_correction_rpm=0.0, vision_kp=0.0,
        search_speed_rpm=0.0, heading_kp=0.0, max_heading_correction_rpm=0.0,
        turn_speed_rpm=TURN_SPEED_MOTOR_RPM, navigation_frame_rate_hz=20.0,
        row_spacing_m=ROW_SPACING_M,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE,
                                       request.slcan_device, True, True),
        vision=VisionConfig(navigation_mode="buds_only", buds=PHASE_A_MARKER,
                            leaves=PHASE_A_MARKER, marker=PHASE_A_MARKER,
                            turn_marker_zone=Zone(.2, .8, .3, 1.0)),
        odometry_geometry=geometry,
        safety=SafetyConfig(in_row_turn_enabled=False,
                            new_row_turn_direction=NEW_ROW_DIRECTION,
                            number_of_rows=NUMBER_OF_ROWS,
                            auto_start_delay_s=0.0, turn_timeout_s=timeout_s),
    ).validate()


def _plan(config: RuntimeConfig) -> DifferentialTurnPlan:
    return new_row_turn_targets(config.odometry_geometry, config.row_spacing_m,
                                config.turn_speed_rpm,
                                config.safety.new_row_turn_direction,
                                config.safety.inner_wheel_min_ratio)


def _position_events(runtime: object) -> tuple[tuple[object, ...], ...]:
    events = getattr(getattr(runtime, "motor", None), "events", ())
    output = tuple(item for item in tuple(events)
                   if isinstance(item, tuple) and len(item) >= 1 and item[0] == "position")
    return output


def _a4_is_active(runtime: object) -> bool:
    """Prove both accepted A4 admission and worker-reported active target."""
    request = getattr(runtime, "_position_turn_request", None)
    status = getattr(getattr(runtime, "motor", None), "position_move_status", None)
    stage = getattr(getattr(runtime, "motor", None), "position_move_stage", None)
    if request is None or not callable(status) or not callable(stage) or not _position_events(runtime):
        return False
    value = status(request)
    if not isinstance(value, tuple) or len(value) != 4:
        raise RuntimeError("ogiltig A4-status från motorgränsen")
    done, succeeded, error, active = value
    if not all(isinstance(item, bool) for item in (done, succeeded, active)) or error is not None and not isinstance(error, str):
        raise RuntimeError("ogiltig A4-status från motorgränsen")
    if done or succeeded or error is not None:
        raise RuntimeError("A4 avslutades innan STOP-testet hann verifiera aktiv målpositionering")
    acknowledged, running = stage(request)
    if not isinstance(acknowledged, bool) or not isinstance(running, bool):
        raise RuntimeError("ogiltig workerägd A4-målstatus")
    return active and acknowledged and running


def _validate_new_row_completion(status: object, plan: DifferentialTurnPlan,
                                 baseline: OdometrySample,
                                 events: tuple[dict[str, object], ...],
                                 tolerance_m: float) -> tuple[float, float]:
    """Completion proof equivalent to Phase A, bound to NEW_ROW state."""
    if getattr(status, "fault", None) is not None:
        raise RuntimeError(f"A4-new-row avslutades med fel: {getattr(status, 'fault', None)}")
    if getattr(status, "state", None) not in ("AUTO_ROW_FOLLOW", "AUTO_SEARCH", "AUTO_COMPLETE"):
        raise RuntimeError(f"A4-new-row nådde inget normalt terminalstate: {getattr(status, 'state', None)}")
    final = _sample(status)
    delta = (final.left_distance_m - baseline.left_distance_m,
             final.right_distance_m - baseline.right_distance_m)
    expected = (plan.left_distance_m, plan.right_distance_m)
    if (not all(math.isfinite(value) for value in delta)
            or tuple(_sign(value) for value in delta) != tuple(_sign(value) for value in expected)):
        raise RuntimeError("new-row encoderdelta är inte teckenkonsistent med A4-turn-planen")
    if any(abs(actual - target) > tolerance_m for actual, target in zip(delta, expected)):
        raise RuntimeError("new-row encoderdelta når inte A4-målet inom konfigurerad turn-tolerans")
    kinds = [entry.get("kind") for entry in events]
    try:
        started = kinds.index("turn_started")
        completed = kinds.index("turn_completed", started + 1)
    except ValueError as exc:
        raise RuntimeError("saknar ordnade runtime-event turn_started följt av turn_completed") from exc
    data = events[completed].get("data")
    if not isinstance(data, dict) or data.get("state") != "AUTO_NEW_ROW_TURN":
        raise RuntimeError("turn_completed bekräftar inte AUTO_NEW_ROW_TURN")
    return delta


def _odometry_timestamp(runtime: object) -> float | None:
    """Return the raw physical-source timestamp only for a valid sample.

    ``RuntimeStatus.observation`` is the last navigation tick and can briefly
    lag a STOP+0x9C settle.  For the HIL start hand-off we therefore inspect
    the source's immutable latest snapshot directly.  This does not relax the
    runtime's normal freshness checks; it merely prevents this runner from
    requesting AUTO while arm's own STOP has invalidated the shared 0x92 read.
    """
    source = getattr(runtime, "_odometry", None)
    snapshot_getter = getattr(source, "snapshot", None)
    if source is None or not callable(snapshot_getter):
        return None
    snapshot = snapshot_getter()
    timestamp = getattr(snapshot, "updated_at_s", None)
    if (not bool(getattr(snapshot, "connected", False))
            or type(getattr(snapshot, "value", None)) is not OdometrySample
            or not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool)
            or not math.isfinite(timestamp)):
        return None
    return float(timestamp)


def _wait_post_arm_odometry(runtime: object, *, previous_timestamp: float | None,
                            timeout_s: float, monotonic: Callable[[], float],
                            sleep: Callable[[float], None]) -> None:
    """Require a new connected 0x92 sample after physical arm's STOP settle.

    Arming intentionally sends STOP+0x9C.  That STOP may preempt the periodic
    shared encoder read, which correctly makes ``OdometrySource`` unavailable
    until its next 0x92 pair.  Starting AUTO in that gap can fault before the
    marker-to-A4 hand-off.  Wait only while stopped and use the configured
    bounded odometry timeout; never reuse the pre-arm sample.

    Injected/dry-run runners have no inspectable physical source and retain
    their existing status-based start behavior.
    """
    source = getattr(runtime, "_odometry", None)
    if source is None or not callable(getattr(source, "snapshot", None)):
        return
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        timestamp = _odometry_timestamp(runtime)
        if timestamp is not None and (previous_timestamp is None or timestamp > previous_timestamp):
            return
        sleep(POLL_S)
    raise TimeoutError("POST_ARM_ODOMETRY_NOT_READY: ingen ny ansluten encoderavläsning efter armering")


def _wait_marker_then_start(app: FieldControlApplication, monotonic: Callable[[], float],
                            sleep: Callable[[float], None]) -> OdometrySample:
    marker_deadline = monotonic() + MARKER_READY_TIMEOUT_S
    while monotonic() < marker_deadline:
        if _ready_marker(app.runtime.status()):
            break
        sleep(POLL_S)
    else:
        diagnostic = runtime_failure_diagnostic(app.runtime)
        error = TimeoutError("NEW_ROW_MARKER_NOT_READY: inget motorutgångskommando före markörberedskap")
        setattr(error, "diagnostic", diagnostic)
        raise error
    app.runtime.select_auto()
    pre_arm_odometry_timestamp = _odometry_timestamp(app.runtime)
    app.runtime.arm_motor_output()
    _wait_post_arm_odometry(
        app.runtime,
        previous_timestamp=pre_arm_odometry_timestamp,
        timeout_s=app.runtime.config.odometry_timeout_s,
        monotonic=monotonic,
        sleep=sleep,
    )
    baseline = _sample(app.runtime.status())
    app.runtime.start_auto()
    return baseline


def _close_after_public_stop(app: FieldControlApplication) -> None:
    """Use the public STOP path before lifecycle close on every HIL exit."""
    try:
        app.runtime.select_manual()
    finally:
        app.close()


def run_new_row_turn(request: NewRowTurnRequest, *,
                     app_factory: Callable[..., FieldControlApplication] = FieldControlApplication,
                     monotonic: Callable[[], float] = time.monotonic,
                     sleep: Callable[[float], None] = time.sleep) -> NewRowTurnResult:
    """Run one normal marker-triggered production ``AUTO_NEW_ROW_TURN``."""
    request.validate()
    config = new_row_config(request)
    plan = _plan(config)
    if math.isclose(abs(plan.left_distance_m), abs(plan.right_distance_m)):
        raise RuntimeError("new-row HIL kräver asymmetriska geometrimål")
    app: FieldControlApplication | None = None
    error: BaseException | None = None
    pre_close_diagnostic: dict[str, object] | None = None
    result: NewRowTurnResult | None = None
    public_stop_done = False
    try:
        app = app_factory(config)
        app.start()
        baseline = _wait_marker_then_start(app, monotonic, sleep)
        start_deadline = monotonic() + config.safety.turn_timeout_s
        turn_deadline: float | None = None
        while True:
            now_s = monotonic()
            status = app.runtime.status()
            if status.fault is not None:
                raise RuntimeError(f"A4-new-row avslutades med fel: {status.fault}")
            events = tuple(app.runtime.events.recent())
            if turn_deadline is None:
                if any(item.get("kind") == "turn_started" for item in events):
                    turn_deadline = now_s + config.safety.turn_timeout_s
                elif now_s >= start_deadline:
                    raise TimeoutError("new-row registrerade inte turn_started inom A4-måldeadline")
            if any(item.get("kind") == "turn_completed" for item in events):
                delta = _validate_new_row_completion(status, plan, baseline, events,
                                                     config.safety.turn_distance_tolerance_m)
                completed = status.state
                # Preserve the ordinary public STOP/disarm path before close.
                app.runtime.select_manual()
                public_stop_done = True
                terminal = app.runtime.status()
                if terminal.fault is not None or terminal.motor_output_armed or terminal.state != "MANUAL":
                    raise RuntimeError("A4-new-row avslutades inte säkert disarmerad i MANUAL")
                result = NewRowTurnResult(plan, delta, completed, events)
                break
            if turn_deadline is not None and now_s >= turn_deadline:
                raise TimeoutError("new-row nådde inte exakt A4-måldeadline")
            sleep(POLL_S)
    except BaseException as exc:
        error = exc
        if app is not None:
            try:
                pre_close_diagnostic = runtime_failure_diagnostic(app.runtime)
                _attach_diagnostic(error, pre_close_diagnostic)
            except Exception:
                pass
    finally:
        if app is not None:
            if pre_close_diagnostic is None:
                try:
                    pre_close_diagnostic = runtime_failure_diagnostic(app.runtime)
                except Exception:
                    pre_close_diagnostic = None
            try:
                if public_stop_done:
                    app.close()
                else:
                    _close_after_public_stop(app)
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


def run_new_row_stop(request: NewRowTurnRequest, *,
                     app_factory: Callable[..., FieldControlApplication] = FieldControlApplication,
                     monotonic: Callable[[], float] = time.monotonic,
                     sleep: Callable[[float], None] = time.sleep) -> NewRowStopResult:
    """Preempt an already active physical A4 new-row target with public STOP."""
    request.validate()
    config = new_row_config(request)
    plan = _plan(config)
    app: FieldControlApplication | None = None
    error: BaseException | None = None
    pre_close_diagnostic: dict[str, object] | None = None
    result: NewRowStopResult | None = None
    try:
        app = app_factory(config)
        app.start()
        _wait_marker_then_start(app, monotonic, sleep)
        active_deadline = monotonic() + A4_ACTIVE_ADMISSION_TIMEOUT_S
        while monotonic() < active_deadline:
            status = app.runtime.status()
            if status.fault is not None:
                raise RuntimeError(f"A4-new-row avslutades med fel före STOP: {status.fault}")
            if _a4_is_active(app.runtime):
                break
            sleep(POLL_S)
        else:
            raise TimeoutError("A4-new-row blev inte aktiv före STOP-deadline")
        before = len(_position_events(app.runtime))
        # This is specifically the public operator STOP, followed by its
        # worker-verified bounded STOP+0x9C settle before app.close().
        settle = getattr(app.runtime, "stop_and_settle", None)
        if not callable(settle):
            raise RuntimeError("runtime saknar publik verifierad STOP+0x9C-settle")
        settle()
        terminal = app.runtime.status()
        if terminal.state != "MANUAL" or terminal.motor_output_armed or terminal.fault is not None:
            raise RuntimeError("STOP avbröt inte A4 säkert till disarmerad MANUAL")
        after = len(_position_events(app.runtime))
        if after != before:
            raise RuntimeError("A4 återadmitterades efter publikt STOP+0x9C-settle")
        result = NewRowStopResult(plan, before, after, terminal.state)
    except BaseException as exc:
        error = exc
        if app is not None:
            try:
                pre_close_diagnostic = runtime_failure_diagnostic(app.runtime)
                _attach_diagnostic(error, pre_close_diagnostic)
            except Exception:
                pass
    finally:
        if app is not None:
            if pre_close_diagnostic is None:
                try:
                    pre_close_diagnostic = runtime_failure_diagnostic(app.runtime)
                except Exception:
                    pre_close_diagnostic = None
            try:
                _close_after_public_stop(app)
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
    parser = argparse.ArgumentParser(description="Raised-wheel 40 RPM A4 AUTO_NEW_ROW_TURN HIL")
    parser.add_argument("--stop-during-active-a4", action="store_true")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    parser.add_argument("--confirm-turn-not-calibrated", action="store_true")
    args = parser.parse_args(argv)
    request = NewRowTurnRequest(args.slcan_device, args.enable_motors,
                                args.confirm_physical_stop_tested,
                                args.confirm_wheels_raised,
                                args.confirm_turn_not_calibrated)
    try:
        result = run_new_row_stop(request) if args.stop_during_active_a4 else run_new_row_turn(request)
    except Exception as exc:
        # Print the exact bounded value which is persisted.  In particular,
        # do not let an unusually deep worker diagnostic appear in terminal
        # output but be collapsed in the durable hand-off record.
        output = _safe_terminal_value(_terminal_error_payload(exc))
        assert isinstance(output, dict)
        persistence_error = _persist_new_row_last_report(output)
        if persistence_error is not None:
            output["report_persistence_error"] = persistence_error
        print(json.dumps(output, allow_nan=False))
        return 2
    output = _terminal_success_payload(
        result,
        test="stop_during_active_a4" if args.stop_during_active_a4 else "new_row_target",
    )
    persistence_error = _persist_new_row_last_report(output)
    if persistence_error is not None:
        output["report_persistence_error"] = persistence_error
    print(json.dumps(output, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
