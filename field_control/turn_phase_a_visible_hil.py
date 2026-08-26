"""Fixed, visible raised-wheel observation of the normal in-row turn.

This HIL profile is separate from both the short Phase-A safety check and the
slower 2-RPM long observation.  It preserves their normal application path,
marker gates, timeout-only success rule and terminal diagnostics, while using
the already physically observed 10 motor-RPM speed for a short visible run.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import time
from typing import Callable

from .app import FieldControlApplication
from .config import PhysicalCanConfig, RuntimeConfig, VisionConfig, Zone
from .odometry import motor_rpm_to_wheel_rpm
from .state_machine import SafetyConfig
from .turn import DifferentialTurnPlan, in_row_turn_plan
from .turn_phase_a_hil import PHASE_A_MARKER
from .turn_phase_a_long_hil import (
    CAN_CHANNEL, MARKER_READY_TIMEOUT_S, MAX_NOMINAL_TRAVEL_RATIO,
    MIN_NOMINAL_TRAVEL_RATIO, POLL_S, REPLY_PROFILE, TurnPhaseALongRequest,
    _ready_marker, _runtime_diagnostics, _sample, _worker_diagnostics_after_close,
    _validate_timeout_evidence,
)


# Fixed motor-side profile; no CLI option may change these values.  The 10 RPM
# value was already physically visible in the individual-wheel HIL checks.
TURN_SPEED_MOTOR_RPM = 10.0
TURN_TIMEOUT_S = 6.0

# Keep the four explicit physical-safety gates identical to the long runner.
TurnPhaseAVisibleRequest = TurnPhaseALongRequest


@dataclass(frozen=True)
class TurnPhaseAVisibleResult:
    plan: DifferentialTurnPlan
    command_sign: tuple[int, int]
    encoder_delta_m: tuple[float, float]
    nominal_wheel_travel_m: tuple[float, float]
    fault: str
    events: tuple[dict[str, object], ...]


def phase_a_visible_config(request: TurnPhaseAVisibleRequest) -> RuntimeConfig:
    """Build only the fixed visible profile; production config is untouched."""
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


def nominal_wheel_travel_m(config: RuntimeConfig) -> tuple[float, float]:
    """Return the fixed distance using the configured gearbox and wheel geometry."""
    wheel_rpm = motor_rpm_to_wheel_rpm(TURN_SPEED_MOTOR_RPM, config.odometry_geometry)
    minutes = TURN_TIMEOUT_S / 60.0
    return (wheel_rpm * minutes * config.odometry_geometry.left_wheel_circumference_m,
            wheel_rpm * minutes * config.odometry_geometry.right_wheel_circumference_m)


def run_turn_phase_a_visible(
        request: TurnPhaseAVisibleRequest, *,
        app_factory: Callable[..., FieldControlApplication] = FieldControlApplication,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
) -> TurnPhaseAVisibleResult:
    """Exercise the ordinary AUTO turn path and accept only its timeout STOP."""
    request.validate()  # Safety gates precede OAK, CAN and app construction.
    config = phase_a_visible_config(request)
    plan = in_row_turn_plan(config.odometry_geometry, config.safety.in_row_turn_wheel_degrees,
                            config.safety.new_row_turn_direction)
    nominal = nominal_wheel_travel_m(config)
    app: FieldControlApplication | None = None
    error: BaseException | None = None
    diagnostics: dict[str, object] | None = None
    result: TurnPhaseAVisibleResult | None = None
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
        # This is the same public lifecycle as the two existing Phase-A HILs.
        app.runtime.select_auto()
        app.runtime.arm_motor_output()
        baseline = _sample(app.runtime.status())
        app.runtime.start_auto()
        deadline = monotonic() + TURN_TIMEOUT_S + 2.0
        while monotonic() < deadline:
            status = app.runtime.status()
            if status.fault is not None:
                events = tuple(app.runtime.events.recent())
                signs, delta = _validate_timeout_evidence(status, plan, baseline, nominal, events)
                result = TurnPhaseAVisibleResult(plan, signs, delta, nominal, status.fault, events)
                break
            sleep(POLL_S)
        if result is None:
            raise TimeoutError("synlig Phase-A nådde inte bounded TURN_TIMEOUT-deadline")
    except BaseException as exc:
        error = exc
        if app is not None:
            try:
                diagnostics = _runtime_diagnostics(app.runtime, monotonic())
            except Exception as diagnostic_exc:
                diagnostics = {"runtime_diagnostics_error": f"{type(diagnostic_exc).__name__}: {diagnostic_exc}"[:1000]}
    finally:
        if app is not None:
            try:
                app.close()
            except BaseException as close_exc:
                if error is None:
                    error = close_exc
            if error is not None:
                if diagnostics is None:
                    try:
                        diagnostics = _runtime_diagnostics(app.runtime, monotonic())
                    except Exception as diagnostic_exc:
                        diagnostics = {
                            "runtime_diagnostics_error": f"{type(diagnostic_exc).__name__}: {diagnostic_exc}"[:1000]
                        }
                diagnostics["can"] = _worker_diagnostics_after_close(app.runtime)
    if error is not None:
        if diagnostics is not None:
            try:
                setattr(error, "diagnostics", diagnostics)
            except Exception:
                pass
        raise error
    assert result is not None
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Synlig Phase-A raised-wheel AUTO_IN_ROW_TURN timeout HIL")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    parser.add_argument("--confirm-turn-not-calibrated", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_turn_phase_a_visible(TurnPhaseAVisibleRequest(**vars(args)))
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
