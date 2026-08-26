"""Fixed Phase-A raised-wheel HIL for the normal automatic in-row turn.

This is deliberately a *failure* test: raised wheels cannot rotate the robot
body, therefore the real OAK/BNO heading must not reach the 180 degree target.
The only passing terminal result is ``TURN_TIMEOUT`` followed by disarm/STOP.
It is not a turn calibration or a successful-turn test.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import time
from typing import Callable

from .app import FieldControlApplication
from .config import HsvFilter, PhysicalCanConfig, RuntimeConfig, VisionConfig, Zone
from .odometry import OdometrySample
from .state_machine import SafetyConfig
from .turn import DifferentialTurnPlan, in_row_turn_plan


CAN_CHANNEL = "can0"
REPLY_PROFILE = "observed-rmdx-same-id"
# This temporary yellow Phase-A test marker is intentionally fixed.  The
# bounds are from the current OAK lower-middle observation: H=26..36,
# S >= 20 and V >= 150.  At these bounds its largest observed component was
# 587 px and the next was 7 px; the normal 100 px minimum therefore remains
# in force. Detection is still performed by the ordinary VisionProcessor; no
# result is injected. Production marker HSV remains configuration supplied.
PHASE_A_MARKER = HsvFilter((26, 20, 150), (36, 255, 255), 100)
MARKER_READY_TIMEOUT_S = 30.0
TURN_TIMEOUT_S = 2.0
TURN_SPEED_MOTOR_RPM = 2.0
POLL_S = .020
# This is an observation guard, not a calibration target.  It matches the
# existing one-millimetre per-wheel raised-wheel HIL acceptance floor.
MIN_ENCODER_DELTA_M = .001


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
    command_sign: tuple[int, int]
    encoder_delta_m: tuple[float, float]
    fault: str
    events: tuple[dict[str, object], ...]


def phase_a_config(request: TurnPhaseARequest) -> RuntimeConfig:
    """The sole non-calibration profile; it exposes no operator knobs."""
    return RuntimeConfig(
        stream_enabled=False, max_rpm=TURN_SPEED_MOTOR_RPM,
        auto_base_rpm=0.0, max_vision_correction_rpm=0.0, vision_kp=0.0,
        search_speed_rpm=0.0, heading_kp=0.0, max_heading_correction_rpm=0.0,
        turn_speed_rpm=TURN_SPEED_MOTOR_RPM,
        navigation_frame_rate_hz=20.0,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device,
                                       True, True),
        vision=VisionConfig(navigation_mode="buds_only", buds=PHASE_A_MARKER,
                            leaves=PHASE_A_MARKER, marker=PHASE_A_MARKER,
                            turn_marker_zone=Zone(.2, .8, .3, 1.0)),
        safety=SafetyConfig(in_row_turn_enabled=True, new_row_turn_direction="left",
                            auto_start_delay_s=0.0,
                            turn_timeout_s=TURN_TIMEOUT_S),
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


def _validate_timeout_evidence(status: object, plan: DifferentialTurnPlan,
                               baseline: OdometrySample, events: tuple[dict[str, object], ...]) -> tuple[tuple[int, int], tuple[float, float]]:
    if getattr(status, "fault", None) != "TURN_TIMEOUT" or getattr(status, "motor_output_armed", True):
        raise RuntimeError(f"Phase-A måste fail-closed med TURN_TIMEOUT, fick {getattr(status, 'fault', None)}")
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
    if (not all(math.isfinite(value) and abs(value) >= MIN_ENCODER_DELTA_M for value in delta)
            or tuple(_sign(value) for value in delta) != expected):
        raise RuntimeError("encoderdeltan är inte ändlig, detekterbar och teckenkonsistent med turn-planen")
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


def run_turn_phase_a(request: TurnPhaseARequest, *, app_factory: Callable[..., FieldControlApplication] = FieldControlApplication,
                     monotonic: Callable[[], float] = time.monotonic,
                     sleep: Callable[[float], None] = time.sleep) -> TurnPhaseAResult:
    """Run the normal app path and require the raised-wheel timeout outcome."""
    request.validate()  # All gates before OAK/CAN/application construction.
    config = phase_a_config(request)
    plan = in_row_turn_plan(config.odometry_geometry, config.safety.in_row_turn_wheel_degrees,
                            config.safety.new_row_turn_direction)
    app: FieldControlApplication | None = None
    error: BaseException | None = None
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
        # Select AUTO while output is still disarmed: its required transition
        # STOP cannot revoke a newly acquired lease.  Then arm, capture the
        # physical baseline, and begin the normal AUTO trigger path.
        app.runtime.select_auto()
        app.runtime.arm_motor_output()
        baseline = _sample(app.runtime.status())
        app.runtime.start_auto()
        deadline = monotonic() + TURN_TIMEOUT_S + 1.0
        while monotonic() < deadline:
            status = app.runtime.status()
            if status.fault is not None:
                events = tuple(app.runtime.events.recent())
                signs, delta = _validate_timeout_evidence(status, plan, baseline, events)
                return TurnPhaseAResult(
                    plan, signs, delta, status.fault, events,
                )
            sleep(POLL_S)
        raise TimeoutError("Phase-A nådde inte bounded TURN_TIMEOUT-deadline")
    except BaseException as exc:
        error = exc
    finally:
        if app is not None:
            try:
                app.close()
            except BaseException as close_exc:
                if error is None:
                    error = close_exc
    assert error is not None
    raise error


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Phase-A raised-wheel AUTO_IN_ROW_TURN timeout HIL")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    parser.add_argument("--confirm-turn-not-calibrated", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_turn_phase_a(TurnPhaseARequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000]}))
        return 2
    print(json.dumps({"ok": True, "fault": result.fault, "direction": result.plan.direction,
                      "plan": {"left_distance_m": result.plan.left_distance_m,
                               "right_distance_m": result.plan.right_distance_m,
                               "left_ratio": result.plan.left_ratio, "right_ratio": result.plan.right_ratio},
                      "command_sign": result.command_sign, "encoder_delta_m": result.encoder_delta_m,
                      "events": result.events}, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
