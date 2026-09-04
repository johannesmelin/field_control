"""Non-actuating preflight for a future raised-wheel automatic-turn HIL.

This module deliberately does not open CAN, start DepthAI, arm a motor, or
construct a runtime.  The production turn path needs one coherent stream of
fresh physical IMU heading and per-wheel odometry *while it is running*.
There is currently no bounded, hardware-backed HIL source that can provide
that stream without also bypassing the normal OAK acquisition path.  A mock
or a CLI-supplied heading would make a physical turn appear verified when it
was not, so it is prohibited here.

The preflight fixes the exact intended first test at the existing
``AUTO_IN_ROW_TURN`` configuration and exposes its immutable geometry.  It is
not a calibration result and cannot be used to initiate motion.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math

from .config import RuntimeConfig
from .state_machine import SafetyConfig
from .turn import DifferentialTurnPlan, in_row_turn_plan


@dataclass(frozen=True)
class TurnHilPreflightRequest:
    slcan_device: str
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False
    confirm_turn_not_calibrated: bool = False

    def validate(self) -> "TurnHilPreflightRequest":
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_wheels_raised is not True:
            raise ValueError("--confirm-wheels-raised krävs")
        if self.confirm_turn_not_calibrated is not True:
            raise ValueError("--confirm-turn-not-calibrated krävs")
        prefix = "/dev/serial/by-id/"
        path = self.slcan_device
        basename = path[len(prefix):] if isinstance(path, str) and path.startswith(prefix) else ""
        if not isinstance(path, str) or not basename or basename in (".", "..") or "/" in basename:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class TurnHilPreflightResult:
    can_path: str
    state: str
    direction: str
    inherited_wheel_degrees: float
    plan: DifferentialTurnPlan
    turn_speed_motor_rpm: float
    heading_target_change_deg: float
    motion_enabled: bool
    blocker: str


def run_turn_preflight(request: TurnHilPreflightRequest,
                       config: RuntimeConfig | None = None) -> TurnHilPreflightResult:
    """Validate the fixed first-turn profile without any hardware access."""
    request.validate()
    # This preflight deliberately has no speed/direction/time CLI knobs.  Its
    # fixed nominal motor-side value only proves that a non-zero controller
    # profile can be formed; it never reaches a motor boundary.
    selected = (config or RuntimeConfig(stream_enabled=False, max_rpm=10.0,
                                        turn_speed_rpm=10.0,
                                        safety=SafetyConfig(in_row_turn_enabled=True))).validate()
    safety = selected.safety
    if safety.in_row_turn_enabled is not True:
        raise ValueError("in_row_turn_enabled måste vara true för AUTO_IN_ROW_TURN-HIL-förberedelse")
    # A zero speed/default deployment config cannot describe a HIL movement.
    # Report it explicitly instead of accepting a profile that could never
    # exercise the normal controller.
    if not all(math.isfinite(value) and value > 0
               for value in (selected.turn_speed_rpm, selected.max_rpm)):
        raise ValueError("turn_speed_rpm och max_rpm måste vara positiva för turn-HIL-förberedelse")
    plan = in_row_turn_plan(selected.odometry_geometry, safety.in_row_turn_wheel_degrees,
                            safety.new_row_turn_direction)
    return TurnHilPreflightResult(
        request.slcan_device, "AUTO_IN_ROW_TURN", safety.new_row_turn_direction,
        safety.in_row_turn_wheel_degrees, plan, selected.turn_speed_rpm, 180.0,
        False,
        "Ingen fysisk automatisk turn-HIL körs: en integrerad, färsk OAK/BNO086-heading "
        "och delad fysisk per-hjulsodometri måste först verifieras genom den normala runtime-vägen.",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Non-actuating automatic-turn HIL preflight")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    parser.add_argument("--confirm-turn-not-calibrated", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_turn_preflight(TurnHilPreflightRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000]}))
        return 2
    print(json.dumps({
        "ok": True, "can_path": result.can_path, "state": result.state,
        "direction": result.direction, "inherited_wheel_degrees": result.inherited_wheel_degrees,
        "left_distance_m": result.plan.left_distance_m, "right_distance_m": result.plan.right_distance_m,
        "left_ratio": result.plan.left_ratio, "right_ratio": result.plan.right_ratio,
        "turn_speed_motor_rpm": result.turn_speed_motor_rpm,
        "heading_target_change_deg": result.heading_target_change_deg,
        "motion_enabled": result.motion_enabled, "blocker": result.blocker,
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
