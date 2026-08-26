"""Preparation-only raised-wheel observation runner with one fixed profile.

It has no generic motor controls and must only be invoked after a later,
separate operator decision.  Its duration is a nominal program observation
window, not a claim about measured physical motor-stop timing.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import time
from typing import Any

from .config import PhysicalCanConfig, RuntimeConfig
from .control import WheelCommand
from .lease import ControlLease
from .odometry import motor_rpm_to_wheel_rpm
from .runtime import FieldControlRuntime
from .sources import LatestValue
from .verified_motor_boundary import open_verified_boundary


CAN_CHANNEL = "can0"
REPLY_PROFILE = "observed-rmdx-same-id"
OBSERVED_MOTION_RPM = 10.0
NOMINAL_OBSERVATION_S = 10.0
REFRESH_PERIOD_S = 0.100
LEASE_TIMEOUT_S = 0.200

def _monotonic() -> float:
    return time.monotonic()


def _bounded_sleep(seconds: float) -> None:
    time.sleep(seconds)


class _StaticSource:
    """MANUAL-only no-op source; no camera, DepthAI or cv2 dependency."""
    def __init__(self) -> None:
        self.latest: LatestValue[object] = LatestValue()
    def start(self) -> None: pass
    def stop(self) -> None: pass


@dataclass(frozen=True)
class ObservedMotionRequest:
    slcan_device: str
    side: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False

    def validate(self) -> "ObservedMotionRequest":
        if self.enable_motors is not True:
            raise ValueError("--enable-motors krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_wheels_raised is not True:
            raise ValueError("--confirm-wheels-raised krävs")
        if self.side not in ("left", "right"):
            raise ValueError("side måste vara left eller right")
        prefix = "/dev/serial/by-id/"
        path = self.slcan_device
        basename = path[len(prefix):] if isinstance(path, str) and path.startswith(prefix) else ""
        if not isinstance(path, str) or not basename or basename in (".", "..") or "/" in basename:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class ObservedMotionResult:
    side: str
    command_motor_rpm: tuple[float, float]
    expected_wheel_rpm: tuple[float, float]
    requested_nominal_s: float
    measured_program_window_s: float
    diagnostics: dict[str, Any]


def _command(side: str) -> WheelCommand:
    if side == "left":
        return WheelCommand(OBSERVED_MOTION_RPM, 0.0, "hil-observed-motion-left")
    if side == "right":
        return WheelCommand(0.0, OBSERVED_MOTION_RPM, "hil-observed-motion-right")
    raise ValueError("side måste vara left eller right")


def _bounded_diagnostics(motor: object) -> dict[str, Any]:
    sink = getattr(motor, "_sink", motor)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if callable(snapshot):
        try:
            return {"worker": repr(snapshot())[:2000]}
        except Exception as exc:
            return {"worker_error": f"{type(exc).__name__}: {exc}"[:2000]}
    return {"adapter_events": [repr(item)[:240] for item in list(getattr(motor, "events", ()))[-8:]]}


def run_observed_motion(request: ObservedMotionRequest) -> ObservedMotionResult:
    """Execute the fixed nominal profile, or fail closed before success."""
    request.validate()  # Complete all gates before any physical open.
    lease = ControlLease(LEASE_TIMEOUT_S, clock=_monotonic)
    motor: object | None = None
    runtime: FieldControlRuntime | None = None
    result: ObservedMotionResult | None = None
    error: BaseException | None = None
    try:
        motor = open_verified_boundary(channel=CAN_CHANNEL, slcan_device=request.slcan_device,
                                       max_rpm=OBSERVED_MOTION_RPM, lease=lease)
        config = RuntimeConfig(
            stream_enabled=False, max_rpm=OBSERVED_MOTION_RPM,
            control_lease_timeout_s=LEASE_TIMEOUT_S, watchdog_period_s=.020,
            max_control_stall_s=.120,
            physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
        )
        runtime = FieldControlRuntime(config, _StaticSource(), _StaticSource(), motor=motor, lease=lease, clock=_monotonic)
        runtime.start()
        runtime.arm_motor_output()
        command = _command(request.side)
        started = _monotonic(); deadline = started + NOMINAL_OBSERVATION_S; next_refresh = started
        while _monotonic() < deadline:
            if deadline - _monotonic() <= 1e-9:
                _bounded_sleep(deadline - _monotonic())
                continue
            status = runtime.status()
            if status.fault is not None or not status.motor_output_armed:
                raise RuntimeError(f"observerad motion avbröts: {status.fault or 'motorutgång disarmerad'}")
            runtime.manual_command(command)  # Sole nonzero path; refreshes shared lease.
            after_command = _monotonic()
            if after_command >= deadline:
                break
            next_refresh += REFRESH_PERIOD_S
            wait_s = max(0.0, min(next_refresh, deadline) - after_command)
            if wait_s:
                before_sleep = _monotonic(); _bounded_sleep(wait_s)
                if _monotonic() <= before_sleep:
                    raise RuntimeError("monotonisk refreshklocka avancerade inte")
        runtime.stop()  # Explicit STOP before adapter-owned verified close/settle.
        status = runtime.status()
        if status.fault is not None or status.motor_output_armed:
            raise RuntimeError(f"observerad motion STOP verifierades inte: {status.fault or 'fortfarande armerad'}")
        result = ObservedMotionResult(
            request.side, (command.left_rpm, command.right_rpm),
            tuple(motor_rpm_to_wheel_rpm(value, config.odometry_geometry)
                  for value in (command.left_rpm, command.right_rpm)),
            NOMINAL_OBSERVATION_S, _monotonic() - started, {},
        )
    except BaseException as exc:
        error = exc
    finally:
        if runtime is not None:
            try: runtime.close()
            except BaseException as exc:
                if error is None: error = exc
        elif motor is not None:
            try: motor.close()  # type: ignore[attr-defined]
            except BaseException as exc:
                if error is None: error = exc
        diagnostics = _bounded_diagnostics(motor) if motor is not None else {"adapter_events": []}
    if error is not None:
        try: setattr(error, "diagnostics", diagnostics)
        except Exception: pass
        raise error
    assert result is not None
    return ObservedMotionResult(result.side, result.command_motor_rpm, result.expected_wheel_rpm, result.requested_nominal_s,
                                result.measured_program_window_s, diagnostics)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Prepared raised-wheel observation: fixed +10 RPM nominal 10 s")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_observed_motion(ObservedMotionRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({"ok": True, "side": result.side, "command_motor_rpm": result.command_motor_rpm,
                      "expected_wheel_rpm": result.expected_wheel_rpm,
                      "requested_nominal_s": result.requested_nominal_s,
                      "measured_program_window_s": result.measured_program_window_s,
                      "physical_duration_note": "requires raised-wheel HIL observation",
                      "diagnostics": result.diagnostics}, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
