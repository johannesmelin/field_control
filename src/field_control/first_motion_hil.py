"""Deliberately tiny, raised-wheel-only first-motion hardware runner.

This module is not a navigation entrypoint.  It can issue one fixed, positive
2 motor-RPM command (before gearbox), then relies on the independent control-lease watchdog to
stop it.  It deliberately contains no AUTO path and no speed CLI option.
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
from .runtime import CONTROL_LEASE_EXPIRED, FieldControlRuntime
from .sources import LatestValue
from .verified_motor_boundary import open_verified_boundary


CAN_CHANNEL = "can0"
REPLY_PROFILE = "observed-rmdx-same-id"
FIRST_MOTION_RPM = 2.0
LEASE_TIMEOUT_S = 0.20
WAIT_MARGIN_S = 0.10


class _StaticSource:
    """Hardware-independent source used only to keep MANUAL runtime alive."""
    def __init__(self) -> None:
        self.latest: LatestValue[object] = LatestValue()

    def start(self) -> None: pass
    def stop(self) -> None: pass


@dataclass(frozen=True)
class FirstMotionRequest:
    slcan_device: str
    side: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False

    def validate(self) -> "FirstMotionRequest":
        if self.enable_motors is not True:
            raise ValueError("--enable-motors krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_wheels_raised is not True:
            raise ValueError("--confirm-wheels-raised krävs")
        if self.side not in ("left", "right"):
            raise ValueError("side måste vara left eller right")
        path = self.slcan_device
        prefix = "/dev/serial/by-id/"
        basename = path[len(prefix):] if isinstance(path, str) and path.startswith(prefix) else ""
        if not isinstance(path, str) or not basename or basename in (".", "..") or "/" in basename:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class FirstMotionResult:
    side: str
    command_motor_rpm: tuple[float, float]
    expected_wheel_rpm: tuple[float, float]
    fault: str | None
    diagnostics: dict[str, Any]


def _command_for_side(side: str) -> WheelCommand:
    if side == "left":
        return WheelCommand(FIRST_MOTION_RPM, 0.0, "hil-first-motion-left")
    if side == "right":
        return WheelCommand(0.0, FIRST_MOTION_RPM, "hil-first-motion-right")
    raise ValueError("side måste vara left eller right")


def _bounded_diagnostics(motor: object) -> dict[str, Any]:
    """Return a small post-close diagnostic summary without reopening output."""
    sink = getattr(motor, "_sink", motor)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if callable(snapshot):
        try:
            value = snapshot()
            return {"worker": repr(value)[:2000]}
        except Exception as exc:
            return {"worker_error": f"{type(exc).__name__}: {exc}"[:2000]}
    events = getattr(motor, "events", ())
    return {"adapter_events": [repr(item)[:240] for item in list(events)[-8:]]}


def run_first_motion(request: FirstMotionRequest) -> FirstMotionResult:
    """Run exactly one fixed command and require watchdog revocation afterward."""
    request.validate()  # Must happen before importing/opening physical output.
    lease = ControlLease(LEASE_TIMEOUT_S)
    motor: object | None = None
    runtime: FieldControlRuntime | None = None
    result: FirstMotionResult | None = None
    error: BaseException | None = None
    try:
        motor = open_verified_boundary(
            channel=CAN_CHANNEL, slcan_device=request.slcan_device,
            max_rpm=FIRST_MOTION_RPM, lease=lease,
        )
        config = RuntimeConfig(
            stream_enabled=False,
            max_rpm=FIRST_MOTION_RPM,
            control_lease_timeout_s=LEASE_TIMEOUT_S,
            watchdog_period_s=.020,
            max_control_stall_s=.120,
            physical_can=PhysicalCanConfig(
                enabled=True, channel=CAN_CHANNEL, reply_profile=REPLY_PROFILE,
                slcan_device=request.slcan_device,
                confirm_physical_stop_tested=True, confirm_wheels_raised=True,
            ),
        )
        runtime = FieldControlRuntime(config, _StaticSource(), _StaticSource(), motor=motor, lease=lease)
        runtime.start()
        runtime.arm_motor_output()  # verified 0x81 + fresh 0x9C zero-speed settle
        token = runtime._lease_token
        if token is None:
            raise RuntimeError("armering gav ingen control-lease")
        command = _command_for_side(request.side)
        # Deliberately bypasses manual_command(): that method refreshes the
        # lease.  This is the sole command admission in this runner.
        motor.command(command, token)  # type: ignore[attr-defined]
        deadline = time.monotonic() + LEASE_TIMEOUT_S + WAIT_MARGIN_S
        while time.monotonic() < deadline:
            status = runtime.status()
            # Observing completion must not mutate the lease.  Only the
            # independent watchdog owns physical expiry/revocation.
            if status.fault is not None and not status.motor_output_armed:
                if status.fault != CONTROL_LEASE_EXPIRED:
                    raise RuntimeError(f"oväntat first-motion-fel: {status.fault}")
                result = FirstMotionResult(
                    request.side, (command.left_rpm, command.right_rpm),
                    tuple(motor_rpm_to_wheel_rpm(value, config.odometry_geometry)
                          for value in (command.left_rpm, command.right_rpm)), status.fault, {},
                )
                break
            time.sleep(.005)
        if result is None:
            raise TimeoutError("watchdog återkallade inte first-motion lease inom bounded deadline")
    except BaseException as exc:
        error = exc
    finally:
        if runtime is not None:
            try:
                runtime.close()
            except BaseException as exc:
                if error is None:
                    error = exc
        elif motor is not None:
            try:
                motor.close()  # type: ignore[attr-defined]
            except BaseException as exc:
                if error is None:
                    error = exc
        diagnostics = _bounded_diagnostics(motor) if motor is not None else {"adapter_events": []}
    if error is not None:
        # Preserve the original failure type for automation while retaining a
        # small post-close diagnostic record for the operator.
        try:
            setattr(error, "diagnostics", diagnostics)
        except Exception:
            pass
        raise error
    assert result is not None
    return FirstMotionResult(result.side, result.command_motor_rpm, result.expected_wheel_rpm, result.fault, diagnostics)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Raised-wheel-only first physical wheel motion")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_first_motion(FirstMotionRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({"ok": True, "side": result.side, "command_motor_rpm": result.command_motor_rpm,
                      "expected_wheel_rpm": result.expected_wheel_rpm,
                      "fault": result.fault, "diagnostics": result.diagnostics}, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
