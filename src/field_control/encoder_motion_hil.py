"""Fixed raised-wheel HIL check for shared CAN encoder motion.

The runner is intentionally narrower than a drive or calibration tool.  It
uses the existing verified boundary, its sole shared CAN encoder worker, the
real :class:`OdometrySource`, and the normal runtime lease.  After all
physical gates it moves exactly one selected motor at +2 motor-RPM for a
nominal one second, refreshing the lease every 100 ms, then explicitly stops
before inspecting the last fresh odometry sample.

Only absolute encoder-derived distance change is checked: the existing
physical sign and protocol handling remain authoritative, and this HIL test
does not infer a direction from them.  The 1 mm inactive-side guard is an
acceptance limit, not a calibration claim.  It is far above one 0x92 encoder
LSB after the configured 8:1 conversion (about 0.003 mm), yet substantially
below the nominal selected-side travel at this profile (about 3.4 mm).  Any
larger inactive-side reading fails this test rather than being explained away.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import time
from typing import Any

from .config import PhysicalCanConfig, RuntimeConfig
from .control import WheelCommand
from .lease import ControlLease
from .odometry import OdometrySample, motor_rpm_to_wheel_rpm
from .runtime import FieldControlRuntime
from .sources import LatestValue, OdometrySource, SourceSnapshot
from .verified_motor_boundary import open_verified_boundary


CAN_CHANNEL = "can0"
REPLY_PROFILE = "observed-rmdx-same-id"
ENCODER_MOTION_RPM = 2.0
NOMINAL_MOTION_S = 1.0
REFRESH_PERIOD_S = 0.100
LEASE_TIMEOUT_S = 0.200
INACTIVE_SIDE_MAX_DELTA_M = 0.001
ACTIVE_SIDE_MIN_DELTA_M = 0.001


def _monotonic() -> float:
    return time.monotonic()


def _bounded_sleep(seconds: float) -> None:
    time.sleep(seconds)


class _StaticSource:
    """MANUAL-only source which cannot start AUTO navigation."""

    def __init__(self) -> None:
        self.latest: LatestValue[object] = LatestValue()

    def start(self) -> None:
        return

    def stop(self) -> None:
        return


@dataclass(frozen=True)
class EncoderMotionRequest:
    slcan_device: str
    side: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False

    def validate(self) -> "EncoderMotionRequest":
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
class EncoderMotionResult:
    side: str
    command_motor_rpm: tuple[float, float]
    expected_wheel_rpm: tuple[float, float]
    baseline_odometry: OdometrySample
    final_odometry: OdometrySample
    active_side_delta_m: float
    inactive_side_delta_m: float
    requested_nominal_s: float
    measured_program_window_s: float
    diagnostics: dict[str, Any]


def _command_for_side(side: str) -> WheelCommand:
    if side == "left":
        return WheelCommand(ENCODER_MOTION_RPM, 0.0, "hil-encoder-motion-left")
    if side == "right":
        return WheelCommand(0.0, ENCODER_MOTION_RPM, "hil-encoder-motion-right")
    raise ValueError("side måste vara left eller right")


def _fresh_odometry(source: object, timeout_s: float) -> OdometrySample:
    snapshot_method = getattr(source, "snapshot", None)
    if not callable(snapshot_method):
        raise RuntimeError("odometrikälla saknar snapshot()")
    snapshot = snapshot_method()
    if not isinstance(snapshot, SourceSnapshot):
        raise RuntimeError("odometrikälla returnerade ogiltig snapshot")
    value = snapshot.value
    age = snapshot.age_s(_monotonic())
    if (type(value) is not OdometrySample or not snapshot.connected or age is None
            or age > timeout_s or not all(math.isfinite(component) for component in (
                value.left_distance_m, value.right_distance_m,
                value.forward_distance_m, value.yaw_change_deg,
            ))):
        raise RuntimeError("färsk, ändlig typed fysisk odometri krävs")
    return value


def _bounded_diagnostics(motor: object) -> dict[str, Any]:
    sink = getattr(motor, "_sink", motor)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if callable(snapshot):
        try:
            return {"worker": repr(snapshot())[:2000]}
        except Exception as exc:
            return {"worker_error": f"{type(exc).__name__}: {exc}"[:2000]}
    return {"adapter_events": [repr(item)[:240] for item in list(getattr(motor, "events", ()))[-8:]]}


def run_encoder_motion(request: EncoderMotionRequest) -> EncoderMotionResult:
    """Perform the single fixed profile or fail closed without a result."""
    request.validate()  # Every gate succeeds before the verified CAN worker opens.
    lease = ControlLease(LEASE_TIMEOUT_S, clock=_monotonic)
    motor: object | None = None
    runtime: FieldControlRuntime | None = None
    result: EncoderMotionResult | None = None
    error: BaseException | None = None
    try:
        motor = open_verified_boundary(channel=CAN_CHANNEL, slcan_device=request.slcan_device,
                                       max_rpm=ENCODER_MOTION_RPM, lease=lease)
        encoder_factory = getattr(motor, "encoder_backend", None)
        if not callable(encoder_factory):
            raise RuntimeError("verifierad CAN-gräns saknar delad encoderadapter")
        odometry = OdometrySource(encoder_factory(), RuntimeConfig().odometry_geometry)
        config = RuntimeConfig(
            stream_enabled=False, max_rpm=ENCODER_MOTION_RPM,
            control_lease_timeout_s=LEASE_TIMEOUT_S, watchdog_period_s=.020,
            max_control_stall_s=.120,
            physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
        )
        runtime = FieldControlRuntime(config, _StaticSource(), _StaticSource(), motor=motor,
                                      odometry=odometry, lease=lease, clock=_monotonic)
        runtime.start()
        # This explicit readiness gate is deliberately before arm; arm performs
        # the same check again inside the normal physical runtime boundary.
        if not odometry.wait_until_ready(config.odometry_timeout_s):
            raise RuntimeError("encoderodometri blev inte redo före armering")
        baseline = _fresh_odometry(odometry, config.odometry_timeout_s)
        runtime.arm_motor_output()
        command = _command_for_side(request.side)
        started = _monotonic()
        deadline = started + NOMINAL_MOTION_S
        next_refresh = started
        while True:
            # Do not admit a final queued A2 at an indistinguishable instant
            # before the absolute deadline because of floating-point clock
            # representation.  The deadline still owns STOP in that case.
            if _monotonic() >= deadline - 1e-9:
                break
            status = runtime.status()
            if status.fault is not None or not status.motor_output_armed:
                raise RuntimeError(f"encoder-rörelse avbröts: {status.fault or 'motorutgång disarmerad'}")
            runtime.manual_command(command)
            after_command = _monotonic()
            if after_command >= deadline:
                break
            next_refresh += REFRESH_PERIOD_S
            wait_s = max(0.0, min(next_refresh, deadline) - after_command)
            if wait_s:
                _bounded_sleep(wait_s)
        runtime.stop()  # Explicit STOP owns the deadline before final validation.
        status = runtime.status()
        if status.fault is not None or status.motor_output_armed:
            raise RuntimeError(f"encoder-rörelse STOP verifierades inte: {status.fault or 'fortfarande armerad'}")
        final = _fresh_odometry(odometry, config.odometry_timeout_s)
        left_delta = final.left_distance_m - baseline.left_distance_m
        right_delta = final.right_distance_m - baseline.right_distance_m
        active_delta, inactive_delta = ((left_delta, right_delta) if request.side == "left"
                                        else (right_delta, left_delta))
        if abs(active_delta) < ACTIVE_SIDE_MIN_DELTA_M:
            raise RuntimeError("vald encodersida ändrades mindre än den fasta miniminivån")
        if abs(inactive_delta) > INACTIVE_SIDE_MAX_DELTA_M:
            raise RuntimeError("okommanderad encodersida ändrades över den fasta gränsen")
        result = EncoderMotionResult(
            request.side, (command.left_rpm, command.right_rpm),
            tuple(motor_rpm_to_wheel_rpm(value, config.odometry_geometry)
                  for value in (command.left_rpm, command.right_rpm)),
            baseline, final, active_delta, inactive_delta, NOMINAL_MOTION_S,
            _monotonic() - started, {},
        )
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
                motor.close()  # type: ignore[attr-defined]  # verified STOP + 0x9C settle
            except BaseException as exc:
                if error is None:
                    error = exc
        diagnostics = _bounded_diagnostics(motor) if motor is not None else {"adapter_events": []}
    if error is not None:
        try:
            setattr(error, "diagnostics", diagnostics)
        except Exception:
            pass
        raise error
    assert result is not None
    return EncoderMotionResult(
        result.side, result.command_motor_rpm, result.expected_wheel_rpm,
        result.baseline_odometry, result.final_odometry, result.active_side_delta_m,
        result.inactive_side_delta_m, result.requested_nominal_s,
        result.measured_program_window_s, diagnostics,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Raised-wheel fixed +2 RPM shared encoder-motion HIL")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_encoder_motion(EncoderMotionRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({
        "ok": True, "side": result.side, "command_motor_rpm": result.command_motor_rpm,
        "expected_wheel_rpm": result.expected_wheel_rpm,
        "baseline_odometry": result.baseline_odometry,
        "final_odometry": result.final_odometry,
        "active_side_delta_m": result.active_side_delta_m,
        "inactive_side_delta_m": result.inactive_side_delta_m,
        "requested_nominal_s": result.requested_nominal_s,
        "measured_program_window_s": result.measured_program_window_s,
        "diagnostics": result.diagnostics,
    }, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
