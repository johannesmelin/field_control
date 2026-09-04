"""Stopped, raised-wheel encoder preflight over the verified CAN worker.

This is deliberately an observation-only HIL entrypoint.  It never arms a
motor boundary, obtains a drive lease, starts a runtime, or calls a drive
command.  The verified worker's normal open/close preflight is allowed to send
only its existing STOP and zero-speed-settle traffic; each observation is its
existing atomic 0x92 pair.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import time

from .lease import ControlLease
from .verified_motor_boundary import open_verified_boundary


CAN_CHANNEL = "can0"
ENCODER_SAMPLE_COUNT = 5
ENCODER_SAMPLE_PERIOD_S = 0.100
# 0.10 motor degrees is ten times the documented 0.01-degree reporting
# granularity.  It is a stationary sanity limit, not a precision claim.
STATIONARY_DELTA_TOLERANCE_MOTOR_DEG = 0.10


@dataclass(frozen=True)
class EncoderPreflightRequest:
    slcan_device: str
    enable_can: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False

    def validate(self) -> "EncoderPreflightRequest":
        if self.enable_can is not True:
            raise ValueError("--enable-can krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_wheels_raised is not True:
            raise ValueError("--confirm-wheels-raised krävs")
        prefix = "/dev/serial/by-id/"
        path = self.slcan_device
        basename = path[len(prefix):] if isinstance(path, str) and path.startswith(prefix) else ""
        if not isinstance(path, str) or not basename or basename in (".", "..") or "/" in basename:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class EncoderPreflightResult:
    can_path: str
    raw_motor_angles_deg: tuple[tuple[float, float], ...]
    deltas_from_first_motor_deg: tuple[tuple[float, float], ...]
    sample_intervals_s: tuple[float, ...]


def _finite_pair(value: object) -> tuple[float, float]:
    if (not isinstance(value, tuple) or len(value) != 2
            or not all(isinstance(item, (int, float)) and not isinstance(item, bool)
                       and math.isfinite(item) for item in value)):
        raise ValueError("encoderavläsningen innehåller ogiltiga motorvinklar")
    return float(value[0]), float(value[1])


def _attach_diagnostics(error: BaseException, boundary: object | None) -> None:
    """Attach the bounded post-close CAN ring without reopening hardware."""
    sink = getattr(boundary, "_sink", boundary)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    try:
        value = snapshot() if callable(snapshot) else ()
        setattr(error, "diagnostics", {"worker": repr(value)[:6000]})
    except Exception as exc:
        setattr(error, "diagnostics", {"worker_error": f"{type(exc).__name__}: {exc}"[:1000]})


def run_encoder_preflight(request: EncoderPreflightRequest) -> EncoderPreflightResult:
    """Read five fixed 10 Hz atomic 0x92 pairs while the raised wheels stay still."""
    request.validate()  # All physical gates must succeed before CAN is opened.
    boundary: object | None = None
    error: BaseException | None = None
    angles: list[tuple[float, float]] = []
    timestamps: list[float] = []
    try:
        # max_rpm is only a constructor limit for the verified boundary.  This
        # runner neither arms it nor invokes its command method.
        boundary = open_verified_boundary(
            channel=CAN_CHANNEL, slcan_device=request.slcan_device,
            max_rpm=1.0, lease=ControlLease(0.20),
        )
        backend_factory = getattr(boundary, "encoder_backend", None)
        if not callable(backend_factory):
            raise RuntimeError("verifierad CAN-gräns saknar delad encoderadapter")
        backend = backend_factory()
        read_angles = getattr(backend, "angles", None)
        if not callable(read_angles):
            raise RuntimeError("delad encoderadapter saknar angles()")

        schedule_start = time.monotonic()
        for sample_index in range(ENCODER_SAMPLE_COUNT):
            wait_s = schedule_start + sample_index * ENCODER_SAMPLE_PERIOD_S - time.monotonic()
            if wait_s > 0:
                time.sleep(wait_s)
            pair = _finite_pair(read_angles())
            timestamp = time.monotonic()
            if timestamps and timestamp <= timestamps[-1]:
                raise RuntimeError("encoderprovets monotona tidsstämplar ökade inte")
            angles.append(pair)
            timestamps.append(timestamp)

        first_left, first_right = angles[0]
        deltas = tuple((left - first_left, right - first_right) for left, right in angles)
        if any(abs(delta) > STATIONARY_DELTA_TOLERANCE_MOTOR_DEG
               for pair in deltas for delta in pair):
            raise RuntimeError(
                "encoderläge ändrades mer än den fasta stillaståendegränsen "
                f"{STATIONARY_DELTA_TOLERANCE_MOTOR_DEG:.2f} motorgrader"
            )
        return EncoderPreflightResult(
            request.slcan_device, tuple(angles), deltas,
            tuple(later - earlier for earlier, later in zip(timestamps, timestamps[1:])),
        )
    except BaseException as exc:
        error = exc
        raise
    finally:
        if boundary is not None:
            try:
                boundary.close()  # verified bounded STOP + 0x9C settle
            except BaseException:
                if error is None:
                    raise
        if error is not None:
            _attach_diagnostics(error, boundary)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Stopped raised-wheel CAN encoder preflight")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-can", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_encoder_preflight(EncoderPreflightRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({
        "ok": True,
        "can_path": result.can_path,
        "raw_motor_angles_deg": result.raw_motor_angles_deg,
        "deltas_from_first_motor_deg": result.deltas_from_first_motor_deg,
        "sample_intervals_s": result.sample_intervals_s,
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
