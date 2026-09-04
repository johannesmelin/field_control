"""One bounded raised-wheel A2 -> verified STOP/0x9C diagnostic.

This is deliberately a narrow diagnostic, not a drive facility.  It admits
exactly one paired logical-forward A2 command at no more than 40 motor RPM,
waits no longer than one second, then asks the established verified boundary
to perform its normal STOP plus zero-speed settle.  The verified CAN worker
remains the only owner of the socket, protocol frames, reply matching and
stale-frame handling.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import re
import threading
import time
from typing import Any

from .control import WheelCommand
from .lease import ControlLease
from .stop_settle_diagnostic import _entry_to_json, _write_report
from .verified_motor_boundary import open_verified_boundary


DEFAULT_CAN_CHANNEL = "can0"
DEFAULT_MOTOR_RPM = 10.0
DEFAULT_DURATION_S = 0.7
MAX_MOTOR_RPM = 40.0
MAX_DURATION_S = 0.7
AUTO_LIKE_DURATION_S = 3.0
# Opt-in repeated A2 mode is deliberately slow enough to leave the CAN worker
# time for reply handling and is bounded by the same movement deadline.
A2_REPEAT_INTERVAL_S = 0.1
# A continuously active independent watchdog owns a zero-output request when
# its lease expires.  It is started before the first A2 admission, then uses
# the same lease after that lease is rebased to the actual motion epoch.
LEASE_TIMEOUT_S = 0.8
AUTO_LIKE_LEASE_TIMEOUT_S = AUTO_LIKE_DURATION_S + 0.1
WATCHDOG_START_TIMEOUT_S = 0.050
_STATUS_SAMPLE_RE = re.compile(r"0x9c sample (-?\d+) dps after ([0-9.]+) ms", re.IGNORECASE)


@dataclass(frozen=True)
class MotionStopDiagnosticRequest:
    slcan_device: str
    can_channel: str = DEFAULT_CAN_CHANNEL
    motor_rpm: float = DEFAULT_MOTOR_RPM
    duration_s: float = DEFAULT_DURATION_S
    recurring_a2: bool = False
    auto_like_window: bool = False
    enable_can: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False

    def validate(self) -> "MotionStopDiagnosticRequest":
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
        if (not isinstance(self.can_channel, str) or not self.can_channel
                or "/" in self.can_channel or any(char.isspace() for char in self.can_channel)):
            raise ValueError("exakt CAN-gränssnitt krävs")
        if not isinstance(self.recurring_a2, bool) or not isinstance(self.auto_like_window, bool):
            raise ValueError("recurring_a2 och auto_like_window måste vara booleska")
        maximum_duration_s = AUTO_LIKE_DURATION_S if self.auto_like_window else MAX_DURATION_S
        for name, value, maximum in (("motor_rpm", self.motor_rpm, MAX_MOTOR_RPM),
                                     ("duration_s", self.duration_s, maximum_duration_s)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or not 0 < float(value) <= maximum):
                raise ValueError(f"{name} måste vara positiv, ändlig och högst {maximum}")
        if self.auto_like_window and float(self.duration_s) != AUTO_LIKE_DURATION_S:
            raise ValueError(f"auto_like_window kräver exakt {AUTO_LIKE_DURATION_S} s rörelsefönster")
        return self


@dataclass(frozen=True)
class MotionStopDiagnosticResult:
    request: MotionStopDiagnosticRequest
    command_started_monotonic_s: float | None
    stop_started_monotonic_s: float | None
    stop_completed_monotonic_s: float | None
    final_outcome: str
    error: str | None
    close_error: str | None
    lease_watchdog_triggered: bool
    worker_diagnostics: tuple[dict[str, Any], ...]
    report_path: str


def _post_close_snapshot(boundary: object | None) -> tuple[dict[str, Any], ...]:
    sink = getattr(boundary, "_sink", None)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if not callable(snapshot):
        return ()
    try:
        return tuple(_entry_to_json(entry) for entry in snapshot())
    except Exception as exc:
        return ({"diagnostic_snapshot_error": f"{type(exc).__name__}: {exc}"[:1000]},)


def _classify_outcome(error: BaseException | None) -> str:
    if error is None:
        return "settled"
    text = str(error).lower()
    if "nådde inte 0 dps" in text:
        return "nonzero_dps_deadline"
    if "timeout" in text or "svarstid" in text:
        return "reply_timeout"
    return "failed"


def _expire_lease(lease: ControlLease, fired: threading.Event, cancelled: threading.Event,
                  started: threading.Event | None = None) -> None:
    """Ask the existing lease callback to queue safe zero output on expiry.

    This timer thread performs no CAN I/O itself.  ``ControlLease`` invokes
    the verified boundary's established revoke callback, which owns the
    worker queueing and is serialized with any admitted command.
    """
    # Acknowledge execution before the first poll so the caller cannot admit
    # A2 based solely on Timer.start() returning.
    if started is not None:
        started.set()
    # Timers normally do not fire early, but the loop makes that scheduler
    # property non-safety-critical.  It also permits normal STOP to cancel an
    # already-starting timer without a competing lease revocation.
    while not cancelled.is_set():
        if lease.watchdog_tick():
            fired.set()
            return
        cancelled.wait(.001)


def _summary(entries: tuple[dict[str, Any], ...]) -> dict[str, Any]:
    """Summarise worker-authored evidence without interpreting raw CAN bytes."""
    status_samples: list[dict[str, float | str]] = []
    for entry in entries:
        detail = str(entry.get("detail", ""))
        match = _STATUS_SAMPLE_RE.search(detail)
        if match is not None:
            status_samples.append({"detail": detail, "dps": float(match.group(1)),
                                   "elapsed_ms": float(match.group(2))})
    stale_entries = [entry for entry in entries
                     if "stale" in str(entry.get("detail", "")).lower()]
    return {"status_samples": status_samples, "stale_reply_or_frame_count": len(stale_entries)}


def _lease_timeout_s(request: MotionStopDiagnosticRequest) -> float:
    """Return the watchdog deadline for the selected fixed diagnostic window."""
    return AUTO_LIKE_LEASE_TIMEOUT_S if request.auto_like_window else LEASE_TIMEOUT_S


def _run_motion_window(*, command: Any, lease_token: str, request: MotionStopDiagnosticRequest,
                       started_s: float, on_first_command_sent: Any | None = None,
                       clock: Any | None = None, sleep: Any | None = None) -> None:
    """Send the sole A2, optionally refreshed at a maximum 10 Hz until deadline.

    A late scheduler wake-up never emits catch-up commands, and no command is
    admitted at or after the absolute movement deadline.
    """
    clock = time.monotonic if clock is None else clock
    sleep = time.sleep if sleep is None else sleep
    forward = WheelCommand(float(request.motor_rpm), float(request.motor_rpm),
                           "motion-stop-diagnostic-forward")
    command(forward, lease_token)
    # The first A2 transaction is bounded but can take non-zero time.  Treat
    # its return as the motion epoch so the fixed movement window and its
    # independent watchdog use the same monotonic origin.
    motion_started_s = max(float(started_s), float(clock()))
    if on_first_command_sent is not None:
        on_first_command_sent(motion_started_s)
    if not (request.recurring_a2 or request.auto_like_window):
        sleep(float(request.duration_s))
        return

    deadline_s = motion_started_s + float(request.duration_s)
    # The first recurring interval starts when the first A2 admission has
    # actually returned.  This prevents a slow worker admission from being
    # followed by an immediate catch-up frame.
    next_command_s = motion_started_s + A2_REPEAT_INTERVAL_S
    while True:
        now_s = float(clock())
        if now_s >= deadline_s:
            return
        wait_s = min(max(0.0, next_command_s - now_s), deadline_s - now_s)
        if wait_s > 0.0:
            sleep(wait_s)
            # A scheduler can wake early.  Re-check the eligible-send time
            # rather than treating the requested sleep as a guarantee.
            continue
        # The current monotonic time is now at least the last actual-send
        # time plus the recurrence interval.
        now_s = float(clock())
        if now_s >= deadline_s:
            return
        command(forward, lease_token)
        # Do not burst missed A2 frames after a delayed wake-up or a slow
        # command admission.  The next one is eligible only 100 ms after the
        # actual send returned.
        next_command_s = float(clock()) + A2_REPEAT_INTERVAL_S


def run_motion_stop_diagnostic(request: MotionStopDiagnosticRequest) -> MotionStopDiagnosticResult:
    """Run one bounded logical-forward A2 pair followed by verified STOP."""
    request.validate()  # All physical gates are checked before CAN opens.
    lease_timeout_s = _lease_timeout_s(request)
    lease = ControlLease(lease_timeout_s)
    boundary: object | None = None
    error: BaseException | None = None
    close_error: str | None = None
    command_started: float | None = None
    stop_started: float | None = None
    stop_completed: float | None = None
    watchdog_fired = threading.Event()
    watchdog_cancelled = threading.Event()
    watchdog_started = threading.Event()
    watchdog_timer: threading.Timer | None = None
    try:
        boundary = open_verified_boundary(channel=request.can_channel, slcan_device=request.slcan_device,
                                          max_rpm=float(request.motor_rpm), lease=lease)
        token = lease.acquire()
        arm = getattr(boundary, "arm", None)
        command = getattr(boundary, "command", None)
        settle = getattr(boundary, "stop_and_settle_for_restart", None)
        if not callable(arm) or not callable(command) or not callable(settle):
            raise RuntimeError("verifierad motorgräns saknar obligatorisk diagnostikoperation")
        arm(token)
        # Arm's bounded initial STOP may consume some lease time.  Establish
        # a provisional lease and watchdog before the first A2; this leaves
        # no unprotected interval if the caller stalls in that admission.
        lease.refresh(token)
        watchdog_timer = threading.Timer(0.0, _expire_lease,
                                        args=(lease, watchdog_fired, watchdog_cancelled, watchdog_started))
        watchdog_timer.daemon = True
        watchdog_timer.start()
        if not watchdog_started.wait(WATCHDOG_START_TIMEOUT_S):
            raise RuntimeError("diagnostikens watchdog-tråd startade inte inom bunden tid")

        def start_motion_watchdog(motion_started_s: float) -> None:
            nonlocal command_started
            # This is an in-place lease rebase.  The already-running
            # watchdog continues polling the same lease, without a cancel,
            # replacement, or protection gap.
            lease.refresh(token)
            command_started = motion_started_s
        _run_motion_window(command=command, lease_token=token, request=request,
                           started_s=time.monotonic(), on_first_command_sent=start_motion_watchdog)
        stop_started = time.monotonic()
        watchdog_cancelled.set()
        if watchdog_timer is not None:
            watchdog_timer.cancel()
        settle("motion-stop-diagnostic")
        stop_completed = time.monotonic()
    except BaseException as exc:
        error = exc
    finally:
        watchdog_cancelled.set()
        if watchdog_timer is not None:
            watchdog_timer.cancel()
        # Always request the verified STOP path if the intended settle did not
        # begin.  close() additionally owns socket release and its bounded
        # shutdown STOP, including after a fault.
        if boundary is not None and stop_started is None:
            stop_started = time.monotonic()
            try:
                settle = getattr(boundary, "stop_and_settle_for_restart", None)
                if callable(settle):
                    settle("motion-stop-diagnostic-finally")
                stop_completed = time.monotonic()
            except BaseException as stop_exc:
                if error is None:
                    error = stop_exc
        if boundary is not None:
            try:
                close = getattr(boundary, "close", None)
                if not callable(close):
                    raise RuntimeError("verifierad motorgräns saknar close()")
                close()
            except BaseException as close_exc:
                close_error = f"{type(close_exc).__name__}: {close_exc}"[:2000]
                if error is None:
                    error = close_exc

    if error is None and watchdog_fired.is_set():
        # A zero request was admitted by the independent deadline before this
        # thread completed its nominal profile.  Do not disguise that timing
        # deviation as a successful motion/STOP measurement.
        error = RuntimeError("diagnostikens oberoende lease-watchdog löste ut före nominellt STOP")

    entries = _post_close_snapshot(boundary)
    outcome = "lease_watchdog_stop" if watchdog_fired.is_set() else _classify_outcome(error)
    error_text = None if error is None else f"{type(error).__name__}: {error}"[:2000]
    payload = {
        "ok": error is None,
        "final_outcome": outcome,
        "request": asdict(request),
        "command_started_monotonic_s": command_started,
        "stop_started_monotonic_s": stop_started,
        "stop_completed_monotonic_s": stop_completed,
        "error": error_text,
        "close_error": close_error,
        "lease_watchdog_triggered": watchdog_fired.is_set(),
        "diagnostic_summary": _summary(entries),
        "worker_diagnostics": list(entries),
    }
    report_path = _write_report(payload, report_prefix="motion_stop")
    return MotionStopDiagnosticResult(request, command_started, stop_started, stop_completed,
                                      outcome, error_text, close_error, watchdog_fired.is_set(), entries,
                                      str(report_path))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Raised-wheel one-pair A2 -> verified STOP diagnostic")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--can-channel", default=DEFAULT_CAN_CHANNEL)
    parser.add_argument("--motor-rpm", type=float, default=DEFAULT_MOTOR_RPM)
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--recurring-a2", action="store_true",
                        help="sänd samma logiska framåt-A2 högst 10 Hz under rörelsefönstret")
    parser.add_argument("--auto-like-window", action="store_true",
                        help="kör det fasta 3,0 s/10 Hz-A2-diagnostikfönstret före STOP")
    parser.add_argument("--enable-can", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        request_args = vars(args)
        if request_args["auto_like_window"]:
            request_args["recurring_a2"] = True
            request_args["duration_s"] = AUTO_LIKE_DURATION_S
        result = run_motion_stop_diagnostic(MotionStopDiagnosticRequest(**request_args))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000]}, ensure_ascii=False))
        return 2
    print(json.dumps({
        "ok": result.final_outcome == "settled",
        "final_outcome": result.final_outcome,
        "error": result.error,
        "close_error": result.close_error,
        "report_path": result.report_path,
    }, ensure_ascii=False))
    return 0 if result.final_outcome == "settled" else 2


if __name__ == "__main__":
    raise SystemExit(main())
