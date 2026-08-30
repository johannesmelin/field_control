"""Bounded STOP/0x9C HIL diagnostic for an intermittent stop-settle fault.

This entry point intentionally has no drive, arming, A2, or A4 path.  It
opens the established verified CAN boundary, requests only its existing
verified STOP+0x9C settle operation a fixed number of times, and records the
worker's post-close diagnostic ring.  The worker remains the sole owner of
the CAN socket and of 0x9C framing/reply validation.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import stat
import time
from typing import Any, Iterable

from .lease import ControlLease
from .verified_motor_boundary import open_verified_boundary


DEFAULT_CAN_CHANNEL = "can0"
DEFAULT_SAMPLE_COUNT = 10
MAX_SAMPLE_COUNT = 30
SAMPLE_PERIOD_S = 0.100
DIAGNOSTICS_DIRECTORY = Path(__file__).resolve().parents[1] / "diagnostics"
REPORT_CREATE_ATTEMPTS = 8
_STATUS_SAMPLE_RE = re.compile(r"0x9C sample (-?\d+) dps after ([0-9.]+) ms")


@dataclass(frozen=True)
class StopSettleDiagnosticRequest:
    slcan_device: str
    can_channel: str = DEFAULT_CAN_CHANNEL
    sample_count: int = DEFAULT_SAMPLE_COUNT
    enable_can: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False

    def validate(self) -> "StopSettleDiagnosticRequest":
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
        if (not isinstance(self.sample_count, int) or isinstance(self.sample_count, bool)
                or not 1 <= self.sample_count <= MAX_SAMPLE_COUNT):
            raise ValueError(f"sample_count måste vara ett heltal 1–{MAX_SAMPLE_COUNT}")
        return self


@dataclass(frozen=True)
class StopSettleAttempt:
    index: int
    started_monotonic_s: float
    completed_monotonic_s: float
    outcome: str
    error: str | None


@dataclass(frozen=True)
class StopSettleDiagnosticResult:
    request: StopSettleDiagnosticRequest
    attempts: tuple[StopSettleAttempt, ...]
    worker_diagnostics: tuple[dict[str, Any], ...]
    final_outcome: str
    close_error: str | None
    report_path: str


def _entry_to_json(entry: object) -> dict[str, Any]:
    """Serialize the worker's immutable post-close ring without CAN I/O."""
    fields = (
        "timestamp_s", "sequence", "phase", "direction", "can_id", "dlc",
        "expected_reply_ids", "pending_reply_ids", "detail",
    )
    result = {name: getattr(entry, name, None) for name in fields}
    data = getattr(entry, "data", None)
    result["data_hex"] = None if data is None else bytes(data).hex()
    for key in ("expected_reply_ids", "pending_reply_ids"):
        value = result[key]
        result[key] = list(value) if isinstance(value, tuple) else []
    return result


def _classify_failure(error: BaseException | None, entries: Iterable[dict[str, Any]]) -> str:
    """Classify evidence without converting an unconfirmed STOP into success."""
    text = "" if error is None else str(error).lower()
    details = "\n".join(str(entry.get("detail") or "").lower() for entry in entries)
    if "svarstimeout" in text or "timed out" in text or "timeout" in text:
        return "reply_timeout"
    if "nådde inte 0 dps" in text:
        if any(_STATUS_SAMPLE_RE.search(str(entry.get("detail") or "")) for entry in entries):
            return "nonzero_dps_deadline"
        return "timing_deadline"
    if "saknar återstående settle-tid" in text or "settle-deadline" in text:
        return "timing_deadline"
    if "retry omitted: insufficient remaining settle time" in details:
        return "timing_deadline"
    return "other_failure"


def _write_report(payload: dict[str, Any], *, report_prefix: str = "stop_settle") -> Path:
    """Create, never overwrite, a private local project diagnostic report.

    The report directory is opened as a non-symlink directory file descriptor;
    the report itself is then created relative to that descriptor with O_EXCL.
    This keeps a diagnostic path override from redirecting output through a
    symlink and makes same-timestamp collisions bounded/recoverable.
    """
    # Callers supply only a fixed, source-controlled report family.  Keeping
    # the stem conservative prevents a future CLI value from influencing a
    # filesystem path.
    if not re.fullmatch(r"[a-z0-9_]+", report_prefix):
        raise ValueError("ogiltigt diagnostikrapportprefix")
    directory = os.fspath(DIAGNOSTICS_DIRECTORY)
    try:
        os.mkdir(directory, mode=0o700)
    except FileExistsError:
        pass
    mode = os.lstat(directory).st_mode
    if not stat.S_ISDIR(mode):
        raise RuntimeError("diagnostikkatalogen måste vara en verklig katalog, inte länk eller fil")
    directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    directory_fd = os.open(directory, directory_flags)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    try:
        for collision_index in range(REPORT_CREATE_ATTEMPTS):
            suffix = "" if collision_index == 0 else f"_{collision_index}"
            filename = f"{report_prefix}_{timestamp}{suffix}.json"
            try:
                report_fd = os.open(
                    filename, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                    0o600, dir_fd=directory_fd,
                )
            except FileExistsError:
                continue
            try:
                with os.fdopen(report_fd, "w", encoding="utf-8") as report:
                    json.dump(payload, report, ensure_ascii=False, indent=2, sort_keys=True)
                    report.write("\n")
            except BaseException:
                try:
                    os.unlink(filename, dir_fd=directory_fd)
                except OSError:
                    pass
                raise
            return DIAGNOSTICS_DIRECTORY / filename
    finally:
        os.close(directory_fd)
    raise RuntimeError(f"kunde inte skapa unik diagnostikrapport efter {REPORT_CREATE_ATTEMPTS} försök")


def _post_close_snapshot(boundary: object | None) -> tuple[dict[str, Any], ...]:
    sink = getattr(boundary, "_sink", None)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if not callable(snapshot):
        return ()
    try:
        return tuple(_entry_to_json(entry) for entry in snapshot())
    except Exception as exc:
        return ({"diagnostic_snapshot_error": f"{type(exc).__name__}: {exc}"[:1000]},)


def run_stop_settle_diagnostic(request: StopSettleDiagnosticRequest) -> StopSettleDiagnosticResult:
    """Run a fixed, STOP-only sample series and save its post-close evidence."""
    request.validate()
    boundary: object | None = None
    attempts: list[StopSettleAttempt] = []
    terminal_error: BaseException | None = None
    close_error: str | None = None
    try:
        boundary = open_verified_boundary(
            channel=request.can_channel, slcan_device=request.slcan_device,
            max_rpm=1.0, lease=ControlLease(0.20),
        )
        start_s = time.monotonic()
        for index in range(request.sample_count):
            target_s = start_s + index * SAMPLE_PERIOD_S
            wait_s = target_s - time.monotonic()
            if wait_s > 0:
                time.sleep(wait_s)
            attempt_start = time.monotonic()
            try:
                settle = getattr(boundary, "stop_and_settle_for_restart", None)
                if not callable(settle):
                    raise RuntimeError("verifierad motorgräns saknar publik STOP+0x9C-settle")
                settle("stop-settle-diagnostic")
            except BaseException as exc:
                terminal_error = exc
                attempts.append(StopSettleAttempt(
                    index, attempt_start, time.monotonic(), "failed", f"{type(exc).__name__}: {exc}"[:2000],
                ))
                break
            attempts.append(StopSettleAttempt(index, attempt_start, time.monotonic(), "settled", None))
    finally:
        if boundary is not None:
            try:
                boundary.close()  # worker-owned bounded STOP+0x9C and socket release
            except BaseException as close_exc:
                close_error = f"{type(close_exc).__name__}: {close_exc}"[:2000]
                if terminal_error is None:
                    terminal_error = RuntimeError(close_error)
                    now = time.monotonic()
                    attempts.append(StopSettleAttempt(
                        len(attempts), now, now, "close_failed",
                        close_error,
                    ))

    entries = _post_close_snapshot(boundary)
    outcome = "settled" if terminal_error is None else _classify_failure(terminal_error, entries)
    payload = {
        "ok": terminal_error is None,
        "final_outcome": outcome,
        "request": asdict(request),
        "attempts": [asdict(attempt) for attempt in attempts],
        "close_error": close_error,
        "worker_diagnostics": list(entries),
    }
    report_path = _write_report(payload)
    return StopSettleDiagnosticResult(request, tuple(attempts), entries, outcome, close_error, str(report_path))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Bounded STOP/0x9C settle diagnostic; never drives or arms motors")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--can-channel", default=DEFAULT_CAN_CHANNEL)
    parser.add_argument("--sample-count", type=int, default=DEFAULT_SAMPLE_COUNT)
    parser.add_argument("--enable-can", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_stop_settle_diagnostic(StopSettleDiagnosticRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000]}, ensure_ascii=False))
        return 2
    print(json.dumps({
        "ok": result.final_outcome == "settled", "final_outcome": result.final_outcome,
        "attempts": [asdict(attempt) for attempt in result.attempts],
        "close_error": result.close_error,
        "report_path": result.report_path,
    }, ensure_ascii=False))
    return 0 if result.final_outcome == "settled" else 2


if __name__ == "__main__":
    raise SystemExit(main())
