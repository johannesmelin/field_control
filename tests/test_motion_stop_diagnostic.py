from __future__ import annotations

import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from field_control.motion_stop_diagnostic import (
    AUTO_LIKE_LEASE_TIMEOUT_S,
    AUTO_LIKE_DURATION_S,
    MotionStopDiagnosticRequest,
    _lease_timeout_s,
    _run_motion_window,
    _expire_lease,
    run_motion_stop_diagnostic,
)
from field_control.lease import ControlLease


class Entry:
    def __init__(self, detail: str | None) -> None:
        self.timestamp_s = 1.0; self.sequence = 1; self.phase = "runtime stop settle"
        self.direction = "rx"; self.can_id = 0x141; self.dlc = 8
        self.data = bytes((0x9C, 0, 0, 0, 0, 0, 0, 0))
        self.expected_reply_ids = (0x141,); self.pending_reply_ids = (); self.detail = detail


class Sink:
    def __init__(self, entries=()) -> None: self.entries = tuple(entries)
    def diagnostic_snapshot(self): return self.entries


class Boundary:
    def __init__(self, *, stop_failure: Exception | None = None, entries=()) -> None:
        self._sink = Sink(entries); self.stop_failure = stop_failure
        self.calls: list[tuple[str, object]] = []

    def arm(self, token): self.calls.append(("arm", token))
    def command(self, command, token): self.calls.append(("command", command, token))
    def stop_and_settle_for_restart(self, reason):
        self.calls.append(("settle", reason))
        if self.stop_failure is not None: raise self.stop_failure
    def close(self): self.calls.append(("close", None))


class MotionStopDiagnosticTests(unittest.TestCase):
    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_can=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True)
        values.update(changes)
        return MotionStopDiagnosticRequest(**values)

    def test_physical_gates_and_bounds_fail_before_open(self):
        with patch("field_control.motion_stop_diagnostic.open_verified_boundary") as opened:
            for changes in ({"enable_can": False}, {"confirm_wheels_raised": False},
                            {"motor_rpm": 40.1}, {"duration_s": 0.71},
                            {"recurring_a2": 1},
                            {"auto_like_window": True},
                            {"auto_like_window": True, "duration_s": 2.99},
                            {"slcan_device": "/dev/ttyUSB0"}):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_motion_stop_diagnostic(self.request(**changes))
            opened.assert_not_called()

    def test_exactly_one_forward_a2_pair_then_verified_stop_and_report(self):
        boundary = Boundary(entries=(Entry("0x9C sample 0 dps after 20.0 ms"),
                                     Entry("stale/unrelated reply ignored")))
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.motion_stop_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.motion_stop_diagnostic._write_report") as write, \
             patch("field_control.motion_stop_diagnostic.time.sleep") as slept:
            write.return_value = Path(directory) / "motion_stop.json"
            result = run_motion_stop_diagnostic(self.request())
        command_calls = [item for item in boundary.calls if item[0] == "command"]
        self.assertEqual(len(command_calls), 1)
        command = command_calls[0][1]
        self.assertEqual((command.left_rpm, command.right_rpm, command.source),
                         (10.0, 10.0, "motion-stop-diagnostic-forward"))
        self.assertEqual([item[0] for item in boundary.calls], ["arm", "command", "settle", "close"])
        slept.assert_called_once_with(0.7)
        self.assertEqual(result.final_outcome, "settled")
        payload = write.call_args.args[0]
        self.assertEqual(write.call_args.kwargs["report_prefix"], "motion_stop")
        self.assertEqual(payload["diagnostic_summary"]["stale_reply_or_frame_count"], 1)
        self.assertEqual(payload["diagnostic_summary"]["status_samples"], [{
            "detail": "0x9C sample 0 dps after 20.0 ms", "dps": 0.0, "elapsed_ms": 20.0,
        }])

    def test_exact_maximum_rpm_admits_exactly_one_paired_forward_command(self):
        boundary = Boundary()
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.motion_stop_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.motion_stop_diagnostic._write_report", return_value=Path(directory) / "motion_stop.json"), \
             patch("field_control.motion_stop_diagnostic.time.sleep"):
            result = run_motion_stop_diagnostic(self.request(motor_rpm=40.0))
        command_calls = [item for item in boundary.calls if item[0] == "command"]
        self.assertEqual(len(command_calls), 1)
        command = command_calls[0][1]
        self.assertEqual((command.left_rpm, command.right_rpm, command.source),
                         (40.0, 40.0, "motion-stop-diagnostic-forward"))
        self.assertEqual(result.final_outcome, "settled")

    def test_opt_in_recurring_a2_uses_10hz_slots_and_never_sends_at_deadline(self):
        now = [0.0]
        calls = []

        def send(command, token):
            calls.append((now[0], command, token))

        def sleep(seconds):
            now[0] += seconds

        request = self.request(motor_rpm=40.0, duration_s=0.7, recurring_a2=True)
        _run_motion_window(command=send, lease_token="token", request=request,
                           started_s=0.0, clock=lambda: now[0], sleep=sleep)
        self.assertEqual([round(call[0], 3) for call in calls], [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        self.assertTrue(all(call[1].left_rpm == call[1].right_rpm == 40.0 for call in calls))
        self.assertTrue(all(call[1].source == "motion-stop-diagnostic-forward" for call in calls))
        self.assertTrue(all(call[0] < 0.7 for call in calls))

    def test_recurring_a2_does_not_burst_after_delayed_wakeup(self):
        now = [0.0]
        calls = []

        def send(command, token):
            calls.append(now[0])

        def sleep(_seconds):
            now[0] = 0.71

        _run_motion_window(command=send, lease_token="token",
                           request=self.request(recurring_a2=True), started_s=0.0,
                           clock=lambda: now[0], sleep=sleep)
        self.assertEqual(calls, [0.0])

    def test_recurring_a2_delayed_wakeup_never_admits_two_frames_within_100ms(self):
        now = [0.0]
        calls = []
        first_sleep = [True]

        def send(command, token):
            calls.append(now[0])

        def sleep(seconds):
            if first_sleep[0]:
                first_sleep[0] = False
                now[0] = 0.350  # Misses three ideal scheduler slots.
            else:
                now[0] += seconds

        _run_motion_window(command=send, lease_token="token",
                           request=self.request(recurring_a2=True), started_s=0.0,
                           clock=lambda: now[0], sleep=sleep)
        self.assertEqual([round(value, 3) for value in calls], [0.0, 0.35, 0.45, 0.55, 0.65])
        self.assertTrue(all((later - earlier) >= 0.1 - 1e-12
                            for earlier, later in zip(calls, calls[1:])))
        self.assertTrue(all(value < 0.7 for value in calls))

    def test_auto_like_window_is_three_seconds_and_forces_10hz_recurring_a2(self):
        now = [0.0]
        calls = []

        def send(command, token):
            calls.append(now[0])

        def sleep(seconds):
            now[0] += seconds

        request = self.request(motor_rpm=40.0, duration_s=AUTO_LIKE_DURATION_S,
                               auto_like_window=True)
        _run_motion_window(command=send, lease_token="token", request=request,
                           started_s=0.0, clock=lambda: now[0], sleep=sleep)
        self.assertEqual(len(calls), 30)
        self.assertEqual([round(value, 1) for value in calls[:3]], [0.0, 0.1, 0.2])
        self.assertEqual(round(calls[-1], 1), 2.9)
        self.assertTrue(all(value < AUTO_LIKE_DURATION_S for value in calls))
        self.assertEqual(_lease_timeout_s(request), AUTO_LIKE_LEASE_TIMEOUT_S)
        self.assertGreater(AUTO_LIKE_LEASE_TIMEOUT_S, AUTO_LIKE_DURATION_S)

    def test_auto_like_watchdog_starts_at_actual_first_a2_after_150ms_admission(self):
        now = [0.0]
        calls, watchdog_epochs = [], []

        def send(command, token):
            calls.append(now[0])
            if len(calls) == 1:
                now[0] += 0.150  # First bounded A2 admission is delayed.

        def sleep(seconds):
            now[0] += seconds

        request = self.request(duration_s=AUTO_LIKE_DURATION_S, auto_like_window=True)
        _run_motion_window(command=send, lease_token="token", request=request, started_s=0.0,
                           on_first_command_sent=watchdog_epochs.append,
                           clock=lambda: now[0], sleep=sleep)
        self.assertEqual(watchdog_epochs, [0.150])
        self.assertEqual(len(calls), 30)
        self.assertEqual(round(calls[-1], 2), 3.05)
        motion_deadline_s = watchdog_epochs[0] + AUTO_LIKE_DURATION_S
        watchdog_deadline_s = watchdog_epochs[0] + _lease_timeout_s(request)
        self.assertGreater(watchdog_deadline_s, motion_deadline_s)
        self.assertTrue(all(value < motion_deadline_s for value in calls))

    def test_default_window_keeps_original_short_watchdog(self):
        self.assertEqual(_lease_timeout_s(self.request()), 0.8)

    def test_provisional_watchdog_revokes_when_first_a2_admission_pauses_before_rebase(self):
        class PausingBoundary(Boundary):
            def command(self, command, token):
                super().command(command, token)
                # Simulates a caller paused after the A2 was admitted but
                # before _run_motion_window can rebase the lease epoch.
                import time
                time.sleep(0.04)

        leases, revocations = [], []

        class RecordingLease(ControlLease):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.set_revoke_callback(lambda: revocations.append("safe-zero-request"))
                leases.append(self)

        boundary = PausingBoundary()
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.motion_stop_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.motion_stop_diagnostic.ControlLease", RecordingLease), \
             patch("field_control.motion_stop_diagnostic._lease_timeout_s", return_value=0.02), \
             patch("field_control.motion_stop_diagnostic._write_report", return_value=Path(directory) / "motion_stop.json"):
            result = run_motion_stop_diagnostic(self.request())
        arm_token = next(call[1] for call in boundary.calls if call[0] == "arm")
        self.assertTrue(result.lease_watchdog_triggered)
        self.assertFalse(leases[0].valid(arm_token))
        self.assertEqual(revocations, ["safe-zero-request"])
        self.assertEqual([call[0] for call in boundary.calls], ["arm", "command", "settle", "close"])

    def test_independent_watchdog_revokes_expired_lease(self):
        now = [0.0]
        lease = ControlLease(.8, clock=lambda: now[0])
        revoked = []
        lease.set_revoke_callback(lambda: revoked.append("safe-stop-request"))
        token = lease.acquire()
        now[0] = .81  # Simulates the main diagnostic thread being delayed.
        fired = threading.Event()
        _expire_lease(lease, fired, threading.Event())
        self.assertTrue(fired.is_set())
        self.assertEqual(revoked, ["safe-stop-request"])
        self.assertFalse(lease.valid(token))

    def test_watchdog_started_before_expiry_waits_until_clock_expires(self):
        now = [0.0]
        lease = ControlLease(.8, clock=lambda: now[0])
        revoked = []
        lease.set_revoke_callback(lambda: revoked.append("safe-stop-request"))
        lease.acquire()
        now[0] = .79
        fired, cancelled = threading.Event(), threading.Event()
        worker = threading.Thread(target=_expire_lease, args=(lease, fired, cancelled), daemon=True)
        worker.start()
        self.assertFalse(fired.wait(.02))
        now[0] = .81
        self.assertTrue(fired.wait(.25))
        worker.join(.25)
        self.assertFalse(worker.is_alive())
        self.assertEqual(revoked, ["safe-stop-request"])

    def test_delayed_main_is_reported_as_watchdog_stop_not_success(self):
        boundary = Boundary(entries=(Entry("0x9C sample 0 dps after 20.0 ms"),))
        timers = []

        class ExpiringLease:
            def __init__(self, *_args, **_kwargs): pass
            def acquire(self): return "token"
            def refresh(self, _token): return None
            def watchdog_tick(self): return True

        class TriggerTimer:
            def __init__(self, _delay, callback, args=()):
                self.callback, self.args, self.daemon, self.cancelled = callback, args, False, False
                timers.append(self)
            def start(self):
                # The worker is scheduled and has acknowledged start; this
                # fake delays only its later polling callback until sleep.
                self.args[3].set()
            def cancel(self): self.cancelled = True
            def fire(self):
                if not self.cancelled: self.callback(*self.args)

        def delayed_sleep(_seconds):
            # Models a main thread that did not regain control before the
            # independent lease timer ran.  No physical code is involved.
            timers[-1].fire()

        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.motion_stop_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.motion_stop_diagnostic.ControlLease", ExpiringLease), \
             patch("field_control.motion_stop_diagnostic.threading.Timer", TriggerTimer), \
             patch("field_control.motion_stop_diagnostic.time.sleep", delayed_sleep), \
             patch("field_control.motion_stop_diagnostic._write_report", return_value=Path(directory) / "motion_stop.json") as write:
            result = run_motion_stop_diagnostic(self.request())
        self.assertEqual(result.final_outcome, "lease_watchdog_stop")
        self.assertTrue(result.lease_watchdog_triggered)
        self.assertFalse(write.call_args.args[0]["ok"])
        self.assertEqual([item[0] for item in boundary.calls], ["arm", "command", "settle", "close"])

    def test_missing_watchdog_thread_acknowledgement_admits_no_a2(self):
        boundary = Boundary()

        class NeverStartedTimer:
            def __init__(self, _delay, callback, args=()):
                self.callback, self.args, self.daemon = callback, args, False
            def start(self): return None
            def cancel(self): return None

        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.motion_stop_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.motion_stop_diagnostic.threading.Timer", NeverStartedTimer), \
             patch("field_control.motion_stop_diagnostic._write_report", return_value=Path(directory) / "motion_stop.json"):
            result = run_motion_stop_diagnostic(self.request())
        self.assertIn("watchdog-tråd startade inte", result.error or "")
        self.assertEqual([item[0] for item in boundary.calls], ["arm", "settle", "close"])

    def test_failed_settle_is_reported_and_close_still_runs(self):
        boundary = Boundary(stop_failure=RuntimeError("motorn nådde inte 0 dps inom runtime-stoppets settle-deadline"))
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.motion_stop_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.motion_stop_diagnostic._write_report", return_value=Path(directory) / "motion_stop.json"), \
             patch("field_control.motion_stop_diagnostic.time.sleep"):
            result = run_motion_stop_diagnostic(self.request())
        self.assertEqual(result.final_outcome, "nonzero_dps_deadline")
        self.assertEqual([item[0] for item in boundary.calls], ["arm", "command", "settle", "close"])

    def test_report_writer_retains_existing_stop_settle_prefix(self):
        # A focused regression for the shared safe report writer's default.
        from field_control.stop_settle_diagnostic import _write_report
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.stop_settle_diagnostic.DIAGNOSTICS_DIRECTORY", Path(directory)):
            result = _write_report({"ok": True})
            self.assertTrue(result.name.startswith("stop_settle_"))
            self.assertEqual(json.loads(result.read_text(encoding="utf-8"))["ok"], True)


if __name__ == "__main__":
    unittest.main()
