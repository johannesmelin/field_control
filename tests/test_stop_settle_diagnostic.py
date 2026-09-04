from __future__ import annotations

import json
import os
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch

from field_control.stop_settle_diagnostic import (
    DIAGNOSTICS_DIRECTORY,
    StopSettleDiagnosticRequest,
    run_stop_settle_diagnostic,
)


class Entry:
    def __init__(self, detail: str | None = None) -> None:
        self.timestamp_s = 0.1; self.sequence = 1; self.phase = "runtime stop settle"
        self.direction = "event"; self.can_id = 0x141; self.dlc = None; self.data = None
        self.expected_reply_ids = (0x141,); self.pending_reply_ids = (0x141,); self.detail = detail


class Sink:
    def __init__(self, entries=()) -> None:
        self.entries = tuple(entries)
    def diagnostic_snapshot(self): return self.entries


class Boundary:
    def __init__(self, *, failure: Exception | None = None, close_failure: Exception | None = None, entries=()) -> None:
        self._sink = Sink(entries); self.failure = failure; self.close_failure = close_failure; self.calls = []; self.closed = 0
    def stop_and_settle_for_restart(self, reason):
        self.calls.append(reason)
        if self.failure is not None: raise self.failure
    def close(self):
        self.closed += 1
        if self.close_failure is not None: raise self.close_failure


class StopSettleDiagnosticTests(unittest.TestCase):
    def test_default_diagnostics_directory_is_at_source_tree_root(self):
        project_root = Path(__file__).resolve().parents[1]
        self.assertEqual(DIAGNOSTICS_DIRECTORY, project_root / "diagnostics")

    def request(self, **changes):
        values = dict(slcan_device="/dev/serial/by-id/usb-CANable_test", enable_can=True,
                      confirm_physical_stop_tested=True, confirm_wheels_raised=True, sample_count=3)
        values.update(changes)
        return StopSettleDiagnosticRequest(**values)

    def test_gates_fail_before_can_open(self):
        with patch("field_control.stop_settle_diagnostic.open_verified_boundary") as opened:
            for changes in ({"enable_can": False}, {"confirm_wheels_raised": False},
                            {"slcan_device": "/dev/ttyUSB0"}, {"can_channel": "can/0"},
                            {"sample_count": 0}):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    run_stop_settle_diagnostic(self.request(**changes))
            opened.assert_not_called()

    def test_stop_only_samples_are_bounded_and_saved(self):
        boundary = Boundary(entries=(Entry("0x9C sample 0 dps after 3.0 ms"),))
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.stop_settle_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.stop_settle_diagnostic.DIAGNOSTICS_DIRECTORY", Path(directory)), \
             patch("field_control.stop_settle_diagnostic.time.sleep"):
            result = run_stop_settle_diagnostic(self.request())
            with open(result.report_path, encoding="utf-8") as report:
                saved = json.load(report)
        self.assertEqual(boundary.calls, ["stop-settle-diagnostic"] * 3)
        self.assertEqual(boundary.closed, 1)
        self.assertEqual(result.final_outcome, "settled")
        self.assertEqual(len(result.attempts), 3)
        self.assertTrue(saved["ok"])
        self.assertEqual(saved["worker_diagnostics"][0]["can_id"], 0x141)

    def test_nonzero_status_deadline_is_distinguished(self):
        error = RuntimeError("STOP+0x9C-settle misslyckades: motorn nådde inte 0 dps inom runtime-stoppets settle-deadline")
        boundary = Boundary(failure=error, entries=(Entry("0x9C sample 25 dps after 300.0 ms"),))
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.stop_settle_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.stop_settle_diagnostic.DIAGNOSTICS_DIRECTORY", Path(directory)):
            result = run_stop_settle_diagnostic(self.request())
        self.assertEqual(result.final_outcome, "nonzero_dps_deadline")
        self.assertEqual(len(boundary.calls), 1)
        self.assertEqual(boundary.closed, 1)

    def test_reply_timeout_is_distinguished(self):
        boundary = Boundary(failure=RuntimeError("CAN-svarstimeout för motorgrupp"))
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.stop_settle_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.stop_settle_diagnostic.DIAGNOSTICS_DIRECTORY", Path(directory)):
            result = run_stop_settle_diagnostic(self.request())
        self.assertEqual(result.final_outcome, "reply_timeout")

    def test_close_failure_is_recorded_in_report(self):
        boundary = Boundary(close_failure=RuntimeError("bounded close settle failed"))
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.stop_settle_diagnostic.open_verified_boundary", return_value=boundary), \
             patch("field_control.stop_settle_diagnostic.DIAGNOSTICS_DIRECTORY", Path(directory)):
            result = run_stop_settle_diagnostic(self.request(sample_count=1))
            with open(result.report_path, encoding="utf-8") as report:
                saved = json.load(report)
        self.assertEqual(result.close_error, "RuntimeError: bounded close settle failed")
        self.assertEqual(saved["close_error"], result.close_error)
        self.assertEqual(result.attempts[-1].outcome, "close_failed")

    def test_report_refuses_diagnostic_directory_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); target = root / "target"; target.mkdir()
            linked = root / "linked"; linked.symlink_to(target, target_is_directory=True)
            with patch("field_control.stop_settle_diagnostic.DIAGNOSTICS_DIRECTORY", linked):
                with self.assertRaisesRegex(RuntimeError, "verklig katalog"):
                    __import__("field_control.stop_settle_diagnostic", fromlist=["_write_report"])._write_report({})

    def test_report_retries_same_timestamp_collision_with_private_mode(self):
        module = __import__("field_control.stop_settle_diagnostic", fromlist=["_write_report", "datetime"])
        with tempfile.TemporaryDirectory() as directory, \
             patch("field_control.stop_settle_diagnostic.DIAGNOSTICS_DIRECTORY", Path(directory)), \
             patch.object(module, "datetime") as mocked_datetime:
            mocked_datetime.now.return_value.strftime.return_value = "fixed"
            first = module._write_report({"attempt": 1})
            second = module._write_report({"attempt": 2})
            self.assertNotEqual(first, second)
            self.assertTrue(second.name.endswith("_1.json"))
            self.assertEqual(stat.S_IMODE(os.stat(first).st_mode), 0o600)


if __name__ == "__main__":
    unittest.main()
