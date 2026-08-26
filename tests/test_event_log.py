import threading
import json
import unittest

from field_control.event_log import EventLog


class EventLogTests(unittest.TestCase):
    def test_drop_oldest_filter_and_monotonic_timestamps(self):
        log = EventLog(capacity=2, level="INFO", clock=lambda: 3.0)
        self.assertFalse(log.record("debug", level="DEBUG"))
        log.record("first", timestamp_s=2.0)
        log.record("second", timestamp_s=1.0)
        log.record("third", timestamp_s=4.0)
        events = log.recent()
        self.assertEqual([event["kind"] for event in events], ["second", "third"])
        self.assertEqual([event["timestamp_s"] for event in events], [2.0, 4.0])

    def test_record_drops_new_event_instead_of_blocking_on_consumer_lock(self):
        log = EventLog()
        locked = threading.Event(); release = threading.Event()
        def consumer():
            with log._lock:
                locked.set(); release.wait(.5)
        thread = threading.Thread(target=consumer); thread.start()
        self.assertTrue(locked.wait(.5))
        self.assertFalse(log.record("must_not_wait"))
        release.set(); thread.join(.5)
        self.assertEqual(log.recent(), [])

    def test_event_data_is_copied_json_safe_and_rejects_unsafe_values(self):
        log = EventLog()
        data = {"speed": 1.5, "armed": False, "reason": None}
        log.record("safe", data=data)
        data["speed"] = 99.0
        self.assertEqual(log.recent()[0]["data"]["speed"], 1.5)
        self.assertIsInstance(json.dumps({"recent_events": log.recent()}), str)
        for invalid in ({1: "key"}, {"nan": float("nan")}, {"nested": []}, ["not", "mapping"]):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError): log.record("invalid", data=invalid)  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
