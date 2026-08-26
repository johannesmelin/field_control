import time
import threading
import unittest

from field_control.odometry import DriveGeometry
from field_control.sources import EncoderReadPreempted, OdometrySource


class EncoderPreemptionTests(unittest.TestCase):
    def test_stop_preemption_is_terminal_and_never_retries_encoder_read(self):
        class Backend:
            def __init__(self): self.calls = 0; self.closed = False
            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    raise EncoderReadPreempted("safe STOP")
                return 0.0, 0.0
            def close(self): self.closed = True

        backend = Backend(); source = OdometrySource(backend, DriveGeometry())
        source.start()
        try:
            self.assertFalse(source.wait_until_ready(.35))
            snapshot = source.snapshot()
            self.assertFalse(snapshot.connected)
            self.assertIn("safe STOP", snapshot.error or "")
            self.assertEqual(backend.calls, 1)
        finally:
            source.stop()
        self.assertTrue(backend.closed)

    def test_real_encoder_error_fails_source_without_retry_loop(self):
        class Backend:
            def __init__(self): self.calls = 0
            def angles(self): self.calls += 1; raise RuntimeError("CAN timeout")
            def close(self): pass

        backend = Backend(); source = OdometrySource(backend, DriveGeometry())
        source.start()
        try:
            deadline = time.monotonic() + .2
            while source.snapshot().error is None and time.monotonic() < deadline:
                time.sleep(.002)
            snapshot = source.snapshot()
            self.assertFalse(snapshot.connected)
            self.assertIn("CAN timeout", snapshot.error or "")
            self.assertEqual(backend.calls, 1)
        finally:
            source.stop()

    def test_shutdown_barrier_prevents_any_later_encoder_admission(self):
        """A close signal cannot cause a second 0x92-equivalent call."""
        class Backend:
            def __init__(self):
                self.calls = 0
                self.first_preemption = threading.Event()
                self.shutdown = threading.Event()
            def angles(self):
                self.calls += 1
                self.first_preemption.set()
                raise EncoderReadPreempted("restart STOP")
            def begin_shutdown(self): self.shutdown.set()
            def close(self): self.shutdown.set()

        backend = Backend(); source = OdometrySource(backend, DriveGeometry())
        source.start()
        try:
            self.assertTrue(backend.first_preemption.wait(.2))
            source.begin_shutdown()
            self.assertTrue(backend.shutdown.is_set())
            # Cross the normal 100 ms sample boundary: no second
            # 0x92-equivalent backend call may be admitted after shutdown.
            time.sleep(.15)
            self.assertEqual(backend.calls, 1)
        finally:
            source.stop()


if __name__ == "__main__":
    unittest.main()
