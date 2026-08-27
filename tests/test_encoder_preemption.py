import time
import threading
import unittest

from field_control.odometry import DriveGeometry
from field_control.sources import EncoderReadPreempted, OdometrySource


class EncoderPreemptionTests(unittest.TestCase):
    def test_stop_preemption_retries_once_at_next_sampling_boundary_with_fresh_read(self):
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
            self.assertTrue(source.wait_until_ready(.35))
            snapshot = source.snapshot()
            self.assertTrue(snapshot.connected)
            self.assertIsNone(snapshot.error)
            self.assertIsNotNone(snapshot.value)
            self.assertEqual(backend.calls, 2)
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

    def test_preemption_invalidates_prior_sample_until_a_new_read_succeeds(self):
        class Backend:
            def __init__(self):
                self.calls = 0
                self.second_read = threading.Event()
                self.preempt = threading.Event()
                self.third_read = threading.Event()
                self.release_fresh = threading.Event()

            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    return 1.0, 1.0
                if self.calls == 2:
                    self.second_read.set()
                    self.preempt.wait(.200)
                    raise EncoderReadPreempted("AUTO STOP")
                self.third_read.set()
                self.release_fresh.wait(.200)
                return 2.0, 2.0

            def close(self):
                self.release_fresh.set()

        backend = Backend(); source = OdometrySource(backend, DriveGeometry())
        source.start()
        try:
            self.assertTrue(source.wait_until_ready(.150))
            self.assertTrue(backend.second_read.wait(.200))
            backend.preempt.set()
            deadline = time.monotonic() + .100
            while source.snapshot().connected and time.monotonic() < deadline:
                time.sleep(.002)
            invalid = source.snapshot()
            self.assertFalse(invalid.connected)
            self.assertIsNone(invalid.value)
            self.assertIsNone(invalid.updated_at_s)

            ready: list[bool] = []
            waiter = threading.Thread(target=lambda: ready.append(source.wait_until_ready(.300)))
            waiter.start()
            self.assertTrue(backend.third_read.wait(.200))
            self.assertTrue(waiter.is_alive())
            backend.release_fresh.set()
            waiter.join(.200)
            self.assertEqual(ready, [True])
            self.assertEqual(backend.calls, 3)
            self.assertTrue(source.snapshot().connected)
        finally:
            source.stop()

    def test_real_error_after_preemption_is_terminal_not_masked_by_recovery(self):
        class Backend:
            def __init__(self): self.calls = 0
            def angles(self):
                self.calls += 1
                if self.calls == 1:
                    return 0.0, 0.0
                if self.calls == 2:
                    raise EncoderReadPreempted("AUTO STOP")
                raise RuntimeError("CAN timeout after STOP")
            def close(self): pass

        backend = Backend(); source = OdometrySource(backend, DriveGeometry())
        source.start()
        try:
            self.assertTrue(source.wait_until_ready(.150))
            deadline = time.monotonic() + .350
            while source.snapshot().error is None and time.monotonic() < deadline:
                time.sleep(.002)
            self.assertIn("CAN timeout after STOP", source.snapshot().error or "")
            self.assertEqual(backend.calls, 3)
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
