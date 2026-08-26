import threading
import unittest

from field_control.control import WheelCommand
from field_control.config import PhysicalCanConfig
from field_control.lease import ControlLease
from field_control.motor_boundary import MotorOutputFault, PhysicalOutputDisabled
from field_control.sources import EncoderReadPreempted
from field_control.verified_motor_boundary import _VerifiedPhysicalMotorBoundary, SharedCanEncoderBackend, open_verified_boundary


class Sink:
    def __init__(self):
        self.commands = []; self.stops = []; self.settles = 0; self.closed = False; self.callback = None
        self.events = []; self.fail_settle = None
    def set_fault_callback(self, callback): self.callback = callback
    def command(self, left, right, reason): self.commands.append((left, right, reason))
    def stop_all(self, reason): self.stops.append(reason)
    def stop_and_settle_for_restart(self):
        self.settles += 1; self.events.append("settle")
        if self.fail_settle is not None: raise self.fail_settle
    def stop_and_settle_and_close(self):
        self.settles += 1; self.events.append("shutdown-settle")
        try:
            if self.fail_settle is not None: raise self.fail_settle
        finally:
            self.close()
    def close(self): self.closed = True; self.events.append("close")
    def read_multi_turn_angles(self): return (12.5, -6.25)


class VerifiedAdapterTests(unittest.TestCase):
    def test_shared_encoder_adapter_reads_fresh_pair_and_never_closes_sink(self):
        sink = Sink(); adapter = SharedCanEncoderBackend(sink)
        self.assertEqual(adapter.angles(), (12.5, -6.25))
        adapter.close()
        self.assertFalse(sink.closed)

    def test_boundary_exposes_non_owning_shared_encoder_adapter(self):
        sink = Sink(); boundary = _VerifiedPhysicalMotorBoundary(sink, ControlLease(1.0), max_rpm=20)
        adapter = boundary.encoder_backend()
        self.assertEqual(adapter.angles(), (12.5, -6.25))
        adapter.close(); self.assertFalse(sink.closed)

    def test_shared_encoder_maps_only_remote_typed_stop_preemption(self):
        from remote_control.physical import AngleReadPreempted

        class PreemptedSink(Sink):
            def read_multi_turn_angles(self): raise AngleReadPreempted("STOP won")

        with self.assertRaises(EncoderReadPreempted):
            SharedCanEncoderBackend(PreemptedSink()).angles()

    def test_shared_encoder_preserves_real_can_errors(self):
        from remote_control.physical import PhysicalCanError

        class FailingSink(Sink):
            def read_multi_turn_angles(self): raise PhysicalCanError("reply timeout")

        with self.assertRaisesRegex(PhysicalCanError, "reply timeout"):
            SharedCanEncoderBackend(FailingSink()).angles()

    def test_shared_encoder_shutdown_is_non_owning_but_blocks_future_reads(self):
        sink = Sink(); adapter = SharedCanEncoderBackend(sink)
        adapter.begin_shutdown()

        with self.assertRaisesRegex(RuntimeError, "stängd"):
            adapter.angles()
        self.assertFalse(sink.closed)

    def test_shared_encoder_shutdown_linearizes_with_inflight_sink_admission(self):
        """A shutdown return is a barrier for every subsequent 0x92 admission."""
        class BlockingSink(Sink):
            def __init__(self):
                super().__init__()
                self.read_entered = threading.Event()
                self.release_read = threading.Event()
                self.read_calls = 0
            def read_multi_turn_angles(self):
                self.read_calls += 1
                self.read_entered.set()
                if not self.release_read.wait(.250):
                    raise RuntimeError("test read was not released")
                return super().read_multi_turn_angles()

        sink = BlockingSink(); adapter = SharedCanEncoderBackend(sink)
        read_result: list[tuple[float, float]] = []
        reader = threading.Thread(target=lambda: read_result.append(adapter.angles()))
        reader.start()
        self.assertTrue(sink.read_entered.wait(.100))

        shutdown_done = threading.Event()
        shutdown = threading.Thread(target=lambda: (adapter.begin_shutdown(), shutdown_done.set()))
        shutdown.start()
        # The barrier waits for the one bounded, already admitted read; it
        # must not claim shutdown concurrently with that sink call.
        self.assertFalse(shutdown_done.wait(.020))
        sink.release_read.set()
        reader.join(.150); shutdown.join(.150)
        self.assertFalse(reader.is_alive())
        self.assertFalse(shutdown.is_alive())
        self.assertEqual(read_result, [(12.5, -6.25)])
        self.assertTrue(shutdown_done.is_set())
        self.assertEqual(sink.read_calls, 1)

        with self.assertRaisesRegex(RuntimeError, "stängd"):
            adapter.angles()
        self.assertEqual(sink.read_calls, 1)
        self.assertFalse(sink.closed)
    def test_arm_uses_verified_settle_and_command_only_queues(self):
        sink = Sink(); lease = ControlLease(1.0)
        boundary = _VerifiedPhysicalMotorBoundary(sink, lease, max_rpm=20)
        token = lease.acquire(); boundary.arm(token)
        boundary.command(WheelCommand(30, -30, "test"), token)
        self.assertEqual(sink.settles, 1)
        self.assertEqual(sink.commands, [(20.0, -20.0, "test")])
        boundary.stop_all("STOP")
        self.assertTrue(sink.stops)

    def test_worker_fault_revokes_and_blocks_later_queue_admission(self):
        sink = Sink(); lease = ControlLease(1.0)
        boundary = _VerifiedPhysicalMotorBoundary(sink, lease, max_rpm=20)
        token = lease.acquire(); boundary.arm(token)
        sink.callback("mock worker fault")
        self.assertFalse(boundary.armed)
        with self.assertRaises(PhysicalOutputDisabled):
            boundary.command(WheelCommand(1, 1, "after fault"), token)

    def test_default_import_path_has_no_reduced_transport_production_reference(self):
        import inspect
        import field_control.app as app
        self.assertNotIn("SocketCanV38Transport", inspect.getsource(app))
        self.assertNotIn("PhysicalMotorBoundary", inspect.getsource(app))

    def test_physical_deployment_requires_all_raised_wheel_confirmations(self):
        with self.assertRaises(ValueError):
            PhysicalCanConfig(True, "can0", "observed-rmdx-same-id", "/dev/serial/by-id/x").validate()

    def test_production_open_signature_has_no_sink_factory(self):
        import inspect
        self.assertNotIn("sink_factory", inspect.signature(open_verified_boundary).parameters)

    def test_close_settles_before_closing_and_is_idempotent(self):
        sink = Sink(); lease = ControlLease(1.0)
        boundary = _VerifiedPhysicalMotorBoundary(sink, lease, max_rpm=20)
        token = lease.acquire(); boundary.arm(token)
        sink.events.clear(); settle_count = sink.settles

        boundary.close()
        boundary.close()

        self.assertEqual(sink.events, ["shutdown-settle", "close"])
        self.assertEqual(sink.settles, settle_count + 1)
        self.assertFalse(lease.valid(token))

    def test_close_settle_failure_latches_fault_and_still_closes_once(self):
        sink = Sink(); sink.fail_settle = RuntimeError("settle timeout")
        boundary = _VerifiedPhysicalMotorBoundary(sink, ControlLease(1.0), max_rpm=20)

        with self.assertRaises(MotorOutputFault):
            boundary.close()
        boundary.close()

        self.assertEqual(sink.events, ["shutdown-settle", "close"])
        self.assertTrue(sink.closed)
        self.assertIn("shutdown-STOP+0x9C-settle", boundary.fault_reason or "")


if __name__ == "__main__":
    unittest.main()
