import unittest
import time

from field_control.control import WheelCommand
from field_control.motor_boundary import (
    CAN_LOCK_PATH, LEFT_ID, OBSERVED_RMDX_SAME_ID_REPLY_PROFILE,
    RIGHT_ID, SocketCanV38Transport, VERIFIED_MAX_WHEEL_RPM,
    MotorOutputFault, PhysicalMotorBoundary, PhysicalOutputDisabled,
    get_motor_reply_profile, v38_speed_frame,
)


class FakeLease:
    def __init__(self):
        self.live = {"lease"}
        self.callback = None

    def valid(self, token): return token in self.live

    def run_if_valid(self, token, operation):
        if token not in self.live:
            return False
        operation()
        return True

    def set_revoke_callback(self, callback): self.callback = callback

    def revoke_any(self):
        active = bool(self.live)
        self.live.clear()
        if active:
            self.callback()
        return active

    def revoke(self):
        self.live.clear()
        self.callback()


class FakeTransport:
    def __init__(self):
        self.speeds, self.stops = [], []
        self.best_effort_stops = 0
        self.fail_speed = self.fail_stop = False
        self.closed = False

    def set_speed_dps(self, motor_id, speed_dps, deadline_s):
        if self.fail_speed: raise RuntimeError("sändfel")
        self.speeds.append((motor_id, speed_dps, deadline_s))

    def stop_pair_acknowledged(self, deadline_s):
        if self.fail_stop: raise RuntimeError("stoppsvar saknas")
        self.stops.append(deadline_s)

    def best_effort_stop_pair(self): self.best_effort_stops += 1
    def close(self): self.closed = True


class FakeMessage:
    def __init__(self, arbitration_id, data, timestamp=None, is_extended_id=False):
        self.arbitration_id, self.data = arbitration_id, bytes(data)
        self.timestamp = timestamp


class QueuedBus:
    """Bus seam that lets tests prove transmit/receive ordering."""
    def __init__(self, stale=(), auto_reply=True):
        self.pending = list(stale)
        self.sent = []
        self.recv_send_counts = []
        self.auto_reply = auto_reply
        self.closed = False

    def send(self, message, timeout=None):
        self.sent.append((message.arbitration_id, bytes(message.data)))
        if self.auto_reply:
            self.pending.append(FakeMessage(message.arbitration_id, message.data, timestamp=time.time()))

    def recv(self, timeout=None):
        self.recv_send_counts.append(len(self.sent))
        return self.pending.pop(0) if self.pending else None

    def shutdown(self): self.closed = True


class MotorBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.lease = FakeLease()
        self.transport = FakeTransport()
        self.boundary = PhysicalMotorBoundary(self.transport, self.lease, max_rpm=80)

    def test_starts_disarmed_and_never_sends_a2(self):
        with self.assertRaises(PhysicalOutputDisabled):
            self.boundary.command(WheelCommand(10, 10, "test"), "lease")
        self.assertEqual(self.transport.speeds, [])
        self.assertEqual(len(self.transport.stops), 1)

    def test_armed_command_clamps_and_applies_verified_motor_signs(self):
        self.boundary.arm("lease")
        self.boundary.command(WheelCommand(100, -100, "test"), "lease")
        self.assertEqual([(motor, speed) for motor, speed, _ in self.transport.speeds],
                         [(LEFT_ID, 480.0), (RIGHT_ID, 480.0)])

    def test_arm_requires_acknowledged_stop_before_output(self):
        self.boundary.arm("lease")
        self.assertEqual(len(self.transport.stops), 1)
        self.assertEqual(self.transport.speeds, [])
        self.assertTrue(self.boundary.armed)

    def test_failed_start_stop_latches_output_before_arm(self):
        self.transport.fail_stop = True
        with self.assertRaises(MotorOutputFault):
            self.boundary.arm("lease")
        self.assertFalse(self.boundary.armed)
        self.assertIsNotNone(self.boundary.fault_reason)
        self.assertEqual(self.transport.best_effort_stops, 1)

    def test_rejects_max_rpm_above_verified_hard_bound(self):
        with self.assertRaises(ValueError):
            PhysicalMotorBoundary(self.transport, self.lease, max_rpm=VERIFIED_MAX_WHEEL_RPM + 0.1)

    def test_expired_or_revoked_lease_stops_and_blocks_future_output(self):
        self.boundary.arm("lease")
        self.lease.revoke()
        self.assertFalse(self.boundary.armed)
        self.assertEqual(len(self.transport.stops), 2)
        with self.assertRaises(PhysicalOutputDisabled):
            self.boundary.command(WheelCommand(10, 10, "test"), "lease")
        self.assertEqual(self.transport.speeds, [])

    def test_can_failure_latches_fault_and_requests_acknowledged_stop(self):
        self.boundary.arm("lease")
        self.transport.fail_speed = True
        with self.assertRaises(MotorOutputFault):
            self.boundary.command(WheelCommand(10, 10, "test"), "lease")
        self.assertIsNotNone(self.boundary.fault_reason)
        self.assertEqual(len(self.transport.stops), 2)
        with self.assertRaises(MotorOutputFault):
            self.boundary.arm("lease")

    def test_failed_acknowledged_stop_latches_fault_and_uses_best_effort(self):
        self.boundary.arm("lease")
        self.transport.fail_stop = True
        self.boundary.stop_all("STOP")
        self.assertIsNotNone(self.boundary.fault_reason)
        self.assertEqual(self.transport.best_effort_stops, 1)

    def test_stop_revokes_lease_before_it_stops_motors(self):
        self.boundary.arm("lease")
        self.boundary.stop_all("STOP")
        self.assertFalse(self.lease.valid("lease"))
        self.assertEqual(len(self.transport.stops), 2)

    def test_v38_speed_frame_is_signed_001_degree_per_second(self):
        self.assertEqual(v38_speed_frame(LEFT_ID, -6.0),
                         bytes((0xA2, 0, 0, 0, 0xA8, 0xFD, 0xFF, 0xFF)))

    def test_transport_uses_existing_shared_can_lock_path(self):
        self.assertEqual(str(CAN_LOCK_PATH), "/run/lock/can0-motor-control.lock")

    def test_observed_same_id_profile_must_be_named_explicitly(self):
        self.assertIs(get_motor_reply_profile("observed-rmdx-same-id"), OBSERVED_RMDX_SAME_ID_REPLY_PROFILE)
        with self.assertRaises(ValueError):
            get_motor_reply_profile(None)

    def test_transport_discards_queued_stale_reply_before_new_request(self):
        stale = FakeMessage(LEFT_ID, bytes((0xA2,)) + bytes(7))
        bus = QueuedBus((stale,))
        transport = SocketCanV38Transport(bus, profile=OBSERVED_RMDX_SAME_ID_REPLY_PROFILE)
        transport.set_speed_dps(LEFT_ID, 6.0, deadline_s=__import__("time").monotonic() + 0.1)
        self.assertEqual(len(bus.sent), 1)
        # First receive was the pre-transmit stale-frame drain.
        self.assertEqual(bus.recv_send_counts[0], 0)

    def test_production_transport_rejects_reply_timestamped_before_send(self):
        bus = QueuedBus(auto_reply=False)
        fake_can = type("FakeCan", (), {"Message": FakeMessage})
        transport = SocketCanV38Transport(
            bus, profile=OBSERVED_RMDX_SAME_ID_REPLY_PROFILE, can_module=fake_can,
        )
        stale = FakeMessage(LEFT_ID, bytes((0xA2,)) + bytes(7), timestamp=time.time() - 1)
        self.assertFalse(transport._reply_is_not_stale(stale, time.time()))
        missing_timestamp = FakeMessage(LEFT_ID, bytes((0xA2,)) + bytes(7))
        self.assertFalse(transport._reply_is_not_stale(missing_timestamp, time.time()))

    def test_stop_pair_transmits_both_before_waiting_for_acknowledgements(self):
        bus = QueuedBus()
        transport = SocketCanV38Transport(bus, profile=OBSERVED_RMDX_SAME_ID_REPLY_PROFILE)
        transport.stop_pair_acknowledged(deadline_s=time.monotonic() + 0.1)
        self.assertEqual([motor_id for motor_id, _ in bus.sent], [LEFT_ID, RIGHT_ID])
        # First recv drains stale input; every later receive occurs only after
        # both 0x81 frames have been submitted.
        self.assertEqual(bus.recv_send_counts[0], 0)
        self.assertTrue(all(count == 2 for count in bus.recv_send_counts[1:]))


if __name__ == "__main__":
    unittest.main()
