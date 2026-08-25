import unittest

from field_control.config import RuntimeConfig
from field_control.heading import RowHeadingReference
from field_control.observation import HeadingProcessor, ImuReading, build_observation
from field_control.sources import LatestValue
from field_control.sources import heading_from_imu_quaternion
from field_control.sources import OdometrySource
from field_control.odometry import DriveGeometry
from field_control.turn import new_row_turn_plan
from field_control.lease import ControlLease


class RuntimeIntegrationTests(unittest.TestCase):
    def test_new_row_turn_plan_uses_track_and_row_spacing(self):
        plan = new_row_turn_plan(DriveGeometry(wheel_track_m=1.0), 2.0, 12.0, "right")
        self.assertAlmostEqual(plan.left_distance_m, 1.5 * 3.141592653589793)
        self.assertAlmostEqual(plan.right_distance_m, .5 * 3.141592653589793)
        self.assertEqual(plan.speed_rpm, 12.0)

    def test_odometry_source_publishes_latest_forward_distance(self):
        class Angles:
            value = (0.0, 0.0)
            def angles(self): return self.value
            def close(self): pass

        backend = Angles(); source = OdometrySource(backend, DriveGeometry())
        self.assertEqual(source._read_distance(), 0.0)
        backend.value = (360.0, -360.0)
        self.assertAlmostEqual(source._read_distance(), DriveGeometry().left_wheel_circumference_m / 8.0)
        source.stop()

    def test_control_lease_expires_using_injected_monotonic_clock(self):
        now = [10.0]
        lease = ControlLease(.5, lambda: now[0])
        token = lease.acquire()
        self.assertTrue(lease.valid(token))
        now[0] = 10.5
        self.assertFalse(lease.valid(token))
        self.assertFalse(lease.run_if_valid(token, lambda: self.fail("expired lease ran operation")))

    def test_control_lease_refresh_and_revoke_callback_are_fail_closed(self):
        now = [1.0]; revoked = []
        lease = ControlLease(1.0, lambda: now[0]); lease.set_revoke_callback(lambda: revoked.append(True))
        token = lease.acquire(); now[0] = 1.5; lease.refresh(token)
        self.assertTrue(lease.valid(token)); self.assertTrue(lease.revoke_any())
        self.assertEqual(revoked, [True]); self.assertFalse(lease.valid(token))

    def test_latest_value_replaces_old_data_and_keeps_timestamp(self):
        latest = LatestValue()
        latest.publish("old", 10.0)
        latest.publish("new", 12.0)
        snapshot = latest.snapshot()
        self.assertEqual(snapshot.value, "new")
        self.assertEqual(snapshot.updated_at_s, 12.0)
        self.assertEqual(snapshot.age_s(13.5), 1.5)

    def test_latest_value_rejects_non_monotonic_timestamps(self):
        latest = LatestValue()
        latest.publish("value", 10.0)
        with self.assertRaises(ValueError):
            latest.publish("older", 9.0)

    def test_observation_marks_camera_stale_without_using_old_vision(self):
        camera = LatestValue(); camera.publish(object(), 1.0)
        imu = LatestValue(); imu.publish(ImuReading(10.0, 2.0), 2.0)
        odometry = LatestValue(); odometry.publish(3.0, 2.0)
        heading = HeadingProcessor(.5, RowHeadingReference(2.0, 1.0))
        heading.update(ImuReading(10.0, 2.0), visual_following=False, distance_m=3.0)
        observation = build_observation(
            3.0, camera.snapshot(), imu.snapshot(), odometry.snapshot(),
            object(), heading, .5, 2.0, 2.0,
        )
        self.assertFalse(observation.camera_fresh)
        self.assertIsNone(observation.vision)
        self.assertEqual(observation.fault, "CAMERA_TIMEOUT")

    def test_runtime_configuration_rejects_nonpositive_sensor_timeout(self):
        with self.assertRaises(ValueError):
            RuntimeConfig(camera_timeout_s=0).validate()

    def test_runtime_defaults_to_320_by_240_for_processing_and_streaming(self):
        config = RuntimeConfig()
        self.assertEqual((config.processing_width, config.processing_height), (320, 240))
        self.assertEqual((config.stream_width, config.stream_height), (320, 240))

    def test_tilt_compensated_quaternion_heading_is_circular(self):
        self.assertAlmostEqual(heading_from_imu_quaternion((0, 0.70710678, 0, 0.70710678)), 0.0, places=4)
        calibrated = ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(heading_from_imu_quaternion((0, 0, 0, 1), calibrated), 0.0, places=4)
        with self.assertRaises(ValueError):
            heading_from_imu_quaternion((0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()