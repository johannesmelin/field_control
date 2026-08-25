import unittest

import cv2
import numpy as np

from field_control.config import HsvFilter, RuntimeConfig, VisionConfig, Zone
from field_control.motor_boundary import DisabledMotorBoundary
from field_control.observation import ImuReading
from field_control.runtime import FieldControlRuntime
from field_control.sources import LatestValue
from field_control.state_machine import State
from field_control.state_machine import SafetyConfig


class FakeSource:
    def __init__(self, value, timestamp=1.0):
        self.latest = LatestValue()
        self.latest.publish(value, timestamp)

    def start(self): pass
    def stop(self): pass
    def snapshot(self): return self.latest.snapshot()


class RuntimeE2ETests(unittest.TestCase):
    def test_simulated_runtime_reaches_row_follow_without_motor_output(self):
        hsv = np.zeros((20, 20, 3), dtype=np.uint8)
        hsv[4:8, 8:12] = (0, 255, 255)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        red = HsvFilter((0, 200, 200), (5, 255, 255), 4)
        none = HsvFilter((100, 200, 200), (110, 255, 255), 4)
        config = RuntimeConfig(
            vision=VisionConfig("buds_only", red, none, none, Zone(0, 1, 0, 1),
                                Zone(0, 1, .5, 1), Zone(0, 1, .5, 1), Zone(0, 1, 0, 1), .5, 1),
            safety=SafetyConfig(auto_start_delay_s=0),
            max_rpm=20, auto_base_rpm=5, vision_kp=1,
        )
        camera = FakeSource(frame); imu = FakeSource(ImuReading(0, 1.0)); odometry = FakeSource(0.0)
        motor = DisabledMotorBoundary()
        runtime = FieldControlRuntime(config, camera, imu, motor=motor, odometry=odometry, clock=lambda: 1.0)
        runtime.tick(); runtime.select_auto(); runtime.start_auto(); runtime.tick()
        self.assertEqual(runtime.machine.state, State.AUTO_ROW_FOLLOW)
        self.assertFalse(getattr(motor, "armed", False))
        self.assertTrue(any(event[0] == "stop" for event in motor.events))

    def test_simulated_runtime_faults_when_camera_timestamp_becomes_stale(self):
        camera = FakeSource(np.zeros((20, 20, 3), dtype=np.uint8), 0.0)
        imu = FakeSource(ImuReading(0, 1.0)); odometry = FakeSource(0.0)
        config = RuntimeConfig(camera_timeout_s=.5)
        runtime = FieldControlRuntime(config, camera, imu, odometry=odometry, clock=lambda: 1.0)
        runtime.tick()
        self.assertEqual(runtime.machine.state, State.MANUAL)
        runtime.select_auto()
        with self.assertRaises(ValueError):
            runtime.start_auto()


if __name__ == "__main__":
    unittest.main()