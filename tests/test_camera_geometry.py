from __future__ import annotations

import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from field_control.sources import (CAM_B_SENSOR_SIZE, DepthAICameraBackend,
                                   DepthAICombinedBackend, _full_fov_camera_output)


class _Output:
    def __init__(self, calls: list[dict], queue="queue") -> None:
        self.calls = calls
        self.queue = queue

    def createOutputQueue(self, **kwargs):
        self.calls.append({"queue": kwargs})
        return self.queue


class _Camera:
    def __init__(self, queue="queue") -> None:
        self.calls: list[dict] = []
        self.queue = queue

    def requestOutput(self, **kwargs):
        self.calls.append(kwargs)
        return _Output(self.calls, self.queue)


class _Dai:
    class ImgFrame:
        class Type:
            BGR888p = "BGR888p"

    class ImgResizeMode:
        STRETCH = "STRETCH"


class _FrameQueue:
    def get(self):
        return SimpleNamespace(getCvFrame=lambda: "frame")


class _Imu:
    def __init__(self) -> None:
        self.out = SimpleNamespace(createOutputQueue=lambda **kwargs: "imu-queue")

    def enableIMUSensor(self, *_args): pass
    def setBatchReportThreshold(self, *_args): pass
    def setMaxBatchReports(self, *_args): pass


class _Pipeline:
    def __init__(self, camera: _Camera, imu: _Imu | None = None) -> None:
        self.camera, self.imu = camera, imu
        self.started = False

    def create(self, node):
        if node == "Camera":
            return SimpleNamespace(build=lambda socket: self.camera)
        if node == "IMU":
            return self.imu
        raise AssertionError(f"unexpected node {node}")

    def start(self): self.started = True
    def stop(self): pass
    def wait(self): pass


def _fake_dai(pipeline: _Pipeline):
    calibration = SimpleNamespace(getImuToCameraExtrinsics=lambda _socket: ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    device = SimpleNamespace(
        getConnectedIMU=lambda: "BNO086",
        readCalibration=lambda: calibration,
        getConnectedCameraFeatures=lambda: [SimpleNamespace(socket="CAM_B")],
    )
    pipeline.getDefaultDevice = lambda: device
    return SimpleNamespace(
        Pipeline=lambda: pipeline,
        node=SimpleNamespace(Camera="Camera", IMU="IMU"),
        CameraBoardSocket=SimpleNamespace(CAM_B="CAM_B"),
        ImgFrame=_Dai.ImgFrame,
        ImgResizeMode=_Dai.ImgResizeMode,
        IMUSensor=SimpleNamespace(ROTATION_VECTOR="ROTATION_VECTOR"),
    )


class CameraGeometryTests(unittest.TestCase):
    def test_full_fov_request_is_scale_only_16_by_10_without_crop_or_padding(self):
        camera = _Camera()
        queue = _full_fov_camera_output(camera, _Dai, 320, 200, 10.0)

        self.assertEqual(CAM_B_SENSOR_SIZE, (1280, 800))
        self.assertEqual(queue, "queue")
        self.assertEqual(camera.calls[0], {
            "size": (320, 200), "type": "BGR888p",
            "resizeMode": "STRETCH", "fps": 10.0,
        })
        self.assertEqual(camera.calls[1], {"queue": {"maxSize": 2, "blocking": False}})

    def test_full_fov_request_rejects_a_crop_prone_aspect_ratio(self):
        with self.assertRaisesRegex(ValueError, "16:10"):
            _full_fov_camera_output(_Camera(), _Dai, 320, 240, 10.0)

    def test_depthai_camera_backend_wires_full_fov_stretch_output(self):
        camera = _Camera(_FrameQueue())
        pipeline = _Pipeline(camera)
        backend = DepthAICameraBackend(320, 200, 10.0)
        with patch.dict(sys.modules, {"depthai": _fake_dai(pipeline)}):
            self.assertEqual(next(backend.frames()), "frame")
        backend.close()

        self.assertTrue(pipeline.started)
        self.assertEqual(camera.calls[0]["size"], (320, 200))
        self.assertEqual(camera.calls[0]["resizeMode"], "STRETCH")

    def test_depthai_combined_backend_wires_full_fov_stretch_output(self):
        camera = _Camera(_FrameQueue())
        pipeline = _Pipeline(camera, _Imu())
        backend = DepthAICombinedBackend(320, 200, 10.0, 100)
        with patch.dict(sys.modules, {"depthai": _fake_dai(pipeline)}):
            backend._ensure_pipeline()
        backend.close()

        self.assertTrue(pipeline.started)
        self.assertEqual(camera.calls[0]["size"], (320, 200))
        self.assertEqual(camera.calls[0]["resizeMode"], "STRETCH")


if __name__ == "__main__":
    unittest.main()
