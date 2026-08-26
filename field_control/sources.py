"""Independent latest-value sensor sources.

The concrete OAK-D adapters are lazy and optional so tests and diagnostics can
run without DepthAI hardware or its Python package installed.
"""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import math
from typing import Callable, Generic, Protocol, TypeVar

from .odometry import DriveGeometry, OdometrySample, from_motor_angles


ValueT = TypeVar("ValueT")


class EncoderReadPreempted(RuntimeError):
    """A physical encoder read lost to an intentional STOP/restart settle.

    This is a transient scheduling outcome, not an encoder or CAN failure.
    Only the shared verified-CAN adapter may raise it.
    """


@dataclass(frozen=True)
class SourceSnapshot(Generic[ValueT]):
    value: ValueT | None
    updated_at_s: float | None
    connected: bool
    error: str | None = None

    def age_s(self, now_s: float) -> float | None:
        if self.updated_at_s is None:
            return None
        return max(0.0, now_s - self.updated_at_s)


class LatestValue(Generic[ValueT]):
    """Thread-safe one-element store; publishing replaces, never queues."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: ValueT | None = None
        self._updated_at_s: float | None = None
        self._connected = False
        self._error: str | None = None

    def publish(self, value: ValueT, updated_at_s: float | None = None) -> None:
        timestamp = time.monotonic() if updated_at_s is None else updated_at_s
        if not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp) or timestamp < 0:
            raise ValueError("monotonisk uppdateringstid måste vara giltig")
        with self._lock:
            if self._updated_at_s is not None and timestamp < self._updated_at_s:
                raise ValueError("uppdateringstiden får inte minska")
            self._value, self._updated_at_s = value, float(timestamp)
            self._connected, self._error = True, None

    def fail(self, error: str) -> None:
        with self._lock:
            self._connected, self._error = False, str(error)

    def snapshot(self) -> SourceSnapshot[ValueT]:
        with self._lock:
            return SourceSnapshot(self._value, self._updated_at_s, self._connected, self._error)


class CameraBackend(Protocol):
    def frames(self): ...
    def close(self) -> None: ...


class ImuBackend(Protocol):
    def samples(self): ...
    def close(self) -> None: ...


class EncoderBackend(Protocol):
    def angles(self) -> tuple[float, float]: ...
    def close(self) -> None: ...


class _ThreadedSource(Generic[ValueT]):
    def __init__(self, read: Callable[[], ValueT], name: str) -> None:
        self.latest: LatestValue[ValueT] = LatestValue()
        self._read, self._name = read, name
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=self._name, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._backend.close()
        except Exception:
            pass
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=5.0)

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                self.latest.publish(self._read())
        except Exception as exc:
            if not self._stop.is_set():
                self.latest.fail(f"{type(exc).__name__}: {exc}")


class CameraSource(_ThreadedSource[object]):
    """Camera worker around an injected backend yielding BGR frames."""

    def __init__(self, backend: CameraBackend) -> None:
        super().__init__(lambda: next(self._frames), "field-camera")
        self._backend = backend
        self._frames = iter(backend.frames())

    def stop(self) -> None:
        super().stop()


class ImuSource(_ThreadedSource[object]):
    """IMU worker around an injected backend yielding heading samples."""

    def __init__(self, backend: ImuBackend) -> None:
        super().__init__(lambda: next(self._samples), "field-imu")
        self._backend = backend
        self._samples = iter(backend.samples())

    def stop(self) -> None:
        super().stop()


class OdometrySource(_ThreadedSource[OdometrySample]):
    """Latest immutable per-wheel odometry derived from motor-angle readings."""

    # The verified CAN worker needs at most one shared 40 ms 0x92 reply
    # deadline for an atomic pair.  Sampling at no more than the established
    # 10 Hz control interval leaves a full command slot between samples and
    # prevents sensor polling from continuously occupying the sole worker.
    SAMPLE_PERIOD_S = 0.100

    def __init__(self, backend: EncoderBackend, geometry: DriveGeometry) -> None:
        self._backend = backend
        self._geometry = geometry.validate()
        self._initial: tuple[float, float] | None = None
        # Arming a physical boundary must wait for an actual encoder sample,
        # not merely for this source thread to have been started.  This is a
        # condition rather than a polling delay so the first publish, a read
        # failure and stop/close all wake a blocked armer promptly.
        self._readiness = threading.Condition()
        super().__init__(self._read_sample, "field-odometry")

    def _read_sample(self) -> OdometrySample:
        angles = self._backend.angles()
        if self._initial is None:
            self._initial = angles
        return from_motor_angles(*self._initial, *angles, self._geometry)

    def snapshot(self) -> SourceSnapshot[OdometrySample]:
        """Return one immutable, timestamped latest sample without queueing."""
        return self.latest.snapshot()

    def _run(self) -> None:
        """Publish at a bounded rate; never turn a latest-value source into a CAN queue."""
        try:
            while not self._stop.is_set():
                self.latest.publish(self._read())
                with self._readiness:
                    self._readiness.notify_all()
                self._stop.wait(self.SAMPLE_PERIOD_S)
        except Exception as exc:
            if not self._stop.is_set():
                self.latest.fail(f"{type(exc).__name__}: {exc}")
        finally:
            with self._readiness:
                self._readiness.notify_all()

    def wait_until_ready(self, timeout_s: float) -> bool:
        """Wait boundedly for the first valid sample or a terminal source state.

        ``stop()`` wakes this wait immediately, so runtime shutdown never has
        to wait for the acquisition timeout.  The caller deliberately owns
        lifecycle locking; this method holds only the local condition.
        """
        if (not isinstance(timeout_s, (int, float)) or isinstance(timeout_s, bool)
                or not math.isfinite(timeout_s) or timeout_s <= 0):
            raise ValueError("odometrins readiness-timeout måste vara positiv")
        deadline = time.monotonic() + float(timeout_s)
        with self._readiness:
            while True:
                snapshot = self.snapshot()
                if snapshot.connected and isinstance(snapshot.value, OdometrySample):
                    return True
                if self._stop.is_set() or snapshot.error is not None:
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._readiness.wait(remaining)

    def stop(self) -> None:
        self.begin_shutdown()
        super().stop()

    def begin_shutdown(self) -> None:
        """Cancel future sampling without taking ownership of the CAN sink.

        This small first phase is used by the runtime before the physical
        motor boundary claims its final STOP+close. It wakes a sampler and
        prevents any subsequent 0x92 admission during shutdown.
        """
        # Wake an arming waiter before closing a possibly blocking backend.
        self._stop.set()
        with self._readiness:
            self._readiness.notify_all()
        begin_shutdown = getattr(self._backend, "begin_shutdown", None)
        if callable(begin_shutdown):
            begin_shutdown()


class DepthAICameraBackend:
    """Minimal lazy OAK-D SR BGR backend; queue depth is explicitly bounded."""

    def __init__(self, width: int, height: int, fps: float) -> None:
        self.width, self.height, self.fps = width, height, fps
        self._pipeline = None
        self._queue = None

    def frames(self):
        import depthai as dai
        self._pipeline = dai.Pipeline()
        camera = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self._queue = camera.requestOutput(
            size=(self.width, self.height), type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.CROP, fps=self.fps,
        ).createOutputQueue(maxSize=2, blocking=False)
        self._pipeline.start()
        while True:
            yield self._queue.get().getCvFrame()

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
                self._pipeline.wait()
            except Exception:
                pass
            self._pipeline = None


def heading_from_imu_quaternion(
    quaternion: tuple[float, float, float, float],
    imu_to_camera: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]] | None = None,
) -> float:
    """Tilt-compensated heading of the calibrated housing forward axis.

    The quaternion is the BNO086 body-to-world rotation and the housing's
    forward axis is +Z.  Deployments with a non-identity camera calibration
    should pass a calibrated converter to :class:`DepthAIImuBackend`.
    """
    x, y, z, w = quaternion
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12 or not all(math.isfinite(value) for value in quaternion):
        raise ValueError("ogiltig IMU-quaternion")
    x, y, z, w = (value / norm for value in quaternion)
    rotation = imu_to_camera or ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    camera_to_imu = tuple(tuple(rotation[column][row] for column in range(3)) for row in range(3))
    forward_imu = tuple(camera_to_imu[row][2] for row in range(3))
    forward_x = ((1 - 2 * (y * y + z * z)) * forward_imu[0]
                 + 2 * (x * y - w * z) * forward_imu[1]
                 + 2 * (x * z + w * y) * forward_imu[2])
    forward_y = (2 * (x * y + w * z) * forward_imu[0]
                 + (1 - 2 * (x * x + z * z)) * forward_imu[1]
                 + 2 * (y * z - w * x) * forward_imu[2])
    if math.hypot(forward_x, forward_y) < 1e-3:
        raise ValueError("framaxelns horisontella heading är instabil")
    return (-math.degrees(math.atan2(forward_y, forward_x))) % 360.0


class DepthAIImuBackend:
    """Lazy BNO086 ROTATION_VECTOR backend with a bounded output queue."""

    def __init__(self, rate_hz: int, heading_converter: Callable[[tuple[float, float, float, float]], float] = heading_from_imu_quaternion) -> None:
        self.rate_hz, self.heading_converter = rate_hz, heading_converter
        self._pipeline = None
        self._queue = None

    def samples(self):
        import depthai as dai
        self._pipeline = dai.Pipeline()
        imu = self._pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, self.rate_hz)
        imu.setBatchReportThreshold(1); imu.setMaxBatchReports(10)
        self._queue = imu.out.createOutputQueue(maxSize=10, blocking=False)
        self._pipeline.start()
        while True:
            packet = self._queue.get()
            packet = packet.packets[-1] if hasattr(packet, "packets") else packet
            report = packet.rotationVector if hasattr(packet, "rotationVector") else packet
            quaternion = (float(report.i), float(report.j), float(report.k), float(report.real))
            yield __import__("field_control.observation", fromlist=["ImuReading"]).ImuReading(
                self.heading_converter(quaternion), time.monotonic(),
            )

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop(); self._pipeline.wait()
            except Exception:
                pass
            self._pipeline = None


class DepthAICombinedBackend:
    """One OAK-D pipeline exposing independent bounded camera/IMU queues."""

    def __init__(self, width: int, height: int, camera_fps: float, imu_rate_hz: int,
                 heading_converter: Callable[[tuple[float, float, float, float]], float] = heading_from_imu_quaternion) -> None:
        self.width, self.height, self.camera_fps, self.imu_rate_hz = width, height, camera_fps, imu_rate_hz
        self.heading_converter = heading_converter
        self._pipeline = None
        self._camera_queue = None
        self._imu_queue = None
        self._lock = threading.Lock()

    def _ensure_pipeline(self) -> None:
        with self._lock:
            if self._pipeline is not None:
                return
            import depthai as dai
            pipeline = dai.Pipeline()
            camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            imu = pipeline.create(dai.node.IMU)
            device = pipeline.getDefaultDevice()
            imu_type = str(device.getConnectedIMU())
            if imu_type != "BNO086":
                raise RuntimeError(f"ROTATION_VECTOR kräver BNO086, hittade {imu_type or 'ingen IMU'}")
            calibration = device.readCalibration()
            features = [feature.socket for feature in device.getConnectedCameraFeatures()]
            camera_socket = next(socket for socket in features if str(socket).endswith("CAM_B"))
            matrix = calibration.getImuToCameraExtrinsics(camera_socket)
            rotation = tuple(tuple(float(matrix[row][column]) for column in range(3)) for row in range(3))
            self.heading_converter = lambda quaternion: heading_from_imu_quaternion(quaternion, rotation)
            imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, self.imu_rate_hz)
            imu.setBatchReportThreshold(1); imu.setMaxBatchReports(10)
            camera_queue = camera.requestOutput(
                size=(self.width, self.height), type=dai.ImgFrame.Type.BGR888p,
                resizeMode=dai.ImgResizeMode.CROP, fps=self.camera_fps,
            ).createOutputQueue(maxSize=2, blocking=False)
            imu_queue = imu.out.createOutputQueue(maxSize=10, blocking=False)
            pipeline.start()
            self._pipeline, self._camera_queue, self._imu_queue = pipeline, camera_queue, imu_queue

    def frames(self):
        self._ensure_pipeline()
        while True:
            packet = self._camera_queue.tryGet()
            if packet is not None:
                yield packet.getCvFrame()
            else:
                threading.Event().wait(.002)

    def samples(self):
        self._ensure_pipeline()
        while True:
            data = self._imu_queue.tryGet()
            if data is None:
                threading.Event().wait(.002)
                continue
            packet = data.packets[-1] if hasattr(data, "packets") else data
            report = packet.rotationVector if hasattr(packet, "rotationVector") else packet
            quaternion = (float(report.i), float(report.j), float(report.k), float(report.real))
            from .observation import ImuReading
            yield ImuReading(self.heading_converter(quaternion), time.monotonic())

    def close(self) -> None:
        with self._lock:
            pipeline, self._pipeline = self._pipeline, None
            self._camera_queue = self._imu_queue = None
        if pipeline is not None:
            try:
                pipeline.stop(); pipeline.wait()
            except Exception:
                pass
