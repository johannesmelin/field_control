"""Top-level hardware-independent field-control runtime."""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from .config import RuntimeConfig
from .control import WheelCommand, heading_command, vision_command
from .motor_boundary import DisabledMotorBoundary, MotorBoundary
from .heading import RowHeadingReference
from .lease import ControlLease
from .observation import HeadingProcessor, Observation as SensorObservation, build_observation
from .sources import CameraSource, ImuSource, SourceSnapshot
from .state_machine import FieldStateMachine, Observation, Snapshot, State
from .vision import VisionProcessor, VisionResult


@dataclass(frozen=True)
class RuntimeStatus:
    running: bool
    mode: str
    state: str
    snapshot: Snapshot
    observation: SensorObservation | None
    last_command: WheelCommand | None
    motor_output_armed: bool
    fault: str | None


class FieldControlRuntime:
    """Owns lifecycle and joins latest sensor values without blocking control."""

    def __init__(self, config: RuntimeConfig, camera: CameraSource, imu: ImuSource,
                 *, motor: MotorBoundary | None = None, odometry: object | None = None,
                 clock=time.monotonic) -> None:
        self.config = config.validate()
        self.camera, self.imu = camera, imu
        self.motor = motor or DisabledMotorBoundary()
        self.lease = ControlLease(self.config.control_lease_timeout_s)
        self._clock = clock
        self._odometry = odometry
        self.machine = FieldStateMachine(self.config.safety)
        self.heading = HeadingProcessor(
            self.config.heading_filter_alpha,
            RowHeadingReference(self.config.row_heading_window_m, self.config.heading_reference_min_distance_m),
        )
        self.vision_processor = VisionProcessor()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self._last_frame_timestamp: float | None = None
        self._vision: VisionResult | None = None
        self._frame: object | None = None
        self._observation: SensorObservation | None = None
        self._last_snapshot = self.machine.snapshot(self._clock())
        self._last_command: WheelCommand | None = None
        self._fault: str | None = None
        self._last_imu_timestamp: float | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive(): return
        self._stop.clear()
        self.camera.start(); self.imu.start()
        self._thread = threading.Thread(target=self._run, name="field-control", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self.motor.stop_all("FieldControl shutdown")
        self.camera.stop(); self.imu.stop()
        if self._thread: self._thread.join(timeout=5.0)
        self.motor.stop_all("FieldControl shutdown complete")

    def tick(self) -> RuntimeStatus:
        now = self._clock()
        camera = self.camera.latest.snapshot()
        imu = self.imu.latest.snapshot()
        if imu.value is not None and imu.updated_at_s != self._last_imu_timestamp:
            try:
                self.heading.update(imu.value, visual_following=(camera.connected and camera.age_s(now) is not None
                                                                and camera.age_s(now) <= self.config.camera_timeout_s
                                                                and self._vision is not None and self._vision.target_x is not None),
                                    distance_m=float(self._odometry_snapshot(now).value or 0.0))
            except ValueError as exc:
                imu = SourceSnapshot(imu.value, imu.updated_at_s, False, str(exc))
            self._last_imu_timestamp = imu.updated_at_s
        frame = camera.value
        if frame is not None and camera.updated_at_s != self._last_frame_timestamp:
            self._vision = self.vision_processor.process(frame, camera.updated_at_s or now, self.config.vision)
            self._frame = frame
            self._last_frame_timestamp = camera.updated_at_s
        sensor = build_observation(
            now, camera, imu, self._odometry_snapshot(now), self._vision, self.heading,
            self.config.camera_timeout_s, self.config.imu_timeout_s, self.config.odometry_timeout_s,
        )
        machine_observation = Observation(
            now, sensor.camera_fresh, sensor.imu_fresh, sensor.odometry_fresh, True,
            sensor.visual_target,
            False if sensor.vision is None else sensor.vision.bud_in_trigger_zone,
            False if sensor.vision is None else sensor.vision.bud_in_pick_zone,
            False if sensor.vision is None else sensor.vision.marker_found,
            sensor.distance_m, sensor.row_heading_reliable,
        )
        snapshot = self.machine.tick(machine_observation)
        if snapshot.state is State.FAULT:
            self._fault = snapshot.fault or snapshot.reason
            self.motor.stop_all(self._fault)
        else:
            self._dispatch_command(sensor, snapshot.state)
        with self._lock:
            self._observation, self._last_snapshot = sensor, snapshot
        return self.status()

    def _run(self) -> None:
        period = 1.0 / max(1.0, self.config.navigation_frame_rate_hz)
        while not self._stop.is_set():
            started = self._clock()
            try: self.tick()
            except Exception as exc:
                self._fault = f"RUNTIME_ERROR: {type(exc).__name__}: {exc}"
                self.machine._fault(self._fault)
                self._last_snapshot = self.machine.snapshot(self._clock())
                self.motor.stop_all(self._fault)
            self._stop.wait(max(0.0, period - (self._clock() - started)))

    def status(self) -> RuntimeStatus:
        with self._lock:
            armed = bool(getattr(self.motor, "armed", False))
            return RuntimeStatus(bool(self._thread and self._thread.is_alive()), "AUTO" if self.machine.state.value.startswith("AUTO") else "MANUAL",
                                 self.machine.state.value, self._last_snapshot, self._observation,
                                 self._last_command, armed, self._fault)

    def _odometry_snapshot(self, now_s: float) -> SourceSnapshot[float]:
        if self._odometry is None:
            return SourceSnapshot(0.0, None, False, "ODOMETRY_SOURCE_MISSING")
        return self._odometry.snapshot()

    def _dispatch_command(self, observation: SensorObservation, state: State) -> None:
        active = state in FieldStateMachine._ACTIVE
        command = None
        if active and state not in (State.AUTO_START_DELAY, State.AUTO_PICK,
                                    State.AUTO_IN_ROW_TURN, State.AUTO_NEW_ROW_TURN):
            if observation.visual_target and observation.vision is not None:
                command = vision_command(
                    observation.vision.target_x or 0.0,
                    self.config.vision.x_goal * observation.vision.overlay.shape[1],
                    self.config.auto_base_rpm, self.config.vision_kp,
                    self.config.vision_deadband_px, self.config.max_vision_correction_rpm,
                    self.config.max_rpm,
                )
            elif observation.heading_deg is not None and observation.row_heading_reference_deg is not None and observation.row_heading_reliable:
                command = heading_command(
                    observation.row_heading_reference_deg, observation.heading_deg,
                    self.config.search_speed_rpm, self.config.heading_kp,
                    self.config.heading_deadband_deg, self.config.max_heading_correction_rpm,
                    self.config.max_rpm,
                )
        if command is None:
            self.motor.stop_all(f"state {state.value}")
            self._last_command = None
            return
        self._last_command = command
        if getattr(self.motor, "armed", False):
            self.motor.command(command)

    def select_manual(self) -> None:
        self.motor.stop_all("MANUAL vald"); self.machine.select_manual()

    def select_auto(self) -> None:
        self.motor.stop_all("AUTO valt"); self.machine.select_auto()

    def start_auto(self) -> None:
        with self._lock:
            observation = self._observation
        if observation is None:
            raise ValueError("sensorobservation saknas")
        self.motor.stop_all("AUTO startförberedelse")
        self.machine.request_start_auto(Observation(
            observation.now_s, observation.camera_fresh, observation.imu_fresh,
            observation.odometry_fresh, True, observation.visual_target,
            False if observation.vision is None else observation.vision.bud_in_trigger_zone,
            False if observation.vision is None else observation.vision.bud_in_pick_zone,
            False if observation.vision is None else observation.vision.marker_found,
            observation.distance_m, observation.row_heading_reliable,
        ))

    def stop(self) -> None:
        self.motor.stop_all("STOP"); self.machine.stop()

    def manual_command(self, command: WheelCommand) -> None:
        if self.machine.state is not State.MANUAL:
            raise ValueError("manuellt kommando kräver MANUAL")
        if not getattr(self.motor, "armed", False):
            self.motor.stop_all("manuell output är avstängd")
            raise ValueError("motorutgången är avstängd")
        self.motor.command(command)
        self._last_command = command

    def latest_image(self, view: str) -> bytes | None:
        with self._lock:
            frame, result = self._frame, self._vision
            if frame is None:
                return None
            if view == "raw": image = frame
            elif view == "overlay" and result is not None: image = result.overlay
            elif result is not None: image = result.masks.get(view)
            else: image = None
        if image is None:
            return None
        import cv2
        if image.shape[1] != self.config.stream_width or image.shape[0] != self.config.stream_height:
            image = cv2.resize(image, (self.config.stream_width, self.config.stream_height))
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality])
        return encoded.tobytes() if ok else None
