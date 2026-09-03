"""Top-level application owner for sensors, control runtime and diagnostics."""
from __future__ import annotations

from .config import RuntimeConfig
from .lease import ControlLease
from .motor_boundary import DisabledMotorBoundary
from .runtime import FieldControlRuntime
from .sources import (CameraSource, DepthAICombinedBackend, DepthAIVideoBackend,
                      ImuSource, OdometrySource, EncoderBackend)
from .verified_motor_boundary import open_verified_boundary
from .web import DiagnosticsServer


class FieldControlApplication:
    """Start/stop all software components while keeping motors disarmed."""

    def __init__(self, config: RuntimeConfig, *, encoder_backend: EncoderBackend | None = None,
                 web_host: str = "127.0.0.1", web_port: int = 8080) -> None:
        config = config.validate()
        combined_args = (config.processing_width, config.processing_height,
                         config.navigation_frame_rate_hz, config.imu_sample_hz)
        cam_1_serial = config.vision.camera_serial_for(1)
        sensor_backend = (DepthAICombinedBackend(*combined_args, device_id=cam_1_serial)
                          if cam_1_serial else DepthAICombinedBackend(*combined_args))
        camera = CameraSource(sensor_backend)
        # CAM_2 is intentionally video-only. Its absence must never replace
        # CAM_1's calibrated IMU, and rows 3/4 can simply be disabled.
        camera_2 = CameraSource(DepthAIVideoBackend(
            config.processing_width, config.processing_height,
            config.navigation_frame_rate_hz, config.vision.camera_serial_for(2),
        ))
        imu = ImuSource(sensor_backend)
        lease = ControlLease(config.control_lease_timeout_s)
        motor = DisabledMotorBoundary()
        try:
            if config.physical_can.enabled:
                if encoder_backend is not None:
                    raise ValueError("fysisk CAN-odometri måste dela den verifierade motorworkerns socket")
                motor = open_verified_boundary(
                    channel=config.physical_can.channel or "",
                    slcan_device=config.physical_can.slcan_device or "",
                    max_rpm=config.max_rpm, lease=lease,
                )
                shared_encoder = getattr(motor, "encoder_backend", None)
                if not callable(shared_encoder):
                    raise RuntimeError("verifierad fysisk motorgräns saknar delad encoderadapter")
                encoder_backend = shared_encoder()
            odometry = None if encoder_backend is None else OdometrySource(encoder_backend, config.odometry_geometry)
            self.runtime = FieldControlRuntime(config, camera, imu, camera_2=camera_2,
                                               motor=motor, odometry=odometry, lease=lease)
            self.web = DiagnosticsServer(self.runtime, web_host, web_port) if config.stream_enabled else None
        except Exception:
            close_motor = getattr(motor, "close", None)
            if callable(close_motor):
                close_motor()
            raise

    def start(self) -> None:
        try:
            self.runtime.start()
            if self.web is not None:
                self.web.start()
        except Exception:
            self.runtime.close()
            raise

    def close(self) -> None:
        if self.web is not None:
            self.web.close()
        self.runtime.close()
