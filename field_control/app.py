"""Top-level application owner for sensors, control runtime and diagnostics."""
from __future__ import annotations

from .config import RuntimeConfig
from .motor_boundary import DisabledMotorBoundary
from .runtime import FieldControlRuntime
from .sources import CameraSource, DepthAICombinedBackend, ImuSource, OdometrySource, EncoderBackend
from .web import DiagnosticsServer


class FieldControlApplication:
    """Start/stop all software components while keeping motors disarmed."""

    def __init__(self, config: RuntimeConfig, *, encoder_backend: EncoderBackend | None = None,
                 web_host: str = "127.0.0.1", web_port: int = 8080) -> None:
        sensor_backend = DepthAICombinedBackend(
            config.processing_width, config.processing_height, config.navigation_frame_rate_hz,
            max(3, round(config.navigation_frame_rate_hz)),
        )
        camera = CameraSource(sensor_backend)
        imu = ImuSource(sensor_backend)
        odometry = None if encoder_backend is None else OdometrySource(encoder_backend, config.odometry_geometry)
        self.runtime = FieldControlRuntime(config, camera, imu, motor=DisabledMotorBoundary(), odometry=odometry)
        self.web = DiagnosticsServer(self.runtime, web_host, web_port) if config.stream_enabled else None

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
