"""Raised-wheel HIL runners for fixed manual web endpoint checks.

Each runner starts an isolated MANUAL-only runtime, arms the verified output,
and exercises only its existing diagnostics HTTP route.  It never starts AUTO.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import threading
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .config import PhysicalCanConfig, RuntimeConfig
from .lease import ControlLease
from .odometry import motor_rpm_to_wheel_rpm
from .runtime import CONTROL_LEASE_EXPIRED, FieldControlRuntime
from .sources import LatestValue
from .verified_motor_boundary import open_verified_boundary
from .web import DiagnosticsServer

CAN_CHANNEL = "can0"
REPLY_PROFILE = "observed-rmdx-same-id"
MANUAL_FORWARD_RPM = 10.0
MANUAL_REVERSE_RPM = 10.0
MANUAL_REVERSE_DURATION_S = 5.0
MANUAL_REVERSE_REFRESH_S = 0.100
MANUAL_LEFT_RPM = 10.0
MANUAL_LEFT_DURATION_S = 5.0
MANUAL_LEFT_REFRESH_S = 0.100
MANUAL_RIGHT_RPM = 10.0
MANUAL_RIGHT_DURATION_S = 5.0
MANUAL_RIGHT_REFRESH_S = 0.100
MANUAL_STOP_RPM = 10.0
MANUAL_STOP_ACTIVE_DURATION_S = 3.0
MANUAL_STOP_REFRESH_S = 0.100
LEASE_TIMEOUT_S = 0.200
WAIT_MARGIN_S = 0.100
WEB_REQUEST_TIMEOUT_S = 1.0

# Private seams keep the deadline race test deterministic without changing the
# physical runner's fixed timing or exposing an operator control.
_deadline_event_factory = threading.Event
_deadline_timer_factory = threading.Timer


class _StaticSource:
    """No-op source: this isolated MANUAL HIL never starts OAK or AUTO."""
    def __init__(self) -> None:
        self.latest: LatestValue[object] = LatestValue()
    def start(self) -> None: pass
    def stop(self) -> None: pass


@dataclass(frozen=True)
class ManualWebForwardRequest:
    slcan_device: str
    enable_motors: bool = False
    confirm_physical_stop_tested: bool = False
    confirm_wheels_raised: bool = False

    def validate(self) -> "ManualWebForwardRequest":
        if self.enable_motors is not True:
            raise ValueError("--enable-motors krävs")
        if self.confirm_physical_stop_tested is not True:
            raise ValueError("--confirm-physical-stop-tested krävs")
        if self.confirm_wheels_raised is not True:
            raise ValueError("--confirm-wheels-raised krävs")
        prefix = "/dev/serial/by-id/"
        path = self.slcan_device
        basename = path[len(prefix):] if isinstance(path, str) and path.startswith(prefix) else ""
        if not isinstance(path, str) or not basename or basename in (".", "..") or "/" in basename:
            raise ValueError("exakt stabil /dev/serial/by-id/-sökväg krävs")
        return self


@dataclass(frozen=True)
class ManualWebForwardResult:
    action: str
    command_motor_rpm: tuple[float, float]
    expected_wheel_rpm: tuple[float, float]
    lease_fault: str
    can_path: str
    diagnostics: dict[str, Any]


@dataclass(frozen=True)
class ManualWebReverseResult:
    action: str
    command_motor_rpm: tuple[float, float]
    expected_wheel_rpm: tuple[float, float]
    can_path: str
    diagnostics: dict[str, Any]


@dataclass(frozen=True)
class ManualWebLeftResult:
    action: str
    command_motor_rpm: tuple[float, float]
    expected_wheel_rpm: tuple[float, float]
    can_path: str
    diagnostics: dict[str, Any]


@dataclass(frozen=True)
class ManualWebRightResult:
    action: str
    command_motor_rpm: tuple[float, float]
    expected_wheel_rpm: tuple[float, float]
    can_path: str
    diagnostics: dict[str, Any]


@dataclass(frozen=True)
class ManualWebStopResult:
    """Evidence that the existing web STOP route disarmed active output."""
    action: str
    active_command_motor_rpm: tuple[float, float]
    expected_wheel_rpm: tuple[float, float]
    active_program_window_s: float
    can_path: str
    diagnostics: dict[str, Any]


def _bounded_diagnostics(motor: object) -> dict[str, Any]:
    sink = getattr(motor, "_sink", motor)
    snapshot = getattr(sink, "diagnostic_snapshot", None)
    if callable(snapshot):
        try:
            return {"worker": repr(snapshot())[:2000]}
        except Exception as exc:
            return {"worker_error": f"{type(exc).__name__}: {exc}"[:2000]}
    return {"adapter_events": [repr(item)[:240] for item in list(getattr(motor, "events", ()))[-8:]]}


def _post_forward(server: DiagnosticsServer) -> None:
    host, port = server.address
    request = Request(f"http://{host}:{port}/api/manual/forward", method="POST", data=b"")
    try:
        with urlopen(request, timeout=WEB_REQUEST_TIMEOUT_S) as response:
            if response.status != 200:
                raise RuntimeError(f"webb-forward avvisades med HTTP {response.status}")
            response.read()
    except HTTPError as exc:
        raise RuntimeError(f"webb-forward avvisades med HTTP {exc.code}") from exc
    except URLError as exc:
        raise RuntimeError(f"webb-forward kunde inte nås: {exc.reason}") from exc


def _post_reverse(server: DiagnosticsServer) -> None:
    host, port = server.address
    request = Request(f"http://{host}:{port}/api/manual/reverse", method="POST", data=b"")
    try:
        with urlopen(request, timeout=WEB_REQUEST_TIMEOUT_S) as response:
            if response.status != 200:
                raise RuntimeError(f"webb-back avvisades med HTTP {response.status}")
            response.read()
    except HTTPError as exc:
        raise RuntimeError(f"webb-back avvisades med HTTP {exc.code}") from exc
    except URLError as exc:
        raise RuntimeError(f"webb-back kunde inte nås: {exc.reason}") from exc


def _post_left(server: DiagnosticsServer) -> None:
    """Use only the existing fixed logical-vehicle left HTTP route."""
    host, port = server.address
    request = Request(f"http://{host}:{port}/api/manual/left", method="POST", data=b"")
    try:
        with urlopen(request, timeout=WEB_REQUEST_TIMEOUT_S) as response:
            if response.status != 200:
                raise RuntimeError(f"webb-vänster avvisades med HTTP {response.status}")
            response.read()
    except HTTPError as exc:
        raise RuntimeError(f"webb-vänster avvisades med HTTP {exc.code}") from exc
    except URLError as exc:
        raise RuntimeError(f"webb-vänster kunde inte nås: {exc.reason}") from exc


def _post_right(server: DiagnosticsServer) -> None:
    """Use only the existing fixed logical-vehicle right HTTP route."""
    host, port = server.address
    request = Request(f"http://{host}:{port}/api/manual/right", method="POST", data=b"")
    try:
        with urlopen(request, timeout=WEB_REQUEST_TIMEOUT_S) as response:
            if response.status != 200:
                raise RuntimeError(f"webb-höger avvisades med HTTP {response.status}")
            response.read()
    except HTTPError as exc:
        raise RuntimeError(f"webb-höger avvisades med HTTP {exc.code}") from exc
    except URLError as exc:
        raise RuntimeError(f"webb-höger kunde inte nås: {exc.reason}") from exc


def _post_stop(server: DiagnosticsServer) -> None:
    """Invoke only the existing global STOP HTTP route."""
    host, port = server.address
    request = Request(f"http://{host}:{port}/api/stop", method="POST", data=b"")
    try:
        with urlopen(request, timeout=WEB_REQUEST_TIMEOUT_S) as response:
            if response.status != 200:
                raise RuntimeError(f"webb-STOP avvisades med HTTP {response.status}")
            response.read()
    except HTTPError as exc:
        raise RuntimeError(f"webb-STOP avvisades med HTTP {exc.code}") from exc
    except URLError as exc:
        raise RuntimeError(f"webb-STOP kunde inte nås: {exc.reason}") from exc


def run_manual_web_forward(request: ManualWebForwardRequest) -> ManualWebForwardResult:
    """Issue exactly one fixed forward web request and require lease expiry."""
    request.validate()
    lease = ControlLease(LEASE_TIMEOUT_S)
    motor: object | None = None
    runtime: FieldControlRuntime | None = None
    web: DiagnosticsServer | None = None
    result: ManualWebForwardResult | None = None
    error: BaseException | None = None
    config = RuntimeConfig(
        stream_enabled=False, max_rpm=MANUAL_FORWARD_RPM, manual_rpm=MANUAL_FORWARD_RPM,
        control_lease_timeout_s=LEASE_TIMEOUT_S, watchdog_period_s=.020, max_control_stall_s=.120,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
    )
    try:
        motor = open_verified_boundary(channel=CAN_CHANNEL, slcan_device=request.slcan_device,
                                       max_rpm=MANUAL_FORWARD_RPM, lease=lease)
        runtime = FieldControlRuntime(config, _StaticSource(), _StaticSource(), motor=motor, lease=lease)
        runtime.start()
        web = DiagnosticsServer(runtime, host="127.0.0.1", port=0)
        web.start()
        runtime.arm_motor_output()
        _post_forward(web)  # Sole nonzero path: existing HTTP endpoint.
        deadline = time.monotonic() + LEASE_TIMEOUT_S + WAIT_MARGIN_S
        while time.monotonic() < deadline:
            status = runtime.status()
            if status.fault is not None and not status.motor_output_armed:
                if status.fault != CONTROL_LEASE_EXPIRED:
                    raise RuntimeError(f"oväntat manual-web-fel: {status.fault}")
                command = status.last_admitted_nonzero_command
                if command is None or (command.left_rpm, command.right_rpm) != (MANUAL_FORWARD_RPM, MANUAL_FORWARD_RPM):
                    raise RuntimeError("webb-forward gav inte exakt väntat motor-side kommando")
                result = ManualWebForwardResult(
                    "forward", (command.left_rpm, command.right_rpm),
                    tuple(motor_rpm_to_wheel_rpm(value, config.odometry_geometry)
                          for value in (command.left_rpm, command.right_rpm)),
                    status.fault, request.slcan_device, {},
                )
                break
            time.sleep(.005)
        if result is None:
            raise TimeoutError("watchdog återkallade inte manual-web-lease inom bounded deadline")
    except BaseException as exc:
        error = exc
    finally:
        if web is not None:
            try: web.close()
            except BaseException as exc:
                if error is None: error = exc
        if runtime is not None:
            try: runtime.close()
            except BaseException as exc:
                if error is None: error = exc
        elif motor is not None:
            try: motor.close()  # type: ignore[attr-defined]
            except BaseException as exc:
                if error is None: error = exc
        diagnostics = _bounded_diagnostics(motor) if motor is not None else {"adapter_events": []}
    if error is not None:
        try: setattr(error, "diagnostics", diagnostics)
        except Exception: pass
        raise error
    assert result is not None
    return ManualWebForwardResult(result.action, result.command_motor_rpm, result.expected_wheel_rpm,
                                  result.lease_fault, result.can_path, diagnostics)


def run_manual_web_reverse(request: ManualWebForwardRequest) -> ManualWebReverseResult:
    """Refresh only the fixed reverse web route for five seconds, then STOP.

    This is intentionally a separate runner from forward: forward verifies
    lease-expiry stopping, while this longer observer-visible reverse check
    verifies repeated genuine web requests and explicit STOP handling.
    """
    request.validate()
    lease = ControlLease(LEASE_TIMEOUT_S)
    motor: object | None = None
    runtime: FieldControlRuntime | None = None
    web: DiagnosticsServer | None = None
    deadline_timer: object | None = None
    result: ManualWebReverseResult | None = None
    error: BaseException | None = None
    config = RuntimeConfig(
        stream_enabled=False, max_rpm=MANUAL_REVERSE_RPM, manual_rpm=MANUAL_REVERSE_RPM,
        control_lease_timeout_s=LEASE_TIMEOUT_S, watchdog_period_s=.020, max_control_stall_s=.120,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
    )
    try:
        motor = open_verified_boundary(channel=CAN_CHANNEL, slcan_device=request.slcan_device,
                                       max_rpm=MANUAL_REVERSE_RPM, lease=lease)
        runtime = FieldControlRuntime(config, _StaticSource(), _StaticSource(), motor=motor, lease=lease)
        runtime.start()
        web = DiagnosticsServer(runtime, host="127.0.0.1", port=0)
        web.start()
        runtime.arm_motor_output()
        deadline_reached = _deadline_event_factory()
        deadline_error: list[BaseException] = []

        def deadline_stop() -> None:
            # This runs independently of a blocked HTTP client request.  The
            # route checks the now-disarmed output, so a late handler cannot
            # admit another nonzero command after this STOP.
            try:
                runtime.stop()
            except BaseException as exc:
                deadline_error.append(exc)
            finally:
                deadline_reached.set()

        deadline_timer = _deadline_timer_factory(MANUAL_REVERSE_DURATION_S, deadline_stop)
        deadline_timer.daemon = True
        deadline_timer.start()
        while not deadline_reached.is_set():
            try:
                _post_reverse(web)  # Only nonzero path: existing HTTP reverse endpoint.
            except RuntimeError:
                # A request already in progress when deadline_stop disarms the
                # runtime is correctly rejected by the endpoint.
                if deadline_reached.is_set():
                    break
                raise
            deadline_reached.wait(MANUAL_REVERSE_REFRESH_S)
        if deadline_error:
            raise RuntimeError(f"webb-back STOP misslyckades: {type(deadline_error[0]).__name__}: {deadline_error[0]}")
        command = runtime.status().last_admitted_nonzero_command
        if command is None or (command.left_rpm, command.right_rpm) != (-MANUAL_REVERSE_RPM, -MANUAL_REVERSE_RPM):
            raise RuntimeError("webb-back gav inte exakt väntat motor-side kommando")
        status = runtime.status()
        if status.motor_output_armed or status.fault is not None:
            raise RuntimeError("webb-back STOP lämnade motorutgång armerad eller runtime felad")
        result = ManualWebReverseResult(
            "reverse", (command.left_rpm, command.right_rpm),
            tuple(motor_rpm_to_wheel_rpm(value, config.odometry_geometry)
                  for value in (command.left_rpm, command.right_rpm)),
            request.slcan_device, {},
        )
    except BaseException as exc:
        error = exc
    finally:
        if deadline_timer is not None:
            deadline_timer.cancel()  # type: ignore[attr-defined]
            deadline_timer.join(timeout=0.5)  # type: ignore[attr-defined]
        if web is not None:
            try: web.close()
            except BaseException as exc:
                if error is None: error = exc
        if runtime is not None:
            try: runtime.close()
            except BaseException as exc:
                if error is None: error = exc
        elif motor is not None:
            try: motor.close()  # type: ignore[attr-defined]
            except BaseException as exc:
                if error is None: error = exc
        diagnostics = _bounded_diagnostics(motor) if motor is not None else {"adapter_events": []}
    if error is not None:
        try: setattr(error, "diagnostics", diagnostics)
        except Exception: pass
        raise error
    assert result is not None
    return ManualWebReverseResult(result.action, result.command_motor_rpm, result.expected_wheel_rpm,
                                  result.can_path, diagnostics)


def run_manual_web_left(request: ManualWebForwardRequest) -> ManualWebLeftResult:
    """Refresh the fixed left web route for five seconds, then explicitly STOP.

    Logical vehicle left is deliberately fixed at ``(-10, +10)`` motor RPM.
    The verified physical worker alone applies its configured motor signs.
    An independent deadline owns STOP so a blocked final web request cannot
    re-admit output after the test window.
    """
    request.validate()
    lease = ControlLease(LEASE_TIMEOUT_S)
    motor: object | None = None
    runtime: FieldControlRuntime | None = None
    web: DiagnosticsServer | None = None
    deadline_timer: object | None = None
    result: ManualWebLeftResult | None = None
    error: BaseException | None = None
    config = RuntimeConfig(
        stream_enabled=False, max_rpm=MANUAL_LEFT_RPM, manual_rpm=MANUAL_LEFT_RPM,
        control_lease_timeout_s=LEASE_TIMEOUT_S, watchdog_period_s=.020, max_control_stall_s=.120,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
    )
    try:
        motor = open_verified_boundary(channel=CAN_CHANNEL, slcan_device=request.slcan_device,
                                       max_rpm=MANUAL_LEFT_RPM, lease=lease)
        runtime = FieldControlRuntime(config, _StaticSource(), _StaticSource(), motor=motor, lease=lease)
        runtime.start()
        web = DiagnosticsServer(runtime, host="127.0.0.1", port=0)
        web.start()
        runtime.arm_motor_output()
        deadline_reached = _deadline_event_factory()
        deadline_error: list[BaseException] = []

        def deadline_stop() -> None:
            try:
                runtime.stop()
            except BaseException as exc:
                deadline_error.append(exc)
            finally:
                deadline_reached.set()

        deadline_timer = _deadline_timer_factory(MANUAL_LEFT_DURATION_S, deadline_stop)
        deadline_timer.daemon = True
        deadline_timer.start()
        while not deadline_reached.is_set():
            try:
                _post_left(web)  # Sole nonzero path: the existing HTTP left endpoint.
            except RuntimeError:
                if deadline_reached.is_set():
                    break
                raise
            deadline_reached.wait(MANUAL_LEFT_REFRESH_S)
        if deadline_error:
            detail = deadline_error[0]
            raise RuntimeError(f"webb-vänster STOP misslyckades: {type(detail).__name__}: {detail}")
        command = runtime.status().last_admitted_nonzero_command
        expected = (-MANUAL_LEFT_RPM, MANUAL_LEFT_RPM)
        if command is None or (command.left_rpm, command.right_rpm) != expected:
            raise RuntimeError("webb-vänster gav inte exakt väntat motor-side kommando")
        status = runtime.status()
        if status.motor_output_armed or status.fault is not None:
            raise RuntimeError("webb-vänster STOP lämnade motorutgång armerad eller runtime felad")
        result = ManualWebLeftResult(
            "left", expected,
            tuple(motor_rpm_to_wheel_rpm(value, config.odometry_geometry) for value in expected),
            request.slcan_device, {},
        )
    except BaseException as exc:
        error = exc
    finally:
        if deadline_timer is not None:
            deadline_timer.cancel()  # type: ignore[attr-defined]
            deadline_timer.join(timeout=0.5)  # type: ignore[attr-defined]
        if web is not None:
            try: web.close()
            except BaseException as exc:
                if error is None: error = exc
        if runtime is not None:
            try: runtime.close()
            except BaseException as exc:
                if error is None: error = exc
        elif motor is not None:
            try: motor.close()  # type: ignore[attr-defined]
            except BaseException as exc:
                if error is None: error = exc
        diagnostics = _bounded_diagnostics(motor) if motor is not None else {"adapter_events": []}
    if error is not None:
        try: setattr(error, "diagnostics", diagnostics)
        except Exception: pass
        raise error
    assert result is not None
    return ManualWebLeftResult(result.action, result.command_motor_rpm, result.expected_wheel_rpm,
                               result.can_path, diagnostics)


def run_manual_web_right(request: ManualWebForwardRequest) -> ManualWebRightResult:
    """Refresh the fixed right web route for five seconds, then explicitly STOP.

    Logical vehicle right is deliberately fixed at ``(+10, -10)`` motor RPM.
    The verified physical worker alone applies its configured motor signs.
    An independent deadline owns STOP so a blocked final web request cannot
    re-admit output after the test window.
    """
    request.validate()
    lease = ControlLease(LEASE_TIMEOUT_S)
    motor: object | None = None
    runtime: FieldControlRuntime | None = None
    web: DiagnosticsServer | None = None
    deadline_timer: object | None = None
    result: ManualWebRightResult | None = None
    error: BaseException | None = None
    config = RuntimeConfig(
        stream_enabled=False, max_rpm=MANUAL_RIGHT_RPM, manual_rpm=MANUAL_RIGHT_RPM,
        control_lease_timeout_s=LEASE_TIMEOUT_S, watchdog_period_s=.020, max_control_stall_s=.120,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
    )
    try:
        motor = open_verified_boundary(channel=CAN_CHANNEL, slcan_device=request.slcan_device,
                                       max_rpm=MANUAL_RIGHT_RPM, lease=lease)
        runtime = FieldControlRuntime(config, _StaticSource(), _StaticSource(), motor=motor, lease=lease)
        runtime.start()
        web = DiagnosticsServer(runtime, host="127.0.0.1", port=0)
        web.start()
        runtime.arm_motor_output()
        deadline_reached = _deadline_event_factory()
        deadline_error: list[BaseException] = []

        def deadline_stop() -> None:
            try:
                runtime.stop()
            except BaseException as exc:
                deadline_error.append(exc)
            finally:
                deadline_reached.set()

        deadline_timer = _deadline_timer_factory(MANUAL_RIGHT_DURATION_S, deadline_stop)
        deadline_timer.daemon = True
        deadline_timer.start()
        while not deadline_reached.is_set():
            try:
                _post_right(web)  # Sole nonzero path: the existing HTTP right endpoint.
            except RuntimeError:
                if deadline_reached.is_set():
                    break
                raise
            deadline_reached.wait(MANUAL_RIGHT_REFRESH_S)
        if deadline_error:
            detail = deadline_error[0]
            raise RuntimeError(f"webb-höger STOP misslyckades: {type(detail).__name__}: {detail}")
        command = runtime.status().last_admitted_nonzero_command
        expected = (MANUAL_RIGHT_RPM, -MANUAL_RIGHT_RPM)
        if command is None or (command.left_rpm, command.right_rpm) != expected:
            raise RuntimeError("webb-höger gav inte exakt väntat motor-side kommando")
        status = runtime.status()
        if status.motor_output_armed or status.fault is not None:
            raise RuntimeError("webb-höger STOP lämnade motorutgång armerad eller runtime felad")
        result = ManualWebRightResult(
            "right", expected,
            tuple(motor_rpm_to_wheel_rpm(value, config.odometry_geometry) for value in expected),
            request.slcan_device, {},
        )
    except BaseException as exc:
        error = exc
    finally:
        if deadline_timer is not None:
            deadline_timer.cancel()  # type: ignore[attr-defined]
            deadline_timer.join(timeout=0.5)  # type: ignore[attr-defined]
        if web is not None:
            try: web.close()
            except BaseException as exc:
                if error is None: error = exc
        if runtime is not None:
            try: runtime.close()
            except BaseException as exc:
                if error is None: error = exc
        elif motor is not None:
            try: motor.close()  # type: ignore[attr-defined]
            except BaseException as exc:
                if error is None: error = exc
        diagnostics = _bounded_diagnostics(motor) if motor is not None else {"adapter_events": []}
    if error is not None:
        try: setattr(error, "diagnostics", diagnostics)
        except Exception: pass
        raise error
    assert result is not None
    return ManualWebRightResult(result.action, result.command_motor_rpm, result.expected_wheel_rpm,
                                result.can_path, diagnostics)


def run_manual_web_stop(request: ManualWebForwardRequest) -> ManualWebStopResult:
    """Run fixed forward output, then require web ``/api/stop`` to disarm it.

    The three-second active window is deliberately fixed and lease-refreshed.
    Success is impossible until the active command was admitted and the actual
    web STOP request has returned with output disarmed and no runtime fault.
    ``finally`` still owns the verified STOP+0x9C close path on every error.
    """
    request.validate()
    lease = ControlLease(LEASE_TIMEOUT_S)
    motor: object | None = None
    runtime: FieldControlRuntime | None = None
    web: DiagnosticsServer | None = None
    result: ManualWebStopResult | None = None
    error: BaseException | None = None
    config = RuntimeConfig(
        stream_enabled=False, max_rpm=MANUAL_STOP_RPM, manual_rpm=MANUAL_STOP_RPM,
        control_lease_timeout_s=LEASE_TIMEOUT_S, watchdog_period_s=.020, max_control_stall_s=.120,
        physical_can=PhysicalCanConfig(True, CAN_CHANNEL, REPLY_PROFILE, request.slcan_device, True, True),
    )
    try:
        motor = open_verified_boundary(channel=CAN_CHANNEL, slcan_device=request.slcan_device,
                                       max_rpm=MANUAL_STOP_RPM, lease=lease)
        runtime = FieldControlRuntime(config, _StaticSource(), _StaticSource(), motor=motor, lease=lease)
        runtime.start()
        web = DiagnosticsServer(runtime, host="127.0.0.1", port=0)
        web.start()
        runtime.arm_motor_output()
        started = time.monotonic()
        deadline = started + MANUAL_STOP_ACTIVE_DURATION_S
        sent_active = False
        while time.monotonic() < deadline:
            _post_forward(web)  # Sole nonzero path: existing fixed web route.
            sent_active = True
            status = runtime.status()
            expected = (MANUAL_STOP_RPM, MANUAL_STOP_RPM)
            command = status.last_command
            if status.fault is not None or not status.motor_output_armed:
                raise RuntimeError("webb-STOP-testets aktiva kommando avbröts före STOP")
            if command is None or (command.left_rpm, command.right_rpm) != expected:
                raise RuntimeError("webb-STOP-test gav inte exakt väntat aktivt motor-side kommando")
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(MANUAL_STOP_REFRESH_S, remaining))
        if not sent_active:
            raise RuntimeError("webb-STOP-test hann inte aktivera något manuellt kommando")
        _post_stop(web)  # The result is valid only if this existing route succeeds.
        status = runtime.status()
        if status.motor_output_armed or status.fault is not None:
            raise RuntimeError("webb-STOP lämnade motorutgång armerad eller runtime felad")
        expected = (MANUAL_STOP_RPM, MANUAL_STOP_RPM)
        command = status.last_admitted_nonzero_command
        if command is None or (command.left_rpm, command.right_rpm) != expected:
            raise RuntimeError("webb-STOP-test saknar bevis på föregående aktivt motor-side kommando")
        result = ManualWebStopResult(
            "stop", expected,
            tuple(motor_rpm_to_wheel_rpm(value, config.odometry_geometry) for value in expected),
            time.monotonic() - started, request.slcan_device, {},
        )
    except BaseException as exc:
        error = exc
    finally:
        if web is not None:
            try: web.close()
            except BaseException as exc:
                if error is None: error = exc
        if runtime is not None:
            try: runtime.close()
            except BaseException as exc:
                if error is None: error = exc
        elif motor is not None:
            try: motor.close()  # type: ignore[attr-defined]
            except BaseException as exc:
                if error is None: error = exc
        diagnostics = _bounded_diagnostics(motor) if motor is not None else {"adapter_events": []}
    if error is not None:
        try: setattr(error, "diagnostics", diagnostics)
        except Exception: pass
        raise error
    assert result is not None
    return ManualWebStopResult(result.action, result.active_command_motor_rpm, result.expected_wheel_rpm,
                               result.active_program_window_s, result.can_path, diagnostics)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Raised-wheel HIL: one fixed manual web forward at +10 motor RPM")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_manual_web_forward(ManualWebForwardRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({"ok": True, "action": result.action, "command_motor_rpm": result.command_motor_rpm,
                      "expected_wheel_rpm": result.expected_wheel_rpm, "lease_fault": result.lease_fault,
                      "can_path": result.can_path, "diagnostics": result.diagnostics}, default=str))
    return 0


def main_reverse(argv: list[str] | None = None) -> int:
    """CLI-compatible fixed reverse runner, deliberately without action knobs."""
    parser = argparse.ArgumentParser(description="Raised-wheel HIL: fixed manual web reverse at -10 motor RPM for 5 seconds")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_manual_web_reverse(ManualWebForwardRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({"ok": True, "action": result.action, "command_motor_rpm": result.command_motor_rpm,
                      "expected_wheel_rpm": result.expected_wheel_rpm, "can_path": result.can_path,
                      "diagnostics": result.diagnostics}, default=str))
    return 0


def main_left(argv: list[str] | None = None) -> int:
    """CLI-compatible fixed left runner, deliberately without action knobs."""
    parser = argparse.ArgumentParser(description="Raised-wheel HIL: fixed manual web left at (-10, +10) motor RPM for 5 seconds")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_manual_web_left(ManualWebForwardRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({"ok": True, "action": result.action, "command_motor_rpm": result.command_motor_rpm,
                      "expected_wheel_rpm": result.expected_wheel_rpm, "can_path": result.can_path,
                      "diagnostics": result.diagnostics}, default=str))
    return 0


def main_right(argv: list[str] | None = None) -> int:
    """CLI-compatible fixed right runner, deliberately without action knobs."""
    parser = argparse.ArgumentParser(description="Raised-wheel HIL: fixed manual web right at (+10, -10) motor RPM for 5 seconds")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_manual_web_right(ManualWebForwardRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({"ok": True, "action": result.action, "command_motor_rpm": result.command_motor_rpm,
                      "expected_wheel_rpm": result.expected_wheel_rpm, "can_path": result.can_path,
                      "diagnostics": result.diagnostics}, default=str))
    return 0


def main_stop(argv: list[str] | None = None) -> int:
    """CLI entry point for the fixed active-command web STOP HIL check."""
    parser = argparse.ArgumentParser(description="Raised-wheel HIL: fixed forward command then existing web STOP")
    parser.add_argument("--slcan-device", required=True)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--confirm-physical-stop-tested", action="store_true")
    parser.add_argument("--confirm-wheels-raised", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_manual_web_stop(ManualWebForwardRequest(**vars(args)))
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:2000],
                          "diagnostics": getattr(exc, "diagnostics", {})}, default=str))
        return 2
    print(json.dumps({"ok": True, "action": result.action,
                      "active_command_motor_rpm": result.active_command_motor_rpm,
                      "expected_wheel_rpm": result.expected_wheel_rpm,
                      "active_program_window_s": result.active_program_window_s,
                      "can_path": result.can_path, "diagnostics": result.diagnostics}, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
