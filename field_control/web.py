"""Non-blocking diagnostics HTTP server with latest-frame MJPEG streams."""
from __future__ import annotations

import json
import math
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import uuid
from urllib.parse import parse_qsl, urlsplit

from .control import WheelCommand
from .diagnostics import status_payload
from .runtime import FieldControlRuntime
from .config_profiles import (default_profiles_dir, list_profiles, load_profile,
                              operator_profile_dict, save_profile, select_profile,
                              selected_profile)
from .config_io import runtime_config_from_dict
from dataclasses import asdict
from pathlib import Path


def _manual_rpm_from_query(query: str, *, default_rpm: float, max_rpm: float,
                           motor_turns_per_wheel_turn: float = 1.0) -> float:
    """Return a bounded motor-side RPM from one wheel-side HTTP input.

    Query parsing happens at the HTTP boundary, before a WheelCommand exists.
    Existing HIL callers without a query retain the configured manual speed;
    a browser-provided speed must be the sole, finite, positive ``rpm`` value.
    """
    try:
        parameters = parse_qsl(query, keep_blank_values=True, strict_parsing=True)
    except ValueError as exc:
        raise ValueError("ogiltig RPM-parameter") from exc
    if not parameters:
        rpm = default_rpm
    elif len(parameters) == 1 and parameters[0][0] == "rpm":
        try:
            rpm = float(parameters[0][1])
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("RPM måste vara ett ändligt tal") from exc
    else:
        raise ValueError("manuellt kommando kräver exakt en RPM-parameter")
    if not isinstance(rpm, (int, float)) or isinstance(rpm, bool) or not math.isfinite(rpm):
        raise ValueError("RPM måste vara ett ändligt tal")
    if rpm <= 0:
        raise ValueError("RPM måste vara positiv")
    if (not isinstance(max_rpm, (int, float)) or isinstance(max_rpm, bool)
            or not math.isfinite(max_rpm) or max_rpm <= 0):
        raise ValueError("RPM överskrider konfigurerad max_rpm")
    if not isinstance(motor_turns_per_wheel_turn, (int, float)) or not math.isfinite(motor_turns_per_wheel_turn) or motor_turns_per_wheel_turn <= 0:
        raise ValueError("ogiltig utväxling")
    # Query-less HIL callers remain backwards compatible and use motor RPM.
    motor_rpm = rpm if not parameters else rpm * motor_turns_per_wheel_turn
    if motor_rpm > max_rpm: raise ValueError("RPM överskrider konfigurerad max_rpm")
    return float(motor_rpm)


def _manual_session_and_rpm_from_query(query: str, *, default_rpm: float,
                                       max_rpm: float, motor_turns_per_wheel_turn: float) -> tuple[str, float]:
    """Parse one opaque browser session plus the optional wheel RPM."""
    try:
        parameters = parse_qsl(query, keep_blank_values=True, strict_parsing=True)
    except ValueError as exc:
        raise ValueError("ogiltig manuell parameter") from exc
    sessions = [value for key, value in parameters if key == "session"]
    others = [(key, value) for key, value in parameters if key != "session"]
    if len(sessions) != 1 or not sessions[0] or any(key != "rpm" for key, _value in others):
        raise ValueError("manuellt kommando kräver exakt en webbsession")
    rpm_query = "&".join(f"{key}={value}" for key, value in others)
    return sessions[0], _manual_rpm_from_query(
        rpm_query, default_rpm=default_rpm, max_rpm=max_rpm,
        motor_turns_per_wheel_turn=motor_turns_per_wheel_turn,
    )


def _manual_session_from_query(query: str) -> str:
    try:
        parameters = parse_qsl(query, keep_blank_values=True, strict_parsing=True)
    except ValueError as exc:
        raise ValueError("ogiltig manuell parameter") from exc
    if len(parameters) != 1 or parameters[0][0] != "session" or not parameters[0][1]:
        raise ValueError("manuellt hold kräver exakt en webbsession")
    return parameters[0][1]


def _manual_epoch_from_query(query: str) -> str:
    try:
        parameters = parse_qsl(query, keep_blank_values=True, strict_parsing=True)
    except ValueError as exc:
        raise ValueError("ogiltig manuell parameter") from exc
    if len(parameters) != 1 or parameters[0][0] != "epoch" or not parameters[0][1]:
        raise ValueError("manuell webbsession kräver exakt en freshness-epoch")
    return parameters[0][1]


class DiagnosticsServer:
    def __init__(self, runtime: FieldControlRuntime, host: str = "127.0.0.1", port: int = 8080,
                 *, profiles_dir: Path | None = None) -> None:
        self.runtime = runtime
        self.profiles_dir = profiles_dir or default_profiles_dir()
        # The handler factory captures this value.  It therefore has to exist
        # before ThreadingHTTPServer asks for its RequestHandlerClass.
        # A fresh value per server lets a browser distinguish a replacement
        # process even when the down interval is very short.
        self._instance_id = uuid.uuid4().hex
        self._server = ThreadingHTTPServer((host, port), self._handler())
        self._server.daemon_threads = True
        self._thread: threading.Thread | None = None
        # The HTTP handler only stages a restart after writing its response.
        # The CLI owns shutdown and process replacement, so an HTTP request
        # cannot bypass normal close/STOP behaviour.
        self._restart_requested = threading.Event()

    def restart_requested(self) -> bool:
        return self._restart_requested.is_set()

    @property
    def address(self) -> tuple[str, int]:
        return self._server.server_address

    def _handler(self):
        runtime = self.runtime
        owner = self
        instance_id = getattr(self, "_instance_id", "test-instance")
        # Some focused handler tests construct this object without __init__.
        # Retain that harmless test seam while production always sets it.
        profiles_dir = getattr(self, "profiles_dir", default_profiles_dir())

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                path = urlsplit(self.path).path
                if path == "/api/status":
                    body = json.dumps(self._status_value(), ensure_ascii=True).encode()
                    self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
                if path == "/api/config":
                    try:
                        body = json.dumps({"candidate": operator_profile_dict(runtime.config),
                                           "profiles": list_profiles(profiles_dir),
                                           "selected": selected_profile(profiles_dir),
                                           "apply_on_restart": True}, ensure_ascii=True).encode()
                    except (OSError, ValueError) as exc:
                        self._conflict(str(exc)); return
                    self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
                if path in ("/stream/raw", "/stream/overlay", "/stream/buds", "/stream/leaves", "/stream/marker"):
                    self._stream(path.rsplit("/", 1)[-1]); return
                if path in ("/snapshot/raw", "/snapshot/overlay", "/snapshot/buds", "/snapshot/leaves", "/snapshot/marker"):
                    self._snapshot(path.rsplit("/", 1)[-1]); return
                body = DASHBOARD_HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

            def do_POST(self) -> None:
                request = urlsplit(self.path)
                path = request.path
                if path == "/api/application/restart":
                    if request.query:
                        self._conflict("programomstart accepterar inga parametrar"); return
                    # This deliberately bypasses profile parsing and the
                    # configuration-restart reservation.  The CLI owns the
                    # following close/STOP attempts and process replacement;
                    # write the response first so a slow close cannot make
                    # the browser mistake an accepted restart for failure.
                    self._json({"restarting": True})
                    fence = getattr(runtime, "begin_application_restart", None)
                    # The fence is deliberately best-effort from HTTP's
                    # perspective: it latches any STOP failure itself, and
                    # an accepted process restart must still reach the CLI's
                    # close/exec owner.
                    event = getattr(owner, "_restart_requested", None)
                    try:
                        if callable(fence): fence()
                    finally:
                        if event is not None: event.set()
                    return
                if path in ("/api/config/save", "/api/config/select", "/api/config/restart"):
                    restart_reserved = False
                    try:
                        value = self._json_body()
                        if path == "/api/config/save":
                            if set(value) != {"candidate"}: raise ValueError("save kräver exakt candidate")
                            candidate = value["candidate"]
                            if not isinstance(candidate, dict): raise ValueError("candidate måste vara ett objekt")
                            # Merge through the normal strict parser. physical_can
                            # is supplied exclusively by this deployment.
                            merged = asdict(runtime.config); merged.update(candidate)
                            if "physical_can" in candidate: raise ValueError("physical_can får inte sparas")
                            config = runtime_config_from_dict(merged)
                            name = save_profile(config, profiles_dir)
                            self._json({"saved": name, "apply_on_restart": True}); return
                        if path == "/api/config/restart":
                            if set(value) != {"candidate"}: raise ValueError("restart kräver exakt candidate")
                            candidate = value["candidate"]
                            if not isinstance(candidate, dict): raise ValueError("candidate måste vara ett objekt")
                            if "physical_can" in candidate: raise ValueError("physical_can får inte sparas")
                            # All candidate parsing and strict cross-field
                            # validation completes before the reservation. A
                            # slow, malformed, or invalid client request can
                            # therefore never block local motor authority.
                            merged = asdict(runtime.config); merged.update(candidate)
                            config = runtime_config_from_dict(merged)
                            reserve = getattr(runtime, "reserve_configuration_restart", None)
                            restart_reserved = bool(reserve()) if callable(reserve) else False
                            if not restart_reserved:
                                raise ValueError("konfigurationsomstart kunde inte nå verifierat stopp")
                            name = save_profile(config, profiles_dir)
                            select_profile(name, profiles_dir)
                            # Send the complete response before signaling the
                            # CLI loop to perform its clean disarmed restart.
                            self._json({"selected": name, "restarting": True})
                            event = getattr(owner, "_restart_requested", None)
                            if event is not None: event.set()
                            return
                        if set(value) != {"name"} or not isinstance(value["name"], str):
                            raise ValueError("select kräver exakt filnamn")
                        # Validate selected profile before staging it.
                        load_profile(value["name"], runtime.config, profiles_dir)
                        select_profile(value["name"], profiles_dir)
                        self._json({"selected": value["name"], "apply_on_restart": True}); return
                    except (OSError, ValueError) as exc:
                        if restart_reserved:
                            cancel = getattr(runtime, "cancel_configuration_restart", None)
                            if callable(cancel): cancel()
                        self._conflict(str(exc)); return
                manual_directions = {
                    # WheelCommand is logical vehicle direction. The verified
                    # remote physical worker applies its configured per-motor
                    # forward signs exactly once when constructing raw A2.
                    "/api/manual/forward": (1.0, 1.0, "forward"),
                    "/api/manual/reverse": (-1.0, -1.0, "reverse"),
                    "/api/manual/left": (-1.0, 1.0, "left"),
                    "/api/manual/right": (1.0, -1.0, "right"),
                    # Individual-wheel routes use the same logical vehicle
                    # convention as the established remote-control UI.  Raw
                    # motor signs remain solely the verified motor-boundary's
                    # responsibility.
                    "/api/manual/left/forward": (1.0, 0.0, "left-forward"),
                    "/api/manual/left/reverse": (-1.0, 0.0, "left-reverse"),
                    "/api/manual/right/forward": (0.0, 1.0, "right-forward"),
                    "/api/manual/right/reverse": (0.0, -1.0, "right-reverse"),
                    "/api/manual/both/forward": (1.0, 1.0, "both-forward"),
                    "/api/manual/both/reverse": (-1.0, -1.0, "both-reverse"),
                }
                if path == "/api/manual/session":
                    try:
                        epoch = _manual_epoch_from_query(request.query)
                        begin = getattr(runtime, "begin_manual_web_session", None)
                        if not callable(begin):
                            raise RuntimeError("runtime saknar manuell webbsession")
                        self._json({"session": begin(epoch)})
                    except (ValueError, RuntimeError) as exc:
                        self._conflict(str(exc)); return
                    return
                if path == "/api/manual/hold":
                    try:
                        session = _manual_session_from_query(request.query)
                        command = getattr(runtime, "manual_web_command", None)
                        if not callable(command):
                            raise RuntimeError("runtime saknar manuell webbsession")
                        command(session, WheelCommand(0.0, 0.0, "web-manual-hold"))
                    except (ValueError, RuntimeError) as exc:
                        self._conflict(str(exc)); return
                    self._status(); return
                if path in manual_directions:
                    left, right, direction = manual_directions[path]
                    try:
                        session, rpm = _manual_session_and_rpm_from_query(
                            request.query,
                            default_rpm=runtime.config.manual_rpm,
                            max_rpm=runtime.config.max_rpm,
                            motor_turns_per_wheel_turn=getattr(getattr(runtime.config, "odometry_geometry", None),
                                                               "motor_turns_per_wheel_turn", 1.0),
                        )
                        command = getattr(runtime, "manual_web_command", None)
                        if not callable(command):
                            raise RuntimeError("runtime saknar manuell webbsession")
                        command(session, WheelCommand(left * rpm, right * rpm, f"web-manual-{direction}"))
                    except (ValueError, RuntimeError) as exc:
                        self._conflict(str(exc)); return
                    self._status(); return
                actions = {
                    "/api/manual": runtime.select_manual,
                    "/api/auto": runtime.select_auto,
                    "/api/start-auto": runtime.start_auto,
                    "/api/reset-row-progress": runtime.reset_row_progress,
                    "/api/stop": runtime.stop,
                }
                action = actions.get(path)
                if action is None:
                    self.send_error(HTTPStatus.NOT_FOUND); return
                try:
                    action()
                except (ValueError, RuntimeError) as exc:
                    self._conflict(str(exc)); return
                self._status()

            def _conflict(self, message: str) -> None:
                body = json.dumps({"error": message}, ensure_ascii=True).encode()
                self.send_response(HTTPStatus.CONFLICT); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

            def _json(self, value: object) -> None:
                body = json.dumps(value, ensure_ascii=True).encode()
                self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

            def _json_body(self) -> dict[str, object]:
                length = self.headers.get("Content-Length")
                if length is None: raise ValueError("Content-Length krävs")
                try: size = int(length)
                except ValueError as exc: raise ValueError("ogiltig Content-Length") from exc
                if not 2 <= size <= 65536: raise ValueError("ogiltig JSON-storlek")
                if self.headers.get("Content-Type", "").split(";", 1)[0].lower() != "application/json":
                    raise ValueError("application/json krävs")
                try: value = json.loads(self.rfile.read(size), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                except (UnicodeDecodeError, json.JSONDecodeError) as exc: raise ValueError("ogiltig JSON") from exc
                if not isinstance(value, dict): raise ValueError("JSON måste vara objekt")
                return value

            def _configuration_safe(self) -> bool:
                predicate = getattr(runtime, "configuration_restart_safe", None)
                return bool(predicate()) if callable(predicate) else False

            def _status(self) -> None:
                body = json.dumps(status_payload(runtime), ensure_ascii=True).encode()
                self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

            def _status_value(self) -> dict[str, object]:
                value = dict(status_payload(runtime))
                value["instance_id"] = instance_id
                return value

            def _stream(self, view: str) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache"); self.end_headers()
                interval = 1.0 / runtime.config.stream_fps
                try:
                    while not getattr(runtime, "_stop").is_set():
                        image = runtime.latest_image(view)
                        if image is not None:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(image)).encode() + b"\r\n\r\n" + image + b"\r\n")
                            self.wfile.flush()
                        runtime._stop.wait(interval)
                except (BrokenPipeError, ConnectionResetError):
                    return

            def _snapshot(self, view: str) -> None:
                """Return one latest-value JPEG without keeping a proxy-sensitive stream open."""
                image = runtime.latest_image(view)
                if image is None:
                    self.send_response(HTTPStatus.NO_CONTENT)
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Expires", "0")
                    self.end_headers()
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.send_header("Content-Length", str(len(image)))
                self.end_headers()
                self.wfile.write(image)

            def log_message(self, *_args) -> None:
                return

        return Handler

    def start(self) -> None:
        if self._thread and self._thread.is_alive(): return
        self._thread = threading.Thread(target=self._server.serve_forever, name="field-web", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._server.shutdown(); self._server.server_close()
        if self._thread: self._thread.join(timeout=2.0)


DASHBOARD_HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Field Control Diagnostics</title>
<style>
:root{font-family:ui-sans-serif,system-ui,sans-serif;color:#17212b;background:#eef2f3;--ink:#17212b;--muted:#65727b;--line:#cbd5d8;--accent:#d9573f;--ok:#197a5d}
*{box-sizing:border-box}body{margin:0}.top{background:#173d3a;color:white;padding:20px 5vw;display:flex;justify-content:space-between;align-items:center;gap:20px}.top h1{margin:0;font-size:22px}.top span{color:#b9d5cb;font-size:13px}main{max-width:1720px;margin:24px auto;padding:0 20px;overflow-x:hidden}button,input,select{font:inherit;min-width:0}button{border:0;border-radius:5px;padding:11px 16px;font-weight:700;cursor:pointer;color:white;background:#173d3a}button.stop{background:var(--accent)}button:disabled{opacity:.45;cursor:not-allowed}.control-layout{display:grid;grid-template-columns:minmax(270px,320px) minmax(390px,500px) minmax(260px,1fr);gap:16px;align-items:start}.mode-actions,.manual-controls{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.mode-actions button:first-child{grid-column:span 2}.speed-input,.direct-config input,.direct-config select,.config-fields input{width:100%;margin:4px 0 10px;padding:7px;border:1px solid var(--line);border-radius:5px}.direct-config{margin:12px 0}.direct-config label,.panel>label{display:block;color:var(--muted);font-size:12px}.direct-config .check-row{display:flex;align-items:center;gap:7px}.direct-config .check-row input{width:auto;margin:0}.manual-controls{margin:12px 0}.manual-controls button{min-height:52px}.manual-controls .reverse{background:#8d3b3b}.stop{width:100%;min-height:52px}.panel{background:white;border:1px solid var(--line);border-radius:6px;padding:16px;min-width:0}.panel h2{font-size:15px;margin:0 0 12px;color:#173d3a}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.metric{border-top:1px solid #e5eaeb;padding-top:8px}.label{display:block;color:var(--muted);font-size:12px}.value{font-size:18px;font-weight:700;overflow-wrap:anywhere}.config-sections{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.config-section{border:1px solid #dce5e7;border-radius:6px;padding:10px;background:#f8faf9;min-width:0}.config-section h3{margin:0;color:#173d3a;font-size:13px}.config-section p{margin:3px 0 9px;color:var(--muted);font-size:10px}.config-fields{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:5px 8px}.config-fields label{font-size:10px;color:var(--muted);overflow-wrap:anywhere}.config-fields input{margin:2px 0;padding:5px;font-size:12px}.config-profile-actions{margin-top:14px;display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;align-items:end}.config-profile-actions label{font-size:11px;color:var(--muted)}.config-profile-actions button,.config-profile-actions select{width:100%}.compact-status{font-size:11px}.compact-status h3{font-size:12px;color:#173d3a;margin:12px 0 3px}.compact-status h3:first-of-type{margin-top:0}.compact-status .grid{gap:5px}.compact-status .metric{padding-top:4px}.compact-status .label{font-size:10px}.compact-status .value{font-size:13px}.ok{color:var(--ok)}.bad{color:var(--accent)}.streams{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.streams img{width:100%;aspect-ratio:4/3;object-fit:contain;background:#17212b;border-radius:4px}.streams h3{font-size:13px;margin:0 0 6px}.legend{display:flex;flex-wrap:wrap;gap:4px 10px;margin:0 0 6px;font-size:11px;color:var(--muted)}.legend span{display:inline-flex;align-items:center;gap:4px}.swatch{width:11px;height:11px;border-radius:2px;display:inline-block}.fault{color:var(--accent);min-height:20px;font-weight:700}@media(max-width:1180px){.control-layout{grid-template-columns:minmax(270px,330px) minmax(360px,1fr)}.compact-status{grid-column:span 2}.config-fields{grid-template-columns:repeat(2,minmax(0,1fr))}}@media(max-width:760px){.control-layout,.streams,.config-sections,.config-profile-actions{grid-template-columns:1fr}.compact-status{grid-column:auto}.top{align-items:flex-start;flex-direction:column}}
</style></head><body><header class="top"><div><h1>Field Control</h1><span>Runtime diagnostics</span></div><strong id="state">Loading</strong></header>
<main><div class="control-layout">
<aside class="panel" aria-label="Control"><h2>Control</h2><div class="mode-actions"><button id="manual-mode" onclick="post('/api/manual')">MANUAL</button><button onclick="post('/api/auto')">AUTO</button><button onclick="post('/api/start-auto')">START AUTO</button></div><div class="direct-config"><label>Row following<select data-direct="vision.navigation_mode"><option value="buds_only">Buds only</option><option value="buds_and_leaves">Buds + leaves</option></select></label><label>Max IMU-only navigation (m)<input data-direct="safety.search_length_m" type="number" min="0" step="0.1"></label><label>Pick trigger timeout (s)<input data-direct="safety.max_pick_wait_s" type="number" min="0" step="0.1"></label><label class="check-row"><input data-direct="safety.in_row_turn_enabled" type="checkbox"><span>In-row turn enabled</span></label><label>New-row direction<select data-direct="safety.new_row_turn_direction"><option value="left">Left</option><option value="right">Right</option></select></label><label>Rows to harvest<input data-direct="safety.number_of_rows" type="number" min="1" step="1"></label></div><label class="label" for="rpm">Desired speed (wheel RPM)</label><input class="speed-input" id="rpm" type="number" min="0.01" step="0.1" inputmode="decimal" disabled><div class="manual-controls" aria-label="Manual drive controls"><button data-manual-path="/api/manual/both/forward">BOTH FORWARD</button><button class="reverse" data-manual-path="/api/manual/both/reverse">BOTH REVERSE</button><button data-manual-path="/api/manual/left/forward">LEFT FORWARD</button><button data-manual-path="/api/manual/right/forward">RIGHT FORWARD</button><button class="reverse" data-manual-path="/api/manual/left/reverse">LEFT REVERSE</button><button class="reverse" data-manual-path="/api/manual/right/reverse">RIGHT REVERSE</button></div><button class="stop" id="stop">STOP</button><p id="manual-help">Select MANUAL before using a direction button. Hold a manual button to drive. The browser refreshes the bounded control lease every 100 ms and sends STOP when released. These buttons never arm motors.</p></aside>
<aside class="panel" aria-label="Configuration"><h2>Configuration</h2><p id="config-note" class="label">Changes apply on restart. Motor boundary acceleration is fixed and verified.</p><div id="config-fields" class="config-sections"></div><div class="config-profile-actions"><button id="save-config">Save configuration</button><label>Load saved configuration<select id="profile-select"></select></label><div><button id="select-config">Use selected on next restart</button><button id="restart-config">Reload/restart from configuration</button></div></div></aside>
<aside class="panel compact-status" aria-label="Runtime, sensors, heading, vision and odometry"><h2>Runtime and navigation</h2><h3>Runtime</h3><div class="grid"><div class="metric"><span class="label">Mode</span><span class="value" id="mode">-</span></div><div class="metric"><span class="label">State</span><span class="value" id="state2">-</span></div><div class="metric"><span class="label">Row / pass</span><span class="value" id="row">-</span></div><div class="metric"><span class="label">Motor output</span><span class="value" id="armed">-</span></div><div class="metric"><span class="label">Local web standby</span><span class="value" id="standby">-</span></div></div><p class="fault" id="fault"></p><h3>Sensors</h3><div class="grid"><div class="metric"><span class="label">Camera</span><span class="value" id="camera">-</span></div><div class="metric"><span class="label">IMU</span><span class="value" id="imu">-</span></div><div class="metric"><span class="label">Camera age</span><span class="value" id="camera-age">-</span></div><div class="metric"><span class="label">IMU age</span><span class="value" id="imu-age">-</span></div></div><h3>Heading</h3><div class="grid"><div class="metric"><span class="label">Filtered</span><span class="value" id="heading">-</span></div><div class="metric"><span class="label">Row reference</span><span class="value" id="reference">-</span></div><div class="metric"><span class="label">Error</span><span class="value" id="heading-error">-</span></div><div class="metric"><span class="label">Reference build distance</span><span class="value" id="build-distance">-</span></div></div><h3>Vision and odometry</h3><div class="grid"><div class="metric"><span class="label">Target x / goal</span><span class="value" id="target">-</span></div><div class="metric"><span class="label">Marker</span><span class="value" id="marker">-</span></div><div class="metric"><span class="label">Distance</span><span class="value" id="distance">-</span></div><div class="metric"><span class="label">Search distance</span><span class="value" id="search">-</span></div></div><p class="fault" id="odometry-warning"></p></aside></div>
<section class="panel" style="margin-top:16px"><h2>Live views</h2><div class="streams"><div><h3>Original</h3><div class="legend" aria-label="Zone colours"><span><i class="swatch" style="background:#00b4ff"></i>Navigation 1</span><span><i class="swatch" style="background:#00ffff"></i>Navigation 2</span><span><i class="swatch" style="background:#ffff00"></i>Trigger 1</span><span><i class="swatch" style="background:#b4b400"></i>Trigger 2</span><span><i class="swatch" style="background:#ff00ff"></i>Pick 1</span><span><i class="swatch" style="background:#ff00b4"></i>Pick 2</span><span><i class="swatch" style="background:#ffb400"></i>Turn marker</span><span><i class="swatch" style="background:#ff0000"></i>x goals</span></div><img data-snapshot-view="raw" alt="Original camera"></div><div><h3>Overlay</h3><img data-snapshot-view="overlay" alt="Vision overlay"></div><div><h3>Buds mask</h3><img data-snapshot-view="buds" alt="Buds mask"></div><div><h3>Leaves mask</h3><img data-snapshot-view="leaves" alt="Leaves mask"></div></div></section></main>
<script>
const text=(id,value)=>document.getElementById(id).textContent=value??'-';
const fmt=(value,suffix='')=>value===null||value===undefined?'-':Number(value).toFixed(2)+suffix;
const rpmInput=document.getElementById('rpm');
const dashboardMain=document.querySelector('main'),controlLayout=document.querySelector('.control-layout');
const configurationPanel=document.querySelector('[aria-label="Configuration"]'),liveViews=document.querySelector('section.panel');
const tabBar=document.createElement('nav'),controlTab=document.createElement('div'),configurationTab=document.createElement('div');
tabBar.className='dashboard-tabs';controlTab.className='tab-pane active';configurationTab.className='tab-pane';
const controlTabButton=document.createElement('button'),configurationTabButton=document.createElement('button');
controlTabButton.textContent='Control and live views';configurationTabButton.textContent='Configuration';
tabBar.append(controlTabButton,configurationTabButton);dashboardMain.insertBefore(tabBar,controlLayout);dashboardMain.insertBefore(controlTab,controlLayout);
controlTab.append(controlLayout,liveViews);dashboardMain.append(configurationTab);configurationTab.append(configurationPanel);
function selectDashboardTab(configuration){controlTab.classList.toggle('active',!configuration);configurationTab.classList.toggle('active',configuration);controlTabButton.classList.toggle('active',!configuration);configurationTabButton.classList.toggle('active',configuration);}
controlTabButton.addEventListener('click',()=>selectDashboardTab(false));configurationTabButton.addEventListener('click',()=>selectDashboardTab(true));
document.head.insertAdjacentHTML('beforeend','<style>.dashboard-tabs{display:flex;gap:8px;margin-bottom:12px}.dashboard-tabs button{background:#65727b}.dashboard-tabs button.active{background:#173d3a}.tab-pane{display:none}.tab-pane.active{display:block}.tab-pane .control-layout{margin:0}.tab-pane .streams img{max-height:420px}@media(min-width:1181px){.tab-pane.active{display:grid;grid-template-columns:minmax(240px,.8fr) minmax(330px,1fr) minmax(420px,1.35fr);gap:16px;align-items:start}.tab-pane .control-layout{display:contents}.tab-pane .compact-status{grid-column:1/-1;grid-row:1}.tab-pane .panel[aria-label="Control"]{grid-column:1;grid-row:2;padding:12px}.tab-pane>section.panel{grid-column:2/-1;grid-row:2;margin-top:0!important}.tab-pane>section.panel .streams img{max-height:420px}.tab-pane .panel[aria-label="Configuration"]{grid-column:1/-1}}</style>');
const controlActions=document.querySelector('.mode-actions');
const restartApplicationButton=document.createElement('button');
restartApplicationButton.id='restart-application';
restartApplicationButton.textContent='Restart application';
const resetRowProgressButton=document.createElement('button');
resetRowProgressButton.id='reset-row-progress';
resetRowProgressButton.textContent='Reset row/pass';
controlActions.after(restartApplicationButton,resetRowProgressButton);
document.querySelector('section.panel h2').insertAdjacentHTML('afterend','<p class="label" id="navigation-state">Loading navigation state…</p>');
rpmInput.previousElementSibling.textContent='Manual speed (wheel RPM)';
rpmInput.insertAdjacentHTML('afterend','<label class="label" for="auto-rpm">Auto speed (wheel RPM, applies on restart)</label><input class="speed-input" id="auto-rpm" data-staged-rpm="auto_base_rpm" type="number" min="0.01" step="0.1" inputmode="decimal"><label class="label" for="turn-rpm">Turn speed (wheel RPM, applies on restart)</label><input class="speed-input" id="turn-rpm" data-staged-rpm="turn_speed_rpm" type="number" min="0.01" step="0.1" inputmode="decimal"><label class="label" for="turn-timeout">Turn timeout (s, applies on restart)</label><input class="speed-input" id="turn-timeout" data-staged-value="safety.turn_timeout_s" type="number" min="0.1" step="0.1" inputmode="decimal">');
const stagedSpeedRow=document.createElement('div'),manualSpeedLabel=document.querySelector('label[for="rpm"]');stagedSpeedRow.className='staged-speed-row';rpmInput.parentNode.insertBefore(stagedSpeedRow,manualSpeedLabel);for(const [heading,id] of [['Manual','rpm'],['Auto','auto-rpm'],['Turn','turn-rpm']]){const input=document.getElementById(id),label=document.querySelector(`label[for="${id}"]`),cell=document.createElement('div');cell.innerHTML=`<h3>${heading}</h3>`;cell.append(label,input);if(id==='turn-rpm'){const timeout=document.getElementById('turn-timeout'),timeoutLabel=document.querySelector('label[for="turn-timeout"]');cell.append(timeoutLabel,timeout);}stagedSpeedRow.append(cell);}
document.head.insertAdjacentHTML('beforeend','<style>.staged-speed-row{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:6px;margin:8px 0}.staged-speed-row h3{margin:0 0 2px;font-size:12px;color:#173d3a}.staged-speed-row label{font-size:10px}.staged-speed-row .speed-input{margin:2px 0 5px}@media(max-width:760px){.staged-speed-row{grid-template-columns:1fr}}</style>');
document.head.insertAdjacentHTML('beforeend','<style>.compact-status{padding:9px 12px}.compact-status h2{display:inline;margin:0 10px 0 0;font-size:13px}.compact-status h3{display:inline;margin:0 5px 0 10px;font-size:10px}.compact-status .grid{display:inline-grid;grid-template-columns:repeat(5,minmax(62px,1fr));gap:3px;vertical-align:middle}.compact-status .metric{display:inline-block;border:0;padding:0 4px}.compact-status .label{font-size:8px}.compact-status .value{font-size:11px}.compact-status .fault{display:inline;margin:0 4px;font-size:9px;min-height:0}@media(max-width:1180px){.compact-status h3{display:block;margin:7px 0 2px}.compact-status .grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr))}}</style>');
document.head.insertAdjacentHTML('beforeend','<style>.config-section[data-config-group="rows"]{grid-column:1/-1}.row-zone-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.row-zone-column{border:1px solid #dce5e7;border-radius:5px;padding:8px;background:#fff;min-width:0}.row-zone-column h4{margin:0 0 6px;color:#173d3a;font-size:12px}.row-zone-goal{display:grid;grid-template-columns:1fr;gap:5px}.row-zone-goal label{display:block;font-size:10px;color:var(--muted);overflow-wrap:anywhere}.row-zone-goal input{width:100%;margin:2px 0;padding:5px;border:1px solid var(--line);border-radius:5px;font-size:12px}.row-zone-group{border-top:1px solid #e5eaeb;margin-top:7px;padding-top:6px}.row-zone-group h5,.shared-turn-marker h4{margin:0 0 3px;color:#65727b;font-size:10px}.row-zone-inputs,.shared-turn-marker .config-fields{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:5px}.row-zone-inputs label,.shared-turn-marker label{font-size:10px;color:var(--muted);overflow-wrap:anywhere}.row-zone-inputs input,.shared-turn-marker input{width:100%;margin:2px 0;padding:5px;border:1px solid var(--line);border-radius:5px;font-size:12px}.shared-turn-marker{border-top:1px solid #dce5e7;margin-top:10px;padding-top:8px}.config-validation-warning{grid-column:1/-1;margin:0;color:var(--accent);font-size:11px;font-weight:700}@media(max-width:760px){.row-zone-grid{grid-template-columns:1fr}}</style>');
// Compact controls are shared by both tabs.  The old selector made ``main``
// itself a desktop grid and positioned direct children, which is invalid once
// the panels are deliberately moved below their tab panes.
document.head.insertAdjacentHTML('beforeend','<style>@media(min-width:1181px){.panel[aria-label="Control"] .mode-actions button,.panel[aria-label="Control"] .manual-controls button{min-height:38px;padding:7px 9px;font-size:12px}.panel[aria-label="Control"] .speed-input{margin:2px 0 7px;padding:5px;font-size:12px}.streams{grid-template-columns:1fr 1fr}.streams img{max-height:220px}}</style>');
// The status endpoint reports motor-side RPM while this control deliberately
// displays wheel-side RPM.  Do not initialise the control until the selected
// profile has supplied its verified motor-to-wheel ratio.
let speedInitialized=false, profileCandidate=null, gearRatio=null, manualSpeedStatus=null, dashboardInstanceId=null;
const directPaths=['auto_base_rpm','turn_speed_rpm','safety.turn_timeout_s','vision.navigation_mode','safety.search_length_m','safety.max_pick_wait_s','safety.in_row_turn_enabled','safety.new_row_turn_direction','safety.number_of_rows'];
const goalRelativeZoneMigrationTolerance=1e-6;
function atPath(value,path){return path.split('.').reduce((v,k)=>v?.[k],value)}
function setPath(value,path,next){const keys=path.split('.');let target=value;for(const key of keys.slice(0,-1))target=target[key];target[keys.at(-1)]=next;}
function leafEntries(value,prefix=''){if(Array.isArray(value))return [[prefix,value.join(',')]];if(value&&typeof value==='object')return Object.entries(value).flatMap(([key,item])=>leafEntries(item,prefix?`${prefix}.${key}`:key));return [[prefix,value]];}
function isRpmPath(path){return path.endsWith('_rpm');}
function configLabel(path){const match=path.match(/^vision\\.(navigation|trigger|pick)_zone(_2)?\\.x_distance$/);if(match)return `${match[1]}${match[2]?' 2':' 1'} x distance from x goal`;if(path==='vision.x_goal')return 'x_goal_1';if(path==='vision.x_goal_2')return 'x_goal_2';return isRpmPath(path)?`${path} (wheel RPM)`:path;}
const configGroups=[
  ['rows','Radmål och zoner','x-goal samt navigations-, trigger- och pick-zoner för båda raderna.'],
  ['vision','Bild, HSV och perspektiv','Bildutsnitt, markbredd, HSV-filter och detektion.'],
  ['navigation','Navigation och reglering','Syn-, heading- och IMU-reglering samt fart för sökning.'],
  ['harvest','Skörd och vändningar','Radslut, väntetider och parametrar för vändmanövrer.'],
  ['odometry','Odometri och geometri','Hjul, utväxling och robotens geometri.'],
  ['camera','Kamera och bildström','Bearbetningsformat, sensortimeout och direktsändning.'],
  ['general','Allmänt och gränser','Övriga driftparametrar och hastighetsgränser.'],
];
function configGroupForPath(path){
  if(/^vision\\.(x_goal(?:_2)?|navigation_zone(?:_2)?|trigger_zone(?:_2)?|pick_zone(?:_2)?|turn_marker_zone)/.test(path))return 'rows';
  if(/^vision\\.(buds|leaves|marker|first_crop|x_goal_top|ground_width_)/.test(path))return 'vision';
  if(/^(vision\\.(x_filter|x_outlier)|vision_(kp|deadband_px)|max_vision_correction_rpm|heading_|imu_|row_heading_window_m|heading_reference_min_distance_m|search_speed_rpm)/.test(path))return 'navigation';
  if(/^(safety\\.|row_spacing_m)/.test(path))return 'harvest';
  if(/^(odometry_geometry\\.|odometry_timeout_s)/.test(path))return 'odometry';
  if(/^(processing_|navigation_frame_rate_hz|camera_timeout_s|stream_|jpeg_quality)/.test(path))return 'camera';
  return 'general';
}
function migrateCentredLegacyZones(candidate){const next=structuredClone(candidate),vision=next.vision,goal=Number(vision?.x_goal);if(!Number.isFinite(goal))return next;for(const name of ['navigation','trigger','pick']){const key=`${name}_zone`,zone=vision[key];if(!zone||typeof zone!=='object'||'x_distance'in zone||!['x_min','x_max','y_min','y_max'].every(field=>typeof zone[field]==='number'))continue;const midpoint=(zone.x_min+zone.x_max)/2;if(Math.abs(midpoint-goal)>goalRelativeZoneMigrationTolerance)continue;vision[key]={x_distance:(zone.x_max-zone.x_min)/2,y_min:zone.y_min,y_max:zone.y_max};}return next;}
function rowZoneSlot(path,rowZones){const goal=path.match(/^vision\\.x_goal(_2)?$/);if(goal)return rowZones[goal[1]?1:0].goal;const zone=path.match(/^vision\\.(navigation|pick|trigger)_zone(_2)?\\.(.+)$/);if(!zone)return null;return rowZones[zone[2]?1:0].zones[zone[1]];}
function rowZoneLabel(path){const leaf=path.split('.').at(-1);return leaf==='x_goal'?'x_goal_1':leaf==='x_goal_2'?'x_goal_2':leaf;}
function renderConfig(candidate){candidate=migrateCentredLegacyZones(candidate);profileCandidate=candidate;const ratio=Number(candidate.odometry_geometry?.motor_turns_per_wheel_turn);gearRatio=Number.isFinite(ratio)&&ratio>0?ratio:null;document.querySelectorAll('[data-direct]').forEach(el=>{const value=atPath(candidate,el.dataset.direct);if(el.type==='checkbox')el.checked=Boolean(value);else el.value=value??'';});const root=document.getElementById('config-fields');root.textContent='';const groups=new Map(configGroups.map(([id,title,description])=>{const section=document.createElement('section'),heading=document.createElement('h3'),note=document.createElement('p'),fields=document.createElement('div');section.className='config-section';section.dataset.configGroup=id;heading.textContent=title;note.textContent=description;if(id==='rows'){const rows=[];fields.className='row-zone-grid';for(const rowTitle of ['Rad 1','Rad 2']){const column=document.createElement('section'),columnHeading=document.createElement('h4'),goal=document.createElement('div'),zones={};column.className='row-zone-column';columnHeading.textContent=rowTitle;goal.className='row-zone-goal';column.append(columnHeading,goal);for(const [zoneName,zoneTitle] of [['navigation','navigation_boundaries'],['pick','pick_boundaries'],['trigger','trigger_boundaries']]){const zone=document.createElement('section'),zoneHeading=document.createElement('h5'),inputs=document.createElement('div');zone.className='row-zone-group';zoneHeading.textContent=zoneTitle;inputs.className='row-zone-inputs';zone.append(zoneHeading,inputs);column.append(zone);zones[zoneName]=inputs;}fields.append(column);rows.push({goal,zones});}const shared=document.createElement('section'),sharedHeading=document.createElement('h4'),sharedFields=document.createElement('div');shared.className='shared-turn-marker';sharedHeading.textContent='Shared turn marker zone';sharedFields.className='config-fields';shared.append(sharedHeading,sharedFields);section.append(heading,note,fields,shared);root.append(section);return [id,{rows,shared:sharedFields}];}fields.className='config-fields';section.append(heading,note,fields);root.append(section);return [id,fields];}));const leftCircumference=Number(candidate.odometry_geometry?.left_wheel_circumference_m),rightCircumference=Number(candidate.odometry_geometry?.right_wheel_circumference_m),circumferencesMatch=Number.isFinite(leftCircumference)&&leftCircumference>0&&leftCircumference===rightCircumference;const circumferenceLabel=document.createElement('label'),circumferenceInput=document.createElement('input');circumferenceLabel.textContent='wheel_circumference_m';circumferenceInput.dataset.sharedWheelCircumference='true';circumferenceInput.name='wheel_circumference_m';circumferenceInput.type='number';circumferenceInput.min='0';circumferenceInput.step='any';if(circumferencesMatch)circumferenceInput.value=String(leftCircumference);else{circumferenceInput.placeholder='Set a common positive value';circumferenceLabel.title='Left and right wheel circumferences differ. Enter one common value before saving.';}circumferenceLabel.append(circumferenceInput);const odometryFields=groups.get('odometry');odometryFields.append(circumferenceLabel);if(!circumferencesMatch){const warning=document.createElement('p');warning.className='config-validation-warning';warning.textContent='Left and right wheel circumferences differ. Enter wheel_circumference_m to use one common value before saving.';odometryFields.append(warning);}for(const [path,value] of leafEntries(candidate)){if(directPaths.includes(path)||path==='odometry_geometry.left_wheel_circumference_m'||path==='odometry_geometry.right_wheel_circumference_m')continue;const groupId=configGroupForPath(path),rows=groupId==='rows'?groups.get('rows'):null,slot=rows?rowZoneSlot(path,rows.rows):null;const label=document.createElement('label');label.textContent=slot?rowZoneLabel(path):configLabel(path);const input=document.createElement('input');input.dataset.path=path;input.value=value===null?'':String(isRpmPath(path)?Number(value)/gearRatio:value);input.type=typeof value==='boolean'?'checkbox':'text';if(input.type==='checkbox')input.checked=value;label.append(input);if(rows){if(slot)slot.append(label);else rows.shared.append(label);}else groups.get(groupId).append(label);}if(manualSpeedStatus)configureManualSpeed(manualSpeedStatus);}
function candidateFromForm(){const candidate=structuredClone(profileCandidate);document.querySelectorAll('[data-direct]').forEach(el=>setPath(candidate,el.dataset.direct,el.type==='checkbox'?el.checked:(el.type==='number'?Number(el.value):el.value)));document.querySelectorAll('#config-fields input[data-path]').forEach(el=>{const old=atPath(candidate,el.dataset.path);let value=el.type==='checkbox'?el.checked:el.value;if(Array.isArray(old))value=value.split(',').map(Number);else if(typeof old==='number')value=Number(value);else if(old===null&&value==='')value=null;if(isRpmPath(el.dataset.path))value*=gearRatio;setPath(candidate,el.dataset.path,value);});const sharedWheelCircumference=document.querySelector('[data-shared-wheel-circumference]'),wheelCircumference=Number(sharedWheelCircumference?.value);if(!Number.isFinite(wheelCircumference)||wheelCircumference<=0)throw new Error('wheel_circumference_m must be a positive common value for both wheels');candidate.odometry_geometry.left_wheel_circumference_m=wheelCircumference;candidate.odometry_geometry.right_wheel_circumference_m=wheelCircumference;return candidate;}
const candidateFromFormBase=candidateFromForm;
candidateFromForm=()=>{const candidate=candidateFromFormBase();document.querySelectorAll('[data-staged-rpm]').forEach(el=>{const wheelRpm=Number(el.value);if(!Number.isFinite(wheelRpm)||wheelRpm<=0)throw new Error('Staged wheel RPM must be positive');candidate[el.dataset.stagedRpm]=wheelRpm*gearRatio;});document.querySelectorAll('[data-staged-value]').forEach(el=>{const value=Number(el.value);if(!Number.isFinite(value)||value<=0)throw new Error('Staged value must be positive');setPath(candidate,el.dataset.stagedValue,value);});return candidate;};
const renderConfigBase=renderConfig;
renderConfig=candidate=>{renderConfigBase(candidate);document.querySelectorAll('[data-staged-rpm]').forEach(el=>{const motorRpm=Number(candidate[el.dataset.stagedRpm]);el.value=Number.isFinite(motorRpm)&&gearRatio?String(motorRpm/gearRatio):'';});document.querySelectorAll('[data-staged-value]').forEach(el=>{const value=atPath(candidate,el.dataset.stagedValue);el.value=value??'';});};
async function loadConfig(){try{const response=await fetch('/api/config',{cache:'no-store'});if(!response.ok)throw new Error();const data=await response.json();renderConfig(data.candidate);const select=document.getElementById('profile-select');select.textContent='';for(const name of data.profiles){const option=document.createElement('option');option.value=name;option.textContent=name;if(name===data.selected)option.selected=true;select.append(option);}}catch(_){text('config-note','Configuration unavailable');}}
document.getElementById('save-config').addEventListener('click',async()=>{try{const response=await fetch('/api/config/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({candidate:candidateFromForm()})});const data=await response.json();if(!response.ok)throw new Error(data.error);text('config-note',`Saved ${data.saved}; applies on restart`);loadConfig();}catch(error){text('config-note',`Save failed: ${error.message}`);}});
document.getElementById('select-config').addEventListener('click',async()=>{try{const response=await fetch('/api/config/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:document.getElementById('profile-select').value})});const data=await response.json();if(!response.ok)throw new Error(data.error);text('config-note',`Selected ${data.selected}; applies on restart`);}catch(error){text('config-note',`Selection failed: ${error.message}`);}});
const restartProbeDelayMs=250, restartProbeTimeoutMs=1000, restartReconnectDeadlineMs=90000;
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function restartStatus(){
  const controller=new AbortController();
  const timeout=setTimeout(()=>controller.abort(),restartProbeTimeoutMs);
  try{const response=await fetch('/api/status',{cache:'no-store',signal:controller.signal});return response.ok?await response.json():null;}
  catch(_){return null;}
  finally{clearTimeout(timeout);}
}
async function reconnectAfterRestart(previousInstanceId){
  const deadline=Date.now()+restartReconnectDeadlineMs;
  while(Date.now()<deadline){
    const status=await restartStatus();
    if(!status){text('config-note','Restarting: waiting for the new application…');}
    else if(status.instance_id!==previousInstanceId){
      window.location.reload();
      return;
    }
    await pause(restartProbeDelayMs);
  }
  text('config-note','Restart is taking longer than expected. Reload the page when the application is available.');
  document.getElementById('restart-config').disabled=false;
  restartApplicationButton.disabled=false;
}
document.getElementById('restart-config').addEventListener('click',async()=>{try{const button=document.getElementById('restart-config');button.disabled=true;const previousStatus=dashboardInstanceId?{instance_id:dashboardInstanceId}:await restartStatus();if(!previousStatus?.instance_id)throw new Error('Could not identify the running application');const response=await fetch('/api/config/restart',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({candidate:candidateFromForm()})});const data=await response.json();if(!response.ok)throw new Error(data.error);text('config-note',`Restarting with ${data.selected}; reconnecting automatically…`);void reconnectAfterRestart(previousStatus.instance_id);}catch(error){text('config-note',`Restart failed: ${error.message}`);document.getElementById('restart-config').disabled=false;}});
function configureManualSpeed(status){
  manualSpeedStatus=status;
  if(!Number.isFinite(gearRatio)||gearRatio<=0)return;
  const configuredRpm=Number(status.manual_rpm)/gearRatio, maxRpm=Number(status.max_rpm)/gearRatio;
  if(!Number.isFinite(configuredRpm)||!Number.isFinite(maxRpm)||maxRpm<=0)return;
  rpmInput.max=String(maxRpm);
  rpmInput.disabled=false;
  // Do not overwrite a value the operator has edited during later status
  // refreshes.  Configuration only establishes the initial safe default.
  if(!speedInitialized){rpmInput.value=String(configuredRpm);speedInitialized=true;}
}
function manualRequestPath(path){
  if(typeof manual.session!=='string'||!manual.session)throw new Error('Manual session is unavailable');
  const parameters=new URLSearchParams({session:manual.session});
  if(path==='/api/manual/hold')return `${path}?${parameters}`;
  const rpm=Number(rpmInput.value), maxRpm=Number(rpmInput.max);
  if(!Number.isFinite(rpm)||rpm<=0||!Number.isFinite(maxRpm)||maxRpm<=0||rpm>maxRpm){throw new Error('RPM must be positive and no greater than configured maximum');}
  parameters.set('rpm',rpm);return `${path}?${parameters}`;
}
async function post(path){try{const r=await fetch(path,{method:'POST'});const p=await r.json();if(!r.ok) alert(p.error||'Command rejected');return r.ok}catch(e){alert('Request failed');return false}}
// Manual HTTP delivery deliberately has one request in flight. Slow or lost
// browser requests therefore cannot become an unbounded queue of stale drive
// commands. Release cancels the periodic sender and sends one STOP
// immediately. The server invalidates the session before STOP returns, so an
// already-sent direction with that session cannot reclaim output afterwards.
// Runtime-side lease expiry remains the independent backstop on connection loss.
restartApplicationButton.addEventListener('click',async()=>{try{restartApplicationButton.disabled=true;const previousStatus=dashboardInstanceId?{instance_id:dashboardInstanceId}:await restartStatus();if(!previousStatus?.instance_id)throw new Error('Could not identify the running application');const response=await fetch('/api/application/restart',{method:'POST'});const data=await response.json();if(!response.ok)throw new Error(data.error);text('config-note','Restarting application; reconnecting automatically…');void reconnectAfterRestart(previousStatus.instance_id);}catch(error){text('config-note',`Restart failed: ${error.message}`);restartApplicationButton.disabled=false;}});
resetRowProgressButton.addEventListener('click',async()=>{if(await post('/api/reset-row-progress'))refresh();});
const manual={active:false,path:null,pointerId:null,session:null,epoch:null,starting:false,startGeneration:0,timer:null,inFlight:false,controller:null,stopping:false,pendingStart:null};
function clearManualTimer(){if(manual.timer!==null){clearInterval(manual.timer);manual.timer=null;}}
function observeManualEpoch(status){const epoch=status?.physical_web_standby?.manual_web_epoch;if(typeof epoch==='string'&&epoch)manual.epoch=epoch;}
function sendManualStop(){if(manual.stopping)return;manual.stopping=true;fetch('/api/stop',{method:'POST'}).then(async r=>{const p=await r.json().catch(()=>({}));if(!r.ok)throw new Error(p.error||'STOP rejected');observeManualEpoch(p);}).catch(()=>{text('fault','Manual STOP request failed; runtime lease will stop output')}).finally(()=>{manual.stopping=false;const pending=manual.pendingStart;manual.pendingStart=null;if(pending){refresh().finally(()=>holdManual(pending.path,pending.pointerId));}});}
function stopManual(){const hadSession=manual.active||manual.starting||manual.inFlight||manual.pointerId!==null||manual.path!==null||manual.session!==null;manual.startGeneration++;manual.active=false;manual.starting=false;manual.path=null;manual.pointerId=null;manual.session=null;clearManualTimer();if(hadSession)sendManualStop();}
function stopManualIfActive(){stopManual();}
function sendManual(){if(!manual.active||manual.inFlight||manual.stopping)return;let path;try{path=manualRequestPath(manual.path)}catch(error){text('fault',`Manual request failed: ${error.message}; STOP sent`);stopManual();return;}manual.inFlight=true;manual.controller=new AbortController();fetch(path,{method:'POST',signal:manual.controller.signal}).then(async r=>{if(!r.ok){const p=await r.json().catch(()=>({}));throw new Error(p.error||'Manual command rejected')}}).catch(error=>{if(error.name!=='AbortError'){text('fault',`Manual request failed: ${error.message}; STOP sent`);stopManual();}}).finally(()=>{manual.inFlight=false;manual.controller=null;});}
async function holdManual(path,pointerId){if(manual.stopping){manual.pendingStart={path,pointerId};return;}if(manual.active&&manual.pointerId!==null&&manual.pointerId!==pointerId){stopManual();manual.pendingStart={path,pointerId};return;}if(typeof manual.epoch!=='string'||!manual.epoch){text('fault','Manual session unavailable; wait for current standby status');return;}manual.active=true;manual.starting=true;manual.path=path;manual.pointerId=pointerId;manual.session=null;const generation=++manual.startGeneration;try{const response=await fetch(`/api/manual/session?epoch=${encodeURIComponent(manual.epoch)}`,{method:'POST'});const data=await response.json();if(!response.ok)throw new Error(data.error||'Manual session rejected');if(!manual.active||manual.pointerId!==pointerId||generation!==manual.startGeneration)return;manual.session=data.session;manual.starting=false;sendManual();clearManualTimer();manual.timer=setInterval(sendManual,100);}catch(error){if(generation!==manual.startGeneration)return;text('fault',`Manual session failed: ${error.message}; STOP sent`);stopManual();}}
function releaseManual(pointerId){if(manual.pendingStart&&manual.pendingStart.pointerId===pointerId)manual.pendingStart=null;if(manual.active&&manual.pointerId===pointerId)stopManual();}
document.querySelectorAll('[data-manual-path]').forEach(button=>{const down=e=>{e.preventDefault();button.setPointerCapture?.(e.pointerId);holdManual(button.dataset.manualPath,e.pointerId)};const up=e=>{e.preventDefault();releaseManual(e.pointerId)};button.addEventListener('pointerdown',down);['pointerup','pointercancel','pointerleave'].forEach(event=>button.addEventListener(event,up));});
document.getElementById('stop').addEventListener('click',stopManual);window.addEventListener('blur',stopManualIfActive);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopManualIfActive()});
async function refresh(){try{const p=await (await fetch('/api/status',{cache:'no-store'})).json();if(typeof p.instance_id==='string')dashboardInstanceId=p.instance_id;observeManualEpoch(p);const manualReady=p.mode==='MANUAL'&&p.physical_web_standby.active;configureManualSpeed(p);text('state',p.state);text('state2',p.state);text('mode',p.mode);text('row',`${p.row_number} / ${p.pass_number}`);text('armed',p.motor_output_armed?'ARMED':'DISARMED');text('standby',p.physical_web_standby.active?'READY (no motion)':'-');document.getElementById('manual-mode').disabled=manualReady;text('manual-help',manualReady?'Manual is already ready. Hold a direction button; do not press MANUAL.':'Select MANUAL before using a direction button. Hold a manual button to drive.');text('fault',p.fault||'');text('odometry-warning','');text('camera',p.camera.ok?'OK':'FAULT');text('imu',p.imu.ok?'OK':'FAULT');text('camera-age',fmt(p.camera.age_s,' s'));text('imu-age',fmt(p.imu.age_s,' s'));text('heading',fmt(p.heading.filtered_heading_deg,' deg'));text('reference',fmt(p.heading.row_heading_reference_deg,' deg'));text('heading-error',fmt(p.heading.heading_error_deg,' deg'));text('build-distance',fmt(p.heading.reference_build_distance_m,' m'));text('target',`${fmt(p.vision.target_x_px,' px')} / ${fmt(p.vision.x_goal_px,' px')}`);text('marker',p.vision.marker_found?'FOUND':'-');text('distance',fmt(p.odometry.distance_m,' m'));text('search',fmt(p.search_distance_m,' m'));}catch(e){text('fault','Diagnostics unavailable')}}
const refreshBase=refresh;refresh=async()=>{await refreshBase();try{const p=await (await fetch('/api/status',{cache:'no-store'})).json();const state=p.state;if(state==='MANUAL')text('navigation-state',p.mode==='AUTO'?'AUTO selected — press Start Auto':'MANUAL');else if(state==='AUTO_ROW_FOLLOW')text('navigation-state',p.navigation_mode==='buds_and_leaves'?'AUTO buds + leaves navigation':'AUTO bud navigation');else if(state==='AUTO_SEARCH')text('navigation-state','AUTO IMU-only navigation');else if(state==='AUTO_IN_ROW_TURN')text('navigation-state','In-row turn');else if(state==='AUTO_NEW_ROW_TURN')text('navigation-state','New-row turn');else if(state==='AUTO_PICK')text('navigation-state','Pick active');else if(state==='AUTO_POST_PICK')text('navigation-state','Post-pick navigation');else if(state==='AUTO_START_DELAY')text('navigation-state','AUTO start delay');else if(state==='AUTO_COMPLETE')text('navigation-state','AUTO complete');else if(state==='FAULT')text('navigation-state',`FAULT: ${p.fault||p.state_reason||''}`);else text('navigation-state',state);}catch(_){}};
refresh();setInterval(refresh,1000);
loadConfig();
// VS Code SSH forwarding can buffer multipart MJPEG responses.  Polling
// individual latest-value JPEGs avoids that proxy behaviour.  Each view has
// one in-flight request, so a slow client cannot accumulate stale frames.
const snapshotPollMs=100;
document.querySelectorAll('[data-snapshot-view]').forEach(image=>{
  const snapshot={inFlight:false,currentUrl:null,pendingUrl:null};
  // Keep the last decoded image alive until its replacement is decoded.  A
  // newer response can supersede an undecoded pending blob immediately;
  // intermediate blob URLs are then released even when load events race.
  image.addEventListener('load',()=>{
    const nextUrl=snapshot.pendingUrl;
    if(!nextUrl||image.currentSrc!==nextUrl)return;
    snapshot.pendingUrl=null;
    const previousUrl=snapshot.currentUrl;
    snapshot.currentUrl=nextUrl;
    if(previousUrl)URL.revokeObjectURL(previousUrl);
  });
  image.addEventListener('error',()=>{
    const failedUrl=snapshot.pendingUrl;
    if(failedUrl&&image.currentSrc===failedUrl){snapshot.pendingUrl=null;URL.revokeObjectURL(failedUrl);}
  });
  async function loadSnapshot(){
    if(snapshot.inFlight)return;
    snapshot.inFlight=true;
    try{
      const view=image.dataset.snapshotView;
      const response=await fetch(`/snapshot/${view}?t=${Date.now()}`,{cache:'no-store'});
      if(response.status===204)return;
      if(!response.ok)throw new Error(`HTTP ${response.status}`);
      const nextUrl=URL.createObjectURL(await response.blob());
      const supersededUrl=snapshot.pendingUrl;
      snapshot.pendingUrl=nextUrl;
      if(supersededUrl)URL.revokeObjectURL(supersededUrl);
      image.src=nextUrl;
    }catch(_error){
      // Diagnostics video is best-effort; status polling remains independent.
    }finally{snapshot.inFlight=false;}
  }
  loadSnapshot();setInterval(loadSnapshot,snapshotPollMs);
});
</script></body></html>"""
