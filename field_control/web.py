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
                if path == "/api/manual/hold":
                    if request.query:
                        self._conflict("manuellt hold accepterar inte RPM-parameter"); return
                    try:
                        # This route accepts no client speed or direction. It
                        # keeps the already-claimed MANUAL lease alive with a
                        # verified zero A2 command after pointer release.
                        runtime.manual_command(WheelCommand(0.0, 0.0, "web-manual-hold"))
                    except (ValueError, RuntimeError) as exc:
                        self._conflict(str(exc)); return
                    self._status(); return
                if path in manual_directions:
                    left, right, direction = manual_directions[path]
                    try:
                        rpm = _manual_rpm_from_query(
                            request.query,
                            default_rpm=runtime.config.manual_rpm,
                            max_rpm=runtime.config.max_rpm,
                            motor_turns_per_wheel_turn=getattr(getattr(runtime.config, "odometry_geometry", None),
                                                               "motor_turns_per_wheel_turn", 1.0),
                        )
                        # Runtime enforces MANUAL, lifecycle, armed output and
                        # the shared lease independently of HTTP input.
                        runtime.manual_command(WheelCommand(left * rpm, right * rpm, f"web-manual-{direction}"))
                    except (ValueError, RuntimeError) as exc:
                        self._conflict(str(exc)); return
                    self._status(); return
                actions = {
                    "/api/manual": runtime.select_manual,
                    "/api/auto": runtime.select_auto,
                    "/api/start-auto": runtime.start_auto,
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
*{box-sizing:border-box}body{margin:0}.top{background:#173d3a;color:white;padding:20px 5vw;display:flex;justify-content:space-between;align-items:center;gap:20px}.top h1{margin:0;font-size:22px}.top span{color:#b9d5cb;font-size:13px}main{max-width:1720px;margin:24px auto;padding:0 20px;overflow-x:hidden}button,input,select{font:inherit;min-width:0}button{border:0;border-radius:5px;padding:11px 16px;font-weight:700;cursor:pointer;color:white;background:#173d3a}button.stop{background:var(--accent)}button:disabled{opacity:.45;cursor:not-allowed}.control-layout{display:grid;grid-template-columns:minmax(270px,320px) minmax(390px,500px) minmax(260px,1fr);gap:16px;align-items:start}.mode-actions,.manual-controls{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.mode-actions button:first-child{grid-column:span 2}.speed-input,.direct-config input,.direct-config select,.config-fields input{width:100%;margin:4px 0 10px;padding:7px;border:1px solid var(--line);border-radius:5px}.direct-config{margin:12px 0}.direct-config label,.panel>label{display:block;color:var(--muted);font-size:12px}.direct-config .check-row{display:flex;align-items:center;gap:7px}.direct-config .check-row input{width:auto;margin:0}.manual-controls{margin:12px 0}.manual-controls button{min-height:52px}.manual-controls .reverse{background:#8d3b3b}.stop{width:100%;min-height:52px}.panel{background:white;border:1px solid var(--line);border-radius:6px;padding:16px;min-width:0}.panel h2{font-size:15px;margin:0 0 12px;color:#173d3a}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.metric{border-top:1px solid #e5eaeb;padding-top:8px}.label{display:block;color:var(--muted);font-size:12px}.value{font-size:18px;font-weight:700;overflow-wrap:anywhere}.config-fields{max-height:580px;overflow-y:auto;overflow-x:hidden;display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:5px 8px}.config-fields label{font-size:10px;color:var(--muted);overflow-wrap:anywhere}.config-fields input{margin:2px 0;padding:5px;font-size:12px}.compact-status{font-size:11px}.compact-status h3{font-size:12px;color:#173d3a;margin:12px 0 3px}.compact-status h3:first-of-type{margin-top:0}.compact-status .grid{gap:5px}.compact-status .metric{padding-top:4px}.compact-status .label{font-size:10px}.compact-status .value{font-size:13px}.ok{color:var(--ok)}.bad{color:var(--accent)}.streams{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.streams img{width:100%;aspect-ratio:4/3;object-fit:contain;background:#17212b;border-radius:4px}.streams h3{font-size:13px;margin:0 0 6px}.legend{display:flex;flex-wrap:wrap;gap:4px 10px;margin:0 0 6px;font-size:11px;color:var(--muted)}.legend span{display:inline-flex;align-items:center;gap:4px}.swatch{width:11px;height:11px;border-radius:2px;display:inline-block}.fault{color:var(--accent);min-height:20px;font-weight:700}@media(max-width:1180px){.control-layout{grid-template-columns:minmax(270px,330px) minmax(360px,1fr)}.compact-status{grid-column:span 2}}@media(max-width:760px){.control-layout,.streams{grid-template-columns:1fr}.compact-status{grid-column:auto}.top{align-items:flex-start;flex-direction:column}}
</style></head><body><header class="top"><div><h1>Field Control</h1><span>Runtime diagnostics</span></div><strong id="state">Loading</strong></header>
<main><div class="control-layout">
<aside class="panel" aria-label="Control"><h2>Control</h2><div class="mode-actions"><button id="manual-mode" onclick="post('/api/manual')">MANUAL</button><button onclick="post('/api/auto')">AUTO</button><button onclick="post('/api/start-auto')">START AUTO</button></div><div class="direct-config"><label>Row following<select data-direct="vision.navigation_mode"><option value="buds_only">Buds only</option><option value="buds_and_leaves">Buds + leaves</option></select></label><label>Max IMU-only navigation (m)<input data-direct="safety.search_length_m" type="number" min="0" step="0.1"></label><label>Pick trigger timeout (s)<input data-direct="safety.max_pick_wait_s" type="number" min="0" step="0.1"></label><label class="check-row"><input data-direct="safety.in_row_turn_enabled" type="checkbox"><span>In-row turn enabled</span></label><label>New-row direction<select data-direct="safety.new_row_turn_direction"><option value="left">Left</option><option value="right">Right</option></select></label><label>Rows to harvest<input data-direct="safety.number_of_rows" type="number" min="1" step="1"></label></div><label class="label" for="rpm">Desired speed (wheel RPM)</label><input class="speed-input" id="rpm" type="number" min="0.01" step="0.1" inputmode="decimal" disabled><div class="manual-controls" aria-label="Manual drive controls"><button data-manual-path="/api/manual/both/forward">BOTH FORWARD</button><button class="reverse" data-manual-path="/api/manual/both/reverse">BOTH REVERSE</button><button data-manual-path="/api/manual/left/forward">LEFT FORWARD</button><button data-manual-path="/api/manual/right/forward">RIGHT FORWARD</button><button class="reverse" data-manual-path="/api/manual/left/reverse">LEFT REVERSE</button><button class="reverse" data-manual-path="/api/manual/right/reverse">RIGHT REVERSE</button></div><button class="stop" id="stop">STOP</button><p id="manual-help">Select MANUAL before using a direction button. Hold a manual button to drive. The browser refreshes the bounded control lease every 100 ms and sends STOP when released. These buttons never arm motors.</p></aside>
<aside class="panel" aria-label="Configuration"><h2>Configuration</h2><p id="config-note" class="label">Changes apply on restart. Motor boundary acceleration is fixed and verified.</p><label>X goal (normalised)<input data-direct="vision.x_goal" type="number" min="0" max="1" step="0.01"></label><div id="config-fields" class="config-fields"></div><button id="save-config">Save configuration</button><label>Load saved configuration<select id="profile-select"></select></label><button id="select-config">Use selected on next restart</button><button id="restart-config">Reload/restart from configuration</button></aside>
<aside class="panel compact-status" aria-label="Runtime, sensors, heading, vision and odometry"><h2>Runtime and navigation</h2><h3>Runtime</h3><div class="grid"><div class="metric"><span class="label">Mode</span><span class="value" id="mode">-</span></div><div class="metric"><span class="label">State</span><span class="value" id="state2">-</span></div><div class="metric"><span class="label">Row / pass</span><span class="value" id="row">-</span></div><div class="metric"><span class="label">Motor output</span><span class="value" id="armed">-</span></div><div class="metric"><span class="label">Local web standby</span><span class="value" id="standby">-</span></div></div><p class="fault" id="fault"></p><h3>Sensors</h3><div class="grid"><div class="metric"><span class="label">Camera</span><span class="value" id="camera">-</span></div><div class="metric"><span class="label">IMU</span><span class="value" id="imu">-</span></div><div class="metric"><span class="label">Camera age</span><span class="value" id="camera-age">-</span></div><div class="metric"><span class="label">IMU age</span><span class="value" id="imu-age">-</span></div></div><h3>Heading</h3><div class="grid"><div class="metric"><span class="label">Filtered</span><span class="value" id="heading">-</span></div><div class="metric"><span class="label">Row reference</span><span class="value" id="reference">-</span></div><div class="metric"><span class="label">Error</span><span class="value" id="heading-error">-</span></div><div class="metric"><span class="label">Reference build distance</span><span class="value" id="build-distance">-</span></div></div><h3>Vision and odometry</h3><div class="grid"><div class="metric"><span class="label">Target x / goal</span><span class="value" id="target">-</span></div><div class="metric"><span class="label">Marker</span><span class="value" id="marker">-</span></div><div class="metric"><span class="label">Distance</span><span class="value" id="distance">-</span></div><div class="metric"><span class="label">Search distance</span><span class="value" id="search">-</span></div></div><p class="fault" id="odometry-warning"></p></aside></div>
<section class="panel" style="margin-top:16px"><h2>Live views</h2><div class="streams"><div><h3>Original</h3><div class="legend" aria-label="Zone colours"><span><i class="swatch" style="background:#00b4ff"></i>Navigation zone</span><span><i class="swatch" style="background:#ffff00"></i>Trigger zone</span><span><i class="swatch" style="background:#ff00ff"></i>Pick zone</span><span><i class="swatch" style="background:#ffb400"></i>Turn marker zone</span><span><i class="swatch" style="background:#ff0000"></i>x goal</span></div><img data-snapshot-view="raw" alt="Original camera"></div><div><h3>Overlay</h3><img data-snapshot-view="overlay" alt="Vision overlay"></div><div><h3>Buds mask</h3><img data-snapshot-view="buds" alt="Buds mask"></div><div><h3>Leaves mask</h3><img data-snapshot-view="leaves" alt="Leaves mask"></div></div></section></main>
<script>
const text=(id,value)=>document.getElementById(id).textContent=value??'-';
const fmt=(value,suffix='')=>value===null||value===undefined?'-':Number(value).toFixed(2)+suffix;
const rpmInput=document.getElementById('rpm');
// The status endpoint reports motor-side RPM while this control deliberately
// displays wheel-side RPM.  Do not initialise the control until the selected
// profile has supplied its verified motor-to-wheel ratio.
let speedInitialized=false, profileCandidate=null, gearRatio=null, manualSpeedStatus=null, dashboardInstanceId=null;
const directPaths=['vision.navigation_mode','vision.x_goal','safety.search_length_m','safety.max_pick_wait_s','safety.in_row_turn_enabled','safety.new_row_turn_direction','safety.number_of_rows'];
function atPath(value,path){return path.split('.').reduce((v,k)=>v?.[k],value)}
function setPath(value,path,next){const keys=path.split('.');let target=value;for(const key of keys.slice(0,-1))target=target[key];target[keys.at(-1)]=next;}
function leafEntries(value,prefix=''){if(Array.isArray(value))return [[prefix,value.join(',')]];if(value&&typeof value==='object')return Object.entries(value).flatMap(([key,item])=>leafEntries(item,prefix?`${prefix}.${key}`:key));return [[prefix,value]];}
function isRpmPath(path){return path.endsWith('_rpm');}
function renderConfig(candidate){profileCandidate=candidate;const ratio=Number(candidate.odometry_geometry?.motor_turns_per_wheel_turn);gearRatio=Number.isFinite(ratio)&&ratio>0?ratio:null;document.querySelectorAll('[data-direct]').forEach(el=>{const value=atPath(candidate,el.dataset.direct);if(el.type==='checkbox')el.checked=Boolean(value);else el.value=value??'';});const root=document.getElementById('config-fields');root.textContent='';for(const [path,value] of leafEntries(candidate)){if(directPaths.includes(path))continue;const label=document.createElement('label');label.textContent=isRpmPath(path)?`${path} (wheel RPM)`:path;const input=document.createElement('input');input.dataset.path=path;input.value=value===null?'':String(isRpmPath(path)?Number(value)/gearRatio:value);input.type=typeof value==='boolean'?'checkbox':'text';if(input.type==='checkbox')input.checked=value;label.append(input);root.append(label);}if(manualSpeedStatus)configureManualSpeed(manualSpeedStatus);}
function candidateFromForm(){const candidate=structuredClone(profileCandidate);document.querySelectorAll('[data-direct]').forEach(el=>setPath(candidate,el.dataset.direct,el.type==='checkbox'?el.checked:(el.type==='number'?Number(el.value):el.value)));document.querySelectorAll('#config-fields input').forEach(el=>{const old=atPath(candidate,el.dataset.path);let value=el.type==='checkbox'?el.checked:el.value;if(Array.isArray(old))value=value.split(',').map(Number);else if(typeof old==='number')value=Number(value);else if(old===null&&value==='')value=null;if(isRpmPath(el.dataset.path))value*=gearRatio;setPath(candidate,el.dataset.path,value);});return candidate;}
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
  if(path==='/api/manual/hold')return path;
  const rpm=Number(rpmInput.value), maxRpm=Number(rpmInput.max);
  if(!Number.isFinite(rpm)||rpm<=0||!Number.isFinite(maxRpm)||maxRpm<=0||rpm>maxRpm){throw new Error('RPM must be positive and no greater than configured maximum');}
  return `${path}?rpm=${encodeURIComponent(rpm)}`;
}
async function post(path){try{const r=await fetch(path,{method:'POST'});const p=await r.json();if(!r.ok) alert(p.error||'Command rejected');return r.ok}catch(e){alert('Request failed');return false}}
// Manual HTTP delivery deliberately has one request in flight.  Slow or lost
// browser requests therefore cannot become an unbounded queue of stale drive
// commands.  Pointer release changes to a lease-refreshing zero command;
// explicit STOP, loss of page visibility and failed delivery remain hard
// stop/disarm actions. Runtime-side lease expiry is the independent backstop.
const manual={active:false,path:null,pointerId:null,timer:null,inFlight:false,controller:null,stopping:false};
function clearManualTimer(){if(manual.timer!==null){clearInterval(manual.timer);manual.timer=null;}}
function stopManual(){manual.active=false;manual.path=null;manual.pointerId=null;clearManualTimer();if(manual.controller)manual.controller.abort();if(manual.stopping)return;manual.stopping=true;fetch('/api/stop',{method:'POST'}).then(async r=>{if(!r.ok){const p=await r.json().catch(()=>({}));throw new Error(p.error||'STOP rejected')}}).catch(()=>{text('fault','Manual STOP request failed; runtime lease will stop output')}).finally(()=>{manual.stopping=false;});}
function sendManual(){if(!manual.active||manual.inFlight||manual.stopping)return;let path;try{path=manualRequestPath(manual.path)}catch(error){text('fault',`Manual request failed: ${error.message}; STOP sent`);stopManual();return;}manual.inFlight=true;manual.controller=new AbortController();fetch(path,{method:'POST',signal:manual.controller.signal}).then(async r=>{if(!r.ok){const p=await r.json().catch(()=>({}));throw new Error(p.error||'Manual command rejected')}}).catch(error=>{if(error.name!=='AbortError'){text('fault',`Manual request failed: ${error.message}; STOP sent`);stopManual();}}).finally(()=>{manual.inFlight=false;manual.controller=null;});}
function holdManual(path,pointerId){if(manual.active&&manual.pointerId!==null&&manual.pointerId!==pointerId)stopManual();manual.active=true;manual.path=path;manual.pointerId=pointerId;sendManual();clearManualTimer();manual.timer=setInterval(sendManual,100);}
function releaseManual(pointerId){if(manual.active&&manual.pointerId===pointerId){manual.pointerId=null;manual.path='/api/manual/hold';sendManual();}}
document.querySelectorAll('[data-manual-path]').forEach(button=>{const down=e=>{e.preventDefault();button.setPointerCapture?.(e.pointerId);holdManual(button.dataset.manualPath,e.pointerId)};const up=e=>{e.preventDefault();releaseManual(e.pointerId)};button.addEventListener('pointerdown',down);['pointerup','pointercancel','pointerleave'].forEach(event=>button.addEventListener(event,up));});
document.getElementById('stop').addEventListener('click',stopManual);window.addEventListener('blur',stopManual);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopManual()});
async function refresh(){try{const p=await (await fetch('/api/status',{cache:'no-store'})).json();if(typeof p.instance_id==='string')dashboardInstanceId=p.instance_id;const manualReady=p.mode==='MANUAL'&&p.physical_web_standby.active;configureManualSpeed(p);text('state',p.state);text('state2',p.state);text('mode',p.mode);text('row',`${p.row_number} / ${p.pass_number}`);text('armed',p.motor_output_armed?'ARMED':'DISARMED');text('standby',p.physical_web_standby.active?'READY (no motion)':'-');document.getElementById('manual-mode').disabled=manualReady;text('manual-help',manualReady?'Manual is already ready. Hold a direction button; do not press MANUAL.':'Select MANUAL before using a direction button. Hold a manual button to drive.');text('fault',p.fault||'');text('odometry-warning','');text('camera',p.camera.ok?'OK':'FAULT');text('imu',p.imu.ok?'OK':'FAULT');text('camera-age',fmt(p.camera.age_s,' s'));text('imu-age',fmt(p.imu.age_s,' s'));text('heading',fmt(p.heading.filtered_heading_deg,' deg'));text('reference',fmt(p.heading.row_heading_reference_deg,' deg'));text('heading-error',fmt(p.heading.heading_error_deg,' deg'));text('build-distance',fmt(p.heading.reference_build_distance_m,' m'));text('target',`${fmt(p.vision.target_x_px,' px')} / ${fmt(p.vision.x_goal_px,' px')}`);text('marker',p.vision.marker_found?'FOUND':'-');text('distance',fmt(p.odometry.distance_m,' m'));text('search',fmt(p.search_distance_m,' m'));}catch(e){text('fault','Diagnostics unavailable')}}
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
