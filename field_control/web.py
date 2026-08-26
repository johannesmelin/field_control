"""Non-blocking diagnostics HTTP server with latest-frame MJPEG streams."""
from __future__ import annotations

import json
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
from urllib.parse import urlsplit

from .control import WheelCommand
from .diagnostics import status_payload
from .runtime import FieldControlRuntime


class DiagnosticsServer:
    def __init__(self, runtime: FieldControlRuntime, host: str = "127.0.0.1", port: int = 8080) -> None:
        self.runtime = runtime
        self._server = ThreadingHTTPServer((host, port), self._handler())
        self._server.daemon_threads = True
        self._thread: threading.Thread | None = None

    @property
    def address(self) -> tuple[str, int]:
        return self._server.server_address

    def _handler(self):
        runtime = self.runtime

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                path = urlsplit(self.path).path
                if path == "/api/status":
                    body = json.dumps(status_payload(runtime), ensure_ascii=True).encode()
                    self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
                if path in ("/stream/raw", "/stream/overlay", "/stream/buds", "/stream/leaves", "/stream/marker"):
                    self._stream(path.rsplit("/", 1)[-1]); return
                body = DASHBOARD_HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

            def do_POST(self) -> None:
                path = urlsplit(self.path).path
                manual_directions = {
                    # WheelCommand is logical vehicle direction. The verified
                    # remote physical worker applies its configured per-motor
                    # forward signs exactly once when constructing raw A2.
                    "/api/manual/forward": (1.0, 1.0, "forward"),
                    "/api/manual/reverse": (-1.0, -1.0, "reverse"),
                    "/api/manual/left": (-1.0, 1.0, "left"),
                    "/api/manual/right": (1.0, -1.0, "right"),
                }
                if path in manual_directions:
                    rpm = runtime.config.manual_rpm
                    if rpm <= 0:
                        self._conflict("manual_rpm måste vara positiv för manuell körning"); return
                    left, right, direction = manual_directions[path]
                    try:
                        # Client input never supplies RPM. Runtime enforces
                        # MANUAL, lifecycle, armed output and the shared lease.
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

            def _status(self) -> None:
                body = json.dumps(status_payload(runtime), ensure_ascii=True).encode()
                self.send_response(HTTPStatus.OK); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

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
*{box-sizing:border-box}body{margin:0}.top{background:#173d3a;color:white;padding:20px 5vw;display:flex;justify-content:space-between;align-items:center;gap:20px}.top h1{margin:0;font-size:22px}.top span{color:#b9d5cb;font-size:13px}main{max-width:1200px;margin:24px auto;padding:0 20px}.actions{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:18px}button{border:0;border-radius:5px;padding:11px 16px;font-weight:700;cursor:pointer;color:white;background:#173d3a}button.stop{background:var(--accent)}button:disabled{opacity:.45;cursor:not-allowed}.layout{display:grid;grid-template-columns:1fr 1fr;gap:16px}.panel{background:white;border:1px solid var(--line);border-radius:6px;padding:16px}.panel h2{font-size:15px;margin:0 0 12px;color:#173d3a}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.metric{border-top:1px solid #e5eaeb;padding-top:8px}.label{display:block;color:var(--muted);font-size:12px}.value{font-size:18px;font-weight:700;overflow-wrap:anywhere}.ok{color:var(--ok)}.bad{color:var(--accent)}.streams{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.streams img{width:100%;aspect-ratio:4/3;object-fit:contain;background:#17212b;border-radius:4px}.streams h3{font-size:13px;margin:0 0 6px}.fault{color:var(--accent);min-height:20px;font-weight:700}@media(max-width:760px){.layout,.streams{grid-template-columns:1fr}.top{align-items:flex-start;flex-direction:column}}
</style></head><body><header class="top"><div><h1>Field Control</h1><span>Runtime diagnostics</span></div><strong id="state">Loading</strong></header>
<main><div class="actions"><button onclick="post('/api/manual')">MANUAL</button><button onclick="post('/api/auto')">AUTO</button><button onclick="post('/api/start-auto')">START AUTO</button><button onclick="post('/api/manual/forward')">FORWARD (fixed motor RPM)</button><button onclick="post('/api/manual/reverse')">REVERSE (fixed motor RPM)</button><button onclick="post('/api/manual/left')">LEFT (fixed motor RPM)</button><button onclick="post('/api/manual/right')">RIGHT (fixed motor RPM)</button><button class="stop" onclick="post('/api/stop')">STOP</button></div><p>Manual drive is accepted only in MANUAL with already armed output; each press refreshes the bounded control lease. These buttons never arm motors.</p>
<div class="layout"><section class="panel"><h2>Runtime</h2><div class="grid"><div class="metric"><span class="label">Mode</span><span class="value" id="mode">-</span></div><div class="metric"><span class="label">State</span><span class="value" id="state2">-</span></div><div class="metric"><span class="label">Row / pass</span><span class="value" id="row">-</span></div><div class="metric"><span class="label">Motor output</span><span class="value" id="armed">-</span></div></div><p class="fault" id="fault"></p></section>
<section class="panel"><h2>Sensors</h2><div class="grid"><div class="metric"><span class="label">Camera</span><span class="value" id="camera">-</span></div><div class="metric"><span class="label">IMU</span><span class="value" id="imu">-</span></div><div class="metric"><span class="label">Camera age</span><span class="value" id="camera-age">-</span></div><div class="metric"><span class="label">IMU age</span><span class="value" id="imu-age">-</span></div></div></section>
<section class="panel"><h2>Heading</h2><div class="grid"><div class="metric"><span class="label">Filtered</span><span class="value" id="heading">-</span></div><div class="metric"><span class="label">Row reference</span><span class="value" id="reference">-</span></div><div class="metric"><span class="label">Error</span><span class="value" id="heading-error">-</span></div><div class="metric"><span class="label">Reference build distance</span><span class="value" id="build-distance">-</span></div></div></section>
<section class="panel"><h2>Vision and odometry</h2><div class="grid"><div class="metric"><span class="label">Target x / goal</span><span class="value" id="target">-</span></div><div class="metric"><span class="label">Marker</span><span class="value" id="marker">-</span></div><div class="metric"><span class="label">Distance</span><span class="value" id="distance">-</span></div><div class="metric"><span class="label">Search distance</span><span class="value" id="search">-</span></div></div></section></div>
<section class="panel" style="margin-top:16px"><h2>Live views</h2><div class="streams"><div><h3>Original</h3><img src="/stream/raw" alt="Original camera"></div><div><h3>Overlay</h3><img src="/stream/overlay" alt="Vision overlay"></div><div><h3>Buds mask</h3><img src="/stream/buds" alt="Buds mask"></div><div><h3>Leaves mask</h3><img src="/stream/leaves" alt="Leaves mask"></div></div></section></main>
<script>
const text=(id,value)=>document.getElementById(id).textContent=value??'-';
const fmt=(value,suffix='')=>value===null||value===undefined?'-':Number(value).toFixed(2)+suffix;
async function post(path){try{const r=await fetch(path,{method:'POST'});const p=await r.json();if(!r.ok) alert(p.error||'Command rejected');}catch(e){alert('Request failed')}}
async function refresh(){try{const p=await (await fetch('/api/status',{cache:'no-store'})).json();text('state',p.state);text('state2',p.state);text('mode',p.mode);text('row',`${p.row_number} / ${p.pass_number}`);text('armed',p.motor_output_armed?'ARMED':'DISARMED');text('fault',p.fault||'');text('camera',p.camera.ok?'OK':'FAULT');text('imu',p.imu.ok?'OK':'FAULT');text('camera-age',fmt(p.camera.age_s,' s'));text('imu-age',fmt(p.imu.age_s,' s'));text('heading',fmt(p.heading.filtered_heading_deg,' deg'));text('reference',fmt(p.heading.row_heading_reference_deg,' deg'));text('heading-error',fmt(p.heading.heading_error_deg,' deg'));text('build-distance',fmt(p.heading.reference_build_distance_m,' m'));text('target',`${fmt(p.vision.target_x_px,' px')} / ${fmt(p.vision.x_goal_px,' px')}`);text('marker',p.vision.marker_found?'FOUND':'-');text('distance',fmt(p.odometry.distance_m,' m'));text('search',fmt(p.search_distance_m,' m'));}catch(e){text('fault','Diagnostics unavailable')}}
refresh();setInterval(refresh,1000);
</script></body></html>"""
