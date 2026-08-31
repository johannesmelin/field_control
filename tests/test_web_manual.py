from __future__ import annotations

import json
import io
import threading
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from field_control.web import DiagnosticsServer
from field_control.web import _manual_rpm_from_query
from field_control.config import RuntimeConfig


class FakeRuntime:
    def __init__(self, rpm=6.0, error: Exception | None = None):
        self.config = SimpleNamespace(manual_rpm=rpm, max_rpm=10.0, stream_fps=5)
        self.commands = []; self.error = error
        self.images = {}
        self.row_reset_calls = 0
        self.application_restart_calls = 0
        self._lease_token = None; self._arming_in_progress = False
        self._configuration_restart_pending = False
    def manual_command(self, command):
        if self.error is not None: raise self.error
        self.commands.append(command)
    def select_manual(self): pass
    def select_auto(self): pass
    def start_auto(self): pass
    def stop(self): pass
    def reset_row_progress(self): self.row_reset_calls += 1
    def begin_application_restart(self): self.application_restart_calls += 1
    def latest_image(self, view): return self.images.get(view)
    def configuration_restart_safe(self):
        status = self.status()
        return (status.mode == "MANUAL" and not status.motor_output_armed
                and self._lease_token is None and not self._arming_in_progress
                and not self._configuration_restart_pending)
    def reserve_configuration_restart(self):
        if not self.configuration_restart_safe(): return False
        self._configuration_restart_pending = True
        return True
    def cancel_configuration_restart(self): self._configuration_restart_pending = False


class ManualWebTests(unittest.TestCase):
    def test_browser_wheel_rpm_is_converted_once_at_http_boundary(self):
        self.assertEqual(_manual_rpm_from_query("rpm=5", default_rpm=30, max_rpm=40,
                                                motor_turns_per_wheel_turn=8), 40)
        with self.assertRaises(ValueError):
            _manual_rpm_from_query("rpm=5.1", default_rpm=30, max_rpm=40,
                                   motor_turns_per_wheel_turn=8)

    def test_row_progress_reset_route_and_control_button_are_explicit(self):
        from field_control.web import DASHBOARD_HTML

        runtime = FakeRuntime()
        status, body = self.request(runtime, "/api/reset-row-progress")
        self.assertEqual((status, body), (200, {"ok": True}))
        self.assertEqual(runtime.row_reset_calls, 1)
        self.assertIn("restartApplicationButton.id='restart-application'", DASHBOARD_HTML)
        self.assertIn("fetch('/api/application/restart',{method:'POST'})", DASHBOARD_HTML)
        self.assertIn("resetRowProgressButton.id='reset-row-progress'", DASHBOARD_HTML)
        self.assertIn("post('/api/reset-row-progress')", DASHBOARD_HTML)

    def test_dashboard_stages_auto_and_turn_wheel_rpm_without_live_mutation(self):
        from field_control.web import DASHBOARD_HTML
        self.assertEqual(DASHBOARD_HTML.count('data-staged-rpm="auto_base_rpm"'), 1)
        self.assertEqual(DASHBOARD_HTML.count('data-staged-rpm="turn_speed_rpm"'), 1)
        self.assertIn('candidate[el.dataset.stagedRpm]=wheelRpm*gearRatio', DASHBOARD_HTML)
        self.assertIn("const directPaths=['auto_base_rpm','turn_speed_rpm'", DASHBOARD_HTML)
        # Wheel RPM 5 with the configured 8:1 motor-to-wheel ratio stages
        # the motor-side profile value 40, exactly once.
        self.assertIn('wheelRpm*gearRatio', DASHBOARD_HTML)
        self.assertIn('grid-template-columns:repeat(3,minmax(0,1fr))', DASHBOARD_HTML)
        self.assertIn('.compact-status{grid-column:1/-1;grid-row:1}', DASHBOARD_HTML)
        self.assertIn('.tab-pane>section.panel{grid-column:2/-1;grid-row:2', DASHBOARD_HTML)
        self.assertIn('.panel[aria-label="Control"] .manual-controls button{min-height:38px', DASHBOARD_HTML)
        self.assertIn('.compact-status{padding:9px 12px}', DASHBOARD_HTML)
        self.assertIn('.compact-status .grid{display:inline-grid;grid-template-columns:repeat(5,minmax(62px,1fr))', DASHBOARD_HTML)
        self.assertEqual(DASHBOARD_HTML.count('data-staged-value="safety.turn_timeout_s"'), 1)
        self.assertIn("'safety.turn_timeout_s'", DASHBOARD_HTML)
        self.assertIn('setPath(candidate,el.dataset.stagedValue,value)', DASHBOARD_HTML)

    def test_dashboard_names_goal_relative_zone_width_without_legacy_x_bounds(self):
        from field_control.web import DASHBOARD_HTML
        self.assertIn("x distance from x goal", DASHBOARD_HTML)
        self.assertIn("navigation|trigger|pick", DASHBOARD_HTML)

    def test_dashboard_places_configuration_in_a_scoped_tab_layout(self):
        from field_control.web import DASHBOARD_HTML
        self.assertIn("controlTab.className='tab-pane active'", DASHBOARD_HTML)
        self.assertIn("configurationTab.className='tab-pane'", DASHBOARD_HTML)
        self.assertIn("controlTab.append(controlLayout,liveViews)", DASHBOARD_HTML)
        self.assertIn("configurationTab.append(configurationPanel)", DASHBOARD_HTML)
        self.assertIn(".tab-pane.active{display:grid", DASHBOARD_HTML)
        self.assertNotIn("@media(min-width:1181px){main{display:grid", DASHBOARD_HTML)

    def test_dashboard_migrates_only_centred_legacy_navigation_zones(self):
        from field_control.web import DASHBOARD_HTML
        self.assertIn("const goalRelativeZoneMigrationTolerance=1e-6;", DASHBOARD_HTML)
        self.assertIn("function migrateCentredLegacyZones(candidate)", DASHBOARD_HTML)
        self.assertIn("Math.abs(midpoint-goal)>goalRelativeZoneMigrationTolerance", DASHBOARD_HTML)
        self.assertIn("vision[key]={x_distance:(zone.x_max-zone.x_min)/2,y_min:zone.y_min,y_max:zone.y_max};", DASHBOARD_HTML)
        self.assertIn("for(const name of ['navigation','trigger','pick'])", DASHBOARD_HTML)
        self.assertNotIn("['navigation','trigger','pick','turn_marker']", DASHBOARD_HTML)
        self.assertIn("candidate=migrateCentredLegacyZones(candidate);profileCandidate=candidate;", DASHBOARD_HTML)

    def test_configuration_api_stages_profile_even_while_auto_is_active(self):
        runtime = FakeRuntime(); runtime.config = RuntimeConfig()
        runtime.status = lambda: SimpleNamespace(mode="AUTO", motor_output_armed=False)
        runtime.lease = SimpleNamespace(valid=lambda _now: False)
        with tempfile.TemporaryDirectory() as tmp:
            server = object.__new__(DiagnosticsServer); server.runtime = runtime; server.profiles_dir = Path(tmp)
            handler = object.__new__(server._handler()); response = []; handler.path = "/api/config/save"
            body = b'{"candidate":{}}'; handler.rfile = io.BytesIO(body)
            handler.headers = {"Content-Length": str(len(body)), "Content-Type": "application/json"}; handler.wfile = io.BytesIO()
            handler.send_response = lambda status, *_args: response.append(status); handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None; handler.send_error = lambda status, *_args: response.append(status)
            handler.do_POST()
            self.assertEqual(response, [200])

    def test_configuration_api_saves_restart_staged_profile_when_safe(self):
        runtime = FakeRuntime(); runtime.config = RuntimeConfig()
        runtime.status = lambda: SimpleNamespace(mode="MANUAL", motor_output_armed=False)
        runtime.lease = SimpleNamespace(valid=lambda _now: False)
        with tempfile.TemporaryDirectory() as tmp:
            server = object.__new__(DiagnosticsServer); server.runtime = runtime; server.profiles_dir = Path(tmp)
            handler = object.__new__(server._handler()); response = []; handler.path = "/api/config/save"
            body = json.dumps({"candidate": {"manual_rpm": 2.0}}).encode(); handler.rfile = io.BytesIO(body)
            handler.headers = {"Content-Length": str(len(body)), "Content-Type": "application/json"}; handler.wfile = io.BytesIO()
            handler.send_response = lambda status, *_args: response.append(status); handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None; handler.send_error = lambda status, *_args: response.append(status)
            handler.do_POST()
            self.assertEqual(response, [200]); saved = json.loads(handler.wfile.getvalue())
            self.assertTrue(saved["apply_on_restart"])
            self.assertTrue((Path(tmp) / saved["saved"]).is_file())

    def test_configuration_restart_stages_profile_then_signals_clean_restart(self):
        runtime = FakeRuntime(); runtime.config = RuntimeConfig()
        runtime.status = lambda: SimpleNamespace(mode="MANUAL", motor_output_armed=False)
        runtime.lease = SimpleNamespace(valid=lambda _now: False)
        with tempfile.TemporaryDirectory() as tmp:
            server = object.__new__(DiagnosticsServer)
            server.runtime = runtime; server.profiles_dir = Path(tmp)
            server._restart_requested = threading.Event()
            handler = object.__new__(server._handler()); response = []; handler.path = "/api/config/restart"
            body = json.dumps({"candidate": {"manual_rpm": 2.0}}).encode(); handler.rfile = io.BytesIO(body)
            handler.headers = {"Content-Length": str(len(body)), "Content-Type": "application/json"}; handler.wfile = io.BytesIO()
            handler.send_response = lambda status, *_args: response.append(status); handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None; handler.send_error = lambda status, *_args: response.append(status)
            handler.do_POST()
            self.assertEqual(response, [200])
            payload = json.loads(handler.wfile.getvalue())
            self.assertTrue(payload["restarting"])
            self.assertTrue(server._restart_requested.is_set())
            self.assertEqual(json.loads((Path(tmp) / "selected.json").read_text())["selected"], payload["selected"])

    def test_application_restart_signals_after_response_despite_config_fault_or_reservation(self):
        runtime = FakeRuntime(); runtime._configuration_restart_pending = True
        server = object.__new__(DiagnosticsServer); server.runtime = runtime
        handler = object.__new__(server._handler()); response = []; handler.path = "/api/application/restart"
        handler.wfile = io.BytesIO(); observed_response = []
        class ResponseEvent:
            def set(self): observed_response.append(handler.wfile.getvalue())
            def is_set(self): return bool(observed_response)
        server._restart_requested = ResponseEvent()
        handler.send_response = lambda status, *_args: response.append(status); handler.send_header = lambda *_args: None
        handler.end_headers = lambda: None; handler.send_error = lambda status, *_args: response.append(status)
        handler.do_POST()

        self.assertEqual(response, [200])
        self.assertEqual(json.loads(handler.wfile.getvalue()), {"restarting": True})
        self.assertEqual(observed_response, [handler.wfile.getvalue()])
        self.assertEqual(runtime.application_restart_calls, 1)
        self.assertTrue(server._restart_requested.is_set())

    def test_configuration_restart_rejects_armed_or_leased_runtime(self):
        runtime = FakeRuntime(); runtime.config = RuntimeConfig()
        runtime.status = lambda: SimpleNamespace(mode="MANUAL", motor_output_armed=True)
        runtime.lease = SimpleNamespace(valid=lambda _now: False)
        with tempfile.TemporaryDirectory() as tmp:
            server = object.__new__(DiagnosticsServer); server.runtime = runtime; server.profiles_dir = Path(tmp)
            server._restart_requested = threading.Event()
            handler = object.__new__(server._handler()); response = []; handler.path = "/api/config/restart"
            body = b'{"candidate":{}}'; handler.rfile = io.BytesIO(body)
            handler.headers = {"Content-Length": str(len(body)), "Content-Type": "application/json"}; handler.wfile = io.BytesIO()
            handler.send_response = lambda status, *_args: response.append(status); handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None; handler.do_POST()
            self.assertEqual(response, [409]); self.assertFalse(server._restart_requested.is_set())

    def test_rejected_configuration_restart_releases_its_reservation(self):
        runtime = FakeRuntime(); runtime.config = RuntimeConfig()
        runtime.status = lambda: SimpleNamespace(mode="MANUAL", motor_output_armed=False)
        runtime.lease = SimpleNamespace(valid=lambda _now: False)
        with tempfile.TemporaryDirectory() as tmp:
            server = object.__new__(DiagnosticsServer); server.runtime = runtime; server.profiles_dir = Path(tmp)
            server._restart_requested = threading.Event()
            handler = object.__new__(server._handler()); response = []; handler.path = "/api/config/restart"
            body = b'{"candidate":{"physical_can":{}}}'; handler.rfile = io.BytesIO(body)
            handler.headers = {"Content-Length": str(len(body)), "Content-Type": "application/json"}; handler.wfile = io.BytesIO()
            handler.send_response = lambda status, *_args: response.append(status); handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None; handler.do_POST()
            self.assertEqual(response, [409]); self.assertFalse(runtime._configuration_restart_pending)

    def test_malformed_restart_body_never_reserves_runtime_authority(self):
        runtime = FakeRuntime(); runtime.config = RuntimeConfig()
        runtime.status = lambda: SimpleNamespace(mode="MANUAL", motor_output_armed=False)
        runtime.lease = SimpleNamespace(valid=lambda _now: False)
        with tempfile.TemporaryDirectory() as tmp:
            server = object.__new__(DiagnosticsServer); server.runtime = runtime; server.profiles_dir = Path(tmp)
            server._restart_requested = threading.Event()
            handler = object.__new__(server._handler()); response = []; handler.path = "/api/config/restart"
            body = b'{'; handler.rfile = io.BytesIO(body)
            handler.headers = {"Content-Length": str(len(body)), "Content-Type": "application/json"}; handler.wfile = io.BytesIO()
            handler.send_response = lambda status, *_args: response.append(status); handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None; handler.do_POST()
            self.assertEqual(response, [409])
            self.assertFalse(runtime._configuration_restart_pending)
            self.assertTrue(runtime.configuration_restart_safe())

    def test_invalid_restart_candidate_never_reserves_runtime_authority(self):
        runtime = FakeRuntime(); runtime.config = RuntimeConfig()
        runtime.status = lambda: SimpleNamespace(mode="MANUAL", motor_output_armed=False)
        runtime.lease = SimpleNamespace(valid=lambda _now: False)
        with tempfile.TemporaryDirectory() as tmp:
            server = object.__new__(DiagnosticsServer); server.runtime = runtime; server.profiles_dir = Path(tmp)
            server._restart_requested = threading.Event()
            handler = object.__new__(server._handler()); response = []; handler.path = "/api/config/restart"
            body = json.dumps({"candidate": {"manual_rpm": -1}}).encode(); handler.rfile = io.BytesIO(body)
            handler.headers = {"Content-Length": str(len(body)), "Content-Type": "application/json"}; handler.wfile = io.BytesIO()
            handler.send_response = lambda status, *_args: response.append(status); handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None; handler.do_POST()
            self.assertEqual(response, [409])
            self.assertFalse(runtime._configuration_restart_pending)
            self.assertTrue(runtime.configuration_restart_safe())

    def request(self, runtime, path):
        with patch("field_control.web.status_payload", return_value={"ok": True}):
            server = object.__new__(DiagnosticsServer); server.runtime = runtime
            handler = object.__new__(server._handler())
            response = []; handler.path = path; handler.wfile = io.BytesIO()
            handler.send_response = lambda status, *_args: response.append(status)
            handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None
            handler.send_error = lambda status, *_args: response.append(status)
            handler.do_POST()
            body = handler.wfile.getvalue()
            return response[0], (json.loads(body) if body else None)

    def test_dashboard_html_is_never_cached_across_server_restart(self):
        runtime = FakeRuntime()
        server = object.__new__(DiagnosticsServer); server.runtime = runtime
        handler = object.__new__(server._handler())
        response, headers = [], []
        handler.path = "/"; handler.wfile = io.BytesIO()
        handler.send_response = lambda status, *_args: response.append(status)
        handler.send_header = lambda name, value: headers.append((name, value))
        handler.end_headers = lambda: None
        handler.do_GET()

        self.assertEqual(response, [200])
        self.assertIn(("Content-Type", "text/html; charset=utf-8"), headers)
        self.assertIn(("Cache-Control", "no-store"), headers)

    def test_status_response_includes_the_server_process_instance_id(self):
        runtime = FakeRuntime()
        server = object.__new__(DiagnosticsServer); server.runtime = runtime; server._instance_id = "boot-123"
        handler = object.__new__(server._handler())
        response = []; handler.path = "/api/status"; handler.wfile = io.BytesIO()
        handler.send_response = lambda status, *_args: response.append(status)
        handler.send_header = lambda *_args: None
        handler.end_headers = lambda: None
        with patch("field_control.web.status_payload", return_value={"ok": True}):
            handler.do_GET()

        self.assertEqual(response, [200])
        self.assertEqual(json.loads(handler.wfile.getvalue()), {"ok": True, "instance_id": "boot-123"})

    def test_independent_servers_capture_distinct_nonfallback_instance_ids(self):
        runtime = FakeRuntime()
        class FakeHttpServer:
            def __init__(self, address, handler_class):
                self.server_address = address
                self.RequestHandlerClass = handler_class
                self.daemon_threads = False
            def server_close(self): pass

        with patch("field_control.web.ThreadingHTTPServer", FakeHttpServer):
            servers = [DiagnosticsServer(runtime, port=0), DiagnosticsServer(runtime, port=0)]
            instance_ids = []
            for server in servers:
                handler = object.__new__(server._server.RequestHandlerClass)
                response = []; handler.path = "/api/status"; handler.wfile = io.BytesIO()
                handler.send_response = lambda status, *_args: response.append(status)
                handler.send_header = lambda *_args: None
                handler.end_headers = lambda: None
                with patch("field_control.web.status_payload", return_value={"ok": True}):
                    handler.do_GET()
                self.assertEqual(response, [200])
                instance_ids.append(json.loads(handler.wfile.getvalue())["instance_id"])

            self.assertNotIn("test-instance", instance_ids)
            self.assertEqual(len(set(instance_ids)), 2)

    def test_snapshot_routes_return_one_cache_free_latest_jpeg(self):
        runtime = FakeRuntime(); runtime.images["raw"] = b"jpeg-bytes"
        server = object.__new__(DiagnosticsServer); server.runtime = runtime
        handler = object.__new__(server._handler())
        response, headers = [], []
        handler.path = "/snapshot/raw?ignored=cache-bust"; handler.wfile = io.BytesIO()
        handler.send_response = lambda status, *_args: response.append(status)
        handler.send_header = lambda name, value: headers.append((name, value))
        handler.end_headers = lambda: None
        handler.do_GET()

        self.assertEqual(response, [200])
        self.assertEqual(handler.wfile.getvalue(), b"jpeg-bytes")
        self.assertIn(("Content-Type", "image/jpeg"), headers)
        self.assertIn(("Cache-Control", "no-store, no-cache, must-revalidate"), headers)
        self.assertIn(("Pragma", "no-cache"), headers)
        self.assertIn(("Expires", "0"), headers)

    def test_snapshot_returns_no_content_until_a_latest_frame_exists(self):
        runtime = FakeRuntime()
        server = object.__new__(DiagnosticsServer); server.runtime = runtime
        handler = object.__new__(server._handler())
        response, headers = [], []
        handler.path = "/snapshot/overlay"; handler.wfile = io.BytesIO()
        handler.send_response = lambda status, *_args: response.append(status)
        handler.send_header = lambda name, value: headers.append((name, value))
        handler.end_headers = lambda: None
        handler.do_GET()

        self.assertEqual(response, [204])
        self.assertEqual(handler.wfile.getvalue(), b"")
        self.assertIn(("Cache-Control", "no-store, no-cache, must-revalidate"), headers)

    def test_direction_routes_without_rpm_keep_configured_default_for_hil_compatibility(self):
        runtime = FakeRuntime(6.0)
        expected = {
            "/api/manual/forward": (6.0, 6.0), "/api/manual/reverse": (-6.0, -6.0),
            "/api/manual/left": (-6.0, 6.0), "/api/manual/right": (6.0, -6.0),
        }
        for path, values in expected.items():
            with self.subTest(path=path):
                status, body = self.request(runtime, path)
                self.assertEqual((status, body), (200, {"ok": True}))
                command = runtime.commands[-1]
                self.assertEqual((command.left_rpm, command.right_rpm), values)
                self.assertEqual(command.source, f"web-manual-{path.rsplit('/', 1)[-1]}")

    def test_individual_and_both_wheel_routes_use_server_configured_rpm(self):
        runtime = FakeRuntime(6.0)
        expected = {
            "/api/manual/left/forward": (6.0, 0.0, "left-forward"),
            "/api/manual/left/reverse": (-6.0, 0.0, "left-reverse"),
            "/api/manual/right/forward": (0.0, 6.0, "right-forward"),
            "/api/manual/right/reverse": (0.0, -6.0, "right-reverse"),
            "/api/manual/both/forward": (6.0, 6.0, "both-forward"),
            "/api/manual/both/reverse": (-6.0, -6.0, "both-reverse"),
        }
        for path, values in expected.items():
            with self.subTest(path=path):
                status, body = self.request(runtime, path)
                self.assertEqual((status, body), (200, {"ok": True}))
                command = runtime.commands[-1]
                self.assertEqual(
                    (command.left_rpm, command.right_rpm, command.source),
                    (values[0], values[1], f"web-manual-{values[2]}"),
                )

    def test_direction_routes_accept_one_finite_positive_rpm_within_configured_maximum(self):
        runtime = FakeRuntime(6.0)
        status, body = self.request(runtime, "/api/manual/both/forward?rpm=7.5")
        self.assertEqual((status, body), (200, {"ok": True}))
        command = runtime.commands[-1]
        self.assertEqual((command.left_rpm, command.right_rpm), (7.5, 7.5))

    def test_direction_routes_reject_malformed_multiple_nonfinite_or_out_of_range_rpm(self):
        rejected = (
            "/api/manual/both/forward?rpm=",
            "/api/manual/both/forward?rpm=NaN",
            "/api/manual/both/forward?rpm=Infinity",
            "/api/manual/both/forward?rpm=-1",
            "/api/manual/both/forward?rpm=10.1",
            "/api/manual/both/forward?rpm=2&rpm=3",
            "/api/manual/both/forward?rpm=2&other=3",
            "/api/manual/both/forward?rpm",
        )
        for path in rejected:
            with self.subTest(path=path):
                runtime = FakeRuntime(6.0)
                status, body = self.request(runtime, path)
                self.assertEqual(status, 409)
                self.assertIsInstance(body["error"], str)
                self.assertEqual(runtime.commands, [])

    def test_zero_hold_route_uses_the_same_manual_runtime_gate_without_client_rpm(self):
        runtime = FakeRuntime(0.0)
        status, body = self.request(runtime, "/api/manual/hold")
        self.assertEqual((status, body), (200, {"ok": True}))
        command = runtime.commands[-1]
        self.assertEqual((command.left_rpm, command.right_rpm, command.source), (0.0, 0.0, "web-manual-hold"))

    def test_hold_route_rejects_client_rpm_without_admitting_a_command(self):
        runtime = FakeRuntime(6.0)
        status, body = self.request(runtime, "/api/manual/hold?rpm=6")
        self.assertEqual(status, 409)
        self.assertIn("accepterar inte RPM", body["error"])
        self.assertEqual(runtime.commands, [])

    def test_dashboard_manual_controls_are_held_and_never_arm(self):
        from field_control.web import DASHBOARD_HTML

        for path in (
            "/api/manual/left/forward", "/api/manual/left/reverse",
            "/api/manual/right/forward", "/api/manual/right/reverse",
            "/api/manual/both/forward", "/api/manual/both/reverse",
        ):
            self.assertIn(f'data-manual-path="{path}"', DASHBOARD_HTML)
        self.assertIn("setInterval(sendManual,100)", DASHBOARD_HTML)
        self.assertIn("pointerdown", DASHBOARD_HTML)
        for event in ("pointerup", "pointercancel", "pointerleave", "visibilitychange"):
            self.assertIn(event, DASHBOARD_HTML)
        self.assertIn("fetch('/api/stop',{method:'POST'})", DASHBOARD_HTML)
        self.assertIn("Manual request failed: ${error.message}; STOP sent", DASHBOARD_HTML)
        self.assertIn("/api/manual/hold", DASHBOARD_HTML)
        self.assertIn("function releaseManual(pointerId){if(manual.active&&manual.pointerId===pointerId){manual.pointerId=null;manual.path='/api/manual/hold';sendManual();}}", DASHBOARD_HTML)
        self.assertIn("function stopManualIfActive(){if(manual.active||manual.inFlight||manual.pointerId!==null||manual.path!==null)stopManual();}", DASHBOARD_HTML)
        self.assertIn("window.addEventListener('blur',stopManualIfActive)", DASHBOARD_HTML)
        self.assertIn("if(document.hidden)stopManualIfActive()", DASHBOARD_HTML)
        self.assertNotIn("window.addEventListener('blur',stopManual);", DASHBOARD_HTML)
        self.assertNotIn("/api/arm-motor-output", DASHBOARD_HTML)

    def test_dashboard_places_speed_and_manual_controls_in_requested_safe_order(self):
        from field_control.web import DASHBOARD_HTML

        self.assertIn('class="control-layout"', DASHBOARD_HTML)
        self.assertIn('id="rpm" type="number" min="0.01" step="0.1"', DASHBOARD_HTML)
        ordered = (
            "id=\"manual-mode\"", "START AUTO", "id=\"rpm\"",
            'data-manual-path="/api/manual/both/forward"',
            'data-manual-path="/api/manual/both/reverse"',
            'data-manual-path="/api/manual/left/forward"',
            'data-manual-path="/api/manual/right/forward"',
            'data-manual-path="/api/manual/left/reverse"',
            'data-manual-path="/api/manual/right/reverse"',
            'class="stop" id="stop"',
        )
        positions = [DASHBOARD_HTML.index(item) for item in ordered]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("Desired speed (wheel RPM)", DASHBOARD_HTML)
        for item in ("Row following", "Max IMU-only navigation", "Pick trigger timeout",
                     "In-row turn enabled", "New-row direction", "Rows to harvest"):
            self.assertIn(item, DASHBOARD_HTML)

    def test_dashboard_configuration_panel_is_restart_staged_and_has_save_select(self):
        from field_control.web import DASHBOARD_HTML
        for item in ("/api/config", "/api/config/save", "/api/config/select", "/api/config/restart", "apply on restart",
                     "Reload/restart from configuration", "vision.x_goal", "motor_turns_per_wheel_turn", "acceleration is fixed and verified"):
            self.assertIn(item, DASHBOARD_HTML)

    def test_dashboard_restart_waits_for_process_replacement_before_reload(self):
        from field_control.web import DASHBOARD_HTML
        self.assertIn("const restartProbeDelayMs=250, restartProbeTimeoutMs=1000, restartReconnectDeadlineMs=90000;", DASHBOARD_HTML)
        self.assertIn("const controller=new AbortController();", DASHBOARD_HTML)
        self.assertIn("let speedInitialized=false, profileCandidate=null, gearRatio=null, manualSpeedStatus=null, dashboardInstanceId=null;", DASHBOARD_HTML)
        self.assertIn("else if(status.instance_id!==previousInstanceId){\n      window.location.reload();", DASHBOARD_HTML)
        self.assertIn("const previousStatus=dashboardInstanceId?{instance_id:dashboardInstanceId}:await restartStatus();", DASHBOARD_HTML)
        self.assertIn("Restarting with ${data.selected}; reconnecting automatically", DASHBOARD_HTML)

    def test_dashboard_has_compact_status_and_horizontal_in_row_checkbox(self):
        from field_control.web import DASHBOARD_HTML
        self.assertIn('class="check-row"', DASHBOARD_HTML)
        self.assertIn('class="panel compact-status"', DASHBOARD_HTML)
        for heading in ("Runtime", "Sensors", "Heading", "Vision and odometry"):
            self.assertIn(f"<h3>{heading}</h3>", DASHBOARD_HTML)
        self.assertNotIn("A4 awaits actual encoder angles.", DASHBOARD_HTML)
        self.assertNotIn("A4 turns blocked.", DASHBOARD_HTML)

    def test_every_configuration_rpm_field_is_wheel_side_and_saved_once_at_8_to_1(self):
        from field_control.web import DASHBOARD_HTML
        self.assertIn("function isRpmPath(path){return path.endsWith('_rpm');}", DASHBOARD_HTML)
        self.assertIn("isRpmPath(path)?`${path} (wheel RPM)`", DASHBOARD_HTML)
        self.assertIn("isRpmPath(path)?Number(value)/gearRatio:value", DASHBOARD_HTML)
        self.assertIn("if(isRpmPath(el.dataset.path))value*=gearRatio", DASHBOARD_HTML)
        # The set is intentionally explicit to make a newly added RPM field
        # fail review unless it retains the shared `_rpm` convention.
        for name in ("manual_rpm", "max_rpm", "auto_base_rpm", "search_speed_rpm",
                     "turn_speed_rpm", "max_vision_correction_rpm", "max_heading_correction_rpm"):
            self.assertTrue(name.endswith("_rpm"))

    def test_dashboard_sends_selected_rpm_on_nonzero_manual_requests_without_overwriting_edits(self):
        from field_control.web import DASHBOARD_HTML

        self.assertIn("function manualRequestPath(path)", DASHBOARD_HTML)
        self.assertIn("if(path==='/api/manual/hold')return path", DASHBOARD_HTML)
        self.assertIn("`${path}?rpm=${encodeURIComponent(rpm)}`", DASHBOARD_HTML)
        self.assertIn("if(!speedInitialized){rpmInput.value=String(configuredRpm);speedInitialized=true;}", DASHBOARD_HTML)
        self.assertIn("rpmInput.max=String(maxRpm)", DASHBOARD_HTML)

    def test_dashboard_waits_for_profile_ratio_before_initialising_wheel_rpm(self):
        from field_control.web import DASHBOARD_HTML

        # A status response can arrive before the asynchronous profile load.
        # In that case its motor-side RPM must not become the displayed wheel
        # RPM.  The saved status is converted only after the ratio is known.
        self.assertIn("let speedInitialized=false, profileCandidate=null, gearRatio=null, manualSpeedStatus=null, dashboardInstanceId=null;", DASHBOARD_HTML)
        self.assertIn("gearRatio=Number.isFinite(ratio)&&ratio>0?ratio:null;", DASHBOARD_HTML)
        self.assertIn("if(manualSpeedStatus)configureManualSpeed(manualSpeedStatus);", DASHBOARD_HTML)
        self.assertIn("manualSpeedStatus=status;\n  if(!Number.isFinite(gearRatio)||gearRatio<=0)return;", DASHBOARD_HTML)

    def test_dashboard_uses_bounded_cache_free_snapshot_polling_not_mjpeg(self):
        from field_control.web import DASHBOARD_HTML

        for view in ("raw", "overlay", "buds", "leaves"):
            self.assertIn(f'data-snapshot-view="{view}"', DASHBOARD_HTML)
        self.assertNotIn('src="/stream/raw"', DASHBOARD_HTML)
        self.assertIn("const snapshotPollMs=100;", DASHBOARD_HTML)
        self.assertIn("if(snapshot.inFlight)return;", DASHBOARD_HTML)
        self.assertIn("fetch(`/snapshot/${view}?t=${Date.now()}`,{cache:'no-store'})", DASHBOARD_HTML)
        self.assertIn("URL.createObjectURL(await response.blob())", DASHBOARD_HTML)

    def test_snapshot_client_reclaims_rapid_superseded_urls_only_after_replacement_loads(self):
        from field_control.web import DASHBOARD_HTML

        # Regression sequence for the dashboard's pending/current URL state:
        # rapid A -> B -> C replies before an image load must discard A and B;
        # the displayed C remains valid until D has loaded.
        pending = current = None; revoked = []
        def reply(url):
            nonlocal pending
            superseded = pending; pending = url
            if superseded: revoked.append(superseded)
        def loaded(url):
            nonlocal pending, current
            if pending != url: return
            previous = current; current = pending; pending = None
            if previous: revoked.append(previous)

        reply("A"); reply("B"); reply("C")
        self.assertEqual(revoked, ["A", "B"])
        loaded("C"); self.assertEqual(current, "C")
        reply("D"); self.assertNotIn("C", revoked)
        loaded("D")
        self.assertEqual((current, pending, revoked), ("D", None, ["A", "B", "C"]))

        # Keep that lifecycle mechanically tied to the actual client code.
        self.assertIn("const snapshot={inFlight:false,currentUrl:null,pendingUrl:null};", DASHBOARD_HTML)
        self.assertIn("image.addEventListener('load',()=>{", DASHBOARD_HTML)
        self.assertIn("if(!nextUrl||image.currentSrc!==nextUrl)return;", DASHBOARD_HTML)
        self.assertIn("const supersededUrl=snapshot.pendingUrl;", DASHBOARD_HTML)
        self.assertIn("if(supersededUrl)URL.revokeObjectURL(supersededUrl);", DASHBOARD_HTML)
        self.assertIn("if(previousUrl)URL.revokeObjectURL(previousUrl);", DASHBOARD_HTML)
        self.assertNotIn("image.onload=", DASHBOARD_HTML)

    def test_dashboard_disables_mode_selection_for_ready_physical_manual_standby(self):
        from field_control.web import DASHBOARD_HTML

        self.assertIn('id="manual-mode"', DASHBOARD_HTML)
        self.assertIn("const manualReady=p.mode==='MANUAL'&&p.physical_web_standby.active", DASHBOARD_HTML)
        self.assertIn("document.getElementById('manual-mode').disabled=manualReady", DASHBOARD_HTML)
        self.assertIn("Manual is already ready. Hold a direction button; do not press MANUAL.", DASHBOARD_HTML)

    def test_logical_forward_is_signed_once_by_verified_remote_a2_profile(self):
        from remote_control.config import ControlConfig
        from remote_control.physical import encode_speed_command

        runtime = FakeRuntime(6.0)
        self.request(runtime, "/api/manual/forward")
        command = runtime.commands[-1]; profile = ControlConfig()
        left = encode_speed_command(profile.left_motor_id, command.left_rpm * profile.left_forward_sign)
        right = encode_speed_command(profile.right_motor_id, command.right_rpm * profile.right_forward_sign)
        left_raw = int.from_bytes(left.data[4:8], "little", signed=True)
        right_raw = int.from_bytes(right.data[4:8], "little", signed=True)
        self.assertGreater(left_raw, 0)
        self.assertLess(right_raw, 0)

    def test_zero_rpm_and_runtime_manual_rejections_are_conflicts_without_command(self):
        runtime = FakeRuntime(0.0)
        status, _body = self.request(runtime, "/api/manual/forward")
        self.assertEqual(status, 409); self.assertEqual(runtime.commands, [])
        for message in ("manuellt kommando kräver MANUAL", "motorutgången är avstängd",
                        "manuell control-lease saknas", "runtime stängs"):
            with self.subTest(message=message):
                rejected = FakeRuntime(6.0, ValueError(message))
                status, body = self.request(rejected, "/api/manual/forward")
                self.assertEqual(status, 409); self.assertEqual(body["error"], message)
                self.assertEqual(rejected.commands, [])

    def test_browser_has_no_motor_arm_route(self):
        runtime = FakeRuntime(6.0)
        status, _body = self.request(runtime, "/api/arm-motor-output")
        self.assertEqual(status, 404)
        self.assertEqual(runtime.commands, [])

    def test_status_exposes_manual_default_and_maximum_for_the_speed_input(self):
        from field_control.diagnostics import status_payload

        class Runtime:
            config = SimpleNamespace(
                vision=SimpleNamespace(x_goal=.5), processing_width=320,
                control_lease_timeout_s=.2, manual_rpm=6.0, max_rpm=10.0,
            )
            motor = SimpleNamespace(fault_reason=None)
            lease = SimpleNamespace(valid=lambda _token: False)
            heading = SimpleNamespace(reference=SimpleNamespace(reliable_distance_m=0.0))
            events = SimpleNamespace(recent=lambda: [])
            def status(self):
                return SimpleNamespace(
                    observation=None, running=True, mode="MANUAL", state="MANUAL",
                    snapshot=SimpleNamespace(reason="", fault=None, row_number=1,
                                             pass_number=1, auto_start_remaining_s=0.0,
                                             search_distance_m=0.0, post_pick_distance_m=0.0,
                                             marker_armed=False),
                    fault=None, motor_output_armed=False, last_command=None,
                    last_admitted_nonzero_command=None,
                )
            def web_standby_status(self): return False, None

        payload = status_payload(Runtime())
        self.assertEqual(payload["manual_rpm"], 6.0)
        self.assertEqual(payload["max_rpm"], 10.0)


if __name__ == "__main__":
    unittest.main()
