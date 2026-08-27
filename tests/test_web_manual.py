from __future__ import annotations

import json
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from field_control.web import DiagnosticsServer


class FakeRuntime:
    def __init__(self, rpm=6.0, error: Exception | None = None):
        self.config = SimpleNamespace(manual_rpm=rpm, stream_fps=5)
        self.commands = []; self.error = error
        self.images = {}
    def manual_command(self, command):
        if self.error is not None: raise self.error
        self.commands.append(command)
    def select_manual(self): pass
    def select_auto(self): pass
    def start_auto(self): pass
    def stop(self): pass
    def latest_image(self, view): return self.images.get(view)


class ManualWebTests(unittest.TestCase):
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

    def test_direction_routes_use_only_fixed_motor_side_manual_rpm(self):
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

    def test_zero_hold_route_uses_the_same_manual_runtime_gate_without_client_rpm(self):
        runtime = FakeRuntime(0.0)
        status, body = self.request(runtime, "/api/manual/hold")
        self.assertEqual((status, body), (200, {"ok": True}))
        command = runtime.commands[-1]
        self.assertEqual((command.left_rpm, command.right_rpm, command.source), (0.0, 0.0, "web-manual-hold"))

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
        self.assertNotIn("/api/arm-motor-output", DASHBOARD_HTML)

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


if __name__ == "__main__":
    unittest.main()
