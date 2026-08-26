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
    def manual_command(self, command):
        if self.error is not None: raise self.error
        self.commands.append(command)


class ManualWebTests(unittest.TestCase):
    def request(self, runtime, path):
        with patch("field_control.web.status_payload", return_value={"ok": True}):
            server = object.__new__(DiagnosticsServer); server.runtime = runtime
            handler = object.__new__(server._handler())
            response = []; handler.path = path; handler.wfile = io.BytesIO()
            handler.send_response = lambda status: response.append(status)
            handler.send_header = lambda *_args: None
            handler.end_headers = lambda: None
            handler.do_POST()
            return response[0], json.loads(handler.wfile.getvalue())

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


if __name__ == "__main__":
    unittest.main()
