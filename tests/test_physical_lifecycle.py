"""Mock-only deployment gating for the verified physical path."""
from __future__ import annotations

import math
import inspect
import sys
import threading
import types
import unittest
from unittest.mock import patch

from field_control.app import FieldControlApplication
from field_control.config import PhysicalCanConfig, RuntimeConfig
from field_control.sources import LatestValue
from field_control.verified_motor_boundary import _VerifiedPhysicalMotorBoundary, open_verified_boundary
from field_control.lease import ControlLease
from field_control.control import WheelCommand


class Sink:
    def __init__(self, *, reject_callback: bool = False):
        self.reject_callback = reject_callback; self.closed = False; self.callback = None
    def set_fault_callback(self, callback):
        if self.reject_callback: raise RuntimeError("callback setup failed")
        self.callback = callback
    def command(self, left, right, reason): pass
    def stop_all(self, reason): pass
    def stop_and_settle_for_restart(self): pass
    def stop_and_settle_and_close(self): self.close()
    def close(self): self.closed = True


def deployment_config(max_rpm=20.0):
    return RuntimeConfig(
        stream_enabled=False, max_rpm=max_rpm,
        physical_can=PhysicalCanConfig(
            True, "can0", "observed-rmdx-same-id", "/dev/serial/by-id/test-canable", True, True,
        ),
    )


class VerifiedDeploymentTests(unittest.TestCase):
    def test_web_standby_releases_drive_lease_and_claims_a_fresh_one(self):
        sink = Sink()
        lease = ControlLease(timeout_s=.3)
        boundary = _VerifiedPhysicalMotorBoundary(sink, lease, max_rpm=20)
        first = lease.acquire(); boundary.arm(first)
        boundary.enter_web_standby(first)
        self.assertTrue(boundary.armed)
        self.assertTrue(boundary.web_standby_active)
        self.assertFalse(lease.valid(first))

        with self.assertRaises(Exception):
            boundary.command(WheelCommand(1, 1, "must-not-drive"), first)
        self.assertFalse(boundary.web_standby_active)
        self.assertFalse(boundary.armed)

        # A new handoff can only drive after a fresh ordinary lease claim.
        second = lease.acquire(); boundary.arm(second); boundary.enter_web_standby(second)
        fresh = lease.acquire(); boundary.claim_web_standby(fresh)
        boundary.command(WheelCommand(1, -1, "claimed"), fresh)
        self.assertFalse(boundary.web_standby_active)

    def test_stop_wins_over_web_standby_claim_race(self):
        sink = Sink(); lease = ControlLease(timeout_s=.3)
        boundary = _VerifiedPhysicalMotorBoundary(sink, lease, max_rpm=20)
        first = lease.acquire(); boundary.arm(first); boundary.enter_web_standby(first)
        fresh = lease.acquire(); errors = []
        claim = threading.Thread(target=lambda: self._capture(lambda: boundary.claim_web_standby(fresh), errors))
        claim.start(); boundary.stop_all("operator STOP"); claim.join(.3)
        self.assertFalse(claim.is_alive())
        self.assertFalse(boundary.armed)
        self.assertFalse(boundary.web_standby_active)
        self.assertTrue(not errors or isinstance(errors[0], Exception))

    @staticmethod
    def _capture(callback, errors):
        try: callback()
        except Exception as exc: errors.append(exc)
    def test_app_start_failure_closes_verified_output_once(self):
        class PassiveSource:
            def __init__(self, *_args): self.latest = LatestValue()
            def start(self): pass
            def stop(self): pass

        class FailingOdometry(PassiveSource):
            def start(self): raise RuntimeError("encoder startup failed")

        class RecordingPhysical:
            def __init__(self, lease):
                self.control_lease = lease; self.armed = False; self.fault_reason = None; self.events = []; self.closing = False
            def arm(self, _token): pass
            def command(self, *_args): pass
            def stop_all(self, _reason): pass
            def hold_stopped(self, _reason, _token=None): pass
            def encoder_backend(self): return object()
            def _begin_close(self):
                if self.closing: return False
                self.closing = True; return True
            def _finish_close(self): self.events.extend(("settle", "close"))
            def close(self):
                if self._begin_close(): self._finish_close()

        created = []
        def opened(**kwargs):
            boundary = RecordingPhysical(kwargs["lease"]); created.append(boundary); return boundary

        with patch("field_control.app.CameraSource", PassiveSource), \
             patch("field_control.app.ImuSource", PassiveSource), \
             patch("field_control.app.OdometrySource", FailingOdometry), \
             patch("field_control.app.open_verified_boundary", side_effect=opened):
            app = FieldControlApplication(deployment_config())
            with self.assertRaisesRegex(RuntimeError, "encoder startup failed"):
                app.start()
        self.assertEqual(len(created), 1)
        self.assertEqual(created[0].events, ["settle", "close"])

    def test_default_application_never_calls_verified_open(self):
        with patch("field_control.app.open_verified_boundary") as opened:
            app = FieldControlApplication(RuntimeConfig(stream_enabled=False))
        self.assertFalse(opened.called)
        app.close()

    def test_explicit_application_uses_verified_open_without_public_factory_seam(self):
        opened_calls = []
        def opened(**kwargs):
            opened_calls.append(kwargs)
            from field_control.verified_motor_boundary import _VerifiedPhysicalMotorBoundary
            return _VerifiedPhysicalMotorBoundary(Sink(), kwargs["lease"], max_rpm=kwargs["max_rpm"])
        with patch("field_control.app.open_verified_boundary", side_effect=opened):
            app = FieldControlApplication(deployment_config())
        self.assertEqual(len(opened_calls), 1)
        self.assertEqual(opened_calls[0]["channel"], "can0")
        self.assertNotIn("sink_factory", opened_calls[0])
        app.close()

    def test_application_public_api_has_no_sink_factory(self):
        self.assertNotIn("verified_sink_factory", inspect.signature(FieldControlApplication).parameters)

    def test_invalid_rpm_fails_before_remote_control_can_be_imported(self):
        for rpm in (81.0, math.nan):
            with self.assertRaises(ValueError):
                open_verified_boundary(
                    channel="can0", slcan_device="/dev/serial/by-id/test", max_rpm=rpm,
                    lease=__import__("field_control.lease", fromlist=["ControlLease"]).ControlLease(),
                )

    def test_adapter_construction_failure_closes_created_sink(self):
        class FakePhysical(Sink):
            instance = None
            @classmethod
            def open_for_raised_wheel_test(cls, _config): return cls.instance
        sink = FakePhysical(reject_callback=True); FakePhysical.instance = sink
        package = types.ModuleType("remote_control")
        config_module = types.ModuleType("remote_control.config")
        config_module.ControlConfig = lambda **kwargs: types.SimpleNamespace(**kwargs)
        physical_module = types.ModuleType("remote_control.physical")
        physical_module.PhysicalCanMotors = FakePhysical
        with patch.dict(sys.modules, {
            "remote_control": package, "remote_control.config": config_module,
            "remote_control.physical": physical_module,
        }):
            with self.assertRaises(RuntimeError):
                open_verified_boundary(
                    channel="can0", slcan_device="/dev/serial/by-id/test", max_rpm=20,
                    lease=__import__("field_control.lease", fromlist=["ControlLease"]).ControlLease(),
                )
        self.assertTrue(sink.closed)


if __name__ == "__main__":
    unittest.main()
