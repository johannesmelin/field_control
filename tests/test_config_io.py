from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from field_control.app import FieldControlApplication
from field_control.config import RuntimeConfig
from field_control.config_io import (dump_runtime_config, load_runtime_config,
                                     runtime_config_from_dict, runtime_config_to_dict)


class ConfigIoTests(unittest.TestCase):
    def test_round_trip_preserves_nested_config_and_safe_disabled_output(self):
        config = RuntimeConfig(imu_sample_hz=77, manual_rpm=4.5, log_level="DEBUG", stream_enabled=False)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "field.json"
            dump_runtime_config(config, path)
            loaded = load_runtime_config(path)
            document = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(loaded, config)
        self.assertFalse(document["physical_can"]["enabled"])
        self.assertEqual(document["imu_sample_hz"], 77)
        self.assertEqual(document["manual_rpm"], 4.5)
        minimal = runtime_config_from_dict({"physical_can": {"enabled": False}})
        self.assertFalse(minimal.physical_can.enabled)

    def test_rejects_unknown_keys_nonfinite_and_boolean_numbers(self):
        document = runtime_config_to_dict(RuntimeConfig())
        document["unexpected"] = 1
        with self.assertRaises(ValueError): runtime_config_from_dict(document)
        document = runtime_config_to_dict(RuntimeConfig())
        document["vision"]["unknown"] = 1
        with self.assertRaises(ValueError): runtime_config_from_dict(document)
        document = runtime_config_to_dict(RuntimeConfig())
        document["manual_rpm"] = True
        with self.assertRaises(ValueError): runtime_config_from_dict(document)
        document = runtime_config_to_dict(RuntimeConfig())
        document["vision"]["x_goal"] = float("nan")
        with self.assertRaises(ValueError): runtime_config_from_dict(document)

    def test_rejects_invalid_physical_opt_in_and_json_nan(self):
        document = runtime_config_to_dict(RuntimeConfig())
        document["physical_can"]["enabled"] = True
        with self.assertRaises(ValueError): runtime_config_from_dict(document)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text('{"max_rpm": NaN}', encoding="utf-8")
            with self.assertRaises(ValueError): load_runtime_config(path)

    def test_physical_by_id_path_requires_exactly_one_basename(self):
        document = runtime_config_to_dict(RuntimeConfig())
        physical = document["physical_can"]
        physical.update({
            "enabled": True, "channel": "can0", "reply_profile": "observed-rmdx-same-id",
            "confirm_physical_stop_tested": True, "confirm_wheels_raised": True,
        })
        document["max_rpm"] = 1.0
        physical["slcan_device"] = "/dev/serial/by-id/usb-CANable_123"
        self.assertEqual(runtime_config_from_dict(document).physical_can.slcan_device, physical["slcan_device"])
        for path in ("/dev/serial/by-id/", "/dev/serial/by-id/.", "/dev/serial/by-id/..",
                     "/dev/serial/by-id/nested/usb-CANable_123"):
            with self.subTest(path=path):
                physical["slcan_device"] = path
                with self.assertRaises(ValueError): runtime_config_from_dict(document)

    def test_ground_context_booleans_are_strict_default_false_and_round_trip(self):
        # Old documents omit these trailing fields and retain the disabled,
        # safe defaults rather than being interpreted as a ground deployment.
        legacy = {"physical_can": {"enabled": False}}
        parsed = runtime_config_from_dict(legacy)
        self.assertFalse(parsed.physical_can.confirm_ground_test)
        self.assertFalse(parsed.physical_can.confirm_ground_clear)
        self.assertFalse(parsed.physical_can.confirm_emergency_stop_ready)

        document = runtime_config_to_dict(RuntimeConfig())
        physical = document["physical_can"]
        physical.update({
            "enabled": True, "channel": "can0", "reply_profile": "observed-rmdx-same-id",
            "slcan_device": "/dev/serial/by-id/usb-CANable_123",
            "confirm_physical_stop_tested": True,
            "confirm_ground_test": True, "confirm_ground_clear": True,
            "confirm_emergency_stop_ready": True,
        })
        document["max_rpm"] = 1.0
        self.assertTrue(runtime_config_from_dict(document).physical_can.confirm_ground_test)
        for field in ("confirm_ground_test", "confirm_ground_clear", "confirm_emergency_stop_ready"):
            invalid = runtime_config_to_dict(RuntimeConfig())
            invalid["physical_can"][field] = "true"
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "boolesk"):
                runtime_config_from_dict(invalid)

    def test_imu_rate_is_validated_and_wired_to_combined_oak_backend(self):
        with self.assertRaises(ValueError): RuntimeConfig(imu_sample_hz=True).validate()
        with self.assertRaises(ValueError): RuntimeConfig(imu_sample_hz=0).validate()
        created = []
        class Backend:
            def frames(self): return iter(())
            def samples(self): return iter(())
            def close(self): pass
        def combined(*args):
            created.append(args); return Backend()
        with patch("field_control.app.DepthAICombinedBackend", side_effect=combined):
            app = FieldControlApplication(RuntimeConfig(imu_sample_hz=77, stream_enabled=False))
            app.close()
        self.assertEqual(created[0][-1], 77)

    def test_log_level_is_strict(self):
        with self.assertRaises(ValueError): RuntimeConfig(log_level="verbose").validate()


if __name__ == "__main__":
    unittest.main()
