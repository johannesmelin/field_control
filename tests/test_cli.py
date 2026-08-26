from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from field_control.cli import main
from field_control.config import PhysicalCanConfig, RuntimeConfig
from field_control.config_io import dump_runtime_config


class CliTests(unittest.TestCase):
    def test_write_default_refuses_existing_without_force_and_validates(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "field.json"
            self.assertEqual(main(["--write-default-config", str(path)]), 0)
            self.assertEqual(main(["--validate-config", str(path)]), 0)
            original = path.read_text(encoding="utf-8")
            self.assertEqual(main(["--write-default-config", str(path)]), 2)
            self.assertEqual(path.read_text(encoding="utf-8"), original)
            self.assertEqual(main(["--write-default-config", str(path), "--force"]), 0)

    def test_write_default_never_follows_symlink_even_with_force(self):
        with tempfile.TemporaryDirectory() as directory:
            referent = Path(directory) / "referent.json"; referent.write_text("preserve", encoding="utf-8")
            target = Path(directory) / "link.json"; target.symlink_to(referent)
            self.assertEqual(main(["--write-default-config", str(target), "--force"]), 2)
            self.assertEqual(referent.read_text(encoding="utf-8"), "preserve")

    def test_normal_cli_rejects_physical_config_before_application_construction(self):
        config = RuntimeConfig(stream_enabled=False, max_rpm=1,
                               physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                                              "/dev/serial/by-id/canable", True, True))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "physical.json"; dump_runtime_config(config, path)
            with patch("field_control.cli.FieldControlApplication") as application:
                self.assertEqual(main(["--config", str(path)]), 2)
            application.assert_not_called()

    def test_normal_cli_starts_and_closes_diagnostics_application(self):
        class ImmediateStop:
            def wait(self, _timeout): return True
            def set(self): pass
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "safe.json"; dump_runtime_config(RuntimeConfig(stream_enabled=False), path)
            with patch("field_control.cli.FieldControlApplication") as application, \
                 patch("field_control.cli.threading.Event", return_value=ImmediateStop()):
                self.assertEqual(main(["--config", str(path), "--host", "127.0.0.2", "--port", "9000"]), 0)
            application.assert_called_once()
            application.return_value.start.assert_called_once()
            application.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
