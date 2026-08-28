from __future__ import annotations

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from field_control.cli import _restart_argv, main
from field_control.config import PhysicalCanConfig, RuntimeConfig
from field_control.config_io import dump_runtime_config


class CliTests(unittest.TestCase):
    def setUp(self) -> None:
        """Keep CLI cases independent of an operator's saved profiles."""
        self._profiles = tempfile.TemporaryDirectory()
        self._profiles_patch = patch(
            "field_control.cli.default_profiles_dir",
            return_value=Path(self._profiles.name),
        )
        self._profiles_patch.start()

    def tearDown(self) -> None:
        self._profiles_patch.stop()
        self._profiles.cleanup()

    @staticmethod
    def physical_config() -> RuntimeConfig:
        return RuntimeConfig(stream_enabled=False, max_rpm=1,
                             physical_can=PhysicalCanConfig(True, "can0", "observed-rmdx-same-id",
                                                            "/dev/serial/by-id/canable", True, True))

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
        config = self.physical_config()
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

    def test_physical_web_requires_distinct_opt_in_confirmation_and_loopback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "physical.json"; dump_runtime_config(self.physical_config(), path)
            for argv in (
                ["--config", str(path)],
                ["--config", str(path), "--physical-web"],
                ["--config", str(path), "--physical-web", "--confirm-physical-web", "--host", "0.0.0.0"],
                ["--config", str(path), "--physical-web", "--confirm-physical-web", "--host", "localhost"],
            ):
                with self.subTest(argv=argv), patch("field_control.cli.FieldControlApplication") as application:
                    self.assertEqual(main(argv), 2)
                    application.assert_not_called()

    def test_physical_web_starts_disarmed_without_local_arm_option(self):
        class ImmediateStop:
            def wait(self, _timeout): return True
            def set(self): pass
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "physical.json"; dump_runtime_config(self.physical_config(), path)
            with patch("field_control.cli.FieldControlApplication") as application, \
                 patch("field_control.cli.threading.Event", return_value=ImmediateStop()):
                self.assertEqual(main(["--config", str(path), "--physical-web", "--confirm-physical-web"]), 0)
            application.return_value.start.assert_called_once()
            application.return_value.runtime.arm_motor_output.assert_not_called()
            application.return_value.close.assert_called_once()

    def test_physical_web_arm_is_local_and_after_started_manual_runtime(self):
        class ImmediateStop:
            def wait(self, _timeout): return True
            def set(self): pass
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "physical.json"; dump_runtime_config(self.physical_config(), path)
            calls: list[str] = []
            with patch("field_control.cli.FieldControlApplication") as application, \
                 patch("field_control.cli.threading.Event", return_value=ImmediateStop()):
                runtime = application.return_value.runtime
                runtime.status.return_value = SimpleNamespace(state="MANUAL", motor_output_armed=False)
                application.return_value.start.side_effect = lambda: calls.append("start")
                runtime.arm_motor_output_for_web_standby.side_effect = lambda: calls.append("standby")
                self.assertEqual(main(["--config", str(path), "--physical-web", "--confirm-physical-web",
                                       "--arm-motor-output"]), 0)
            self.assertEqual(calls, ["start", "standby"])
            application.return_value.close.assert_called_once()

    def test_physical_web_arm_rejection_closes_application(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "physical.json"; dump_runtime_config(self.physical_config(), path)
            with patch("field_control.cli.FieldControlApplication") as application:
                runtime = application.return_value.runtime
                runtime.status.return_value = SimpleNamespace(state="AUTO_ROW_FOLLOW", motor_output_armed=False)
                self.assertEqual(main(["--config", str(path), "--physical-web", "--confirm-physical-web",
                                       "--arm-motor-output"]), 2)
            runtime.arm_motor_output.assert_not_called()
            application.return_value.close.assert_called_once()

    def test_web_requested_restart_closes_then_execs_without_arm_option(self):
        class RestartWait:
            def wait(self, _timeout): return False
            def set(self): pass
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "physical.json"; dump_runtime_config(self.physical_config(), path)
            with patch("field_control.cli.FieldControlApplication") as application, \
                 patch("field_control.cli.threading.Event", return_value=RestartWait()), \
                 patch("field_control.cli.os.execv", side_effect=SystemExit) as execv:
                application.return_value.web.restart_requested.return_value = True
                application.return_value.runtime.status.return_value = SimpleNamespace(state="MANUAL", motor_output_armed=False)
                with self.assertRaises(SystemExit):
                    main(["--config", str(path), "--physical-web", "--confirm-physical-web", "--arm-motor-output"])
            application.return_value.close.assert_called_once()
            argv = execv.call_args.args[1]
            self.assertIn("--arm-motor-output", argv)

    def test_restart_arguments_remove_one_run_profile_in_both_supported_forms(self):
        self.assertEqual(
            _restart_argv(["--config", "deployment.json", "--profile", "old.json", "--arm-motor-output"]),
            ["--config", "deployment.json", "--arm-motor-output"],
        )
        self.assertEqual(
            _restart_argv(["--config", "deployment.json", "--profile=old.json"]),
            ["--config", "deployment.json"],
        )
        self.assertNotIn("--arm-motor-output", _restart_argv(["--config", "deployment.json"]))


if __name__ == "__main__":
    unittest.main()
