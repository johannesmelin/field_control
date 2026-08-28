from __future__ import annotations

from dataclasses import replace
from datetime import datetime
import json
from pathlib import Path
import stat
import tempfile
import unittest

from field_control.config import PhysicalCanConfig, RuntimeConfig
from field_control.config_profiles import (list_profiles, load_selected_or_latest,
                                           load_profile, operator_profile_dict, save_profile,
                                           select_profile)


class ConfigProfileTests(unittest.TestCase):
    def test_save_is_private_and_never_contains_physical_can(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "konfigurationer"
            config = replace(RuntimeConfig(), physical_can=PhysicalCanConfig(enabled=False))
            name = save_profile(config, directory, now=datetime(2026, 8, 28, 12, 34, 56, 123))
            self.assertEqual(name, "konfig_20260828_123456.json")
            saved = json.loads((directory / name).read_text())
            self.assertNotIn("physical_can", saved)
            self.assertEqual(stat.S_IMODE((directory / name).stat().st_mode), 0o600)

    def test_latest_and_selected_merge_without_restoring_physical_deployment(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            deployment = RuntimeConfig()
            first = save_profile(replace(deployment, manual_rpm=2), directory, now=datetime(2026, 1, 1, 1, 1, 1))
            second = save_profile(replace(deployment, manual_rpm=3), directory, now=datetime(2026, 1, 1, 1, 1, 2))
            config, name = load_selected_or_latest(deployment, directory)
            self.assertEqual((name, config.manual_rpm), (second, 3))
            select_profile(first, directory)
            config, name = load_selected_or_latest(deployment, directory)
            self.assertEqual((name, config.manual_rpm), (first, 2))

    def test_symlink_profile_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp); target = directory / "outside.json"; target.write_text("{}")
            (directory / "konfig_20260101_010101.json").symlink_to(target)
            with self.assertRaises(ValueError): list_profiles(directory)

    def test_dangling_profile_and_selected_symlinks_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / "konfig_20260101_010101.json").symlink_to(directory / "missing.json")
            with self.assertRaises(ValueError): list_profiles(directory)
            (directory / "konfig_20260101_010101.json").unlink()
            (directory / "selected.json").symlink_to(directory / "missing-selected.json")
            with self.assertRaises(ValueError): load_selected_or_latest(RuntimeConfig(), directory)

    def test_operator_dict_excludes_physical_can(self):
        profile = operator_profile_dict(RuntimeConfig())
        for key in ("physical_can", "control_lease_timeout_s", "watchdog_period_s",
                    "max_control_stall_s", "physical_web_standby_timeout_s"):
            self.assertNotIn(key, profile)

    def test_profile_cannot_override_deployment_control_boundaries(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp); name = "konfig_20260101_010101.json"
            (directory / name).write_text(json.dumps({"manual_rpm": 2.0,
                "control_lease_timeout_s": 99.0, "watchdog_period_s": 99.0,
                "max_control_stall_s": 99.0, "physical_web_standby_timeout_s": 99.0}))
            deployment = RuntimeConfig(control_lease_timeout_s=.3, watchdog_period_s=.02,
                                       max_control_stall_s=.12, physical_web_standby_timeout_s=30.0)
            with self.assertRaises(ValueError): load_profile(name, deployment, directory)
