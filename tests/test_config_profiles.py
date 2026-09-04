from __future__ import annotations

from dataclasses import replace
from datetime import datetime
import json
from pathlib import Path
import stat
import tempfile
import unittest

from field_control.config import GoalRelativeZone, PhysicalCanConfig, RuntimeConfig, Zone
from field_control.config_profiles import (list_profiles, load_selected_or_latest,
                                           load_profile, operator_profile_dict, save_profile,
                                           select_profile)
from field_control.config_profiles import default_profiles_dir


class ConfigProfileTests(unittest.TestCase):
    def test_default_profiles_dir_is_at_source_tree_root(self):
        project_root = Path(__file__).resolve().parents[1]
        self.assertEqual(default_profiles_dir(), project_root / "konfigurationer")

    def test_selected_profile_takes_precedence_over_latest(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            deployment = RuntimeConfig()
            selected = save_profile(replace(deployment, manual_rpm=2), directory,
                                    now=datetime(2026, 1, 1, 1, 1, 1))
            save_profile(replace(deployment, manual_rpm=3), directory,
                         now=datetime(2026, 1, 1, 1, 1, 2))
            select_profile(selected, directory)

            config, name = load_selected_or_latest(deployment, directory)

        self.assertEqual((name, config.manual_rpm), (selected, 2))

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

    def test_selected_legacy_320_by_240_profile_migrates_to_full_fov_geometry(self):
        """Selected old profiles remain startable while adopting full CAM_B FOV."""
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            deployment = RuntimeConfig()
            name = "konfig_20260901_151942.json"
            profile = operator_profile_dict(deployment)
            profile.update({
                "processing_width": 320, "processing_height": 240,
                "stream_width": 320, "stream_height": 240,
            })
            (directory / name).write_text(json.dumps(profile))
            select_profile(name, directory)

            loaded, selected = load_selected_or_latest(deployment, directory)

        self.assertEqual(selected, name)
        self.assertEqual((loaded.processing_width, loaded.processing_height), (320, 200))
        self.assertEqual((loaded.stream_width, loaded.stream_height), (320, 200))

    def test_profile_does_not_silently_normalize_arbitrary_non_16_by_10_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            name = "konfig_20260901_151943.json"
            profile = operator_profile_dict(RuntimeConfig())
            profile.update({"processing_width": 640, "processing_height": 480})
            (directory / name).write_text(json.dumps(profile))

            with self.assertRaisesRegex(ValueError, "16:10"):
                load_profile(name, RuntimeConfig(), directory)

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

    def test_legacy_zone_profile_replaces_modern_default_shape_atomically(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp); name = "konfig_20260831_153611.json"
            profile = operator_profile_dict(RuntimeConfig())
            profile["vision"]["x_goal"] = .25
            for key, y_min in (("navigation_zone", .3), ("trigger_zone", .8),
                               ("pick_zone", .5), ("turn_marker_zone", .6)):
                profile["vision"][key] = {"x_min": .1, "x_max": .4,
                                          "y_min": y_min, "y_max": 1.0}
            (directory / name).write_text(json.dumps(profile))
            loaded = load_profile(name, RuntimeConfig(), directory)
            self.assertEqual(loaded.vision.x_goal, .25)
            self.assertEqual(loaded.vision.navigation_zone, Zone(.1, .4, .3, 1.0))
            self.assertEqual(loaded.vision.pick_zone, Zone(.1, .4, .5, 1.0))

    def test_modern_goal_relative_profile_loads_and_bad_zone_shape_is_not_merged(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp); name = "konfig_20260101_010101.json"
            profile = operator_profile_dict(RuntimeConfig())
            (directory / name).write_text(json.dumps(profile))
            self.assertEqual(load_profile(name, RuntimeConfig(), directory).vision.navigation_zone,
                             GoalRelativeZone(.3, .3, 1.0))
            profile["vision"]["navigation_zone"] = {"x_min": .1, "x_max": .4,
                                                      "y_min": .3, "y_max": 1.0,
                                                      "unexpected": 1}
            (directory / name).write_text(json.dumps(profile))
            with self.assertRaises(ValueError): load_profile(name, RuntimeConfig(), directory)

    def test_row_three_and_four_zone_profiles_replace_discriminated_shapes_atomically(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp); name = "konfig_20260903_120000.json"
            profile = operator_profile_dict(RuntimeConfig())
            # Deployment defaults are goal-relative. A recursive merge here
            # would fabricate an invalid x_distance/x_min hybrid instead of
            # accepting this deliberate legacy rectangle replacement.
            profile["vision"]["navigation_zone_3"] = {"x_min": .1, "x_max": .4,
                                                        "y_min": .3, "y_max": 1.0}
            profile["vision"]["navigation_zone_4"] = {"x_min": .6, "x_max": .9,
                                                        "y_min": .3, "y_max": 1.0}
            (directory / name).write_text(json.dumps(profile))
            loaded = load_profile(name, RuntimeConfig(), directory)
        self.assertEqual(loaded.vision.navigation_zone_3, Zone(.1, .4, .3, 1.0))
        self.assertEqual(loaded.vision.navigation_zone_4, Zone(.6, .9, .3, 1.0))

    def test_merged_legacy_cam_one_serial_normalizes_empty_new_field(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp); name = "konfig_20260903_120001.json"
            profile = operator_profile_dict(RuntimeConfig())
            profile["vision"]["camera_serial_number"] = "19443010813DFC5A00"
            # This is what a merge against a modern deployment contributes
            # when the historical profile could not contain the new key.
            profile["vision"]["cam_1_serial_number"] = ""
            (directory / name).write_text(json.dumps(profile))
            loaded = load_profile(name, RuntimeConfig(), directory)
        self.assertEqual(loaded.vision.cam_1_serial_number, "19443010813DFC5A00")
        self.assertEqual(loaded.vision.camera_serial_for(1), "19443010813DFC5A00")
