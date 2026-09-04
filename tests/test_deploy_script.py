from __future__ import annotations

from pathlib import Path
import unittest


class DeployScriptTests(unittest.TestCase):
    def test_preflight_recognizes_both_documented_runtime_forms(self):
        script = (Path(__file__).resolve().parents[1] / "scripts" / "deploy.sh").read_text(encoding="utf-8")
        self.assertIn("-m[[:space:]]field_control[.]cli", script)
        self.assertIn("([^[:space:]]*/)?field-control", script)


if __name__ == "__main__":
    unittest.main()
