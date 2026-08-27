from __future__ import annotations

import unittest

from field_control.turn_phase_a_hil import TurnPhaseARequest, phase_a_config, run_turn_phase_a
from field_control.turn_phase_a_visible_hil import (
    TurnPhaseAVisibleRequest, phase_a_visible_config, run_turn_phase_a_visible,
)


class TurnPhaseAVisibleCompatibilityTests(unittest.TestCase):
    def test_legacy_visible_entrypoint_is_the_canonical_a4_hil(self):
        self.assertIs(TurnPhaseAVisibleRequest, TurnPhaseARequest)
        self.assertIs(phase_a_visible_config, phase_a_config)
        self.assertIs(run_turn_phase_a_visible, run_turn_phase_a)


if __name__ == "__main__":
    unittest.main()
