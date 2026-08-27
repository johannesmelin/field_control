from __future__ import annotations

import unittest

from field_control.turn_phase_a_hil import TurnPhaseARequest, phase_a_config, run_turn_phase_a
from field_control.turn_phase_a_long_hil import (
    TurnPhaseALongRequest, phase_a_long_config, run_turn_phase_a_long,
)


class TurnPhaseALongCompatibilityTests(unittest.TestCase):
    def test_legacy_long_entrypoint_is_the_canonical_a4_hil(self):
        self.assertIs(TurnPhaseALongRequest, TurnPhaseARequest)
        self.assertIs(phase_a_long_config, phase_a_config)
        self.assertIs(run_turn_phase_a_long, run_turn_phase_a)


if __name__ == "__main__":
    unittest.main()
