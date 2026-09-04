"""Deprecated compatibility entry point for the canonical A4 Phase-A HIL.

The former 30-second A2 observation must not remain callable after production
turns changed to verified A4 position targets. Keep the command name for
operators, but run exactly the one bounded canonical target test.
"""
from .turn_phase_a_hil import (
    TurnPhaseARequest as TurnPhaseALongRequest,
    TurnPhaseAResult as TurnPhaseALongResult,
    main,
    phase_a_config as phase_a_long_config,
    run_turn_phase_a as run_turn_phase_a_long,
)

__all__ = ["TurnPhaseALongRequest", "TurnPhaseALongResult", "phase_a_long_config",
           "run_turn_phase_a_long", "main"]
