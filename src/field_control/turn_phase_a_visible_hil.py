"""Deprecated compatibility entry point for the canonical A4 Phase-A HIL.

This name previously performed a fixed A2 velocity observation. It now
delegates to the one bounded, target-confirmed A4 raised-wheel test.
"""
from .turn_phase_a_hil import (
    TurnPhaseARequest as TurnPhaseAVisibleRequest,
    TurnPhaseAResult as TurnPhaseAVisibleResult,
    main,
    phase_a_config as phase_a_visible_config,
    run_turn_phase_a as run_turn_phase_a_visible,
)

__all__ = ["TurnPhaseAVisibleRequest", "TurnPhaseAVisibleResult", "phase_a_visible_config",
           "run_turn_phase_a_visible", "main"]
