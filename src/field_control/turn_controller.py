"""Pure tick-driven turn controller; no I/O, threads, sleep or CAN."""
from __future__ import annotations

from dataclasses import dataclass
import math

from .control import WheelCommand
from .turn import DifferentialTurnPlan


def _wrap(degrees: float) -> float: return (degrees + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class TurnObservation:
    now_s: float
    left_distance_m: float
    right_distance_m: float
    heading_deg: float | None
    heading_fresh: bool
    heading_timestamp_s: float | None
    heading_sequence: int | None


@dataclass(frozen=True)
class TurnDecision:
    command: WheelCommand | None
    terminal: bool
    succeeded: bool
    fault: str | None = None


class TurnController:
    def __init__(self, plan: DifferentialTurnPlan, *, initial_heading_deg: float, start_s: float,
                 turn_speed_motor_rpm: float, max_motor_rpm: float, timeout_s: float,
                 distance_tolerance_m: float, heading_tolerance_deg: float, heading_confirm_frames: int,
                 heading_max_age_s: float = .2) -> None:
        values = (initial_heading_deg, start_s, turn_speed_motor_rpm, max_motor_rpm, timeout_s,
                  distance_tolerance_m, heading_tolerance_deg, heading_max_age_s)
        if not all(math.isfinite(value) for value in values) or turn_speed_motor_rpm <= 0 or max_motor_rpm <= 0 or timeout_s <= 0 or distance_tolerance_m < 0 or heading_tolerance_deg < 0 or heading_confirm_frames < 1 or heading_max_age_s <= 0:
            raise ValueError("ogiltig turn-controller-konfiguration")
        self._validate_plan(plan)
        self.plan, self.initial_heading_deg, self.start_s = plan, initial_heading_deg, start_s
        self.speed = min(turn_speed_motor_rpm, max_motor_rpm)
        self.timeout_s, self.distance_tolerance_m = timeout_s, distance_tolerance_m
        self.heading_tolerance_deg, self.heading_confirm_frames = heading_tolerance_deg, heading_confirm_frames
        self.heading_max_age_s = heading_max_age_s
        self._confirmations = 0
        self._last_heading_sequence: int | None = None
        self._last_heading_timestamp_s: float | None = None
        self._terminal: TurnDecision | None = None

    @staticmethod
    def _validate_plan(plan: DifferentialTurnPlan) -> None:
        if plan.direction not in ("left", "right"):
            raise ValueError("ogiltig turn-plan direction")
        targets = (plan.left_distance_m, plan.right_distance_m)
        ratios = (plan.left_ratio, plan.right_ratio)
        if not all(math.isfinite(value) and value != 0 for value in targets):
            raise ValueError("turn-plan targets måste vara ändliga och icke-noll")
        if not all(math.isfinite(value) and 0 < abs(value) <= 1 for value in ratios):
            raise ValueError("turn-plan ratios måste vara ändliga och normaliserade")
        if any(target * ratio <= 0 for target, ratio in zip(targets, ratios)):
            raise ValueError("turn-plan ratio måste ha samma tecken som target")
        if not math.isclose(max(abs(value) for value in ratios), 1.0, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("turn-plan ratios måste ha maxmagnitud 1")

    def tick(self, observation: TurnObservation) -> TurnDecision:
        if self._terminal is not None: return self._terminal
        values = (observation.now_s, observation.left_distance_m, observation.right_distance_m)
        if not all(math.isfinite(value) for value in values) or observation.now_s < self.start_s:
            return self._fault("TURN_INVALID_OBSERVATION")
        if observation.now_s >= self.start_s + self.timeout_s: return self._fault("TURN_TIMEOUT")
        if (not observation.heading_fresh or observation.heading_deg is None or observation.heading_timestamp_s is None
                or not isinstance(observation.heading_sequence, int) or isinstance(observation.heading_sequence, bool)
                or observation.heading_sequence < 0 or not math.isfinite(observation.heading_deg)
                or not math.isfinite(observation.heading_timestamp_s)
                or observation.heading_timestamp_s > observation.now_s
                or observation.now_s - observation.heading_timestamp_s > self.heading_max_age_s):
            return self._fault("TURN_HEADING_STALE")
        for actual, target in ((observation.left_distance_m, self.plan.left_distance_m),
                               (observation.right_distance_m, self.plan.right_distance_m)):
            if (target > 0 and actual < -self.distance_tolerance_m) or (target < 0 and actual > self.distance_tolerance_m):
                return self._fault("TURN_REVERSE")
            if abs(actual) > abs(target) + self.distance_tolerance_m: return self._fault("TURN_OVERSHOOT")
        distance_done = (abs(observation.left_distance_m - self.plan.left_distance_m) <= self.distance_tolerance_m
                         and abs(observation.right_distance_m - self.plan.right_distance_m) <= self.distance_tolerance_m)
        heading_done = abs(_wrap(observation.heading_deg - (self.initial_heading_deg + 180.0))) <= self.heading_tolerance_deg
        if distance_done and heading_done:
            # A confirmation needs a new sensor capture. Repeated or older
            # samples cannot make a turn complete.
            if (self._last_heading_sequence is None
                    or (observation.heading_sequence > self._last_heading_sequence
                        and observation.heading_timestamp_s > self._last_heading_timestamp_s)):
                self._last_heading_sequence = observation.heading_sequence
                self._last_heading_timestamp_s = observation.heading_timestamp_s
                self._confirmations += 1
                if self._confirmations >= self.heading_confirm_frames:
                    self._terminal = TurnDecision(None, True, True)
                    return self._terminal
        else:
            self._confirmations = 0
        return TurnDecision(WheelCommand(self.plan.left_ratio * self.speed, self.plan.right_ratio * self.speed, "turn"), False, False)

    def _fault(self, reason: str) -> TurnDecision:
        self._terminal = TurnDecision(None, True, False, reason)
        return self._terminal
