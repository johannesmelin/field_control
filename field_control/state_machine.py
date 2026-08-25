"""Pure, fail-closed field-navigation state machine.

This module deliberately has no camera, IMU, CAN, web, or motor dependency.
It decides only which controller may be active.  A future hardware integration
must route every stop and output command through the verified control lease and
physical MotorTransport boundary from the existing projects.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class State(str, Enum):
    MANUAL = "MANUAL"
    AUTO_START_DELAY = "AUTO_START_DELAY"
    AUTO_ROW_FOLLOW = "AUTO_ROW_FOLLOW"
    AUTO_PICK = "AUTO_PICK"
    AUTO_POST_PICK = "AUTO_POST_PICK"
    AUTO_SEARCH = "AUTO_SEARCH"
    AUTO_IN_ROW_TURN = "AUTO_IN_ROW_TURN"
    AUTO_NEW_ROW_TURN = "AUTO_NEW_ROW_TURN"
    AUTO_COMPLETE = "AUTO_COMPLETE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class SafetyConfig:
    auto_start_delay_s: float = 3.0
    navigation_lost_timeout_s: float = 1.0
    navigation_reacquire_frames: int = 3
    search_length_m: float = 1.0
    max_pick_wait_s: float = 30.0
    pick_clear_time_s: float = 1.0
    post_pick_lockout_distance_m: float = 0.3
    turn_marker_confirm_frames: int = 3
    turn_marker_rearm_distance_m: float = 0.5
    in_row_turn_enabled: bool = False
    number_of_rows: int = 1

    def validate(self) -> "SafetyConfig":
        nonnegative = (self.auto_start_delay_s, self.navigation_lost_timeout_s,
                       self.search_length_m, self.max_pick_wait_s,
                       self.pick_clear_time_s, self.post_pick_lockout_distance_m,
                       self.turn_marker_rearm_distance_m)
        if not all(math.isfinite(value) and value >= 0 for value in nonnegative):
            raise ValueError("tids- och sträckgränser måste vara ändliga och icke-negativa")
        if self.navigation_reacquire_frames < 1 or self.turn_marker_confirm_frames < 1:
            raise ValueError("frame-bekräftelse måste vara minst 1")
        if self.number_of_rows < 1:
            raise ValueError("number_of_rows måste vara minst 1")
        return self


@dataclass(frozen=True)
class Observation:
    """Latest non-blocking sensor/control observation.

    ``frame_fresh`` means the camera delivered a new valid frame inside its
    timeout.  ``visual_target`` is distinct: it may be false while the camera
    is healthy, which is the sole condition that permits AUTO_SEARCH.
    """
    now_s: float
    frame_fresh: bool
    imu_fresh: bool
    odometry_fresh: bool
    can_healthy: bool
    visual_target: bool
    bud_in_trigger_zone: bool = False
    bud_in_pick_zone: bool = False
    marker_seen: bool = False
    distance_m: float = 0.0
    row_heading_reliable: bool = False


@dataclass(frozen=True)
class Snapshot:
    state: State
    reason: str
    fault: str | None
    row_number: int
    pass_number: int
    auto_start_remaining_s: float | None
    search_distance_m: float
    post_pick_distance_m: float
    marker_armed: bool


class FieldStateMachine:
    """Explicit state transitions, designed for a deterministic control loop."""

    _ACTIVE = frozenset({
        State.AUTO_START_DELAY, State.AUTO_ROW_FOLLOW, State.AUTO_PICK,
        State.AUTO_POST_PICK, State.AUTO_SEARCH, State.AUTO_IN_ROW_TURN,
        State.AUTO_NEW_ROW_TURN,
    })

    def __init__(self, config: SafetyConfig | None = None) -> None:
        self.config = (config or SafetyConfig()).validate()
        self.state = State.MANUAL
        self.reason = "Manuell styrning vald; automatisk motorutgång är avstängd"
        self.fault: str | None = None
        self.row_number = 1
        self.pass_number = 1
        self._auto_start_at: float | None = None
        self._last_visual_at: float | None = None
        self._search_started_distance_m: float | None = None
        self._post_pick_start_distance_m: float | None = None
        self._pick_clear_started_at: float | None = None
        self._pick_started_at: float | None = None
        self._marker_frames = 0
        self._marker_rearm_distance_start_m: float | None = None
        self._marker_armed = True
        self._reacquire_frames = 0
        self._last_distance_m = 0.0

    def select_manual(self) -> None:
        """Switch mode. Caller must issue an immediate verified motor stop."""
        self._transition(State.MANUAL, "Manuell styrning vald")

    def select_auto(self) -> None:
        """Switch mode but never begin motion automatically."""
        self._transition(State.MANUAL, "Automatiskt läge valt; välj Start Auto")

    def request_start_auto(self, observation: Observation) -> None:
        self._require_sensors(observation)
        if self.state is not State.MANUAL:
            raise ValueError("Start Auto är bara tillåten från MANUAL")
        if not observation.visual_target:
            raise ValueError("Start Auto kräver minst ett giltigt visuellt navigationsmål")
        self._auto_start_at = observation.now_s + self.config.auto_start_delay_s
        self._transition(State.AUTO_START_DELAY, "Automatisk startfördröjning aktiv")

    def stop(self, reason: str = "STOP begärd av operatör") -> None:
        """Caller must issue an immediate verified motor stop before/with this."""
        self._transition(State.MANUAL, reason)

    def complete_turn(self, observation: Observation, succeeded: bool) -> None:
        if self.state not in (State.AUTO_IN_ROW_TURN, State.AUTO_NEW_ROW_TURN):
            raise ValueError("ingen vändning är aktiv")
        if not succeeded:
            self._fault("TURN_FAILURE")
            return
        self._marker_armed = False
        self._marker_rearm_distance_start_m = observation.distance_m
        if self.state is State.AUTO_IN_ROW_TURN:
            self.pass_number = 2 if self.pass_number == 1 else 1
        else:
            self.row_number += 1
            self.pass_number = 1
            if self.row_number > self.config.number_of_rows:
                self._transition(State.AUTO_COMPLETE, "Samtliga fysiska rader är färdiga")
                return
        self._search_started_distance_m = observation.distance_m
        # A completed 180° turn supplies a derived reliable row-heading. If
        # targets are absent the control layer may enter SEARCH immediately.
        self._transition(
            State.AUTO_ROW_FOLLOW if observation.visual_target else State.AUTO_SEARCH,
            "Vändning klar",
        )

    def tick(self, observation: Observation) -> Snapshot:
        if not math.isfinite(observation.now_s) or not math.isfinite(observation.distance_m):
            self._fault("INVALID_SENSOR_VALUE")
            return self.snapshot(observation.now_s)
        self._last_distance_m = observation.distance_m
        if self.state in self._ACTIVE:
            try:
                self._require_sensors(observation)
            except ValueError as exc:
                self._fault(str(exc))
                return self.snapshot(observation.now_s)
        self._update_marker_rearm(observation)
        if self.state is State.AUTO_START_DELAY:
            if observation.now_s >= (self._auto_start_at or observation.now_s):
                if observation.visual_target:
                    self._last_visual_at = observation.now_s
                    self._transition(State.AUTO_ROW_FOLLOW, "Automatisk radföljning startad")
                else:
                    self._fault("START_TARGET_LOST")
            return self.snapshot(observation.now_s)
        if self.state not in self._ACTIVE:
            return self.snapshot(observation.now_s)
        if self.state in (State.AUTO_IN_ROW_TURN, State.AUTO_NEW_ROW_TURN):
            return self.snapshot(observation.now_s)
        if self._marker_armed:
            self._marker_frames = self._marker_frames + 1 if observation.marker_seen else 0
            if self._marker_frames >= self.config.turn_marker_confirm_frames:
                self._start_turn()
                return self.snapshot(observation.now_s)
        if self.state is State.AUTO_PICK:
            self._tick_pick(observation)
            return self.snapshot(observation.now_s)
        if self.state is State.AUTO_POST_PICK:
            self._tick_post_pick(observation)
            return self.snapshot(observation.now_s)
        if observation.bud_in_trigger_zone:
            self._pick_started_at = observation.now_s
            self._pick_clear_started_at = None
            self._transition(State.AUTO_PICK, "Knopp i trigger_zone")
            return self.snapshot(observation.now_s)
        self._tick_navigation(observation)
        return self.snapshot(observation.now_s)

    def _tick_navigation(self, observation: Observation) -> None:
        if observation.visual_target:
            self._last_visual_at = observation.now_s
            if self.state is State.AUTO_SEARCH:
                self._reacquire_frames += 1
                if self._reacquire_frames >= self.config.navigation_reacquire_frames:
                    self._transition(State.AUTO_ROW_FOLLOW, "Visuellt navigationsmål återfångat")
            else:
                self._reacquire_frames = 0
            if self.state is not State.AUTO_ROW_FOLLOW and self.state is not State.AUTO_SEARCH:
                self._transition(State.AUTO_ROW_FOLLOW, "Visuell radföljning")
            return
        self._reacquire_frames = 0
        if self._last_visual_at is None:
            self._last_visual_at = observation.now_s
        if self.state is State.AUTO_ROW_FOLLOW and (
            observation.now_s - self._last_visual_at >= self.config.navigation_lost_timeout_s
        ):
            if not observation.row_heading_reliable:
                self._fault("ROW_HEADING_UNAVAILABLE")
                return
            self._search_started_distance_m = observation.distance_m
            self._transition(State.AUTO_SEARCH, "Visuella navigationsmål saknas; headingbaserad sökning")
        if self.state is State.AUTO_SEARCH:
            distance = self._distance_since(self._search_started_distance_m, observation.distance_m)
            if distance >= self.config.search_length_m:
                self._fault("ROW_LOST")

    def _tick_pick(self, observation: Observation) -> None:
        if not observation.bud_in_pick_zone:
            self._pick_clear_started_at = self._pick_clear_started_at or observation.now_s
        else:
            self._pick_clear_started_at = None
        clear_for = 0.0 if self._pick_clear_started_at is None else observation.now_s - self._pick_clear_started_at
        waited = observation.now_s - (self._pick_started_at or observation.now_s)
        if clear_for >= self.config.pick_clear_time_s or waited >= self.config.max_pick_wait_s:
            self._post_pick_start_distance_m = observation.distance_m
            self._transition(State.AUTO_POST_PICK, "PICK_TIMEOUT" if waited >= self.config.max_pick_wait_s else "Pick-zon fri")

    def _tick_post_pick(self, observation: Observation) -> None:
        if self._distance_since(self._post_pick_start_distance_m, observation.distance_m) < self.config.post_pick_lockout_distance_m:
            return
        self._transition(
            State.AUTO_ROW_FOLLOW if observation.visual_target else State.AUTO_SEARCH,
            "Pick-lockout passerad",
        )
        if self.state is State.AUTO_SEARCH:
            if not observation.row_heading_reliable:
                self._fault("ROW_HEADING_UNAVAILABLE")
            else:
                self._search_started_distance_m = observation.distance_m

    def _start_turn(self) -> None:
        if self.config.in_row_turn_enabled and self.pass_number == 1:
            self._transition(State.AUTO_IN_ROW_TURN, "Vändmarkör: in-row-turn")
        else:
            self._transition(State.AUTO_NEW_ROW_TURN, "Vändmarkör: new-row-turn")

    def _update_marker_rearm(self, observation: Observation) -> None:
        if self._marker_armed or self._marker_rearm_distance_start_m is None:
            return
        if self._distance_since(self._marker_rearm_distance_start_m, observation.distance_m) >= self.config.turn_marker_rearm_distance_m:
            self._marker_armed = True
            self._marker_frames = 0

    def _require_sensors(self, observation: Observation) -> None:
        missing = []
        if not observation.frame_fresh: missing.append("CAMERA_TIMEOUT")
        if not observation.imu_fresh: missing.append("IMU_TIMEOUT")
        if not observation.odometry_fresh: missing.append("ODOMETRY_TIMEOUT")
        if not observation.can_healthy: missing.append("CAN_FAILURE")
        if missing:
            raise ValueError("/".join(missing))

    @staticmethod
    def _distance_since(start: float | None, current: float) -> float:
        return 0.0 if start is None else max(0.0, current - start)

    def _fault(self, reason: str) -> None:
        self.fault = reason
        self._transition(State.FAULT, reason)

    def _transition(self, state: State, reason: str) -> None:
        self.state, self.reason = state, reason

    def snapshot(self, now_s: float) -> Snapshot:
        remaining = None
        if self.state is State.AUTO_START_DELAY and self._auto_start_at is not None:
            remaining = max(0.0, self._auto_start_at - now_s)
        return Snapshot(
            state=self.state, reason=self.reason, fault=self.fault,
            row_number=self.row_number, pass_number=self.pass_number,
            auto_start_remaining_s=remaining,
            search_distance_m=self._distance_since(self._search_started_distance_m, self._last_distance_m),
            post_pick_distance_m=self._distance_since(self._post_pick_start_distance_m, self._last_distance_m),
            marker_armed=self._marker_armed,
        )
