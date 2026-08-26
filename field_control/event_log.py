"""Bounded, non-blocking control-event journal with monotonic timestamps."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Mapping


_LEVELS = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40, "CRITICAL": 50}


@dataclass(frozen=True)
class Event:
    timestamp_s: float
    level: str
    kind: str
    data: Mapping[str, str | float | int | bool | None]

    def as_dict(self) -> dict[str, object]:
        return {"timestamp_s": self.timestamp_s, "level": self.level, "kind": self.kind, "data": dict(self.data)}


class EventLog:
    """Drop-oldest journal; ``record`` never waits for a consumer lock."""

    def __init__(self, *, capacity: int = 200, level: str = "INFO", clock=time.monotonic) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError("eventlog-kapacitet måste vara positivt heltal")
        if level not in _LEVELS:
            raise ValueError("okänd loggnivå")
        self._events: deque[Event] = deque(maxlen=capacity)
        self._level, self._clock, self._last_timestamp = level, clock, 0.0
        self._lock = threading.Lock()

    def record(self, kind: str, *, level: str = "INFO", timestamp_s: float | None = None,
               data: Mapping[str, str | float | int | bool | None] | None = None) -> bool:
        """Attempt to append one event; return false if filtered or contention drops it."""
        if level not in _LEVELS or not isinstance(kind, str) or not kind:
            raise ValueError("ogiltig eventlog-post")
        if _LEVELS[level] < _LEVELS[self._level]:
            return False
        timestamp = self._clock() if timestamp_s is None else timestamp_s
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            raise ValueError("eventtimestamp måste vara ändlig")
        event_data: dict[str, str | float | int | bool | None] = {}
        if data is not None:
            if not isinstance(data, Mapping):
                raise ValueError("eventdata måste vara en mapping")
            for key, value in data.items():
                if not isinstance(key, str):
                    raise ValueError("eventdata-nycklar måste vara strängar")
                if isinstance(value, float) and not math.isfinite(value):
                    raise ValueError("eventdata-float måste vara ändlig")
                if not (value is None or isinstance(value, (str, int, float, bool))):
                    raise ValueError("eventdata måste innehålla JSON-säkra skalärer")
                event_data[key] = value
        # The control loop must never wait behind diagnostics serialization.
        if not self._lock.acquire(blocking=False):
            return False
        try:
            monotonic_timestamp = max(self._last_timestamp, float(timestamp))
            self._last_timestamp = monotonic_timestamp
            self._events.append(Event(monotonic_timestamp, level, kind, event_data))
            return True
        finally:
            self._lock.release()

    def recent(self) -> list[dict[str, object]]:
        """Diagnostics-only snapshot; consumers may wait, control records never do."""
        with self._lock:
            return [event.as_dict() for event in self._events]
