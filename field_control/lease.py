"""Monotonic control lease for the physical motor boundary."""
from __future__ import annotations

import secrets
import threading
import time
from typing import Callable


class ControlLease:
    """Explicit expiring bearer token with fail-closed revocation."""

    def __init__(self, timeout_s: float = .5, clock: Callable[[], float] = time.monotonic) -> None:
        if timeout_s <= 0:
            raise ValueError("lease-timeout måste vara positiv")
        self.timeout_s, self._clock = float(timeout_s), clock
        self._lock = threading.RLock()
        self._token: str | None = None
        self._expires_at: float | None = None
        self._on_revoke: Callable[[], None] | None = None

    def acquire(self) -> str:
        with self._lock:
            self._token = secrets.token_urlsafe(24)
            self._expires_at = self._clock() + self.timeout_s
            return self._token

    def refresh(self, token: str) -> None:
        with self._lock:
            if not self.valid(token):
                raise ValueError("control-lease saknas eller har löpt ut")
            self._expires_at = self._clock() + self.timeout_s

    def valid(self, token: str | None) -> bool:
        with self._lock:
            if token is None or token != self._token or self._expires_at is None:
                return False
            if self._clock() >= self._expires_at:
                self._revoke_locked()
                return False
            return True

    def run_if_valid(self, token: str | None, operation: Callable[[], object]) -> bool:
        with self._lock:
            if not self.valid(token):
                return False
            operation()
            return True

    def set_revoke_callback(self, callback: Callable[[], None]) -> None:
        with self._lock:
            self._on_revoke = callback

    def watchdog_tick(self) -> bool:
        with self._lock:
            if self._token is None or self._expires_at is None:
                return False
            if self._clock() < self._expires_at:
                return False
            self._revoke_locked()
            return True

    def revoke_any(self) -> bool:
        with self._lock:
            if self._token is None:
                return False
            self._revoke_locked()
            return True

    def _revoke_locked(self) -> None:
        self._token = self._expires_at = None
        callback = self._on_revoke
        if callback is not None:
            callback()