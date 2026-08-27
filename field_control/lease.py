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
        callback = None
        with self._lock:
            if token is None or token != self._token or self._expires_at is None:
                raise ValueError("control-lease saknas eller har löpt ut")
            if self._clock() >= self._expires_at:
                callback = self._revoke_locked()
            else:
                self._expires_at = self._clock() + self.timeout_s
                return
        self._invoke_revoke(callback)
        raise ValueError("control-lease saknas eller har löpt ut")

    def valid(self, token: str | None) -> bool:
        callback = None
        with self._lock:
            if token is None or token != self._token or self._expires_at is None:
                return False
            if self._clock() >= self._expires_at:
                callback = self._revoke_locked()
            else:
                return True
        self._invoke_revoke(callback)
        return False

    def run_if_valid(self, token: str | None, operation: Callable[[], object]) -> bool:
        callback = None
        with self._lock:
            if token is None or token != self._token or self._expires_at is None:
                return False
            if self._clock() >= self._expires_at:
                callback = self._revoke_locked()
            else:
                # The lease lock serialises admission and the complete output
                # transaction. A revocation that races this call is ordered
                # either before admission (no operation) or after it returns.
                operation()
                return True
        self._invoke_revoke(callback)
        return False

    def set_revoke_callback(self, callback: Callable[[], None]) -> None:
        with self._lock:
            self._on_revoke = callback

    def watchdog_tick(self) -> bool:
        callback = None
        with self._lock:
            if self._token is None or self._expires_at is None:
                return False
            if self._clock() < self._expires_at:
                return False
            callback = self._revoke_locked()
        self._invoke_revoke(callback)
        return True

    def revoke_any(self) -> bool:
        callback = None
        with self._lock:
            if self._token is None:
                return False
            callback = self._revoke_locked()
        self._invoke_revoke(callback)
        return True

    def release(self, token: str | None) -> bool:
        """Relinquish one valid lease without invoking its output callback.

        This is deliberately narrower than ``revoke_any``.  It exists solely
        for the physical-web no-motion handoff: the verified boundary has
        atomically changed to its separately bounded standby state before the
        ordinary drive lease is released.  It must never be used to extend or
        renew a drive lease.
        """
        with self._lock:
            if token is None or token != self._token or self._expires_at is None:
                return False
            self._token = self._expires_at = None
            return True

    def _revoke_locked(self) -> Callable[[], None] | None:
        self._token = self._expires_at = None
        return self._on_revoke

    @staticmethod
    def _invoke_revoke(callback: Callable[[], None] | None) -> None:
        if callback is not None:
            callback()
