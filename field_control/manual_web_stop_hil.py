"""Dedicated fixed raised-wheel HIL entry point for web STOP while active."""
from __future__ import annotations

from .manual_web_hil import main_stop


def main(argv: list[str] | None = None) -> int:
    return main_stop(argv)


if __name__ == "__main__":
    raise SystemExit(main())
