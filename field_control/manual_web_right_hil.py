"""Dedicated fixed raised-wheel HIL entry point for logical vehicle right."""
from __future__ import annotations

from .manual_web_hil import main_right


def main(argv: list[str] | None = None) -> int:
    return main_right(argv)


if __name__ == "__main__":
    raise SystemExit(main())
