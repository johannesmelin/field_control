"""Fixed raised-wheel HIL entrypoint for manual web reverse."""
from __future__ import annotations

from .manual_web_hil import main_reverse


def main(argv: list[str] | None = None) -> int:
    """Run the fixed reverse HIL routine without exposing control options."""
    return main_reverse(argv)


if __name__ == "__main__":
    raise SystemExit(main())
