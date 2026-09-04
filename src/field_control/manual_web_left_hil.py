"""Fixed raised-wheel HIL entrypoint for manual web left."""
from __future__ import annotations

from .manual_web_hil import main_left


def main(argv: list[str] | None = None) -> int:
    """Run the fixed logical-left HIL routine without exposing control options."""
    return main_left(argv)


if __name__ == "__main__":
    raise SystemExit(main())
