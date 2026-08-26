"""Safe configuration and diagnostics entry point; never enables physical CAN."""
from __future__ import annotations

import argparse
from pathlib import Path
import signal
import threading
from typing import Sequence

from .app import FieldControlApplication
from .config import RuntimeConfig
from .config_io import dump_runtime_config, load_runtime_config


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="field-control", description="field_control diagnostics/configuration")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--validate-config", metavar="PATH")
    action.add_argument("--write-default-config", metavar="PATH")
    parser.add_argument("--force", action="store_true", help="allow overwriting --write-default-config target")
    parser.add_argument("--config", metavar="PATH", help="strict JSON config for diagnostics runtime")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.force and not args.write_default_config:
        _parser().error("--force kräver --write-default-config")
    try:
        if args.write_default_config:
            target = Path(args.write_default_config)
            if target.is_symlink():
                print(f"vägrar skriva till symlänk: {target}")
                return 2
            if target.exists() and not target.is_file():
                print(f"vägrar skriva till icke-vanlig fil: {target}")
                return 2
            if target.exists() and not args.force:
                print(f"vägrar skriva över befintlig fil: {target}")
                return 2
            dump_runtime_config(RuntimeConfig(), target)
            return 0
        if args.validate_config:
            load_runtime_config(args.validate_config)
            return 0
        if not args.config:
            _parser().error("normal körning kräver --config PATH")
        if not 1 <= args.port <= 65535:
            raise ValueError("--port måste ligga mellan 1 och 65535")
        config = load_runtime_config(args.config)
        # The ordinary CLI is diagnostics-only. Physical output remains
        # exclusively owned by the bounded raised-wheel HIL runners.
        if config.physical_can.enabled:
            raise ValueError("field-control CLI avvisar physical_can.enabled; använd endast avgränsad HIL-körare")
    except (OSError, ValueError) as exc:
        print(f"konfigurationsfel: {exc}")
        return 2

    application = FieldControlApplication(config, web_host=args.host, web_port=args.port)
    stopping = threading.Event()
    previous_handlers: dict[int, object] = {}
    def request_stop(_signum, _frame) -> None:
        stopping.set()
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_stop)
        application.start()
        while not stopping.wait(.2):
            pass
    except KeyboardInterrupt:
        stopping.set()
    finally:
        try:
            application.close()
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
