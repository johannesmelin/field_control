"""Configuration, diagnostics, and explicitly gated local physical web entrypoint."""
from __future__ import annotations

import argparse
import ipaddress
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
    parser.add_argument("--physical-web", action="store_true",
                        help="allow the separate local-only physical web deployment")
    parser.add_argument("--confirm-physical-web", action="store_true",
                        help="acknowledge that --physical-web opens the configured physical CAN boundary")
    parser.add_argument("--arm-motor-output", action="store_true",
                        help="locally arm output after startup and fresh physical odometry; never a web API")
    return parser


def _is_loopback_host(host: str) -> bool:
    """Accept only an IP literal that the OS identifies as loopback.

    Do not resolve host names here: a mutable resolver or hosts file must not
    turn a local-only physical deployment into a network listener.
    """
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def _arm_physical_web_output(application: FieldControlApplication) -> None:
    """Perform the local-only arm step after application startup.

    ``arm_motor_output`` itself waits for a fresh shared 0x92 odometry pair.
    Keep the additional MANUAL-state gate here, immediately before that call,
    so no browser request can race startup into an armed AUTO state.
    """
    status = application.runtime.status()
    if status.state != "MANUAL":
        raise ValueError("fysisk webbarmning kräver MANUAL efter uppstart")
    if status.motor_output_armed:
        raise ValueError("motorutgången är redan armerad")
    application.runtime.arm_motor_output_for_web_standby()


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.force and not args.write_default_config:
        _parser().error("--force kräver --write-default-config")
    if args.confirm_physical_web and not args.physical_web:
        _parser().error("--confirm-physical-web kräver --physical-web")
    if args.arm_motor_output and not args.physical_web:
        _parser().error("--arm-motor-output kräver --physical-web")
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
        # The ordinary CLI is diagnostics-only.  Physical CAN requires both
        # this distinct opt-in and its own already-validated configuration
        # gates.  A physical listener is intentionally local-only until a
        # separately designed authentication boundary exists.
        if config.physical_can.enabled:
            if not args.physical_web:
                raise ValueError("field-control CLI avvisar physical_can.enabled utan --physical-web")
            if not args.confirm_physical_web:
                raise ValueError("--physical-web kräver --confirm-physical-web")
            if not _is_loopback_host(args.host):
                raise ValueError("fysisk webbdrift får endast bindas till loopback-IP")
        elif args.physical_web:
            raise ValueError("--physical-web kräver physical_can.enabled i konfigurationen")
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
        # No CLI path arms by default.  This explicit local option runs only
        # after all sources, runtime and independent watchdog are alive.
        if args.arm_motor_output:
            _arm_physical_web_output(application)
        while not stopping.wait(.2):
            pass
    except KeyboardInterrupt:
        stopping.set()
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"startfel: {exc}")
        return 2
    finally:
        try:
            application.close()
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
