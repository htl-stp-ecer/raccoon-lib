from __future__ import annotations

import atexit
import signal
from types import FrameType
from typing import Dict

from libstp import hal as _hal
from libstp.class_name_logger import ClassNameLogger
from libstp.foundation import error, info, debug, initialize_logging

Motor = _hal.Motor

_PREVIOUS_SIGNAL_HANDLERS: Dict[int, signal.Handlers] = {}
_HOOKS_INSTALLED = False


def _disable_all_motors() -> None:
    disable = getattr(Motor, "disable_all", None)
    if disable is None:
        return

    try:
        info("Disabling all motors")
        disable()
    except Exception:
        error("Failed to disable all motors")
        # Never let shutdown hooks raise to callers
        pass


def _forward_signal(signum: int, frame: FrameType | None) -> None:
    handler = _PREVIOUS_SIGNAL_HANDLERS.get(signum)
    if handler in (None, signal.SIG_IGN):
        return

    if handler is signal.SIG_DFL:
        if signum == getattr(signal, "SIGINT", None):
            raise KeyboardInterrupt
        raise SystemExit(128 + signum)

    handler(signum, frame)


def _install_shutdown_hooks() -> None:
    global _HOOKS_INSTALLED
    if _HOOKS_INSTALLED:
        return

    _HOOKS_INSTALLED = True

    atexit.register(_disable_all_motors)
    print("Registered atexit shutdown hook to disable all motors")

    for sig_name in ("SIGINT", "SIGTERM"):
        sig = getattr(signal, sig_name, None)
        if sig is None:
            continue

        try:
            previous = signal.getsignal(sig)
        except Exception:
            debug(f"Failed to get previous handler for signal {sig_name}")
            continue

        _PREVIOUS_SIGNAL_HANDLERS[sig] = previous

        def handler(signum: int, frame: FrameType | None) -> None:
            _disable_all_motors()
            _forward_signal(signum, frame)

        signal.signal(sig, handler)
        print(f"Registered signal handler for {sig_name}")


_install_shutdown_hooks()
initialize_logging()
