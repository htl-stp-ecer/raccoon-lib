"""Thin wrappers around the C++ logging backend.

Every call captures the caller's Python source location (file, line, function)
so those land as discrete fields in the JSONL log file. There is no runtime
level filtering: every compiled-in call is captured. The console shows only
warn/error; the JSONL file captures everything (debug and up).

When logged through a :class:`~raccoon.class_name_logger.ClassNameLogger`, the
``_cls`` hook qualifies the ``func`` field as ``"ClassName.method"``.
"""

from __future__ import annotations

import sys

from raccoon.foundation import Level, _log


def _emit(level: Level, message: str, stacklevel: int, cls: str | None) -> None:
    frame = sys._getframe(stacklevel + 1)  # +1 to skip this helper
    code = frame.f_code
    func = code.co_name
    if cls is not None:
        func = f"{cls}.{func}"
    _log(level, code.co_filename, frame.f_lineno, func, message)


def debug(message: str, *, _stacklevel: int = 1, _cls: str | None = None) -> None:
    _emit(Level.debug, message, _stacklevel, _cls)


def info(message: str, *, _stacklevel: int = 1, _cls: str | None = None) -> None:
    _emit(Level.info, message, _stacklevel, _cls)


def warn(message: str, *, _stacklevel: int = 1, _cls: str | None = None) -> None:
    _emit(Level.warn, message, _stacklevel, _cls)


def error(message: str, *, _stacklevel: int = 1, _cls: str | None = None) -> None:
    _emit(Level.error, message, _stacklevel, _cls)


def trace(message: str, *, _stacklevel: int = 1, _cls: str | None = None) -> None:
    # Real TRACE level. Unlike the C++ LIBSTP_LOG_TRACE macro (compiled out unless
    # the build sets -DLIBSTP_TRACE_LOGGING=ON), Python has no compile-time gate, so
    # these always emit to the JSONL file. Reserve trace() for per-tick detail.
    _emit(Level.trace, message, _stacklevel, _cls)
