"""Thin wrappers around the C++ logging backend with per-file filtering.

Every call automatically captures the caller's Python filename so that
``set_file_level("my_module.py", Level.debug)`` works the same way it does
for C++ source files.
"""

from __future__ import annotations

import os
import sys

from libstp.foundation import Level, _log_filtered


def _caller_filename(stacklevel: int) -> str:
    frame = sys._getframe(stacklevel + 1)  # +1 to skip this helper
    return os.path.basename(frame.f_code.co_filename)


def debug(message: str, *, _stacklevel: int = 1) -> None:
    _log_filtered(Level.debug, _caller_filename(_stacklevel), message)


def info(message: str, *, _stacklevel: int = 1) -> None:
    _log_filtered(Level.info, _caller_filename(_stacklevel), message)


def warn(message: str, *, _stacklevel: int = 1) -> None:
    _log_filtered(Level.warn, _caller_filename(_stacklevel), message)


def error(message: str, *, _stacklevel: int = 1) -> None:
    _log_filtered(Level.error, _caller_filename(_stacklevel), message)
