"""
Global ``--debug`` flag.

When active, debugging aids that are otherwise no-ops become live. In
particular ``breakpoint()`` steps pause the mission and wait for a
hardware button press instead of returning immediately.

Activated by the ``LIBSTP_DEBUG=1`` environment variable, which is set
automatically by ``raccoon run --debug``.
"""

from __future__ import annotations

import os


def is_debug() -> bool:
    """Return ``True`` when debug mode is active."""
    return os.environ.get("LIBSTP_DEBUG") == "1"
