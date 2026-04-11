"""
Global ``--no-checkpoints`` flag.

When active, ``wait_for_checkpoint`` steps return immediately instead of
waiting for the mission-relative clock to reach the checkpoint time.

Activated by the ``LIBSTP_NO_CHECKPOINTS=1`` environment variable,
which is set automatically by ``raccoon run --no-checkpoints``.
"""
from __future__ import annotations

import os


def is_no_checkpoints() -> bool:
    """Return ``True`` when checkpoint waits should be skipped."""
    return os.environ.get("LIBSTP_NO_CHECKPOINTS") == "1"
