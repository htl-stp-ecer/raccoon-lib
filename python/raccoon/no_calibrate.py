"""
Global ``--no-calibrate`` flag.

When active, interactive calibration steps are skipped and stored
values from ``raccoon.project.yml`` / ``racoon.calibration.yml`` are
used instead.

Activated by the ``LIBSTP_NO_CALIBRATE=1`` environment variable,
which is set automatically by ``raccoon run --no-calibrate``.
"""

from __future__ import annotations

import os


def is_no_calibrate() -> bool:
    """Return ``True`` when calibration should be skipped."""
    return os.environ.get("LIBSTP_NO_CALIBRATE") == "1"
