"""End-to-end smoke: localization world pose survives motion boundaries.

Runs two back-to-back ``drive_forward(20)`` steps against the mock sim with
``enable_localization=True``. The Phase-2 promise is that
``robot.localization.get_pose()`` accumulates across both motions despite
the ``odometry.reset()`` inside each motion's ``start()``.

Tolerances are intentionally generous (5 cm / 5°): the value of this test
is the *trend* — pose continues to grow — not exact PID convergence.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
RUNNER = Path(__file__).parent / "_localization_cross_motion_runner.py"


def _raccoon_available() -> bool:
    try:
        from raccoon import sim

        return hasattr(sim, "mock")
    except ImportError:
        return False


pytestmark = pytest.mark.skipif(
    not _raccoon_available(),
    reason="raccoon mock-bundle wheel not installed (rebuild with "
    "`pip install -e . --config-settings=cmake.define.DRIVER_BUNDLE=mock`)",
)


def _run_runner() -> dict:
    env = os.environ.copy()
    env.setdefault("LIBSTP_LOG_LEVEL", "warn")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")

    proc = subprocess.run(
        [sys.executable, str(RUNNER)],
        capture_output=True,
        text=True,
        timeout=60,
        env=env,
        check=False,
    )
    if proc.returncode != 0:
        msg = (
            f"runner failed (exit={proc.returncode})\n"
            f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
        )
        raise AssertionError(msg)

    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:") :])
    msg = f"runner did not emit RESULTS line\nstdout: {proc.stdout}\nstderr: {proc.stderr}"
    raise AssertionError(msg)


def test_world_pose_accumulates_across_two_drives() -> None:
    """Two 20 cm drives → world pose ~ (0.40, 0, 0)."""
    out = _run_runner()

    p1 = out["after_motion_1"]
    p2 = out["after_motion_2"]

    # First motion: ~0.20 m forward in the localization frame.
    assert p1[0] == pytest.approx(0.20, abs=0.05), f"after motion 1: {p1}"
    assert p1[1] == pytest.approx(0.00, abs=0.05), f"after motion 1: {p1}"

    # Second motion: continues from where the first ended — *not* reset back
    # to ~0.20 m. This is the Phase-2 headline promise.
    assert p2[0] == pytest.approx(0.40, abs=0.05), (
        f"world pose did not survive motion boundary: motion 1 ended at {p1}, "
        f"motion 2 ended at {p2} (expected ~0.40 m)"
    )
    assert p2[1] == pytest.approx(0.00, abs=0.05), f"after motion 2: {p2}"
