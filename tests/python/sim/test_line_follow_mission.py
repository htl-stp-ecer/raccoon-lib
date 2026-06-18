from __future__ import annotations

import json
import os
import subprocess
import sys
from importlib import import_module
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_line_follow_runner.py"


def _raccoon_available() -> bool:
    try:
        sim = import_module("raccoon.sim")
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


def test_line_follow_builder_step_runs_against_sim() -> None:
    results = _run_runner()
    x, y, theta = results["forward_lateral_hold"]
    assert x > 70.0
    assert y == pytest.approx(50.0, abs=5.0)
    assert theta == pytest.approx(0.0, abs=0.4)
