"""End-to-end mission test: ``await drive_forward(N).run_step(robot)`` against the sim.

This is the smallest test that proves the entire stack works as a unit:

- A real ``raccoon.hal.Motor`` is constructed (which initializes MockPlatform).
- A real ``DifferentialKinematics`` + ``Drive`` + ``Stm32Odometry`` are wired.
- The simulator is attached via ``raccoon.testing.sim.use_scene(...)``.
- The unmodified motion steps run against this robot inside an asyncio loop.
- The fixed-rate motion loop ticks at 100 Hz; auto-tick advances the sim with
  wall-clock dt as the loop reads odometry.
- After the step finishes, the sim's ground-truth pose should match the
  commanded distance.

The actual scenario logic lives in ``_drive_mission_runner.py`` and is
executed in a separate subprocess so any C++ extension teardown races at
interpreter shutdown stay isolated from pytest. The runner ``os._exit``s
after writing its results, bypassing Python destruction entirely — this
works around a pybind11 lifetime issue with the mock HAL singleton that
otherwise crashes pytest's test loop.

Skipped if the wheel was built without DRIVER_BUNDLE=mock.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
RUNNER = Path(__file__).parent / "_drive_mission_runner.py"


def _raccoon_available() -> bool:
    try:
        import raccoon  # noqa: F401
        from raccoon import sim  # noqa: F401
        return hasattr(sim, "mock")
    except ImportError:
        return False


pytestmark = pytest.mark.skipif(
    not _raccoon_available(),
    reason="raccoon mock-bundle wheel not installed (rebuild with "
           "`pip install -e . --config-settings=cmake.define.DRIVER_BUNDLE=mock`)",
)


def _run_runner() -> dict:
    """Spawn the runner subprocess and parse its results JSON."""
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
        raise AssertionError(
            f"runner failed (exit={proc.returncode})\n"
            f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
        )

    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:"):])
    raise AssertionError(
        f"runner did not emit RESULTS line\n"
        f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
    )


def test_drive_steps_against_sim():
    """End-to-end: drive forward, drive shorter, drive into a wall.

    All scenarios run in one subprocess via the runner script. Each scenario
    uses a fresh ``use_scene`` context against the same Drive/Odometry stack.
    """
    results = _run_runner()

    # Scenario 1: drive(30) on an open table starting at (50, 50, 0).
    x, y, theta = results["drive_30"]
    assert 76.0 < x < 86.0, f"drive(30) ended at x={x:.2f}, expected ~80 ± 6"
    assert abs(y - 50.0) < 3.0
    assert abs(theta) < 0.1

    # Scenario 2: drive(15) starting at (20, 50, 0).
    x, y, _ = results["drive_15"]
    assert 31.0 < x < 41.0, f"drive(15) ended at x={x:.2f}, expected ~35 ± 6"
    assert abs(y - 50.0) < 3.0

    # Scenario 3: wall_box scene blocks the robot before it reaches its goal.
    x, _, _ = results["wall"]
    assert 80.0 < x < 95.0, (
        f"wall didn't stop robot in expected range — x={x:.2f}"
    )
