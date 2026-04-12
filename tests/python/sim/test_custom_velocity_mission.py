"""End-to-end mission tests for ``custom_velocity`` against the simulator.

Proves that:
- The velocity function is called each cycle and its output is scaled and
  applied to the drive controller.
- The until condition terminates the step at the right time/distance.
- Zero velocity leaves the robot stationary.
- Spin-only command produces heading change without significant translation.

Each scenario is tested with three robot configurations:
- ``default``: idealized robot with no motor drag
- ``drumbot``: differential-drive robot with real calibration + drag
- ``packingbot``: mecanum-drive robot with real calibration + drag

Scenario logic lives in ``_custom_velocity_runner.py`` and runs in a
subprocess for the same reason as the drive mission tests — pybind11 /
mock-HAL singleton teardown is fragile at interpreter shutdown.

Skipped if the wheel was built without DRIVER_BUNDLE=mock.
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_custom_velocity_runner.py"

ROBOT_CONFIGS = ["default", "drumbot", "packingbot"]


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


def _run_runner(config_name: str) -> dict:
    """Spawn the runner subprocess and parse its results JSON."""
    env = os.environ.copy()
    env.setdefault("LIBSTP_LOG_LEVEL", "warn")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")

    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--config", config_name],
        capture_output=True,
        text=True,
        timeout=60,
        env=env,
        check=False,
    )
    if proc.returncode != 0:
        raise AssertionError(
            f"runner failed (exit={proc.returncode}, config={config_name})\n"
            f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
        )

    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:"):])
    raise AssertionError(
        f"runner did not emit RESULTS line (config={config_name})\n"
        f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
    )


@pytest.fixture(scope="module", params=ROBOT_CONFIGS)
def results(request):
    """Run all scenarios for each robot config and cache the results."""
    return _run_runner(request.param)


def test_forward_1s_moves_significantly(results):
    """Forward at 100% for 1 s — robot must travel a meaningful distance."""
    x, y, theta = results["forward_1s"]
    assert x > 70.0, (
        f"forward_1s: robot barely moved — x={x:.2f}, expected > 70"
    )
    assert x < 150.0, (
        f"forward_1s: robot overshot — x={x:.2f}, expected < 150"
    )
    assert abs(y - 50.0) < 6.0, (
        f"forward_1s: unexpected lateral drift — y={y:.2f}"
    )
    assert abs(theta) < 0.25, (
        f"forward_1s: unexpected heading change — theta={math.degrees(theta):.1f}°"
    )


def test_forward_until_cm_stops_near_target(results):
    """Forward until after_forward_cm(20) — should stop near 20 cm displacement."""
    x, y, _ = results["forward_until_cm"]
    assert 58.0 < x < 80.0, (
        f"forward_until_cm: stopped at unexpected x={x:.2f}, expected ~70 ± 12"
    )
    assert abs(y - 50.0) < 6.0, (
        f"forward_until_cm: unexpected lateral drift — y={y:.2f}"
    )


def test_spin_rotates_without_translating(results):
    """Pure spin at 100% for 1 s — heading changes, position stays."""
    x, y, theta = results["spin_1s"]
    assert abs(x - 50.0) < 10.0, (
        f"spin_1s: unexpected x translation — x={x:.2f}"
    )
    assert abs(y - 50.0) < 10.0, (
        f"spin_1s: unexpected y translation — y={y:.2f}"
    )
    total_rotation = abs(theta)
    assert total_rotation > 0.2, (
        f"spin_1s: rotation too small — theta={math.degrees(theta):.1f}°, "
        f"expected > 11°"
    )


def test_zero_velocity_no_drift(results):
    """Zero velocity for 0.5 s — robot must not move."""
    x, y, theta = results["zero_velocity"]
    assert abs(x - 50.0) < 2.0, (
        f"zero_velocity: unexpected x drift — x={x:.2f}"
    )
    assert abs(y - 50.0) < 2.0, (
        f"zero_velocity: unexpected y drift — y={y:.2f}"
    )
    assert abs(theta) < 0.1, (
        f"zero_velocity: unexpected heading change — theta={math.degrees(theta):.1f}°"
    )
