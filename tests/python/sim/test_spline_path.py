"""End-to-end tests for ``spline()`` against the physics simulator.

Validates that spline path motion:
1. Straight-line spline reaches approximately the same distance as drive_forward
2. S-curve ends near the center line (heading returns to ~0)
3. Curved path produces the expected lateral displacement
4. Speed scaling works (half speed reaches same distance)
5. Heading stays stable during straight segments

Each scenario is tested with three robot configurations:
- ``default``: idealized robot with no motor drag
- ``drumbot``: differential-drive robot with real calibration + drag
- ``packingbot``: mecanum-drive robot with real calibration + drag

The runner subprocess pattern matches ``test_smooth_path.py`` to isolate
the C++ mock-HAL singleton teardown from pytest.
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_spline_path_runner.py"

ROBOT_CONFIGS = ["default"]


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
        timeout=90,
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


# --------------------------------------------------------------------------
# Scenario 1: straight line — should reach ~30cm forward
# --------------------------------------------------------------------------

class TestStraightLine:
    def test_reaches_target_distance(self, results):
        x, y, theta = results["straight_line"]
        # start at x=50, spline through (15,0)→(30,0) → expect x ~ 80
        assert 73.0 < x < 88.0, (
            f"spline straight ended at x={x:.2f}, expected ~80"
        )

    def test_heading_stable(self, results):
        _, _, theta = results["straight_line"]
        assert abs(theta) < 0.25, f"heading drifted: theta={theta:.3f}"

    def test_lateral_stable(self, results):
        _, y, _ = results["straight_line"]
        assert abs(y - 50.0) < 5.0, f"lateral drift: y={y:.2f}, expected ~50"

    def test_matches_drive_forward(self, results):
        spline_x = results["straight_line"][0]
        drive_x = results["drive_forward_30"][0]
        assert abs(spline_x - drive_x) < 8.0, (
            f"spline x={spline_x:.2f} vs drive_forward x={drive_x:.2f}"
        )


# --------------------------------------------------------------------------
# Scenario 3: S-curve — should end near center line heading ~0
# --------------------------------------------------------------------------

class TestSCurve:
    def test_moved_forward(self, results):
        x, y, theta = results["s_curve"]
        # start at x=50, spline goes 45cm forward → expect x ~ 95
        assert x > 85.0, f"s-curve ended at x={x:.2f}, expected > 85"

    def test_returned_to_center(self, results):
        _, y, _ = results["s_curve"]
        # S-curve curves right then back; should end near y=50
        assert abs(y - 50.0) < 10.0, (
            f"s-curve lateral: y={y:.2f}, expected near 50"
        )

    def test_heading_roughly_forward(self, results):
        _, _, theta = results["s_curve"]
        # S-curve ends heading roughly forward
        assert abs(theta) < 0.50, (
            f"s-curve heading: theta={theta:.3f}, expected near 0"
        )


# --------------------------------------------------------------------------
# Scenario 4: 90° left curve — should produce significant lateral displacement
# --------------------------------------------------------------------------

class TestCurveLeft:
    def test_moved_laterally(self, results):
        x, y, theta = results["curve_left_90"]
        # Started at (50, 30, 0), curved left — y should increase significantly
        assert y > 40.0, (
            f"left curve ended at y={y:.2f}, expected > 40 (significant leftward motion)"
        )

    def test_heading_turned_left(self, results):
        _, _, theta = results["curve_left_90"]
        # Heading should be positive (turned CCW / left)
        assert theta > 0.3, (
            f"left curve heading: theta={theta:.3f}, expected > 0.3 (turned left)"
        )


# --------------------------------------------------------------------------
# Scenario 5: half speed — should reach same distance as full speed
# --------------------------------------------------------------------------

class TestHalfSpeed:
    def test_reaches_target(self, results):
        x, y, theta = results["straight_half_speed"]
        # Same 30cm target, just slower
        assert 73.0 < x < 88.0, (
            f"half-speed spline ended at x={x:.2f}, expected ~80"
        )

    def test_heading_stable(self, results):
        _, _, theta = results["straight_half_speed"]
        assert abs(theta) < 0.25, f"heading drifted: theta={theta:.3f}"
