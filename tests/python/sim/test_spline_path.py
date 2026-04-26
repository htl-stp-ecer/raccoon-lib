"""End-to-end tests for ``spline()`` against the physics simulator.

Validates that spline path motion:
1. Straight-line spline reaches approximately the same distance as drive_forward
2. S-curve ends near the center line (heading returns to ~0)
3. Curved path produces the expected lateral displacement
4. Speed scaling works (half speed reaches same distance)
5. Heading stays stable during straight segments

Tolerances:
- Absolute position: ±2 cm around the theoretical target (spline kinematics
  are less rigid than linear/turn so a slightly wider band is appropriate).
- Heading stability: < 0.15 rad (≈9°).
- Spline vs drive comparison: < 3 cm.

Only the ``default`` (idealized) config is tested — realistic bots introduce
motor drag that affects spline profile accuracy more than linear drives.

The runner subprocess pattern matches ``test_smooth_path.py`` to isolate
the C++ mock-HAL singleton teardown from pytest.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_spline_path_runner.py"

ROBOT_CONFIGS = ["default"]


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
        msg = (
            f"runner failed (exit={proc.returncode}, config={config_name})\n"
            f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
        )
        raise AssertionError(msg)

    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:") :])
    msg = (
        f"runner did not emit RESULTS line (config={config_name})\n"
        f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
    )
    raise AssertionError(msg)


@pytest.fixture(scope="module", params=ROBOT_CONFIGS)
def results(request):
    """Run all scenarios for each robot config and cache the results."""
    return _run_runner(request.param)


# --------------------------------------------------------------------------
# Scenario 1: straight line — should reach ~30 cm forward
# Target: start x=50, waypoints (15,0)→(30,0) → x ≈ 80
# --------------------------------------------------------------------------


class TestStraightLine:
    def test_reaches_target_distance(self, results):
        x, y, theta = results["straight_line"]
        assert 78.0 < x < 82.0, f"spline straight: x={x:.3f}, expected ~80"

    def test_heading_stable(self, results):
        _, _, theta = results["straight_line"]
        assert abs(theta) < 0.15, f"heading drifted: theta={theta:.4f} rad"

    def test_lateral_stable(self, results):
        _, y, _ = results["straight_line"]
        assert abs(y - 50.0) < 2.0, f"lateral drift: y={y:.3f}, expected ~50"

    def test_matches_drive_forward(self, results):
        spline_x = results["straight_line"][0]
        drive_x = results["drive_forward_30"][0]
        assert (
            abs(spline_x - drive_x) < 3.0
        ), f"spline x={spline_x:.3f} vs drive_forward x={drive_x:.3f}"


# --------------------------------------------------------------------------
# Scenario 3: S-curve — should end near center line, heading ~0
# Start x=50, spline goes 45 cm forward → x ≈ 95
# --------------------------------------------------------------------------


class TestSCurve:
    def test_moved_forward(self, results):
        x, y, theta = results["s_curve"]
        assert x > 90.0, f"s-curve: x={x:.3f}, expected > 90"

    def test_returned_to_center(self, results):
        _, y, _ = results["s_curve"]
        assert abs(y - 50.0) < 5.0, f"s-curve lateral: y={y:.3f}, expected near 50"

    def test_heading_roughly_forward(self, results):
        _, _, theta = results["s_curve"]
        assert abs(theta) < 0.25, f"s-curve heading: theta={theta:.4f}, expected near 0"


# --------------------------------------------------------------------------
# Scenario 4: 90° left curve — should produce significant lateral displacement
# Start (50, 30, 0), curves left
# --------------------------------------------------------------------------


class TestCurveLeft:
    def test_moved_laterally(self, results):
        x, y, theta = results["curve_left_90"]
        assert y > 43.0, f"left curve: y={y:.3f}, expected > 43 (leftward motion)"

    def test_heading_turned_left(self, results):
        _, _, theta = results["curve_left_90"]
        assert theta > 0.4, f"left curve heading: theta={theta:.4f}, expected > 0.4 (turned CCW)"


# --------------------------------------------------------------------------
# Scenario 5: half speed — should reach the same 30 cm target
# --------------------------------------------------------------------------


class TestHalfSpeed:
    def test_reaches_target(self, results):
        x, y, theta = results["straight_half_speed"]
        assert 78.0 < x < 82.0, f"half-speed spline: x={x:.3f}, expected ~80"

    def test_heading_stable(self, results):
        _, _, theta = results["straight_half_speed"]
        assert abs(theta) < 0.15, f"heading drifted: theta={theta:.4f} rad"
