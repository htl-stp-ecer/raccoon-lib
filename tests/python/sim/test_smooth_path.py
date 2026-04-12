"""End-to-end tests for ``smooth_path()`` against the physics simulator.

Validates that smooth_path:
1. Single-segment works identically to a normal drive step
2. Same-type chaining (drive+drive) reaches the correct total distance
3. Cross-type chaining (drive+turn+drive) reaches the correct pose
4. Heading stays stable during straight-line segments

Each scenario is tested with three robot configurations:
- ``default``: idealized robot with no motor drag
- ``drumbot``: differential-drive robot with real calibration + drag
- ``packingbot``: mecanum-drive robot with real calibration + drag

The runner subprocess pattern matches ``test_drive_mission.py`` to isolate
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

RUNNER = Path(__file__).parent / "_smooth_path_runner.py"

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
# Scenario 1: single drive — should behave like a normal drive step
# --------------------------------------------------------------------------

class TestSingleSegment:
    def test_reaches_target_distance(self, results):
        x, y, theta = results["single_drive"]
        # start at x=50, drive 30cm forward -> expect x ~ 80
        assert 74.0 < x < 88.0, f"smooth_path single drive ended at x={x:.2f}, expected ~80"

    def test_heading_stable(self, results):
        _, _, theta = results["single_drive"]
        assert abs(theta) < 0.20, f"heading drifted: theta={theta:.3f}"

    def test_lateral_stable(self, results):
        _, y, _ = results["single_drive"]
        assert abs(y - 50.0) < 4.0, f"lateral drift: y={y:.2f}, expected ~50"


# --------------------------------------------------------------------------
# Scenario 2: two same-type drives — should reach same total distance
# --------------------------------------------------------------------------

class TestSameTypeDrives:
    def test_reaches_total_distance(self, results):
        x, y, theta = results["two_drives"]
        # start at x=50, drive 20+20=40cm -> expect x ~ 90
        assert 83.0 < x < 98.0, (
            f"smooth_path two drives ended at x={x:.2f}, expected ~90"
        )

    def test_heading_stable(self, results):
        _, _, theta = results["two_drives"]
        assert abs(theta) < 0.20, f"heading drifted: theta={theta:.3f}"

    def test_matches_seq_distance(self, results):
        smooth_x = results["two_drives"][0]
        seq_x = results["two_drives_seq"][0]
        # smooth_path should reach at least as far as seq (no overshoot > 5cm)
        assert abs(smooth_x - seq_x) < 6.0, (
            f"smooth_path x={smooth_x:.2f} vs seq x={seq_x:.2f}, "
            f"diff={abs(smooth_x - seq_x):.2f}"
        )


# --------------------------------------------------------------------------
# Scenario 4: three drives
# --------------------------------------------------------------------------

class TestThreeDrives:
    def test_reaches_total_distance(self, results):
        x, y, theta = results["three_drives"]
        # start at x=30, drive 15+15+15=45cm -> expect x ~ 75
        assert 68.0 < x < 84.0, (
            f"smooth_path three drives ended at x={x:.2f}, expected ~75"
        )

    def test_heading_stable(self, results):
        _, _, theta = results["three_drives"]
        assert abs(theta) < 0.20, f"heading drifted: theta={theta:.3f}"


# --------------------------------------------------------------------------
# Scenario 5: drive + turn + drive (cross-type)
# --------------------------------------------------------------------------

class TestCrossType:
    def test_completes_without_error(self, results):
        assert "drive_turn_drive" in results, "drive+turn+drive scenario missing"

    def test_turned_approximately_90_degrees(self, results):
        _, _, theta = results["drive_turn_drive"]
        # turn_right(90) should produce theta ~ -pi/2 (-1.57)
        expected = -math.pi / 2
        assert abs(theta - expected) < 0.35, (
            f"heading after turn: theta={theta:.3f}, expected ~{expected:.3f}"
        )

    def test_moved_in_both_directions(self, results):
        x, y, _ = results["drive_turn_drive"]
        # Started at (50, 50, 0): drove 20cm forward (x++), turned right,
        # drove 20cm forward (now going in -y direction).
        # Expect x ~ 70 (forward 20cm from start)
        assert x > 61.0, f"x={x:.2f}, expected > 61 (first drive segment)"
        # Expect y < 50 (drove in -y after turning right)
        assert y < 44.0, f"y={y:.2f}, expected < 44 (second drive after right turn)"

    def test_cross_type_matches_seq_roughly(self, results):
        smooth_x, smooth_y, _ = results["drive_turn_drive"]
        seq_x, seq_y, _ = results["drive_turn_drive_seq"]
        # Both should end up in roughly the same place
        dist = math.sqrt((smooth_x - seq_x)**2 + (smooth_y - seq_y)**2)
        assert dist < 10.0, (
            f"smooth_path ended at ({smooth_x:.1f}, {smooth_y:.1f}) vs "
            f"seq at ({seq_x:.1f}, {seq_y:.1f}), dist={dist:.1f}"
        )
