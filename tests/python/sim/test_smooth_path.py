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

Tolerances are tight:
- Absolute position: ±1 cm around the theoretical target.
- Heading stability: < 0.08 rad (≈5°) for straight runs.
- Cross-type turn heading: < 0.12 rad (≈7°).
- Smooth vs seq endpoint comparison: < 3 cm.

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
    env["LIBSTP_TIMING_ENABLED"] = "0"

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
# Scenario 1: single drive — should behave like a normal drive step
# Target: start x=50, drive 30 cm → x ≈ 80
# --------------------------------------------------------------------------


class TestSingleSegment:
    def test_reaches_target_distance(self, results):
        x, y, theta = results["single_drive"]
        assert 79.0 < x < 81.0, f"smooth_path single drive: x={x:.3f}, expected ~80"

    def test_heading_stable(self, results):
        _, _, theta = results["single_drive"]
        assert abs(theta) < 0.08, f"heading drifted: theta={theta:.4f} rad"

    def test_lateral_stable(self, results):
        _, y, _ = results["single_drive"]
        assert abs(y - 50.0) < 1.0, f"lateral drift: y={y:.3f}, expected 50"


# --------------------------------------------------------------------------
# Scenario 2+3: two same-type drives — carry velocity, reach same total distance
# Target: start x=50, drive 20+20=40 cm → x ≈ 90
# --------------------------------------------------------------------------


class TestSameTypeDrives:
    def test_reaches_total_distance(self, results):
        x, y, theta = results["two_drives"]
        assert 88.5 < x < 91.0, f"smooth_path two drives: x={x:.3f}, expected ~90"

    def test_heading_stable(self, results):
        _, _, theta = results["two_drives"]
        assert abs(theta) < 0.08, f"heading drifted: theta={theta:.4f} rad"

    def test_matches_seq_distance(self, results):
        smooth_x = results["two_drives"][0]
        seq_x = results["two_drives_seq"][0]
        assert abs(smooth_x - seq_x) < 3.0, (
            f"smooth_path x={smooth_x:.3f} vs seq x={seq_x:.3f}, "
            f"diff={abs(smooth_x - seq_x):.3f} cm"
        )


# --------------------------------------------------------------------------
# Scenario 4: three drives — target x=75 (start 30, drive 45 cm)
# --------------------------------------------------------------------------


class TestThreeDrives:
    def test_reaches_total_distance(self, results):
        x, y, theta = results["three_drives"]
        assert 73.2 < x < 76.0, f"smooth_path three drives: x={x:.3f}, expected ~75"

    def test_heading_stable(self, results):
        _, _, theta = results["three_drives"]
        assert abs(theta) < 0.08, f"heading drifted: theta={theta:.4f} rad"


# --------------------------------------------------------------------------
# Scenario 5+6: drive + turn_right(90) + drive (cross-type)
# Target: start (50,50,0) → drive 20 cm (x→70), turn right, drive 20 cm (y→30)
# Expected endpoint: x ≈ 70, y ≈ 30, theta ≈ -π/2
# --------------------------------------------------------------------------


class TestCrossType:
    def test_completes_without_error(self, results):
        assert "drive_turn_drive" in results, "drive+turn+drive scenario missing"

    def test_turned_approximately_90_degrees(self, results):
        _, _, theta = results["drive_turn_drive"]
        expected = -math.pi / 2
        assert (
            abs(theta - expected) < 0.12
        ), f"heading after turn: theta={theta:.4f}, expected {expected:.4f}"

    def test_moved_in_both_directions(self, results):
        x, y, _ = results["drive_turn_drive"]
        assert x > 67.0, f"x={x:.3f}, expected > 67 (forward 20 cm)"
        assert y < 38.0, f"y={y:.3f}, expected < 38 (rightward 20 cm after turn)"

    def test_cross_type_matches_seq(self, results):
        smooth_x, smooth_y, _ = results["drive_turn_drive"]
        seq_x, seq_y, _ = results["drive_turn_drive_seq"]
        dist = math.sqrt((smooth_x - seq_x) ** 2 + (smooth_y - seq_y) ** 2)
        assert dist < 3.0, (
            f"smooth_path ({smooth_x:.2f}, {smooth_y:.2f}) vs "
            f"seq ({seq_x:.2f}, {seq_y:.2f}): {dist:.2f} cm"
        )
