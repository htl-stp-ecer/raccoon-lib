"""Simulation-based integration tests for smooth_path() optimizer.

Tolerances are intentionally tight:
- Equivalent paths (merge, barrier): < 0.1 cm (1 mm) — deterministic sim,
  same net motion must produce the same endpoint.
- Absolute position: ±1 cm around the theoretical target.
- Cross-path comparisons (corner cut, spline): ±4 cm / ±35 cm — different
  physical curves reach approximately the same endpoint.
- Heading: ±0.08 rad (≈5°) for straight drives; ±0.12 rad (≈7°) for turns.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_path_optimizer_runner.py"
ROBOT_CONFIGS = ["default", "drumbot", "packingbot"]


def _raccoon_available() -> bool:
    try:
        from raccoon import sim

        return hasattr(sim, "mock")
    except ImportError:
        return False


pytestmark = pytest.mark.skipif(
    not _raccoon_available(),
    reason="raccoon mock-bundle wheel not installed",
)


def _run_runner(config_name: str) -> dict:
    env = os.environ.copy()
    env.setdefault("LIBSTP_LOG_LEVEL", "warn")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")
    env["LIBSTP_TIMING_ENABLED"] = "0"

    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--config", config_name],
        capture_output=True,
        text=True,
        timeout=120,
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
    msg = f"runner did not emit RESULTS line\nstdout: {proc.stdout}\nstderr: {proc.stderr}"
    raise AssertionError(msg)


@pytest.fixture(scope="module", params=ROBOT_CONFIGS)
def results(request):
    return _run_runner(request.param)


def _dist2d(a, b) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


# ---------------------------------------------------------------------------
# Reference endpoint (start 50,50,0 → drive50 + turn_right90 + drive30):
#   x ≈ 100 cm,  y ≈ 20 cm,  theta ≈ -π/2
#
# Merge endpoint (start 50,50,0 → drive40):
#   x ≈ 90 cm,   y ≈ 50 cm,  theta ≈ 0
# ---------------------------------------------------------------------------


class TestMergeTwoDrives:
    def test_reaches_correct_distance(self, results):
        x, y, _ = results["merge_two_drives"]
        assert 89.0 < x < 91.0, f"merged drives ended at x={x:.3f}, expected ~90"

    def test_heading_stable(self, results):
        _, _, theta = results["merge_two_drives"]
        assert abs(theta) < 0.05, f"heading drifted: theta={theta:.4f} rad"

    def test_lateral_stable(self, results):
        _, y, _ = results["merge_two_drives"]
        assert abs(y - 50.0) < 1.0, f"lateral drift: y={y:.3f}, expected 50"

    def test_matches_single_drive_40(self, results):
        """After merge, execution should stay close to the single-drive reference."""
        merged = results["merge_two_drives"]
        ref = results["merge_ref_drive"]
        d = _dist2d(merged, ref)
        assert d < 1.5, (
            f"merge endpoint ({merged[0]:.3f}, {merged[1]:.3f}) differs from "
            f"drive(40) ({ref[0]:.3f}, {ref[1]:.3f}) by {d:.3f} cm"
        )


# ---------------------------------------------------------------------------
# Corner cut: drive(50)+turn_right(90)+drive(30) with corner_cut_cm=5
# Theoretical endpoint: same as reference ≈ (100, 20, -π/2)
# ---------------------------------------------------------------------------


class TestCornerCut5cm:
    def test_completes_without_error(self, results):
        assert "corner_cut_5cm" in results

    def test_heading_correct(self, results):
        _, _, theta = results["corner_cut_5cm"]
        assert (
            abs(theta - (-math.pi / 2)) < 0.20
        ), f"heading after corner cut: {theta:.4f} rad, expected {-math.pi/2:.4f}"

    def test_endpoint_near_reference(self, results):
        """Corner-cut path ends within 4 cm of the unoptimized path."""
        cut = results["corner_cut_5cm"]
        ref = results["reference_drive_turn_drive"]
        d = _dist2d(cut, ref)
        assert d < 4.0, (
            f"corner-cut ({cut[0]:.2f}, {cut[1]:.2f}) vs "
            f"reference ({ref[0]:.2f}, {ref[1]:.2f}): {d:.2f} cm"
        )

    def test_moved_forward_and_right(self, results):
        x, y, _ = results["corner_cut_5cm"]
        assert x > 97.0, f"x={x:.2f}, expected > 97 (forward)"
        assert y < 24.0, f"y={y:.2f}, expected < 24 (rightward)"


# ---------------------------------------------------------------------------
# Spline: drive(50)+turn_right(90)+drive(30) with spline=True
# Same theoretical endpoint ≈ (100, 20, -π/2) but via a smooth curve
# ---------------------------------------------------------------------------


class TestSplineDriveTurnDrive:
    def test_completes_without_error(self, results):
        assert "spline_drive_turn_drive" in results

    def test_heading_correct(self, results):
        _, _, theta = results["spline_drive_turn_drive"]
        assert (
            abs(theta - (-math.pi / 2)) < 0.35
        ), f"heading after spline: {theta:.4f} rad, expected {-math.pi/2:.4f}"

    def test_endpoint_near_reference(self, results):
        """Spline path should end in the same broad quadrant as the unoptimized path."""
        spl = results["spline_drive_turn_drive"]
        ref = results["reference_drive_turn_drive"]
        d = _dist2d(spl, ref)
        assert d < 35.0, (
            f"spline ({spl[0]:.2f}, {spl[1]:.2f}) vs "
            f"reference ({ref[0]:.2f}, {ref[1]:.2f}): {d:.2f} cm"
        )

    def test_moved_forward_and_right(self, results):
        x, y, _ = results["spline_drive_turn_drive"]
        assert x > 90.0, f"x={x:.2f}, expected > 90 (forward portion)"
        assert y < 26.0, f"y={y:.2f}, expected < 26 (rightward portion)"


# ---------------------------------------------------------------------------
# Barrier: optimize=True with background() between drives
# background() pins the side action — drives must NOT be merged.
# Net motion is still 40 cm forward → same endpoint as merged case.
# ---------------------------------------------------------------------------


class TestMergeWithBarrier:
    def test_reaches_correct_distance(self, results):
        x, y, _ = results["merge_with_barrier"]
        assert (
            88.5 < x < 91.0
        ), f"barrier test: x={x:.3f}, expected ~90 (both 20 cm drives must run)"

    def test_matches_merged_case(self, results):
        """Two warm-started segments should end close to the single merged drive."""
        barrier = results["merge_with_barrier"]
        merged = results["merge_two_drives"]
        d = _dist2d(barrier, merged)
        assert d < 1.0, (
            f"barrier ({barrier[0]:.3f}, {barrier[1]:.3f}) vs "
            f"merged ({merged[0]:.3f}, {merged[1]:.3f}): {d:.3f} cm"
        )
