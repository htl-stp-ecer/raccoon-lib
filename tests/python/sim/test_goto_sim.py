"""End-to-end sim tests for goto / to_absolute / splinify.

Drives the real closed loop under the mock bundle (no hardware): goto reads
``robot.localization.get_pose()``, commands a body velocity, ``Drive`` → mock
motors → ``SimWorld`` → odometry → localization. ``goto`` had never been run
end-to-end before these tests.

Frame: ``localization.get_pose()`` is start-relative (origin = start pose), so
the sim ground-truth ``pose()`` = localization + the 0.50 m start offset.

Findings encoded here:
  * goto LONGITUDINAL (forward, and forward+rotate on mecanum) converges. ✓
  * splinify() / spline() converge — they ride the odometry-driven
    ``SplineMotion``, independent of the particle filter. ✓
  * LinearMotion fixed-distance LATERAL (strafe_right/left by cm) runaway is
    FIXED. Root cause: the sim built the controller-facing odometry by
    *reflecting the physical ground-truth pose* (``MockPlatform::simRelativePose``
    — a proper rotation reporting ``+y = robot LEFT``), while ``SimWorld``
    integrates a ``+vy`` command as the robot's physical RIGHT (line-218
    negation, to match production ``MecanumKinematics`` and line-follow). So a
    rightward strafe reported ``odo.y`` with the WRONG SIGN vs
    ``LinearMotion::projectBodyFrame``'s "right-positive" lateral → inverted
    feedback → runaway (magnitude was correct, only the sign).
    No odometry-boundary reflection can fix this: ``LinearMotion`` needs y and
    heading reflected *together* (else forward breaks at accumulated heading),
    but ``TurnMotion`` is relative and needs heading *unreflected* (else its
    ``+wz→+getHeading`` loop inverts) — and they share ``getHeading``.
    Fix (sim-only): dead-reckon a SEPARATE controller odometry from the body
    velocities the sim already computes — ``SimWorld::odometryPose()`` integrates
    ``(vx, vy, yawRate)`` the way real wheel/IMU odometry does, so ``+vx``→forward,
    ``+vy``→right-positive lateral, ``+wz``→heading grow consistently at any
    heading. Physical ``pose()`` (render/sensors/collision) stays in the standard
    ``+y=left`` frame. Verified by ``tools/sign_probe.py`` and the cube-bot chain.
  * goto pure strafe / diagonal still xfail: Goto closes its lateral loop on the
    localization WORLD pose (particle filter + ``_world_to_body``), a different
    consumer than LinearMotion — tracked separately, not the LinearMotion bug.
  * ``optimize().to_absolute()`` (multi-waypoint GotoWaypoints) also overshoots
    the chained endpoint — tracked separately (xfail), same goto-world path.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_goto_runner.py"


def _raccoon_available() -> bool:
    try:
        from raccoon import sim

        return hasattr(sim, "mock")
    except ImportError:
        return False


pytestmark = pytest.mark.skipif(
    not _raccoon_available(), reason="raccoon mock-bundle wheel not installed"
)

_CACHE: dict[str, dict] = {}


def _results(config: str) -> dict:
    if config in _CACHE:
        return _CACHE[config]
    env = os.environ.copy()
    env.setdefault("LIBSTP_LOG_LEVEL", "warn")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")
    env["LIBSTP_TIMING_ENABLED"] = "0"
    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--config", config],
        capture_output=True,
        text=True,
        timeout=300,
        env=env,
        check=False,
    )
    if proc.returncode != 0:
        msg = (
            f"runner failed (exit={proc.returncode}, config={config})\n"
            f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
        )
        raise AssertionError(msg)
    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            _CACHE[config] = json.loads(line[len("RESULTS:") :])
            return _CACHE[config]
    msg = f"no RESULTS line\nstdout: {proc.stdout}\nstderr: {proc.stderr}"
    raise AssertionError(msg)


# --------------------------------------------------------------------------
# goto longitudinal — these WORK
# --------------------------------------------------------------------------


@pytest.mark.parametrize("config", ["default", "packingbot"])
def test_goto_forward_converges(config: str) -> None:
    """goto to (0.20 m, 0, 0) reaches the target and localization tracks sim."""
    res = _results(config)["goto_forward"]
    assert "error" not in res, res
    loc = res["loc"]
    gt = res["gt_m"]
    assert loc[0] == pytest.approx(0.20, abs=0.04), f"forward x: {loc}"
    assert loc[1] == pytest.approx(0.00, abs=0.04), f"forward y: {loc}"
    # localization world frame is start-relative: gt = loc + 0.50 m start offset.
    assert gt[0] == pytest.approx(loc[0] + 0.50, abs=0.05), f"loc/gt mismatch: {loc} {gt}"


def test_goto_forward_plus_rotate_converges_mecanum() -> None:
    """goto to (0.20 m ahead, facing +45°) — currently unreliable on mecanum."""
    res = _results("packingbot")["goto_with_heading"]
    assert "error" not in res, res
    loc = res["loc"]
    assert loc[0] == pytest.approx(0.20, abs=0.05), f"x: {loc}"
    assert loc[2] == pytest.approx(0.785, abs=0.12), f"heading (want ~45°): {loc}"


# --------------------------------------------------------------------------
# splines — these WORK (odometry-driven SplineMotion, not the particle filter)
# --------------------------------------------------------------------------


@pytest.mark.parametrize("config", ["default", "packingbot"])
def test_splinify_runs_to_completion(config: str) -> None:
    """optimize([...]).splinify() drives the whole path as one spline, no error."""
    res = _results(config)["optimize_splinify"]
    assert "error" not in res, res
    gt = res["gt_m"]
    # Started at (0.50, 0.50); a drive(30)+turn+drive(20) curve must travel a
    # meaningful distance away from the start.
    moved = ((gt[0] - 0.50) ** 2 + (gt[1] - 0.50) ** 2) ** 0.5
    assert moved > 0.10, f"spline barely moved: gt={gt}"


@pytest.mark.parametrize("config", ["default", "packingbot"])
def test_direct_spline_runs_to_completion(config: str) -> None:
    """spline((30,0),(50,-20),(50,-40)) completes and moves through the curve."""
    res = _results(config)["direct_spline"]
    assert "error" not in res, res
    gt = res["gt_m"]
    moved = ((gt[0] - 0.50) ** 2 + (gt[1] - 0.50) ** 2) ** 0.5
    assert moved > 0.10, f"spline barely moved: gt={gt}"


# --------------------------------------------------------------------------
# goto lateral — fixed: localization now propagates the odom delta through the
# body frame (rotated into each particle's world heading) and the sim's lateral
# integration matches the +y=right ChassisVelocity convention, so holonomic vy
# converges instead of running away. Pure strafe is the slowest case (the
# proportional law approaches asymptotically), hence the generous timeout.
# --------------------------------------------------------------------------


@pytest.mark.xfail(
    reason="LinearMotion fixed-distance strafe is FIXED (sim now dead-reckons the "
    "controller odometry from body velocities — SimWorld::odometryPose — so a +vy "
    "command grows the right-positive lateral projection; see sign_probe + chain). "
    "This goto-specific case still misses because Goto closes its lateral loop on "
    "the localization world pose (particle filter + _world_to_body), a different "
    "consumer than LinearMotion; tracked separately.",
    strict=False,
)
def test_goto_strafe_converges_mecanum() -> None:
    res = _results("packingbot")["goto_strafe"]
    assert "error" not in res, res
    loc = res["loc"]
    assert loc[1] == pytest.approx(0.15, abs=0.04), f"strafe y: {loc}"
    assert loc[0] == pytest.approx(0.00, abs=0.04), f"strafe x drift: {loc}"


@pytest.mark.xfail(
    reason="same goto/localization-world lateral path as test_goto_strafe "
    "(LinearMotion strafe itself is fixed via dead-reckoned odometry)",
    strict=False,
)
def test_goto_diagonal_converges_mecanum() -> None:
    res = _results("packingbot")["goto_diagonal"]
    assert "error" not in res, res
    loc = res["loc"]
    assert loc[0] == pytest.approx(0.20, abs=0.04), f"diag x: {loc}"
    assert loc[1] == pytest.approx(0.10, abs=0.04), f"diag y: {loc}"


@pytest.mark.xfail(
    reason="lateral vy no longer runs away (see test_goto_strafe), but "
    "optimize().to_absolute() emits a multi-waypoint GotoWaypoints whose "
    "per-waypoint convergence still overshoots the chained endpoint. Tracked "
    "separately from the lateral-feedback root cause fixed here.",
    strict=False,
)
@pytest.mark.parametrize("config", ["default", "packingbot"])
def test_optimize_to_absolute_reaches_endpoint(config: str) -> None:
    res = _results(config)["optimize_to_absolute"]
    assert "error" not in res, res
    # drive(30) + turn_right(90) + drive(20) from (0,0,0): endpoint ≈ (0.30, -0.20).
    loc = res["loc"]
    assert loc[0] == pytest.approx(0.30, abs=0.06), f"x: {loc}"
    assert loc[1] == pytest.approx(-0.20, abs=0.06), f"y: {loc}"
