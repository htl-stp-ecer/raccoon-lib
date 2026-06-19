"""Unit tests for the closed-loop ``goto`` navigate-to-pose step.

The proportional control law (``_compute_body_velocity``) is pure and is
tested directly with tiny pose-like / target-like duck types — no robot and
no native module required. A separate construction test (guarded by
``requires_libstp``) checks the public ``goto()`` factory and signature.

Sign conventions under test (verified against the HAL ``ChassisVelocity`` and
the executor ``_world_to_body``):
  - vx > 0 = body-forward
  - vy > 0 = body-right   (target to the LEFT therefore yields vy < 0)
  - wz > 0 = counter-clockwise
"""

from __future__ import annotations

import importlib.util
import math

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# Tiny duck types so the pure control law is testable without C++ Pose.
# ---------------------------------------------------------------------------


class FakePose:
    def __init__(self, x: float, y: float, heading: float) -> None:
        self.position = (x, y, 0.0)
        self.heading = heading


class FakeTarget:
    def __init__(self, x_m: float, y_m: float, theta_rad: float | None = None) -> None:
        self.x_m = x_m
        self.y_m = y_m
        self.theta_rad = theta_rad


# goto.py imports `from raccoon.foundation import ChassisVelocity` and
# `from .. import dsl` at module scope, so loading the (pure) control law still
# requires the native package — hence the module-wide guard.
pytestmark = requires_libstp


@pytest.fixture(scope="module")
def compute():
    import importlib

    mod = importlib.import_module("raccoon.step.motion.goto")
    return mod._compute_body_velocity


# ---------------------------------------------------------------------------
# Pure control-law tests
# ---------------------------------------------------------------------------


def test_at_target_is_reached_and_zero_velocity(compute):
    cmd = compute(FakePose(1.0, 0.5, 0.0), FakeTarget(1.0, 0.5, 0.0))
    assert cmd.reached is True
    assert cmd.vx == pytest.approx(0.0)
    assert cmd.vy == pytest.approx(0.0)
    assert cmd.wz == pytest.approx(0.0)


def test_target_straight_ahead_positive_vx(compute):
    # Robot at origin facing +X (heading 0), target 1 m ahead.
    cmd = compute(FakePose(0.0, 0.0, 0.0), FakeTarget(1.0, 0.0, None))
    assert cmd.vx > 0.0
    assert cmd.vy == pytest.approx(0.0, abs=1e-9)
    assert cmd.reached is False


def test_target_to_the_left_negative_vy(compute):
    # Robot facing +X; target 1 m to the LEFT is +Y in world. Body-right vy is
    # NEGATIVE because vy>0 means right (HAL ChassisVelocity convention).
    cmd = compute(FakePose(0.0, 0.0, 0.0), FakeTarget(0.0, 1.0, None))
    assert cmd.vy < 0.0
    assert cmd.vx == pytest.approx(0.0, abs=1e-9)
    assert cmd.reached is False


def test_target_to_the_right_positive_vy(compute):
    cmd = compute(FakePose(0.0, 0.0, 0.0), FakeTarget(0.0, -1.0, None))
    assert cmd.vy > 0.0
    assert cmd.vx == pytest.approx(0.0, abs=1e-9)


def test_target_behind_negative_vx(compute):
    cmd = compute(FakePose(0.0, 0.0, 0.0), FakeTarget(-1.0, 0.0, None))
    assert cmd.vx < 0.0
    assert cmd.vy == pytest.approx(0.0, abs=1e-9)


def test_heading_error_only_rotates_until_within_tol(compute):
    # Position already on target; heading is off by +90° (target CCW).
    pose = FakePose(0.0, 0.0, 0.0)
    target = FakeTarget(0.0, 0.0, math.radians(90.0))
    cmd = compute(pose, target)
    assert cmd.vx == pytest.approx(0.0)
    assert cmd.vy == pytest.approx(0.0)
    assert cmd.wz > 0.0  # positive = counter-clockwise toward +90°
    assert cmd.reached is False

    # Within heading tolerance -> reached, no rotation.
    pose_aligned = FakePose(0.0, 0.0, math.radians(89.0))
    cmd2 = compute(pose_aligned, target, heading_tol_rad=math.radians(3.0))
    assert cmd2.reached is True
    assert cmd2.wz == pytest.approx(0.0)


def test_heading_clockwise_negative_wz(compute):
    pose = FakePose(0.0, 0.0, 0.0)
    target = FakeTarget(0.0, 0.0, math.radians(-90.0))
    cmd = compute(pose, target)
    assert cmd.wz < 0.0  # negative = clockwise


def test_pose_rotated_90deg_rotates_error_into_body_frame(compute):
    # Robot at origin but rotated +90° (facing world +Y). Target is 1 m along
    # world +X, which is now to the robot's RIGHT. So body-forward (vx) should
    # be ~0 and body-lateral should be a forward->lateral swap: world-+X with
    # heading 90° -> body-right, i.e. vy > 0.
    pose = FakePose(0.0, 0.0, math.radians(90.0))
    target = FakeTarget(1.0, 0.0, None)
    cmd = compute(pose, target)
    assert cmd.vx == pytest.approx(0.0, abs=1e-9)
    assert cmd.vy > 0.0


def test_speed_scales_command(compute):
    target = FakeTarget(1.0, 0.0, None)
    full = compute(FakePose(0.0, 0.0, 0.0), target, speed=1.0)
    half = compute(FakePose(0.0, 0.0, 0.0), target, speed=0.5)
    assert half.vx == pytest.approx(full.vx * 0.5)


def test_position_reached_but_heading_off_not_reached(compute):
    # Inside position tol, but heading requested and off -> translational zero,
    # still rotating, reached False.
    pose = FakePose(1.0, 1.0, 0.0)
    target = FakeTarget(1.0, 1.0, math.radians(45.0))
    cmd = compute(pose, target)
    assert cmd.vx == pytest.approx(0.0)
    assert cmd.vy == pytest.approx(0.0)
    assert cmd.wz != 0.0
    assert cmd.reached is False


def test_no_theta_means_heading_ignored(compute):
    # Position reached, no theta -> reached True regardless of heading.
    pose = FakePose(0.0, 0.0, math.radians(123.0))
    target = FakeTarget(0.0, 0.0, None)
    cmd = compute(pose, target)
    assert cmd.reached is True
    assert cmd.wz == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Construction / factory test
# ---------------------------------------------------------------------------


@requires_libstp
def test_goto_factory_builds_and_signature():
    from raccoon.step.motion import goto

    step = goto(100, 50, theta_deg=90)
    assert step._generate_signature() == "Goto(x=1.00, y=0.50, theta=90.0)"
    # cm -> m, deg -> rad conversion happened in the factory.
    assert step._target.x_m == pytest.approx(1.0)
    assert step._target.y_m == pytest.approx(0.5)
    assert step._target.theta_rad == pytest.approx(math.radians(90.0))
    assert step._pos_tol_m == pytest.approx(0.02)
    assert "localization" in step.required_resources()


@requires_libstp
def test_goto_factory_no_theta_signature():
    from raccoon.step.motion import goto

    step = goto(30, 0)
    assert step._generate_signature() == "Goto(x=0.30, y=0.00, theta=None)"
    assert step._target.theta_rad is None
