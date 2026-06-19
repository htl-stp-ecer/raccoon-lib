"""Unit tests for the displacement / heading stop conditions.

These conditions measure travel along a single *reference line* (the heading
frozen at ``start()``). That is deliberate: ``.until(after_forward_cm(20))``
means "drive 20 cm along this line", and a line-follow that corrects around
the reference, or a robot that turns away from it, makes correspondingly
less forward progress along the line (at 90° it can never complete).

The bug this module pins down ("after_forward_cm never stops" during line
following): when the robot is **crooked at start()** — e.g. the line-follow
is still steering onto the line — freezing the *instantaneous* heading pins
the reference to a skewed line, and the subsequent straight drive along the
real line never reaches the target. The fix is ``heading=<deg>``, which pins
the reference to a *targeted absolute* heading via the HeadingReferenceService
(the same reference the motion controllers hold), so the line you intend to
drive is the line that is measured.

The conditions are loaded directly from the workspace source so the test
exercises the code that actually deploys, independent of the (stale)
site-packages install. They are pure Python and need no compiled wheel, so
this module never gets skipped.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

_CONDITION_SRC = (
    Path(__file__).resolve().parents[2] / "modules/libstp-step/python/raccoon/step/condition.py"
)


def _load_condition_module():
    spec = importlib.util.spec_from_file_location("_cond_under_test", _CONDITION_SRC)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cond = _load_condition_module()
after_forward_cm = cond.after_forward_cm
after_lateral_cm = cond.after_lateral_cm
after_degrees = cond.after_degrees


class _FakePose:
    def __init__(self, x: float, y: float, heading: float) -> None:
        self.position = [x, y, 0.0]
        self.heading = heading


class _FakeOdometry:
    """Kinematic odometry: drive at the current heading, steer with omega."""

    def __init__(self, *, x: float = 0.0, y: float = 0.0, heading: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.heading = heading

    def drive(self, *, v: float, omega: float = 0.0, dt: float = 0.01) -> None:
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt
        self.heading += omega * dt

    def turn_in_place(self, target_heading: float, *, rate: float = math.radians(180)) -> None:
        """Rotate the body to ``target_heading`` without translating."""
        steps = max(1, int(abs(target_heading - self.heading) / (rate * 0.01)))
        delta = (target_heading - self.heading) / steps
        for _ in range(steps):
            self.heading += delta
        self.heading = target_heading

    def get_pose(self) -> _FakePose:
        return _FakePose(self.x, self.y, self.heading)


class _FakeHeadingService:
    """Stand-in for HeadingReferenceService: heading=deg -> absolute radians."""

    def __init__(self, reference_rad: float = 0.0) -> None:
        self._reference_rad = reference_rad

    def target_absolute_rad(self, deg: float) -> float:
        return self._reference_rad + math.radians(deg)


def _robot(
    odom: _FakeOdometry, *, heading_service: _FakeHeadingService | None = None, localization=None
) -> SimpleNamespace:
    robot = SimpleNamespace(odometry=odom, localization=localization)
    robot.get_service = lambda _cls: heading_service
    return robot


def _run(cond_obj, odom: _FakeOdometry, robot, drive_kwargs, *, max_steps: int) -> int | None:
    cond_obj.start(robot)
    for i in range(max_steps):
        odom.drive(**drive_kwargs)
        if cond_obj.check(robot):
            return i
    return None


# --------------------------------------------------------------------------
# after_forward_cm — reference-line semantics (intended)
# --------------------------------------------------------------------------


def test_forward_stops_on_straight_drive_along_reference() -> None:
    """Driving straight along the start heading reaches the target."""
    odom = _FakeOdometry(heading=math.radians(40.0))
    robot = _robot(odom)
    fired = _run(after_forward_cm(20), odom, robot, {"v": 0.30}, max_steps=5000)

    assert fired is not None
    forward_cm = 0.30 * (fired + 1) * 0.01 * 100.0
    assert forward_cm == pytest.approx(20.0, abs=1.0)


def test_forward_never_completes_when_driving_90deg_off_reference() -> None:
    """Turning 90° off the reference and driving makes zero forward progress.

    This is intended: the reference line is fixed, so a perpendicular drive
    never advances along it.
    """
    odom = _FakeOdometry(heading=0.0)
    robot = _robot(odom)
    cond_obj = after_forward_cm(20)
    cond_obj.start(robot)  # freeze reference at heading 0
    odom.turn_in_place(math.radians(90.0))  # now point along +y

    fired = None
    for i in range(5000):
        odom.drive(v=0.30)  # drive straight along heading 90
        if cond_obj.check(robot):
            fired = i
            break
    assert fired is None


# --------------------------------------------------------------------------
# after_forward_cm — the bug + fix: crooked start, targeted absolute heading
# --------------------------------------------------------------------------


def test_forward_crooked_start_default_reference_massively_overshoots() -> None:
    """Crooked at start() -> default reference is the skewed instantaneous
    heading, so a straight drive *along the real line* overshoots wildly.

    This is the "never stops driving" symptom: the robot starts pointing 80°
    off the line, immediately steers onto the line, then drives straight —
    but the frozen reference is the 80°-off line, so each centimetre along
    the real line only counts as cos(80°) ≈ 0.17 cm of "forward". To reach
    the 20 cm target it has to drive ~115 cm; as the crooked angle nears 90°
    the required distance diverges to infinity (it never stops at all).
    """
    line_heading = 0.0
    odom = _FakeOdometry(heading=math.radians(80.0))  # crooked at start
    robot = _robot(odom)

    cond_obj = after_forward_cm(20)  # default: freezes the crooked heading
    cond_obj.start(robot)

    odom.turn_in_place(line_heading)  # steer onto the real line
    travelled_cm = 0.0
    fired = None
    for i in range(20000):  # allow up to 60 m of travel
        odom.drive(v=0.30)
        travelled_cm += 0.30 * 0.01 * 100.0
        if cond_obj.check(robot):
            fired = i
            break
    assert fired is not None
    # Intended target was 20 cm; the skewed reference forces a wild overshoot.
    assert travelled_cm > 80.0, (
        f"expected a large overshoot from the crooked reference, "
        f"only travelled {travelled_cm:.0f} cm"
    )


def test_forward_targeted_absolute_heading_completes_from_crooked_start() -> None:
    """heading=<deg> pins the reference to the intended line -> completes.

    Same crooked start as above, but the condition references the targeted
    absolute heading (the line), so once the robot is on the line the forward
    distance accumulates correctly and the 20 cm target fires.

    RED before the fix: ``after_forward_cm`` did not accept ``heading=`` and
    this raised ``TypeError`` at construction.
    """
    line_heading = 0.0
    odom = _FakeOdometry(heading=math.radians(80.0))
    service = _FakeHeadingService(reference_rad=0.0)
    robot = _robot(odom, heading_service=service)

    cond_obj = after_forward_cm(20, heading=0)  # reference the heading-0 line
    cond_obj.start(robot)

    odom.turn_in_place(line_heading)  # steer onto the line
    travelled_cm = 0.0
    fired = None
    for i in range(8000):
        odom.drive(v=0.30)
        travelled_cm += 0.30 * 0.01 * 100.0
        if cond_obj.check(robot):
            fired = i
            break
    assert fired is not None, "targeted-heading reference must complete on the line"
    assert travelled_cm == pytest.approx(20.0, abs=1.0)


def test_forward_heading_and_absolute_are_mutually_exclusive() -> None:
    with pytest.raises(ValueError, match="mutually exclusive"):
        after_forward_cm(20, heading=0, absolute=True)


def test_forward_backward_negative_target() -> None:
    odom = _FakeOdometry(heading=math.radians(115.0))
    robot = _robot(odom)
    fired = _run(after_forward_cm(-15), odom, robot, {"v": -0.30}, max_steps=5000)

    assert fired is not None
    backward_cm = 0.30 * (fired + 1) * 0.01 * 100.0
    assert backward_cm == pytest.approx(15.0, abs=1.0)


# --------------------------------------------------------------------------
# after_lateral_cm
# --------------------------------------------------------------------------


def test_lateral_does_not_fire_on_pure_forward() -> None:
    odom = _FakeOdometry(heading=math.radians(25.0))
    robot = _robot(odom)
    fired = _run(after_lateral_cm(10), odom, robot, {"v": 0.30}, max_steps=2000)
    assert fired is None


def test_lateral_targeted_heading_accepts_heading_kwarg() -> None:
    service = _FakeHeadingService(reference_rad=0.0)
    odom = _FakeOdometry(heading=math.radians(30.0))
    robot = _robot(odom, heading_service=service)
    # heading=0 reference: strafing along +y (lateral of the heading-0 line)
    cond_obj = after_lateral_cm(10, heading=0)
    cond_obj.start(robot)

    fired = None
    for i in range(5000):
        odom.y += 0.25 * 0.01  # pure +y strafe
        if cond_obj.check(robot):
            fired = i
            break
    assert fired is not None
    lateral_cm = 0.25 * (fired + 1) * 0.01 * 100.0
    assert lateral_cm == pytest.approx(10.0, abs=1.0)


# --------------------------------------------------------------------------
# after_degrees — must read heading from odometry, not localization (WIP)
# --------------------------------------------------------------------------


def test_after_degrees_uses_odometry_not_localization() -> None:
    """after_degrees must work off robot.odometry (localization is WIP).

    A robot with a live odometry heading but NO localization service must be
    able to use after_degrees. The old implementation hard-required
    robot.localization and raised without it.
    """
    odom = _FakeOdometry()
    robot = _robot(odom, localization=None)
    cond_obj = after_degrees(90)

    cond_obj.start(robot)
    fired = None
    for i in range(5000):
        odom.heading += math.radians(60.0) * 0.01  # spin at 60 deg/s
        if cond_obj.check(robot):
            fired = i
            break
    assert fired is not None, "after_degrees must fire using the odometry heading"
    assert math.degrees(odom.heading) == pytest.approx(90.0, abs=2.0)
