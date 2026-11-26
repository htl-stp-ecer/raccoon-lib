"""Warnings and errors of the ``spline()`` heading options."""

from __future__ import annotations

import importlib.util
import math

import pytest

requires_libstp = pytest.mark.skipif(
    importlib.util.find_spec("raccoon") is None,
    reason="raccoon native module not installed",
)

TANGENT = ((30, 0), (50, 20))
EXPLICIT = ((30, 0, 0), (50, 20, 45))


class _Pose:
    heading = 0.0


class _Odometry:
    def __init__(self):
        self.pose = _Pose()

    def get_pose(self):
        return self.pose


class _Drive:
    def supports_lateral_motion(self):
        return True


class _Robot:
    localization = None
    drive = _Drive()
    motion_pid_config = object()

    def __init__(self):
        self.odometry = _Odometry()
        self._services = {}

    def get_service(self, cls):
        return self._services.setdefault(cls, cls(self))


def _marked_robot(heading_deg):
    """Robot whose reference is its heading now, turned by ``heading_deg`` since."""
    from raccoon.robot.heading_reference import HeadingReferenceService

    robot = _Robot()
    robot.get_service(HeadingReferenceService).mark()
    robot.odometry.pose.heading = math.radians(heading_deg)
    return robot


def _start(monkeypatch, waypoints, robot=None, **options):
    """Run ``on_start`` without native motion; return the logged warnings."""
    import raccoon.step.motion.spline_path as module

    class _Motion:
        def __init__(self, _drive, _odometry, _pid, config):
            robot.config = config

        def start(self):
            pass

    robot = robot or _Robot()
    monkeypatch.setattr(module, "SplineMotion", _Motion)
    step = module.spline(*waypoints, **options)
    warnings = []
    step.warn = warnings.append
    step.on_start(robot)
    return warnings


@requires_libstp
@pytest.mark.parametrize(
    ("waypoints", "options", "warning"),
    [
        (TANGENT, {"absolute_heading": True}, "absolute_heading=True has no effect"),
        (EXPLICIT, {"final_heading": 0}, "final_heading has no effect"),
    ],
)
def test_warns_about_options_without_effect(monkeypatch, waypoints, options, warning):
    warnings = _start(monkeypatch, waypoints, **options)

    assert len(warnings) == 1
    assert warning in warnings[0]


@requires_libstp
@pytest.mark.parametrize(
    ("waypoints", "options"),
    [(TANGENT, {}), (EXPLICIT, {}), (TANGENT, {"final_heading": 90})],
)
def test_relative_headings_need_no_reference(monkeypatch, waypoints, options):
    assert _start(monkeypatch, waypoints, **options) == []


@requires_libstp
@pytest.mark.parametrize(
    ("waypoints", "options"),
    [(EXPLICIT, {}), (TANGENT, {"final_heading": 0})],
)
def test_absolute_heading_without_reference_raises(monkeypatch, waypoints, options):
    with pytest.raises(RuntimeError, match="Call mark_heading_reference"):
        _start(monkeypatch, waypoints, absolute_heading=True, **options)


@requires_libstp
def test_absolute_waypoint_headings_are_measured_from_the_reference(monkeypatch):
    robot = _marked_robot(heading_deg=30)

    _start(monkeypatch, EXPLICIT, robot=robot, absolute_heading=True)

    # Reference 0° and 45°, seen from a start heading of 30°.
    assert robot.config.headings_rad == pytest.approx([math.radians(-30), math.radians(15)])


@requires_libstp
def test_absolute_final_heading_is_measured_from_the_reference(monkeypatch):
    robot = _marked_robot(heading_deg=30)

    _start(monkeypatch, TANGENT, robot=robot, absolute_heading=True, final_heading=90)

    assert robot.config.final_heading_rad == pytest.approx(math.radians(60))
