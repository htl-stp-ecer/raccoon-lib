"""Regression tests for ``WaitUntilDistance``.

The step runs inside a ``parallel()`` branch alongside a drive and fires once
the robot has driven a threshold distance. Motion steps no longer reset
odometry on start (the world pose lives in the continuously-accumulating
localization frame), so the step must measure displacement relative to its
OWN start — not the absolute distance from the mission's odometry origin.

Regression: previously it compared the absolute ``straight_line`` distance
from origin against the threshold, so mid-mission (where the origin distance
is already large) it returned on the first poll instead of after driving the
requested distance.
"""

from __future__ import annotations

import asyncio

from raccoon.step.motion.at_distance import WaitUntilDistance


class _Dist:
    def __init__(self, forward=0.0, lateral=0.0, straight_line=0.0):
        self.forward = forward
        self.lateral = lateral
        self.straight_line = straight_line


class _ScriptedOdometry:
    """Returns a scripted sequence of distance samples, one per poll."""

    def __init__(self, samples):
        self._samples = list(samples)
        self.polls = 0

    def get_distance_from_origin(self):
        # Hold the last sample once the script is exhausted.
        idx = min(self.polls, len(self._samples) - 1)
        self.polls += 1
        return self._samples[idx]


class _Robot:
    def __init__(self, odometry):
        self.odometry = odometry


def _run(step, robot):
    asyncio.run(step._execute_step(robot))


def test_does_not_fire_immediately_when_origin_distance_already_large():
    # Mid-mission: robot is 1.2 m from the odometry origin but has not moved
    # since this step began. It must NOT complete on the first poll.
    forward = 1.2
    samples = [
        _Dist(forward=forward + i * 0.02, straight_line=forward + i * 0.02) for i in range(20)
    ]
    odo = _ScriptedOdometry(samples)
    step = WaitUntilDistance(cm=15)  # 0.15 m threshold

    _run(step, _Robot(odo))

    # 0.15 m at 0.02 m/poll from a 1.2 m baseline => ~8 polls, well past the
    # first. The buggy version returned after a single poll.
    assert odo.polls >= 7


def test_fires_after_relative_displacement_reached():
    # Baseline (0.5, 0.3); drive straight along +forward. Displacement crosses
    # 0.15 m between forward=0.64 and 0.66.
    samples = [
        _Dist(forward=0.50, lateral=0.30),
        _Dist(forward=0.58, lateral=0.30),
        _Dist(forward=0.64, lateral=0.30),  # traveled 0.14
        _Dist(forward=0.66, lateral=0.30),  # traveled 0.16 -> fire
        _Dist(forward=0.80, lateral=0.30),
    ]
    odo = _ScriptedOdometry(samples)
    step = WaitUntilDistance(cm=15)

    _run(step, _Robot(odo))

    assert odo.polls == 4  # baseline read + 3 loop polls


def test_stall_detection_returns_when_motion_stops():
    # Robot drives a little then stalls before reaching threshold; the step
    # must give up via stall detection rather than hang forever.
    samples = [_Dist(forward=0.0), _Dist(forward=0.05), _Dist(forward=0.05)]
    odo = _ScriptedOdometry(samples)
    step = WaitUntilDistance(cm=50)  # unreachable

    _run(step, _Robot(odo))

    # Held at 0.05 m forever; stall (0.5 s) fires. Just assert it terminated
    # and never reached the (unreachable) target.
    assert odo.polls > 3
