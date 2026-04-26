"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: wall_align.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .wall_align import (
    WallAlignForward,
    WallAlignBackward,
    WallAlignStrafeLeft,
    WallAlignStrafeRight,
)


class WallAlignForwardBuilder(StepBuilder):
    """Builder for WallAlignForward. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._speed = 1.0
        self._accel_threshold = 0.5
        self._settle_duration = 0.2
        self._max_duration = 5.0
        self._grace_period = 0.3

    def speed(self, value: float):
        self._speed = value
        return self

    def accel_threshold(self, value: float):
        self._accel_threshold = value
        return self

    def settle_duration(self, value: float):
        self._settle_duration = value
        return self

    def max_duration(self, value: float):
        self._max_duration = value
        return self

    def grace_period(self, value: float):
        self._grace_period = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["speed"] = self._speed
        kwargs["accel_threshold"] = self._accel_threshold
        kwargs["settle_duration"] = self._settle_duration
        kwargs["max_duration"] = self._max_duration
        kwargs["grace_period"] = self._grace_period
        return WallAlignForward(**kwargs)


@dsl(tags=["motion", "wall"])
def wall_align_forward(
    speed: float = 1.0,
    accel_threshold: float = 0.5,
    settle_duration: float = 0.2,
    max_duration: float = 5.0,
    grace_period: float = 0.3,
):
    """
    Drive forward into a wall and align the front of the robot.

    Apply constant forward velocity without heading correction so the robot
    naturally rotates flush against the wall surface.  The step monitors
    gravity-compensated linear acceleration from the IMU and stops once a
    collision spike is detected followed by a short settle period.

    The grace period prevents false triggers from the initial acceleration
    transient when the robot starts moving.

    After the step completes, ``step.bump_result`` contains the impact
    magnitude, estimated wall misalignment angle, and the heading
    correction applied during the settle push.

    Args:
        speed: Drive speed in m/s (default 1.0).
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s² to classify as a bump (default 0.5).  Lower values are more sensitive but may false-trigger on rough surfaces.
        settle_duration: Seconds to keep pushing after the bump is detected, letting the chassis rotate flush (default 0.2).
        max_duration: Safety timeout in seconds — the step finishes even if no bump is detected (default 5.0).
        grace_period: Seconds to ignore acceleration after starting, so the robot's own acceleration doesn't trigger detection (default 0.3).

    Returns:
        A WallAlignForwardBuilder (chainable via ``.speed()``, ``.accel_threshold()``, ``.settle_duration()``, ``.max_duration()``, ``.grace_period()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import wall_align_forward, drive_forward

        # Drive near the wall, then bump-align against it
        seq([drive_forward(30), wall_align_forward()])

        # More sensitive detection at slower speed
        wall_align_forward(speed=0.3, accel_threshold=0.3)
    """
    b = WallAlignForwardBuilder()
    b._speed = speed
    b._accel_threshold = accel_threshold
    b._settle_duration = settle_duration
    b._max_duration = max_duration
    b._grace_period = grace_period
    return b


class WallAlignBackwardBuilder(StepBuilder):
    """Builder for WallAlignBackward. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._speed = 1.0
        self._accel_threshold = 0.5
        self._settle_duration = 0.2
        self._max_duration = 5.0
        self._grace_period = 0.3

    def speed(self, value: float):
        self._speed = value
        return self

    def accel_threshold(self, value: float):
        self._accel_threshold = value
        return self

    def settle_duration(self, value: float):
        self._settle_duration = value
        return self

    def max_duration(self, value: float):
        self._max_duration = value
        return self

    def grace_period(self, value: float):
        self._grace_period = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["speed"] = self._speed
        kwargs["accel_threshold"] = self._accel_threshold
        kwargs["settle_duration"] = self._settle_duration
        kwargs["max_duration"] = self._max_duration
        kwargs["grace_period"] = self._grace_period
        return WallAlignBackward(**kwargs)


@dsl(tags=["motion", "wall"])
def wall_align_backward(
    speed: float = 1.0,
    accel_threshold: float = 0.5,
    settle_duration: float = 0.2,
    max_duration: float = 5.0,
    grace_period: float = 0.3,
):
    """
    Drive backward into a wall and align the back of the robot.

    Apply constant backward velocity without heading correction so the robot
    naturally rotates flush against the wall surface.  Uses IMU bump
    detection to know when the wall has been reached.

    Args:
        speed: Drive speed in m/s (default 1.0).
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s² to classify as a bump (default 0.5).
        settle_duration: Seconds to keep pushing after impact (default 0.2).
        max_duration: Safety timeout in seconds (default 5.0).
        grace_period: Seconds to ignore acceleration at start (default 0.3).

    Returns:
        A WallAlignBackwardBuilder (chainable via ``.speed()``, ``.accel_threshold()``, ``.settle_duration()``, ``.max_duration()``, ``.grace_period()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import wall_align_backward, drive_backward

        # Drive to the wall in reverse, then align against it
        seq([drive_backward(30), wall_align_backward()])
    """
    b = WallAlignBackwardBuilder()
    b._speed = speed
    b._accel_threshold = accel_threshold
    b._settle_duration = settle_duration
    b._max_duration = max_duration
    b._grace_period = grace_period
    return b


class WallAlignStrafeLeftBuilder(StepBuilder):
    """Builder for WallAlignStrafeLeft. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._speed = 0.5
        self._accel_threshold = 0.5
        self._settle_duration = 0.2
        self._max_duration = 5.0
        self._grace_period = 0.3

    def speed(self, value: float):
        self._speed = value
        return self

    def accel_threshold(self, value: float):
        self._accel_threshold = value
        return self

    def settle_duration(self, value: float):
        self._settle_duration = value
        return self

    def max_duration(self, value: float):
        self._max_duration = value
        return self

    def grace_period(self, value: float):
        self._grace_period = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["speed"] = self._speed
        kwargs["accel_threshold"] = self._accel_threshold
        kwargs["settle_duration"] = self._settle_duration
        kwargs["max_duration"] = self._max_duration
        kwargs["grace_period"] = self._grace_period
        return WallAlignStrafeLeft(**kwargs)


@dsl(tags=["motion", "wall"])
def wall_align_strafe_left(
    speed: float = 0.5,
    accel_threshold: float = 0.5,
    settle_duration: float = 0.2,
    max_duration: float = 5.0,
    grace_period: float = 0.3,
):
    """
    Strafe left into a wall and align the left side of the robot.

    Apply constant leftward velocity without heading correction so the robot
    naturally rotates flush against the wall surface.  Uses IMU bump
    detection.  Requires a mecanum or omni drivetrain capable of lateral
    movement.

    Args:
        speed: Strafe speed in m/s (default 0.5).
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s² to classify as a bump (default 0.5).
        settle_duration: Seconds to keep pushing after impact (default 0.2).
        max_duration: Safety timeout in seconds (default 5.0).
        grace_period: Seconds to ignore acceleration at start (default 0.3).

    Returns:
        A WallAlignStrafeLeftBuilder (chainable via ``.speed()``, ``.accel_threshold()``, ``.settle_duration()``, ``.max_duration()``, ``.grace_period()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import wall_align_strafe_left

        # Strafe-align the left side against a wall
        wall_align_strafe_left(speed=0.4)
    """
    b = WallAlignStrafeLeftBuilder()
    b._speed = speed
    b._accel_threshold = accel_threshold
    b._settle_duration = settle_duration
    b._max_duration = max_duration
    b._grace_period = grace_period
    return b


class WallAlignStrafeRightBuilder(StepBuilder):
    """Builder for WallAlignStrafeRight. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._speed = 0.5
        self._accel_threshold = 0.5
        self._settle_duration = 0.2
        self._max_duration = 5.0
        self._grace_period = 0.3

    def speed(self, value: float):
        self._speed = value
        return self

    def accel_threshold(self, value: float):
        self._accel_threshold = value
        return self

    def settle_duration(self, value: float):
        self._settle_duration = value
        return self

    def max_duration(self, value: float):
        self._max_duration = value
        return self

    def grace_period(self, value: float):
        self._grace_period = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["speed"] = self._speed
        kwargs["accel_threshold"] = self._accel_threshold
        kwargs["settle_duration"] = self._settle_duration
        kwargs["max_duration"] = self._max_duration
        kwargs["grace_period"] = self._grace_period
        return WallAlignStrafeRight(**kwargs)


@dsl(tags=["motion", "wall"])
def wall_align_strafe_right(
    speed: float = 0.5,
    accel_threshold: float = 0.5,
    settle_duration: float = 0.2,
    max_duration: float = 5.0,
    grace_period: float = 0.3,
):
    """
    Strafe right into a wall and align the right side of the robot.

    Apply constant rightward velocity without heading correction so the robot
    naturally rotates flush against the wall surface.  Uses IMU bump
    detection.  Requires a mecanum or omni drivetrain capable of lateral
    movement.

    Args:
        speed: Strafe speed in m/s (default 0.5).
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s² to classify as a bump (default 0.5).
        settle_duration: Seconds to keep pushing after impact (default 0.2).
        max_duration: Safety timeout in seconds (default 5.0).
        grace_period: Seconds to ignore acceleration at start (default 0.3).

    Returns:
        A WallAlignStrafeRightBuilder (chainable via ``.speed()``, ``.accel_threshold()``, ``.settle_duration()``, ``.max_duration()``, ``.grace_period()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import wall_align_strafe_right

        # Strafe-align the right side against a wall
        wall_align_strafe_right(speed=0.4)
    """
    b = WallAlignStrafeRightBuilder()
    b._speed = speed
    b._accel_threshold = accel_threshold
    b._settle_duration = settle_duration
    b._max_duration = max_duration
    b._grace_period = grace_period
    return b


__all__ = [
    "WallAlignForwardBuilder",
    "wall_align_forward",
    "WallAlignBackwardBuilder",
    "wall_align_backward",
    "WallAlignStrafeLeftBuilder",
    "wall_align_strafe_left",
    "WallAlignStrafeRightBuilder",
    "wall_align_strafe_right",
]
