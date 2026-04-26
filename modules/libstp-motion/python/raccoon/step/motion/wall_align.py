"""
Wall alignment step with IMU-based bump detection.

Drives toward a wall at constant velocity without heading correction.
Uses the IMU's gravity-compensated linear acceleration to detect the
moment of impact, then optionally continues pushing for a short settle
period so the robot can rotate flush against the wall surface.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from raccoon.foundation import ChassisVelocity
from raccoon.hal import IMU

from .. import dsl
from ..annotation import dsl_step
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


class WallDirection(Enum):
    """Direction to drive into a wall."""

    FORWARD = "forward"
    BACKWARD = "backward"
    STRAFE_LEFT = "strafe_left"
    STRAFE_RIGHT = "strafe_right"


# Expected deceleration angle (radians) for each drive direction.
# When driving forward, the wall pushes back at ~180 deg, etc.
_EXPECTED_DECEL_ANGLE = {
    WallDirection.FORWARD: math.pi,  # decel in -X
    WallDirection.BACKWARD: 0.0,  # decel in +X
    WallDirection.STRAFE_LEFT: math.pi / 2,  # decel in +Y
    WallDirection.STRAFE_RIGHT: -math.pi / 2,  # decel in -Y
}


def _normalize_angle(a: float) -> float:
    """Wrap angle to (-180, 180] degrees."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


@dataclass
class BumpResult:
    """Information about a detected wall impact."""

    accel_magnitude: float
    """XY acceleration magnitude at impact in m/s²."""
    impact_angle_deg: float
    """Wall misalignment angle in degrees.  0 = hit the wall square-on.
    Positive = wall surface angled CCW from perpendicular.
    Computed from the peak accel vector; may be noisy on single samples."""
    heading_correction_deg: float
    """How many degrees the robot actually rotated during the settle push
    to become flush with the wall.  This is the reliable metric — it
    comes from the IMU heading, not a single accel sample."""


@dsl(hidden=True)
class WallAlign(MotionStep):
    """
    Drive into a wall using IMU bump detection, then settle flush.

    The robot drives at constant velocity without heading correction.
    When the horizontal linear acceleration exceeds a threshold (indicating
    a collision), it records the impact and continues pushing for a settle
    period to let the chassis rotate flush against the wall.
    """

    def __init__(
        self,
        direction: WallDirection,
        speed: float,
        accel_threshold: float,
        settle_duration: float,
        max_duration: float,
        grace_period: float,
    ):
        super().__init__()
        if not isinstance(direction, WallDirection):
            msg = f"direction must be a WallDirection, got {type(direction).__name__}"
            raise TypeError(msg)
        if not isinstance(speed, int | float) or speed <= 0:
            msg = f"speed must be > 0, got {speed}"
            raise ValueError(msg)
        if not isinstance(accel_threshold, int | float) or accel_threshold <= 0:
            msg = f"accel_threshold must be > 0, got {accel_threshold}"
            raise ValueError(msg)
        if not isinstance(settle_duration, int | float) or settle_duration < 0:
            msg = f"settle_duration must be >= 0, got {settle_duration}"
            raise ValueError(msg)
        if not isinstance(max_duration, int | float) or max_duration <= 0:
            msg = f"max_duration must be > 0, got {max_duration}"
            raise ValueError(msg)
        if not isinstance(grace_period, int | float) or grace_period < 0:
            msg = f"grace_period must be >= 0, got {grace_period}"
            raise ValueError(msg)
        self.direction = direction
        self.speed = speed
        self.accel_threshold = accel_threshold
        self.settle_duration = settle_duration
        self.max_duration = max_duration
        self.grace_period = grace_period

        self._imu: IMU | None = None
        self._elapsed: float = 0.0
        self._bump_time: float | None = None
        self._heading_at_bump: float = 0.0
        self._peak_accel: float = 0.0
        self._peak_ax: float = 0.0
        self._peak_ay: float = 0.0
        self.bump_result: BumpResult | None = None

    def _generate_signature(self) -> str:
        return (
            f"WallAlign(direction={self.direction.value}, "
            f"speed={self.speed:.2f}, threshold={self.accel_threshold:.1f}, "
            f"settle={self.settle_duration:.2f})"
        )

    def _get_velocity(self) -> ChassisVelocity:
        if self.direction == WallDirection.FORWARD:
            return ChassisVelocity(self.speed, 0.0, 0.0)
        if self.direction == WallDirection.BACKWARD:
            return ChassisVelocity(-self.speed, 0.0, 0.0)
        if self.direction == WallDirection.STRAFE_LEFT:
            return ChassisVelocity(0.0, -self.speed, 0.0)
        # STRAFE_RIGHT
        return ChassisVelocity(0.0, self.speed, 0.0)

    def on_start(self, robot: "GenericRobot") -> None:
        self._imu = IMU()
        robot.drive.set_velocity(self._get_velocity())
        self._elapsed = 0.0
        self._bump_time = None
        self._peak_accel = 0.0
        self.bump_result = None

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        robot.drive.update(dt)
        self._elapsed += dt

        # Safety timeout
        if self._elapsed >= self.max_duration:
            self.debug("Wall align timed out without detecting a bump")
            return True

        ax, ay, _az = self._imu.get_linear_acceleration()
        accel_mag = math.hypot(ax, ay)

        if self._bump_time is None:
            # Skip the grace period while the robot accelerates from rest
            if self._elapsed < self.grace_period:
                return False

            if accel_mag >= self.accel_threshold:
                self._bump_time = asyncio.get_event_loop().time()
                self._heading_at_bump = robot.odometry.get_heading()
                self._peak_accel = accel_mag
                self._peak_ax = ax
                self._peak_ay = ay
                self.debug(f"Bump detected: {accel_mag:.2f} m/s²")
        else:
            # Track the peak accel sample during the settle window
            # — the true impact peak often arrives 1-2 samples after
            # the threshold crossing
            if accel_mag > self._peak_accel:
                self._peak_accel = accel_mag
                self._peak_ax = ax
                self._peak_ay = ay

            # Post-bump: keep pushing to align flush against the wall
            if asyncio.get_event_loop().time() - self._bump_time >= self.settle_duration:
                heading_now = robot.odometry.get_heading()
                heading_correction = math.degrees(heading_now - self._heading_at_bump)

                # Wall misalignment = raw accel angle minus expected decel direction
                raw_angle = math.degrees(math.atan2(self._peak_ay, self._peak_ax))
                expected = math.degrees(_EXPECTED_DECEL_ANGLE[self.direction])
                impact_angle = _normalize_angle(raw_angle - expected)

                self.bump_result = BumpResult(
                    accel_magnitude=self._peak_accel,
                    impact_angle_deg=impact_angle,
                    heading_correction_deg=heading_correction,
                )
                self.debug(
                    f"Wall align done: peak={self._peak_accel:.2f} m/s², "
                    f"wall_angle={impact_angle:.1f} deg, "
                    f"heading_correction={heading_correction:.1f} deg"
                )
                return True

        return False


# ---------------------------------------------------------------------------
# Public dsl_step subclasses
# ---------------------------------------------------------------------------


@dsl_step(tags=["motion", "wall"])
class WallAlignForward(WallAlign):
    """Drive forward into a wall and align the front of the robot.

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
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s²
            to classify as a bump (default 0.5).  Lower values are more
            sensitive but may false-trigger on rough surfaces.
        settle_duration: Seconds to keep pushing after the bump is detected,
            letting the chassis rotate flush (default 0.2).
        max_duration: Safety timeout in seconds — the step finishes even if
            no bump is detected (default 5.0).
        grace_period: Seconds to ignore acceleration after starting, so the
            robot's own acceleration doesn't trigger detection (default 0.3).

    Returns:
        A WallAlignForward step driving forward with bump detection.

    Example::

        from raccoon.step.motion import wall_align_forward, drive_forward

        # Drive near the wall, then bump-align against it
        seq([drive_forward(30), wall_align_forward()])

        # More sensitive detection at slower speed
        wall_align_forward(speed=0.3, accel_threshold=0.3)
    """

    def __init__(
        self,
        speed: float = 1.0,
        accel_threshold: float = 0.5,
        settle_duration: float = 0.2,
        max_duration: float = 5.0,
        grace_period: float = 0.3,
    ):
        super().__init__(
            direction=WallDirection.FORWARD,
            speed=abs(speed),
            accel_threshold=accel_threshold,
            settle_duration=settle_duration,
            max_duration=max_duration,
            grace_period=grace_period,
        )

    def _generate_signature(self) -> str:
        return (
            f"WallAlignForward(speed={self.speed:.2f}, "
            f"threshold={self.accel_threshold:.1f}, "
            f"settle={self.settle_duration:.2f})"
        )


@dsl_step(tags=["motion", "wall"])
class WallAlignBackward(WallAlign):
    """Drive backward into a wall and align the back of the robot.

    Apply constant backward velocity without heading correction so the robot
    naturally rotates flush against the wall surface.  Uses IMU bump
    detection to know when the wall has been reached.

    Args:
        speed: Drive speed in m/s (default 1.0).
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s²
            to classify as a bump (default 0.5).
        settle_duration: Seconds to keep pushing after impact (default 0.2).
        max_duration: Safety timeout in seconds (default 5.0).
        grace_period: Seconds to ignore acceleration at start (default 0.3).

    Returns:
        A WallAlignBackward step driving backward with bump detection.

    Example::

        from raccoon.step.motion import wall_align_backward, drive_backward

        # Drive to the wall in reverse, then align against it
        seq([drive_backward(30), wall_align_backward()])
    """

    def __init__(
        self,
        speed: float = 1.0,
        accel_threshold: float = 0.5,
        settle_duration: float = 0.2,
        max_duration: float = 5.0,
        grace_period: float = 0.3,
    ):
        super().__init__(
            direction=WallDirection.BACKWARD,
            speed=abs(speed),
            accel_threshold=accel_threshold,
            settle_duration=settle_duration,
            max_duration=max_duration,
            grace_period=grace_period,
        )

    def _generate_signature(self) -> str:
        return (
            f"WallAlignBackward(speed={self.speed:.2f}, "
            f"threshold={self.accel_threshold:.1f}, "
            f"settle={self.settle_duration:.2f})"
        )


@dsl_step(tags=["motion", "wall"])
class WallAlignStrafeLeft(WallAlign):
    """Strafe left into a wall and align the left side of the robot.

    Apply constant leftward velocity without heading correction so the robot
    naturally rotates flush against the wall surface.  Uses IMU bump
    detection.  Requires a mecanum or omni drivetrain capable of lateral
    movement.

    Args:
        speed: Strafe speed in m/s (default 0.5).
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s²
            to classify as a bump (default 0.5).
        settle_duration: Seconds to keep pushing after impact (default 0.2).
        max_duration: Safety timeout in seconds (default 5.0).
        grace_period: Seconds to ignore acceleration at start (default 0.3).

    Returns:
        A WallAlignStrafeLeft step strafing left with bump detection.

    Example::

        from raccoon.step.motion import wall_align_strafe_left

        # Strafe-align the left side against a wall
        wall_align_strafe_left(speed=0.4)
    """

    def __init__(
        self,
        speed: float = 0.5,
        accel_threshold: float = 0.5,
        settle_duration: float = 0.2,
        max_duration: float = 5.0,
        grace_period: float = 0.3,
    ):
        super().__init__(
            direction=WallDirection.STRAFE_LEFT,
            speed=abs(speed),
            accel_threshold=accel_threshold,
            settle_duration=settle_duration,
            max_duration=max_duration,
            grace_period=grace_period,
        )

    def _generate_signature(self) -> str:
        return (
            f"WallAlignStrafeLeft(speed={self.speed:.2f}, "
            f"threshold={self.accel_threshold:.1f}, "
            f"settle={self.settle_duration:.2f})"
        )


@dsl_step(tags=["motion", "wall"])
class WallAlignStrafeRight(WallAlign):
    """Strafe right into a wall and align the right side of the robot.

    Apply constant rightward velocity without heading correction so the robot
    naturally rotates flush against the wall surface.  Uses IMU bump
    detection.  Requires a mecanum or omni drivetrain capable of lateral
    movement.

    Args:
        speed: Strafe speed in m/s (default 0.5).
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s²
            to classify as a bump (default 0.5).
        settle_duration: Seconds to keep pushing after impact (default 0.2).
        max_duration: Safety timeout in seconds (default 5.0).
        grace_period: Seconds to ignore acceleration at start (default 0.3).

    Returns:
        A WallAlignStrafeRight step strafing right with bump detection.

    Example::

        from raccoon.step.motion import wall_align_strafe_right

        # Strafe-align the right side against a wall
        wall_align_strafe_right(speed=0.4)
    """

    def __init__(
        self,
        speed: float = 0.5,
        accel_threshold: float = 0.5,
        settle_duration: float = 0.2,
        max_duration: float = 5.0,
        grace_period: float = 0.3,
    ):
        super().__init__(
            direction=WallDirection.STRAFE_RIGHT,
            speed=abs(speed),
            accel_threshold=accel_threshold,
            settle_duration=settle_duration,
            max_duration=max_duration,
            grace_period=grace_period,
        )

    def _generate_signature(self) -> str:
        return (
            f"WallAlignStrafeRight(speed={self.speed:.2f}, "
            f"threshold={self.accel_threshold:.1f}, "
            f"settle={self.settle_duration:.2f})"
        )
