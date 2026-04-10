import math

from raccoon.motion import DiagonalMotion, DiagonalMotionConfig
from typing import TYPE_CHECKING, Optional

from .. import SimulationStep, SimulationStepDelta
from ..annotation import dsl_step
from ..condition import StopCondition
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["motion", "drive"])
class DriveAngle(MotionStep):
    """Drive at an arbitrary angle with distance or condition-based termination.

    Decomposes the desired heading into forward and lateral velocity
    components, then runs a profiled PID controller in a rotated
    coordinate frame with heading maintenance and cross-track correction.

    Requires a mecanum or omni-wheel drivetrain.

    Angle convention (robot-centric): ``0`` = forward, ``90`` = right,
    ``-90`` = left, ``180`` = backward.

    Args:
        angle_deg: Travel angle in degrees.
        cm: Distance to travel in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.

    Example::

        from raccoon.step.motion import drive_angle

        # Drive diagonally forward-right at 45 degrees
        drive_angle(45, cm=30)

        # Drive pure right (same as strafe_right)
        drive_angle(90, cm=20)
    """

    _SENTINEL_DISTANCE_M = 100.0

    def __init__(self, angle_deg: float, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None) -> None:
        super().__init__()
        if cm is None and until is None:
            raise ValueError(
                "DriveAngle requires either 'cm' or 'until'"
            )
        self._angle_deg = angle_deg
        self._cm = cm
        self._speed = speed
        self._until = until
        self._motion: Optional[DiagonalMotion] = None

    def _generate_signature(self) -> str:
        mode = f"{self._cm:.1f}cm" if self._cm else "until"
        return (
            f"DriveAngle(angle={self._angle_deg:.1f}\u00b0, mode={mode}, "
            f"speed={self._speed:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        angle_rad = math.radians(self._angle_deg)
        distance_m = self._cm / 100.0 if self._cm is not None else 0.0
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=distance_m * math.cos(angle_rad),
            strafe=distance_m * math.sin(angle_rad),
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        config = DiagonalMotionConfig()
        config.angle_rad = math.radians(self._angle_deg)
        config.distance_m = (
            self._cm / 100.0
            if self._cm is not None
            else self._SENTINEL_DISTANCE_M
        )
        config.speed_scale = self._speed
        self._motion = DiagonalMotion(robot.drive, robot.odometry, robot.motion_pid_config, config)
        self._motion.start()

        if self._until is not None:
            self._until.start(robot)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        if self._until is not None and self._until.check(robot):
            return True
        self._motion.update(dt)
        return self._motion.is_finished()


@dsl_step(tags=["motion", "drive"])
class DriveAngleLeft(DriveAngle):
    """Drive at an angle to the left with distance or condition-based termination.

    Convenience wrapper around ``DriveAngle`` that negates the angle so
    that the ``angle_deg`` parameter is always positive (pointing left).

    The angle is measured as degrees to the left of forward:
    ``0`` = forward, ``45`` = forward-left diagonal, ``90`` = pure left.

    Args:
        angle_deg: Degrees to the left of forward (0 = forward, 90 = pure left).
        cm: Distance to travel in centimeters. Omit to use condition-only
            mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.

    Example::

        from raccoon.step.motion import drive_angle_left

        # Drive diagonally forward-left at 45 degrees
        drive_angle_left(45, cm=30)

        # Drive pure left until sensor
        drive_angle_left(90, speed=0.6).until(on_black(s))
    """

    def __init__(self, angle_deg: float, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None) -> None:
        super().__init__(angle_deg=-angle_deg, cm=cm, speed=speed, until=until)


@dsl_step(tags=["motion", "drive"])
class DriveAngleRight(DriveAngle):
    """Drive at an angle to the right with distance or condition-based termination.

    Convenience wrapper around ``DriveAngle`` that passes the angle
    directly (positive = right in the robot frame).

    The angle is measured as degrees to the right of forward:
    ``0`` = forward, ``45`` = forward-right diagonal, ``90`` = pure right.

    Args:
        angle_deg: Degrees to the right of forward (0 = forward, 90 = pure right).
        cm: Distance to travel in centimeters. Omit to use condition-only
            mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.

    Example::

        from raccoon.step.motion import drive_angle_right

        # Drive diagonally forward-right at 45 degrees
        drive_angle_right(45, cm=30)

        # Drive pure right until sensor
        drive_angle_right(90, speed=0.6).until(on_black(s))
    """

    def __init__(self, angle_deg: float, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None) -> None:
        super().__init__(angle_deg=angle_deg, cm=cm, speed=speed, until=until)
