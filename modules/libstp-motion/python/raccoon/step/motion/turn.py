from __future__ import annotations

import math
from typing import TYPE_CHECKING

from raccoon.foundation import ChassisVelocity
from raccoon.motion import TurnConfig, TurnMotion

from ..annotation import dsl_step
from ..condition import StopCondition
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


class _ConditionalTurn(MotionStep):
    """Base for directional turn steps with optional stop conditions.

    Supports two modes:
    - Angle-based: profiled TurnMotion (PID heading control)
    - Condition-based: constant angular velocity until condition triggers
    """

    _sign: float = 1.0  # +1 = CCW (left), -1 = CW (right)

    def __init__(
        self, degrees: float | None = None, speed: float = 1.0, until: StopCondition = None
    ):
        super().__init__()
        if degrees is None and until is None:
            msg = f"{self.__class__.__name__} requires either 'degrees' or 'until'"
            raise ValueError(msg)
        if degrees is not None:
            if not isinstance(degrees, int | float):
                msg = f"degrees must be a number, got {type(degrees).__name__}"
                raise TypeError(msg)
            if degrees <= 0:
                msg = f"degrees must be > 0, got {degrees}"
                raise ValueError(msg)
        if not isinstance(speed, int | float):
            msg = f"speed must be a number, got {type(speed).__name__}"
            raise TypeError(msg)
        if not (0.0 < speed <= 1.0):
            msg = f"speed must be in (0.0, 1.0], got {speed}"
            raise ValueError(msg)
        if until is not None and not isinstance(until, StopCondition):
            msg = f"until must be a StopCondition, got {type(until).__name__}"
            raise TypeError(msg)
        self._degrees = degrees
        self._speed = speed
        self._until = until
        self._motion: TurnMotion | None = None

    def _generate_signature(self) -> str:
        mode = f"{self._degrees:.1f}deg" if self._degrees else "until"
        return f"{self.__class__.__name__}(mode={mode}, speed={self._speed:.2f})"

    def on_start(self, robot: "GenericRobot") -> None:
        if self._degrees is not None:
            config = TurnConfig()
            config.target_angle_rad = self._sign * math.radians(self._degrees)
            config.speed_scale = self._speed
            self._motion = TurnMotion(
                robot.drive,
                robot.odometry,
                robot.motion_pid_config,
                config,
            )
            self._motion.start()
        else:
            max_w = robot.motion_pid_config.angular.max_velocity
            vel = ChassisVelocity(0, 0, self._sign * self._speed * max_w)
            robot.drive.set_velocity(vel)

        if self._until is not None:
            self._until.start(robot)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        if self._until is not None and self._until.check(robot):
            return True
        if self._motion:
            self._motion.update(dt)
            return self._motion.is_finished()
        robot.drive.update(dt)
        return False


@dsl_step(tags=["motion", "turn"])
class TurnLeft(_ConditionalTurn):
    """Turn left (counter-clockwise) with angle or condition-based termination.

    Uses a PID controller on the IMU heading to rotate the robot in place.
    The controller saturates output at the configured max angular rate,
    producing an implicit trapezoidal velocity profile.

    Example::

        turn_left(90)
        turn_left(speed=0.5).until(on_black(s))
    """

    _sign = 1.0

    def __init__(
        self, degrees: float | None = None, speed: float = 1.0, until: StopCondition = None
    ):
        super().__init__(degrees=degrees, speed=speed, until=until)


@dsl_step(tags=["motion", "turn"])
class TurnRight(_ConditionalTurn):
    """Turn right (clockwise) with angle or condition-based termination.

    Uses a PID controller on the IMU heading to rotate the robot in place.
    The controller saturates output at the configured max angular rate,
    producing an implicit trapezoidal velocity profile.

    Example::

        turn_right(90)
        turn_right(speed=0.5).until(on_black(s))
    """

    _sign = -1.0

    def __init__(
        self, degrees: float | None = None, speed: float = 1.0, until: StopCondition = None
    ):
        super().__init__(degrees=degrees, speed=speed, until=until)
