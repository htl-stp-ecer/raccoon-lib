import math
from libstp.motion import TurnMotion, TurnConfig
from typing import TYPE_CHECKING

from .. import SimulationStep, SimulationStepDelta, dsl
from .motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class Turn(MotionStep):
    """Step wrapper around the native `TurnMotion` controller."""

    def __init__(self, config: TurnConfig):
        super().__init__()
        self.config = config
        self._motion: TurnMotion | None = None

    def _generate_signature(self) -> str:
        return (
            f"Turn(angle_deg={math.degrees(self.config.target_angle_rad):.1f}, "
            f"speed_scale={self.config.speed_scale:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=0.0,
            strafe=0.0,
            angular=self.config.target_angle_rad,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        self._motion = TurnMotion(robot.drive, robot.odometry, robot.motion_pid_config, self.config)
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        self._motion.update(dt)
        return self._motion.is_finished()


@dsl(tags=["motion", "turn"])
def turn_left(degrees: float, speed: float = 1.0) -> Turn:
    """
    Turn counter-clockwise (left) by a specified angle.

    Uses a PID controller on the IMU heading to rotate the robot in place.
    The controller saturates output at the configured max angular rate,
    producing an implicit trapezoidal velocity profile.

    Args:
        degrees: Angle to turn in degrees (positive = counter-clockwise).
        speed: Fraction of max angular speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A Turn step configured for counter-clockwise rotation.

    Example::

        from libstp.step.motion import turn_left

        # Turn 90 degrees to the left
        turn_left(90)

        # Gentle 45-degree turn at half speed
        turn_left(45, speed=0.5)
    """
    config = TurnConfig()
    config.target_angle_rad = math.radians(degrees)  # Positive for CCW
    config.speed_scale = speed
    return Turn(config)


@dsl(tags=["motion", "turn"])
def turn_right(degrees: float, speed: float = 1.0) -> Turn:
    """
    Turn clockwise (right) by a specified angle.

    Uses a PID controller on the IMU heading to rotate the robot in place.
    The controller saturates output at the configured max angular rate,
    producing an implicit trapezoidal velocity profile.

    Args:
        degrees: Angle to turn in degrees (positive = clockwise).
        speed: Fraction of max angular speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A Turn step configured for clockwise rotation.

    Example::

        from libstp.step.motion import turn_right

        # Turn 90 degrees to the right
        turn_right(90)
    """
    config = TurnConfig()
    config.target_angle_rad = -math.radians(degrees)  # Negative for CW
    config.speed_scale = speed
    return Turn(config)
