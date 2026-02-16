import math
from libstp.motion import TurnMotion, TurnConfig
from typing import TYPE_CHECKING

from .. import SimulationStep, SimulationStepDelta, dsl
from .motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class Turn(MotionStep):
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
    Turn counter-clockwise by specified degrees at a given speed.

    Args:
        degrees: The angle to turn in degrees (positive values turn CCW)
        speed: Fraction of max angular speed, 0-1 (default 1.0)

    Returns:
        Turn step configured for counter-clockwise rotation
    """
    config = TurnConfig()
    config.target_angle_rad = math.radians(degrees)  # Positive for CCW
    config.speed_scale = speed
    return Turn(config)


@dsl(tags=["motion", "turn"])
def turn_right(degrees: float, speed: float = 1.0) -> Turn:
    """
    Turn clockwise by specified degrees at a given speed.

    Args:
        degrees: The angle to turn in degrees (positive values turn CW)
        speed: Fraction of max angular speed, 0-1 (default 1.0)

    Returns:
        Turn step configured for clockwise rotation
    """
    config = TurnConfig()
    config.target_angle_rad = -math.radians(degrees)  # Negative for CW
    config.speed_scale = speed
    return Turn(config)
