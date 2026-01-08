import asyncio
import math
from libstp.motion import TurnMotion, TurnConfig
from libstp.robot.api import GenericRobot

from . import Step, SimulationStep, SimulationStepDelta


class Turn(Step):
    def __init__(self, config: TurnConfig):
        """
        Initialize the Turn step.

        Args:
            config: The TurnConfig object specifying turn parameters
        """
        super().__init__()
        self.config = config

    def _generate_signature(self) -> str:
        return (
            f"Turn(angle_deg={math.degrees(self.config.target_angle_rad):.1f}, "
            f"speed={self.config.max_angular_rate:.3f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=0.0,
            strafe=0.0,
            angular=self.config.target_angle_rad,
        )
        return base

    async def _execute_step(self, robot: GenericRobot) -> None:
        """
        Run the Turn step.

        :param robot: The robot instance to interact with hardware
        """
        motion = TurnMotion(robot.drive, robot.odometry, self.config)
        motion.start()  # Explicitly start the motion to reset odometry

        update_rate = 1 / 10  # 10 Hz
        last_time = asyncio.get_event_loop().time() - update_rate  # seed so first dt ~= update_rate
        while not motion.is_finished():
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            # Avoid near-zero dt on the very first iteration (causes huge accel FF)
            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            motion.update(delta_time)
            await asyncio.sleep(update_rate)


def turn_ccw(degrees: float, speed: float = 1.0) -> Turn:
    """
    Turn counter-clockwise by specified degrees at a given speed.

    Args:
        degrees: The angle to turn in degrees (positive values turn CCW)
        speed: Maximum angular speed in rad/s (default 1.0 rad/s)

    Returns:
        Turn step configured for counter-clockwise rotation
    """
    config = TurnConfig()
    config.target_angle_rad = math.radians(degrees)  # Positive for CCW
    config.max_angular_rate = speed
    return Turn(config)


def turn_cw(degrees: float, speed: float = 1.0) -> Turn:
    """
    Turn clockwise by specified degrees at a given speed.

    Args:
        degrees: The angle to turn in degrees (positive values turn CW)
        speed: Maximum angular speed in rad/s (default 1.0 rad/s)

    Returns:
        Turn step configured for clockwise rotation
    """
    config = TurnConfig()
    config.target_angle_rad = -math.radians(degrees)  # Negative for CW
    config.max_angular_rate = speed
    return Turn(config)
