import asyncio
import math
from libstp.motion import TurnMotion, TurnConfig
from libstp.robot.api import GenericRobot

from . import Step


class Turn(Step):
    def __init__(self, config: TurnConfig):
        """
        Initialize the Turn step.

        Args:
            config: The TurnConfig object specifying turn parameters
        """
        super().__init__()
        self.config = config

    async def run_step(self, robot: GenericRobot) -> None:
        """
        Run the Turn step.

        :param robot: The robot instance to interact with hardware
        """
        await super().run_step(robot)
        motion = TurnMotion(robot.drive, robot.odometry, self.config)
        motion.start()  # Explicitly start the motion to reset odometry

        update_rate = 1 / 10  # 10 Hz
        last_time = asyncio.get_event_loop().time()
        while not motion.is_finished():
            current_time = asyncio.get_event_loop().time()
            delta_time = current_time - last_time
            last_time = current_time
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
    config.angle_tolerance_rad = math.radians(1.0)  # 1 degree tolerance
    config.angle_kp = 3.0
    config.min_angular_rate = 0.1
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
    config.angle_tolerance_rad = math.radians(1.0)  # 1 degree tolerance
    config.angle_kp = 3.0
    config.min_angular_rate = 0.1
    return Turn(config)
