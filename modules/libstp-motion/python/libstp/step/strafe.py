import asyncio
from libstp.motion import StrafeMotion, StrafeConfig
from libstp.robot.api import GenericRobot

from . import Step


class Strafe(Step):
    def __init__(self, config: StrafeConfig):
        """
        Initialize the Strafe step.

        Args:
            config: The StrafeConfig object specifying strafe parameters
        """
        super().__init__()
        self.config = config

    async def run_step(self, robot: GenericRobot) -> None:
        """
        Run the Strafe step.

        :param robot: The robot instance to interact with hardware
        """
        await super().run_step(robot)
        motion = StrafeMotion(robot.drive, robot.odometry, self.config)
        motion.start()  # Explicitly start the motion to reset odometry

        update_rate = 1 / 10  # 10 Hz
        last_time = asyncio.get_event_loop().time()
        while not motion.is_finished():
            current_time = asyncio.get_event_loop().time()
            delta_time = current_time - last_time
            last_time = current_time
            motion.update(delta_time)
            await asyncio.sleep(update_rate)


def strafe_left(cm: float, speed: float = 0.3) -> Strafe:
    """
    Strafe left by specified distance at a given speed.

    Args:
        cm: The distance to strafe in centimeters (positive values move left)
        speed: Maximum lateral speed in m/s (default 0.3 m/s)

    Returns:
        Strafe step configured for leftward motion
    """
    config = StrafeConfig()
    config.target_distance_m = -cm / 100.0  # Negative for left (odometry convention: negative lateral = left)
    config.max_speed_mps = speed
    config.distance_tolerance_m = 0.02  # 2 cm tolerance
    config.distance_kp = 2.0
    config.heading_kp = 3.0
    config.min_speed_mps = 0.05
    return Strafe(config)


def strafe_right(cm: float, speed: float = 0.3) -> Strafe:
    """
    Strafe right by specified distance at a given speed.

    Args:
        cm: The distance to strafe in centimeters (positive values move right)
        speed: Maximum lateral speed in m/s (default 0.3 m/s)

    Returns:
        Strafe step configured for rightward motion
    """
    config = StrafeConfig()
    config.target_distance_m = cm / 100.0  # Positive for right (odometry convention: positive lateral = right)
    config.max_speed_mps = speed
    config.distance_tolerance_m = 0.02  # 2 cm tolerance
    config.distance_kp = 2.0
    config.heading_kp = 3.0
    config.min_speed_mps = 0.05
    return Strafe(config)
