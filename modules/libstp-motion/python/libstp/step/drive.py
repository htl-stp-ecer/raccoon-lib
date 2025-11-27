import asyncio
from libstp.motion import DriveStraightMotion, DriveStraightConfig
from libstp.robot.api import GenericRobot

from . import Step


class Drive(Step):
    def __init__(self,
                 config: DriveStraightConfig,
                 ):
        """
        Initialize the Drive step.

        Args:
            distance_cm: The distance to drive in centimeters.
            speed: The speed at which to drive in percent (0 - 1).
        """
        super().__init__()
        self.config = config

    def _generate_signature(self) -> str:
        return (
            f"Drive(distance_m={self.config.distance_m:.3f}, "
            f"speed={self.config.max_speed_mps:.3f})"
        )

    async def _execute_step(self, robot: GenericRobot) -> None:
        """
        Run the Drive step.

        :param robot: The robot instance to interact with hardware
        """
        motion = DriveStraightMotion(robot.drive, robot.odometry, self.config)
        motion.start()  # Explicitly start the motion to reset odometry

        update_rate = 1 / 10
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


def drive_forward(cm: float, speed: float = 1.0) -> Drive:
    """Drive forward for a specified duration at a given speed"""
    config = DriveStraightConfig()
    config.distance_m = cm / 100.0
    config.distance_tolerance_m = 0.01
    config.heading_kp = 5
    config.max_speed_mps = speed
    return Drive(config)


def drive_backward(cm: float, speed: float = 1.0) -> Drive:
    """Drive backward for a specified duration at a given speed"""
    config = DriveStraightConfig()
    config.distance_m = -cm / 100.0  # Negative distance for backwards
    config.distance_tolerance_m = 0.01
    config.heading_kp = 5
    config.max_speed_mps = speed  # Speed should be positive (magnitude only)
    return Drive(config)

#
# def strafe_left(seconds: float, speed: float, do_correction=True) -> Drive:
#     """Strafe left for a specified duration at a given speed"""
#     return Drive(for_seconds_condition(seconds), Speed(0, speed, 0), do_correction)
#
#
# def strafe_right(seconds: float, speed: float, do_correction=True) -> Drive:
#     """Strafe right for a specified duration at a given speed"""
#     return Drive(for_seconds_condition(seconds), Speed(0, -speed, 0), do_correction)
#
#
# def turn_cw(degrees: float, speed: float, do_correction=True) -> Drive:
#     """Turn clockwise by specified degrees at a given speed"""
#     return Drive(for_cw_condition(degrees), Speed(0, 0, -speed), do_correction)
#
#
# def turn_ccw(degrees: float, speed: float, do_correction=True) -> Drive:
#     """Turn counter-clockwise by specified degrees at a given speed"""
#     return Drive(for_ccw_condition(degrees), Speed(0, 0, speed), do_correction)
