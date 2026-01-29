import asyncio
from libstp.motion import DriveStraightMotion, DriveStraightConfig, UnifiedMotionPidConfig
from libstp.robot.api import GenericRobot

from .. import Step, SimulationStep, SimulationStepDelta

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

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=self.config.distance_m,
            strafe=0.0,
            angular=0.0,
        )
        return base

    async def _execute_step(self, robot: GenericRobot) -> None:
        """
        Run the Drive step.

        :param robot: The robot instance to interact with hardware
        """
        motion = DriveStraightMotion(robot.drive, robot.odometry, UnifiedMotionPidConfig(), self.config)
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


def _drive_forward_uncalibrated(cm: float, speed: float = 1.0) -> Drive:
    """
    Internal: Drive forward without calibration check.

    Used by calibrate_distance() to perform the calibration drive.
    Do not use directly - use drive_forward() instead.
    """
    config = DriveStraightConfig()
    config.distance_m = cm / 100.0
    config.max_speed_mps = speed
    return Drive(config)


def drive_forward(cm: float, speed: float = 1.0) -> Drive:
    """
    Drive forward a specified distance.

    Requires distance calibration to be performed first via calibrate_distance().
    Raises CalibrationRequiredError if not calibrated.

    Note: Calibration is now applied via per-wheel ticks_to_rad adjustment,
    so no runtime scaling is needed here. The calibration affects odometry
    directly through the motor's ticks_to_rad value.

    Args:
        cm: Distance to drive in centimeters
        speed: Speed (0-1, default 1.0)

    Returns:
        Drive step with calibrated distance

    Raises:
        CalibrationRequiredError: If calibrate_distance() has not been run
    """
    from libstp.step.calibration import check_distance_calibration
    check_distance_calibration()
    config = DriveStraightConfig()
    config.distance_m = cm / 100.0
    config.max_speed_mps = speed
    return Drive(config)


def drive_backward(cm: float, speed: float = 1.0) -> Drive:
    """
    Drive backward a specified distance.

    Requires distance calibration to be performed first via calibrate_distance().
    Raises CalibrationRequiredError if not calibrated.

    Note: Calibration is now applied via per-wheel ticks_to_rad adjustment,
    so no runtime scaling is needed here. The calibration affects odometry
    directly through the motor's ticks_to_rad value.

    Args:
        cm: Distance to drive in centimeters
        speed: Speed (0-1, default 1.0)

    Returns:
        Drive step with calibrated distance

    Raises:
        CalibrationRequiredError: If calibrate_distance() has not been run
    """
    from libstp.step.calibration import check_distance_calibration
    check_distance_calibration()
    config = DriveStraightConfig()
    config.distance_m = -cm / 100.0  # Negative distance for backwards
    config.max_speed_mps = speed
    return Drive(config)