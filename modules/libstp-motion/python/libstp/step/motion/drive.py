from libstp.motion import LinearMotion, LinearMotionConfig, LinearAxis
from typing import TYPE_CHECKING

from .. import SimulationStep, SimulationStepDelta, dsl
from .motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class Drive(MotionStep):
    """Step wrapper around the native `LinearMotion` controller."""

    def __init__(self, config: LinearMotionConfig):
        super().__init__()
        self.config = config
        self._motion: LinearMotion | None = None

    def _generate_signature(self) -> str:
        axis = "Forward" if self.config.axis == LinearAxis.Forward else "Lateral"
        return (
            f"Drive(axis={axis}, distance_m={self.config.distance_m:.3f}, "
            f"speed_scale={self.config.speed_scale:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        if self.config.axis == LinearAxis.Forward:
            base.delta = SimulationStepDelta(
                forward=self.config.distance_m,
                strafe=0.0,
                angular=0.0,
            )
        else:
            base.delta = SimulationStepDelta(
                forward=0.0,
                strafe=self.config.distance_m,
                angular=0.0,
            )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        self._motion = LinearMotion(robot.drive, robot.odometry, robot.motion_pid_config, self.config)
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        self._motion.update(dt)
        return self._motion.is_finished()


# Keep Strafe as an alias for backward compatibility
Strafe = Drive


@dsl(hidden=True)
def _drive_forward_uncalibrated(cm: float, speed: float = 1.0) -> Drive:
    """
    Internal: Drive forward without calibration check.

    Used by calibrate_distance() to perform the calibration drive.
    Do not use directly - use drive_forward() instead.
    """
    config = LinearMotionConfig()
    config.axis = LinearAxis.Forward
    config.distance_m = cm / 100.0
    config.speed_scale = speed
    return Drive(config)


@dsl(tags=["motion", "drive"])
def drive_forward(cm: float, speed: float = 1.0) -> Drive:
    """
    Drive forward a specified distance.

    Requires distance calibration to be performed first via calibrate_distance().
    Raises CalibrationRequiredError if not calibrated.

    Args:
        cm: Distance to drive in centimeters
        speed: Fraction of max speed, 0-1 (default 1.0)

    Returns:
        Drive step with calibrated distance

    Raises:
        CalibrationRequiredError: If calibrate_distance() has not been run
    """
    from libstp.step.calibration import check_distance_calibration
    check_distance_calibration()
    config = LinearMotionConfig()
    config.axis = LinearAxis.Forward
    config.distance_m = cm / 100.0
    config.speed_scale = speed
    return Drive(config)


@dsl(tags=["motion", "drive"])
def drive_backward(cm: float, speed: float = 1.0) -> Drive:
    """
    Drive backward a specified distance.

    Requires distance calibration to be performed first via calibrate_distance().
    Raises CalibrationRequiredError if not calibrated.

    Args:
        cm: Distance to drive in centimeters
        speed: Fraction of max speed, 0-1 (default 1.0)

    Returns:
        Drive step with calibrated distance

    Raises:
        CalibrationRequiredError: If calibrate_distance() has not been run
    """
    from libstp.step.calibration import check_distance_calibration
    check_distance_calibration()
    config = LinearMotionConfig()
    config.axis = LinearAxis.Forward
    config.distance_m = -cm / 100.0
    config.speed_scale = speed
    return Drive(config)


@dsl(tags=["motion", "strafe"])
def strafe_left(cm: float, speed: float = 1.0) -> Drive:
    """
    Strafe left by specified distance at a given speed.

    Args:
        cm: The distance to strafe in centimeters (positive values move left)
        speed: Fraction of max speed, 0-1 (default 1.0)

    Returns:
        Drive step configured for leftward motion
    """
    config = LinearMotionConfig()
    config.axis = LinearAxis.Lateral
    config.distance_m = -cm / 100.0  # Negative for left (odometry convention: negative lateral = left)
    config.speed_scale = speed
    return Drive(config)


@dsl(tags=["motion", "strafe"])
def strafe_right(cm: float, speed: float = 1.0) -> Drive:
    """
    Strafe right by specified distance at a given speed.

    Args:
        cm: The distance to strafe in centimeters (positive values move right)
        speed: Fraction of max speed, 0-1 (default 1.0)

    Returns:
        Drive step configured for rightward motion
    """
    config = LinearMotionConfig()
    config.axis = LinearAxis.Lateral
    config.distance_m = cm / 100.0  # Positive for right (odometry convention: positive lateral = right)
    config.speed_scale = speed
    return Drive(config)
