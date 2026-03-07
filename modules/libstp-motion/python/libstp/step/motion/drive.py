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
    Drive forward a specified distance using profiled PID motion control.

    The robot accelerates, cruises, and decelerates along a trapezoidal
    velocity profile while maintaining heading via IMU feedback. Odometry
    tracks the distance traveled and the step completes when the target
    is reached.

    Requires ``calibrate_distance()`` to have been run first so that
    encoder-to-meter conversion is accurate.

    Args:
        cm: Distance to drive in centimeters.
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A Drive step configured for forward motion.

    Raises:
        CalibrationRequiredError: If ``calibrate_distance()`` has not been run.

    Example::

        from libstp.step.motion import drive_forward

        # Drive forward 50 cm at full speed
        drive_forward(50)

        # Drive forward 30 cm at half speed
        drive_forward(30, speed=0.5)
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
    Drive backward a specified distance using profiled PID motion control.

    Identical to ``drive_forward()`` but in reverse. The robot drives
    backward while maintaining heading via IMU feedback.

    Requires ``calibrate_distance()`` to have been run first.

    Args:
        cm: Distance to drive in centimeters.
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A Drive step configured for backward motion.

    Raises:
        CalibrationRequiredError: If ``calibrate_distance()`` has not been run.

    Example::

        from libstp.step.motion import drive_backward

        # Back up 20 cm
        drive_backward(20)
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
    Strafe left by a specified distance using profiled PID motion control.

    Requires a mecanum or omni-wheel drivetrain. The robot moves laterally
    to the left while maintaining heading via IMU feedback.

    Args:
        cm: Distance to strafe in centimeters.
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A Drive step configured for leftward lateral motion.

    Example::

        from libstp.step.motion import strafe_left

        # Strafe left 15 cm to dodge an obstacle
        strafe_left(15)
    """
    config = LinearMotionConfig()
    config.axis = LinearAxis.Lateral
    config.distance_m = -cm / 100.0  # Negative for left (odometry convention: negative lateral = left)
    config.speed_scale = speed
    return Drive(config)


@dsl(tags=["motion", "strafe"])
def strafe_right(cm: float, speed: float = 1.0) -> Drive:
    """
    Strafe right by a specified distance using profiled PID motion control.

    Requires a mecanum or omni-wheel drivetrain. The robot moves laterally
    to the right while maintaining heading via IMU feedback.

    Args:
        cm: Distance to strafe in centimeters.
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A Drive step configured for rightward lateral motion.

    Example::

        from libstp.step.motion import strafe_right

        # Strafe right 15 cm to align with a game piece
        strafe_right(15)
    """
    config = LinearMotionConfig()
    config.axis = LinearAxis.Lateral
    config.distance_m = cm / 100.0  # Positive for right (odometry convention: positive lateral = right)
    config.speed_scale = speed
    return Drive(config)
