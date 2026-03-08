from libstp.motion import LinearMotion, LinearMotionConfig, LinearAxis
from typing import TYPE_CHECKING, Optional

from ..annotation import dsl_step
from ..condition import StopCondition
from .motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot

class _ConditionalDrive(MotionStep):
    """Base for directional drive steps with optional stop conditions.

    Always uses profiled LinearMotion. When no distance is given,
    a large sentinel distance (100 m) is used so the motion provider
    handles velocity/heading control and the condition triggers the stop.
    """

    # Subclasses set these
    _axis: LinearAxis = LinearAxis.Forward
    _sign: float = 1.0  # +1 or -1 for direction

    _SENTINEL_DISTANCE_M = 100.0  # Large distance; condition stops early

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None):
        super().__init__()
        if cm is None and until is None:
            raise ValueError(
                f"{self.__class__.__name__} requires either 'cm' or 'until'"
            )
        self._cm = cm
        self._speed = speed
        self._until = until
        self._skip_calibration = False
        self._motion: Optional[LinearMotion] = None

    def _generate_signature(self) -> str:
        mode = f"{self._cm:.1f}cm" if self._cm else "until"
        return (
            f"{self.__class__.__name__}(mode={mode}, "
            f"speed={self._speed:.2f})"
        )

    def on_start(self, robot: "GenericRobot") -> None:
        if self._cm is not None and not self._skip_calibration:
            from libstp.step.calibration import check_distance_calibration
            check_distance_calibration()

        config = LinearMotionConfig()
        config.axis = self._axis
        config.distance_m = (
            self._sign * self._cm / 100.0
            if self._cm is not None
            else self._sign * self._SENTINEL_DISTANCE_M
        )
        config.speed_scale = self._speed
        self._motion = LinearMotion(
            robot.drive, robot.odometry,
            robot.motion_pid_config, config,
        )
        self._motion.start()

        if self._until:
            self._until.start(robot)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        if self._until and self._until.check(robot):
            return True
        self._motion.update(dt)
        return self._motion.is_finished()


@dsl_step(tags=["motion", "drive"])
class DriveForward(_ConditionalDrive):
    """Drive forward with distance or condition-based termination.

    Uses profiled PID motion control with a trapezoidal velocity profile.
    The robot accelerates, cruises, and decelerates while maintaining
    heading via IMU feedback. In condition-only mode the robot drives at
    constant speed until the condition triggers.

    Requires ``calibrate_distance()`` for distance-based mode.

    Example::

        drive_forward(25)                            # 25 cm
        drive_forward(speed=0.8).until(on_black(s))  # until sensor
    """
    _axis = LinearAxis.Forward
    _sign = 1.0

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None):
        super().__init__(cm=cm, speed=speed, until=until)


@dsl_step(tags=["motion", "drive"])
class DriveBackward(_ConditionalDrive):
    """Drive backward with distance or condition-based termination.

    Identical to ``drive_forward()`` but in reverse. Uses profiled PID
    motion control while maintaining heading via IMU feedback.

    Requires ``calibrate_distance()`` for distance-based mode.

    Example::

        drive_backward(20)
        drive_backward(speed=0.5).until(on_white(s))
    """
    _axis = LinearAxis.Forward
    _sign = -1.0

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None):
        super().__init__(cm=cm, speed=speed, until=until)


@dsl_step(tags=["motion", "strafe"])
class StrafeLeft(_ConditionalDrive):
    """Strafe left with distance or condition-based termination.

    Requires a mecanum or omni-wheel drivetrain. The robot moves
    laterally to the left while maintaining heading via IMU feedback.

    Example::

        strafe_left(15)
        strafe_left(speed=0.6).until(on_black(s))
    """
    _axis = LinearAxis.Lateral
    _sign = -1.0

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None):
        super().__init__(cm=cm, speed=speed, until=until)


@dsl_step(tags=["motion", "strafe"])
class StrafeRight(_ConditionalDrive):
    """Strafe right with distance or condition-based termination.

    Requires a mecanum or omni-wheel drivetrain. The robot moves
    laterally to the right while maintaining heading via IMU feedback.

    Example::

        strafe_right(15)
        strafe_right(speed=0.6).until(on_black(s))
    """
    _axis = LinearAxis.Lateral
    _sign = 1.0

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None):
        super().__init__(cm=cm, speed=speed, until=until)


def _drive_forward_uncalibrated(cm: float, speed: float = 1.0) -> DriveForward:
    """Internal: drive forward without calibration check.

    Used by ``calibrate_distance()`` to perform the calibration drive
    before calibration data exists.
    """
    step = DriveForward(cm=cm, speed=speed)
    step._skip_calibration = True
    return step
