import math

from raccoon.motion import LinearMotion, LinearMotionConfig, LinearAxis
from typing import TYPE_CHECKING, Optional

from ..annotation import dsl_step
from ..condition import StopCondition
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

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
                 until: StopCondition = None,
                 heading: float = None):
        super().__init__()
        if cm is None and until is None:
            raise ValueError(
                f"{self.__class__.__name__} requires either 'cm' or 'until'"
            )
        if cm is not None:
            if not isinstance(cm, (int, float)):
                raise TypeError(f"cm must be a number, got {type(cm).__name__}")
            if cm <= 0:
                raise ValueError(f"cm must be > 0, got {cm}")
        if not isinstance(speed, (int, float)):
            raise TypeError(f"speed must be a number, got {type(speed).__name__}")
        if not (0.0 < speed <= 1.0):
            raise ValueError(f"speed must be in (0.0, 1.0], got {speed}")
        if until is not None and not isinstance(until, StopCondition):
            raise TypeError(
                f"until must be a StopCondition, got {type(until).__name__}"
            )
        if heading is not None and not isinstance(heading, (int, float)):
            raise TypeError(f"heading must be a number, got {type(heading).__name__}")
        self._cm = cm
        self._speed = speed
        self._until = until
        self._heading_deg = heading
        self._skip_calibration = False
        self._motion: Optional[LinearMotion] = None

    def _generate_signature(self) -> str:
        mode = f"{self._cm:.1f}cm" if self._cm else "until"
        heading = f", heading={self._heading_deg:.1f}°" if self._heading_deg is not None else ""
        return (
            f"{self.__class__.__name__}(mode={mode}, "
            f"speed={self._speed:.2f}{heading})"
        )

    def on_start(self, robot: "GenericRobot") -> None:
        if self._cm is not None and not self._skip_calibration:
            from raccoon.step.calibration import check_distance_calibration
            check_distance_calibration()

        config = LinearMotionConfig()
        config.axis = self._axis
        config.distance_m = (
            self._sign * self._cm / 100.0
            if self._cm is not None
            else self._sign * self._SENTINEL_DISTANCE_M
        )
        config.speed_scale = self._speed

        if self._heading_deg is not None:
            from raccoon.robot.heading_reference import HeadingReferenceService
            ref_svc = robot.get_service(HeadingReferenceService)
            sign = 1.0 if ref_svc._positive_direction == "left" else -1.0
            config.target_heading_rad = (
                ref_svc._reference_rad + sign * math.radians(self._heading_deg)
            )

        self._motion = LinearMotion(
            robot.drive, robot.odometry,
            robot.motion_pid_config, config,
        )
        self._motion.start()

        if self._until is not None:
            self._until.start(robot)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        if self._until is not None and self._until.check(robot):
            return True
        self._motion.update(dt)
        return self._motion.is_finished()


@dsl_step(tags=["motion", "drive"])
class DriveForward(_ConditionalDrive):
    """Drive forward with distance or condition-based termination.

    Uses profiled PID motion control with a trapezoidal velocity profile.
    The robot accelerates, cruises, and decelerates while maintaining
    heading via IMU feedback. When ``heading`` is given, the controller
    holds that absolute heading (degrees from heading reference) instead
    of the heading at start, preventing drift accumulation across
    consecutive drives.

    Requires ``calibrate_distance()`` for distance-based mode.
    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        heading: Absolute heading in degrees from the heading reference
            to hold during this drive. ``None`` (default) holds the
            heading at the start of the drive (relative mode).

    Example::

        drive_forward(25)                            # 25 cm, relative heading
        drive_forward(25, heading=90)                # hold 90° absolute
        drive_forward(speed=0.8).until(on_black(s))  # until sensor
    """
    _axis = LinearAxis.Forward
    _sign = 1.0

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None,
                 heading: float = None):
        super().__init__(cm=cm, speed=speed, until=until, heading=heading)


@dsl_step(tags=["motion", "drive"])
class DriveBackward(_ConditionalDrive):
    """Drive backward with distance or condition-based termination.

    Identical to ``drive_forward()`` but in reverse. Uses profiled PID
    motion control while maintaining heading via IMU feedback. Supports
    the same ``heading`` parameter for absolute heading hold.

    Requires ``calibrate_distance()`` for distance-based mode.
    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        heading: Absolute heading in degrees from the heading reference
            to hold during this drive. ``None`` (default) holds the
            heading at the start of the drive (relative mode).

    Example::

        drive_backward(20)
        drive_backward(20, heading=0)                # hold 0° absolute
        drive_backward(speed=0.5).until(on_white(s))
    """
    _axis = LinearAxis.Forward
    _sign = -1.0

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None,
                 heading: float = None):
        super().__init__(cm=cm, speed=speed, until=until, heading=heading)


@dsl_step(tags=["motion", "strafe"])
class StrafeLeft(_ConditionalDrive):
    """Strafe left with distance or condition-based termination.

    Requires a mecanum or omni-wheel drivetrain. The robot moves
    laterally to the left while maintaining heading via IMU feedback.
    Supports the same ``heading`` parameter for absolute heading hold.

    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        heading: Absolute heading in degrees from the heading reference
            to hold during this strafe. ``None`` (default) holds the
            heading at the start of the strafe (relative mode).

    Example::

        strafe_left(15)
        strafe_left(15, heading=90)                  # hold 90° absolute
        strafe_left(speed=0.6).until(on_black(s))
    """
    _axis = LinearAxis.Lateral
    _sign = -1.0

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None,
                 heading: float = None):
        super().__init__(cm=cm, speed=speed, until=until, heading=heading)


@dsl_step(tags=["motion", "strafe"])
class StrafeRight(_ConditionalDrive):
    """Strafe right with distance or condition-based termination.

    Requires a mecanum or omni-wheel drivetrain. The robot moves
    laterally to the right while maintaining heading via IMU feedback.
    Supports the same ``heading`` parameter for absolute heading hold.

    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        heading: Absolute heading in degrees from the heading reference
            to hold during this strafe. ``None`` (default) holds the
            heading at the start of the strafe (relative mode).

    Example::

        strafe_right(15)
        strafe_right(15, heading=0)                  # hold 0° absolute
        strafe_right(speed=0.6).until(on_black(s))
    """
    _axis = LinearAxis.Lateral
    _sign = 1.0

    def __init__(self, cm: float = None, speed: float = 1.0,
                 until: StopCondition = None,
                 heading: float = None):
        super().__init__(cm=cm, speed=speed, until=until, heading=heading)


def _drive_forward_uncalibrated(cm: float, speed: float = 1.0) -> DriveForward:
    """Internal: drive forward without calibration check.

    Used by ``calibrate_distance()`` to perform the calibration drive
    before calibration data exists.
    """
    step = DriveForward(cm=cm, speed=speed)
    step._skip_calibration = True
    return step
