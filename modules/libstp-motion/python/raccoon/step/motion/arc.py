import math
from raccoon.motion import ArcMotion, ArcMotionConfig
from typing import TYPE_CHECKING

from .. import SimulationStep, SimulationStepDelta, dsl
from ..annotation import dsl_step
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl(hidden=True)
class Arc(MotionStep):
    """Step wrapper around the native `ArcMotion` controller."""

    def __init__(self, config: ArcMotionConfig):
        super().__init__()
        self.config = config
        self._motion: ArcMotion | None = None

    def _generate_signature(self) -> str:
        return (
            f"Arc(radius_cm={self.config.radius_m * 100:.1f}, "
            f"arc_deg={math.degrees(self.config.arc_angle_rad):.1f}, "
            f"speed_scale={self.config.speed_scale:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        r = self.config.radius_m
        theta = self.config.arc_angle_rad
        if self.config.lateral:
            # Strafe arc: primary motion is lateral
            # strafe = R * sin(|theta|), forward drift from turning
            strafe_sign = 1.0 if theta >= 0 else -1.0
            base.delta = SimulationStepDelta(
                forward=r * (1 - math.cos(theta)),
                strafe=strafe_sign * r * math.sin(abs(theta)),
                angular=theta,
            )
        else:
            # Drive arc: primary motion is forward
            base.delta = SimulationStepDelta(
                forward=r * math.sin(abs(theta)),
                strafe=r * (1 - math.cos(theta)) * (-1 if theta > 0 else 1),
                angular=theta,
            )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        self._motion = ArcMotion(robot.drive, robot.odometry, robot.motion_pid_config, self.config)
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        self._motion.update(dt)
        return self._motion.is_finished()


@dsl_step(tags=["motion", "arc"])
class DriveArcLeft(Arc):
    """Drive along a circular arc curving to the left.

    The robot drives forward while simultaneously turning counter-clockwise,
    tracing a circular arc of the given radius. The motion completes when
    the robot has turned by the specified number of degrees.

    Args:
        radius_cm: Turning radius in centimeters (center of arc to robot center).
        degrees: Arc angle in degrees (how much the robot turns).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A DriveArcLeft step configured for a left (CCW) arc.

    Example::

        from raccoon.step.motion import drive_arc_left

        # Quarter-circle left with 30 cm radius
        drive_arc_left(radius_cm=30, degrees=90)

        # Gentle wide arc at half speed
        drive_arc_left(radius_cm=50, degrees=45, speed=0.5)
    """

    def __init__(self, radius_cm: float, degrees: float, speed: float = 1.0) -> None:
        self._radius_cm = radius_cm
        self._degrees = degrees
        self._speed = speed
        config = ArcMotionConfig()
        config.radius_m = radius_cm / 100.0
        config.arc_angle_rad = math.radians(degrees)  # Positive for CCW
        config.speed_scale = speed
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"DriveArcLeft(radius_cm={self._radius_cm:.1f}, "
            f"degrees={self._degrees:.1f}, speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "arc"])
class DriveArcRight(Arc):
    """Drive along a circular arc curving to the right.

    The robot drives forward while simultaneously turning clockwise,
    tracing a circular arc of the given radius. The motion completes when
    the robot has turned by the specified number of degrees.

    Args:
        radius_cm: Turning radius in centimeters (center of arc to robot center).
        degrees: Arc angle in degrees (how much the robot turns).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A DriveArcRight step configured for a right (CW) arc.

    Example::

        from raccoon.step.motion import drive_arc_right

        # Quarter-circle right with 30 cm radius
        drive_arc_right(radius_cm=30, degrees=90)
    """

    def __init__(self, radius_cm: float, degrees: float, speed: float = 1.0) -> None:
        self._radius_cm = radius_cm
        self._degrees = degrees
        self._speed = speed
        config = ArcMotionConfig()
        config.radius_m = radius_cm / 100.0
        config.arc_angle_rad = -math.radians(degrees)  # Negative for CW
        config.speed_scale = speed
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"DriveArcRight(radius_cm={self._radius_cm:.1f}, "
            f"degrees={self._degrees:.1f}, speed={self._speed:.2f})"
        )


@dsl(hidden=True)
class DriveArc(Arc):
    """Drive along a circular arc with explicit direction (internal use only).

    Positive degrees = counter-clockwise (left), negative = clockwise (right).
    """

    def __init__(self, radius_cm: float, degrees: float, speed: float = 1.0) -> None:
        self._radius_cm = radius_cm
        self._degrees = degrees
        self._speed = speed
        config = ArcMotionConfig()
        config.radius_m = radius_cm / 100.0
        config.arc_angle_rad = math.radians(degrees)
        config.speed_scale = speed
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"DriveArc(radius_cm={self._radius_cm:.1f}, "
            f"degrees={self._degrees:.1f}, speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "strafe", "arc"])
class StrafeArcLeft(Arc):
    """Strafe along a circular arc curving to the left.

    The robot strafes laterally (to the right) while simultaneously turning
    counter-clockwise, tracing a circular arc of the given radius. The motion
    completes when the robot has turned by the specified number of degrees.

    Internally uses a profiled PID on heading and derives the lateral velocity
    from the angular velocity command: ``vy = |omega| * radius``. This produces
    coordinated acceleration along the arc.

    Prerequisites:
        Requires a mecanum or omni-wheel drivetrain capable of lateral motion.

    Args:
        radius_cm: Turning radius in centimeters (center of arc to robot center).
        degrees: Arc angle in degrees (how much the robot turns).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A StrafeArcLeft step configured for a left (CCW) strafe arc.

    Example::

        from raccoon.step.motion import strafe_arc_left

        # Quarter-circle strafe arc to the left with 30 cm radius
        strafe_arc_left(radius_cm=30, degrees=90)

        # Gentle wide strafe arc at half speed
        strafe_arc_left(radius_cm=50, degrees=45, speed=0.5)
    """

    def __init__(self, radius_cm: float, degrees: float, speed: float = 1.0) -> None:
        self._radius_cm = radius_cm
        self._degrees = degrees
        self._speed = speed
        config = ArcMotionConfig()
        config.radius_m = radius_cm / 100.0
        config.arc_angle_rad = math.radians(degrees)  # Positive for CCW
        config.speed_scale = speed
        config.lateral = True
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"StrafeArcLeft(radius_cm={self._radius_cm:.1f}, "
            f"degrees={self._degrees:.1f}, speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "strafe", "arc"])
class StrafeArcRight(Arc):
    """Strafe along a circular arc curving to the right.

    The robot strafes laterally (to the left) while simultaneously turning
    clockwise, tracing a circular arc of the given radius. The motion
    completes when the robot has turned by the specified number of degrees.

    Internally uses a profiled PID on heading and derives the lateral velocity
    from the angular velocity command: ``vy = |omega| * radius``. This produces
    coordinated acceleration along the arc.

    Prerequisites:
        Requires a mecanum or omni-wheel drivetrain capable of lateral motion.

    Args:
        radius_cm: Turning radius in centimeters (center of arc to robot center).
        degrees: Arc angle in degrees (how much the robot turns).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A StrafeArcRight step configured for a right (CW) strafe arc.

    Example::

        from raccoon.step.motion import strafe_arc_right

        # Quarter-circle strafe arc to the right with 30 cm radius
        strafe_arc_right(radius_cm=30, degrees=90)
    """

    def __init__(self, radius_cm: float, degrees: float, speed: float = 1.0) -> None:
        self._radius_cm = radius_cm
        self._degrees = degrees
        self._speed = speed
        config = ArcMotionConfig()
        config.radius_m = radius_cm / 100.0
        config.arc_angle_rad = -math.radians(degrees)  # Negative for CW
        config.speed_scale = speed
        config.lateral = True
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"StrafeArcRight(radius_cm={self._radius_cm:.1f}, "
            f"degrees={self._degrees:.1f}, speed={self._speed:.2f})"
        )


@dsl(hidden=True)
class StrafeArc(Arc):
    """Strafe along a circular arc with explicit direction (internal use only).

    Positive degrees = counter-clockwise (left), negative = clockwise (right).
    """

    def __init__(self, radius_cm: float, degrees: float, speed: float = 1.0) -> None:
        self._radius_cm = radius_cm
        self._degrees = degrees
        self._speed = speed
        config = ArcMotionConfig()
        config.radius_m = radius_cm / 100.0
        config.arc_angle_rad = math.radians(degrees)
        config.speed_scale = speed
        config.lateral = True
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"StrafeArc(radius_cm={self._radius_cm:.1f}, "
            f"degrees={self._degrees:.1f}, speed={self._speed:.2f})"
        )
