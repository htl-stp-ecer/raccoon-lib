import math

from libstp.motion import DiagonalMotion, DiagonalMotionConfig
from typing import TYPE_CHECKING

from .. import SimulationStep, SimulationStepDelta
from ..annotation import dsl_step
from .motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl_step(tags=["motion", "drive"])
class DriveAngle(MotionStep):
    """Drive at an arbitrary angle for a specified distance.

    Decomposes the desired heading into forward and lateral velocity
    components, then runs a profiled PID controller in a rotated
    coordinate frame with heading maintenance and cross-track correction.

    Requires a mecanum or omni-wheel drivetrain.

    Angle convention (robot-centric): ``0`` = forward, ``90`` = right,
    ``-90`` = left, ``180`` = backward.

    Args:
        angle_deg: Travel angle in degrees.
        cm: Distance to travel in centimeters.
        speed: Fraction of max speed, 0.0 to 1.0.

    Example::

        from libstp.step.motion import drive_angle

        # Drive diagonally forward-right at 45 degrees
        drive_angle(45, cm=30)

        # Drive pure right (same as strafe_right)
        drive_angle(90, cm=20)
    """

    def __init__(self, angle_deg: float, cm: float, speed: float = 1.0) -> None:
        super().__init__()
        self._angle_deg = angle_deg
        self._cm = cm
        self._speed = speed
        self._motion: DiagonalMotion | None = None

    def _generate_signature(self) -> str:
        return (
            f"DriveAngle(angle={self._angle_deg:.1f}\u00b0, distance_cm={self._cm:.1f}, "
            f"speed={self._speed:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        angle_rad = math.radians(self._angle_deg)
        distance_m = self._cm / 100.0
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=distance_m * math.cos(angle_rad),
            strafe=distance_m * math.sin(angle_rad),
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        config = DiagonalMotionConfig()
        config.angle_rad = math.radians(self._angle_deg)
        config.distance_m = self._cm / 100.0
        config.speed_scale = self._speed
        self._motion = DiagonalMotion(robot.drive, robot.odometry, robot.motion_pid_config, config)
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        self._motion.update(dt)
        return self._motion.is_finished()
