import math

from libstp.motion import DiagonalMotion, DiagonalMotionConfig
from typing import TYPE_CHECKING

from .. import SimulationStep, SimulationStepDelta, dsl
from .motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class DriveAngle(MotionStep):
    def __init__(self, config: DiagonalMotionConfig):
        super().__init__()
        self.config = config
        self._motion: DiagonalMotion | None = None

    def _generate_signature(self) -> str:
        angle_deg = math.degrees(self.config.angle_rad)
        return (
            f"DriveAngle(angle={angle_deg:.1f}°, distance_m={self.config.distance_m:.3f}, "
            f"speed={self.config.max_speed_mps:.3f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=self.config.distance_m * math.cos(self.config.angle_rad),
            strafe=self.config.distance_m * math.sin(self.config.angle_rad),
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        self._motion = DiagonalMotion(robot.drive, robot.odometry, robot.motion_pid_config, self.config)
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        self._motion.update(dt)
        return self._motion.is_finished()


@dsl(tags=["motion", "drive"])
def drive_angle(angle_deg: float, cm: float, speed: float = 0.3) -> DriveAngle:
    """
    Drive at an arbitrary angle for a specified distance.

    Uses profiled PID with heading maintenance and cross-track correction
    in a rotated coordinate frame. Mecanum-only.

    Angle convention: 0 = forward, 90 = right, -90 = left, 180 = backward.

    Args:
        angle_deg: Travel angle in degrees (0=forward, 90=right, -90=left)
        cm: Distance to travel in centimeters
        speed: Maximum speed in m/s (default 0.3)

    Returns:
        DriveAngle step
    """
    config = DiagonalMotionConfig()
    config.angle_rad = math.radians(angle_deg)
    config.distance_m = cm / 100.0
    config.max_speed_mps = speed
    return DriveAngle(config)
