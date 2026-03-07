import math
from typing import TYPE_CHECKING

from .service import RobotService

if TYPE_CHECKING:
    from .api import GenericRobot


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class HeadingReferenceService(RobotService):
    """Stores an absolute IMU heading reference and computes turns relative to it.

    Use ``robot.get_service(HeadingReferenceService)`` to access.
    """

    def __init__(self, robot: "GenericRobot") -> None:
        super().__init__(robot)
        self._reference_rad: float | None = None

    def mark(self) -> None:
        """Capture the current absolute IMU heading as the reference."""
        self._reference_rad = self.robot.odometry.get_absolute_heading()
        self.info(
            f"Heading reference set to {math.degrees(self._reference_rad):.1f} deg (absolute)"
        )

    @property
    def reference_deg(self) -> float | None:
        """The stored reference in degrees, or None if not set."""
        if self._reference_rad is None:
            return None
        return math.degrees(self._reference_rad)

    def compute_turn(self, target_deg: float) -> float:
        """Compute the signed relative turn angle to reach *target_deg* from reference.

        Args:
            target_deg: Desired heading in degrees relative to the reference.

        Returns:
            Signed angle in degrees (positive = CCW / left, negative = CW / right).
            Normalized to [-180, 180] for the shortest path.

        Raises:
            RuntimeError: If no reference has been marked yet.
        """
        if self._reference_rad is None:
            raise RuntimeError(
                "No heading reference set. Call mark_heading_reference() first."
            )

        target_absolute = self._reference_rad + math.radians(target_deg)
        current_absolute = self.robot.odometry.get_absolute_heading()
        relative_rad = _normalize_angle(target_absolute - current_absolute)
        relative_deg = math.degrees(relative_rad)

        self.debug(
            f"ref={math.degrees(self._reference_rad):.1f}° "
            f"target_abs={math.degrees(target_absolute):.1f}° "
            f"current_abs={math.degrees(current_absolute):.1f}° "
            f"→ relative={relative_deg:.1f}°"
        )

        return relative_deg
