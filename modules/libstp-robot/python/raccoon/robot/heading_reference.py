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
        self._positive_direction: str = "left"

    def mark(self, origin_offset_deg: float = 0.0, positive_direction: str = "left") -> None:
        """Capture the current absolute IMU heading as the reference.

        Args:
            origin_offset_deg: Offset in degrees added to the captured heading.
                Use this to define a consistent origin regardless of the robot's
                physical starting rotation.  For example, if the robot is placed
                at 30° to the board edge but you want 0° to mean "along the
                board edge", pass ``origin_offset_deg=-30``.
            positive_direction: Which physical direction corresponds to positive
                angles.  ``"left"`` (default) means CCW is positive, matching the
                standard mathematical convention.  ``"right"`` flips the sign so
                CW is positive.
        """
        raw = self.robot.odometry.get_absolute_heading()
        self._reference_rad = raw + math.radians(origin_offset_deg)
        self._positive_direction = positive_direction
        self.info(
            f"Heading reference set to {math.degrees(self._reference_rad):.1f} deg "
            f"(absolute={math.degrees(raw):.1f}°, offset={origin_offset_deg:.1f}°, "
            f"positive={positive_direction})"
        )

    @property
    def reference_deg(self) -> float | None:
        """The stored reference in degrees, or None if not set."""
        if self._reference_rad is None:
            return None
        return math.degrees(self._reference_rad)

    def target_absolute_rad(self, target_deg: float) -> float:
        """Convert a relative target (degrees from reference) to absolute IMU radians.

        Used by motion controllers that want to *hold* an absolute heading
        rather than *turn* to it — they need the raw absolute target
        without [-180, 180] normalisation, since the chassis controller
        carries continuous heading state across consecutive commands.

        Raises:
            RuntimeError: If no reference has been marked yet.
        """
        if self._reference_rad is None:
            raise RuntimeError(
                "No heading reference set. Call mark_heading_reference() first."
            )
        sign = 1.0 if self._positive_direction == "left" else -1.0
        return self._reference_rad + sign * math.radians(target_deg)

    def compute_turn(
        self,
        target_deg: float,
        force_direction: str | None = None,
    ) -> float:
        """Compute the signed relative turn angle to reach *target_deg* from reference.

        Args:
            target_deg: Desired heading in degrees relative to the reference.
            force_direction: ``"left"`` to force CCW, ``"right"`` to force CW,
                or ``None`` (default) for shortest path.

        Returns:
            Signed angle in degrees (positive = CCW / left, negative = CW / right).
            Normalized to [-180, 180] for shortest path, or adjusted to the
            forced direction.

        Raises:
            RuntimeError: If no reference has been marked yet.
        """
        if self._reference_rad is None:
            raise RuntimeError(
                "No heading reference set. Call mark_heading_reference() first."
            )

        sign = 1.0 if self._positive_direction == "left" else -1.0
        target_absolute = self._reference_rad + sign * math.radians(target_deg)
        current_absolute = self.robot.odometry.get_absolute_heading()
        relative_rad = _normalize_angle(target_absolute - current_absolute)
        relative_deg = math.degrees(relative_rad)

        if force_direction == "left" and relative_deg < 0:
            relative_deg += 360.0
        elif force_direction == "right" and relative_deg > 0:
            relative_deg -= 360.0

        self.debug(
            f"ref={math.degrees(self._reference_rad):.1f}° "
            f"target_abs={math.degrees(target_absolute):.1f}° "
            f"current_abs={math.degrees(current_absolute):.1f}° "
            f"→ relative={relative_deg:.1f}°"
            f"{f' (forced {force_direction})' if force_direction else ''}"
        )

        return relative_deg
