from __future__ import annotations

import math
from enum import Enum
from typing import TYPE_CHECKING

from .service import RobotService

if TYPE_CHECKING:
    from .api import GenericRobot


class TurnDirection(str, Enum):
    """Physical direction a heading turn may be forced to take.

    Subclasses ``str`` so existing call sites that pass the bare strings
    ``"left"`` / ``"right"`` keep working (``TurnDirection.LEFT == "left"``)
    while new code gets a real, self-validating type. Coerce any user input
    via :meth:`coerce`, which raises ``ValueError`` on an unknown value
    instead of silently falling back to the shortest path.
    """

    LEFT = "left"
    """Counter-clockwise (CCW) — positive angular direction."""

    RIGHT = "right"
    """Clockwise (CW) — negative angular direction."""

    @classmethod
    def coerce(cls, value: "TurnDirection | str | None") -> "TurnDirection | None":
        """Normalize ``value`` to a :class:`TurnDirection`, validating it.

        ``None`` passes through (meaning "shortest path"). A
        :class:`TurnDirection` is returned as-is. A string is matched
        case-insensitively against the members. Anything else raises
        ``ValueError`` so a typo (e.g. ``"lft"``) fails loudly rather than
        being silently ignored.
        """
        if value is None or isinstance(value, cls):
            return value
        if isinstance(value, str):
            try:
                return cls(value.strip().lower())
            except ValueError:
                pass
        valid = ", ".join(repr(member.value) for member in cls)
        msg = f"Invalid turn direction {value!r}; expected one of {valid} or None."
        raise ValueError(msg)


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _world_heading(robot: "GenericRobot") -> float:
    """Read the current heading, preferring odometry over localization.

    The motion controllers (``LinearMotion``/``TurnMotion``) close their heading
    loop on ``odometry().getHeading()``. To keep the reference, the turn-delta
    computation and the executed feedback in a single frame, all heading math
    here must use the same source — odometry. Localization is only a fallback
    when no odometry source is enabled.
    """
    odom = getattr(robot, "odometry", None)
    loc = getattr(robot, "localization", None)
    if odom is None and loc is None:
        msg = (
            "HeadingReferenceService requires robot.odometry or robot.localization "
            "(at least one heading source must be enabled)."
        )
        raise RuntimeError(msg)
    if odom is None:
        return float(loc.get_pose().heading)
    return float(odom.get_pose().heading)


class HeadingReferenceService(RobotService):
    """Stores an absolute IMU heading reference and computes turns relative to it.

    Use ``robot.get_service(HeadingReferenceService)`` to access.
    """

    def __init__(self, robot: "GenericRobot") -> None:
        super().__init__(robot)
        self._reference_rad: float | None = None
        self._positive_direction: str = "left"
        # Flat DMP orientation quaternion (w, x, y, z) captured at mark(), used
        # by on_incline / on_level as the "zero tilt" baseline. None until a
        # mark() runs; all-zero if the IMU has no quaternion to offer.
        self._tilt_reference_quat: tuple[float, float, float, float] | None = None

    def mark(self, origin_offset_deg: float = 0.0, positive_direction: str = "left") -> None:
        """Capture the current absolute world heading as the reference.

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
        raw = _world_heading(self.robot)
        self._reference_rad = raw + math.radians(origin_offset_deg)
        self._positive_direction = positive_direction
        self._capture_tilt_reference()
        self.info(
            f"Heading reference set to {math.degrees(self._reference_rad):.1f} deg "
            f"(absolute={math.degrees(raw):.1f}°, offset={origin_offset_deg:.1f}°, "
            f"positive={positive_direction})"
        )

    def _capture_tilt_reference(self) -> None:
        """Snapshot the current DMP orientation as the flat tilt baseline.

        Stored normalised so on_incline / on_level can measure tilt relative to
        it. Best-effort: if the IMU exposes no (or an all-zero) quaternion, the
        reference stays ``None`` and those conditions raise a clear error when
        used. Must be called on flat ground and after the DMP has converged.
        """
        try:
            from raccoon.hal import IMU

            w, x, y, z = IMU().get_quaternion()
        except Exception as exc:  # never let a mark() fail on this
            self._tilt_reference_quat = None
            self.debug(f"tilt reference not captured: {exc}")
            return
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm < 0.5:
            self._tilt_reference_quat = None
            self.debug("tilt reference not captured: quaternion unavailable (all-zero)")
            return
        self._tilt_reference_quat = (w / norm, x / norm, y / norm, z / norm)
        self.info(f"Tilt reference captured (quat w={w / norm:.3f})")

    def tilt_reference_quat(self) -> tuple[float, float, float, float] | None:
        """The flat DMP orientation quaternion from the last mark(), or None."""
        return self._tilt_reference_quat

    @property
    def reference_deg(self) -> float | None:
        """The stored reference in degrees, or None if not set."""
        if self._reference_rad is None:
            return None
        return math.degrees(self._reference_rad)

    def current_relative_deg(self) -> float:
        """The current world heading in degrees relative to the reference.

        Uses the same positive-direction convention as :meth:`compute_turn`
        (``"left"`` → CCW positive). Useful for logging "where we are now"
        before a heading turn.

        Raises:
            RuntimeError: If no reference has been marked yet.
        """
        if self._reference_rad is None:
            msg = "No heading reference set. Call mark_heading_reference() first."
            raise RuntimeError(msg)
        sign = 1.0 if self._positive_direction == "left" else -1.0
        current_absolute = _world_heading(self.robot)
        return math.degrees(sign * _normalize_angle(current_absolute - self._reference_rad))

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
            msg = "No heading reference set. Call mark_heading_reference() first."
            raise RuntimeError(msg)
        sign = 1.0 if self._positive_direction == "left" else -1.0
        return self._reference_rad + sign * math.radians(target_deg)

    def compute_turn(
        self,
        target_deg: float,
        force_direction: "TurnDirection | str | None" = None,
    ) -> float:
        """Compute the signed relative turn angle to reach *target_deg* from reference.

        Reads the current world heading via :func:`_world_heading` (odometry,
        the same source the motion controllers regulate on), so the computed
        turn delta and the executed feedback share one frame.

        Args:
            target_deg: Desired heading in degrees relative to the reference.
            force_direction: :attr:`TurnDirection.LEFT` to force CCW,
                :attr:`TurnDirection.RIGHT` to force CW, or ``None`` (default)
                for shortest path. Plain strings ``"left"`` / ``"right"`` are
                accepted and validated; an unknown value raises ``ValueError``.

        Returns:
            Signed angle in degrees (positive = CCW / left, negative = CW / right).
            Normalized to [-180, 180] for shortest path, or extended past
            ±180 (up to ±360) to honour the forced direction.

        Raises:
            RuntimeError: If no reference has been marked yet.
            ValueError: If ``force_direction`` is not a valid direction.
        """
        if self._reference_rad is None:
            msg = "No heading reference set. Call mark_heading_reference() first."
            raise RuntimeError(msg)

        direction = TurnDirection.coerce(force_direction)

        sign = 1.0 if self._positive_direction == "left" else -1.0
        target_absolute = self._reference_rad + sign * math.radians(target_deg)
        current_absolute = _world_heading(self.robot)
        relative_rad = _normalize_angle(target_absolute - current_absolute)
        relative_deg = math.degrees(relative_rad)

        if direction is TurnDirection.LEFT and relative_deg < 0:
            relative_deg += 360.0
        elif direction is TurnDirection.RIGHT and relative_deg > 0:
            relative_deg -= 360.0

        self.debug(
            f"ref={math.degrees(self._reference_rad):.1f}° "
            f"target_abs={math.degrees(target_absolute):.1f}° "
            f"current_abs={math.degrees(current_absolute):.1f}° "
            f"→ relative={relative_deg:.1f}°"
            f"{f' (forced {direction.value})' if direction else ''}"
        )

        return relative_deg
