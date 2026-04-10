"""Map-corrected odometry using IR sensor readings and known field geometry.

Wraps an existing odometry source and applies lateral corrections when IR
sensors detect line crossings.  The IMU heading is trusted; only the
translational (x, y) position is nudged.

Correction strategy
-------------------
Each update the wrapper:

1. Reads every registered IR sensor's ``probabilityOfBlack()``.
2. For each sensor whose probability crosses the **on-line threshold**
   (rising edge), it projects the sensor's expected field position using
   the current odometry pose + robot geometry.
3. It finds the **nearest map line** to that projected point.
4. It computes the signed perpendicular error from the projected sensor
   position to the line's centerline.
5. It applies a fraction (``correction_gain``) of that error as a lateral
   nudge to the stored position offset, perpendicular to the line.

Because heading is accurate, the perpendicular correction fixes the dominant
drift mode (sideways slip) without perturbing distance-along-travel.

Coordinate frames
-----------------
* **Odometry frame** — origin at the point of last ``reset()``, meters,
  heading 0 = initial forward direction.
* **Field frame** — origin at bottom-left of the Botball table, centimeters,
  heading 0 = +X (right), positive CCW.

The wrapper converts between them using the configured ``start_pose``
(field position + heading at the moment odometry was last reset).

Example::

    from raccoon.robot.map_corrected_odometry import MapCorrectedOdometry

    mc = MapCorrectedOdometry(
        inner=robot.odometry,
        table_map=robot.table_map,
        start_x_cm=90.0,
        start_y_cm=47.0,
        start_heading_rad=0.0,
    )
    mc.register_sensor(robot.defs.front_ir, robot.sensor_position(robot.defs.front_ir))
    mc.register_sensor(robot.defs.rear_ir, robot.sensor_position(robot.defs.rear_ir))

    # In your update loop (or motion step):
    mc.update(dt)
    pose = mc.get_pose()           # corrected pose in odometry frame
    field_pos = mc.get_field_position()  # (x_cm, y_cm) on the table
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from raccoon.robot.geometry import SensorPosition
    from raccoon.robot.table_map import TableMap, MapSegment

logger = logging.getLogger("raccoon.map_odom")


@dataclass
class _SensorEntry:
    """Internal bookkeeping for one registered IR sensor."""
    sensor: Any                    # IRSensor instance
    position: "SensorPosition"     # offset from robot center
    was_on_line: bool = False      # previous frame state


def _nearest_segment(
    x: float, y: float, segments: list,
) -> Optional[Tuple[Any, float, float, float]]:
    """Find the segment closest to (x, y).

    Returns ``(segment, distance, perp_x, perp_y)`` where ``(perp_x, perp_y)``
    is the unit vector perpendicular to the segment (pointing from the segment
    centerline toward the query point), or *None* if *segments* is empty.
    """
    best: Optional[Tuple[Any, float, float, float]] = None
    best_dist = float("inf")

    for seg in segments:
        ax, ay = seg.start_x, seg.start_y
        bx, by = seg.end_x, seg.end_y
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq < 1e-12:
            # degenerate zero-length segment
            dist = math.hypot(x - ax, y - ay)
            if dist < best_dist:
                d = dist if dist > 1e-9 else 1e-9
                best = (seg, dist, (x - ax) / d, (y - ay) / d)
                best_dist = dist
            continue

        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len_sq))
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        dist = math.hypot(x - proj_x, y - proj_y)

        if dist < best_dist:
            if dist > 1e-9:
                best = (seg, dist, (x - proj_x) / dist, (y - proj_y) / dist)
            else:
                # point is exactly on the segment — perpendicular is the segment normal
                seg_len = math.sqrt(seg_len_sq)
                best = (seg, 0.0, -dy / seg_len, dx / seg_len)
            best_dist = dist

    return best


class MapCorrectedOdometry:
    """Odometry wrapper that corrects position using IR sensor + map data.

    Delegates all standard odometry calls to the *inner* odometry and
    maintains a correction offset that is applied transparently.

    Args:
        inner: The underlying odometry instance (FusedOdometry / Stm32Odometry).
        table_map: The loaded TableMap with field line/wall geometry.
        start_x_cm: Robot center X on field at odometry origin (cm).
        start_y_cm: Robot center Y on field at odometry origin (cm).
        start_heading_rad: Robot heading on field at odometry origin (rad, 0 = +X, CCW+).
        correction_gain: Fraction of perpendicular error applied per event (0–1).
            Lower values are more conservative.  Default 0.6.
        on_line_threshold: ``probabilityOfBlack()`` value above which the
            sensor is considered "on a line".  Default 0.7.
        max_correction_cm: Clamp for a single correction step to avoid
            jumps from misidentified lines.  Default 3.0 cm.
    """

    def __init__(
        self,
        inner: Any,
        table_map: "TableMap",
        start_x_cm: float,
        start_y_cm: float,
        start_heading_rad: float = 0.0,
        correction_gain: float = 0.6,
        on_line_threshold: float = 0.7,
        max_correction_cm: float = 3.0,
    ) -> None:
        self._inner = inner
        self._map = table_map
        self._start_x_cm = start_x_cm
        self._start_y_cm = start_y_cm
        self._start_heading_rad = start_heading_rad
        self._gain = correction_gain
        self._threshold = on_line_threshold
        self._max_corr = max_correction_cm

        # Correction offset in **field cm** — added to the raw converted position.
        self._offset_x_cm: float = 0.0
        self._offset_y_cm: float = 0.0

        self._sensors: List[_SensorEntry] = []

    # ── Sensor registration ──────────────────────────────────────

    def register_sensor(
        self, sensor: Any, position: "SensorPosition",
    ) -> None:
        """Register an IR sensor for map-based correction.

        Args:
            sensor: An IRSensor instance (must have ``probabilityOfBlack()``).
            position: The sensor's ``SensorPosition`` on the robot.
        """
        self._sensors.append(_SensorEntry(sensor=sensor, position=position))

    # ── Coordinate conversion helpers ────────────────────────────

    def _odom_to_field(self, pose: Any) -> Tuple[float, float, float]:
        """Convert an odometry Pose to field coordinates (cm).

        Returns (field_x_cm, field_y_cm, field_heading_rad).
        """
        # Odometry pose: position in meters, heading in radians (relative to origin).
        odom_x_m = float(pose.position[0])
        odom_y_m = float(pose.position[1])
        heading_rad = float(pose.heading)

        # Rotate odometry displacement by start heading, convert m→cm.
        cos_s = math.cos(self._start_heading_rad)
        sin_s = math.sin(self._start_heading_rad)
        dx_cm = (odom_x_m * cos_s - odom_y_m * sin_s) * 100.0
        dy_cm = (odom_x_m * sin_s + odom_y_m * cos_s) * 100.0

        field_x = self._start_x_cm + dx_cm + self._offset_x_cm
        field_y = self._start_y_cm + dy_cm + self._offset_y_cm
        field_heading = self._start_heading_rad + heading_rad

        return (field_x, field_y, field_heading)

    def _field_offset_to_odom_m(self, dx_cm: float, dy_cm: float) -> Tuple[float, float]:
        """Convert a field-frame offset (cm) to odometry-frame offset (m)."""
        cos_s = math.cos(-self._start_heading_rad)
        sin_s = math.sin(-self._start_heading_rad)
        odom_dx = (dx_cm * cos_s - dy_cm * sin_s) * 0.01
        odom_dy = (dx_cm * sin_s + dy_cm * cos_s) * 0.01
        return (odom_dx, odom_dy)

    # ── Core update ──────────────────────────────────────────────

    def update(self, dt: float) -> None:
        """Update inner odometry and apply map-based corrections.

        Call this at the same rate as the inner odometry's update loop.
        """
        self._inner.update(dt)
        self._apply_corrections()

    def _apply_corrections(self) -> None:
        """Check each sensor and apply corrections on rising-edge line detections."""
        if not self._sensors or not self._map:
            return

        pose = self._inner.get_pose()
        field_x, field_y, field_heading = self._odom_to_field(pose)

        for entry in self._sensors:
            prob = entry.sensor.probabilityOfBlack()
            on_line = prob >= self._threshold

            if on_line and not entry.was_on_line:
                # Rising edge — sensor just moved onto a line.
                self._correct_from_sensor(
                    field_x, field_y, field_heading, entry.position,
                )

            entry.was_on_line = on_line

    def _correct_from_sensor(
        self,
        robot_x: float,
        robot_y: float,
        robot_heading: float,
        sensor_pos: "SensorPosition",
    ) -> None:
        """Apply a single correction based on a sensor that just hit a line."""
        # Where do we think the sensor is on the field?
        sx, sy = self._map.sensor_field_position(
            robot_x, robot_y, robot_heading, sensor_pos
        )

        # Find the nearest line segment.
        result = _nearest_segment(sx, sy, self._map._lines)
        if result is None:
            return

        seg, dist, perp_x, perp_y = result

        # Only correct if within a reasonable distance (avoid snapping to
        # a far-away line that isn't the one the sensor actually hit).
        half_width = seg.width_cm / 2.0
        error = dist - 0.0  # we want the sensor on the centerline
        if error > self._max_corr + half_width:
            logger.debug(
                "Skipping correction: nearest line %.1f cm away (max %.1f)",
                dist, self._max_corr,
            )
            return

        # Nudge = move the robot so the sensor lands on the line centerline.
        # The perpendicular vector points from line toward current position,
        # so we subtract to move closer.
        nudge = min(error, self._max_corr) * self._gain
        self._offset_x_cm -= perp_x * nudge
        self._offset_y_cm -= perp_y * nudge

        logger.debug(
            "Map correction: sensor at (%.1f, %.1f), line dist=%.2f cm, "
            "nudge=(%.2f, %.2f) cm",
            sx, sy, dist, -perp_x * nudge, -perp_y * nudge,
        )

    # ── Public query methods (match IOdometry interface) ─────────

    def get_pose(self) -> Any:
        """Get corrected pose in odometry frame.

        The returned Pose has the correction baked into the position.
        Heading is unchanged (trusted from IMU).
        """
        pose = self._inner.get_pose()

        # Convert our field-frame offset back to odometry-frame meters
        # and add to the raw pose.
        odom_dx, odom_dy = self._field_offset_to_odom_m(
            self._offset_x_cm, self._offset_y_cm
        )
        pose.position[0] += odom_dx
        pose.position[1] += odom_dy
        return pose

    def get_field_position(self) -> Tuple[float, float]:
        """Get the corrected robot position in field coordinates (cm).

        Returns:
            (x_cm, y_cm) on the Botball table.
        """
        pose = self._inner.get_pose()
        fx, fy, _ = self._odom_to_field(pose)
        return (fx, fy)

    def get_field_heading(self) -> float:
        """Get the robot heading in field coordinates (radians).

        Returns:
            Heading in radians (0 = +X, CCW positive).
        """
        pose = self._inner.get_pose()
        return self._start_heading_rad + float(pose.heading)

    # ── Delegated IOdometry methods ──────────────────────────────

    def get_heading(self) -> float:
        return self._inner.get_heading()

    def get_absolute_heading(self) -> float:
        return self._inner.get_absolute_heading()

    def get_heading_error(self, target_heading_rad: float) -> float:
        return self._inner.get_heading_error(target_heading_rad)

    def get_distance_from_origin(self) -> Any:
        return self._inner.get_distance_from_origin()

    def get_path_length(self) -> float:
        return self._inner.get_path_length()

    def reset(self) -> None:
        """Reset inner odometry and clear accumulated corrections."""
        self._inner.reset()
        self._offset_x_cm = 0.0
        self._offset_y_cm = 0.0
        for entry in self._sensors:
            entry.was_on_line = False

    def reset_with_field_pose(
        self, x_cm: float, y_cm: float, heading_rad: float = 0.0,
    ) -> None:
        """Reset odometry and set a new field-frame starting pose.

        Use this when you know exactly where the robot is (e.g. after
        aligning against a wall or starting position).

        Args:
            x_cm: New field X position (cm from left).
            y_cm: New field Y position (cm from bottom).
            heading_rad: New field heading (radians, 0 = +X, CCW+).
        """
        self._inner.reset()
        self._start_x_cm = x_cm
        self._start_y_cm = y_cm
        self._start_heading_rad = heading_rad
        self._offset_x_cm = 0.0
        self._offset_y_cm = 0.0
        for entry in self._sensors:
            entry.was_on_line = False

    @property
    def correction_offset_cm(self) -> Tuple[float, float]:
        """Current accumulated correction in field cm (for diagnostics)."""
        return (self._offset_x_cm, self._offset_y_cm)

    def __repr__(self) -> str:
        n = len(self._sensors)
        return (
            f"MapCorrectedOdometry(sensors={n}, "
            f"offset=({self._offset_x_cm:.2f}, {self._offset_y_cm:.2f}) cm)"
        )
