"""Table map model for Botball field geometry.

Stores line and wall segments from the .ftmap vector format and provides
spatial queries (point-on-line, point-on-wall, distance-to-nearest-line).

The coordinate system matches the Botball field: origin at bottom-left,
X right, Y up, all values in centimeters.

Example usage::

    from raccoon.robot.table_map import TableMap

    # Load from ftmap dict (as stored in raccoon.project.yml)
    table_map = TableMap.from_ftmap(config["robot"]["physical"]["table_map"])

    # Load from .ftmap file
    table_map = TableMap.load("config/table_map.ftmap")

    # Query
    table_map.is_on_line(50.0, 30.0)       # True/False
    table_map.is_on_wall(0.0, 50.0)        # True/False
    table_map.distance_to_nearest_line(50.0, 30.0)  # cm
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, TYPE_CHECKING

if TYPE_CHECKING:
    from raccoon.robot.geometry import SensorPosition


@dataclass(frozen=True)
class MapSegment:
    """A single line or wall segment on the field, in centimeters."""

    start_x: float
    start_y: float
    end_x: float
    end_y: float
    width_cm: float
    kind: str  # 'line' or 'wall'

    @property
    def length(self) -> float:
        return math.hypot(self.end_x - self.start_x, self.end_y - self.start_y)


def _point_to_segment_distance(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> float:
    """Shortest distance from point (px, py) to line segment (ax,ay)-(bx,by)."""
    dx = bx - ax
    dy = by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


class TableMap:
    """Botball field table map with line and wall segments.

    All coordinates are in centimeters with origin at bottom-left of the field.
    """

    def __init__(
        self,
        width_cm: float,
        height_cm: float,
        segments: Sequence[MapSegment] = (),
    ) -> None:
        self.width_cm = width_cm
        self.height_cm = height_cm
        self._lines: List[MapSegment] = []
        self._walls: List[MapSegment] = []
        for seg in segments:
            if seg.kind == "wall":
                self._walls.append(seg)
            else:
                self._lines.append(seg)

    @property
    def lines(self) -> List[MapSegment]:
        return list(self._lines)

    @property
    def walls(self) -> List[MapSegment]:
        return list(self._walls)

    @property
    def all_segments(self) -> List[MapSegment]:
        return self._lines + self._walls

    # ── Construction ─────────────────────────────────────────────

    @classmethod
    def from_ftmap(cls, data: Dict[str, Any]) -> "TableMap":
        """Create a TableMap from an ftmap dict (as stored in project config).

        Args:
            data: Dict with keys ``format``, ``version``, ``table``, ``lines``.

        Returns:
            A populated TableMap instance.

        Raises:
            ValueError: If the format or version is unsupported.
        """
        fmt = data.get("format")
        if fmt != "flowchart-table-map":
            raise ValueError(f"Unsupported table map format: {fmt!r}")
        version = data.get("version")
        if version != 1:
            raise ValueError(f"Unsupported table map version: {version!r}")

        table = data.get("table", {})
        width_cm = float(table.get("widthCm", 0))
        height_cm = float(table.get("heightCm", 0))

        segments: List[MapSegment] = []
        for entry in data.get("lines", []):
            segments.append(MapSegment(
                start_x=float(entry["startX"]),
                start_y=float(entry["startY"]),
                end_x=float(entry["endX"]),
                end_y=float(entry["endY"]),
                width_cm=float(entry["widthCm"]),
                kind=str(entry.get("kind", "line")),
            ))

        return cls(width_cm, height_cm, segments)

    @classmethod
    def load(cls, path: str | Path) -> "TableMap":
        """Load a TableMap from a ``.ftmap`` JSON file.

        Args:
            path: Path to the .ftmap file.

        Returns:
            A populated TableMap instance.
        """
        text = Path(path).read_text(encoding="utf-8")
        data = json.loads(text)
        return cls.from_ftmap(data)

    def to_ftmap(self) -> Dict[str, Any]:
        """Serialize this TableMap back to the ftmap dict format."""
        return {
            "format": "flowchart-table-map",
            "version": 1,
            "table": {
                "widthCm": self.width_cm,
                "heightCm": self.height_cm,
            },
            "lines": [
                {
                    "kind": seg.kind,
                    "startX": seg.start_x,
                    "startY": seg.start_y,
                    "endX": seg.end_x,
                    "endY": seg.end_y,
                    "widthCm": seg.width_cm,
                }
                for seg in self._lines + self._walls
            ],
        }

    # ── Spatial queries ──────────────────────────────────────────

    def distance_to_nearest_line(self, x_cm: float, y_cm: float) -> float:
        """Shortest distance from a point to any line segment centerline.

        Args:
            x_cm: X position on field (cm from left).
            y_cm: Y position on field (cm from bottom).

        Returns:
            Distance in cm, or ``float('inf')`` if there are no lines.
        """
        if not self._lines:
            return float("inf")
        return min(
            _point_to_segment_distance(
                x_cm, y_cm, s.start_x, s.start_y, s.end_x, s.end_y
            )
            for s in self._lines
        )

    def distance_to_nearest_wall(self, x_cm: float, y_cm: float) -> float:
        """Shortest distance from a point to any wall segment centerline.

        Args:
            x_cm: X position on field (cm from left).
            y_cm: Y position on field (cm from bottom).

        Returns:
            Distance in cm, or ``float('inf')`` if there are no walls.
        """
        if not self._walls:
            return float("inf")
        return min(
            _point_to_segment_distance(
                x_cm, y_cm, s.start_x, s.start_y, s.end_x, s.end_y
            )
            for s in self._walls
        )

    def is_on_line(self, x_cm: float, y_cm: float) -> bool:
        """Check if a point is within any line segment's width.

        A point is "on" a line if its perpendicular distance to the segment
        centerline is less than half the line's width.

        Args:
            x_cm: X position on field (cm from left).
            y_cm: Y position on field (cm from bottom).

        Returns:
            True if the point overlaps a line.
        """
        for seg in self._lines:
            dist = _point_to_segment_distance(
                x_cm, y_cm, seg.start_x, seg.start_y, seg.end_x, seg.end_y
            )
            if dist <= seg.width_cm / 2.0:
                return True
        return False

    def is_on_wall(self, x_cm: float, y_cm: float) -> bool:
        """Check if a point is within any wall segment's width.

        Args:
            x_cm: X position on field (cm from left).
            y_cm: Y position on field (cm from bottom).

        Returns:
            True if the point overlaps a wall.
        """
        for seg in self._walls:
            dist = _point_to_segment_distance(
                x_cm, y_cm, seg.start_x, seg.start_y, seg.end_x, seg.end_y
            )
            if dist <= seg.width_cm / 2.0:
                return True
        return False

    # ── Sensor queries ───────────────────────────────────────────

    def sensor_field_position(
        self,
        robot_x_cm: float,
        robot_y_cm: float,
        robot_heading_rad: float,
        sensor: "SensorPosition",
    ) -> tuple[float, float]:
        """Compute the field position of a sensor given robot pose.

        The robot pose is in field coordinates (origin bottom-left, heading 0 = +X, CCW positive).
        The sensor position is in robot-relative coordinates (forward/strafe from geometric center).

        Args:
            robot_x_cm: Robot center X on field (cm).
            robot_y_cm: Robot center Y on field (cm).
            robot_heading_rad: Robot heading (radians, 0 = +X, CCW positive).
            sensor: SensorPosition with forward_cm and strafe_cm offsets.

        Returns:
            Tuple of (field_x_cm, field_y_cm) for the sensor.
        """
        cos_h = math.cos(robot_heading_rad)
        sin_h = math.sin(robot_heading_rad)
        # forward_cm is along heading, strafe_cm is 90° CCW from heading
        field_x = robot_x_cm + sensor.forward_cm * cos_h - sensor.strafe_cm * sin_h
        field_y = robot_y_cm + sensor.forward_cm * sin_h + sensor.strafe_cm * cos_h
        return (field_x, field_y)

    def sensor_is_on_line(
        self,
        robot_x_cm: float,
        robot_y_cm: float,
        robot_heading_rad: float,
        sensor: "SensorPosition",
    ) -> bool:
        """Check if a sensor would be over a black line given current robot pose.

        Args:
            robot_x_cm: Robot center X on field (cm).
            robot_y_cm: Robot center Y on field (cm).
            robot_heading_rad: Robot heading (radians).
            sensor: SensorPosition offset from robot center.

        Returns:
            True if the sensor's field position overlaps a line.
        """
        sx, sy = self.sensor_field_position(
            robot_x_cm, robot_y_cm, robot_heading_rad, sensor
        )
        return self.is_on_line(sx, sy)

    def sensor_is_on_wall(
        self,
        robot_x_cm: float,
        robot_y_cm: float,
        robot_heading_rad: float,
        sensor: "SensorPosition",
    ) -> bool:
        """Check if a sensor would be over a wall given current robot pose.

        Args:
            robot_x_cm: Robot center X on field (cm).
            robot_y_cm: Robot center Y on field (cm).
            robot_heading_rad: Robot heading (radians).
            sensor: SensorPosition offset from robot center.

        Returns:
            True if the sensor's field position overlaps a wall.
        """
        sx, sy = self.sensor_field_position(
            robot_x_cm, robot_y_cm, robot_heading_rad, sensor
        )
        return self.is_on_wall(sx, sy)

    def __repr__(self) -> str:
        return (
            f"TableMap({self.width_cm}x{self.height_cm}cm, "
            f"{len(self._lines)} lines, {len(self._walls)} walls)"
        )
