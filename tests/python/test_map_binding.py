"""Smoke tests for the raccoon.map pybind11 module (libstp::map).

The C++ test suite covers the geometric edge cases. This file just makes
sure the Python surface — duck-typed sensor coercion, the from_ftmap
classmethod, and the property aliases — actually works against the live
binding.
"""

from __future__ import annotations

import math
from collections import namedtuple

import pytest

from raccoon.map import (
    FtmapParseError,
    MapSegment,
    Point2D,
    SensorOffset,
    WheelOffset,
    WorldMap,
)

SINGLE_LINE_FTMAP = {
    "format": "flowchart-table-map",
    "version": 1,
    "table": {"widthCm": 200, "heightCm": 100},
    "lines": [
        {
            "kind": "line",
            "startX": 50,
            "startY": 50,
            "endX": 150,
            "endY": 50,
            "widthCm": 1.5,
        },
    ],
}


def test_from_ftmap_parses_dimensions_and_lines() -> None:
    m = WorldMap.from_ftmap(SINGLE_LINE_FTMAP)
    assert m.table_width_cm == pytest.approx(200.0)
    assert m.table_height_cm == pytest.approx(100.0)
    # Aliases
    assert m.width_cm == pytest.approx(200.0)
    assert m.height_cm == pytest.approx(100.0)

    lines = m.lines()
    assert len(lines) == 1
    assert lines[0].kind == MapSegment.Kind.LINE
    assert lines[0].length == pytest.approx(100.0)


def test_from_ftmap_rejects_bad_format() -> None:
    bad = {"format": "wrong", "version": 1, "table": {"widthCm": 1, "heightCm": 1}}
    with pytest.raises(FtmapParseError):
        WorldMap.from_ftmap(bad)


def test_parse_ftmap_string_matches_dict() -> None:
    # Same JSON document via parse_ftmap and from_ftmap must produce maps
    # that agree on the geometric queries — the silent-divergence trap the
    # design plan calls out.
    import json

    payload = json.dumps(SINGLE_LINE_FTMAP)
    via_string = WorldMap()
    via_string.parse_ftmap(payload)
    via_dict = WorldMap.from_ftmap(SINGLE_LINE_FTMAP)

    for x, y in [(100.0, 50.0), (100.0, 60.0), (10.0, 50.0)]:
        assert via_string.is_on_line(x, y) == via_dict.is_on_line(x, y), (x, y)


def test_is_on_line_and_wall() -> None:
    m = WorldMap.from_ftmap(SINGLE_LINE_FTMAP)
    assert m.is_on_line(100.0, 50.0)
    # is_on_black_line legacy alias
    assert m.is_on_black_line(100.0, 50.0)
    assert not m.is_on_line(100.0, 60.0)
    # Border edges count as walls even with no explicit walls in the map
    assert m.is_on_wall(0.0, 50.0)
    assert m.is_on_wall(200.0, 50.0)
    assert not m.is_on_wall(100.0, 50.0)


def test_distance_queries() -> None:
    m = WorldMap.from_ftmap(SINGLE_LINE_FTMAP)
    assert m.distance_to_nearest_line(100.0, 60.0) == pytest.approx(10.0, rel=1e-4)
    # No explicit walls — nearest wall is a border.
    assert m.distance_to_nearest_wall(10.0, 50.0) == pytest.approx(10.0, rel=1e-4)


# Duck-typed stand-in for raccoon.robot.geometry.SensorPosition. The C++
# binding must accept anything with forward_cm/strafe_cm so users don't
# have to convert their existing dataclass instances.
FakeSensorPos = namedtuple("FakeSensorPos", ["forward_cm", "strafe_cm"])


def test_sensor_field_position_with_real_sensor_offset() -> None:
    m = WorldMap()
    m.set_table(200.0, 200.0)
    s = SensorOffset(forward_cm=10.0, strafe_cm=0.0)
    p = m.sensor_field_position(50.0, 50.0, 0.0, s)
    assert isinstance(p, Point2D)
    assert p.x_cm == pytest.approx(60.0)
    assert p.y_cm == pytest.approx(50.0)
    # Tuple-style unpacking like the old TableMap returned.
    x, y = p
    assert (x, y) == (pytest.approx(60.0), pytest.approx(50.0))


def test_sensor_field_position_accepts_duck_typed_sensor() -> None:
    m = WorldMap()
    m.set_table(200.0, 200.0)
    sensor = FakeSensorPos(forward_cm=0.0, strafe_cm=5.0)
    p = m.sensor_field_position(50.0, 50.0, 0.0, sensor)
    # Strafe is 90° CCW from heading; at heading 0 that's +Y.
    assert p.x_cm == pytest.approx(50.0)
    assert p.y_cm == pytest.approx(55.0)


def test_sensor_is_on_line_rotation() -> None:
    m = WorldMap.from_ftmap(SINGLE_LINE_FTMAP)
    s = FakeSensorPos(forward_cm=10.0, strafe_cm=0.0)
    # Robot at (90, 50) facing +X → sensor at (100, 50): on the line.
    assert m.sensor_is_on_line(90.0, 50.0, 0.0, s)
    # Facing +Y → sensor at (90, 60): off the line.
    assert not m.sensor_is_on_line(90.0, 50.0, math.pi / 2, s)


def test_sensor_argument_must_be_duck_typed() -> None:
    m = WorldMap()
    m.set_table(100.0, 100.0)
    with pytest.raises(TypeError):
        m.sensor_field_position(0.0, 0.0, 0.0, "not a sensor")


def test_wheel_offset_round_trip() -> None:
    w = WheelOffset(forward_cm=5.0, strafe_cm=-3.0)
    assert w.forward_cm == pytest.approx(5.0)
    assert w.strafe_cm == pytest.approx(-3.0)
