"""Convenience wrapper that parameterises a drive arc by heading change + arc distance."""

import math

from raccoon.step.annotation import dsl
from .arc import DriveArc
from .arc_dsl import DriveArcBuilder

_UNSET = object()


@dsl(tags=["motion", "arc"])
def drive_arc_segment(
    heading_degrees: float = _UNSET,
    distance_cm: float = _UNSET,
    speed: float = 1.0,
) -> DriveArcBuilder:
    """Drive along a circular arc specified by heading change and travel distance.

    Instead of choosing a radius and arc angle separately, specify the desired
    heading change and how far the robot should travel along the arc.  The
    radius is computed automatically:
    ``radius = distance_cm / radians(|heading_degrees|)``.

    Positive heading_degrees = counter-clockwise (left), negative = clockwise
    (right) — same convention as ``drive_arc``.

    Args:
        heading_degrees: Desired heading change in degrees.
            Positive = left/CCW, negative = right/CW.
        distance_cm: Distance the robot travels along the arc path in
            centimeters (always positive).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Raises:
        ValueError: If heading_degrees is zero (arc would have infinite radius)
            or distance_cm is not positive.

    Returns:
        A DriveArcBuilder configured with the computed radius and heading.

    Example::

        from raccoon.step.motion import drive_arc_segment

        # Gently curve left by 10 degrees over 25 cm of travel
        drive_arc_segment(heading_degrees=10, distance_cm=25)

        # Curve right by 30 degrees over 40 cm at half speed
        drive_arc_segment(heading_degrees=-30, distance_cm=40, speed=0.5)
    """
    if heading_degrees is _UNSET or distance_cm is _UNSET:
        # Return a builder that will be filled in later via chaining
        b = DriveArcBuilder()
        if heading_degrees is not _UNSET:
            b._degrees = heading_degrees
        if distance_cm is not _UNSET:
            # Can't compute radius without both, store for later
            pass
        b._speed = speed
        return b

    if heading_degrees == 0:
        raise ValueError(
            "heading_degrees must be non-zero (use drive_forward for straight lines)"
        )
    if distance_cm <= 0:
        raise ValueError("distance_cm must be positive")

    theta_rad = math.radians(abs(heading_degrees))
    radius_cm = distance_cm / theta_rad

    b = DriveArcBuilder()
    b._radius_cm = radius_cm
    b._degrees = heading_degrees
    b._speed = speed
    return b
