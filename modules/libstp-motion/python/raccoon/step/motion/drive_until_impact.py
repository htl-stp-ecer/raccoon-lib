"""
Drive straight until an obstacle is hit, recording how far the robot got.

Drives at constant velocity (no heading correction) and watches the IMU's
gravity-compensated linear acceleration for a collision spike, exactly like
:mod:`wall_align`.  The difference is the *result*: instead of settling flush
against a wall, this step snapshots the odometer at start and stores the
distance travelled at the moment of impact.

The point is collision-aware distancing.  You drive forward expecting to reach
some nominal distance, but a game piece may be in the way.  When the bump fires
early, ``step.impact_result.forward_cm`` tells you how far you actually got, so
a following step can drive back exactly that far (minus a clearance) instead of
a fixed amount.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from raccoon.foundation import ChassisVelocity
from raccoon.hal import IMU

from .. import dsl
from ..annotation import dsl_step
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dataclass
class ImpactResult:
    """Outcome of a :func:`drive_until_impact` run."""

    impacted: bool
    """True if a collision spike stopped the step; False if it reached
    ``max_cm`` (or timed out) without hitting anything."""
    forward_cm: float
    """Distance the robot actually drove before stopping, in centimeters.
    Always positive regardless of drive direction.  This is the value you
    feed back into a drive-back step."""
    accel_magnitude: float
    """XY linear-acceleration magnitude at impact in m/s² (0.0 if no bump
    was detected)."""

    def __str__(self) -> str:
        outcome = "impact" if self.impacted else "no-impact"
        return (
            f"ImpactResult({outcome}, forward={self.forward_cm:.1f}cm, "
            f"accel={self.accel_magnitude:.2f}m/s^2)"
        )


@dsl_step(tags=["motion", "drive"])
class DriveUntilImpact(MotionStep):
    """Drive straight until a collision is detected, measuring the distance driven.

    The robot drives at constant velocity with no heading correction while the
    IMU's gravity-compensated horizontal acceleration is monitored.  A spike
    above ``accel_threshold`` is treated as an impact: the step stops
    immediately and records how far it drove.  If no impact occurs, the step
    stops cleanly once ``max_cm`` is reached (or after ``max_duration`` as a
    safety backstop).

    Distance is measured relative to where *this step* started (the odometer is
    snapshotted in ``on_start``), because motion steps no longer reset odometry.
    Detection only arms after an initial guard so the robot's own launch
    acceleration isn't mistaken for a collision: it stays disabled until BOTH
    ``grace_period`` seconds have elapsed AND the robot has driven at least
    ``grace_distance_cm`` clear of the start.

    This factory drives in and stops on impact — use it when you only need the
    "stop when I hit something" behaviour.  To *reuse the measured distance*
    (e.g. drive back exactly as far as you came), reach for
    :func:`drive_until_impact_back`, which captures the live step instance and
    wires up the drive-back for you.  Reading ``impact_result`` off the value
    this factory returns does NOT work: the factory returns a builder that
    ``seq`` resolves into a *fresh* step instance, so the object you hold never
    runs and its ``impact_result`` stays ``None``.

    Prerequisites:
        A working IMU (``raccoon.hal.IMU``) and odometry.  Detection quality
        depends on ``accel_threshold`` being tuned above the cruise-noise floor
        on real hardware; the mock simulator does not model collision impacts,
        so the *triggering* can only be validated on the robot.

    Args:
        max_cm: Nominal distance to drive in centimeters if nothing is hit
            (default 50.0).  Doubles as a safety cap on travel.
        speed: Drive speed in m/s (default 0.5).  Lower is gentler and makes
            the bump cleaner and less likely to shove the game piece.
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s² to
            classify as an impact (default 0.5).  Lower = more sensitive but
            more prone to false triggers on rough surfaces.
        grace_period: Seconds to ignore acceleration after starting, so the
            robot's own acceleration doesn't trigger detection (default 0.3).
        grace_distance_cm: Centimeters the robot must drive clear of the start
            before impact detection arms (default 3.0).  Guards against the
            launch lurch or an obstacle already touching the robot at rest
            triggering instantly.  Combined with ``grace_period`` — both must be
            satisfied before detection is active.
        max_duration: Safety timeout in seconds — the step finishes even if
            neither an impact nor ``max_cm`` is reached (default 8.0).
        backward: Drive in reverse instead of forward (default False).

    Returns:
        A DriveUntilImpact step that drives in and stops on impact.

    Example::

        from raccoon.step.motion import drive_until_impact

        # Nudge forward up to 30 cm, stopping the instant something is hit.
        drive_until_impact(max_cm=30, speed=0.4)
    """

    def __init__(
        self,
        max_cm: float = 50.0,
        speed: float = 0.5,
        accel_threshold: float = 0.5,
        grace_period: float = 0.3,
        grace_distance_cm: float = 3.0,
        max_duration: float = 8.0,
        backward: bool = False,
    ) -> None:
        super().__init__()
        if not isinstance(max_cm, int | float) or max_cm <= 0:
            msg = f"max_cm must be > 0, got {max_cm}"
            raise ValueError(msg)
        if not isinstance(speed, int | float) or speed <= 0:
            msg = f"speed must be > 0, got {speed}"
            raise ValueError(msg)
        if not isinstance(accel_threshold, int | float) or accel_threshold <= 0:
            msg = f"accel_threshold must be > 0, got {accel_threshold}"
            raise ValueError(msg)
        if not isinstance(grace_period, int | float) or grace_period < 0:
            msg = f"grace_period must be >= 0, got {grace_period}"
            raise ValueError(msg)
        if not isinstance(grace_distance_cm, int | float) or grace_distance_cm < 0:
            msg = f"grace_distance_cm must be >= 0, got {grace_distance_cm}"
            raise ValueError(msg)
        if not isinstance(max_duration, int | float) or max_duration <= 0:
            msg = f"max_duration must be > 0, got {max_duration}"
            raise ValueError(msg)

        self._max_m = max_cm / 100.0
        # Negative x drives the chassis backward; speed is stored positive.
        self._vx = -abs(speed) if backward else abs(speed)
        self._speed = abs(speed)
        self._accel_threshold = accel_threshold
        self._grace_period = grace_period
        self._grace_distance_m = grace_distance_cm / 100.0
        self._max_duration = max_duration
        self._backward = backward

        self._imu: IMU | None = None
        self._elapsed: float = 0.0
        self._start_len: float = 0.0
        self.impact_result: ImpactResult | None = None

    def _generate_signature(self) -> str:
        direction = "backward" if self._backward else "forward"
        return (
            f"DriveUntilImpact(dir={direction}, max_m={self._max_m:.2f}, "
            f"speed={self._speed:.2f}, threshold={self._accel_threshold:.1f})"
        )

    def on_start(self, robot: "GenericRobot") -> None:
        self._imu = IMU()
        self._elapsed = 0.0
        # Odometer is monotonic and direction-agnostic; snapshot it so the
        # travelled distance is measured from this step's own start.
        self._start_len = robot.odometry.get_path_length()
        self.impact_result = None
        robot.drive.set_velocity(ChassisVelocity(self._vx, 0.0, 0.0))

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        robot.drive.update(dt)
        self._elapsed += dt

        driven_m = robot.odometry.get_path_length() - self._start_len

        # Reached the nominal distance without hitting anything.
        if driven_m >= self._max_m:
            self.impact_result = ImpactResult(
                impacted=False,
                forward_cm=driven_m * 100.0,
                accel_magnitude=0.0,
            )
            self.debug(f"Reached max distance {driven_m * 100.0:.1f} cm without impact")
            return True

        # Safety timeout — motion never progressed or the cap wasn't hit.
        if self._elapsed >= self._max_duration:
            self.debug(
                f"Timed out after {self._elapsed:.1f}s at "
                f"{driven_m * 100.0:.1f} cm without impact"
            )
            self.impact_result = ImpactResult(
                impacted=False,
                forward_cm=driven_m * 100.0,
                accel_magnitude=0.0,
            )
            return True

        # Ignore the acceleration transient while the robot spins up from rest:
        # detection only arms once BOTH the grace time has elapsed AND the robot
        # has actually driven clear of the start (so the launch lurch and any
        # standing contact can't be mistaken for an impact).
        if self._elapsed < self._grace_period or driven_m < self._grace_distance_m:
            return False

        ax, ay, _az = self._imu.get_linear_acceleration()
        accel_mag = math.hypot(ax, ay)
        if accel_mag >= self._accel_threshold:
            self.impact_result = ImpactResult(
                impacted=True,
                forward_cm=driven_m * 100.0,
                accel_magnitude=accel_mag,
            )
            self.debug(f"Impact detected: {accel_mag:.2f} m/s² after " f"{driven_m * 100.0:.1f} cm")
            return True

        return False


@dsl(tags=["motion", "drive"])
def drive_until_impact_back(
    max_cm: float = 50.0,
    clearance_cm: float = 0.0,
    speed: float = 0.5,
    accel_threshold: float = 0.5,
    grace_period: float = 0.3,
    grace_distance_cm: float = 3.0,
    max_duration: float = 8.0,
):
    """Drive forward until impact, then reverse exactly as far as was driven.

    Collision-aware distancing in one step.  The robot drives forward up to
    ``max_cm``, stopping early if it bumps a game piece (IMU acceleration
    spike).  It then reverses by the distance it actually travelled, minus
    ``clearance_cm``.  Hit a piece at 22 cm and it backs out ~22 cm; reach the
    full ``max_cm`` untouched and it backs out that far — so the return leg
    always matches how far you really got.

    Internally this constructs a live :class:`DriveUntilImpact` instance,
    captures it in a closure, and appends a ``defer(...)`` that reads its
    ``impact_result`` at runtime to build the drive-back leg.  This is the
    supported way to reuse the measured distance — see :func:`drive_until_impact`
    for why holding the plain factory's return value does not work.

    If the measured return distance rounds to zero (e.g. ``clearance_cm`` is
    larger than the distance driven), the drive-back leg is skipped.

    Prerequisites:
        A working IMU (``raccoon.hal.IMU``) and odometry.  ``accel_threshold``
        must be tuned above the cruise-noise floor on real hardware; the mock
        simulator does not model collision impacts, so the *triggering* can
        only be validated on the robot.

    Args:
        max_cm: Nominal forward distance in centimeters if nothing is hit
            (default 50.0).  Doubles as a safety cap on travel.
        clearance_cm: Centimeters to leave when returning — the drive-back is
            ``driven - clearance`` (default 0.0 = return to the exact start).
        speed: Drive speed in m/s (default 0.5).  Lower is gentler and makes
            the bump cleaner and less likely to shove the game piece.
        accel_threshold: Minimum XY linear-acceleration magnitude in m/s² to
            classify as an impact (default 0.5).
        grace_period: Seconds to ignore acceleration after starting, so the
            robot's own acceleration doesn't trigger detection (default 0.3).
        grace_distance_cm: Centimeters the robot must drive clear of the start
            before impact detection arms (default 3.0), so the launch lurch
            doesn't trigger instantly.  Combined with ``grace_period``.
        max_duration: Safety timeout in seconds for the forward leg (default
            8.0).

    Returns:
        A ``seq`` step: drive-until-impact followed by a deferred drive-back.

    Example::

        from raccoon.step.motion import drive_until_impact_back

        # Drive up to 40 cm forward; if a cube stops us early, back out just
        # as far, leaving a 3 cm clearance.
        drive_until_impact_back(max_cm=40, clearance_cm=3.0, speed=0.4)
    """
    # Lazy imports: this module is imported mid-initialization of raccoon.step,
    # so importing seq/defer at module top can hit a partially-initialized
    # package. Resolving them here (like the path passes do) sidesteps that.
    from raccoon.step import seq
    from raccoon.step.logic import defer

    from .drive_dsl import drive_backward
    from .stop_dsl import stop

    measure = DriveUntilImpact(
        max_cm=max_cm,
        speed=speed,
        accel_threshold=accel_threshold,
        grace_period=grace_period,
        grace_distance_cm=grace_distance_cm,
        max_duration=max_duration,
    )

    def _drive_back(_robot: "GenericRobot"):
        result = measure.impact_result
        back_cm = 0.0 if result is None else max(result.forward_cm - clearance_cm, 0.0)
        # drive_backward requires cm > 0; nothing meaningful to undo → no-op.
        if back_cm < 0.05:
            return stop()
        return drive_backward(back_cm)

    return seq([measure, defer(_drive_back)])
