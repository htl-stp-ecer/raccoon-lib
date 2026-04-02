"""
Single-sensor lineup on a line with known width.

Drives across the line, measures the apparent crossing width, and computes
the angular displacement from ``acos(actual_width / apparent_width)``.
Then executes a corrective turn.

The direction of the correction cannot be determined from a single crossing,
so it must be provided by the caller.
"""
import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity
from libstp.sensor_ir import IRSensor
from libstp.step import Sequential, seq, defer

from ..turn_dsl import turn_left, turn_right
from ... import SimulationStep, SimulationStepDelta, dsl
from ..motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class CorrectionSide(Enum):
    """Which direction to apply the angular correction."""
    LEFT = "left"
    RIGHT = "right"


@dataclass
class SingleSensorLineupConfig:
    sensor: IRSensor
    line_width_cm: float  # known width of the line
    correction_side: CorrectionSide = CorrectionSide.LEFT
    forward_speed: float = 0.15  # approach speed in m/s
    entry_threshold: float = 0.7  # black confidence for leading edge
    exit_threshold: float = 0.3  # black confidence for trailing edge


@dsl(hidden=True)
class SingleSensorCrossing(MotionStep):
    """Drive across a line with one IR sensor and measure the apparent crossing width.

    The robot drives at the configured speed while monitoring a single IR
    sensor's black-surface confidence.  The crossing proceeds in two phases:

    * **Phase 1 (searching):** Drive until the sensor reads above
      ``entry_threshold`` -- this marks the leading edge of the line.  The
      odometry position is recorded.
    * **Phase 2 (on line):** Continue driving until the sensor drops below
      ``exit_threshold`` -- this marks the trailing edge.  The odometry
      position is recorded again.

    The apparent crossing width (leading-to-trailing distance) is compared
    against the known physical ``line_width_cm`` to compute the angular
    displacement via ``acos(actual_width / apparent_width)``.  A wider
    apparent crossing means the robot is more skewed relative to the line.

    Because a single sensor cannot determine *which direction* the robot is
    skewed, the caller must supply ``correction_side`` to indicate the turn
    direction.

    This is an internal building-block step -- use ``forward_single_lineup``
    or ``backward_single_lineup`` instead.

    Args:
        config: A ``SingleSensorLineupConfig`` containing the sensor,
            line width, thresholds, speed, and correction direction.
    """

    def __init__(self, config: SingleSensorLineupConfig):
        super().__init__()
        self.config = config
        self._phase = 0  # 0=searching, 1=on_line
        self._leading_edge_pos = 0.0
        self._trailing_edge_pos = 0.0
        self.apparent_width_m = 0.0
        self.crossing_angle_rad = 0.0

    def _generate_signature(self) -> str:
        direction = "forward" if self.config.forward_speed > 0 else "backward"
        return (
            f"SingleSensorCrossing({direction}, "
            f"line_width={self.config.line_width_cm:.1f}cm)"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=0.1 if self.config.forward_speed > 0 else -0.1,
            strafe=0.0,
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        self._phase = 0
        self._leading_edge_pos = 0.0
        self._trailing_edge_pos = 0.0
        robot.odometry.reset()
        robot.drive.set_velocity(
            ChassisVelocity(self.config.forward_speed, 0.0, 0.0)
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        robot.odometry.update(dt)
        robot.drive.update(dt)

        current = robot.odometry.get_distance_from_origin().forward
        confidence = self.config.sensor.probabilityOfBlack()

        if self._phase == 0:
            # Searching for leading edge
            if confidence >= self.config.entry_threshold:
                self._leading_edge_pos = current
                self._phase = 1
                self.debug(
                    f"Leading edge at {current * 100:.1f}cm "
                    f"(confidence={confidence:.2f})"
                )
        elif self._phase == 1:
            # On the line, waiting for trailing edge
            if confidence <= self.config.exit_threshold:
                self._trailing_edge_pos = current
                self.apparent_width_m = abs(
                    self._trailing_edge_pos - self._leading_edge_pos
                )
                actual_width_m = self.config.line_width_cm / 100.0

                # acos(actual / apparent) — clamp ratio to [0, 1] for safety
                ratio = min(actual_width_m / self.apparent_width_m, 1.0) \
                    if self.apparent_width_m > 0 else 1.0
                self.crossing_angle_rad = math.acos(ratio)

                self.debug(
                    f"Trailing edge at {current * 100:.1f}cm, "
                    f"apparent={self.apparent_width_m * 100:.1f}cm, "
                    f"angle={math.degrees(self.crossing_angle_rad):.1f}deg"
                )
                return True

        return False


def _compute_single_sensor_turn(crossing: SingleSensorCrossing, _robot: "GenericRobot"):
    from libstp.foundation import info
    from libstp.step import Run

    degrees = math.degrees(crossing.crossing_angle_rad)

    if degrees < 0.1:
        info(f"Already at target heading (error={degrees:.3f}°) — skipping turn")
        return Run(lambda _: None)

    if crossing.config.correction_side == CorrectionSide.RIGHT:
        return turn_right(degrees)
    return turn_left(degrees)


@dsl(hidden=True)
def single_sensor_lineup(
    sensor: IRSensor,
    line_width_cm: float = 5.0,
    correction_side: CorrectionSide = CorrectionSide.LEFT,
    forward_speed: float = 0.15,
    entry_threshold: float = 0.7,
    exit_threshold: float = 0.3,
) -> Sequential:
    """Compose a single-sensor crossing measurement with a deferred corrective turn.

    Drives across the line with one sensor, measures the apparent crossing
    width, computes the angular displacement from the known line width, and
    executes a corrective turn in the specified direction.  If the measured
    skew is less than 1 degree the turn is skipped.

    This is an internal helper -- prefer ``forward_single_lineup`` or
    ``backward_single_lineup``.

    Args:
        sensor: The IR sensor instance used to detect the line.
        line_width_cm: Known physical width of the line in centimeters.
        correction_side: Direction to turn for correction (LEFT or RIGHT).
            Must be determined by the caller since a single sensor cannot
            detect skew direction.
        forward_speed: Approach speed in m/s (negative for backward).
        entry_threshold: Black confidence (0--1) for detecting the leading edge
            of the line.
        exit_threshold: Black confidence (0--1) for detecting the trailing edge
            of the line.  Should be lower than ``entry_threshold`` to avoid
            false exits.

    Returns:
        Sequential: A two-step sequence (crossing measurement + corrective
        turn).
    """
    config = SingleSensorLineupConfig(
        sensor=sensor,
        line_width_cm=line_width_cm,
        correction_side=correction_side,
        forward_speed=forward_speed,
        entry_threshold=entry_threshold,
        exit_threshold=exit_threshold,
    )
    crossing = SingleSensorCrossing(config)
    return seq([crossing, defer(lambda robot: _compute_single_sensor_turn(crossing, robot))])


@dsl(tags=["motion", "lineup"])
def forward_single_lineup(
    sensor: IRSensor,
    line_width_cm: float = 5.0,
    correction_side: CorrectionSide = CorrectionSide.LEFT,
    forward_speed: float = 0.15,
    entry_threshold: float = 0.7,
    exit_threshold: float = 0.3,
) -> Sequential:
    """Drive forward across a black line, measure skew, and correct with a turn.

    The robot drives forward at ``forward_speed`` while a single IR sensor
    watches for a black line.  As it crosses, the apparent width of the line
    is measured via odometry and compared against the known ``line_width_cm``
    to compute the angular displacement using
    ``acos(line_width / apparent_crossing_width)``.  A corrective turn is then
    executed in the direction given by ``correction_side``.

    Because a single sensor cannot determine which way the robot is skewed,
    the caller must know (from the arena layout or prior steps) whether to
    correct left or right.

    Prerequisites:
        One IR line sensor and odometry.  No lateral-drive capability required.

    Args:
        sensor: The IR sensor instance used to detect the line.
        line_width_cm: Known physical width of the line in centimeters.
        correction_side: Direction to turn for correction (LEFT or RIGHT).
        forward_speed: Approach speed in m/s.  Always driven forward
            (the absolute value is used).
        entry_threshold: Black confidence (0--1) for detecting the leading
            edge of the line.
        exit_threshold: Black confidence (0--1) for detecting the trailing
            edge.  Should be lower than ``entry_threshold``.

    Returns:
        Sequential: crossing measurement + corrective turn.

    Example::

        from libstp.step.motion.lineup.single import (
            forward_single_lineup, CorrectionSide,
        )

        step = forward_single_lineup(
            sensor=robot.center_line_sensor,
            line_width_cm=5.0,
            correction_side=CorrectionSide.LEFT,
            forward_speed=0.10,
        )
        step.run(robot)
    """
    return single_sensor_lineup(
        sensor=sensor,
        line_width_cm=line_width_cm,
        correction_side=correction_side,
        forward_speed=abs(forward_speed),
        entry_threshold=entry_threshold,
        exit_threshold=exit_threshold,
    )


@dsl(tags=["motion", "lineup"])
def backward_single_lineup(
    sensor: IRSensor,
    line_width_cm: float = 5.0,
    correction_side: CorrectionSide = CorrectionSide.LEFT,
    forward_speed: float = 0.15,
    entry_threshold: float = 0.7,
    exit_threshold: float = 0.3,
) -> Sequential:
    """Drive backward across a black line, measure skew, and correct with a turn.

    Identical to ``forward_single_lineup`` but the robot reverses into the
    line instead of driving forward.  The ``forward_speed`` value is negated
    internally so the robot always moves backward regardless of the sign
    passed in.  The same ``acos(line_width / apparent_crossing_width)``
    computation is used to determine the corrective angle.

    Prerequisites:
        One IR line sensor and odometry.  No lateral-drive capability required.

    Args:
        sensor: The IR sensor instance used to detect the line.
        line_width_cm: Known physical width of the line in centimeters.
        correction_side: Direction to turn for correction (LEFT or RIGHT).
        forward_speed: Approach speed in m/s.  The absolute value is negated
            so the robot always drives backward.
        entry_threshold: Black confidence (0--1) for detecting the leading
            edge of the line.
        exit_threshold: Black confidence (0--1) for detecting the trailing
            edge.  Should be lower than ``entry_threshold``.

    Returns:
        Sequential: crossing measurement (backward) + corrective turn.

    Example::

        from libstp.step.motion.lineup.single import (
            backward_single_lineup, CorrectionSide,
        )

        step = backward_single_lineup(
            sensor=robot.center_line_sensor,
            line_width_cm=5.0,
            correction_side=CorrectionSide.RIGHT,
            forward_speed=0.12,
        )
        step.run(robot)
    """
    return single_sensor_lineup(
        sensor=sensor,
        line_width_cm=line_width_cm,
        correction_side=correction_side,
        forward_speed=-abs(forward_speed),
        entry_threshold=entry_threshold,
        exit_threshold=exit_threshold,
    )
