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

from ..turn import turn_left, turn_right
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
    """
    Drive across a line and measure the apparent crossing width.

    Phase 1 (searching): Drive until sensor reads above ``entry_threshold``
        → record leading edge position.
    Phase 2 (on line): Continue until sensor reads below ``exit_threshold``
        → record trailing edge position.

    After completion, ``apparent_width_m`` holds the measured crossing
    distance and ``crossing_angle_rad`` holds the computed displacement
    angle.
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
                self.info(
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

                self.info(
                    f"Trailing edge at {current * 100:.1f}cm, "
                    f"apparent={self.apparent_width_m * 100:.1f}cm, "
                    f"angle={math.degrees(self.crossing_angle_rad):.1f}deg"
                )
                return True

        return False


def _compute_single_sensor_turn(crossing: SingleSensorCrossing, _robot: "GenericRobot"):
    from libstp.step import run

    angle = crossing.crossing_angle_rad
    if angle < math.radians(1.0):
        return run(lambda _: None)

    degrees = math.degrees(angle)
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
    """
    Compose crossing measurement + corrective turn.

    Returns a Sequential that drives across the line, measures the
    displacement angle, and turns to correct it.
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
    """
    Drive forward across a black line, measure skew, and correct angle.

    Uses the known line width to compute the robot's angular displacement
    from ``acos(line_width / apparent_crossing_width)``.

    Args:
        sensor: The IR sensor instance
        line_width_cm: Known width of the line in centimeters
        correction_side: Which direction to turn for correction (LEFT or RIGHT)
        forward_speed: Approach speed in m/s
        entry_threshold: Black confidence for detecting the leading edge
        exit_threshold: Black confidence for detecting the trailing edge

    Returns:
        Sequential: crossing measurement + corrective turn
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
    """
    Drive backward across a black line, measure skew, and correct angle.

    Same as ``forward_single_lineup`` but approaches the line in reverse.

    Args:
        sensor: The IR sensor instance
        line_width_cm: Known width of the line in centimeters
        correction_side: Which direction to turn for correction (LEFT or RIGHT)
        forward_speed: Approach speed in m/s (will be negated)
        entry_threshold: Black confidence for detecting the leading edge
        exit_threshold: Black confidence for detecting the trailing edge

    Returns:
        Sequential: crossing measurement + corrective turn
    """
    return single_sensor_lineup(
        sensor=sensor,
        line_width_cm=line_width_cm,
        correction_side=correction_side,
        forward_speed=-abs(forward_speed),
        entry_threshold=entry_threshold,
        exit_threshold=exit_threshold,
    )
