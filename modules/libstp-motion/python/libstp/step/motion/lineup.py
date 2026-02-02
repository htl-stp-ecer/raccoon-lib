"""
Line up on black or white lines using two IR sensors.

This module provides steps for aligning the robot on lines using differential
wheel control based on sensor feedback.
"""
import asyncio
import time
from dataclasses import dataclass
from libstp.foundation import ChassisVelocity, PIDController
from libstp.foundation import info
from libstp.sensor_ir import IRSensor
from libstp.step import Sequential, seq
from typing import TYPE_CHECKING

from .drive_until import SurfaceColor, drive_until_black, drive_until_white
from .. import Step, SimulationStep, SimulationStepDelta, dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dataclass
class LineUpConfig:
    """Configuration for LineUp step."""
    left_sensor: IRSensor
    right_sensor: IRSensor
    base_speed: float = 0.3  # m/s base forward/backward speed
    confidence_threshold: float = 0.7  # stop when both sensors exceed this
    dead_zone: float = 0.1  # minimum confidence error to act
    target: SurfaceColor = SurfaceColor.BLACK
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0

    # Gradual back-off parameters
    initial_speed_mult: float = 1.0  # approach at base_speed (was 1.5, caused overshoot)
    min_speed_mult: float = 0.5  # minimum speed multiplier near target
    backoff_start: float = 0.02  # start slowing at first hint of line
    backoff_curve: float = 0.5  # aggressive early drop (< 1 = drop fast, > 1 = stay fast longer)

    # Stall detection parameters
    no_progress_eps: float = 0.004  # confidence delta that counts as "no movement"
    stall_delay_s: float = 0.35  # how long to allow no progress before boost
    boost_time_s: float = 0.18  # duration of boost kick
    boost_duty: float = 0.85  # boost speed as fraction of max


@dsl(hidden=True)
class LineUp(Step):
    """
    Align the robot on a line using two IR sensors with PID control.

    Drives forward (or backward if base_speed < 0) using differential wheel
    speeds based on individual sensor confidences. Each wheel slows as its
    corresponding sensor approaches the target confidence.

    Includes stall detection that applies a boost kick if no progress is detected.
    """

    def __init__(self, config: LineUpConfig):
        super().__init__()
        self.config = config

    def _generate_signature(self) -> str:
        return (
            f"LineUp(target={self.config.target.value}, "
            f"speed={self.config.base_speed:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        # Estimate small forward movement to reach line
        base.delta = SimulationStepDelta(
            forward=0.05 if self.config.base_speed > 0 else -0.05,
            strafe=0.0,
            angular=0.0,
        )
        return base

    def _get_confidences(self) -> tuple[float, float]:
        """Get left and right sensor confidences for the target color."""
        if self.config.target == SurfaceColor.WHITE:
            left_conf = self.config.left_sensor.probabilityOfWhite()
            right_conf = self.config.right_sensor.probabilityOfWhite()
        else:
            left_conf = self.config.left_sensor.probabilityOfBlack()
            right_conf = self.config.right_sensor.probabilityOfBlack()

        return left_conf, right_conf

    def _is_aligned(self, left_conf: float, right_conf: float) -> bool:
        """Check if both sensors are at the edge (near confidence_threshold)."""
        return (abs(left_conf - self.config.confidence_threshold) <= self.config.dead_zone and
                abs(right_conf - self.config.confidence_threshold) <= self.config.dead_zone)

    def _get_speed_multiplier(self, left_conf: float, right_conf: float) -> float:
        """
        Calculate speed multiplier based on proximity to target.

        Returns a value between min_speed_mult and initial_speed_mult.
        - Below backoff_start -> initial_speed_mult (full speed)
        - At/above confidence_threshold -> min_speed_mult (slow/precise)
        - In between -> gradual curve

        Uses the MAXIMUM confidence (leading sensor) so we slow down
        as soon as one sensor starts seeing the line.
        """
        cfg = self.config
        max_conf = max(left_conf, right_conf)

        # Below backoff_start -> full speed
        if max_conf < cfg.backoff_start:
            return cfg.initial_speed_mult

        # At or above threshold -> minimum speed
        if max_conf >= (cfg.confidence_threshold - cfg.dead_zone*2):
            return cfg.min_speed_mult

        # Map [backoff_start, confidence_threshold] -> [0, 1]
        range_size = cfg.confidence_threshold - cfg.backoff_start
        proximity = (max_conf - cfg.backoff_start) / range_size

        # Apply exponential curve
        decay = proximity ** cfg.backoff_curve
        mult_range = cfg.initial_speed_mult - cfg.min_speed_mult
        return cfg.initial_speed_mult - (decay * mult_range)

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Execute the lineup step."""
        # Initialize PID controllers for each side
        left_pid = PIDController(self.config.kp, self.config.ki, self.config.kd)
        right_pid = PIDController(self.config.kp, self.config.ki, self.config.kd)

        # Stall detection state
        last_left_conf: float | None = None
        last_right_conf: float | None = None
        stalled_since: float | None = None

        update_rate = 1 / 20  # 20 Hz
        last_time = asyncio.get_event_loop().time() - update_rate

        while True:
            # print current confidences

            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            left_conf, right_conf = self._get_confidences()
            speed_mult = self._get_speed_multiplier(left_conf, right_conf)
            info(f"LineUp: L={left_conf:.2f} R={right_conf:.2f} mult={speed_mult:.2f}")
            # Check if aligned
            if self._is_aligned(left_conf, right_conf):
                robot.drive.hard_stop()
                return

            now = time.monotonic()

            # Stall detection
            # if last_left_conf is not None and last_right_conf is not None:
            #     no_change = (
            #         abs(left_conf - last_left_conf) < self.config.no_progress_eps and
            #         abs(right_conf - last_right_conf) < self.config.no_progress_eps
            #     )
            #     if no_change:
            #         stalled_since = stalled_since or now
            #     else:
            #         stalled_since = None

            last_left_conf = left_conf
            last_right_conf = right_conf

            # Apply boost kick if stalled
            # if stalled_since is not None and now - stalled_since > self.config.stall_delay_s:
            #     if now - stalled_since < self.config.stall_delay_s + self.config.boost_time_s:
            #         # Apply straight boost
            #         boost_speed = self.config.boost_duty if self.config.base_speed >= 0 else -self.config.boost_duty
            #         velocity = ChassisVelocity(boost_speed, 0.0, 0.0)
            #         robot.drive.set_velocity(velocity)
            #         robot.drive.update(delta_time)
            #         await asyncio.sleep(update_rate)
            #         continue
            #     else:
            #         stalled_since = None  # Reset after kick

            # Error: positive = need to move toward line, negative = need to back off
            left_error = self.config.confidence_threshold - left_conf
            right_error = self.config.confidence_threshold - right_conf

            left_output = left_pid.calculate(left_error)
            right_output = right_pid.calculate(right_error)

            effective_speed = self.config.base_speed * speed_mult

            # For lineup: we want ROTATION to bring lagging sensor to edge
            # Only add forward/backward when BOTH sensors need to move same direction
            # (both too low = drive forward, both too high = back up)

            # Rotation: difference between outputs
            wz = (right_output - left_output) * effective_speed * 2.0

            # Forward/backward: only when both errors have same sign
            # This prevents driving forward while one sensor is already past the edge
            if left_error * right_error > 0:
                # Both need same direction - use the smaller magnitude (conservative)
                vx = min(abs(left_output), abs(right_output)) * effective_speed
                if left_error < 0:  # both past edge, back up
                    vx = -vx
            else:
                # Sensors on opposite sides of target - pure rotation, no forward/back
                vx = 0.0

            velocity = ChassisVelocity(vx, 0.0, wz)
            robot.drive.set_velocity(velocity)
            robot.drive.update(delta_time)

            await asyncio.sleep(update_rate)


@dsl(hidden=True)
def lineup(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        base_speed: float = 0.3,
        confidence_threshold: float = 0.7,
        target: SurfaceColor = SurfaceColor.BLACK,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> LineUp:
    """
    Create a LineUp step to align on a line.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        base_speed: Base speed in m/s (positive = forward, negative = backward)
        confidence_threshold: Stop when both sensors exceed this confidence
        target: Target surface color (BLACK or WHITE)
        kp: Proportional gain for PID
        ki: Integral gain for PID
        kd: Derivative gain for PID

    Returns:
        LineUp step configured for alignment
    """
    config = LineUpConfig(
        left_sensor=left_sensor,
        right_sensor=right_sensor,
        base_speed=base_speed,
        confidence_threshold=confidence_threshold,
        target=target,
        kp=kp,
        ki=ki,
        kd=kd,
    )
    return LineUp(config)


@dsl(tags=["motion", "lineup"])
def forward_lineup_on_black(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        base_speed: float = 0.3,
        confidence_threshold: float = 0.7,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> Sequential:
    """
    Drive forward past white, then line up on black.

    First drives until one sensor sees white (off the line), then
    performs the lineup on black.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        base_speed: Forward speed in m/s (must be positive)
        confidence_threshold: Stop when confidence exceeds this
        kp, ki, kd: PID gains

    Returns:
        Sequential step: drive_until_white -> lineup_on_black
    """
    return seq([
        drive_until_white([left_sensor, right_sensor], abs(base_speed)),
        lineup(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            base_speed=abs(base_speed),
            confidence_threshold=confidence_threshold,
            target=SurfaceColor.BLACK,
            kp=kp, ki=ki, kd=kd,
        ),
    ])


@dsl(tags=["motion", "lineup"])
def forward_lineup_on_white(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        base_speed: float = 0.3,
        confidence_threshold: float = 0.7,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> Sequential:
    """
    Drive forward past black, then line up on white.

    First drives until one sensor sees black (on the line), then
    performs the lineup on white.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        base_speed: Forward speed in m/s (must be positive)
        confidence_threshold: Stop when confidence exceeds this
        kp, ki, kd: PID gains

    Returns:
        Sequential step: drive_until_black -> lineup_on_white
    """
    return seq([
        drive_until_black([left_sensor, right_sensor], abs(base_speed)),
        lineup(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            base_speed=abs(base_speed),
            confidence_threshold=confidence_threshold,
            target=SurfaceColor.WHITE,
            kp=kp, ki=ki, kd=kd,
        ),
    ])


@dsl(tags=["motion", "lineup"])
def backward_lineup_on_black(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        base_speed: float = 0.3,
        confidence_threshold: float = 0.7,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> Sequential:
    """
    Drive backward past white, then line up on black.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        base_speed: Backward speed in m/s (will be negated)
        confidence_threshold: Stop when confidence exceeds this
        kp, ki, kd: PID gains

    Returns:
        Sequential step: drive_until_white (backward) -> lineup_on_black (backward)
    """
    return seq([
        drive_until_white([left_sensor, right_sensor], -abs(base_speed)),
        lineup(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            base_speed=-abs(base_speed),
            confidence_threshold=confidence_threshold,
            target=SurfaceColor.BLACK,
            kp=kp, ki=ki, kd=kd,
        ),
    ])


@dsl(tags=["motion", "lineup"])
def backward_lineup_on_white(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        base_speed: float = 0.3,
        confidence_threshold: float = 0.7,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> Sequential:
    """
    Drive backward past black, then line up on white.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        base_speed: Backward speed in m/s (will be negated)
        confidence_threshold: Stop when confidence exceeds this
        kp, ki, kd: PID gains

    Returns:
        Sequential step: drive_until_black (backward) -> lineup_on_white (backward)
    """
    return seq([
        drive_until_black([left_sensor, right_sensor], -abs(base_speed)),
        lineup(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            base_speed=-abs(base_speed),
            confidence_threshold=confidence_threshold,
            target=SurfaceColor.WHITE,
            kp=kp, ki=ki, kd=kd,
        ),
    ])
