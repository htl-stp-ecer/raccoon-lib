"""
Max angular velocity measurement step.

Spins the robot at full power and uses the fused IMU orientation
(via odometry) to measure the peak angular velocity achievable.
"""
import asyncio
import math
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity

from .. import Step, dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class MeasureMaxAngularVelocity(Step):
    """
    Command full turning power and measure peak angular velocity.

    Sets all motors to maximum turning speed for a given duration,
    derives angular velocity from the fused IMU heading (odometry),
    and logs peak and average steady-state angular velocity.
    """

    def __init__(self, duration: float = 3.0, clockwise: bool = True):
        super().__init__()
        self.duration = duration
        self.clockwise = clockwise

    def _generate_signature(self) -> str:
        direction = "CW" if self.clockwise else "CCW"
        return f"MeasureMaxAngularVelocity(duration={self.duration:.1f}, dir={direction})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        # Use a very large omega so Drive clamps to its max_omega limit,
        # which pushes the kinematics to request maximum wheel speeds.
        sign = -1.0 if self.clockwise else 1.0
        max_wz = sign * 999.0  # will be clamped to chassis max_omega

        robot.odometry.reset()
        robot.drive.set_velocity(ChassisVelocity(0.0, 0.0, max_wz))

        update_rate = 1 / 100  # 100 Hz for accurate measurement
        last_time = asyncio.get_event_loop().time() - update_rate
        start_time = asyncio.get_event_loop().time()

        # EMA-filtered angular velocity from heading differentiation
        ema_alpha = 0.15
        ema_rate = 0.0
        prev_heading = 0.0

        peak_rate = 0.0
        rate_sum = 0.0
        samples = 0

        spin_up_time = 0.5  # seconds to ignore while motors accelerate

        self.info("Starting max angular velocity measurement "
                  f"({'CW' if self.clockwise else 'CCW'}, {self.duration:.1f}s)...")

        while (asyncio.get_event_loop().time() - start_time) < self.duration:
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            robot.odometry.update(delta_time)
            robot.drive.update(delta_time)

            # Angular velocity from fused IMU heading (filtered, reliable)
            heading = robot.odometry.get_heading()
            heading_diff = heading - prev_heading
            # Unwrap angle discontinuity at +/-pi
            if heading_diff > math.pi:
                heading_diff -= 2 * math.pi
            elif heading_diff < -math.pi:
                heading_diff += 2 * math.pi
            instant_rate = abs(heading_diff / delta_time)
            prev_heading = heading

            # EMA filter to smooth out noise
            ema_rate = ema_alpha * instant_rate + (1.0 - ema_alpha) * ema_rate

            elapsed = current_time - start_time
            if elapsed > spin_up_time:
                if ema_rate > peak_rate:
                    peak_rate = ema_rate
                rate_sum += ema_rate
                samples += 1

            await asyncio.sleep(update_rate)

        robot.drive.hard_stop()

        avg_rate = rate_sum / samples if samples > 0 else 0.0

        self.info("=" * 50)
        self.info("MAX ANGULAR VELOCITY RESULTS")
        self.info("=" * 50)
        self.info(f"  Peak:     {peak_rate:.3f} rad/s "
                  f"({math.degrees(peak_rate):.1f} deg/s)")
        self.info(f"  Average:  {avg_rate:.3f} rad/s "
                  f"({math.degrees(avg_rate):.1f} deg/s)")
        self.info(f"  Samples:  {samples}")
        self.info("=" * 50)


@dsl(tags=["motion", "calibration"])
def measure_max_angular_velocity(
    duration: float = 3.0,
    clockwise: bool = True,
) -> MeasureMaxAngularVelocity:
    """
    Spin the robot at full motor power and measure peak angular velocity.

    Args:
        duration: How long to spin in seconds (default 3.0)
        clockwise: Spin direction (default True = clockwise)

    Returns:
        Step that measures and logs the max angular velocity
    """
    return MeasureMaxAngularVelocity(duration=duration, clockwise=clockwise)
