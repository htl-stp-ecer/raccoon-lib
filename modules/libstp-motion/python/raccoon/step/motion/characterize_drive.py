"""
Characterize drive limits by commanding raw motor power and measuring response.

Bypasses ALL velocity control (library PID and firmware BEMF PID) to discover
the robot's true physical limits: max velocity, acceleration, and deceleration
for each axis.  Motors are driven at 100% PWM via the kinematics layer's
``applyPowerCommand``, which uses inverse kinematics to compute per-wheel
direction signs while commanding raw open-loop power.

Results are persisted to raccoon.project.yml under robot.motion_pid and applied
in-memory.
"""

from __future__ import annotations

import asyncio
import statistics
from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar

from raccoon.foundation import ChassisVelocity
from raccoon.project_yaml import find_project_root, update_project_value

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# EMA filter coefficient for velocity estimation from position finite-difference
_VEL_EMA_ALPHA = 0.3

# Plateau detection: derivative below this for N consecutive cycles
_PLATEAU_DERIV_THRESHOLD = 0.005  # m/s per cycle
_PLATEAU_HOLD_CYCLES = 20

# Stopped threshold
_STOPPED_THRESHOLD_MPS = 0.005

# Ramp-up region: 10%-90% of max velocity for linear regression
_RAMP_LOW_FRAC = 0.10
_RAMP_HIGH_FRAC = 0.90

# Direction vectors for each axis (only ratios matter)
_AXIS_DIRECTION = {
    "forward": ChassisVelocity(1.0, 0.0, 0.0),
    "lateral": ChassisVelocity(0.0, 1.0, 0.0),
    "angular": ChassisVelocity(0.0, 0.0, 1.0),
}


@dataclass
class AxisResult:
    """Measured limits for a single axis."""

    max_velocity: float = 0.0
    acceleration: float = 0.0
    deceleration: float = 0.0


@dataclass
class _Sample:
    time: float
    position: float
    velocity: float = 0.0


def _compute_velocities(samples: list[_Sample]) -> None:
    """Compute EMA-filtered velocity from position finite differences."""
    if len(samples) < 2:
        return
    # Seed from first finite difference so decel phase starts at actual velocity
    dt0 = samples[1].time - samples[0].time
    seed_vel = (samples[1].position - samples[0].position) / dt0 if dt0 > 1e-06 else 0.0
    samples[0].velocity = seed_vel
    prev_vel = seed_vel
    for i in range(1, len(samples)):
        dt = samples[i].time - samples[i - 1].time
        if dt < 1e-6:
            samples[i].velocity = prev_vel
            continue
        raw_vel = (samples[i].position - samples[i - 1].position) / dt
        filtered = _VEL_EMA_ALPHA * raw_vel + (1 - _VEL_EMA_ALPHA) * prev_vel
        samples[i].velocity = filtered
        prev_vel = filtered


def _find_plateau(samples: list[_Sample]) -> int | None:
    """Find the index where velocity plateaus. Returns None if no plateau."""
    if len(samples) < _PLATEAU_HOLD_CYCLES + 2:
        return None
    hold = 0
    for i in range(2, len(samples)):
        dt = samples[i].time - samples[i - 1].time
        if dt < 1e-6:
            continue
        deriv = abs(samples[i].velocity - samples[i - 1].velocity) / dt
        if deriv < _PLATEAU_DERIV_THRESHOLD:
            hold += 1
            if hold >= _PLATEAU_HOLD_CYCLES:
                return i - _PLATEAU_HOLD_CYCLES + 1
        else:
            hold = 0
    return None


def _analyze_max_velocity(samples: list[_Sample], plateau_start: int) -> float:
    """Median velocity in the plateau region."""
    plateau_vels = [abs(s.velocity) for s in samples[plateau_start:]]
    if not plateau_vels:
        return 0.0
    return statistics.median(plateau_vels)


def _analyze_accel(samples: list[_Sample], max_vel: float) -> float:
    """Acceleration = velocity delta / time delta between 10% and 90% crossings."""
    low = max_vel * _RAMP_LOW_FRAC
    high = max_vel * _RAMP_HIGH_FRAC

    t_low: float | None = None
    t_high: float | None = None

    for s in samples:
        v = abs(s.velocity)
        if t_low is None and v >= low:
            t_low = s.time
        if t_low is not None and t_high is None and v >= high:
            t_high = s.time
            break

    if t_low is None or t_high is None or t_high <= t_low:
        return 0.0

    return (high - low) / (t_high - t_low)


def _analyze_decel(samples: list[_Sample]) -> float:
    """Deceleration = velocity delta / time delta between 90% and 10% crossings."""
    if len(samples) < 3:
        return 0.0
    peak_vel = max(abs(s.velocity) for s in samples)
    if peak_vel < _STOPPED_THRESHOLD_MPS:
        return 0.0

    high = peak_vel * _RAMP_HIGH_FRAC
    low = peak_vel * _RAMP_LOW_FRAC

    t_high: float | None = None
    t_low: float | None = None

    for s in samples:
        v = abs(s.velocity)
        # During coast-down, velocity drops: find first crossing below 90%, then below 10%
        if t_high is None and v <= high:
            t_high = s.time
        if t_high is not None and t_low is None and v <= low:
            t_low = s.time
            break

    if t_high is None or t_low is None or t_low <= t_high:
        return 0.0

    return (high - low) / (t_low - t_high)


@dsl_step(tags=["motion", "calibration", "characterize"])
class CharacterizeDrive(Step):
    """Characterize the robot's physical drive limits at full motor power.

    Drives each axis at 100 %% raw PWM (open-loop ``setSpeed``) via the
    kinematics layer, completely bypassing both the library velocity PID and
    the firmware BEMF PID.  This reveals the true hardware ceiling for each
    degree of freedom.

    For each axis the step runs multiple independent trials consisting of two
    phases:

    1. **Acceleration phase** -- commands 100 %% power and records odometry at
       ~100 Hz until a velocity plateau is detected or the timeout expires.
       Max velocity and acceleration are extracted with 10 %%--90 %% rise-time
       analysis.

    2. **Deceleration phase** -- ramps back to full speed, then cuts power and
       records the coast-down.  Deceleration is computed from 90 %%--10 %%
       fall-time analysis.

    The median of all valid trials for each metric is taken as the final
    result.  Measured values are applied to the in-memory
    ``motion_pid_config`` immediately and, if ``persist`` is enabled, written
    to ``raccoon.project.yml`` under ``robot.motion_pid``.

    This step is intended for initial robot setup and should be run on a flat
    surface with enough room for the robot to accelerate to full speed.

    Args:
        axes: Which axes to characterize. Options are ``"forward"``,
            ``"lateral"``, and ``"angular"``. Default ``["forward"]``.
        trials: Number of trials per axis. The median is used for
            robustness against outliers. Default 3.
        power_percent: Motor power percentage (1--100). Default 100
            for true maximum characterization.
        accel_timeout: Maximum time in seconds to wait for the acceleration
            phase before giving up. Default 3.0.
        decel_timeout: Maximum time in seconds to record the deceleration
            (coast-down) phase. Default 3.0.
        persist: If ``True``, write the measured limits to
            ``raccoon.project.yml`` under ``robot.motion_pid``. Default
            ``True``.

    Example::

        from raccoon.step.motion import characterize_drive

        # Characterize forward axis at full power
        characterize_drive()

        # Characterize forward and angular axes with 5 trials each
        characterize_drive(
            axes=["forward", "angular"],
            trials=5,
        )

        # Characterize at 80% power without saving to disk (dry run)
        characterize_drive(
            axes=["forward", "lateral", "angular"],
            power_percent=80,
            persist=False,
        )
    """

    def __init__(
        self,
        axes: list[str] | None = None,
        trials: int = 3,
        power_percent: int = 100,
        accel_timeout: float = 3.0,
        decel_timeout: float = 3.0,
        persist: bool = True,
    ):
        super().__init__()
        if axes is None:
            axes = ["forward"]
        self.axes = axes
        self.trials = max(1, trials)
        self.power_percent = max(1, min(100, power_percent))
        self.accel_timeout = accel_timeout
        self.decel_timeout = decel_timeout
        self.persist = persist
        self.results: dict[str, AxisResult] = {}

    def _generate_signature(self) -> str:
        return (
            f"CharacterizeDrive(axes={self.axes}, "
            f"trials={self.trials}, power={self.power_percent}%, "
            f"persist={self.persist})"
        )

    def _get_position(self, robot: "GenericRobot", axis: str) -> float:
        if axis == "angular":
            return robot.odometry.get_heading()
        dist = robot.odometry.get_distance_from_origin()
        if axis == "forward":
            return dist.forward
        return dist.lateral

    def _command_power(self, robot: "GenericRobot", axis: str, power: int) -> None:
        """Command raw motor power for the given axis via kinematics."""
        direction = _AXIS_DIRECTION[axis]
        robot.drive.apply_power_command(direction, power)

    def _stop_motors(self, robot: "GenericRobot") -> None:
        """Brake all drive motors."""
        for motor in robot.drive.get_motors():
            motor.brake()

    async def _record_accel_samples(
        self,
        robot: "GenericRobot",
        axis: str,
        timeout: float,
    ) -> list[_Sample]:
        """Command full power and record position at ~100 Hz until plateau or timeout."""
        rate = 1 / 100
        samples: list[_Sample] = []

        self._command_power(robot, axis, self.power_percent)

        loop = asyncio.get_event_loop()
        t0 = loop.time()
        last = t0 - rate

        plateau_hold = 0
        prev_vel = 0.0

        while True:
            now = loop.time()
            elapsed = now - t0
            if elapsed > timeout:
                break

            dt = now - last
            last = now
            if dt < 1e-4:
                await asyncio.sleep(rate)
                continue

            robot.odometry.update(dt)

            pos = self._get_position(robot, axis)
            samples.append(_Sample(time=elapsed, position=pos))

            # Early plateau detection
            if len(samples) >= 2:
                raw_vel = (samples[-1].position - samples[-2].position) / dt
                filtered = _VEL_EMA_ALPHA * raw_vel + (1 - _VEL_EMA_ALPHA) * prev_vel
                prev_vel = filtered
                samples[-1].velocity = filtered

                if len(samples) >= 3:
                    vel_deriv = abs(filtered - samples[-2].velocity) / dt
                    if vel_deriv < _PLATEAU_DERIV_THRESHOLD:
                        plateau_hold += 1
                        if plateau_hold >= _PLATEAU_HOLD_CYCLES:
                            break
                    else:
                        plateau_hold = 0

            await asyncio.sleep(rate)

        return samples

    async def _record_decel_samples(
        self,
        robot: "GenericRobot",
        axis: str,
        timeout: float,
    ) -> list[_Sample]:
        """Cut power and record coast-down at ~100 Hz."""
        rate = 1 / 100
        samples: list[_Sample] = []

        # Cut power — brake to get active deceleration
        self._stop_motors(robot)

        loop = asyncio.get_event_loop()
        t0 = loop.time()
        last = t0 - rate

        while True:
            now = loop.time()
            elapsed = now - t0
            if elapsed > timeout:
                break

            dt = now - last
            last = now
            if dt < 1e-4:
                await asyncio.sleep(rate)
                continue

            robot.odometry.update(dt)

            pos = self._get_position(robot, axis)
            samples.append(_Sample(time=elapsed, position=pos))

            await asyncio.sleep(rate)

        return samples

    async def _run_single_trial(self, robot: "GenericRobot", axis: str) -> AxisResult:
        """Run one accel + decel trial for a single axis."""
        result = AxisResult()

        # Reset odometry
        robot.odometry.reset()
        await asyncio.sleep(0.05)
        robot.odometry.update(0.05)

        # Phase 1: Acceleration at full power
        accel_samples = await self._record_accel_samples(robot, axis, self.accel_timeout)
        self._stop_motors(robot)

        _compute_velocities(accel_samples)

        if len(accel_samples) < 10:
            self.warn(f"    Too few samples ({len(accel_samples)})")
            return result

        # Find max velocity
        plateau_idx = _find_plateau(accel_samples)
        if plateau_idx is not None:
            result.max_velocity = _analyze_max_velocity(accel_samples, plateau_idx)
        else:
            # No plateau found — use 90th percentile instead of max to
            # reject noise spikes from finite-difference jitter.
            sorted_vels = sorted(abs(s.velocity) for s in accel_samples)
            p90_idx = int(len(sorted_vels) * 0.90)
            result.max_velocity = sorted_vels[min(p90_idx, len(sorted_vels) - 1)]
            self.warn(
                f"    No velocity plateau detected — using p90 "
                f"({result.max_velocity:.4f}) instead of max "
                f"({sorted_vels[-1]:.4f})"
            )

        # Compute acceleration
        result.acceleration = _analyze_accel(accel_samples, result.max_velocity)

        # Brief pause before decel test
        await asyncio.sleep(0.3)

        # Phase 2: Deceleration (coast-down from full speed)
        robot.odometry.reset()
        await asyncio.sleep(0.05)
        robot.odometry.update(0.05)

        # Ramp up to full speed first
        self._command_power(robot, axis, self.power_percent)
        loop = asyncio.get_event_loop()
        ramp_start = loop.time()
        last = ramp_start - 0.01
        while loop.time() - ramp_start < 1.5:
            now = loop.time()
            dt = now - last
            last = now
            if dt < 1e-4:
                await asyncio.sleep(0.01)
                continue
            robot.odometry.update(dt)
            await asyncio.sleep(0.01)

        # Cut power and record coast-down
        decel_samples = await self._record_decel_samples(robot, axis, self.decel_timeout)
        self._stop_motors(robot)

        _compute_velocities(decel_samples)
        result.deceleration = _analyze_decel(decel_samples)

        return result

    async def _run_axis(self, robot: "GenericRobot", axis: str) -> AxisResult:
        """Run multiple trials for an axis and return the median result."""
        is_angular = axis == "angular"
        unit = "rad/s" if is_angular else "m/s"
        accel_unit = "rad/s²" if is_angular else "m/s²"

        trial_results: list[AxisResult] = []

        for trial_num in range(1, self.trials + 1):
            self.info(
                f"  [{axis}] Trial {trial_num}/{self.trials} " f"(power={self.power_percent}%)"
            )
            result = await self._run_single_trial(robot, axis)
            trial_results.append(result)

            self.info(
                f"    max_vel={result.max_velocity:.4f} {unit}, "
                f"accel={result.acceleration:.4f} {accel_unit}, "
                f"decel={result.deceleration:.4f} {accel_unit}"
            )

            # Settle between trials
            self._stop_motors(robot)
            await asyncio.sleep(0.5)

        # Filter out zero results (failed trials)
        valid = [r for r in trial_results if r.max_velocity > 0]
        if not valid:
            self.warn(f"  [{axis}] All {self.trials} trials failed")
            return AxisResult()

        # Take median of each metric independently
        final = AxisResult(
            max_velocity=statistics.median(r.max_velocity for r in valid),
            acceleration=statistics.median(r.acceleration for r in valid),
            deceleration=statistics.median(r.deceleration for r in valid),
        )

        if len(valid) >= 2:
            # Report spread
            vel_spread = max(r.max_velocity for r in valid) - min(r.max_velocity for r in valid)
            self.info(
                f"  [{axis}] Median of {len(valid)} trials " f"(vel spread={vel_spread:.4f} {unit})"
            )

        return final

    # Mapping from axis name to the AxisConstraints attribute name on
    # UnifiedMotionPidConfig (for in-memory and YAML persistence).
    _AXIS_ATTR: ClassVar[dict[str, str]] = {
        "forward": "linear",
        "lateral": "lateral",
        "angular": "angular",
    }

    def _persist_to_yaml(self) -> bool:
        """Write per-axis results to robot.motion_pid, following !include refs."""
        project_root = find_project_root()
        if project_root is None:
            return False

        updated = False
        for axis, result in self.results.items():
            attr = self._AXIS_ATTR.get(axis)
            if attr is None:
                continue
            base = ("robot", "motion_pid", attr)
            if result.acceleration > 0:
                update_project_value(
                    project_root, [*base, "acceleration"], round(result.acceleration, 4)
                )
                updated = True
            if result.deceleration > 0:
                update_project_value(
                    project_root, [*base, "deceleration"], round(result.deceleration, 4)
                )
                updated = True
            if result.max_velocity > 0:
                update_project_value(
                    project_root, [*base, "max_velocity"], round(result.max_velocity, 4)
                )
                updated = True

        return updated

    def _apply_to_config(self, robot: "GenericRobot") -> None:
        """Update in-memory motion_pid_config with measured values."""
        cfg = robot.motion_pid_config
        for axis, result in self.results.items():
            attr = self._AXIS_ATTR.get(axis)
            if attr is None:
                continue
            axis_constraints = getattr(cfg, attr)
            if result.acceleration > 0:
                axis_constraints.acceleration = result.acceleration
            if result.deceleration > 0:
                axis_constraints.deceleration = result.deceleration
            if result.max_velocity > 0:
                axis_constraints.max_velocity = result.max_velocity

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping drive characterization, using stored values")
            return

        self.info("=" * 60)
        self.info("  DRIVE CHARACTERIZATION (raw motor power)")
        self.info(
            f"  Axes: {', '.join(self.axes)}, Trials: {self.trials}, "
            f"Power: {self.power_percent}%"
        )
        self.info("=" * 60)

        for axis in self.axes:
            if axis not in _AXIS_DIRECTION:
                self.warn(f"  Unknown axis '{axis}' — skipping")
                continue
            self.info(f"\n--- {axis.upper()} axis ---")
            result = await self._run_axis(robot, axis)
            self.results[axis] = result

            # Stop and settle between axes
            self._stop_motors(robot)
            await asyncio.sleep(1.0)

        # Summary
        self.info("\n" + "=" * 60)
        self.info("  RESULTS")
        self.info("=" * 60)
        for axis, r in self.results.items():
            if axis == "angular":
                self.info(
                    f"  {axis}: max_rate={r.max_velocity:.4f} rad/s, "
                    f"accel={r.acceleration:.4f} rad/s², "
                    f"decel={r.deceleration:.4f} rad/s²"
                )
            else:
                self.info(
                    f"  {axis}: max_vel={r.max_velocity:.4f} m/s, "
                    f"accel={r.acceleration:.4f} m/s², "
                    f"decel={r.deceleration:.4f} m/s²"
                )

        # Apply to in-memory config
        self._apply_to_config(robot)
        self.info("  Applied to in-memory motion config")

        # Persist
        if self.persist:
            if self._persist_to_yaml():
                self.info("  Saved to raccoon.project.yml (robot.motion_pid)")
            else:
                self.warn("  Failed to save to raccoon.project.yml")

        self.info("=" * 60)
