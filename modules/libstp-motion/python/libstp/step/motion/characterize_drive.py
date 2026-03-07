"""
Characterize drive limits by commanding raw velocities and measuring response.

Bypasses the profile/PID system to discover the robot's true physical limits:
max velocity, acceleration, and deceleration for each axis. Runs multiple
trials per axis for statistical robustness. Results are persisted to
raccoon.project.yml under robot.motion_pid and applied in-memory.
"""
import asyncio
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import yaml

from libstp.foundation import ChassisVelocity

from .. import Step, dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


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
    if dt0 > 1e-6:
        seed_vel = (samples[1].position - samples[0].position) / dt0
    else:
        seed_vel = 0.0
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


def _find_plateau(samples: list[_Sample]) -> Optional[int]:
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


def _find_project_root() -> Optional[Path]:
    """Find project root by searching upward for raccoon.project.yml."""
    try:
        current = Path.cwd().resolve()
    except (FileNotFoundError, OSError):
        return None
    while current != current.parent:
        if (current / "raccoon.project.yml").exists():
            return current
        current = current.parent
    return None


@dsl(hidden=True)
class CharacterizeDrive(Step):
    """Characterize drive limits by commanding raw velocities.

    For each configured axis, runs multiple acceleration + deceleration
    trials. Acceleration phase records odometry at ~100 Hz until a velocity
    plateau is detected; deceleration phase ramps up then commands zero and
    records coast-down. Max velocity, acceleration, and deceleration are
    extracted via 10%/90% rise/fall-time analysis, and the median across
    trials is taken as the final result. Results are applied in-memory and
    optionally persisted to ``raccoon.project.yml``.
    """

    def __init__(
        self,
        axes: list[str],
        trials: int,
        command_speed: float,
        accel_timeout: float,
        decel_timeout: float,
        persist: bool,
    ):
        super().__init__()
        self.axes = axes
        self.trials = trials
        self.command_speed = command_speed
        self.accel_timeout = accel_timeout
        self.decel_timeout = decel_timeout
        self.persist = persist
        self.results: dict[str, AxisResult] = {}

    def _generate_signature(self) -> str:
        return (
            f"CharacterizeDrive(axes={self.axes}, "
            f"trials={self.trials}, persist={self.persist})"
        )

    def _make_velocity(self, axis: str, speed: float) -> ChassisVelocity:
        if axis == "forward":
            return ChassisVelocity(speed, 0.0, 0.0)
        elif axis == "lateral":
            return ChassisVelocity(0.0, speed, 0.0)
        elif axis == "angular":
            return ChassisVelocity(0.0, 0.0, speed)
        raise ValueError(f"Unknown axis: {axis}")

    def _get_position(self, robot: "GenericRobot", axis: str) -> float:
        if axis == "angular":
            return robot.odometry.get_heading()
        dist = robot.odometry.get_distance_from_origin()
        if axis == "forward":
            return dist.forward
        return dist.lateral

    async def _record_samples(
        self,
        robot: "GenericRobot",
        axis: str,
        velocity: ChassisVelocity,
        timeout: float,
        stop_on_plateau: bool,
    ) -> list[_Sample]:
        """Command velocity and record position at ~100 Hz."""
        rate = 1 / 100
        samples: list[_Sample] = []

        robot.drive.set_velocity(velocity)

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
            robot.drive.update(dt)

            pos = self._get_position(robot, axis)
            samples.append(_Sample(time=elapsed, position=pos))

            # Early plateau detection for accel phase
            if stop_on_plateau and len(samples) >= 2:
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

    async def _run_single_trial(
        self, robot: "GenericRobot", axis: str
    ) -> AxisResult:
        """Run one accel + decel trial for a single axis."""
        result = AxisResult()
        speed = self.command_speed
        vel_cmd = self._make_velocity(axis, speed)

        # Reset odometry
        robot.odometry.reset()
        await asyncio.sleep(0.05)
        robot.odometry.update(0.05)

        # Phase 1: Acceleration + max velocity
        accel_samples = await self._record_samples(
            robot, axis, vel_cmd, self.accel_timeout, stop_on_plateau=True
        )
        robot.drive.hard_stop()

        _compute_velocities(accel_samples)

        if len(accel_samples) < 10:
            self.warn(f"    Too few samples ({len(accel_samples)})")
            return result

        # Find max velocity
        plateau_idx = _find_plateau(accel_samples)
        if plateau_idx is not None:
            result.max_velocity = _analyze_max_velocity(
                accel_samples, plateau_idx
            )
        else:
            result.max_velocity = max(
                abs(s.velocity) for s in accel_samples
            )

        # Compute acceleration
        result.acceleration = _analyze_accel(accel_samples, result.max_velocity)

        # Brief pause before decel test
        await asyncio.sleep(0.2)

        # Phase 2: Deceleration (coast-down from max velocity)
        robot.odometry.reset()
        await asyncio.sleep(0.05)
        robot.odometry.update(0.05)

        # Ramp up to speed first
        robot.drive.set_velocity(vel_cmd)
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
            robot.drive.update(dt)
            await asyncio.sleep(0.01)

        # Command zero and record coast-down
        zero_vel = ChassisVelocity(0.0, 0.0, 0.0)
        decel_samples = await self._record_samples(
            robot, axis, zero_vel, self.decel_timeout, stop_on_plateau=False
        )
        robot.drive.hard_stop()

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
                f"  [{axis}] Trial {trial_num}/{self.trials} "
                f"(cmd={self.command_speed:.2f} {unit})"
            )
            result = await self._run_single_trial(robot, axis)
            trial_results.append(result)

            self.info(
                f"    max_vel={result.max_velocity:.4f} {unit}, "
                f"accel={result.acceleration:.4f} {accel_unit}, "
                f"decel={result.deceleration:.4f} {accel_unit}"
            )

            # Settle between trials
            robot.drive.hard_stop()
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
                f"  [{axis}] Median of {len(valid)} trials "
                f"(vel spread={vel_spread:.4f} {unit})"
            )

        return final

    # Mapping from axis name to the AxisConstraints attribute name on
    # UnifiedMotionPidConfig (for in-memory and YAML persistence).
    _AXIS_ATTR = {
        "forward": "linear",
        "lateral": "lateral",
        "angular": "angular",
    }

    def _persist_to_yaml(self) -> bool:
        """Write per-axis results to robot.motion_pid in raccoon.project.yml."""
        project_root = _find_project_root()
        if project_root is None:
            return False

        config_path = project_root / "raccoon.project.yml"

        try:
            if config_path.exists():
                with open(config_path, "r", encoding="utf-8") as f:
                    config = yaml.safe_load(f) or {}
            else:
                config = {}
        except (yaml.YAMLError, OSError):
            return False

        if not isinstance(config, dict):
            return False

        robot_cfg = config.setdefault("robot", {})
        if not isinstance(robot_cfg, dict):
            return False
        motion_pid = robot_cfg.setdefault("motion_pid", {})
        if not isinstance(motion_pid, dict):
            return False

        updated = False

        for axis, result in self.results.items():
            attr = self._AXIS_ATTR.get(axis)
            if attr is None:
                continue
            axis_cfg = motion_pid.setdefault(attr, {})
            if not isinstance(axis_cfg, dict):
                continue
            if result.acceleration > 0:
                axis_cfg["acceleration"] = round(result.acceleration, 4)
                updated = True
            if result.deceleration > 0:
                axis_cfg["deceleration"] = round(result.deceleration, 4)
                updated = True
            if result.max_velocity > 0:
                axis_cfg["max_velocity"] = round(result.max_velocity, 4)
                updated = True

        if not updated:
            return False

        try:
            with open(config_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(
                    config, f, sort_keys=False, default_flow_style=False
                )
            return True
        except OSError:
            return False

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
        self.info("=" * 60)
        self.info("  DRIVE CHARACTERIZATION")
        self.info(f"  Axes: {', '.join(self.axes)}, Trials: {self.trials}")
        self.info("=" * 60)

        for axis in self.axes:
            self.info(f"\n--- {axis.upper()} axis ---")
            result = await self._run_axis(robot, axis)
            self.results[axis] = result

            # Stop and settle between axes
            robot.drive.hard_stop()
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


@dsl(tags=["motion", "calibration", "characterize"])
def characterize_drive(
    axes: list[str] | None = None,
    trials: int = 3,
    command_speed: float = 1.0,
    accel_timeout: float = 3.0,
    decel_timeout: float = 3.0,
    persist: bool = True,
) -> CharacterizeDrive:
    """Characterize the robot's physical drive limits by commanding raw velocities.

    Bypasses the profile and PID systems entirely to discover the true hardware
    capabilities of each drive axis. For each axis, the step runs multiple
    independent trials consisting of two phases:

    1. **Acceleration phase** -- commands the specified velocity and records
       odometry at ~100 Hz until a velocity plateau is detected or the timeout
       expires. From this data, max velocity and acceleration are extracted
       using 10%--90% rise-time analysis.

    2. **Deceleration phase** -- ramps the robot back up to speed, then
       commands zero velocity and records the coast-down. Deceleration is
       computed from 90%--10% fall-time analysis.

    The median of all valid trials for each metric is taken as the final
    result. Measured values are applied to the in-memory
    ``motion_pid_config`` immediately and, if ``persist`` is enabled, written
    to the ``robot.motion_pid`` section of ``raccoon.project.yml`` so they
    survive restarts.

    This step is intended for initial robot setup and should be run on a flat
    surface with enough room for the robot to accelerate to full speed. It is
    typically the first phase of the full ``auto_tune()`` pipeline.

    Args:
        axes: Which axes to characterize. Options are ``"forward"``,
            ``"lateral"``, and ``"angular"``. Default ``["forward"]``.
        trials: Number of trials per axis. The median is used for
            robustness against outliers. Default 3.
        command_speed: Raw velocity command magnitude sent to the motors.
            For forward/lateral this is in m/s; for angular it is in rad/s.
            Should be at or near the maximum the robot can sustain.
            Default 1.0.
        accel_timeout: Maximum time in seconds to wait for the acceleration
            phase before giving up. Default 3.0.
        decel_timeout: Maximum time in seconds to record the deceleration
            (coast-down) phase. Default 3.0.
        persist: If ``True``, write the measured limits to
            ``raccoon.project.yml`` under ``robot.motion_pid``. Default
            ``True``.

    Returns:
        A ``CharacterizeDrive`` step that measures and persists drive limits.

    Example::

        from libstp.step.motion import characterize_drive

        # Characterize forward axis with default settings
        step = characterize_drive()

        # Characterize forward and angular axes with 5 trials each
        step = characterize_drive(
            axes=["forward", "angular"],
            trials=5,
        )

        # Characterize without saving to disk (dry run)
        step = characterize_drive(
            axes=["forward", "lateral", "angular"],
            persist=False,
        )
    """
    if axes is None:
        axes = ["forward"]
    return CharacterizeDrive(
        axes=axes,
        trials=max(1, trials),
        command_speed=command_speed,
        accel_timeout=accel_timeout,
        decel_timeout=decel_timeout,
        persist=persist,
    )
