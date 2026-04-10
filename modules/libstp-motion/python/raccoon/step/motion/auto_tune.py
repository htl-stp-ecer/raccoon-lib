"""
Auto-tune PID controllers via system identification and iterative optimization.

Three-phase sequential pipeline:

Phase 1 - Drive characterization:
    Measure physical limits (max velocity, acceleration, deceleration) per axis
    by commanding raw velocities and observing response. Uses the existing
    CharacterizeDrive step. Results are applied in-memory and persisted so that
    subsequent phases have accurate constraints.

Phase 2 - Velocity controller tuning:
    Step-response identification using inflection tangent method (ported from
    Torsten Brischalle's ControlTheory, MIT license). Extracts plant parameters
    (gain Ks, dead time Tu, time constant Tg), then computes PID gains via CHR
    set-point-follow formulas (kp, ki, kd). Validates by comparing ISE
    before/after.

Phase 3 - Motion controller tuning:
    Coordinate descent (Hooke-Jeeves) optimizer that runs real test motions,
    scores settling time + overshoot + final error, and iteratively adjusts
    distance.kp/kd and heading.kp/kd.

Step classes:
    AutoTune             - full pipeline (characterize, velocity, motion)
    AutoTuneVelocity     - phase 2 only
    AutoTuneMotion       - phase 3 only
"""
import asyncio
import csv
import math
import os
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

from raccoon.project_yaml import find_project_root, update_project_value

from raccoon.foundation import (
    ChassisVelocity,
    Feedforward,
    PidGains,
)
from raccoon.drive import (
    AxisVelocityControlConfig,
    ChassisVelocityControlConfig,
)
from raccoon.motion import (
    LinearMotion,
    LinearMotionConfig,
    LinearAxis,
    TurnMotion,
    TurnConfig,
)

from .. import Step
from ..annotation import dsl_step
from .characterize_drive import CharacterizeDrive

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_DEFAULT_CSV_DIR = "/tmp/auto_tune"

# Step response sampling
_STEP_HZ = 100
_STEP_DURATION_S = 2.0
_STEP_COMMAND_FRAC = 0.50  # fraction of max velocity

# Inflection tangent: local regression window (half-width in samples)
_REGRESSION_HALF_WINDOW = 3

# CHR set-point-follow scaling (reduced because kV=1.0 FF handles steady-state)
_CHR_KP_SCALE = 0.3
_CHR_KI_SCALE = 0.6   # ki = scale * kp / Tg  (slow integral to remove steady-state error)
_CHR_KD_SCALE = 0.15

# Coordinate descent
_CD_MAX_ITERATIONS = 10
_CD_INITIAL_DELTA_FRAC = 0.25  # fraction of initial gain
_CD_MIN_DELTA = 0.01
_CD_DELTA_SHRINK = 0.5

# Motion trial scoring weights
_SCORE_SETTLE_WEIGHT = 1.0
_SCORE_OVERSHOOT_WEIGHT = 10.0
_SCORE_ERROR_WEIGHT = 5.0
_SCORE_TIMEOUT_PENALTY = 50.0
_SCORE_CONSTRAINT_BREACH_BASE = 25.0

# Constraint-aware scoring thresholds (fastest is the goal, but only after
# endpoint behavior is kept within reasonable bounds).
_LINEAR_OVERSHOOT_SOFT_M = 0.010
_LINEAR_FINAL_ERROR_SOFT_M = 0.010
_TURN_OVERSHOOT_SOFT_RAD = math.radians(3.0)
_TURN_FINAL_ERROR_SOFT_RAD = math.radians(2.0)

# Penalty slopes for breaching soft limits (large enough to dominate settle time)
_SCORE_LINEAR_OVERSHOOT_BREACH_PER_M = 2000.0
_SCORE_LINEAR_ERROR_BREACH_PER_M = 1000.0
_SCORE_TURN_OVERSHOOT_BREACH_PER_RAD = 400.0
_SCORE_TURN_ERROR_BREACH_PER_RAD = 250.0

# Motion trial parameters
_LINEAR_TEST_DISTANCE_M = 0.50
_TURN_TEST_ANGLE_RAD = math.radians(90)
_MOTION_TIMEOUT_S = 10.0
_MOTION_SETTLE_S = 1.0
# Optimize for fastest command speed; lower speeds are monitored separately.
_MOTION_PRIMARY_SPEED_SCALE = 1.0
_MOTION_WATCH_SPEED_SCALES = (0.5, 0.3)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class PlantParams:
    """Identified plant parameters from step response."""
    Ks: float = 0.0   # static gain (output / input at steady state)
    Tu: float = 0.0   # dead time (s)
    Tg: float = 0.0   # time constant (s)
    method: str = ""   # "inflection" or "rise_time"


@dataclass
class StepResponseData:
    """Raw step response recording."""
    times: list[float] = field(default_factory=list)
    commanded: list[float] = field(default_factory=list)
    measured: list[float] = field(default_factory=list)


@dataclass
class VelocityTuneResult:
    """Result of velocity controller tuning for one axis."""
    axis: str = ""
    plant: PlantParams = field(default_factory=PlantParams)
    pid: PidGains = field(default_factory=PidGains)
    ff: Feedforward = field(default_factory=Feedforward)
    baseline_ise: float = 0.0
    tuned_ise: float = 0.0
    accepted: bool = False


@dataclass
class MotionTuneResult:
    """Result of motion controller tuning for one parameter set."""
    param_name: str = ""  # "distance" or "heading"
    initial_kp: float = 0.0
    initial_kd: float = 0.0
    final_kp: float = 0.0
    final_kd: float = 0.0
    initial_score: float = 0.0
    final_score: float = 0.0
    iterations: int = 0


# ---------------------------------------------------------------------------
# Plant identification (inflection tangent method)
# ---------------------------------------------------------------------------

def _local_regression_derivatives(
    times: list[float], values: list[float], idx: int, half_w: int
) -> tuple[float, float]:
    """
    Estimate first and second derivative at index via local quadratic regression.

    Fits y = a + b*t + c*t^2 over [idx-half_w, idx+half_w], returns (b, 2*c)
    evaluated at t=0 (centered on times[idx]).
    """
    n = len(times)
    lo = max(0, idx - half_w)
    hi = min(n - 1, idx + half_w)
    if hi - lo < 2:
        return 0.0, 0.0

    t0 = times[idx]
    # Build least-squares for quadratic: y = a + b*dt + c*dt^2
    sum_1 = 0.0
    sum_t = 0.0
    sum_t2 = 0.0
    sum_t3 = 0.0
    sum_t4 = 0.0
    sum_y = 0.0
    sum_yt = 0.0
    sum_yt2 = 0.0
    count = 0

    for i in range(lo, hi + 1):
        dt = times[i] - t0
        y = values[i]
        dt2 = dt * dt
        sum_1 += 1.0
        sum_t += dt
        sum_t2 += dt2
        sum_t3 += dt2 * dt
        sum_t4 += dt2 * dt2
        sum_y += y
        sum_yt += y * dt
        sum_yt2 += y * dt2
        count += 1

    if count < 3:
        return 0.0, 0.0

    # Solve 3x3 normal equations: [[n, st, st2], [st, st2, st3], [st2, st3, st4]]
    # We only need b (first deriv) and c (half second deriv)
    # Using Cramer's rule for the 3x3 system
    A = [
        [sum_1, sum_t, sum_t2],
        [sum_t, sum_t2, sum_t3],
        [sum_t2, sum_t3, sum_t4],
    ]
    B = [sum_y, sum_yt, sum_yt2]

    det = (
        A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
        - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
        + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])
    )
    if abs(det) < 1e-20:
        return 0.0, 0.0

    # b = det with column 1 replaced by B / det
    det_b = (
        A[0][0] * (B[1] * A[2][2] - A[1][2] * B[2])
        - B[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
        + A[0][2] * (A[1][0] * B[2] - B[1] * A[2][0])
    )
    # c = det with column 2 replaced by B / det
    det_c = (
        A[0][0] * (A[1][1] * B[2] - B[1] * A[2][1])
        - A[0][1] * (A[1][0] * B[2] - B[1] * A[2][0])
        + B[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])
    )

    b = det_b / det  # first derivative at t0
    c = det_c / det  # half of second derivative
    return b, 2.0 * c


def _identify_plant_inflection(
    times: list[float], measured: list[float], command: float
) -> Optional[PlantParams]:
    """
    Inflection tangent method: find where second derivative changes sign,
    draw tangent at that point, extract Tu and Tg from x-intercepts.
    """
    n = len(times)
    if n < 2 * _REGRESSION_HALF_WINDOW + 3:
        return None

    # Compute derivatives at each interior point
    derivs_1: list[float] = []
    derivs_2: list[float] = []
    indices: list[int] = []

    for i in range(_REGRESSION_HALF_WINDOW, n - _REGRESSION_HALF_WINDOW):
        d1, d2 = _local_regression_derivatives(
            times, measured, i, _REGRESSION_HALF_WINDOW
        )
        derivs_1.append(d1)
        derivs_2.append(d2)
        indices.append(i)

    if len(derivs_2) < 3:
        return None

    # Find inflection: sign change in second derivative
    inflection_idx = None
    inflection_pos = None
    max_d1 = 0.0
    for j in range(1, len(derivs_2)):
        if derivs_2[j - 1] * derivs_2[j] < 0:
            # Pick the side with larger |d1| (steepest slope)
            if abs(derivs_1[j]) > max_d1:
                max_d1 = abs(derivs_1[j])
                inflection_idx = j
                inflection_pos = indices[j]

    if inflection_idx is None or inflection_pos is None:
        return None

    # Tangent line at inflection point: y = y0 + slope * (t - t0)
    t_infl = times[inflection_pos]
    y_infl = measured[inflection_pos]
    slope = derivs_1[inflection_idx]

    if abs(slope) < 1e-10:
        return None

    # Steady-state value (last 10% of samples)
    tail_start = max(0, n - n // 10)
    y_ss = sum(measured[tail_start:]) / max(1, n - tail_start)

    if abs(y_ss) < 1e-10:
        return None

    # Tangent intersects y=0 at: t0 - y0/slope
    t_zero = t_infl - y_infl / slope
    # Tangent intersects y=y_ss at: t0 + (y_ss - y0)/slope
    t_ss = t_infl + (y_ss - y_infl) / slope

    Tu = max(t_zero - times[0], 0.001)  # dead time
    Tg = max(t_ss - t_zero, 0.001)      # time constant
    Ks = y_ss / command if abs(command) > 1e-10 else 0.0

    if Tu <= 0 or Tg <= 0 or Ks <= 0:
        return None

    return PlantParams(Ks=Ks, Tu=Tu, Tg=Tg, method="inflection")


def _identify_plant_rise_time(
    times: list[float], measured: list[float], command: float
) -> PlantParams:
    """
    Fallback: estimate plant from 10%/90% rise time.
    Tu ≈ time to reach 10%, Tg ≈ time from 10% to 63%.
    """
    n = len(times)
    tail_start = max(0, n - n // 10)
    y_ss = sum(measured[tail_start:]) / max(1, n - tail_start)

    if abs(y_ss) < 1e-10:
        return PlantParams(method="rise_time")

    Ks = y_ss / command if abs(command) > 1e-10 else 1.0

    low = abs(y_ss) * 0.10
    high = abs(y_ss) * 0.63  # one time constant

    t_low = None
    t_high = None
    for i in range(n):
        v = abs(measured[i])
        if t_low is None and v >= low:
            t_low = times[i]
        if t_low is not None and t_high is None and v >= high:
            t_high = times[i]
            break

    if t_low is None:
        t_low = times[0]
    if t_high is None:
        t_high = times[-1]

    Tu = max(t_low - times[0], 0.01)
    Tg = max(t_high - t_low, 0.01)

    return PlantParams(Ks=Ks, Tu=Tu, Tg=Tg, method="rise_time")


def _compute_chr_gains(plant: PlantParams) -> PidGains:
    """
    CHR set-point-follow PID gains, scaled down since kV=1.0 feedforward
    handles most of the steady-state. A small ki removes residual error
    from friction / load changes without aggressive windup.
    """
    if plant.Tu <= 0 or plant.Ks <= 0 or plant.Tg <= 0:
        return PidGains(0.0, 0.0, 0.0)

    kp = _CHR_KP_SCALE * plant.Tg / (plant.Ks * plant.Tu)
    ki = _CHR_KI_SCALE * kp / plant.Tg
    kd = _CHR_KD_SCALE * plant.Tg / plant.Ks
    return PidGains(kp, ki, kd)


def _compute_ise(
    times: list[float], commanded: list[float], measured: list[float]
) -> float:
    """Integral of squared error (trapezoidal integration)."""
    if len(times) < 2:
        return float("inf")
    ise = 0.0
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        e0 = commanded[i - 1] - measured[i - 1]
        e1 = commanded[i] - measured[i]
        ise += 0.5 * (e0 * e0 + e1 * e1) * dt
    return ise


# ---------------------------------------------------------------------------
# Step response recording
# ---------------------------------------------------------------------------

async def _run_step_response(
    robot: "GenericRobot",
    axis: str,
    command: float,
    duration: float = _STEP_DURATION_S,
) -> StepResponseData:
    """
    Command a step velocity and record (time, commanded, measured) at 100 Hz.

    Uses robot.drive.set_velocity() to bypass the motion PID layer, and
    robot.drive.estimate_state() to read back actual chassis velocity.
    """
    data = StepResponseData()
    rate = 1.0 / _STEP_HZ

    # Build velocity command
    if axis == "vx":
        vel = ChassisVelocity(command, 0.0, 0.0)
    elif axis == "vy":
        vel = ChassisVelocity(0.0, command, 0.0)
    elif axis == "wz":
        vel = ChassisVelocity(0.0, 0.0, command)
    else:
        raise ValueError(f"Unknown velocity axis: {axis}")

    robot.drive.set_velocity(vel)
    loop = asyncio.get_event_loop()
    t0 = loop.time()
    last = t0 - rate

    while True:
        now = loop.time()
        elapsed = now - t0
        if elapsed > duration:
            break

        dt = now - last
        last = now
        if dt < 1e-4:
            await asyncio.sleep(rate)
            continue

        robot.odometry.update(dt)
        robot.drive.update(dt)

        state = robot.drive.estimate_state()
        if axis == "vx":
            meas = state.vx
        elif axis == "vy":
            meas = state.vy
        else:
            meas = state.wz

        data.times.append(elapsed)
        data.commanded.append(command)
        data.measured.append(meas)

        await asyncio.sleep(rate)

    robot.drive.hard_stop()
    # Brief settle
    await asyncio.sleep(0.3)

    return data


# ---------------------------------------------------------------------------
# CSV output helpers
# ---------------------------------------------------------------------------

def _write_step_csv(
    path: str, data: StepResponseData, label: str = ""
) -> None:
    """Write step response data to CSV."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "commanded", "measured", "label"])
        for i in range(len(data.times)):
            writer.writerow([
                f"{data.times[i]:.4f}",
                f"{data.commanded[i]:.4f}",
                f"{data.measured[i]:.4f}",
                label,
            ])


# ---------------------------------------------------------------------------
# YAML persistence (follows characterize_drive.py pattern)
# ---------------------------------------------------------------------------

def _persist_velocity_config(
    results: dict[str, VelocityTuneResult],
) -> bool:
    """Write velocity control gains, following !include refs."""
    project_root = find_project_root()
    if project_root is None:
        return False

    updated = False
    for axis_name, result in results.items():
        if not result.accepted:
            continue
        base = ["robot", "drive", "vel_config", axis_name]
        update_project_value(project_root, [*base, "pid"], {
            "kp": round(result.pid.kp, 6),
            "ki": round(result.pid.ki, 6),
            "kd": round(result.pid.kd, 6),
        })
        update_project_value(project_root, [*base, "ff"], {
            "kS": round(result.ff.kS, 6),
            "kV": round(result.ff.kV, 6),
            "kA": round(result.ff.kA, 6),
        })
        updated = True

    return updated


def _persist_motion_config(
    results: dict[str, MotionTuneResult],
) -> bool:
    """Write motion PID gains, following !include refs."""
    project_root = find_project_root()
    if project_root is None:
        return False

    updated = False
    for param_name, result in results.items():
        # "lateral" shares the distance PID
        yaml_key = "distance" if param_name == "lateral" else param_name
        base = ["robot", "motion_pid", yaml_key]
        update_project_value(project_root, [*base, "kp"],
                             round(result.final_kp, 4))
        update_project_value(project_root, [*base, "kd"],
                             round(result.final_kd, 4))
        updated = True

    return updated


# ---------------------------------------------------------------------------
# Apply gains in-memory
# ---------------------------------------------------------------------------

def _apply_velocity_gains(
    robot: "GenericRobot", results: dict[str, VelocityTuneResult]
) -> None:
    """Apply tuned velocity controller gains to the drive system.

    Reads the current config first so non-tuned axes keep their gains.
    """
    cfg = robot.drive.get_velocity_control_config()
    for axis_name, result in results.items():
        if not result.accepted:
            continue
        axis_cfg = AxisVelocityControlConfig(result.pid, result.ff)
        if axis_name == "vx":
            cfg.vx = axis_cfg
        elif axis_name == "vy":
            cfg.vy = axis_cfg
        elif axis_name == "wz":
            cfg.wz = axis_cfg
    robot.drive.set_velocity_control_config(cfg)
    robot.drive.reset_velocity_controllers()


def _apply_motion_gains(
    robot: "GenericRobot", results: dict[str, MotionTuneResult]
) -> None:
    """Apply tuned motion PID gains to the in-memory config."""
    cfg = robot.motion_pid_config
    for param_name, result in results.items():
        # "lateral" shares the distance PID
        pid_attr = "distance" if param_name == "lateral" else param_name
        pid_cfg = getattr(cfg, pid_attr, None)
        if pid_cfg is None:
            continue
        pid_cfg.kp = result.final_kp
        pid_cfg.kd = result.final_kd


# ---------------------------------------------------------------------------
# Phase 1: Velocity Controller Tuning
# ---------------------------------------------------------------------------

def _get_max_velocity(robot: "GenericRobot", axis: str) -> float:
    """Get max velocity for an axis from characterization, or fallback."""
    cfg = robot.motion_pid_config
    if axis == "vx":
        v = cfg.linear.max_velocity
        return v if v > 0 else 0.3  # conservative fallback m/s
    elif axis == "vy":
        v = cfg.lateral.max_velocity
        return v if v > 0 else 0.3  # conservative fallback m/s
    elif axis == "wz":
        v = cfg.angular.max_velocity
        return v if v > 0 else 2.0  # conservative fallback rad/s
    return 1.0


async def _tune_velocity_axis(
    robot: "GenericRobot",
    axis: str,
    csv_dir: Optional[str],
    log_fn,
) -> VelocityTuneResult:
    """Tune velocity controller for one axis via step response identification."""
    result = VelocityTuneResult(axis=axis)

    max_vel = _get_max_velocity(robot, axis)
    command = max_vel * _STEP_COMMAND_FRAC
    unit = "rad/s" if axis == "wz" else "m/s"

    log_fn(f"  [{axis}] Step response: cmd={command:.3f} {unit} "
           f"(max={max_vel:.3f}, frac={_STEP_COMMAND_FRAC})")

    # 1. Baseline step response (with current gains, typically kV=1 passthrough)
    baseline = await _run_step_response(robot, axis, command)

    if csv_dir:
        _write_step_csv(
            os.path.join(csv_dir, f"vel_{axis}_baseline.csv"),
            baseline, "baseline"
        )

    result.baseline_ise = _compute_ise(
        baseline.times, baseline.commanded, baseline.measured
    )
    log_fn(f"  [{axis}] Baseline ISE: {result.baseline_ise:.4f}")

    if len(baseline.times) < 10:
        log_fn(f"  [{axis}] Too few samples, skipping")
        return result

    # 2. Plant identification
    plant = _identify_plant_inflection(
        baseline.times, baseline.measured, command
    )
    if plant is None:
        log_fn(f"  [{axis}] Inflection method failed, using rise-time fallback")
        plant = _identify_plant_rise_time(
            baseline.times, baseline.measured, command
        )

    result.plant = plant
    log_fn(f"  [{axis}] Plant: Ks={plant.Ks:.4f}, Tu={plant.Tu:.4f}s, "
           f"Tg={plant.Tg:.4f}s ({plant.method})")

    # 3. Compute PID gains via CHR
    pid = _compute_chr_gains(plant)
    if pid.kp <= 0:
        log_fn(f"  [{axis}] CHR produced zero gains, skipping")
        return result

    # Keep kV=1.0 feedforward, add PID on top
    ff = Feedforward(0.0, 1.0, 0.0)
    result.pid = pid
    result.ff = ff
    log_fn(f"  [{axis}] CHR gains: kp={pid.kp:.4f}, ki={pid.ki:.4f}, kd={pid.kd:.4f}")

    # 4. Validate: apply gains and re-run step response
    saved_cfg = robot.drive.get_velocity_control_config()
    test_cfg = robot.drive.get_velocity_control_config()
    axis_cfg = AxisVelocityControlConfig(pid, ff)
    if axis == "vx":
        test_cfg.vx = axis_cfg
    elif axis == "vy":
        test_cfg.vy = axis_cfg
    elif axis == "wz":
        test_cfg.wz = axis_cfg
    robot.drive.set_velocity_control_config(test_cfg)
    robot.drive.reset_velocity_controllers()

    tuned_data = await _run_step_response(robot, axis, command)

    if csv_dir:
        _write_step_csv(
            os.path.join(csv_dir, f"vel_{axis}_tuned.csv"),
            tuned_data, "tuned"
        )

    result.tuned_ise = _compute_ise(
        tuned_data.times, tuned_data.commanded, tuned_data.measured
    )
    log_fn(f"  [{axis}] Tuned ISE: {result.tuned_ise:.4f} "
           f"(baseline: {result.baseline_ise:.4f})")

    # 5. Accept or revert
    if result.tuned_ise < result.baseline_ise:
        result.accepted = True
        log_fn(f"  [{axis}] ACCEPTED (ISE improved by "
               f"{(1 - result.tuned_ise / result.baseline_ise) * 100:.1f}%)")
    else:
        result.accepted = False
        # Revert to config before this axis test
        robot.drive.set_velocity_control_config(saved_cfg)
        robot.drive.reset_velocity_controllers()
        log_fn(f"  [{axis}] REVERTED (ISE worse, keeping baseline)")

    return result


# ---------------------------------------------------------------------------
# Phase 2: Motion Controller Tuning (coordinate descent)
# ---------------------------------------------------------------------------

async def _run_linear_trial(
    robot: "GenericRobot",
    distance_m: float,
    speed_scale: float = _MOTION_PRIMARY_SPEED_SCALE,
    axis: "LinearAxis" = None,
) -> tuple[float, float, float]:
    """
    Run a linear drive trial. Returns (settling_time, overshoot, final_error).
    """
    config = LinearMotionConfig()
    config.axis = axis if axis is not None else LinearAxis.Forward
    config.distance_m = distance_m
    config.speed_scale = speed_scale

    motion = LinearMotion(
        robot.drive, robot.odometry, robot.motion_pid_config, config
    )

    robot.odometry.reset()
    await asyncio.sleep(0.05)
    robot.odometry.update(0.05)

    motion.start()

    rate = 1.0 / 50
    loop = asyncio.get_event_loop()
    t0 = loop.time()
    last = t0 - rate

    while not motion.is_finished():
        now = loop.time()
        if now - t0 > _MOTION_TIMEOUT_S:
            robot.drive.hard_stop()
            return _SCORE_TIMEOUT_PENALTY, 0.0, abs(distance_m)
        dt = max(now - last, 0.0)
        last = now
        if dt < 1e-4:
            await asyncio.sleep(rate)
            continue
        motion.update(dt)
        await asyncio.sleep(rate)

    settle_time = loop.time() - t0
    robot.drive.hard_stop()

    # Get final telemetry
    telemetry = list(motion.get_telemetry())
    if not telemetry:
        return settle_time, 0.0, abs(distance_m)

    final_error = abs(telemetry[-1].actual_error_m)

    # Compute overshoot: max position beyond target
    target = abs(distance_m)
    max_pos = max(abs(t.position_m) for t in telemetry)
    overshoot = max(0.0, max_pos - target)

    return settle_time, overshoot, final_error


async def _run_turn_trial(
    robot: "GenericRobot",
    angle_rad: float,
    speed_scale: float = _MOTION_PRIMARY_SPEED_SCALE,
) -> tuple[float, float, float]:
    """
    Run a turn trial. Returns (settling_time, overshoot, final_error).
    Manually tracks heading since TurnMotion has no get_telemetry().
    """
    config = TurnConfig()
    config.target_angle_rad = angle_rad
    config.speed_scale = speed_scale

    motion = TurnMotion(
        robot.drive, robot.odometry, robot.motion_pid_config, config
    )

    robot.odometry.reset()
    await asyncio.sleep(0.05)
    robot.odometry.update(0.05)

    motion.start()

    rate = 1.0 / 50
    loop = asyncio.get_event_loop()
    t0 = loop.time()
    last = t0 - rate

    heading_history: list[float] = []

    while not motion.is_finished():
        now = loop.time()
        if now - t0 > _MOTION_TIMEOUT_S:
            robot.drive.hard_stop()
            return _SCORE_TIMEOUT_PENALTY, 0.0, abs(angle_rad)
        dt = max(now - last, 0.0)
        last = now
        if dt < 1e-4:
            await asyncio.sleep(rate)
            continue
        motion.update(dt)
        heading_history.append(robot.odometry.get_heading())
        await asyncio.sleep(rate)

    settle_time = loop.time() - t0
    robot.drive.hard_stop()

    if not heading_history:
        return settle_time, 0.0, abs(angle_rad)

    final_heading = heading_history[-1]
    final_error = abs(angle_rad - final_heading)

    # Overshoot: how far past the target did the heading go
    target = angle_rad
    if target > 0:
        max_heading = max(heading_history)
        overshoot = max(0.0, max_heading - target)
    else:
        min_heading = min(heading_history)
        overshoot = max(0.0, target - min_heading)  # both negative

    return settle_time, overshoot, final_error


def _score_trial(
    settle_time: float, overshoot: float, final_error: float
) -> float:
    """Weighted score: lower is better."""
    return (
        _SCORE_SETTLE_WEIGHT * settle_time
        + _SCORE_OVERSHOOT_WEIGHT * overshoot
        + _SCORE_ERROR_WEIGHT * final_error
    )


def _score_motion_trial(
    param_name: str,
    settle_time: float,
    overshoot: float,
    final_error: float,
) -> float:
    """
    Constraint-aware motion score.

    Primary goal is still fast completion, but we heavily penalize candidates
    that exceed soft overshoot/final-error bounds so the optimizer doesn't pick
    "fast but sloppy" gains at 100% speed.
    """
    score = _score_trial(settle_time, overshoot, final_error)

    if param_name in ("distance", "lateral"):
        overshoot_soft = _LINEAR_OVERSHOOT_SOFT_M
        error_soft = _LINEAR_FINAL_ERROR_SOFT_M
        overshoot_penalty_per_unit = _SCORE_LINEAR_OVERSHOOT_BREACH_PER_M
        error_penalty_per_unit = _SCORE_LINEAR_ERROR_BREACH_PER_M
    else:
        overshoot_soft = _TURN_OVERSHOOT_SOFT_RAD
        error_soft = _TURN_FINAL_ERROR_SOFT_RAD
        overshoot_penalty_per_unit = _SCORE_TURN_OVERSHOOT_BREACH_PER_RAD
        error_penalty_per_unit = _SCORE_TURN_ERROR_BREACH_PER_RAD

    overshoot_breach = max(0.0, overshoot - overshoot_soft)
    if overshoot_breach > 0.0:
        score += _SCORE_CONSTRAINT_BREACH_BASE + overshoot_penalty_per_unit * overshoot_breach

    error_breach = max(0.0, final_error - error_soft)
    if error_breach > 0.0:
        score += _SCORE_CONSTRAINT_BREACH_BASE + error_penalty_per_unit * error_breach

    return score


async def _evaluate_gains(
    robot: "GenericRobot",
    param_name: str,
    kp: float,
    kd: float,
    trial_idx: int,
    *,
    speed_scale: float = _MOTION_PRIMARY_SPEED_SCALE,
) -> float:
    """
    Set gains, run alternating fwd/bwd or CW/CCW trials, return average score.
    """
    cfg = robot.motion_pid_config
    # "lateral" uses the same distance PID (shared by LinearMotion for both axes)
    pid_attr = "distance" if param_name == "lateral" else param_name
    pid_cfg = getattr(cfg, pid_attr)
    pid_cfg.kp = kp
    pid_cfg.kd = kd

    # Alternate direction based on trial index
    if param_name == "distance":
        sign = 1.0 if trial_idx % 2 == 0 else -1.0
        settle, overshoot, error = await _run_linear_trial(
            robot, sign * _LINEAR_TEST_DISTANCE_M, speed_scale=speed_scale,
        )
    elif param_name == "lateral":
        sign = 1.0 if trial_idx % 2 == 0 else -1.0
        settle, overshoot, error = await _run_linear_trial(
            robot, sign * _LINEAR_TEST_DISTANCE_M, speed_scale=speed_scale,
            axis=LinearAxis.Lateral,
        )
    else:  # heading
        sign = 1.0 if trial_idx % 2 == 0 else -1.0
        settle, overshoot, error = await _run_turn_trial(
            robot, sign * _TURN_TEST_ANGLE_RAD, speed_scale=speed_scale
        )

    await asyncio.sleep(_MOTION_SETTLE_S)
    return _score_motion_trial(param_name, settle, overshoot, error)


async def _watch_motion_performance(
    robot: "GenericRobot",
    param_name: str,
    kp: float,
    kd: float,
    speed_scales: tuple[float, ...],
    log_fn,
) -> None:
    """
    Run secondary checks at lower speeds. These do not affect optimization.
    """
    for speed_scale in speed_scales:
        # Evaluate both directions so the watch is less sensitive to one-off bias.
        score_a = await _evaluate_gains(
            robot, param_name, kp, kd, trial_idx=0, speed_scale=speed_scale
        )
        score_b = await _evaluate_gains(
            robot, param_name, kp, kd, trial_idx=1, speed_scale=speed_scale
        )
        avg_score = 0.5 * (score_a + score_b)
        log_fn(
            f"  [{param_name}] watch speed={speed_scale:.2f}: "
            f"score≈{avg_score:.4f} (secondary, not optimized)"
        )


async def _coordinate_descent(
    robot: "GenericRobot",
    param_name: str,
    initial_kp: float,
    initial_kd: float,
    log_fn,
) -> MotionTuneResult:
    """
    Hooke-Jeeves coordinate descent: try +/- delta per parameter, keep best,
    halve deltas when no improvement.
    """
    result = MotionTuneResult(
        param_name=param_name,
        initial_kp=initial_kp,
        initial_kd=initial_kd,
    )

    kp = initial_kp
    kd = initial_kd
    delta_kp = max(initial_kp * _CD_INITIAL_DELTA_FRAC, _CD_MIN_DELTA)
    delta_kd = max(initial_kd * _CD_INITIAL_DELTA_FRAC, _CD_MIN_DELTA)

    # Evaluate initial gains
    trial_idx = 0
    best_score = await _evaluate_gains(robot, param_name, kp, kd, trial_idx)
    trial_idx += 1
    result.initial_score = best_score
    log_fn(f"  [{param_name}] Initial: kp={kp:.4f}, kd={kd:.4f}, "
           f"score={best_score:.4f}")

    for iteration in range(_CD_MAX_ITERATIONS):
        improved = False

        # Try kp +/- delta
        for kp_candidate in [kp + delta_kp, kp - delta_kp]:
            if kp_candidate <= 0:
                continue
            score = await _evaluate_gains(
                robot, param_name, kp_candidate, kd, trial_idx
            )
            trial_idx += 1
            if score < best_score:
                best_score = score
                kp = kp_candidate
                improved = True
                log_fn(f"  [{param_name}] iter {iteration}: kp={kp:.4f} "
                       f"(score={score:.4f})")
                break

        # Try kd +/- delta
        for kd_candidate in [kd + delta_kd, kd - delta_kd]:
            if kd_candidate < 0:
                continue
            score = await _evaluate_gains(
                robot, param_name, kp, kd_candidate, trial_idx
            )
            trial_idx += 1
            if score < best_score:
                best_score = score
                kd = kd_candidate
                improved = True
                log_fn(f"  [{param_name}] iter {iteration}: kd={kd:.4f} "
                       f"(score={score:.4f})")
                break

        if not improved:
            delta_kp *= _CD_DELTA_SHRINK
            delta_kd *= _CD_DELTA_SHRINK
            log_fn(f"  [{param_name}] iter {iteration}: no improvement, "
                   f"deltas -> ({delta_kp:.4f}, {delta_kd:.4f})")
            if delta_kp < _CD_MIN_DELTA and delta_kd < _CD_MIN_DELTA:
                log_fn(f"  [{param_name}] Deltas below minimum, stopping")
                break

        result.iterations = iteration + 1

    result.final_kp = kp
    result.final_kd = kd
    result.final_score = best_score
    return result


# ---------------------------------------------------------------------------
# Step classes
# ---------------------------------------------------------------------------

@dsl_step(tags=["motion", "calibration", "auto-tune"])
class AutoTuneVelocity(Step):
    """Tune velocity controllers via step-response system identification.

    This is Phase 2 of the full auto-tune pipeline, usable standalone when
    drive characterization has already been performed (or when the hardware
    limits are already configured in ``raccoon.project.yml``).

    For each velocity axis the process is:

    1. **Baseline recording** -- command a step velocity at 50% of the
       characterized max and record the response at 100 Hz.
    2. **Plant identification** -- extract gain (Ks), dead time (Tu), and
       time constant (Tg) using the inflection tangent method (local
       quadratic regression to find the inflection point, then tangent-line
       intersections). Falls back to 10%/63% rise-time estimation if the
       inflection method fails.
    3. **Gain computation** -- compute PID gains via CHR set-point-follow
       formulas, scaled down because a kV=1.0 feedforward handles
       steady-state.
    4. **Validation** -- apply the new gains and re-run the step response.
       If the integral of squared error (ISE) improves, the gains are
       accepted; otherwise the baseline gains are restored.

    Accepted gains are applied in-memory to the drive's velocity control
    config and optionally persisted to ``raccoon.project.yml`` under
    ``robot.drive.vel_config``.

    Prerequisites: Drive characterization should have been run first so that
    max velocity values are available for computing the step command
    magnitude.

    Args:
        axes: Velocity axes to tune. Each entry is a velocity component
            name: ``"vx"`` (forward), ``"vy"`` (lateral/strafe), or
            ``"wz"`` (angular). Default ``["vx", "vy", "wz"]``.
        persist: If ``True``, write accepted gains to
            ``raccoon.project.yml``. Default ``True``.
        csv_dir: Directory for step-response CSV files (baseline and tuned
            recordings per axis). Default ``"/tmp/auto_tune"``.

    Example::

        from raccoon.step.motion import auto_tune_velocity

        # Tune both velocity axes with defaults
        auto_tune_velocity()

        # Tune only the forward axis, save CSVs for plotting
        auto_tune_velocity(
            axes=["vx"],
            csv_dir="/tmp/vel_tune_plots",
        )
    """

    def __init__(
        self,
        axes: list[str] = None,
        persist: bool = True,
        csv_dir: Optional[str] = "/tmp/auto_tune",
    ):
        super().__init__()
        self.axes = axes  # None = auto-detect from kinematics
        self.persist = persist
        self.csv_dir = csv_dir
        self.results: dict[str, VelocityTuneResult] = {}

    def _generate_signature(self) -> str:
        return (
            f"AutoTuneVelocity(axes={self.axes}, persist={self.persist})"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping velocity auto-tune, using stored values")
            return

        # Resolve default axes based on kinematics lateral support
        axes = self.axes
        if axes is None:
            has_lateral = robot.drive.supports_lateral_motion()
            axes = ["vx", "vy", "wz"] if has_lateral else ["vx", "wz"]
            if not has_lateral:
                self.info("  Kinematics does not support lateral motion — skipping vy")

        if self.csv_dir:
            os.makedirs(self.csv_dir, exist_ok=True)

        self.info("=" * 60)
        self.info("  AUTO-TUNE: VELOCITY CONTROLLERS (Phase 2)")
        self.info(f"  Axes: {', '.join(axes)}")
        self.info("=" * 60)

        for axis in axes:
            self.info(f"\n--- {axis.upper()} ---")
            result = await _tune_velocity_axis(
                robot, axis, self.csv_dir, self.info
            )
            self.results[axis] = result
            robot.drive.hard_stop()
            await asyncio.sleep(0.5)

        # Apply accepted gains
        accepted = {k: v for k, v in self.results.items() if v.accepted}
        if accepted:
            _apply_velocity_gains(robot, self.results)
            self.info(f"\n  Applied {len(accepted)} axis gains in-memory")
        else:
            self.info("\n  No axes improved, keeping baseline")

        # Persist
        if self.persist and accepted:
            if _persist_velocity_config(self.results):
                self.info("  Saved to raccoon.project.yml "
                          "(robot.drive.vel_config)")
            else:
                self.warn("  Failed to save to raccoon.project.yml")

        # Summary
        self.info("\n" + "=" * 60)
        self.info("  VELOCITY TUNE RESULTS")
        self.info("=" * 60)
        for axis, r in self.results.items():
            status = "ACCEPTED" if r.accepted else "REVERTED"
            self.info(
                f"  {axis}: {status} | plant=({r.plant.Ks:.3f}, "
                f"{r.plant.Tu:.3f}s, {r.plant.Tg:.3f}s) | "
                f"pid=(kp={r.pid.kp:.4f}, ki={r.pid.ki:.4f}, kd={r.pid.kd:.4f}) | "
                f"ISE: {r.baseline_ise:.4f} -> {r.tuned_ise:.4f}"
            )
        self.info("=" * 60)


@dsl_step(tags=["motion", "calibration", "auto-tune"])
class AutoTuneMotion(Step):
    """Tune motion PID controllers via iterative real-world optimization.

    This is Phase 3 of the full auto-tune pipeline, usable standalone when
    velocity controllers are already tuned and drive limits are characterized.

    Uses Hooke-Jeeves coordinate descent to optimize the high-level distance
    and/or heading PID controllers (kp and kd). The process for each
    parameter set is:

    1. **Initial evaluation** -- run a test motion (0.5 m drive for distance,
       90-degree turn for heading) with the current gains and compute a
       weighted score from settling time, overshoot, and final error.
    2. **Coordinate descent** -- try perturbing kp up/down by a delta, keep
       the direction that improves the score. Then do the same for kd.
       Repeat for up to 10 iterations. When neither direction improves,
       halve the deltas. Stop when deltas fall below a minimum threshold.
    3. **Constraint-aware scoring** -- the optimizer prioritizes fast
       completion but heavily penalizes candidates that exceed soft limits
       on overshoot (10 mm / 3 degrees) or final error (10 mm / 2 degrees),
       preventing "fast but sloppy" solutions.
    4. **Secondary speed checks** -- after optimization at full speed, the
       final gains are tested at lower speeds (50%, 30%) for monitoring
       purposes (these do not affect the optimization).

    Trials alternate between forward/backward (or CW/CCW) to reduce
    directional bias.

    Prerequisites: Velocity controllers should be tuned first (Phase 2) and
    drive limits characterized (Phase 1) for best results. The robot needs
    enough space for 0.5 m drives and 90-degree turns.

    Args:
        axes: Motion parameters to optimize. Options are ``"distance"``
            (forward drive kp/kd), ``"lateral"`` (strafe kp/kd — shares
            the distance PID but optimizes with lateral trials), and
            ``"heading"`` (turn kp/kd). Default
            ``["distance", "lateral", "heading"]``.
        persist: If ``True``, write final gains to
            ``raccoon.project.yml`` under ``robot.motion_pid``. Default
            ``True``.
        csv_dir: Directory for diagnostic CSV output. Default
            ``"/tmp/auto_tune"``.

    Example::

        from raccoon.step.motion import auto_tune_motion

        # Tune both distance and heading controllers
        auto_tune_motion()

        # Tune only the heading controller
        auto_tune_motion(axes=["heading"])

        # Tune without persisting (for experimentation)
        auto_tune_motion(persist=False)
    """

    def __init__(
        self,
        axes: list[str] = None,
        persist: bool = True,
        csv_dir: Optional[str] = "/tmp/auto_tune",
    ):
        super().__init__()
        self.axes = axes  # None = auto-detect from kinematics
        self.persist = persist
        self.csv_dir = csv_dir
        self.results: dict[str, MotionTuneResult] = {}

    def _generate_signature(self) -> str:
        return (
            f"AutoTuneMotion(axes={self.axes}, persist={self.persist})"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping motion auto-tune, using stored values")
            return

        # Resolve default axes based on kinematics lateral support
        axes = self.axes
        if axes is None:
            has_lateral = robot.drive.supports_lateral_motion()
            axes = ["distance", "lateral", "heading"] if has_lateral else ["distance", "heading"]
            if not has_lateral:
                self.info("  Kinematics does not support lateral motion — skipping lateral")

        self.info("=" * 60)
        self.info("  AUTO-TUNE: MOTION CONTROLLERS (Phase 3)")
        self.info(f"  Parameters: {', '.join(axes)}")
        self.info(f"  Primary objective speed_scale={_MOTION_PRIMARY_SPEED_SCALE:.2f} "
                  "(fastest)")
        if _MOTION_WATCH_SPEED_SCALES:
            watch_s = ", ".join(f"{s:.2f}" for s in _MOTION_WATCH_SPEED_SCALES)
            self.info(f"  Lower-speed watch (secondary only): {watch_s}")
        self.info("=" * 60)

        cfg = robot.motion_pid_config

        for param_name in axes:
            # "lateral" shares the distance PID
            pid_attr = "distance" if param_name == "lateral" else param_name
            pid_cfg = getattr(cfg, pid_attr, None)
            if pid_cfg is None:
                self.warn(f"  [{param_name}] Not found in motion config")
                continue

            self.info(f"\n--- {param_name.upper()} ---")
            self.info(f"  [{param_name}] Current: kp={pid_cfg.kp:.4f}, "
                       f"kd={pid_cfg.kd:.4f}")

            result = await _coordinate_descent(
                robot, param_name, pid_cfg.kp, pid_cfg.kd, self.info
            )
            self.results[param_name] = result

            if _MOTION_WATCH_SPEED_SCALES:
                await _watch_motion_performance(
                    robot,
                    param_name,
                    result.final_kp,
                    result.final_kd,
                    _MOTION_WATCH_SPEED_SCALES,
                    self.info,
                )

            robot.drive.hard_stop()
            await asyncio.sleep(_MOTION_SETTLE_S)

        # Apply final gains
        if self.results:
            _apply_motion_gains(robot, self.results)
            self.info(f"\n  Applied {len(self.results)} parameter sets in-memory")

        # Persist
        if self.persist and self.results:
            if _persist_motion_config(self.results):
                self.info("  Saved to raccoon.project.yml "
                          "(robot.motion_pid.{distance,heading})")
            else:
                self.warn("  Failed to save to raccoon.project.yml")

        # Summary
        self.info("\n" + "=" * 60)
        self.info("  MOTION TUNE RESULTS")
        self.info("=" * 60)
        for name, r in self.results.items():
            improvement = (
                (1 - r.final_score / r.initial_score) * 100
                if r.initial_score > 0 else 0.0
            )
            self.info(
                f"  {name}: kp {r.initial_kp:.4f}->{r.final_kp:.4f}, "
                f"kd {r.initial_kd:.4f}->{r.final_kd:.4f} | "
                f"score: {r.initial_score:.4f}->{r.final_score:.4f} "
                f"({improvement:+.1f}%) | {r.iterations} iters"
            )
        self.info("=" * 60)


@dsl_step(tags=["motion", "calibration", "auto-tune"])
class AutoTune(Step):
    """Auto-tune the full drive system: characterize, velocity PID, motion PID.

    Runs a three-phase sequential pipeline that takes the robot from unknown
    hardware limits to fully tuned PID controllers. Each phase builds on the
    results of the previous one:

    **Phase 1 -- Drive characterization.** Commands raw velocities to measure
    max velocity, acceleration, and deceleration for each axis. Multiple
    trials are run and the median is taken. Results are stored so that
    subsequent phases have accurate physical constraints for profile
    generation.

    **Phase 2 -- Velocity controller tuning.** For each velocity axis (e.g.,
    ``vx``, ``vy``, ``wz``), a step-response is recorded at 100 Hz, plant parameters
    (gain Ks, dead time Tu, time constant Tg) are identified via the
    inflection tangent method, and PID gains are computed using CHR
    set-point-follow formulas. A validation step-response is run with the
    new gains; if the integral of squared error (ISE) improves, the gains
    are accepted, otherwise the baseline is kept.

    **Phase 3 -- Motion controller tuning.** Uses Hooke-Jeeves coordinate
    descent to optimize the high-level distance, lateral, and heading PID
    controllers. Real test drives, strafes, and turns are executed, scored
    on a weighted combination of settling time, overshoot, and final error.
    The optimizer adjusts kp/kd iteratively, halving the search delta when
    no improvement is found.

    All results are applied in-memory immediately and, if ``persist`` is
    enabled, written to ``raccoon.project.yml`` so they survive restarts.

    This step requires significant clear space and takes several minutes to
    complete. It is intended for initial robot setup, not competition runs.

    Args:
        vel_axes: Velocity axes to tune in Phase 2. Each entry is a velocity
            component name (``"vx"`` for forward, ``"vy"`` for lateral/strafe,
            ``"wz"`` for angular). Default ``["vx", "vy", "wz"]``.
        characterize_axes: Axes to characterize in Phase 1. Options are
            ``"forward"``, ``"lateral"``, ``"angular"``. Default
            ``["forward", "lateral", "angular"]``.
        motion_axes: Motion parameters to optimize in Phase 3. Options are
            ``"distance"``, ``"lateral"`` (shares the distance PID but
            optimizes with lateral trials), and ``"heading"``. Default
            ``["distance", "lateral", "heading"]``.
        tune_characterize: Whether to run Phase 1. Set to ``False`` if the
            robot's limits are already known. Default ``True``.
        tune_velocity: Whether to run Phase 2. Default ``True``.
        tune_motion: Whether to run Phase 3. Default ``True``.
        characterize_trials: Number of trials per axis in Phase 1. More
            trials improve robustness but take longer. Default 3.
        characterize_power_percent: Motor power percentage (1--100) for
            Phase 1 drive characterization. Default 100.
        persist: If ``True``, write all results to
            ``raccoon.project.yml``. Default ``True``.
        csv_dir: Directory for diagnostic CSV output (step-response
            recordings, etc.). Default ``"/tmp/auto_tune"``.

    Example::

        from raccoon.step.motion import auto_tune

        # Full auto-tune with defaults
        auto_tune()

        # Skip characterization (already done), tune only velocity + motion
        auto_tune(tune_characterize=False)

        # Tune only the forward velocity axis and distance controller
        auto_tune(
            vel_axes=["vx"],
            motion_axes=["distance"],
            characterize_axes=["forward"],
        )

        # Dry run without persisting to YAML
        auto_tune(persist=False, csv_dir="/tmp/auto_tune_test")
    """

    def __init__(
        self,
        vel_axes: list[str] = None,
        characterize_axes: list[str] = None,
        motion_axes: list[str] = None,
        tune_characterize: bool = True,
        tune_velocity: bool = True,
        tune_motion: bool = True,
        characterize_trials: int = 3,
        characterize_power_percent: int = 100,
        persist: bool = True,
        csv_dir: Optional[str] = "/tmp/auto_tune",
    ):
        super().__init__()
        # None = auto-detect from kinematics at execution time
        self.characterize_axes = characterize_axes
        self.vel_axes = vel_axes
        self.motion_axes = motion_axes
        self.tune_characterize = tune_characterize
        self.tune_velocity = tune_velocity
        self.tune_motion = tune_motion
        self.characterize_trials = characterize_trials
        self.characterize_power_percent = characterize_power_percent
        self.persist = persist
        self.csv_dir = csv_dir

    def _generate_signature(self) -> str:
        return (
            f"AutoTune(char={self.tune_characterize}, "
            f"vel={self.vel_axes}, motion={self.motion_axes}, "
            f"persist={self.persist})"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping full auto-tune, using stored values")
            return

        # Resolve default axes based on kinematics lateral support
        has_lateral = robot.drive.supports_lateral_motion()
        characterize_axes = self.characterize_axes
        if characterize_axes is None:
            characterize_axes = (
                ["forward", "lateral", "angular"] if has_lateral
                else ["forward", "angular"]
            )
            if not has_lateral:
                self.info("  Kinematics does not support lateral motion — "
                          "skipping lateral axes across all phases")

        self.info("=" * 60)
        self.info("  AUTO-TUNE PID CONTROLLERS")
        phases = []
        if self.tune_characterize:
            phases.append("characterize")
        if self.tune_velocity:
            phases.append("velocity PID")
        if self.tune_motion:
            phases.append("motion PID")
        self.info(f"  Phases: {' -> '.join(phases)}")
        self.info("=" * 60)

        if self.tune_characterize:
            char_step = CharacterizeDrive(
                axes=characterize_axes,
                trials=self.characterize_trials,
                power_percent=self.characterize_power_percent,
                accel_timeout=3.0,
                decel_timeout=3.0,
                persist=self.persist,
            )
            await char_step._execute_step(robot)

        if self.tune_velocity:
            vel_step = AutoTuneVelocity(
                self.vel_axes, self.persist, self.csv_dir
            )
            await vel_step._execute_step(robot)

        if self.tune_motion:
            motion_step = AutoTuneMotion(
                self.motion_axes, self.persist, self.csv_dir
            )
            await motion_step._execute_step(robot)

        self.info("\n" + "=" * 60)
        self.info("  AUTO-TUNE COMPLETE")
        self.info("=" * 60)


