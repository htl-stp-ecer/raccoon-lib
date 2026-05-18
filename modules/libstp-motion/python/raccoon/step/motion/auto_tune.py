"""
Auto-tune PID controllers via system identification and iterative optimization.

The full pipeline lives in C++ (``raccoon.autotune.AutoTuner``) so phase-to-phase
state coherence is guaranteed: every phase reads its inputs from the live
``IMotor`` / ``Drive`` / ``UnifiedMotionPidConfig`` objects and writes its
results back to those same objects before returning. The Python layer below
is a thin orchestrator that adds UI confirmations and YAML persistence.

Phases run in this dependency order:

Phase 1 - Velocity LPF alpha (per-motor IIR filter)
    Tunes the filter that downstream velocity feedback depends on, so every
    subsequent phase sees clean velocity estimates.

Phase 2 - Static friction (kS, PWM percent)
    Sweep per motor to find the duty cycle that overcomes static friction.

Phase 3 - Firmware velocity PID (STM32 MAV-mode inner loop)
    BEMF step-response identification per motor, CHR gains pushed to the STM32.

Phase 4 - Encoder calibration (ticks_to_rad)
    IMU-vs-odometry sweep scales each motor's ticks_to_rad and re-publishes
    the kinematics config to the STM32.

Phase 5 - Drive characterization (max velocity, accel, decel)
    Raw-PWM trials measure the physical limits of every axis.

Phase 6 - Velocity controller tuning (outer chassis loop)
    Step-response identification + CHR gains, ISE-validated accept/revert.

Phase 7 - Motion controller tuning (distance / heading PID)
    Hooke-Jeeves coordinate descent on real LinearMotion / TurnMotion trials.

Phase 8 - Tolerance derivation (pure math)
    Reads back motion-trial residuals and updates distance/angle tolerances.

Step classes:
    AutoTune                  - full pipeline (all phases)
    AutoTuneVelLpf            - Phase 1 only
    AutoTuneStaticFriction    - Phase 2 only
    AutoTuneFirmwarePid       - Phase 3 only
    AutoTuneVelocity          - Phase 6 only
    AutoTuneMotion            - Phase 7 only
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from raccoon.project_yaml import find_project_root, read_project_value, update_project_value
from raccoon.ui import UIStep

from .. import Step
from ..annotation import dsl_step
from .characterize_drive import CharacterizeDrive

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# YAML persistence helpers
# ---------------------------------------------------------------------------


def _persist_velocity_config(results: dict) -> bool:
    """Write velocity control gains to raccoon.project.yml."""
    project_root = find_project_root()
    if project_root is None:
        return False

    updated = False
    for axis_name, result in results.items():
        if not result.accepted:
            continue
        base = ["robot", "drive", "vel_config", axis_name]
        update_project_value(
            project_root,
            [*base, "pid"],
            {
                "kp": round(result.pid.kp, 6),
                "ki": round(result.pid.ki, 6),
                "kd": round(result.pid.kd, 6),
            },
        )
        update_project_value(
            project_root,
            [*base, "ff"],
            {
                "kS": round(result.ff.kS, 6),
                "kV": round(result.ff.kV, 6),
                "kA": round(result.ff.kA, 6),
            },
        )
        updated = True

    return updated


def _persist_motion_config(results: dict) -> bool:
    """Write motion PID gains to raccoon.project.yml."""
    project_root = find_project_root()
    if project_root is None:
        return False

    updated = False
    for param_name, result in results.items():
        yaml_key = "distance" if param_name == "lateral" else param_name
        base = ["robot", "motion_pid", yaml_key]
        update_project_value(project_root, [*base, "kp"], round(result.final_kp, 4))
        update_project_value(project_root, [*base, "kd"], round(result.final_kd, 4))
        updated = True

    return updated


def _motor_name_by_port(project_root, port: int) -> str | None:
    """Map a motor port (int) to its YAML definition name.

    Motor definitions live under ``definitions.<name>`` with ``type: Motor`` and a
    ``port`` field; the calibration we persist lives under
    ``definitions.<name>.calibration.*``.
    """
    definitions = read_project_value(project_root, ["definitions"], {}) or {}
    for name, definition in definitions.items():
        if (
            isinstance(definition, dict)
            and definition.get("type") == "Motor"
            and definition.get("port") == port
        ):
            return name
    return None


_VEL_LPF_SENTINEL_SCORE = 1e6  # any score above this means the sweep saw no usable signal


def _persist_vel_lpf(results: dict) -> bool:
    """Write tuned vel_lpf_alpha values per motor to definitions.<motor>.calibration.

    Motors whose `min_score` is at the sentinel level (~5e8) are skipped — that
    indicates BEMF readings were a constant zero throughout the sweep, so the
    tuned alpha (typically clamped to alpha_min) carries no information.
    """
    project_root = find_project_root()
    if project_root is None:
        return False
    updated = False
    for port, r in results.items():
        if not r.applied:
            continue
        if r.min_score >= _VEL_LPF_SENTINEL_SCORE:
            continue
        motor_name = _motor_name_by_port(project_root, port)
        if motor_name is None:
            continue
        if update_project_value(
            project_root,
            ["definitions", motor_name, "calibration", "vel_lpf_alpha"],
            round(r.tuned_alpha, 4),
        ):
            updated = True
    return updated


def _persist_ticks_to_rad(result) -> bool:
    """Write tuned per-motor ticks_to_rad to definitions.<motor>.calibration."""
    if not getattr(result, "success", False):
        return False
    project_root = find_project_root()
    if project_root is None:
        return False
    updated = False
    for port, t2r in enumerate(result.ticks_to_rad):
        if t2r <= 0.0:
            continue
        motor_name = _motor_name_by_port(project_root, port)
        if motor_name is None:
            continue
        if update_project_value(
            project_root,
            ["definitions", motor_name, "calibration", "ticks_to_rad"],
            round(t2r, 8),
        ):
            updated = True
    return updated


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _resolve_velocity_axes(robot: "GenericRobot", axes: list[str] | None) -> list[str]:
    if axes is not None:
        return axes
    has_lateral = robot.drive.supports_lateral_motion()
    return ["vx", "vy", "wz"] if has_lateral else ["vx", "wz"]


def _resolve_motion_params(robot: "GenericRobot", params: list[str] | None) -> list[str]:
    if params is not None:
        return params
    has_lateral = robot.drive.supports_lateral_motion()
    return ["distance", "lateral", "heading"] if has_lateral else ["distance", "heading"]


def _get_max_velocity(robot: "GenericRobot", axis: str) -> float:
    cfg = robot.motion_pid_config
    if axis == "vx":
        v = cfg.linear.max_velocity
        return v if v > 0 else 0.3
    if axis == "vy":
        v = cfg.lateral.max_velocity
        return v if v > 0 else 0.3
    if axis == "wz":
        v = cfg.angular.max_velocity
        return v if v > 0 else 2.0
    return 1.0


def _make_tuner(robot: "GenericRobot"):
    """Construct a fresh C++ AutoTuner bound to this robot's drive / odom / pid config."""
    from raccoon.autotune import AutoTuner

    return AutoTuner(robot.drive, robot.odometry, robot.motion_pid_config)


def _run_in_executor(callable_):
    return asyncio.get_event_loop().run_in_executor(None, callable_)


# ---------------------------------------------------------------------------
# Single-phase step classes
# ---------------------------------------------------------------------------


@dsl_step(tags=["motion", "calibration", "auto-tune"])
class AutoTuneVelLpf(Step):
    """Tune the IIR velocity-filter alpha per motor (Phase 1).

    Collects raw BEMF samples at a steady velocity, replays them through IIR
    low-pass filters with varying alpha, and applies the alpha that minimises
    a weighted noise+lag score. Runs first in the full pipeline because every
    downstream phase relies on the same velocity-feedback filter.

    Args:
        persist: Write tuned ``vel_lpf_alpha`` values to
            ``raccoon.project.yml``. Default ``True``.

    Example::

        from raccoon.step.motion import auto_tune_vel_lpf

        auto_tune_vel_lpf()
    """

    def __init__(self, persist: bool = True):
        super().__init__()
        self.persist = persist

    def _generate_signature(self) -> str:
        return f"AutoTuneVelLpf(persist={self.persist})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping vel LPF tune")
            return

        from raccoon.autotune import VelLpfConfig

        tuner = _make_tuner(robot)

        self.info("=" * 60)
        self.info("  AUTO-TUNE: VELOCITY LPF ALPHA (Phase 1)")
        self.info("=" * 60)

        results = await _run_in_executor(lambda: tuner.run_vel_lpf(VelLpfConfig()))

        for port, r in results.items():
            self.info(
                f"  port={port}: alpha {r.initial_alpha:.2f} -> {r.tuned_alpha:.2f} "
                f"(score={r.min_score:.4f})"
            )

        if self.persist and results and _persist_vel_lpf(results):
            self.info("  Saved vel_lpf_alpha to raccoon.project.yml")


@dsl_step(tags=["calibration", "auto-tune"])
class AutoTuneStaticFriction(Step):
    """Measure per-motor static-friction threshold kS in PWM percent (Phase 2).

    For each drive motor the PWM is swept from a low starting percentage upward
    in both directions. The first PWM level where the median BEMF exceeds a
    motion threshold is recorded as kS.

    Args:
        persist: Reserved for future YAML persistence. Currently logs only.

    Example::

        from raccoon.step.motion import auto_tune_static_friction

        auto_tune_static_friction()
    """

    def __init__(self, persist: bool = True):
        super().__init__()
        self.persist = persist

    def _generate_signature(self) -> str:
        return f"AutoTuneStaticFriction(persist={self.persist})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping static friction measurement")
            return

        from raccoon.autotune import StaticFrictionConfig

        tuner = _make_tuner(robot)

        self.info("=" * 60)
        self.info("  AUTO-TUNE: STATIC FRICTION (Phase 2)")
        self.info("=" * 60)

        results = await _run_in_executor(lambda: tuner.run_static_friction(StaticFrictionConfig()))

        for port, r in results.items():
            if r.measured:
                self.info(
                    f"  port={port}: kS_pos={r.ks_positive_pct}% "
                    f"kS_neg={r.ks_negative_pct}% kS_avg={r.ks_avg_pct}%"
                )
            else:
                self.warn(f"  port={port}: motor never moved — check wiring/power")


@dsl_step(tags=["calibration", "auto-tune"])
class AutoTuneFirmwarePid(Step):
    """Tune per-motor STM32 MAV-mode velocity PID via BEMF step response (Phase 3).

    For each drive motor: record a BEMF step response, fit a FOPDT plant,
    derive CHR PID gains, push them to the firmware. Gains are accepted only
    when the tuned ISE is strictly smaller than the baseline ISE.

    Args:
        persist: Unused — gains are applied directly to firmware state.
        max_bemf_speeds: Optional ``{port: max_bemf_speed}`` map. If unset,
            the C++ side runs a brief power sweep to estimate it.
        csv_dir: When set, every step-response sample is dumped to a CSV
            under this directory (one per motor + phase) plus a summary CSV
            with the plant fit and gains. Default ``"/tmp/auto_tune"``.

    Example::

        from raccoon.step.motion import auto_tune_firmware_pid

        auto_tune_firmware_pid()
    """

    def __init__(
        self,
        persist: bool = True,
        max_bemf_speeds: dict[int, int] | None = None,
        csv_dir: str | None = "/tmp/auto_tune",
    ):
        super().__init__()
        self.persist = persist
        self.max_bemf_speeds = max_bemf_speeds
        self.csv_dir = csv_dir

    def _generate_signature(self) -> str:
        return f"AutoTuneFirmwarePid(persist={self.persist})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping firmware PID tune")
            return

        from raccoon.autotune import FirmwarePidConfig

        tuner = _make_tuner(robot)

        cfg = FirmwarePidConfig()
        if self.csv_dir:
            cfg.csv_dir = self.csv_dir

        self.info("=" * 60)
        self.info("  AUTO-TUNE: FIRMWARE PID (Phase 3)")
        if self.csv_dir:
            self.info(f"  CSV dump: {self.csv_dir}")
        self.info("=" * 60)

        results = await _run_in_executor(lambda: tuner.run_firmware_pid(cfg, self.max_bemf_speeds))

        for port, r in results.items():
            status = "ACCEPTED" if r.accepted else "REVERTED"
            self.info(
                f"  port={port}: {status} | kp={r.kp:.4f} ki={r.ki:.4f} kd={r.kd:.4f} | "
                f"ISE: {r.baseline_ise:.4f} -> {r.tuned_ise:.4f}"
            )


@dsl_step(tags=["motion", "calibration", "auto-tune"])
class AutoTuneVelocity(Step):
    """Tune velocity controllers via step-response system identification (Phase 6).

    Records a baseline step response, fits a FOPDT plant model, derives CHR
    PID gains, applies them, then re-runs the step response to validate via
    ISE. The accepted gains are pushed to ``drive.set_velocity_control_config``
    immediately by the C++ tuner.

    Prerequisites: Phase 5 (drive characterization) should have run first so
    a max-velocity-per-axis is known. Phase 3 (firmware PID) should also have
    run so the inner loop is stable.

    Args:
        axes: Velocity axes to tune (``"vx"``, ``"vy"``, ``"wz"``).
            Default auto-detects from kinematics.
        persist: Write accepted gains to ``raccoon.project.yml``. Default ``True``.

    Example::

        from raccoon.step.motion import auto_tune_velocity

        auto_tune_velocity()
        auto_tune_velocity(axes=["vx"])
    """

    def __init__(
        self,
        axes: list[str] | None = None,
        persist: bool = True,
    ):
        super().__init__()
        self.axes = axes
        self.persist = persist

    def _generate_signature(self) -> str:
        return f"AutoTuneVelocity(axes={self.axes}, persist={self.persist})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping velocity auto-tune, using stored values")
            return

        from raccoon.autotune import VelocityTuneConfig

        axes = _resolve_velocity_axes(robot, self.axes)
        max_velocities = {ax: _get_max_velocity(robot, ax) for ax in axes}

        tuner = _make_tuner(robot)

        self.info("=" * 60)
        self.info("  AUTO-TUNE: VELOCITY CONTROLLERS (Phase 6)")
        self.info(f"  Axes: {', '.join(axes)}")
        self.info("=" * 60)

        self.results = await _run_in_executor(
            lambda: tuner.run_velocity(axes, max_velocities, VelocityTuneConfig())
        )

        accepted_count = sum(1 for r in self.results.values() if r.accepted)
        self.info(f"\n  Applied {accepted_count} axis gains in-memory")

        if self.persist and any(r.accepted for r in self.results.values()):
            if _persist_velocity_config(self.results):
                self.info("  Saved to raccoon.project.yml (robot.drive.vel_config)")
            else:
                self.warn("  Failed to save to raccoon.project.yml")

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


@dsl_step(tags=["motion", "calibration", "auto-tune"])
class AutoTuneMotion(Step):
    """Tune motion PID controllers via iterative real-world optimization (Phase 7).

    Uses Hooke-Jeeves coordinate descent on the distance/heading PID kp & kd
    via real LinearMotion and TurnMotion trials with constraint-aware scoring.

    Prerequisites: velocity controllers tuned (Phase 6) and drive limits
    characterized (Phase 5).

    Args:
        axes: Motion parameters to tune (``"distance"``, ``"lateral"``,
            ``"heading"``). Default auto-detects from kinematics.
        persist: Write final gains to ``raccoon.project.yml``. Default ``True``.

    Example::

        from raccoon.step.motion import auto_tune_motion

        auto_tune_motion()
        auto_tune_motion(axes=["heading"])
    """

    def __init__(
        self,
        axes: list[str] | None = None,
        persist: bool = True,
    ):
        super().__init__()
        self.axes = axes
        self.persist = persist

    def _generate_signature(self) -> str:
        return f"AutoTuneMotion(axes={self.axes}, persist={self.persist})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping motion auto-tune, using stored values")
            return

        from raccoon.autotune import MotionTuneConfig

        params = _resolve_motion_params(robot, self.axes)
        tuner = _make_tuner(robot)

        self.info("=" * 60)
        self.info("  AUTO-TUNE: MOTION CONTROLLERS (Phase 7)")
        self.info(f"  Parameters: {', '.join(params)}")
        self.info("=" * 60)

        self.results = await _run_in_executor(lambda: tuner.run_motion(params, MotionTuneConfig()))

        if self.persist and self.results and _persist_motion_config(self.results):
            self.info("  Saved to raccoon.project.yml (robot.motion_pid)")

        self.info("\n" + "=" * 60)
        self.info("  MOTION TUNE RESULTS")
        self.info("=" * 60)
        for name, r in self.results.items():
            improvement = (
                (1 - r.final_score / r.initial_score) * 100 if r.initial_score > 0 else 0.0
            )
            self.info(
                f"  {name}: kp {r.initial_kp:.4f}->{r.final_kp:.4f}, "
                f"kd {r.initial_kd:.4f}->{r.final_kd:.4f} | "
                f"score: {r.initial_score:.4f}->{r.final_score:.4f} "
                f"({improvement:+.1f}%) | {r.iterations} iters"
            )


# ---------------------------------------------------------------------------
# Confirm messages (extended for phases 4-6)
# ---------------------------------------------------------------------------

_CONFIRM_MESSAGES: dict[str, str] = {
    # Phase 1 — vel LPF
    "vel_lpf": "Phase 1 – vel LPF: Motoren kurz drehen, BEMF samplen",
    # Phase 2 — static friction
    "static_friction": "Phase 2 – static friction: Motoren einzeln auf Losbrechmoment testen",
    # Phase 3 — firmware PID
    "firmware_pid": "Phase 3 – firmware PID: Motoren kurz drehen (BEMF step response)",
    # Phase 4 — encoder calibration
    "encoder_cal": "Phase 4 – encoder cal: ~720° Drehplatz freimachen (IMU als Referenz)",
    # Phase 5 — Characterize
    "char_forward": "Phase 5 – forward: ~1.5 m vorwärts freimachen",
    "char_lateral": "Phase 5 – lateral: ~1.5 m seitlich rechts freimachen",
    "char_angular": "Phase 5 – angular: 360°+ Drehplatz freimachen",
    # Phase 6 — Velocity
    "vel_vx": "Phase 6 – vx: ~0.5 m vorwärts freimachen",
    "vel_vy": "Phase 6 – vy: ~0.5 m seitlich rechts freimachen",
    "vel_wz": "Phase 6 – wz: 270° Drehplatz freimachen",
    # Phase 7 — Motion
    "motion_distance": "Phase 7 – distance: ~0.5 m vorwärts (mehrfach)",
    "motion_lateral": "Phase 7 – lateral: ~0.5 m seitlich (mehrfach)",
    "motion_heading": "Phase 7 – heading: 90° Drehung (mehrfach)",
}


@dsl_step(tags=["motion", "calibration", "auto-tune"])
class AutoTune(UIStep):
    """Run the full auto-tune pipeline.

    Drives the C++ ``AutoTuner`` one phase at a time so UI confirmations can
    pause between phases. Each phase reads its inputs from the live drive /
    motors / motion-config objects and writes back to those same objects, so
    state stays coherent without Python having to shuttle calibration values.

    Phase order (each depends on the previous):

    1. vel_lpf — per-motor IIR alpha for velocity feedback
    2. static_friction — kS per motor
    3. firmware_pid — STM32 MAV-mode inner loop
    4. encoder_cal — ticks_to_rad scaling via IMU ground truth
    5. characterize — physical drive limits
    6. velocity — chassis-axis velocity PID
    7. motion — distance / heading PID
    8. tolerances — derived from motion-trial residuals

    Args:
        vel_axes: Override the auto-detected velocity axis list for Phase 6.
        characterize_axes: Override the auto-detected axis list for Phase 5.
        motion_axes: Override the auto-detected motion-parameter list for Phase 7.
        tune_vel_lpf: Enable Phase 1 (vel LPF alpha). Default ``True``.
        tune_static_friction: Enable Phase 2 (static friction). Default ``True``.
        tune_firmware_pid: Enable Phase 3 (firmware PID). Default ``False``.
        tune_encoder_cal: Enable Phase 4 (encoder calibration). Default ``True``.
        tune_characterize: Enable Phase 5 (drive characterization). Default ``True``.
        tune_velocity: Enable Phase 6 (velocity PID). Default ``True``.
        tune_motion: Enable Phase 7 (motion PID). Default ``True``.
        tune_tolerances: Enable Phase 8 (tolerance derivation). Default ``True``.
        characterize_trials: Number of Phase 5 trials per axis. Default ``3``.
        characterize_power_percent: Raw PWM for Phase 5 trials (1–100). Default ``100``.
        persist: Write phase results to ``raccoon.project.yml``. Default ``True``.
        step_confirm: Pause for a button press before every phase. Default ``True``.
    """

    def __init__(
        self,
        vel_axes: list[str] | None = None,
        characterize_axes: list[str] | None = None,
        motion_axes: list[str] | None = None,
        tune_vel_lpf: bool = True,
        tune_static_friction: bool = True,
        tune_firmware_pid: bool = False,
        tune_encoder_cal: bool = True,
        tune_characterize: bool = True,
        tune_velocity: bool = True,
        tune_motion: bool = True,
        tune_tolerances: bool = True,
        characterize_trials: int = 3,
        characterize_power_percent: int = 100,
        persist: bool = True,
        step_confirm: bool = True,
    ):
        super().__init__()
        self.characterize_axes = characterize_axes
        self.vel_axes = vel_axes
        self.motion_axes = motion_axes
        self.tune_vel_lpf = tune_vel_lpf
        self.tune_static_friction = tune_static_friction
        self.tune_firmware_pid = tune_firmware_pid
        self.tune_encoder_cal = tune_encoder_cal
        self.tune_characterize = tune_characterize
        self.tune_velocity = tune_velocity
        self.tune_motion = tune_motion
        self.tune_tolerances = tune_tolerances
        self.characterize_trials = characterize_trials
        self.characterize_power_percent = characterize_power_percent
        self.persist = persist
        self.step_confirm = step_confirm

    def _generate_signature(self) -> str:
        return (
            f"AutoTune(vel_lpf={self.tune_vel_lpf}, "
            f"static={self.tune_static_friction}, "
            f"fw_pid={self.tune_firmware_pid}, "
            f"enc_cal={self.tune_encoder_cal}, "
            f"char={self.tune_characterize}, "
            f"vel={self.tune_velocity}, "
            f"motion={self.tune_motion}, "
            f"tol={self.tune_tolerances})"
        )

    async def _confirm(self, key: str) -> None:
        if self.step_confirm:
            msg = _CONFIRM_MESSAGES.get(key, key)
            await self.wait_for_button(msg)

    async def _phase(self, message: str, fn):
        """Run a blocking C++ tuner call in an executor while showing a progress screen.

        ``message`` is displayed on the robot's screen for the duration of the call.
        Returns whatever ``fn()`` returns.
        """
        from raccoon.ui.screens.basic import ProgressScreen

        async def runner():
            return await _run_in_executor(fn)

        return await self.run_with_ui(ProgressScreen(message), runner)

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping full auto-tune, using stored values")
            return

        from raccoon.autotune import (
            EncoderCalConfig,
            FirmwarePidConfig,
            MotionTuneConfig,
            StaticFrictionConfig,
            ToleranceConfig,
            VelLpfConfig,
            VelocityTuneConfig,
        )

        has_lateral = robot.drive.supports_lateral_motion()

        characterize_axes = self.characterize_axes or (
            ["forward", "lateral", "angular"] if has_lateral else ["forward", "angular"]
        )
        velocity_axes = self.vel_axes or (["vx", "vy", "wz"] if has_lateral else ["vx", "wz"])
        motion_params = self.motion_axes or (
            ["distance", "lateral", "heading"] if has_lateral else ["distance", "heading"]
        )

        if not has_lateral:
            self.info("  Kinematics does not support lateral motion — skipping lateral axes")

        active_phases = []
        if self.tune_vel_lpf:
            active_phases.append("vel_lpf")
        if self.tune_static_friction:
            active_phases.append("static friction")
        if self.tune_firmware_pid:
            active_phases.append("firmware PID")
        if self.tune_encoder_cal:
            active_phases.append("encoder cal")
        if self.tune_characterize:
            active_phases.append("characterize")
        if self.tune_velocity:
            active_phases.append("velocity PID")
        if self.tune_motion:
            active_phases.append("motion PID")
        if self.tune_tolerances:
            active_phases.append("tolerances")

        self.info("=" * 60)
        self.info("  AUTO-TUNE PID CONTROLLERS")
        self.info(f"  Phases: {' -> '.join(active_phases)}")
        self.info("=" * 60)

        tuner = _make_tuner(robot)

        # -------- Phase 1: vel_lpf --------
        if self.tune_vel_lpf:
            await self._confirm("vel_lpf")
            res = await self._phase(
                "Phase 1/8 – vel LPF\nMotoren samplen, alpha tunen…",
                lambda: tuner.run_vel_lpf(VelLpfConfig()),
            )
            if self.persist and res:
                _persist_vel_lpf(res)

        # -------- Phase 2: static_friction --------
        if self.tune_static_friction:
            await self._confirm("static_friction")
            await self._phase(
                "Phase 2/8 – static friction\nLosbrechmoment messen…",
                lambda: tuner.run_static_friction(StaticFrictionConfig()),
            )

        # -------- Phase 3: firmware_pid --------
        if self.tune_firmware_pid:
            await self._confirm("firmware_pid")
            fw_cfg = FirmwarePidConfig()
            fw_cfg.csv_dir = "/tmp/auto_tune"
            await self._phase(
                f"Phase 3/8 – firmware PID\nBEMF step response per Motor…\nCSV: {fw_cfg.csv_dir}",
                lambda: tuner.run_firmware_pid(fw_cfg, None),
            )

        # -------- Phase 4: encoder_cal (ticks_to_rad) --------
        if self.tune_encoder_cal:
            await self._confirm("encoder_cal")
            enc = await self._phase(
                "Phase 4/8 – encoder cal\nticks_to_rad via IMU…",
                lambda: tuner.run_encoder_calibration(EncoderCalConfig()),
            )
            if self.persist and getattr(enc, "success", False):
                _persist_ticks_to_rad(enc)

        # -------- Phase 5: characterize --------
        if self.tune_characterize:
            for axis in characterize_axes:
                await self._confirm(f"char_{axis}")
                char_step = CharacterizeDrive(
                    axes=[axis],
                    trials=self.characterize_trials,
                    power_percent=self.characterize_power_percent,
                    accel_timeout=3.0,
                    decel_timeout=3.0,
                    persist=self.persist,
                )
                await char_step._execute_step(robot)

        # -------- Phase 6: velocity --------
        motion_results: dict = {}
        if self.tune_velocity:
            for axis in velocity_axes:
                await self._confirm(f"vel_{axis}")
                max_velocities = {axis: _get_max_velocity(robot, axis)}
                results = await self._phase(
                    f"Phase 6/8 – velocity PID ({axis})\nStep response + CHR tune…",
                    lambda ax=axis, mv=max_velocities: tuner.run_velocity(
                        [ax], mv, VelocityTuneConfig()
                    ),
                )
                if self.persist and any(r.accepted for r in results.values()):
                    _persist_velocity_config(results)

        # -------- Phase 7: motion --------
        if self.tune_motion:
            for param in motion_params:
                await self._confirm(f"motion_{param}")
                results = await self._phase(
                    f"Phase 7/8 – motion PID ({param})\nHooke-Jeeves Trials…",
                    lambda p=param: tuner.run_motion([p], MotionTuneConfig()),
                )
                motion_results.update(results)
            if self.persist and motion_results:
                _persist_motion_config(motion_results)

        # -------- Phase 8: tolerances --------
        if self.tune_tolerances and motion_results:
            await self._phase(
                "Phase 8/8 – tolerances\nResiduen auswerten…",
                lambda: tuner.run_tolerances(motion_results, MotionTuneConfig(), ToleranceConfig()),
            )

        self.info("\n" + "=" * 60)
        self.info("  AUTO-TUNE COMPLETE")
        self.info("=" * 60)
