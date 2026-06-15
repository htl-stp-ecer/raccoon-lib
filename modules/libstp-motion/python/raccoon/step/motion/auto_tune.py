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
from raccoon.ui import (
    Card,
    Divider,
    ProgressSpinner,
    Row,
    Text,
    UIScreen,
    UIStep,
    Widget,
)

from .. import Step
from ..annotation import dsl_step
from .characterize_drive import CharacterizeDrive

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# Custom per-phase progress screen
# ---------------------------------------------------------------------------


class AutoTuneProgressScreen(UIScreen[None]):
    """Live checklist of the auto-tune pipeline.

    Shows every active phase with a status glyph so the operator can follow the
    pipeline's progress on the robot screen. Reused across all phases of a single
    :class:`AutoTune` run — the orchestrator mutates this object via :meth:`mark`
    and re-displays it for each phase, so the checklist persists.
    """

    title = "Auto-Tune"

    def __init__(self, phases: list[tuple[str, str]]):
        super().__init__()
        self._phases = list(phases)
        self.status: dict[str, str] = {key: "pending" for key, _ in self._phases}
        self.detail: str = ""

    def mark(self, key: str, status: str, detail: str = "") -> None:
        """Update a phase's status (``pending``/``running``/``done``/``failed``/``skipped``)."""
        self.status[key] = status
        self.detail = detail

    def build(self) -> Widget:
        n_total = len(self._phases)
        n_done = sum(1 for s in self.status.values() if s in ("done", "skipped"))

        children: list[Widget] = [
            Text(text=f"Auto-Tune  {n_done}/{n_total}", size="large", bold=True),
            Divider(),
        ]

        for key, label in self._phases:
            status = self.status.get(key, "pending")
            if status == "running":
                glyph: Widget = ProgressSpinner(size="small", color="amber")
            elif status == "done":
                glyph = Text(text="✓", color="green")
            elif status == "failed":
                glyph = Text(text="✗", color="amber")
            elif status == "skipped":
                glyph = Text(text="–", muted=True, color="grey")
            else:  # pending
                glyph = Text(text="○", muted=True, color="grey")

            children.append(
                Row(
                    children=[
                        glyph,
                        Text(
                            text=label,
                            bold=(status == "running"),
                            muted=(status in ("pending", "skipped")),
                        ),
                    ],
                    spacing=8,
                )
            )

        if self.detail:
            children.append(Divider())
            children.append(Text(text=self.detail, size="small", muted=True))

        return Card(title="Auto-Tune", children=children)


# ---------------------------------------------------------------------------
# YAML persistence helpers
# ---------------------------------------------------------------------------


def _persist_velocity_config(results: dict) -> bool:
    """Persist the calibrated MCU chassis velocity-command gains.

    On the MCU-chassis path the per-axis velocity loop runs on the coprocessor
    (forward kinematics + per-wheel MAV PID), so there are no host velocity PID
    gains to persist. The velocity phase instead calibrates a per-axis command
    gain — folded into the STM32 forward-kinematics matrix — so a commanded body
    velocity is actually achieved (drivetrain-efficiency compensation). It is
    written under ``robot.drive.kinematics.velocity_command_gain.{vx,vy,wz}`` and
    re-applied at robot construction via ``set_velocity_command_gains``.
    """
    project_root = find_project_root()
    if project_root is None:
        return False

    updated = False
    for axis_name, result in results.items():
        if not result.accepted:
            continue
        if update_project_value(
            project_root,
            ["robot", "drive", "kinematics", "velocity_command_gain", axis_name],
            round(result.velocity_command_gain, 5),
        ):
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
    """Write tuned per-motor ticks_to_rad (and bemf_offset, when the result
    carries it) to definitions.<motor>.calibration."""
    if not getattr(result, "success", False):
        return False
    project_root = find_project_root()
    if project_root is None:
        return False
    # bemf_offset only exists on the BEMF→velocity result, not the encoder-cal one.
    bemf_offsets = getattr(result, "bemf_offset", None)
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
        if (
            bemf_offsets is not None
            and port < len(bemf_offsets)
            and update_project_value(
                project_root,
                ["definitions", motor_name, "calibration", "bemf_offset"],
                round(bemf_offsets[port], 4),
            )
        ):
            updated = True
    return updated


def _persist_static_friction(results: dict) -> bool:
    """Write measured static-friction kS (PWM percent) per motor to
    definitions.<motor>.calibration.static_friction_pct.

    Only motors that actually moved (``measured``) with a positive ``ks_avg_pct``
    are written; the rest carry no usable threshold.
    """
    project_root = find_project_root()
    if project_root is None:
        return False
    updated = False
    for port, r in results.items():
        if not getattr(r, "measured", False):
            continue
        if r.ks_avg_pct <= 0:
            continue
        motor_name = _motor_name_by_port(project_root, port)
        if motor_name is None:
            continue
        if update_project_value(
            project_root,
            ["definitions", motor_name, "calibration", "static_friction_pct"],
            int(r.ks_avg_pct),
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


def _write_phase_report(phase_name: str, result, step) -> None:
    """Persist a per-phase data+plot report; never let it abort the tune."""
    from ._autotune_report import write_report

    try:
        write_report(phase_name, result, step.info)
    except Exception as exc:  # reporting must never fail a tune
        step.warn(f"  report generation failed ({phase_name}): {exc}")


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

        _write_phase_report("vel_lpf", results, self)

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

        _write_phase_report("static_friction", results, self)


@dsl_step(tags=["calibration", "auto-tune"])
class AutoTuneBemfVelocity(Step):
    """Calibrate per-motor ``ticks_to_rad`` (BEMF→rad) against the calibration board.

    Fully automatic. Drives the chassis straight forward at a sweep of open-loop
    PWM levels (back-and-forth, staying near the start) and, for each level,
    compares the ground-truth distance travelled — read from the external
    calibration board's optical-flow + IMU odometry — against the accumulated
    BEMF ticks per motor. From that it derives, per motor,
    ``ticks_to_rad = (distance / wheel_radius) / Δticks``.

    Crucially it does **not** assume the ADC-BEMF↔velocity relationship is
    linear: it computes the per-motor coefficient of variation of
    ``ticks_to_rad`` across the speed range plus an ω-vs-BEMF linear fit
    (slope/intercept/R²) and reports whether a single scale actually holds. If
    the relationship is clearly curved or offset, that is logged as a warning
    rather than silently persisting a misleading single value.

    Prerequisites:
        - The calibration board must be connected and backing the odometry pose
          (``robot.odometry.get_active_source() == CALIBRATION_BOARD``); the
          tuner aborts otherwise.
        - Roughly 1 m of clear runway forward/back.

    Args:
        persist: Write tuned ``ticks_to_rad`` per motor to ``raccoon.project.yml``.
            Default ``True``.
        pwm_min_percent: Lowest PWM level in the sweep (percent). Default ``30``.
        pwm_max_percent: Highest PWM level in the sweep (percent). Default ``90``.
        pwm_steps: Number of evenly spaced PWM levels. Default ``6``.
        sweeps: Number of full sweeps to run; points from all sweeps are pooled
            into one per-motor fit. More sweeps stabilise the extrapolated
            ``bemf_offset`` (the ω=0 intercept is noise-sensitive). Default ``3``.

    Returns:
        :class:`AutoTuneBemfVelocity` step.

    Example::

        from raccoon.step.motion import auto_tune_bemf_velocity

        auto_tune_bemf_velocity()
    """

    def __init__(
        self,
        persist: bool = True,
        pwm_min_percent: int = 30,
        pwm_max_percent: int = 90,
        pwm_steps: int = 6,
        sweeps: int = 3,
    ):
        super().__init__()
        self.persist = persist
        self.pwm_min_percent = pwm_min_percent
        self.pwm_max_percent = pwm_max_percent
        self.pwm_steps = pwm_steps
        self.sweeps = sweeps

    def _generate_signature(self) -> str:
        return (
            f"AutoTuneBemfVelocity(persist={self.persist}, "
            f"pwm={self.pwm_min_percent}-{self.pwm_max_percent}x{self.pwm_steps}, "
            f"sweeps={self.sweeps})"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping BEMF→velocity calibration")
            return

        from raccoon.autotune import BemfVelocityConfig, BemfVelocityTuner

        self.info("=" * 60)
        self.info("  AUTO-TUNE: BEMF→VELOCITY (calibration-board ground truth)")
        self.info("=" * 60)

        cfg = BemfVelocityConfig()
        cfg.pwm_min_percent = self.pwm_min_percent
        cfg.pwm_max_percent = self.pwm_max_percent
        cfg.pwm_steps = self.pwm_steps
        cfg.sweeps = self.sweeps
        cfg.apply = True
        cfg.require_calib_board = True

        tuner = BemfVelocityTuner(robot.drive, robot.odometry)
        result = await _run_in_executor(lambda: tuner.tune(cfg))

        if not result.success:
            self.warn(f"  BEMF→velocity calibration failed: {result.failure_reason}")
            return

        for fit in result.motors:
            if fit.n_points == 0:
                continue
            self.info(
                f"  port={fit.port}: ticks_to_rad={fit.ticks_to_rad_median:.6g} "
                f"(CV={100.0 * fit.ticks_to_rad_cv:.1f}% over {fit.n_points} speeds) | "
                f"ω={fit.bemf_omega_slope:.4g}·bemf+{fit.bemf_omega_intercept:.4g} "
                f"R²={fit.bemf_omega_r2:.4f} -> "
                f"{'LINEAR' if fit.linear else 'NON-LINEAR'}"
            )

        if not result.linear_overall:
            self.warn(
                "  BEMF↔velocity is NOT well described by a single ticks_to_rad "
                "across the speed range — a single scale will be imprecise "
                "(the firmware integration likely needs a speed-dependent model)."
            )

        # Persist data + plots under .raccoon/auto_tune/bemf_velocity/<ts>/.
        from ._bemf_report import write_report

        try:
            write_report(result, self.info)
        except Exception as exc:  # never let reporting abort the tune
            self.warn(f"  report generation failed: {exc}")

        if self.persist and _persist_ticks_to_rad(result):
            self.info("  Saved ticks_to_rad to raccoon.project.yml")


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

        if results:
            _write_phase_report(
                "firmware_pid",
                {"csv_dir": self.csv_dir, "results": results},
                self,
            )


@dsl_step(tags=["motion", "calibration", "auto-tune"])
class AutoTuneVelocity(Step):
    """Calibrate the MCU chassis velocity-command gain per axis (Phase 6).

    With the chassis velocity loop running on the coprocessor (forward
    kinematics + per-wheel MAV PID), there is no host velocity PID left to tune.
    What this phase tunes instead is the chassis-level command ACCURACY: it
    commands a mid-range body velocity, measures the achieved velocity against
    external ground truth (the calib board), and folds a per-axis correction
    gain into the STM32 forward-kinematics matrix so commanded == achieved. This
    compensates drivetrain efficiency the ideal geometry ignores (most notably
    mecanum roller slip, where wheels track their BEMF setpoint correctly yet
    the chassis travels less than predicted). The candidate gain is validated by
    re-measuring and accepted only if the effective gain moved closer to 1.0.

    Prerequisites: Phase 5 (drive characterization) should have run first so a
    max-velocity-per-axis is known. Phases 1–4 (LPF, static friction, firmware
    MAV PID, ticks_to_rad) should also have run so the inner loop is stable.
    A calib board must be connected for the external measurement.

    Args:
        axes: Velocity axes to tune (``"vx"``, ``"vy"``, ``"wz"``).
            Default auto-detects from kinematics.
        persist: Write accepted gains to ``raccoon.project.yml``
            (``robot.drive.kinematics.velocity_command_gain``). Default ``True``.

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
        self.info(f"\n  Applied {accepted_count} axis command gains in-memory (pushed to STM32)")

        _write_phase_report("velocity", self.results, self)

        if self.persist and any(r.accepted for r in self.results.values()):
            if _persist_velocity_config(self.results):
                self.info(
                    "  Saved to raccoon.project.yml "
                    "(robot.drive.kinematics.velocity_command_gain)"
                )
            else:
                self.warn("  Failed to save to raccoon.project.yml")

        self.info("\n" + "=" * 60)
        self.info("  VELOCITY GAIN CALIBRATION RESULTS")
        self.info("=" * 60)
        for axis, r in self.results.items():
            status = "ACCEPTED" if r.accepted else "REVERTED"
            self.info(
                f"  {axis}: {status} | effective gain "
                f"{r.measured_gain_before:.3f} -> {r.measured_gain_after:.3f} "
                f"(target 1.0) | command gain={r.velocity_command_gain:.4f}"
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
    # ticks_to_rad via calibration board
    "bemf_velocity": "ticks: ~1 m vorwärts/rückwärts frei (Calib-Board als Referenz)",
    # vel LPF
    "vel_lpf": "Phase 1 – vel LPF: Motoren kurz drehen, BEMF samplen",
    # Phase 2 — static friction
    "static_friction": "Phase 2 – static friction: Motoren einzeln auf Losbrechmoment testen",
    # Phase 3 — encoder calibration
    "encoder_cal": "Phase 3 – encoder cal: ~720° Drehplatz freimachen (IMU als Referenz)",
    # Phase 4 — firmware PID
    "firmware_pid": "Phase 4 – firmware PID: Motoren kurz drehen (BEMF step response)",
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

    Only the phases that are currently validated run by default. The remaining
    phases are kept but disabled — pass the matching ``tune_*=True`` to re-enable.

    Default-enabled phases:

    * ``bemf_velocity`` — per-motor ``ticks_to_rad`` against the calibration
      board (the big validated win; supersedes the IMU ``encoder_cal``).
    * ``vel_lpf`` — per-motor IIR alpha for velocity feedback.
    * ``static_friction`` — kS per motor (PWM percent).
    * ``firmware_pid`` — STM32 MAV-mode inner velocity loop.

    Default-disabled phases (re-enable explicitly):

    * ``encoder_cal`` — IMU ticks_to_rad; superseded by ``bemf_velocity``.
    * ``characterize`` — max velocity / accel / decel per axis at 100% PWM,
      measured against calib-board ground truth (frame-independent straight-line
      distance).
    * ``velocity`` — MCU chassis velocity-command gain per axis: makes commanded
      body velocity match the calib-board-measured achieved velocity.
    * ``motion`` — distance / heading PID; untested.
    * ``tolerances`` — derived from motion-trial residuals; untested.

    Args:
        vel_axes: Override the auto-detected velocity axis list.
        characterize_axes: Override the auto-detected characterize axis list.
        motion_axes: Override the auto-detected motion-parameter list.
        tune_bemf_velocity: Enable BEMF→velocity ``ticks_to_rad`` calibration
            against the calibration board. Default ``True``.
        tune_vel_lpf: Enable vel LPF alpha tuning. Default ``True``.
        tune_static_friction: Enable static friction measurement. Default ``True``.
        tune_firmware_pid: Enable firmware velocity PID tuning. Default ``True``.
        tune_encoder_cal: Enable IMU encoder calibration. Default ``False``.
        tune_characterize: Enable drive characterization. Default ``False``.
        tune_velocity: Enable velocity PID tuning. Default ``False``.
        tune_motion: Enable motion PID tuning. Default ``False``.
        tune_tolerances: Enable tolerance derivation. Default ``False``.
        pwm_min_percent: Lowest PWM level for the bemf_velocity sweep. Default ``30``.
        pwm_max_percent: Highest PWM level for the bemf_velocity sweep. Default ``90``.
        pwm_steps: Number of bemf_velocity sweep PWM levels. Default ``6``.
        sweeps: Number of bemf_velocity sweeps to pool. Default ``2``.
        characterize_trials: Number of characterize trials per axis. Default ``3``.
        characterize_power_percent: Raw PWM for characterize trials (1–100). Default ``100``.
        persist: Write phase results to ``raccoon.project.yml``. Default ``True``.
        step_confirm: Pause for a button press before every phase. Default ``True``.
    """

    def __init__(
        self,
        vel_axes: list[str] | None = None,
        characterize_axes: list[str] | None = None,
        motion_axes: list[str] | None = None,
        tune_bemf_velocity: bool = True,
        tune_vel_lpf: bool = True,
        tune_static_friction: bool = True,
        tune_firmware_pid: bool = True,
        tune_encoder_cal: bool = False,
        tune_characterize: bool = False,
        tune_velocity: bool = False,
        tune_motion: bool = False,
        tune_tolerances: bool = False,
        pwm_min_percent: int = 30,
        pwm_max_percent: int = 90,
        pwm_steps: int = 6,
        sweeps: int = 2,
        characterize_trials: int = 3,
        characterize_power_percent: int = 100,
        persist: bool = True,
        step_confirm: bool = True,
    ):
        super().__init__()
        self.characterize_axes = characterize_axes
        self.vel_axes = vel_axes
        self.motion_axes = motion_axes
        self.tune_bemf_velocity = tune_bemf_velocity
        self.tune_vel_lpf = tune_vel_lpf
        self.tune_static_friction = tune_static_friction
        self.tune_firmware_pid = tune_firmware_pid
        self.tune_encoder_cal = tune_encoder_cal
        self.tune_characterize = tune_characterize
        self.tune_velocity = tune_velocity
        self.tune_motion = tune_motion
        self.tune_tolerances = tune_tolerances
        self.pwm_min_percent = pwm_min_percent
        self.pwm_max_percent = pwm_max_percent
        self.pwm_steps = pwm_steps
        self.sweeps = sweeps
        self.characterize_trials = characterize_trials
        self.characterize_power_percent = characterize_power_percent
        self.persist = persist
        self.step_confirm = step_confirm

    def _generate_signature(self) -> str:
        return (
            f"AutoTune(bemf_vel={self.tune_bemf_velocity}, "
            f"vel_lpf={self.tune_vel_lpf}, "
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

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping full auto-tune, using stored values")
            return

        from raccoon.autotune import (
            BemfVelocityConfig,
            BemfVelocityTuner,
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

        # Build the ordered (key, label) list of enabled phases.
        phase_labels: list[tuple[str, str]] = [
            ("bemf_velocity", "ticks_to_rad (calib board)"),
            ("vel_lpf", "velocity LPF alpha"),
            ("static_friction", "static friction kS"),
            ("firmware_pid", "firmware velocity PID"),
            ("encoder_cal", "encoder cal (IMU)"),
            ("characterize", "drive limits"),
            ("velocity", "velocity PID"),
            ("motion", "motion PID"),
            ("tolerances", "tolerances"),
        ]
        enabled = {
            "bemf_velocity": self.tune_bemf_velocity,
            "vel_lpf": self.tune_vel_lpf,
            "static_friction": self.tune_static_friction,
            "firmware_pid": self.tune_firmware_pid,
            "encoder_cal": self.tune_encoder_cal,
            "characterize": self.tune_characterize,
            "velocity": self.tune_velocity,
            "motion": self.tune_motion,
            "tolerances": self.tune_tolerances,
        }
        active_phases = [(k, label) for k, label in phase_labels if enabled[k]]

        self.info("=" * 60)
        self.info("  AUTO-TUNE PID CONTROLLERS")
        self.info(f"  Phases: {' -> '.join(label for _, label in active_phases)}")
        self.info("=" * 60)

        screen = AutoTuneProgressScreen(active_phases)
        tuner = _make_tuner(robot)
        motion_results: dict = {}

        for key, _label in active_phases:
            await self._confirm(key)

            # -------- bemf_velocity --------
            if key == "bemf_velocity":
                screen.mark(key, "running", detail="ticks_to_rad vs calib board…")

                async def runner_bemf():
                    try:
                        cfg = BemfVelocityConfig()
                        cfg.pwm_min_percent = self.pwm_min_percent
                        cfg.pwm_max_percent = self.pwm_max_percent
                        cfg.pwm_steps = self.pwm_steps
                        cfg.sweeps = self.sweeps
                        cfg.apply = True
                        cfg.require_calib_board = True
                        bemf_tuner = BemfVelocityTuner(robot.drive, robot.odometry)
                        result = await _run_in_executor(lambda: bemf_tuner.tune(cfg))
                        if not getattr(result, "success", False):
                            reason = getattr(result, "failure_reason", "unknown")
                            self.warn(f"  bemf_velocity failed: {reason}")
                            screen.mark("bemf_velocity", "failed", detail=str(reason))
                            await screen.refresh()
                            return
                        from ._bemf_report import write_report

                        try:
                            write_report(result, self.info)
                        except Exception as exc:  # reporting must never abort a tune
                            self.warn(f"  bemf_velocity report failed: {exc}")
                        if self.persist:
                            _persist_ticks_to_rad(result)
                        screen.mark("bemf_velocity", "done")
                        await screen.refresh()
                    except Exception as exc:
                        self.warn(f"  bemf_velocity raised: {exc}")
                        screen.mark("bemf_velocity", "failed", detail=str(exc))
                        await screen.refresh()

                await self.run_with_ui(screen, runner_bemf)

            # -------- vel_lpf --------
            elif key == "vel_lpf":
                screen.mark(key, "running", detail="Motoren samplen, alpha tunen…")

                async def runner_vel_lpf():
                    try:
                        res = await _run_in_executor(lambda: tuner.run_vel_lpf(VelLpfConfig()))
                        if res:
                            _write_phase_report("vel_lpf", res, self)
                            if self.persist:
                                _persist_vel_lpf(res)
                            screen.mark("vel_lpf", "done")
                        else:
                            screen.mark("vel_lpf", "failed", detail="no result")
                        await screen.refresh()
                    except Exception as exc:
                        self.warn(f"  vel_lpf raised: {exc}")
                        screen.mark("vel_lpf", "failed", detail=str(exc))
                        await screen.refresh()

                await self.run_with_ui(screen, runner_vel_lpf)

            # -------- static_friction --------
            elif key == "static_friction":
                screen.mark(key, "running", detail="Losbrechmoment messen…")

                async def runner_sf():
                    try:
                        res = await _run_in_executor(
                            lambda: tuner.run_static_friction(StaticFrictionConfig())
                        )
                        if res:
                            _write_phase_report("static_friction", res, self)
                            if self.persist:
                                _persist_static_friction(res)
                            screen.mark("static_friction", "done")
                        else:
                            screen.mark("static_friction", "failed", detail="no result")
                        await screen.refresh()
                    except Exception as exc:
                        self.warn(f"  static_friction raised: {exc}")
                        screen.mark("static_friction", "failed", detail=str(exc))
                        await screen.refresh()

                await self.run_with_ui(screen, runner_sf)

            # -------- firmware_pid --------
            elif key == "firmware_pid":
                screen.mark(key, "running", detail="BEMF step response per Motor…")

                async def runner_fw():
                    try:
                        fw_cfg = FirmwarePidConfig()
                        fw_cfg.csv_dir = "/tmp/auto_tune"
                        res = await _run_in_executor(lambda: tuner.run_firmware_pid(fw_cfg, None))
                        if res:
                            _write_phase_report(
                                "firmware_pid",
                                {"csv_dir": fw_cfg.csv_dir, "results": res},
                                self,
                            )
                            screen.mark("firmware_pid", "done")
                        else:
                            screen.mark("firmware_pid", "failed", detail="no result")
                        await screen.refresh()
                    except Exception as exc:
                        self.warn(f"  firmware_pid raised: {exc}")
                        screen.mark("firmware_pid", "failed", detail=str(exc))
                        await screen.refresh()

                await self.run_with_ui(screen, runner_fw)

            # -------- encoder_cal (IMU ticks_to_rad) --------
            elif key == "encoder_cal":
                screen.mark(key, "running", detail="ticks_to_rad via IMU…")

                async def runner_enc():
                    try:
                        enc = await _run_in_executor(
                            lambda: tuner.run_encoder_calibration(EncoderCalConfig())
                        )
                        if getattr(enc, "success", False):
                            if self.persist:
                                _persist_ticks_to_rad(enc)
                            screen.mark("encoder_cal", "done")
                        else:
                            screen.mark("encoder_cal", "failed", detail="no result")
                        await screen.refresh()
                    except Exception as exc:
                        self.warn(f"  encoder_cal raised: {exc}")
                        screen.mark("encoder_cal", "failed", detail=str(exc))
                        await screen.refresh()

                await self.run_with_ui(screen, runner_enc)

            # -------- characterize --------
            elif key == "characterize":
                screen.mark(key, "running", detail="drive limits messen…")
                ok = True
                for axis in characterize_axes:
                    try:
                        char_step = CharacterizeDrive(
                            axes=[axis],
                            trials=self.characterize_trials,
                            power_percent=self.characterize_power_percent,
                            accel_timeout=3.0,
                            decel_timeout=3.0,
                            persist=self.persist,
                        )
                        await char_step._execute_step(robot)
                    except Exception as exc:
                        ok = False
                        self.warn(f"  characterize[{axis}] raised: {exc}")
                screen.mark("characterize", "done" if ok else "failed")

            # -------- velocity --------
            elif key == "velocity":
                screen.mark(key, "running", detail="Step response + CHR tune…")

                async def runner_vel():
                    try:
                        for axis in velocity_axes:
                            max_velocities = {axis: _get_max_velocity(robot, axis)}
                            results = await _run_in_executor(
                                lambda ax=axis, mv=max_velocities: tuner.run_velocity(
                                    [ax], mv, VelocityTuneConfig()
                                )
                            )
                            if results:
                                _write_phase_report("velocity", results, self)
                                if self.persist and any(r.accepted for r in results.values()):
                                    _persist_velocity_config(results)
                        screen.mark("velocity", "done")
                        await screen.refresh()
                    except Exception as exc:
                        self.warn(f"  velocity raised: {exc}")
                        screen.mark("velocity", "failed", detail=str(exc))
                        await screen.refresh()

                await self.run_with_ui(screen, runner_vel)

            # -------- motion --------
            elif key == "motion":
                screen.mark(key, "running", detail="Hooke-Jeeves Trials…")

                async def runner_motion():
                    try:
                        for param in motion_params:
                            results = await _run_in_executor(
                                lambda p=param: tuner.run_motion([p], MotionTuneConfig())
                            )
                            motion_results.update(results)
                        if self.persist and motion_results:
                            _persist_motion_config(motion_results)
                        screen.mark("motion", "done")
                        await screen.refresh()
                    except Exception as exc:
                        self.warn(f"  motion raised: {exc}")
                        screen.mark("motion", "failed", detail=str(exc))
                        await screen.refresh()

                await self.run_with_ui(screen, runner_motion)

            # -------- tolerances --------
            elif key == "tolerances":
                if not motion_results:
                    screen.mark(key, "skipped", detail="no motion residuals")
                    continue
                screen.mark(key, "running", detail="Residuen auswerten…")

                async def runner_tol():
                    try:
                        await _run_in_executor(
                            lambda: tuner.run_tolerances(
                                motion_results, MotionTuneConfig(), ToleranceConfig()
                            )
                        )
                        screen.mark("tolerances", "done")
                        await screen.refresh()
                    except Exception as exc:
                        self.warn(f"  tolerances raised: {exc}")
                        screen.mark("tolerances", "failed", detail=str(exc))
                        await screen.refresh()

                await self.run_with_ui(screen, runner_tol)

        self.info("\n" + "=" * 60)
        self.info("  AUTO-TUNE COMPLETE")
        self.info("=" * 60)
