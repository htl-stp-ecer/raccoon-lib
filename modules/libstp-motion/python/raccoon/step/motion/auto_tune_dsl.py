"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: auto_tune.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .auto_tune import (
    AutoTuneVelLpf,
    AutoTuneStaticFriction,
    AutoTuneBemfVelocity,
    AutoTuneFirmwarePid,
    AutoTuneVelocity,
    AutoTuneMotion,
    AutoTune,
)


class AutoTuneVelLpfBuilder(StepBuilder):
    """Builder for AutoTuneVelLpf. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._persist = True

    def persist(self, value: bool):
        self._persist = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["persist"] = self._persist
        return AutoTuneVelLpf(**kwargs)


@dsl(tags=["motion", "calibration", "auto-tune"])
def auto_tune_vel_lpf(persist: bool = True):
    """
    Tune the IIR velocity-filter alpha per motor (Phase 1).

    Collects raw BEMF samples at a steady velocity, replays them through IIR
    low-pass filters with varying alpha, and applies the alpha that minimises
    a weighted noise+lag score. Runs first in the full pipeline because every
    downstream phase relies on the same velocity-feedback filter.

    Args:
        persist: Write tuned ``vel_lpf_alpha`` values to ``raccoon.project.yml``. Default ``True``.

    Returns:
        A AutoTuneVelLpfBuilder (chainable via ``.persist()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import auto_tune_vel_lpf

        auto_tune_vel_lpf()
    """
    b = AutoTuneVelLpfBuilder()
    b._persist = persist
    return b


class AutoTuneStaticFrictionBuilder(StepBuilder):
    """Builder for AutoTuneStaticFriction. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._persist = True

    def persist(self, value: bool):
        self._persist = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["persist"] = self._persist
        return AutoTuneStaticFriction(**kwargs)


@dsl(tags=["calibration", "auto-tune"])
def auto_tune_static_friction(persist: bool = True):
    """
    Measure per-motor static-friction threshold kS in PWM percent (Phase 2).

    For each drive motor the PWM is swept from a low starting percentage upward
    in both directions. The first PWM level where the median BEMF exceeds a
    motion threshold is recorded as kS.

    Args:
        persist: Reserved for future YAML persistence. Currently logs only.

    Returns:
        A AutoTuneStaticFrictionBuilder (chainable via ``.persist()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import auto_tune_static_friction

        auto_tune_static_friction()
    """
    b = AutoTuneStaticFrictionBuilder()
    b._persist = persist
    return b


class AutoTuneBemfVelocityBuilder(StepBuilder):
    """Builder for AutoTuneBemfVelocity. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._persist = True
        self._pwm_min_percent = 30
        self._pwm_max_percent = 90
        self._pwm_steps = 6
        self._sweeps = 3

    def persist(self, value: bool):
        self._persist = value
        return self

    def pwm_min_percent(self, value: int):
        self._pwm_min_percent = value
        return self

    def pwm_max_percent(self, value: int):
        self._pwm_max_percent = value
        return self

    def pwm_steps(self, value: int):
        self._pwm_steps = value
        return self

    def sweeps(self, value: int):
        self._sweeps = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["persist"] = self._persist
        kwargs["pwm_min_percent"] = self._pwm_min_percent
        kwargs["pwm_max_percent"] = self._pwm_max_percent
        kwargs["pwm_steps"] = self._pwm_steps
        kwargs["sweeps"] = self._sweeps
        return AutoTuneBemfVelocity(**kwargs)


@dsl(tags=["calibration", "auto-tune"])
def auto_tune_bemf_velocity(
    persist: bool = True,
    pwm_min_percent: int = 30,
    pwm_max_percent: int = 90,
    pwm_steps: int = 6,
    sweeps: int = 3,
):
    """
    Calibrate per-motor ``ticks_to_rad`` (BEMF→rad) against the calibration board.

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
        persist: Write tuned ``ticks_to_rad`` per motor to ``raccoon.project.yml``. Default ``True``.
        pwm_min_percent: Lowest PWM level in the sweep (percent). Default ``30``.
        pwm_max_percent: Highest PWM level in the sweep (percent). Default ``90``.
        pwm_steps: Number of evenly spaced PWM levels. Default ``6``.
        sweeps: Number of full sweeps to run; points from all sweeps are pooled into one per-motor fit. More sweeps stabilise the extrapolated ``bemf_offset`` (the ω=0 intercept is noise-sensitive). Default ``3``.

    Returns:
        A AutoTuneBemfVelocityBuilder (chainable via ``.persist()``, ``.pwm_min_percent()``, ``.pwm_max_percent()``, ``.pwm_steps()``, ``.sweeps()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import auto_tune_bemf_velocity

        auto_tune_bemf_velocity()
    """
    b = AutoTuneBemfVelocityBuilder()
    b._persist = persist
    b._pwm_min_percent = pwm_min_percent
    b._pwm_max_percent = pwm_max_percent
    b._pwm_steps = pwm_steps
    b._sweeps = sweeps
    return b


class AutoTuneFirmwarePidBuilder(StepBuilder):
    """Builder for AutoTuneFirmwarePid. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._persist = True
        self._max_bemf_speeds = None
        self._csv_dir = "/tmp/auto_tune"

    def persist(self, value: bool):
        self._persist = value
        return self

    def max_bemf_speeds(self, value: dict[int, int] | None):
        self._max_bemf_speeds = value
        return self

    def csv_dir(self, value: str | None):
        self._csv_dir = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["persist"] = self._persist
        kwargs["max_bemf_speeds"] = self._max_bemf_speeds
        kwargs["csv_dir"] = self._csv_dir
        return AutoTuneFirmwarePid(**kwargs)


@dsl(tags=["calibration", "auto-tune"])
def auto_tune_firmware_pid(
    persist: bool = True,
    max_bemf_speeds: dict[int, int] | None = None,
    csv_dir: str | None = "/tmp/auto_tune",
):
    """
    Tune per-motor STM32 MAV-mode velocity PID via BEMF step response (Phase 3).

    For each drive motor: record a BEMF step response, fit a FOPDT plant,
    derive CHR PID gains, push them to the firmware. Gains are accepted only
    when the tuned ISE is strictly smaller than the baseline ISE.

    Args:
        persist: Unused — gains are applied directly to firmware state.
        max_bemf_speeds: Optional ``{port: max_bemf_speed}`` map. If unset, the C++ side runs a brief power sweep to estimate it.
        csv_dir: When set, every step-response sample is dumped to a CSV under this directory (one per motor + phase) plus a summary CSV with the plant fit and gains. Default ``"/tmp/auto_tune"``.

    Returns:
        A AutoTuneFirmwarePidBuilder (chainable via ``.persist()``, ``.max_bemf_speeds()``, ``.csv_dir()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import auto_tune_firmware_pid

        auto_tune_firmware_pid()
    """
    b = AutoTuneFirmwarePidBuilder()
    b._persist = persist
    b._max_bemf_speeds = max_bemf_speeds
    b._csv_dir = csv_dir
    return b


class AutoTuneVelocityBuilder(StepBuilder):
    """Builder for AutoTuneVelocity. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._axes = None
        self._persist = True

    def axes(self, value: list[str] | None):
        self._axes = value
        return self

    def persist(self, value: bool):
        self._persist = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["axes"] = self._axes
        kwargs["persist"] = self._persist
        return AutoTuneVelocity(**kwargs)


@dsl(tags=["motion", "calibration", "auto-tune"])
def auto_tune_velocity(axes: list[str] | None = None, persist: bool = True):
    """
    Calibrate the MCU chassis velocity-command gain per axis (Phase 6).

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
        axes: Velocity axes to tune (``"vx"``, ``"vy"``, ``"wz"``). Default auto-detects from kinematics.
        persist: Write accepted gains to ``raccoon.project.yml`` (``robot.drive.kinematics.velocity_command_gain``). Default ``True``.

    Returns:
        A AutoTuneVelocityBuilder (chainable via ``.axes()``, ``.persist()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import auto_tune_velocity

        auto_tune_velocity()
        auto_tune_velocity(axes=["vx"])
    """
    b = AutoTuneVelocityBuilder()
    b._axes = axes
    b._persist = persist
    return b


class AutoTuneMotionBuilder(StepBuilder):
    """Builder for AutoTuneMotion. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._axes = None
        self._persist = True

    def axes(self, value: list[str] | None):
        self._axes = value
        return self

    def persist(self, value: bool):
        self._persist = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["axes"] = self._axes
        kwargs["persist"] = self._persist
        return AutoTuneMotion(**kwargs)


@dsl(tags=["motion", "calibration", "auto-tune"])
def auto_tune_motion(axes: list[str] | None = None, persist: bool = True):
    """
    Tune motion PID controllers via iterative real-world optimization (Phase 7).

    Uses Hooke-Jeeves coordinate descent on the distance/heading PID kp & kd
    via real LinearMotion and TurnMotion trials with constraint-aware scoring.

    Prerequisites: velocity controllers tuned (Phase 6) and drive limits
    characterized (Phase 5).

    Args:
        axes: Motion parameters to tune (``"distance"``, ``"lateral"``, ``"heading"``). Default auto-detects from kinematics.
        persist: Write final gains to ``raccoon.project.yml``. Default ``True``.

    Returns:
        A AutoTuneMotionBuilder (chainable via ``.axes()``, ``.persist()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import auto_tune_motion

        auto_tune_motion()
        auto_tune_motion(axes=["heading"])
    """
    b = AutoTuneMotionBuilder()
    b._axes = axes
    b._persist = persist
    return b


class AutoTuneBuilder(StepBuilder):
    """Builder for AutoTune. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._vel_axes = None
        self._characterize_axes = None
        self._motion_axes = None
        self._tune_bemf_velocity = True
        self._tune_vel_lpf = True
        self._tune_static_friction = True
        self._tune_firmware_pid = True
        self._tune_encoder_cal = False
        self._tune_characterize = False
        self._tune_velocity = False
        self._tune_motion = False
        self._tune_tolerances = False
        self._pwm_min_percent = 30
        self._pwm_max_percent = 90
        self._pwm_steps = 6
        self._sweeps = 2
        self._characterize_trials = 3
        self._characterize_power_percent = 100
        self._persist = True
        self._step_confirm = True

    def vel_axes(self, value: list[str] | None):
        self._vel_axes = value
        return self

    def characterize_axes(self, value: list[str] | None):
        self._characterize_axes = value
        return self

    def motion_axes(self, value: list[str] | None):
        self._motion_axes = value
        return self

    def tune_bemf_velocity(self, value: bool):
        self._tune_bemf_velocity = value
        return self

    def tune_vel_lpf(self, value: bool):
        self._tune_vel_lpf = value
        return self

    def tune_static_friction(self, value: bool):
        self._tune_static_friction = value
        return self

    def tune_firmware_pid(self, value: bool):
        self._tune_firmware_pid = value
        return self

    def tune_encoder_cal(self, value: bool):
        self._tune_encoder_cal = value
        return self

    def tune_characterize(self, value: bool):
        self._tune_characterize = value
        return self

    def tune_velocity(self, value: bool):
        self._tune_velocity = value
        return self

    def tune_motion(self, value: bool):
        self._tune_motion = value
        return self

    def tune_tolerances(self, value: bool):
        self._tune_tolerances = value
        return self

    def pwm_min_percent(self, value: int):
        self._pwm_min_percent = value
        return self

    def pwm_max_percent(self, value: int):
        self._pwm_max_percent = value
        return self

    def pwm_steps(self, value: int):
        self._pwm_steps = value
        return self

    def sweeps(self, value: int):
        self._sweeps = value
        return self

    def characterize_trials(self, value: int):
        self._characterize_trials = value
        return self

    def characterize_power_percent(self, value: int):
        self._characterize_power_percent = value
        return self

    def persist(self, value: bool):
        self._persist = value
        return self

    def step_confirm(self, value: bool):
        self._step_confirm = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["vel_axes"] = self._vel_axes
        kwargs["characterize_axes"] = self._characterize_axes
        kwargs["motion_axes"] = self._motion_axes
        kwargs["tune_bemf_velocity"] = self._tune_bemf_velocity
        kwargs["tune_vel_lpf"] = self._tune_vel_lpf
        kwargs["tune_static_friction"] = self._tune_static_friction
        kwargs["tune_firmware_pid"] = self._tune_firmware_pid
        kwargs["tune_encoder_cal"] = self._tune_encoder_cal
        kwargs["tune_characterize"] = self._tune_characterize
        kwargs["tune_velocity"] = self._tune_velocity
        kwargs["tune_motion"] = self._tune_motion
        kwargs["tune_tolerances"] = self._tune_tolerances
        kwargs["pwm_min_percent"] = self._pwm_min_percent
        kwargs["pwm_max_percent"] = self._pwm_max_percent
        kwargs["pwm_steps"] = self._pwm_steps
        kwargs["sweeps"] = self._sweeps
        kwargs["characterize_trials"] = self._characterize_trials
        kwargs["characterize_power_percent"] = self._characterize_power_percent
        kwargs["persist"] = self._persist
        kwargs["step_confirm"] = self._step_confirm
        return AutoTune(**kwargs)


@dsl(tags=["motion", "calibration", "auto-tune"])
def auto_tune(
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
    """
    Run the full auto-tune pipeline.

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
        tune_bemf_velocity: Enable BEMF→velocity ``ticks_to_rad`` calibration against the calibration board. Default ``True``.
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

    Returns:
        A AutoTuneBuilder (chainable via ``.vel_axes()``, ``.characterize_axes()``, ``.motion_axes()``, ``.tune_bemf_velocity()``, ``.tune_vel_lpf()``, ``.tune_static_friction()``, ``.tune_firmware_pid()``, ``.tune_encoder_cal()``, ``.tune_characterize()``, ``.tune_velocity()``, ``.tune_motion()``, ``.tune_tolerances()``, ``.pwm_min_percent()``, ``.pwm_max_percent()``, ``.pwm_steps()``, ``.sweeps()``, ``.characterize_trials()``, ``.characterize_power_percent()``, ``.persist()``, ``.step_confirm()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import auto_tune

        auto_tune()
    """
    b = AutoTuneBuilder()
    b._vel_axes = vel_axes
    b._characterize_axes = characterize_axes
    b._motion_axes = motion_axes
    b._tune_bemf_velocity = tune_bemf_velocity
    b._tune_vel_lpf = tune_vel_lpf
    b._tune_static_friction = tune_static_friction
    b._tune_firmware_pid = tune_firmware_pid
    b._tune_encoder_cal = tune_encoder_cal
    b._tune_characterize = tune_characterize
    b._tune_velocity = tune_velocity
    b._tune_motion = tune_motion
    b._tune_tolerances = tune_tolerances
    b._pwm_min_percent = pwm_min_percent
    b._pwm_max_percent = pwm_max_percent
    b._pwm_steps = pwm_steps
    b._sweeps = sweeps
    b._characterize_trials = characterize_trials
    b._characterize_power_percent = characterize_power_percent
    b._persist = persist
    b._step_confirm = step_confirm
    return b


__all__ = [
    "AutoTuneVelLpfBuilder",
    "auto_tune_vel_lpf",
    "AutoTuneStaticFrictionBuilder",
    "auto_tune_static_friction",
    "AutoTuneBemfVelocityBuilder",
    "auto_tune_bemf_velocity",
    "AutoTuneFirmwarePidBuilder",
    "auto_tune_firmware_pid",
    "AutoTuneVelocityBuilder",
    "auto_tune_velocity",
    "AutoTuneMotionBuilder",
    "auto_tune_motion",
    "AutoTuneBuilder",
    "auto_tune",
]
