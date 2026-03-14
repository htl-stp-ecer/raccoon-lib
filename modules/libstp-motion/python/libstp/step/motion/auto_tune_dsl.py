"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: auto_tune.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .auto_tune import AutoTuneVelocity, AutoTuneMotion, AutoTune


class AutoTuneVelocityBuilder(StepBuilder):
    """Builder for AutoTuneVelocity. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._axes = None
        self._persist = True
        self._csv_dir = '/tmp/auto_tune'

    def axes(self, value: list[str]):
        self._axes = value
        return self

    def persist(self, value: bool):
        self._persist = value
        return self

    def csv_dir(self, value: Optional[str]):
        self._csv_dir = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['axes'] = self._axes
        kwargs['persist'] = self._persist
        kwargs['csv_dir'] = self._csv_dir
        return AutoTuneVelocity(**kwargs)


@dsl(tags=['motion', 'calibration', 'auto-tune'])
def auto_tune_velocity(axes: list[str] = None, persist: bool = True, csv_dir: Optional[str] = '/tmp/auto_tune'):
    """
    Tune velocity controllers via step-response system identification.

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
        axes: Velocity axes to tune. Each entry is a velocity component name: ``"vx"`` (forward), ``"vy"`` (lateral/strafe), or ``"wz"`` (angular). Default ``["vx", "vy", "wz"]``.
        persist: If ``True``, write accepted gains to ``raccoon.project.yml``. Default ``True``.
        csv_dir: Directory for step-response CSV files (baseline and tuned recordings per axis). Default ``"/tmp/auto_tune"``.

    Returns:
        A AutoTuneVelocityBuilder (chainable via ``.axes()``, ``.persist()``, ``.csv_dir()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import auto_tune_velocity

        # Tune both velocity axes with defaults
        auto_tune_velocity()

        # Tune only the forward axis, save CSVs for plotting
        auto_tune_velocity(
            axes=["vx"],
            csv_dir="/tmp/vel_tune_plots",
        )
    """
    b = AutoTuneVelocityBuilder()
    b._axes = axes
    b._persist = persist
    b._csv_dir = csv_dir
    return b


class AutoTuneMotionBuilder(StepBuilder):
    """Builder for AutoTuneMotion. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._axes = None
        self._persist = True
        self._csv_dir = '/tmp/auto_tune'

    def axes(self, value: list[str]):
        self._axes = value
        return self

    def persist(self, value: bool):
        self._persist = value
        return self

    def csv_dir(self, value: Optional[str]):
        self._csv_dir = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['axes'] = self._axes
        kwargs['persist'] = self._persist
        kwargs['csv_dir'] = self._csv_dir
        return AutoTuneMotion(**kwargs)


@dsl(tags=['motion', 'calibration', 'auto-tune'])
def auto_tune_motion(axes: list[str] = None, persist: bool = True, csv_dir: Optional[str] = '/tmp/auto_tune'):
    """
    Tune motion PID controllers via iterative real-world optimization.

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
        axes: Motion parameters to optimize. Options are ``"distance"`` (forward drive kp/kd), ``"lateral"`` (strafe kp/kd — shares the distance PID but optimizes with lateral trials), and ``"heading"`` (turn kp/kd). Default ``["distance", "lateral", "heading"]``.
        persist: If ``True``, write final gains to ``raccoon.project.yml`` under ``robot.motion_pid``. Default ``True``.
        csv_dir: Directory for diagnostic CSV output. Default ``"/tmp/auto_tune"``.

    Returns:
        A AutoTuneMotionBuilder (chainable via ``.axes()``, ``.persist()``, ``.csv_dir()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import auto_tune_motion

        # Tune both distance and heading controllers
        auto_tune_motion()

        # Tune only the heading controller
        auto_tune_motion(axes=["heading"])

        # Tune without persisting (for experimentation)
        auto_tune_motion(persist=False)
    """
    b = AutoTuneMotionBuilder()
    b._axes = axes
    b._persist = persist
    b._csv_dir = csv_dir
    return b


class AutoTuneBuilder(StepBuilder):
    """Builder for AutoTune. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._vel_axes = None
        self._characterize_axes = None
        self._motion_axes = None
        self._tune_characterize = True
        self._tune_velocity = True
        self._tune_motion = True
        self._characterize_trials = 3
        self._characterize_power_percent = 100
        self._persist = True
        self._csv_dir = '/tmp/auto_tune'

    def vel_axes(self, value: list[str]):
        self._vel_axes = value
        return self

    def characterize_axes(self, value: list[str]):
        self._characterize_axes = value
        return self

    def motion_axes(self, value: list[str]):
        self._motion_axes = value
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

    def characterize_trials(self, value: int):
        self._characterize_trials = value
        return self

    def characterize_power_percent(self, value: int):
        self._characterize_power_percent = value
        return self

    def persist(self, value: bool):
        self._persist = value
        return self

    def csv_dir(self, value: Optional[str]):
        self._csv_dir = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['vel_axes'] = self._vel_axes
        kwargs['characterize_axes'] = self._characterize_axes
        kwargs['motion_axes'] = self._motion_axes
        kwargs['tune_characterize'] = self._tune_characterize
        kwargs['tune_velocity'] = self._tune_velocity
        kwargs['tune_motion'] = self._tune_motion
        kwargs['characterize_trials'] = self._characterize_trials
        kwargs['characterize_power_percent'] = self._characterize_power_percent
        kwargs['persist'] = self._persist
        kwargs['csv_dir'] = self._csv_dir
        return AutoTune(**kwargs)


@dsl(tags=['motion', 'calibration', 'auto-tune'])
def auto_tune(vel_axes: list[str] = None, characterize_axes: list[str] = None, motion_axes: list[str] = None, tune_characterize: bool = True, tune_velocity: bool = True, tune_motion: bool = True, characterize_trials: int = 3, characterize_power_percent: int = 100, persist: bool = True, csv_dir: Optional[str] = '/tmp/auto_tune'):
    """
    Auto-tune the full drive system: characterize, velocity PID, motion PID.

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
        vel_axes: Velocity axes to tune in Phase 2. Each entry is a velocity component name (``"vx"`` for forward, ``"vy"`` for lateral/strafe, ``"wz"`` for angular). Default ``["vx", "vy", "wz"]``.
        characterize_axes: Axes to characterize in Phase 1. Options are ``"forward"``, ``"lateral"``, ``"angular"``. Default ``["forward", "lateral", "angular"]``.
        motion_axes: Motion parameters to optimize in Phase 3. Options are ``"distance"``, ``"lateral"`` (shares the distance PID but optimizes with lateral trials), and ``"heading"``. Default ``["distance", "lateral", "heading"]``.
        tune_characterize: Whether to run Phase 1. Set to ``False`` if the robot's limits are already known. Default ``True``.
        tune_velocity: Whether to run Phase 2. Default ``True``.
        tune_motion: Whether to run Phase 3. Default ``True``.
        characterize_trials: Number of trials per axis in Phase 1. More trials improve robustness but take longer. Default 3.
        characterize_power_percent: Motor power percentage (1--100) for Phase 1 drive characterization. Default 100.
        persist: If ``True``, write all results to ``raccoon.project.yml``. Default ``True``.
        csv_dir: Directory for diagnostic CSV output (step-response recordings, etc.). Default ``"/tmp/auto_tune"``.

    Returns:
        A AutoTuneBuilder (chainable via ``.vel_axes()``, ``.characterize_axes()``, ``.motion_axes()``, ``.tune_characterize()``, ``.tune_velocity()``, ``.tune_motion()``, ``.characterize_trials()``, ``.characterize_power_percent()``, ``.persist()``, ``.csv_dir()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import auto_tune

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
    b = AutoTuneBuilder()
    b._vel_axes = vel_axes
    b._characterize_axes = characterize_axes
    b._motion_axes = motion_axes
    b._tune_characterize = tune_characterize
    b._tune_velocity = tune_velocity
    b._tune_motion = tune_motion
    b._characterize_trials = characterize_trials
    b._characterize_power_percent = characterize_power_percent
    b._persist = persist
    b._csv_dir = csv_dir
    return b


__all__ = ['AutoTuneVelocityBuilder', 'auto_tune_velocity', 'AutoTuneMotionBuilder', 'auto_tune_motion', 'AutoTuneBuilder', 'auto_tune']
