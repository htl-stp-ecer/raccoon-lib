"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: characterize_drive.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .characterize_drive import CharacterizeDrive


class CharacterizeDriveBuilder(StepBuilder):
    """Builder for CharacterizeDrive. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._axes = None
        self._trials = 3
        self._power_percent = 100
        self._accel_timeout = 3.0
        self._decel_timeout = 3.0
        self._persist = True

    def axes(self, value: list[str] | None):
        self._axes = value
        return self

    def trials(self, value: int):
        self._trials = value
        return self

    def power_percent(self, value: int):
        self._power_percent = value
        return self

    def accel_timeout(self, value: float):
        self._accel_timeout = value
        return self

    def decel_timeout(self, value: float):
        self._decel_timeout = value
        return self

    def persist(self, value: bool):
        self._persist = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["axes"] = self._axes
        kwargs["trials"] = self._trials
        kwargs["power_percent"] = self._power_percent
        kwargs["accel_timeout"] = self._accel_timeout
        kwargs["decel_timeout"] = self._decel_timeout
        kwargs["persist"] = self._persist
        return CharacterizeDrive(**kwargs)


@dsl(tags=["motion", "calibration", "characterize"])
def characterize_drive(
    axes: list[str] | None = None,
    trials: int = 3,
    power_percent: int = 100,
    accel_timeout: float = 3.0,
    decel_timeout: float = 3.0,
    persist: bool = True,
):
    """
    Characterize the robot's physical drive limits at full motor power.

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
        axes: Which axes to characterize. Options are ``"forward"``, ``"lateral"``, and ``"angular"``. Default ``["forward"]``.
        trials: Number of trials per axis. The median is used for robustness against outliers. Default 3.
        power_percent: Motor power percentage (1--100). Default 100 for true maximum characterization.
        accel_timeout: Maximum time in seconds to wait for the acceleration phase before giving up. Default 3.0.
        decel_timeout: Maximum time in seconds to record the deceleration (coast-down) phase. Default 3.0.
        persist: If ``True``, write the measured limits to ``raccoon.project.yml`` under ``robot.motion_pid``. Default ``True``.

    Returns:
        A CharacterizeDriveBuilder (chainable via ``.axes()``, ``.trials()``, ``.power_percent()``, ``.accel_timeout()``, ``.decel_timeout()``, ``.persist()``, ``.on_anomaly()``, ``.skip_timing()``).

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
    b = CharacterizeDriveBuilder()
    b._axes = axes
    b._trials = trials
    b._power_percent = power_percent
    b._accel_timeout = accel_timeout
    b._decel_timeout = decel_timeout
    b._persist = persist
    return b


__all__ = ["CharacterizeDriveBuilder", "characterize_drive"]
