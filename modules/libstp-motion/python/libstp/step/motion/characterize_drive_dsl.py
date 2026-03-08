"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: characterize_drive.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .characterize_drive import CharacterizeDrive


class CharacterizeDriveBuilder(StepBuilder):
    """Builder for CharacterizeDrive. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._axes = None
        self._trials = 3
        self._command_speed = 1.0
        self._accel_timeout = 3.0
        self._decel_timeout = 3.0
        self._persist = True

    def axes(self, value: list[str]):
        self._axes = value
        return self

    def trials(self, value: int):
        self._trials = value
        return self

    def command_speed(self, value: float):
        self._command_speed = value
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
        kwargs['axes'] = self._axes
        kwargs['trials'] = self._trials
        kwargs['command_speed'] = self._command_speed
        kwargs['accel_timeout'] = self._accel_timeout
        kwargs['decel_timeout'] = self._decel_timeout
        kwargs['persist'] = self._persist
        return CharacterizeDrive(**kwargs)


@dsl(tags=['motion', 'calibration', 'characterize'])
def characterize_drive(axes: list[str] = None, trials: int = 3, command_speed: float = 1.0, accel_timeout: float = 3.0, decel_timeout: float = 3.0, persist: bool = True):
    """
    Characterize the robot's physical drive limits by commanding raw velocities.

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
        axes: Which axes to characterize. Options are ``"forward"``, ``"lateral"``, and ``"angular"``. Default ``["forward"]``.
        trials: Number of trials per axis. The median is used for robustness against outliers. Default 3.
        command_speed: Raw velocity command magnitude sent to the motors. For forward/lateral this is in m/s; for angular it is in rad/s. Should be at or near the maximum the robot can sustain. Default 1.0.
        accel_timeout: Maximum time in seconds to wait for the acceleration phase before giving up. Default 3.0.
        decel_timeout: Maximum time in seconds to record the deceleration (coast-down) phase. Default 3.0.
        persist: If ``True``, write the measured limits to ``raccoon.project.yml`` under ``robot.motion_pid``. Default ``True``.

    Returns:
        A CharacterizeDriveBuilder (chainable via ``.axes()``, ``.trials()``, ``.command_speed()``, ``.accel_timeout()``, ``.decel_timeout()``, ``.persist()``).

    Example::

        from libstp.step.motion import characterize_drive

        # Characterize forward axis with default settings
        characterize_drive()

        # Characterize forward and angular axes with 5 trials each
        characterize_drive(
            axes=["forward", "angular"],
            trials=5,
        )

        # Characterize without saving to disk (dry run)
        characterize_drive(
            axes=["forward", "lateral", "angular"],
            persist=False,
        )
    """
    b = CharacterizeDriveBuilder()
    b._axes = axes
    b._trials = trials
    b._command_speed = command_speed
    b._accel_timeout = accel_timeout
    b._decel_timeout = decel_timeout
    b._persist = persist
    return b


__all__ = ['CharacterizeDriveBuilder', 'characterize_drive']
