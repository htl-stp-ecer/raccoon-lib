"""
Characterize drive limits by commanding raw motor power and measuring response.

Bypasses ALL velocity control (library PID and firmware BEMF PID) to discover
the robot's true physical limits: max velocity, acceleration, and deceleration
for each axis.  Motors are driven at 100% PWM via the kinematics layer's
``applyPowerCommand``, which uses inverse kinematics to compute per-wheel
direction signs while commanding raw open-loop power.

Results are persisted to raccoon.project.yml under robot.motion_pid and applied
in-memory.

The sampling loop runs in C++ (libstp-autotune) at up to 500 Hz in a tight
thread-sleep loop, completely bypassing asyncio and GIL overhead.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, ClassVar

from raccoon.project_yaml import find_project_root, update_project_value

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


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
       up to 500 Hz until a velocity plateau is detected or the timeout expires.
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
        sample_hz: Position sampling rate in Hz for the C++ measurement loop.
            Default 500. Higher values give better accel/decel resolution.
        operating_velocity_frac: Fraction of the measured peak velocity that
            is written back as ``max_velocity`` (and applied to the in-memory
            motion config). The peak measured at 100 %% PWM is the hardware
            ceiling, not a sensible operating speed — running the motion
            controllers at the ceiling means saturated wheels, no headroom
            for closed-loop correction, and unstable behaviour. Default
            ``0.6`` (≈30 cm/s for a robot that peaks at 50 cm/s).
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

    # Mapping from axis name to the AxisConstraints attribute name on
    # UnifiedMotionPidConfig (for in-memory and YAML persistence).
    _AXIS_ATTR: ClassVar[dict[str, str]] = {
        "forward": "linear",
        "lateral": "lateral",
        "angular": "angular",
    }

    def __init__(
        self,
        axes: list[str] | None = None,
        trials: int = 3,
        power_percent: int = 100,
        accel_timeout: float = 3.0,
        decel_timeout: float = 3.0,
        sample_hz: int = 500,
        operating_velocity_frac: float = 0.6,
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
        self.sample_hz = sample_hz
        self.operating_velocity_frac = max(0.1, min(1.0, operating_velocity_frac))
        self.persist = persist

    def _generate_signature(self) -> str:
        return (
            f"CharacterizeDrive(axes={self.axes}, "
            f"trials={self.trials}, power={self.power_percent}%, "
            f"hz={self.sample_hz}, persist={self.persist})"
        )

    def _persist_to_yaml(self, results: dict) -> bool:
        project_root = find_project_root()
        if project_root is None:
            return False

        updated = False
        for axis, result in results.items():
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
                operating_vel = result.max_velocity * self.operating_velocity_frac
                update_project_value(project_root, [*base, "max_velocity"], round(operating_vel, 4))
                updated = True

        return updated

    def _apply_to_config(self, robot: "GenericRobot", results: dict) -> None:
        cfg = robot.motion_pid_config
        for axis, result in results.items():
            attr = self._AXIS_ATTR.get(axis)
            if attr is None:
                continue
            axis_constraints = getattr(cfg, attr)
            if result.acceleration > 0:
                axis_constraints.acceleration = result.acceleration
            if result.deceleration > 0:
                axis_constraints.deceleration = result.deceleration
            if result.max_velocity > 0:
                axis_constraints.max_velocity = result.max_velocity * self.operating_velocity_frac

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping drive characterization, using stored values")
            return

        from raccoon.autotune import CharacterizeConfig, DriveCharacterizer

        cfg = CharacterizeConfig()
        cfg.power_percent = self.power_percent
        cfg.trials = self.trials
        cfg.accel_timeout = self.accel_timeout
        cfg.decel_timeout = self.decel_timeout
        cfg.sample_hz = self.sample_hz

        char = DriveCharacterizer(robot.drive, robot.odometry)

        # Runs in C++ at sample_hz, GIL released for the full sampling loop.
        self.results = await asyncio.get_event_loop().run_in_executor(
            None, lambda: char.characterize(self.axes, cfg)
        )

        self._apply_to_config(robot, self.results)
        self.info("  Applied to in-memory motion config")

        if self.persist:
            if self._persist_to_yaml(self.results):
                self.info("  Saved to raccoon.project.yml (robot.motion_pid)")
            else:
                self.warn("  Failed to save to raccoon.project.yml")
