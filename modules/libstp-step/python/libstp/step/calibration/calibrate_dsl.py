"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: calibrate.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .calibrate import Calibrate


class CalibrateBuilder(StepBuilder):
    """Builder for Calibrate. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._distance_cm = 30.0
        self._persist_to_yaml = True
        self._ema_alpha = 0.7
        self._calibration_sets = None
        self._exclude_ir_sensors = None

    def distance_cm(self, value: float):
        self._distance_cm = value
        return self

    def persist_to_yaml(self, value: bool):
        self._persist_to_yaml = value
        return self

    def ema_alpha(self, value: float):
        self._ema_alpha = value
        return self

    def calibration_sets(self, value: Optional[List[str]]):
        self._calibration_sets = value
        return self

    def exclude_ir_sensors(self, value: Optional[List['IRSensor']]):
        self._exclude_ir_sensors = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['distance_cm'] = self._distance_cm
        kwargs['persist_to_yaml'] = self._persist_to_yaml
        kwargs['ema_alpha'] = self._ema_alpha
        kwargs['calibration_sets'] = self._calibration_sets
        kwargs['exclude_ir_sensors'] = self._exclude_ir_sensors
        return Calibrate(**kwargs)


@dsl(tags=['calibration'])
def calibrate(distance_cm: float = 30.0, persist_to_yaml: bool = True, ema_alpha: float = 0.7, calibration_sets: Optional[List[str]] = None, exclude_ir_sensors: Optional[List['IRSensor']] = None):
    """
    Run a unified distance and IR sensor calibration.

    This is the recommended all-in-one calibration entry point. It drives
    the robot a known distance, prompts the user to measure the actual
    distance traveled, then adjusts the per-wheel ``ticks_to_rad`` values
    to correct odometry. After distance calibration, it automatically
    calibrates IR sensors by sampling them during a drive over the
    calibration surface(s).

    Calibration values are persisted to ``raccoon.project.yml`` using an
    exponential moving average (EMA), so the baseline converges toward the
    true value over multiple calibration runs.

    Prerequisites:
        The robot must have drive motors with encoder feedback and
        a configured kinematics model. For IR calibration, IR sensors
        must be registered in ``robot.defs.analog_sensors``.

    Args:
        distance_cm: Distance (in cm) the robot drives during calibration. Longer distances yield more accurate results.
        persist_to_yaml: If ``True``, write the EMA-filtered baseline back to ``raccoon.project.yml`` so it persists across runs.
        ema_alpha: EMA smoothing coefficient between 0.0 and 1.0. Higher values produce slower convergence but a more stable baseline.
        calibration_sets: List of named IR calibration surface sets to run (e.g. ``["default", "transparent"]``). Each set beyond the first triggers an additional drive-and-sample cycle.
        exclude_ir_sensors: List of ``IRSensor`` instances to skip during IR calibration.

    Returns:
        A CalibrateBuilder (chainable via ``.distance_cm()``, ``.persist_to_yaml()``, ``.ema_alpha()``, ``.calibration_sets()``, ``.exclude_ir_sensors()``).

    Example::

        from libstp.step.calibration import calibrate

        # Basic calibration with defaults
        calibrate()

        # Custom: longer drive, two IR surface sets
        calibrate(
            distance_cm=50.0,
            calibration_sets=["default", "transparent"],
        )
    """
    b = CalibrateBuilder()
    b._distance_cm = distance_cm
    b._persist_to_yaml = persist_to_yaml
    b._ema_alpha = ema_alpha
    b._calibration_sets = calibration_sets
    b._exclude_ir_sensors = exclude_ir_sensors
    return b


__all__ = ['CalibrateBuilder', 'calibrate']
