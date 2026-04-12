"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: calibrate_distance.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .calibrate_distance import CalibrateDistance


class CalibrateDistanceBuilder(StepBuilder):
    """Builder for CalibrateDistance. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._distance_cm = 30.0
        self._speed = 1.0
        self._calibrate_light_sensors = False
        self._persist_to_yaml = True
        self._ema_alpha = 0.7
        self._calibration_sets = None
        self._exclude_ir_sensors = None

    def distance_cm(self, value: float):
        self._distance_cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def calibrate_light_sensors(self, value: bool):
        self._calibrate_light_sensors = value
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
        kwargs['speed'] = self._speed
        kwargs['calibrate_light_sensors'] = self._calibrate_light_sensors
        kwargs['persist_to_yaml'] = self._persist_to_yaml
        kwargs['ema_alpha'] = self._ema_alpha
        kwargs['calibration_sets'] = self._calibration_sets
        kwargs['exclude_ir_sensors'] = self._exclude_ir_sensors
        return CalibrateDistance(**kwargs)


@dsl(tags=['calibration', 'distance'])
def calibrate_distance(distance_cm: float = 30.0, speed: float = 1.0, calibrate_light_sensors: bool = False, persist_to_yaml: bool = True, ema_alpha: float = 0.7, calibration_sets: Optional[List[str]] = None, exclude_ir_sensors: Optional[List['IRSensor']] = None):
    """
    Calibrate per-wheel distance estimation via encoder measurement.

    Drives the robot a known distance, then prompts the user to enter the
    actual measured distance. The step computes a corrected ``ticks_to_rad``
    value for each drive motor so that odometry matches real-world distances.

    The calibration operates in two modes simultaneously:

    - **Runtime**: Applies the measured ``ticks_to_rad`` directly for best
      accuracy during this run.
    - **Persistent**: Updates the YAML baseline using an exponential moving
      average (EMA) so the stored value converges toward the true value over
      multiple calibration runs.

    Args:
        distance_cm: Distance (in cm) the robot drives during calibration. Longer distances yield better accuracy.
        speed: Drive speed during the calibration runs, as a fraction of max speed in ``[0.0, 1.0]``. Lower speeds reduce wheel slip and usually produce more accurate calibration.
        calibrate_light_sensors: If ``True``, run IR sensor calibration after the distance calibration is confirmed.
        persist_to_yaml: If ``True``, write the EMA-filtered baseline to ``raccoon.project.yml`` so it persists across program runs.
        ema_alpha: EMA smoothing coefficient between 0.0 and 1.0. Higher values produce slower convergence but a more stable baseline.
        calibration_sets: List of named IR calibration surface sets (e.g. ``["default", "transparent"]``). Only used when ``calibrate_light_sensors`` is ``True``.
        exclude_ir_sensors: List of ``IRSensor`` instances to skip during IR calibration.

    Returns:
        A CalibrateDistanceBuilder (chainable via ``.distance_cm()``, ``.speed()``, ``.calibrate_light_sensors()``, ``.persist_to_yaml()``, ``.ema_alpha()``, ``.calibration_sets()``, ``.exclude_ir_sensors()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.calibration import calibrate_distance

        # Distance-only calibration with defaults
        calibrate_distance()

        # Distance + IR sensor calibration
        calibrate_distance(
            distance_cm=50.0,
            calibrate_light_sensors=True,
            calibration_sets=["default", "transparent"],
        )
    """
    b = CalibrateDistanceBuilder()
    b._distance_cm = distance_cm
    b._speed = speed
    b._calibrate_light_sensors = calibrate_light_sensors
    b._persist_to_yaml = persist_to_yaml
    b._ema_alpha = ema_alpha
    b._calibration_sets = calibration_sets
    b._exclude_ir_sensors = exclude_ir_sensors
    return b


__all__ = ['CalibrateDistanceBuilder', 'calibrate_distance']
