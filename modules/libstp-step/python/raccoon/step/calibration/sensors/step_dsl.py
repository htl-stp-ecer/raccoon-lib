"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: step.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .step import CalibrateSensors


class CalibrateSensorsBuilder(StepBuilder):
    """Builder for CalibrateSensors. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._calibration_time = 5.0
        self._allow_use_existing = True
        self._calibration_sets = None

    def calibration_time(self, value: float):
        self._calibration_time = value
        return self

    def allow_use_existing(self, value: bool):
        self._allow_use_existing = value
        return self

    def calibration_sets(self, value: Optional[List[str]]):
        self._calibration_sets = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["calibration_time"] = self._calibration_time
        kwargs["allow_use_existing"] = self._allow_use_existing
        kwargs["calibration_sets"] = self._calibration_sets
        return CalibrateSensors(**kwargs)


@dsl(tags=["calibration", "sensor"])
def calibrate_sensors(
    calibration_time: float = 5.0,
    allow_use_existing: bool = True,
    calibration_sets: Optional[List[str]] = None,
):
    """
    Calibrate IR sensors by sampling black and white surface readings.

    Guides the operator through sampling each IR sensor over calibration
    surfaces to establish black/white thresholds. The operator places the
    robot on the calibration surface and the step samples readings for the
    configured duration. Supports multiple named calibration sets (e.g.
    ``"default"`` and ``"transparent"``) for surface-specific thresholds.

    Args:
        calibration_time: Duration for calibration sampling in seconds.
        allow_use_existing: If ``True``, offer to reuse existing calibration values instead of re-sampling.
        calibration_sets: Named calibration sets to calibrate (e.g. ``["default", "transparent"]``). Defaults to ``["default"]``.

    Returns:
        A CalibrateSensorsBuilder (chainable via ``.calibration_time()``, ``.allow_use_existing()``, ``.calibration_sets()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.calibration import calibrate_sensors

        # Basic IR calibration
        calibrate_sensors()

        # Two surface sets with longer sampling
        calibrate_sensors(
            calibration_time=8.0,
            calibration_sets=["default", "transparent"],
        )
    """
    b = CalibrateSensorsBuilder()
    b._calibration_time = calibration_time
    b._allow_use_existing = allow_use_existing
    b._calibration_sets = calibration_sets
    return b


__all__ = ["CalibrateSensorsBuilder", "calibrate_sensors"]
