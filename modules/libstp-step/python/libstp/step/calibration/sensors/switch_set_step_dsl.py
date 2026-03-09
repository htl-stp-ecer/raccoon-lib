"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: switch_set_step.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .switch_set_step import SwitchCalibrationSet


class SwitchCalibrationSetBuilder(StepBuilder):
    """Builder for SwitchCalibrationSet. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._set_name = 'default'

    def set_name(self, value: str):
        self._set_name = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['set_name'] = self._set_name
        return SwitchCalibrationSet(**kwargs)


@dsl(tags=['calibration', 'sensor'])
def switch_calibration_set(set_name: str = 'default'):
    """
    Switch IR sensors to a named calibration set.

    Loads calibration data for the given set name from the calibration
    store and applies it to all registered IR sensors. Each sensor looks
    up its per-port calibration key (e.g. ``"transparent_port3"``) and
    sets its black/white thresholds accordingly.

    Use this to swap between surface-specific calibrations at runtime
    (e.g. switching from the default table surface to transparent objects).

    Args:
        set_name: Name of the calibration set to apply (e.g. ``"default"``, ``"transparent"``).

    Returns:
        A SwitchCalibrationSetBuilder (chainable via ``.set_name()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.calibration import switch_calibration_set

        # Switch to transparent calibration before scoring
        switch_calibration_set("transparent")
    """
    b = SwitchCalibrationSetBuilder()
    b._set_name = set_name
    return b


__all__ = ['SwitchCalibrationSetBuilder', 'switch_calibration_set']
