"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: wait_for_light.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .wait_for_light import WaitForLight, WaitForLightLegacy


class WaitForLightBuilder(StepBuilder):
    """Builder for WaitForLight. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._drop_fraction = 0.15
        self._confirm_count = 3
        self._warmup_seconds = 1.0
        self._poll_interval = 0.005

    def sensor(self, value: AnalogSensor):
        self._sensor = value
        return self

    def drop_fraction(self, value: float):
        self._drop_fraction = value
        return self

    def confirm_count(self, value: int):
        self._confirm_count = value
        return self

    def warmup_seconds(self, value: float):
        self._warmup_seconds = value
        return self

    def poll_interval(self, value: float):
        self._poll_interval = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        kwargs['drop_fraction'] = self._drop_fraction
        kwargs['confirm_count'] = self._confirm_count
        kwargs['warmup_seconds'] = self._warmup_seconds
        kwargs['poll_interval'] = self._poll_interval
        return WaitForLight(**kwargs)


@dsl(tags=['timing', 'wait'])
def wait_for_light(sensor: AnalogSensor = _UNSET, drop_fraction: float = 0.15, confirm_count: int = 3, warmup_seconds: float = 1.0, poll_interval: float = 0.005):
    """
    Wait for the start lamp using automatic Kalman-filtered flank detection.

    Mount the light sensor facing downward with no shielding. The step
    establishes a stable baseline reading via a 1D Kalman filter during a
    short warm-up phase, then arms and polls for a sharp brightness
    increase (sensor value drop). When the raw reading falls below
    ``baseline * (1 - drop_fraction)`` for ``confirm_count`` consecutive
    samples, the step returns and the mission begins.

    The downward-facing mount reduces environmental noise by up to 76%
    compared to a horizontal mount (Gosling et al., 2023). No black tape
    or straw shielding is required.

    Prerequisites:
        An analog light sensor (LDR) connected to the Wombat and mounted
        facing the table surface. The start lamp should be positioned
        diagonally above the sensor.

    Args:
        sensor: The AnalogSensor instance for the light sensor.
        drop_fraction: Fraction the raw value must drop below the baseline to trigger. 0.15 means a 15% brightness increase triggers the start. Lower values are more sensitive (faster but riskier), higher values are safer but need a stronger signal.
        confirm_count: Number of consecutive triggering samples required before starting. At the default 200 Hz poll rate, 3 samples equals ~15 ms of confirmation — effectively instant while rejecting single-sample noise spikes.
        warmup_seconds: Duration in seconds to collect baseline samples before arming the detector.
        poll_interval: Seconds between sensor reads. 0.005 gives ~200 Hz.

    Returns:
        A WaitForLightBuilder (chainable via ``.sensor()``, ``.drop_fraction()``, ``.confirm_count()``, ``.warmup_seconds()``, ``.poll_interval()``).

    Example::

        from libstp.step.wait_for_light import wait_for_light

        # Default settings — works well for most setups
        wait_for_light(robot.defs.wait_for_light_sensor)

        # More sensitive (10% drop, 2 confirms) for weak lamp signal
        wait_for_light(robot.defs.wait_for_light_sensor, drop_fraction=0.10, confirm_count=2)
    """
    b = WaitForLightBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._drop_fraction = drop_fraction
    b._confirm_count = confirm_count
    b._warmup_seconds = warmup_seconds
    b._poll_interval = poll_interval
    return b


class WaitForLightLegacyBuilder(StepBuilder):
    """Builder for WaitForLightLegacy. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET

    def sensor(self, value: AnalogSensor):
        self._sensor = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        return WaitForLightLegacy(**kwargs)


@dsl(tags=['timing', 'wait'])
def wait_for_light_legacy(sensor: AnalogSensor = _UNSET):
    """
    Wait for light using the legacy manual-calibration threshold method.

    Runs the traditional two-step calibration flow: the operator measures
    the sensor with the lamp off, then with it on, confirms the threshold,
    and the robot starts immediately. This approach requires manual
    interaction at the start of each run and uses a fixed midpoint
    threshold between the dark and light readings.

    Use this only if the automatic flank-detection method
    (``wait_for_light``) does not work for your setup, for example when
    the sensor is not mounted downward or when the lamp signal is too
    weak for reliable flank detection.

    Args:
        sensor: The AnalogSensor instance for the light sensor.

    Returns:
        A WaitForLightLegacyBuilder (chainable via ``.sensor()``).

    Example::

        from libstp.step.wait_for_light import wait_for_light_legacy

        wait_for_light_legacy(robot.defs.wait_for_light_sensor)
    """
    b = WaitForLightLegacyBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    return b


__all__ = ['WaitForLightBuilder', 'wait_for_light', 'WaitForLightLegacyBuilder', 'wait_for_light_legacy']
