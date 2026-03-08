"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: step.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .step import CalibrateDeadzone


class CalibrateDeadzoneBuilder(StepBuilder):
    """Builder for CalibrateDeadzone. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor_ports = None
        self._start_percent = 1
        self._max_percent = 30
        self._settle_time = 0.3

    def motor_ports(self, value: Optional[List[int]]):
        self._motor_ports = value
        return self

    def start_percent(self, value: int):
        self._start_percent = value
        return self

    def max_percent(self, value: int):
        self._max_percent = value
        return self

    def settle_time(self, value: float):
        self._settle_time = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['motor_ports'] = self._motor_ports
        kwargs['start_percent'] = self._start_percent
        kwargs['max_percent'] = self._max_percent
        kwargs['settle_time'] = self._settle_time
        return CalibrateDeadzone(**kwargs)


@dsl(tags=['calibration', 'motor', 'deadzone'])
def calibrate_deadzone(motor_ports: Optional[List[int]] = None, start_percent: int = 1, max_percent: int = 30, settle_time: float = 0.3):
    """
    Calibrate motor deadzone via UI-based human observation.

    The deadzone is the minimum power percentage required to overcome
    static friction and start the motor turning. BEMF readings are
    unreliable at low RPM, so this step ramps motor power from
    ``start_percent`` upward and asks the operator to confirm when
    the wheel starts spinning. The result is stored as the motor's
    ``ff.kS`` (static friction) feedforward value.

    Args:
        motor_ports: List of motor ports to calibrate. ``None`` calibrates all drive motors.
        start_percent: Starting power percentage to test.
        max_percent: Maximum power percentage before giving up.
        settle_time: Seconds to wait after setting power before asking the operator.

    Returns:
        A CalibrateDeadzoneBuilder (chainable via ``.motor_ports()``, ``.start_percent()``, ``.max_percent()``, ``.settle_time()``).

    Example::

        from libstp.step.calibration import calibrate_deadzone

        # Calibrate all motors with defaults
        calibrate_deadzone()

        # Calibrate only motors 0 and 1
        calibrate_deadzone(motor_ports=[0, 1])
    """
    b = CalibrateDeadzoneBuilder()
    b._motor_ports = motor_ports
    b._start_percent = start_percent
    b._max_percent = max_percent
    b._settle_time = settle_time
    return b


__all__ = ['CalibrateDeadzoneBuilder', 'calibrate_deadzone']
