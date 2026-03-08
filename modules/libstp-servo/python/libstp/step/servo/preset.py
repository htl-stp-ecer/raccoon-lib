"""
ServoPreset — named positions for a servo, generated from project YAML.

A ``ServoPreset`` wraps a raw ``Servo`` and provides callable attributes
for each named position. It is the recommended way to interact with servos
that have well-defined positions (e.g., "open", "closed", "up", "down").

Calling a position without ``speed`` uses an instant ``servo()`` command.
Calling with ``speed`` uses ``SlowServo`` for eased motion.

The optional ``offset`` shifts all positions by a fixed amount — useful
when a servo is remounted slightly off from its original alignment.

Instances are normally **code-generated** from ``raccoon.project.yml``
into ``defs.py``, so users just import and call::

    from src.hardware.defs import Defs

    # In a mission sequence:
    Defs.pom_arm.down()            # instant
    Defs.pom_arm.up(speed=250)     # eased at 250 deg/s

YAML definition::

    definitions:
      pom_arm:
        type: Servo
        port: 1
        offset: 0           # optional, default 0
        positions:
          down: 0
          above_pom: 50
          up: 90
"""
from __future__ import annotations

from libstp.hal import Servo

from .steps import servo as _servo_step, SlowServo


class ServoPreset:
    """Named servo positions with optional mounting offset.

    Each position becomes a callable attribute that returns a Step.

    Args:
        servo: The underlying ``Servo`` hardware device.
        positions: Mapping of position name to angle in degrees.
        offset: Fixed angle offset added to every position (default 0).
    """

    def __init__(self, servo: Servo, positions: dict[str, float], offset: float = 0):
        self._servo = servo
        self._offset = offset
        self._positions = dict(positions)
        for name, angle in positions.items():
            self._register(name, angle)

    @property
    def device(self) -> Servo:
        """The underlying Servo hardware device, for raw access."""
        return self._servo

    @property
    def offset(self) -> float:
        """Current mounting offset in degrees."""
        return self._offset

    @property
    def positions(self) -> dict[str, float]:
        """Copy of the position name → angle mapping (before offset)."""
        return dict(self._positions)

    def _register(self, name: str, angle: float) -> None:
        actual = angle + self._offset

        def make_step(speed=None):
            if speed is None:
                return _servo_step(self._servo, actual)
            return SlowServo(self._servo, actual, speed)

        make_step.__name__ = name
        make_step.__doc__ = f"Move to {name} ({angle}{'%+g' % self._offset if self._offset else ''} deg)"
        setattr(self, name, make_step)
