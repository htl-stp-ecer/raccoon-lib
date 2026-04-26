"""Tests for raccoon.drive bindings.

Construction-time and configuration-roundtrip tests for the velocity-control
config types. The Drive class itself needs an IMU and a kinematics — that
exercise belongs in the integration suite under ``tests/python/sim/``.
"""

from __future__ import annotations

import pytest


def test_axis_velocity_control_config_default_construction() -> None:
    from raccoon.drive import AxisVelocityControlConfig

    cfg = AxisVelocityControlConfig()
    # Both PID and feedforward sub-configs must be present and accessible
    # so downstream code can mutate them in place.
    assert cfg.pid is not None
    assert cfg.ff is not None


def test_axis_velocity_control_config_explicit_construction() -> None:
    from raccoon.drive import AxisVelocityControlConfig
    from raccoon.foundation import Feedforward, PidGains

    pid = PidGains(kp=2.5, ki=0.1, kd=0.3)
    ff = Feedforward(kS=0.05, kV=0.04, kA=0.01)
    cfg = AxisVelocityControlConfig(pid=pid, ff=ff)

    assert cfg.pid.kp == pytest.approx(2.5)
    assert cfg.pid.ki == pytest.approx(0.1)
    assert cfg.pid.kd == pytest.approx(0.3)
    assert cfg.ff.kS == pytest.approx(0.05)
    assert cfg.ff.kV == pytest.approx(0.04)
    assert cfg.ff.kA == pytest.approx(0.01)


def test_axis_velocity_control_config_field_mutation() -> None:
    from raccoon.drive import AxisVelocityControlConfig
    from raccoon.foundation import PidGains

    cfg = AxisVelocityControlConfig()
    cfg.pid = PidGains(kp=1.7, ki=0.0, kd=0.4)
    assert cfg.pid.kp == pytest.approx(1.7)
    assert cfg.pid.kd == pytest.approx(0.4)


def test_chassis_velocity_control_config_default_construction() -> None:
    from raccoon.drive import ChassisVelocityControlConfig

    cfg = ChassisVelocityControlConfig()
    # The three axis sub-configs must be independently mutable.
    assert cfg.vx is not None
    assert cfg.vy is not None
    assert cfg.wz is not None


def test_chassis_velocity_control_config_axis_independence() -> None:
    """Mutating one axis must not bleed into the others (no shared instance)."""
    from raccoon.drive import AxisVelocityControlConfig, ChassisVelocityControlConfig
    from raccoon.foundation import Feedforward, PidGains

    cfg = ChassisVelocityControlConfig()
    cfg.vx = AxisVelocityControlConfig(
        pid=PidGains(kp=3.0, ki=0.0, kd=0.0),
        ff=Feedforward(),
    )
    cfg.wz = AxisVelocityControlConfig(
        pid=PidGains(kp=5.0, ki=0.0, kd=0.0),
        ff=Feedforward(),
    )

    assert cfg.vx.pid.kp == pytest.approx(3.0)
    assert cfg.wz.pid.kp == pytest.approx(5.0)
