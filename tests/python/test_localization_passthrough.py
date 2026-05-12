"""Smoke tests for the raccoon.localization pybind11 module.

The C++ side is what owns the threaded tick loop and the particle-filter update;
that contract is covered by the libstp-localization C++ test suite. This
file just verifies the Python surface — module imports, type construction,
sigma coercion, and the GenericRobot.localization property default.
"""

from __future__ import annotations

import math

import pytest

import raccoon
from raccoon.foundation import Pose
from raccoon.localization import (
    Localization,
    LocalizationConfig,
    Observation,
    SurfaceKind,
    SurfaceMeasurement,
)
from raccoon.map import SensorOffset
from raccoon.robot.api import GenericRobot


def test_module_imports() -> None:
    assert Localization is not None
    assert LocalizationConfig is not None
    assert Observation is not None
    assert SurfaceKind is not None
    assert SurfaceMeasurement is not None


def test_top_level_reexport() -> None:
    assert raccoon.Localization is not None
    assert raccoon.LocalizationConfig is not None
    assert raccoon.Observation is not None


def test_localization_config_defaults() -> None:
    cfg = LocalizationConfig()
    assert cfg.tick_period_ms == 10
    assert cfg.particle_count == 128
    assert cfg.rng_seed == 0x5EED1234

    cfg2 = LocalizationConfig(tick_period_ms=20, particle_count=64, rng_seed=7)
    assert cfg2.tick_period_ms == 20
    assert cfg2.particle_count == 64
    assert cfg2.rng_seed == 7

    cfg.tick_period_ms = 5
    assert cfg.tick_period_ms == 5
    cfg.process_translation_noise_m = 0.01
    assert cfg.process_translation_noise_m == pytest.approx(0.01)


def test_observation_default_sigma() -> None:
    obs = Observation(pose=Pose())
    assert tuple(obs.sigma) == pytest.approx((1e-3, 1e-3, 1e-3))


def test_observation_sigma_with_inf() -> None:
    """``inf`` on an axis means "ignore" — round-trip through the property."""

    obs = Observation(pose=Pose(), sigma=(0.01, math.inf, 0.05))
    sx, sy, st = obs.sigma
    assert sx == pytest.approx(0.01)
    assert math.isinf(sy)
    assert st == pytest.approx(0.05)


def test_observation_sigma_accepts_list() -> None:
    obs = Observation(pose=Pose(), sigma=[0.1, 0.2, 0.3])
    assert tuple(obs.sigma) == pytest.approx((0.1, 0.2, 0.3))

    obs.sigma = (1.0, 2.0, 3.0)
    assert tuple(obs.sigma) == pytest.approx((1.0, 2.0, 3.0))


def test_observation_surface_measurements_round_trip() -> None:
    obs = Observation(pose=Pose())
    obs.surface_measurements = [
        SurfaceMeasurement(
            SurfaceKind.LINE,
            SensorOffset(forward_cm=4.0, strafe_cm=-1.5),
            detected=True,
            sigma_cm=0.6,
        )
    ]

    measurement = obs.surface_measurements[0]
    assert measurement.kind == SurfaceKind.LINE
    assert tuple(measurement.sensor) == pytest.approx((4.0, -1.5))
    assert measurement.detected is True
    assert measurement.sigma_cm == pytest.approx(0.6)


def test_generic_robot_localization_default_none() -> None:
    """``GenericRobot.localization`` is opt-in — defaults to None.

    Subclasses set ``self._localization`` in their ``__init__`` once Phase-3
    wires the service up. Until then the property must read None without
    requiring the subclass to declare anything.
    """
    # Property is defined on the class, returns None when ``_localization``
    # is missing on the instance. We can read it off a fresh class without
    # constructing a full robot subclass.
    descriptor = GenericRobot.__dict__["localization"]
    assert isinstance(descriptor, property)

    class _Stub:
        pass

    stub = _Stub()
    assert descriptor.fget(stub) is None  # type: ignore[misc]

    stub._localization = "sentinel"  # type: ignore[attr-defined]
    assert descriptor.fget(stub) == "sentinel"  # type: ignore[misc]
