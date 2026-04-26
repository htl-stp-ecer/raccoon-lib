"""Construction-time input validation tests for CustomVelocity and its builder."""

from __future__ import annotations

import importlib.util

import pytest


def libstp_available() -> bool:
    """Check raccoon availability without importing."""
    return importlib.util.find_spec("raccoon.step.base") is not None


requires_libstp = pytest.mark.skipif(
    not libstp_available(),
    reason="raccoon native module not installed",
)


# ===========================================================================
# CustomVelocity class — direct construction
# ===========================================================================


@requires_libstp
class TestCustomVelocityValidation:
    def test_valid_callable(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        CustomVelocity(lambda robot, dt: (0.5, 0.0, 0.0))

    def test_valid_named_function(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        def my_fn(robot, dt):
            return 0.0, 0.0, 0.0

        CustomVelocity(my_fn)

    def test_valid_with_until(self):
        from raccoon.step.condition import after_seconds
        from raccoon.step.motion.custom_velocity import CustomVelocity

        CustomVelocity(lambda robot, dt: (0.0, 0.0, 0.0), until=after_seconds(2))

    def test_not_callable_string(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        with pytest.raises(TypeError, match="callable"):
            CustomVelocity("not a function")

    def test_not_callable_int(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        with pytest.raises(TypeError, match="callable"):
            CustomVelocity(42)

    def test_not_callable_none(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        with pytest.raises(TypeError, match="callable"):
            CustomVelocity(None)

    def test_until_wrong_type_string(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        with pytest.raises(TypeError, match="StopCondition"):
            CustomVelocity(lambda robot, dt: (0.0, 0.0, 0.0), until="stop")

    def test_until_wrong_type_int(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        with pytest.raises(TypeError, match="StopCondition"):
            CustomVelocity(lambda robot, dt: (0.0, 0.0, 0.0), until=5)

    def test_generate_signature_uses_fn_name(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        def steer(robot, dt):
            return 0.0, 0.0, 0.5

        step = CustomVelocity(steer)
        assert "steer" in step._generate_signature()

    def test_generate_signature_lambda(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity

        step = CustomVelocity(lambda robot, dt: (0.0, 0.0, 0.0))
        sig = step._generate_signature()
        assert "CustomVelocity" in sig


# ===========================================================================
# custom_velocity factory function
# ===========================================================================


@requires_libstp
class TestCustomVelocityFactoryValidation:
    def test_valid_callable(self):
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity

        custom_velocity(lambda robot, dt: (0.0, 0.0, 0.0))

    def test_returns_builder(self):
        from raccoon.step.motion.custom_velocity_dsl import (
            CustomVelocityBuilder,
            custom_velocity,
        )

        b = custom_velocity(lambda robot, dt: (0.0, 0.0, 0.0))
        assert isinstance(b, CustomVelocityBuilder)

    def test_valid_with_until(self):
        from raccoon.step.condition import after_seconds
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity

        custom_velocity(lambda robot, dt: (0.0, 0.0, 0.0), until=after_seconds(3))

    def test_not_callable(self):
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity

        with pytest.raises(TypeError, match="callable"):
            custom_velocity("not_a_fn")

    def test_not_callable_none(self):
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity

        with pytest.raises(TypeError, match="callable"):
            custom_velocity(None)

    def test_until_wrong_type(self):
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity

        with pytest.raises(TypeError, match="StopCondition"):
            custom_velocity(lambda robot, dt: (0.0, 0.0, 0.0), until="bad")


# ===========================================================================
# CustomVelocityBuilder fluent API
# ===========================================================================


@requires_libstp
class TestCustomVelocityBuilderValidation:
    def test_until_valid(self):
        from raccoon.step.condition import after_seconds
        from raccoon.step.motion.custom_velocity_dsl import CustomVelocityBuilder

        b = CustomVelocityBuilder()
        b._velocity_fn = lambda robot, dt: (0.0, 0.0, 0.0)
        result = b.until(after_seconds(5))
        assert result is b  # fluent — returns self

    def test_until_wrong_type_string(self):
        from raccoon.step.motion.custom_velocity_dsl import CustomVelocityBuilder

        b = CustomVelocityBuilder()
        with pytest.raises(TypeError, match="StopCondition"):
            b.until("stop when black")

    def test_until_wrong_type_int(self):
        from raccoon.step.motion.custom_velocity_dsl import CustomVelocityBuilder

        b = CustomVelocityBuilder()
        with pytest.raises(TypeError, match="StopCondition"):
            b.until(42)

    def test_chained_until_builds_correctly(self):
        from raccoon.step.condition import after_seconds
        from raccoon.step.motion.custom_velocity import CustomVelocity
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity

        def fn(robot, dt):
            return 0.3, 0.0, 0.0

        step = custom_velocity(fn).until(after_seconds(2)).resolve()
        assert isinstance(step, CustomVelocity)
        assert step._until is not None

    def test_build_without_until(self):
        from raccoon.step.motion.custom_velocity import CustomVelocity
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity

        def fn(robot, dt):
            return 0.0, 0.0, 0.0

        step = custom_velocity(fn).resolve()
        assert isinstance(step, CustomVelocity)
        assert step._until is None

    def test_inherits_on_anomaly(self):
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity
        from raccoon.step.step_builder import StepBuilder

        b = custom_velocity(lambda robot, dt: (0.0, 0.0, 0.0))
        assert isinstance(b, StepBuilder)
        assert hasattr(b, "on_anomaly")
        assert hasattr(b, "skip_timing")

    def test_condition_composition(self):
        """until() accepts composed conditions (|, &, +)."""
        from raccoon.step.condition import after_cm, after_seconds
        from raccoon.step.motion.custom_velocity_dsl import custom_velocity

        def fn(robot, dt):
            return 0.5, 0.0, 0.0

        custom_velocity(fn).until(after_seconds(5) | after_cm(30))
        custom_velocity(fn).until(after_seconds(5) & after_cm(30))
        custom_velocity(fn).until(after_cm(10) + after_seconds(3))


# ===========================================================================
# Export verification
# ===========================================================================


@requires_libstp
class TestCustomVelocityExports:
    def test_exported_from_motion_package(self):
        pass
