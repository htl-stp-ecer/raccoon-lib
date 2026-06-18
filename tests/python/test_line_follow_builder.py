from __future__ import annotations

import pytest

from raccoon.step.motion import (
    DirectionalLineFollow,
    DirectionalSingleLineFollow,
    LineSide,
    line_follow,
)


class _DummySensor:
    def probabilityOfBlack(self) -> float:
        return 0.5

    def read(self) -> int:
        return 512


def test_line_follow_builder_builds_single_sensor_lateral_follow() -> None:
    step = (
        line_follow()
        .single(_DummySensor(), side=LineSide.RIGHT)
        .move(forward=0.7)
        .correct_lateral(hold_heading=False)
        .pid(0.6, 0.2, 0.05)
        .distance_cm(40)
        .resolve()
    )

    assert isinstance(step, DirectionalSingleLineFollow)
    assert step.config.heading_speed == pytest.approx(0.7)
    assert step.config.strafe_speed == pytest.approx(0.0)
    assert step.config.lateral_correction is True
    assert step.config.forward_correction is False
    assert step.config.heading_hold is False
    assert step.config.kp == pytest.approx(0.6)
    assert step.config.ki == pytest.approx(0.2)
    assert step.config.kd == pytest.approx(0.05)


def test_line_follow_builder_builds_dual_sensor_forward_follow() -> None:
    left = _DummySensor()
    right = _DummySensor()
    step = (
        line_follow().dual(left, right).move(strafe=0.8).correct_forward().distance_cm(30).resolve()
    )

    assert isinstance(step, DirectionalLineFollow)
    assert step.config.heading_speed == pytest.approx(0.0)
    assert step.config.strafe_speed == pytest.approx(0.8)
    assert step.config.forward_correction is True
    assert step.config.lateral_correction is False
    assert step.config.correction_sign == pytest.approx(-1.0)


def test_line_follow_builder_rejects_missing_motion_and_tracking() -> None:
    with pytest.raises(ValueError, match="tracking mode"):
        line_follow().distance_cm(20).resolve()


def test_line_follow_builder_rejects_invalid_correction_axis_mix() -> None:
    with pytest.raises(ValueError, match="base strafe must stay 0"):
        (
            line_follow()
            .single(_DummySensor(), side=LineSide.LEFT)
            .move(forward=0.5, strafe=0.2)
            .correct_lateral()
            .distance_cm(20)
            .resolve()
        )
