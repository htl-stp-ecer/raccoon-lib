"""Unit tests for raccoon.testing._project.

These exercise the fragile parts of the project loader — the YAML
``!include`` / ``!include-merge`` handling and the config-derivation logic
that turns a drumbot-shaped project.yml into a SimRobotConfig — without
requiring a real raccoon project or the mock HAL to be installed.
"""
from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("yaml")
pytest.importorskip("raccoon.testing._project")

from raccoon.testing import _project  # noqa: E402
from raccoon.testing.sim import SimRobotConfig  # noqa: E402


# --------------------------------------------------------------------------
# find_project_root
# --------------------------------------------------------------------------

def test_find_project_root_current_dir(tmp_path: Path) -> None:
    (tmp_path / _project.PROJECT_FILENAME).write_text("name: x\n")
    assert _project.find_project_root(tmp_path) == tmp_path


def test_find_project_root_walks_upward(tmp_path: Path) -> None:
    (tmp_path / _project.PROJECT_FILENAME).write_text("name: x\n")
    nested = tmp_path / "tests" / "unit"
    nested.mkdir(parents=True)
    assert _project.find_project_root(nested) == tmp_path


def test_find_project_root_raises_when_missing(tmp_path: Path) -> None:
    with pytest.raises(_project.ProjectNotFoundError):
        _project.find_project_root(tmp_path)


# --------------------------------------------------------------------------
# !include / !include-merge loader
# --------------------------------------------------------------------------

def test_include_tag_resolves_relative_path(tmp_path: Path) -> None:
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "motors.yml").write_text(
        "left:\n  port: 0\nright:\n  port: 1\n"
    )
    (tmp_path / "raccoon.project.yml").write_text(
        "definitions: !include 'config/motors.yml'\n"
    )

    data = _project.load_project(tmp_path)
    assert data["definitions"]["left"]["port"] == 0
    assert data["definitions"]["right"]["port"] == 1


def test_include_merge_flattens_into_parent(tmp_path: Path) -> None:
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "extras.yml").write_text(
        "drum_motor:\n  type: Motor\n  port: 2\n"
    )
    (tmp_path / "config" / "hw.yml").write_text(
        "front_left_motor:\n  port: 0\n"
        "front_right_motor:\n  port: 1\n"
        "_extras: !include-merge 'extras.yml'\n"
    )
    (tmp_path / "raccoon.project.yml").write_text(
        "definitions: !include 'config/hw.yml'\n"
    )

    data = _project.load_project(tmp_path)
    defs = data["definitions"]
    # drum_motor got merged up out of _extras into the parent mapping
    assert "drum_motor" in defs
    assert defs["drum_motor"]["port"] == 2
    assert "_extras" not in defs  # merge sentinel consumed


def test_nested_includes(tmp_path: Path) -> None:
    (tmp_path / "a.yml").write_text("value: !include 'b.yml'\n")
    (tmp_path / "b.yml").write_text("42\n")
    (tmp_path / "raccoon.project.yml").write_text("x: !include 'a.yml'\n")

    data = _project.load_project(tmp_path)
    assert data["x"]["value"] == 42


# --------------------------------------------------------------------------
# derive_sim_config
# --------------------------------------------------------------------------

def _drumbot_shaped_project() -> dict:
    """Minimal drumbot-shaped project.yml (post-include-resolution)."""
    return {
        "robot": {
            "drive": {
                "kinematics": {
                    "type": "differential",
                    "wheel_radius": 0.0345,
                    "wheelbase": 0.16,
                    "left_motor": "front_left_motor",
                    "right_motor": "front_right_motor",
                },
            },
            "motion_pid": {
                "linear": {"max_velocity": 0.2368},
            },
            "physical": {
                "width_cm": 13.0,
                "length_cm": 19.0,
                "rotation_center": {"x_cm": 6.5, "y_cm": 5.5},
            },
        },
        "definitions": {
            "front_left_motor": {
                "type": "Motor",
                "port": 0,
                "inverted": False,
                "calibration": {"ticks_to_rad": 1.992595700973688e-05},
            },
            "front_right_motor": {
                "type": "Motor",
                "port": 1,
                "inverted": True,
                "calibration": {"ticks_to_rad": 1.6050075169098126e-05},
            },
        },
    }


def test_derive_kinematics_from_drumbot_shape() -> None:
    cfg = _project.derive_sim_config(_drumbot_shaped_project())
    assert cfg.wheel_radius_m == pytest.approx(0.0345)
    assert cfg.track_width_m == pytest.approx(0.16)
    assert cfg.wheelbase_m == pytest.approx(0.16)


def test_derive_motor_ports_and_inversion() -> None:
    cfg = _project.derive_sim_config(_drumbot_shaped_project())
    assert cfg.left_motor_port == 0
    assert cfg.right_motor_port == 1
    assert cfg.left_motor_inverted is False
    assert cfg.right_motor_inverted is True


def test_derive_physical_dimensions() -> None:
    cfg = _project.derive_sim_config(_drumbot_shaped_project())
    assert cfg.width_cm == pytest.approx(13.0)
    assert cfg.length_cm == pytest.approx(19.0)
    # rotation_center (6.5, 5.5) relative to (width/2=6.5, length/2=9.5)
    # → strafe 0, forward -4
    assert cfg.rotation_center_strafe_cm == pytest.approx(0.0)
    assert cfg.rotation_center_forward_cm == pytest.approx(-4.0)


def test_derive_ticks_to_rad_from_left_motor() -> None:
    cfg = _project.derive_sim_config(_drumbot_shaped_project())
    assert cfg.ticks_to_rad == pytest.approx(1.992595700973688e-05)


def test_derive_max_wheel_velocity_uses_linear_pid_cap() -> None:
    cfg = _project.derive_sim_config(_drumbot_shaped_project())
    # linear cap 0.2368 m/s ÷ wheel 0.0345 m × 1.5 headroom ≈ 10.3 rad/s
    expected = (0.2368 / 0.0345) * 1.5
    assert cfg.max_wheel_velocity_rad_s == pytest.approx(expected, rel=1e-3)


def test_derive_empty_project_returns_defaults() -> None:
    cfg = _project.derive_sim_config({})
    default = SimRobotConfig()
    assert cfg.wheel_radius_m == default.wheel_radius_m
    assert cfg.track_width_m == default.track_width_m
    assert cfg.left_motor_port == default.left_motor_port


def test_derive_survives_missing_kinematics() -> None:
    partial = {"robot": {"physical": {"width_cm": 20.0}}}
    cfg = _project.derive_sim_config(partial)
    assert cfg.width_cm == pytest.approx(20.0)
    assert cfg.wheel_radius_m == SimRobotConfig().wheel_radius_m


def test_derive_ignores_non_string_motor_ref() -> None:
    # A malformed project file might accidentally put the whole motor
    # definition inline under left_motor instead of a reference — don't
    # crash, just fall back to defaults.
    data = {
        "robot": {
            "drive": {
                "kinematics": {
                    "wheel_radius": 0.03,
                    "wheelbase": 0.15,
                    "left_motor": {"port": 0},
                    "right_motor": {"port": 1},
                },
            },
        },
        "definitions": {},
    }
    cfg = _project.derive_sim_config(data)
    # wheel params still applied
    assert cfg.wheel_radius_m == pytest.approx(0.03)
    # motor ports fell back to defaults since the names weren't strings
    assert cfg.left_motor_port == SimRobotConfig().left_motor_port
