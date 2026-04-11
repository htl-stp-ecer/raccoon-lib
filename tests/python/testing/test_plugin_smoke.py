"""Smoke tests for the raccoon.testing pytest plugin module.

These don't run the plugin inside pytest — they just verify the module
loads, exposes the expected fixture functions, and that the scene
resolution helper finds bundled scenes. Full end-to-end coverage of the
fixtures lives in the drive-mission integration test which requires a
mock-bundle wheel.
"""
from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("raccoon.testing.pytest_plugin")


def test_plugin_module_imports_without_sim() -> None:
    """The plugin module itself must import even on a non-mock wheel.

    Sim-dependent imports happen lazily inside fixtures so that projects
    without the mock bundle can still collect tests and get a clean skip
    message rather than an import error at collection time.
    """
    # If this raises, the plugin is doing sim work at import time, which
    # breaks test collection on wombat-bundle wheels.
    from raccoon.testing import pytest_plugin  # noqa: F401


def test_plugin_exposes_expected_fixtures() -> None:
    from raccoon.testing import pytest_plugin

    for name in ("project_info", "robot_sim_config", "robot", "scene", "run_step"):
        fn = getattr(pytest_plugin, name, None)
        assert fn is not None, f"plugin missing fixture: {name}"
        # pytest fixtures are wrapped functions; at minimum they should
        # be callable.
        assert callable(fn), f"{name} is not callable"


def test_resolve_scene_finds_bundled_file(tmp_path: Path) -> None:
    """When the wheel was built with scenes bundled, the plugin's scene
    resolver should find them by bare filename."""
    from raccoon.testing import pytest_plugin

    bundled = pytest_plugin._bundled_scenes_dir()
    if bundled is None or not any(bundled.glob("*.ftmap")):
        pytest.skip("scenes not bundled with this raccoon install")

    any_scene = next(bundled.glob("*.ftmap")).name
    resolved = pytest_plugin._resolve_scene(any_scene, tmp_path)
    assert resolved.is_file()
    assert resolved.name == any_scene


def test_resolve_scene_prefers_project_over_bundled(tmp_path: Path) -> None:
    from raccoon.testing import pytest_plugin

    project_scene = tmp_path / "scenes" / "custom.ftmap"
    project_scene.parent.mkdir()
    project_scene.write_text("# fake scene\n")

    resolved = pytest_plugin._resolve_scene("custom.ftmap", tmp_path)
    assert resolved == project_scene


def test_resolve_scene_accepts_absolute_path(tmp_path: Path) -> None:
    from raccoon.testing import pytest_plugin

    scene_file = tmp_path / "somewhere.ftmap"
    scene_file.write_text("# fake\n")

    resolved = pytest_plugin._resolve_scene(str(scene_file), tmp_path)
    assert resolved == scene_file


def test_resolve_scene_raises_with_helpful_message(tmp_path: Path) -> None:
    from raccoon.testing import pytest_plugin

    with pytest.raises(FileNotFoundError, match="Searched:"):
        pytest_plugin._resolve_scene("does_not_exist.ftmap", tmp_path)
