"""Pytest plugin for the raccoon sim test harness.

This module is registered as a ``pytest11`` entry point in the raccoon
wheel, so once ``raccoon`` is installed in a project's test environment the
fixtures below are available with no conftest boilerplate.

The three fixtures that matter:

- ``robot``: a fully wired instance of the project's generated
  ``src.hardware.robot.Robot`` class, backed by the mock HAL.
- ``scene``: a factory for entering a :func:`raccoon.testing.sim.use_scene`
  context. Automatically resolves scene names against the project's own
  ``scenes/`` dir first, then raccoon's bundled scenes.
- ``run_step``: a sync callable that awaits ``step.run_step(robot)`` with a
  timeout. Tests stay sync; no pytest-asyncio dependency.

Typical usage::

    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.testing.sim import pose


    def test_drives_30cm(robot, scene, run_step):
        scene("empty_table.ftmap", start=(20, 50, 0))
        run_step(drive_forward(cm=30), robot)
        assert pose().x > 45
"""

from __future__ import annotations

import asyncio
import importlib
import importlib.resources
import os
import sys
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

from . import _project
from .sim import PoseLike, SimRobotConfig, use_scene


class _MockHal:
    """Tracks whether the mock-bundle robot fixture has been instantiated.

    Wrapped in a class so the toggle is a simple attribute write at the
    call sites — no ``global`` statement needed and no lint noise from
    PLW0603. Consumed by ``pytest_sessionfinish`` to decide whether to
    short-circuit the interpreter shutdown.
    """

    touched: bool = False


# --------------------------------------------------------------------------
# Project discovery + robot import
# --------------------------------------------------------------------------

ROBOT_IMPORT_PATH = "src.hardware.robot"
ROBOT_CLASS_NAME = "Robot"


def _try_import_robot_class() -> Any:
    """Import the project's generated Robot class.

    Raises ImportError with a clear message if the project layout doesn't
    match the toolchain's generated output.
    """
    try:
        module = importlib.import_module(ROBOT_IMPORT_PATH)
    except ImportError as exc:
        msg = (
            f"Could not import {ROBOT_IMPORT_PATH}. The raccoon.testing "
            "fixtures expect the toolchain-generated hardware module at "
            "src/hardware/robot.py. Make sure you're running pytest from "
            f"the project root (where {_project.PROJECT_FILENAME} lives) "
            "and that `raccoon` codegen has produced the hardware files."
        )
        raise ImportError(msg) from exc

    robot_cls = getattr(module, ROBOT_CLASS_NAME, None)
    if robot_cls is None:
        msg = (
            f"{ROBOT_IMPORT_PATH} does not expose a `{ROBOT_CLASS_NAME}` "
            "class. Has the toolchain-generated file been modified?"
        )
        raise ImportError(msg)
    return robot_cls


def _ensure_project_on_sys_path(project_root: Path) -> None:
    """Add the project root to ``sys.path`` so ``src.hardware.robot`` imports.

    Pytest's rootdir discovery and conftest.py handling usually take care
    of this, but when the user has no conftest at the project root the
    import can fail in a confusing way. Be explicit.
    """
    root_str = str(project_root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)


def _bundled_scenes_dir() -> Path | None:
    """Locate the scenes directory shipped with the raccoon wheel, if any."""
    try:
        files = importlib.resources.files("raccoon").joinpath("scenes")
    except (ModuleNotFoundError, AttributeError):
        return None
    try:
        if files.is_dir():
            return Path(str(files))
    except (FileNotFoundError, NotADirectoryError):
        return None
    return None


def _resolve_scene(name: str, project_root: Path) -> Path:
    """Resolve a scene name against project + bundled scenes dirs."""
    path = Path(name)
    if path.is_absolute() and path.is_file():
        return path

    project_scene = project_root / "scenes" / name
    if project_scene.is_file():
        return project_scene

    bundled = _bundled_scenes_dir()
    if bundled is not None:
        bundled_scene = bundled / name
        if bundled_scene.is_file():
            return bundled_scene

    searched = [str(project_root / "scenes")]
    if bundled is not None:
        searched.append(str(bundled))
    msg = (
        f"Scene '{name}' not found. Searched: {', '.join(searched)}. "
        "Pass an absolute path, add the file to <project>/scenes/, or "
        "use one of the bundled scene names."
    )
    raise FileNotFoundError(msg)


# --------------------------------------------------------------------------
# Sim availability gate
# --------------------------------------------------------------------------


def _sim_available() -> bool:
    """True if raccoon was built with DRIVER_BUNDLE=mock."""
    try:
        from raccoon import sim as _sim

        return hasattr(_sim, "mock")
    except ImportError:
        return False


_SIM_SKIP_REASON = (
    "raccoon was not built with DRIVER_BUNDLE=mock — the raccoon.testing "
    "sim fixtures are unavailable. Rebuild with "
    "`pip install -e . --config-settings=cmake.define.DRIVER_BUNDLE=mock` "
    "to run sim-backed tests."
)


# --------------------------------------------------------------------------
# Fixtures
# --------------------------------------------------------------------------


@pytest.fixture(scope="session")
def project_info() -> _project.ProjectInfo:
    """The discovered raccoon project (root, raw yml data, derived SimRobotConfig).

    Session-scoped because the project layout doesn't change during a test
    run. If your tests aren't inside a raccoon project, requesting this
    fixture fails with a clear error.
    """
    try:
        return _project.build_project_info()
    except _project.ProjectNotFoundError as exc:
        pytest.fail(str(exc), pytrace=False)


@pytest.fixture
def robot_sim_config(project_info: _project.ProjectInfo) -> SimRobotConfig:
    """A fresh :class:`SimRobotConfig` derived from the project's yml.

    Function-scoped so individual tests can mutate it (e.g. add
    ``line_sensors``) without leaking state across tests. Use this as an
    override hook when a test needs non-default sim geometry.
    """
    return replace(project_info.sim_config)


@pytest.fixture
def robot(project_info: _project.ProjectInfo) -> Any:
    """Instantiate the project's generated Robot class.

    Skips with a helpful message if the installed raccoon wheel wasn't
    built with the mock driver bundle. Uses a fresh instance per test so
    any mutable state on the Robot (motion history, calibration caches)
    doesn't leak between tests.
    """
    if not _sim_available():
        pytest.skip(_SIM_SKIP_REASON)

    _MockHal.touched = True

    _ensure_project_on_sys_path(project_info.root)
    robot_cls = _try_import_robot_class()
    return robot_cls()


@pytest.fixture
def scene(
    project_info: _project.ProjectInfo,
    robot_sim_config: SimRobotConfig,
) -> Callable[..., None]:
    """Factory that enters a ``use_scene`` context for the current test.

    Call it once per test::

        def test_foo(robot, scene, run_step):
            scene("empty_table.ftmap", start=(20, 50, 0))
            run_step(my_step, robot)

    The scene is detached automatically at the end of the test. Calling
    ``scene`` twice in one test replaces the previous scene.
    """
    if not _sim_available():
        pytest.skip(_SIM_SKIP_REASON)

    active_cm: list = []

    def _enter(
        name: str,
        *,
        start: PoseLike = (0.0, 0.0, 0.0),
        robot: SimRobotConfig | None = None,
        auto_tick: bool = True,
        auto_tick_max_step_sec: float = 0.05,
    ) -> None:
        # Clean up any previous scene the same test set up.
        while active_cm:
            active_cm.pop().__exit__(None, None, None)

        cfg = robot if robot is not None else robot_sim_config
        scene_path = _resolve_scene(name, project_info.root)
        cm = use_scene(
            scene_path,
            robot=cfg,
            start=start,
            auto_tick=auto_tick,
            auto_tick_max_step_sec=auto_tick_max_step_sec,
        )
        cm.__enter__()
        active_cm.append(cm)

    yield _enter

    while active_cm:
        active_cm.pop().__exit__(None, None, None)


@pytest.fixture
def run_step() -> Callable[..., Any]:
    """Sync wrapper that awaits ``step.run_step(robot)`` with a timeout.

    Tests stay sync so this plugin doesn't drag in pytest-asyncio. If you
    need multiple awaits sharing an event loop, write the test as
    ``async def`` and use pytest-asyncio yourself — ``run_step`` is the
    simple path for the common case.
    """

    def _run(step: Any, robot: Any, *, timeout: float = 10.0) -> Any:
        return asyncio.run(asyncio.wait_for(step.run_step(robot), timeout=timeout))

    return _run


# --------------------------------------------------------------------------
# Session finish: dodge the mock-HAL singleton teardown segfault
# --------------------------------------------------------------------------


@pytest.hookimpl(trylast=True)
def pytest_sessionfinish(_session: Any, exitstatus: int) -> None:
    """Bypass interpreter shutdown when the mock HAL was exercised.

    The pybind11-bound MockPlatform singleton has a destruction-order race
    with the motor/IMU wrapper destructors that can segfault at interpreter
    shutdown. The library's own end-to-end tests work around this by
    running the mission in a subprocess that ``os._exit``s before any
    destructors run. The plugin does the same thing here, transparently.

    Only fires when:
    - The ``robot`` fixture was actually used (so we know the mock HAL
      has state to tear down).
    - All tests passed — on failure we want pytest's normal exit path so
      CI gets an accurate error code and any debugger post-mortem runs.
    - The user hasn't set ``RACCOON_TESTING_NO_EXIT_SHORTCUT=1`` to debug.
    """
    if not _MockHal.touched:
        return
    if exitstatus != 0:
        return
    if os.environ.get("RACCOON_TESTING_NO_EXIT_SHORTCUT"):
        return

    # Flush stdio first so pytest's final summary reaches the terminal.
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except (OSError, ValueError):  # pragma: no cover - defensive
        pass
    os._exit(0)
