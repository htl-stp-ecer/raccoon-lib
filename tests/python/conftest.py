"""Pytest configuration for raccoon binding tests."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

# Add build directory to path for local testing
build_dir = Path(__file__).parent.parent.parent / "build"
if build_dir.exists():
    sys.path.insert(0, str(build_dir))

repo_root = Path(__file__).parent.parent.parent


def _load_workspace_module(module_name: str, path: Path, *, is_package: bool = False) -> None:
    kwargs = {"submodule_search_locations": [str(path.parent)]} if is_package else {}
    spec = importlib.util.spec_from_file_location(module_name, path, **kwargs)
    if spec is None or spec.loader is None:
        msg = f"could not create spec for {module_name} from {path}"
        raise RuntimeError(msg)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)


def _prefer_workspace_motion_python() -> None:
    if not _libstp_available():
        return

    import raccoon.step.motion as motion_pkg

    motion_dir = repo_root / "modules/libstp-motion/python/raccoon/step/motion"
    for module_name in (
        "raccoon.step.motion.path.executor",
        "raccoon.step.motion.path.compiler",
        "raccoon.step.motion.path.motion_factory",
        "raccoon.step.motion.path",
        "raccoon.step.motion.smooth_path",
    ):
        sys.modules.pop(module_name, None)
    _load_workspace_module(
        "raccoon.step.motion.path",
        motion_dir / "path/__init__.py",
        is_package=True,
    )
    _load_workspace_module(
        "raccoon.step.motion.smooth_path",
        motion_dir / "smooth_path.py",
    )
    motion_pkg.path = sys.modules["raccoon.step.motion.path"]
    motion_pkg.smooth_path = sys.modules["raccoon.step.motion.smooth_path"]


def _libstp_available() -> bool:
    """Check if raccoon module is available without importing it.

    Using ``importlib.util.find_spec`` avoids the side-effects of an actual
    ``import raccoon`` (signal handlers, atexit hooks, banner log) just to
    answer the question "is the wheel installed?".
    """
    return importlib.util.find_spec("raccoon") is not None


# Skip all tests in this directory if raccoon is not available
def pytest_collection_modifyitems(config, items):
    if not _libstp_available():
        skip = pytest.mark.skip(reason="raccoon module not installed (run pip install -e . first)")
        for item in items:
            if "tests/python" in str(item.fspath):
                item.add_marker(skip)


_prefer_workspace_motion_python()


@pytest.fixture
def chassis_velocity():
    """Factory for ChassisVelocity objects."""
    pytest.importorskip("raccoon.foundation")
    from raccoon.foundation import ChassisVelocity

    def _make(vx=0.0, vy=0.0, wz=0.0):
        return ChassisVelocity(vx, vy, wz)

    return _make
