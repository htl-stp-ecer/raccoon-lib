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


@pytest.fixture
def chassis_velocity():
    """Factory for ChassisVelocity objects."""
    pytest.importorskip("raccoon.foundation")
    from raccoon.foundation import ChassisVelocity

    def _make(vx=0.0, vy=0.0, wz=0.0):
        return ChassisVelocity(vx, vy, wz)

    return _make
