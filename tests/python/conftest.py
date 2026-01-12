"""Pytest configuration for libstp binding tests."""
import pytest
import sys
from pathlib import Path

# Add build directory to path for local testing
build_dir = Path(__file__).parent.parent.parent / "build"
if build_dir.exists():
    sys.path.insert(0, str(build_dir))


def _libstp_available():
    """Check if libstp module is available."""
    try:
        import libstp
        return True
    except ImportError:
        return False


# Skip all tests in this directory if libstp is not available
def pytest_collection_modifyitems(config, items):
    if not _libstp_available():
        skip = pytest.mark.skip(reason="libstp module not installed (run pip install -e . first)")
        for item in items:
            if "tests/python" in str(item.fspath):
                item.add_marker(skip)


@pytest.fixture
def chassis_velocity():
    """Factory for ChassisVelocity objects."""
    pytest.importorskip("libstp.foundation")
    from libstp.foundation import ChassisVelocity
    def _make(vx=0.0, vy=0.0, wz=0.0):
        return ChassisVelocity(vx, vy, wz)
    return _make
