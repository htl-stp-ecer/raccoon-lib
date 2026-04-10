"""Tests for raccoon.motion bindings."""
import pytest


def test_motion_module_import():
    """Test that motion module can be imported."""
    try:
        from raccoon import motion
        assert motion is not None
    except ImportError:
        pytest.skip("motion module not available")


# TODO: Add more tests for UnifiedMotionPidConfig once bindings are confirmed
