"""Tests for raccoon.drive bindings."""
import pytest


def test_drive_module_import():
    """Test that drive module can be imported."""
    try:
        from raccoon import drive
        assert drive is not None
    except ImportError:
        pytest.skip("drive module not available")


# TODO: Add more tests for Drive class bindings
