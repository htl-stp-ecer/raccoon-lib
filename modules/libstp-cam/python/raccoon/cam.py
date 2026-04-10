"""
Convenience wrapper around the native CamSensor C++ binding.

Usage::

    from raccoon.cam import CamSensor

    cam = CamSensor()
    if cam.is_detected("orange"):
        print(f"orange blob at ({cam.get_blob_x('orange')}, {cam.get_blob_y('orange')})")

The native pybind11 extension is compiled as ``cam.<abi-tag>.so`` and lives
next to this file.  We import it via importlib so this pure-Python module
can shadow it without creating a circular import.
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
from typing import List

# Locate the native extension (.so / .pyd) that sits next to this file.
_pkg_dir = os.path.dirname(os.path.abspath(__file__))
_ext_spec = None
for _fname in os.listdir(_pkg_dir):
    if _fname.startswith("cam.") and (_fname.endswith(".so") or _fname.endswith(".pyd")):
        _ext_path = os.path.join(_pkg_dir, _fname)
        _ext_spec = importlib.util.spec_from_file_location("raccoon._cam_native", _ext_path)
        break

if _ext_spec is None:
    raise ImportError("Could not find the native cam extension (.so/.pyd) in " + _pkg_dir)

_native = importlib.util.module_from_spec(_ext_spec)
_ext_spec.loader.exec_module(_native)

# Expose the native CamSensor directly so ``from raccoon.cam import CamSensor`` works.
CamSensor = _native.CamSensor

__all__ = ["CamSensor"]
