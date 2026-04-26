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
from pathlib import Path

# Locate the native extension (.so / .pyd) that sits next to this file.
_pkg_dir = Path(__file__).resolve().parent
_ext_spec = None
for _candidate in _pkg_dir.iterdir():
    if _candidate.name.startswith("cam.") and _candidate.suffix in (".so", ".pyd"):
        _ext_spec = importlib.util.spec_from_file_location("raccoon._cam_native", _candidate)
        break

if _ext_spec is None:
    msg = f"Could not find the native cam extension (.so/.pyd) in {_pkg_dir}"
    raise ImportError(msg)

_native = importlib.util.module_from_spec(_ext_spec)
_ext_spec.loader.exec_module(_native)

# Expose the native CamSensor directly so ``from raccoon.cam import CamSensor`` works.
CamSensor = _native.CamSensor

__all__ = ["CamSensor"]
