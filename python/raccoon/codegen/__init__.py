"""raccoon code generation utilities.

This package provides build-time code generation that produces companion
files from annotated source classes.  It uses only the Python standard
library (ast, pathlib, re) so it can run without installing raccoon or
any C++ dependencies.

The raccoon toolchain imports from here rather than duplicating the logic.
"""

from __future__ import annotations

from .step_builder_gen import (
    ParamInfo,
    StepClassInfo,
    scan_file,
    generate_file,
    generate_for_source_dirs,
)

__all__ = [
    "ParamInfo",
    "StepClassInfo",
    "scan_file",
    "generate_file",
    "generate_for_source_dirs",
]
