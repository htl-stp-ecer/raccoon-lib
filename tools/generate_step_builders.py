#!/usr/bin/env python3
"""CLI entry point for step builder code generation.

The actual logic lives in ``libstp.codegen.step_builder_gen``.
This script adds the library's Python path and provides CLI argument parsing
so it can be run standalone during the build (no pip install needed).

Usage:
    python tools/generate_step_builders.py                  # scan all modules
    python tools/generate_step_builders.py --dry-run        # preview only
    python tools/generate_step_builders.py --module motion  # one module
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Load the codegen module directly to avoid triggering libstp/__init__.py
# which requires the C++ extension (libstp._core) and other native deps.
_LIB_ROOT = Path(__file__).resolve().parent.parent
_CODEGEN_PATH = _LIB_ROOT / "python" / "libstp" / "codegen" / "step_builder_gen.py"

import importlib.util as _ilu

_spec = _ilu.spec_from_file_location("step_builder_gen", _CODEGEN_PATH)
_mod = _ilu.module_from_spec(_spec)  # type: ignore[arg-type]
sys.modules[_spec.name] = _mod
_spec.loader.exec_module(_mod)  # type: ignore[union-attr]

generate_for_source_dirs = _mod.generate_for_source_dirs
camel_to_snake = _mod.camel_to_snake


def _collect_source_dirs(lib_root: Path, module: str | None) -> list[Path]:
    modules_dir = lib_root / "modules"
    if module:
        candidates = list(modules_dir.glob(f"libstp-{module}/python/libstp"))
        if not candidates:
            print(f"ERROR: No module 'libstp-{module}' found", file=sys.stderr)
            sys.exit(1)
        return candidates

    return sorted(
        d / "python" / "libstp"
        for d in modules_dir.iterdir()
        if (d / "python" / "libstp").is_dir()
    )


def main():
    parser = argparse.ArgumentParser(
        description="Generate StepBuilder classes from @dsl_step annotations"
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--module", type=str, default=None)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="  %(message)s",
    )

    source_dirs = _collect_source_dirs(_LIB_ROOT, args.module)
    print(f"Scanning {len(source_dirs)} module(s) for @dsl_step classes...")

    results = generate_for_source_dirs(source_dirs, dry_run=args.dry_run)

    total_builders = sum(
        len([l for l in code.splitlines() if l.startswith("class ") and "Builder" in l])
        for code in results.values()
    )
    action = "Would generate" if args.dry_run else "Generated"
    print(f"{action} {total_builders} builder(s) in {len(results)} file(s)")


if __name__ == "__main__":
    main()
