#!/usr/bin/env python3
"""Compile-check Python sources without producing on-disk bytecode.

Wired into the CMake build via a custom target so a SyntaxError in any
shipped ``.py`` file fails ``cmake --build`` instead of crashing on the Pi
at import time. We use ``compile()`` (not ``compileall``) so the source
tree stays free of ``__pycache__`` directories — the build's stamp-file
provides incremental behaviour.

Usage:
    python3 tools/check_python_syntax.py FILE [FILE ...]

Exits non-zero on the first failing file, after printing every failure so
contributors fixing one error can see the others in the same run.
"""

from __future__ import annotations

import sys
from pathlib import Path


def check(paths: list[str]) -> int:
    failed = 0
    for arg in paths:
        path = Path(arg)
        try:
            source = path.read_bytes()
        except OSError as exc:
            print(f"{path}: read failed: {exc}", file=sys.stderr)
            failed += 1
            continue
        try:
            compile(source, str(path), "exec", dont_inherit=True)
        except SyntaxError as exc:
            # SyntaxError already carries filename / lineno / offset; format
            # in the same shape as the Python interpreter so editors can
            # jump to the location.
            print(
                f"{exc.filename}:{exc.lineno}:{exc.offset}: {exc.msg}",
                file=sys.stderr,
            )
            if exc.text:
                print(f"    {exc.text.rstrip()}", file=sys.stderr)
            failed += 1
        except ValueError as exc:
            # Raised for things like null bytes in source.
            print(f"{path}: {exc}", file=sys.stderr)
            failed += 1
    return failed


def main(argv: list[str]) -> int:
    if not argv:
        print("usage: check_python_syntax.py FILE [FILE ...]", file=sys.stderr)
        return 2

    failed = check(argv)
    if failed:
        word = "file" if failed == 1 else "files"
        print(
            f"\nPython syntax check failed: {failed} {word} did not compile.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
