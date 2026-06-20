#!/usr/bin/env python3
"""Lightweight C++ mutation tester (no mull required).

Mutates a single C++ source file one operator at a time, does an INCREMENTAL
rebuild of the affected test target, and runs ctest. If the tests fail the
mutant is *killed* (good); if they still pass the mutant *survived* (a gap in
the tests). A mutation that no longer compiles is reported separately and
excluded from the score (it is not a useful test of the suite).

Why not mull: mull needs an LLVM-version-matched build (~2-3 h, uncertain C++20
support). This catches the same class of gaps for comparison/arithmetic/logical
logic at the cost of one incremental rebuild per mutant — fast because only the
one changed .o is recompiled and the test relinked.

Mutations are applied only to operators that are SPACE-PADDED (`a + b`, `x <= y`,
`p && q`). The codebase is clang-formatted, so binary operators are padded while
unary/deref/template tokens (`*p`, `-x`, `vector<int>`) are not — this keeps the
mutants compilable and meaningful and avoids template/pointer false positives.

Usage:
  scripts/mutation_cpp.py <build_dir> <source.cpp> <ctest_regex> --target <cmake_target> [--max N]

Example:
  scripts/mutation_cpp.py build-coverage \\
      modules/libstp-foundation/src/pid.cpp 'Pid|Motion' --target test_motion_pid
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess

# operator -> the single mutant we substitute (one per site keeps runtime bounded)
MUT = {
    "+": "-",
    "-": "+",
    "*": "/",
    "/": "*",
    "==": "!=",
    "!=": "==",
    "<=": "<",
    ">=": ">",
    "<": "<=",
    ">": ">=",
    "&&": "||",
    "||": "&&",
}
# Longest-first so "<=" is matched before "<".
_OPS = sorted(MUT, key=len, reverse=True)
_OP_RE = re.compile(r" (" + "|".join(re.escape(o) for o in _OPS) + r") ")


def _string_spans(line: str) -> list[tuple[int, int]]:
    """Char ranges covered by "..." or '...' literals, so we skip operators there."""
    spans, i, n = [], 0, len(line)
    while i < n:
        c = line[i]
        if c in "\"'":
            j = i + 1
            while j < n and line[j] != c:
                if line[j] == "\\":
                    j += 1
                j += 1
            spans.append((i, j))
            i = j + 1
        else:
            i += 1
    return spans


def find_sites(path: str) -> list[tuple[int, int, str, str]]:
    """Return (line_idx, col, op, mutant) for every mutable operator occurrence."""
    sites = []
    with open(path) as f:
        lines = f.readlines()
    in_block = False
    for li, line in enumerate(lines):
        stripped = line.lstrip()
        # Skip preprocessor and comment lines / block comments.
        if stripped.startswith("#"):
            continue
        if in_block:
            if "*/" in line:
                in_block = False
            continue
        if stripped.startswith("//"):
            continue
        if "/*" in line and "*/" not in line:
            in_block = True
        code = line.split("//", 1)[0]
        strings = _string_spans(code)
        for m in _OP_RE.finditer(code):
            col = m.start(1)
            if any(a <= col < b for a, b in strings):
                continue
            op = m.group(1)
            sites.append((li, col, op, MUT[op]))
    return sites, lines


def run(cmd: list[str], timeout: int) -> tuple[int, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)
        return p.returncode, p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        return 124, "timeout"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("build_dir")
    ap.add_argument("source")
    ap.add_argument("ctest_regex")
    ap.add_argument("--target", required=True, help="cmake test target to (re)build")
    ap.add_argument("--max", type=int, default=80, help="cap number of mutants")
    ap.add_argument("--build-timeout", type=int, default=300)
    ap.add_argument("--test-timeout", type=int, default=120)
    args = ap.parse_args()

    jobs = str(os.cpu_count() or 4)
    build = ["cmake", "--build", args.build_dir, "--target", args.target, "-j", jobs]
    ctest = [
        "ctest",
        "--test-dir",
        args.build_dir,
        "-R",
        args.ctest_regex,
        "--output-on-failure",
        "--quiet",
    ]

    # --- baseline must be green ---
    print(f"==> baseline build+test ({args.target}, -R '{args.ctest_regex}')")
    rc, out = run(build, args.build_timeout)
    if rc != 0:
        print("BASELINE BUILD FAILED:\n" + out[-2000:])
        return 2
    rc, out = run(ctest, args.test_timeout)
    if rc != 0:
        print("BASELINE TESTS FAIL — fix tests before mutating:\n" + out[-2000:])
        return 2
    print("    baseline green")

    sites, lines = find_sites(args.source)
    if len(sites) > args.max:
        print(f"    {len(sites)} sites found; capping to {args.max} (raise with --max)")
        sites = sites[: args.max]
    print(f"==> {len(sites)} mutants to check\n")

    killed = survived = uncompilable = 0
    survivors = []
    orig = list(lines)
    try:
        for n, (li, col, op, mut) in enumerate(sites, 1):
            line = orig[li]
            lines[li] = line[:col] + mut + line[col + len(op) :]
            with open(args.source, "w") as f:
                f.writelines(lines)
            lines[li] = line  # restore in-memory for next iteration

            rc, _ = run(build, args.build_timeout)
            if rc != 0:
                uncompilable += 1
                tag = "uncompilable"
            else:
                rc, _ = run(ctest, args.test_timeout)
                if rc != 0:
                    killed += 1
                    tag = "killed"
                else:
                    survived += 1
                    tag = "SURVIVED"
                    survivors.append(
                        f"{args.source}:{li+1}  ' {op} ' -> ' {mut} '  | {orig[li].strip()[:90]}"
                    )
            print(f"  [{n}/{len(sites)}] L{li+1} ' {op} '->' {mut} ' : {tag}")
    finally:
        with open(args.source, "w") as f:
            f.writelines(orig)
        # rebuild clean so the tree is left in a working state
        run(build, args.build_timeout)

    meaningful = killed + survived
    score = 100.0 * killed / meaningful if meaningful else 0.0
    print("\n" + "=" * 60)
    print(f"  Mutation score: {killed}/{meaningful} killed ({score:.1f}%)")
    print(f"  survived: {survived}   uncompilable(excluded): {uncompilable}")
    if survivors:
        print("\n  SURVIVING mutants (gaps — add assertions to kill these):")
        for s in survivors:
            print("   - " + s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
