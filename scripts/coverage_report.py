#!/usr/bin/env python3
"""Summarise raccoon Python coverage, grouped by owning source module.

`coverage`/pytest-cov measure the *installed* package, so the raw report is
keyed by `.venv/.../site-packages/raccoon/...` paths. That is noisy and hides
the fact that the sources actually live split across `modules/libstp-*/python`
and `python/`. This script reads ``coverage.json``, rewrites every measured
file to its canonical source path, merges any duplicates (e.g. a module that
gets re-imported from the workspace by conftest), and prints a per-module
table plus an overall total.

It is the gate the project enforces: ``--fail-under N`` exits non-zero when the
overall *line* coverage drops below ``N`` (and ``--fail-under-module N`` can
gate the weakest module). Run via ``scripts/coverage.sh``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def _source_roots() -> list[Path]:
    """Directories that contain a `raccoon/` package fragment, in lookup order."""
    roots = [REPO_ROOT / "python"]
    roots += sorted(REPO_ROOT.glob("modules/libstp-*/python"))
    return [r for r in roots if (r / "raccoon").is_dir()]


def _canonical(measured: str, roots: list[Path]) -> tuple[str, str]:
    """Map a measured file path to (canonical_source_path, owning_module).

    Falls back to the measured path (and module ``"?"``) when no source file
    matches — e.g. compiled extensions or files deleted from the tree.
    """
    p = measured.replace("\\", "/")
    marker = "/raccoon/"
    idx = p.rfind(marker)
    if idx == -1:
        return measured, "?"
    rel = "raccoon/" + p[idx + len(marker) :]
    for root in roots:
        cand = root / rel
        if cand.exists():
            canon = cand.relative_to(REPO_ROOT).as_posix()
            if root.name == "python" and root.parent == REPO_ROOT:
                module = "python (core)"
            else:
                # modules/libstp-step/python -> libstp-step
                module = root.parent.name
            return canon, module
    return rel, "(uninstalled source)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", required=True, type=Path, help="coverage.json path")
    ap.add_argument(
        "--fail-under",
        type=float,
        default=0.0,
        help="fail if overall line coverage is below this percent",
    )
    ap.add_argument(
        "--fail-under-module",
        type=float,
        default=0.0,
        help="fail if any single module is below this percent",
    )
    ap.add_argument("--show-files", action="store_true", help="also list the least-covered files")
    args = ap.parse_args()

    if not args.json.exists():
        print(f"error: coverage data not found: {args.json}", file=sys.stderr)
        print("       (did the test run produce any coverage?)", file=sys.stderr)
        return 2

    data = json.loads(args.json.read_text())
    roots = _source_roots()

    # canonical path -> {"stmts": set[int], "covered": set[int], "module": str}
    files: dict[str, dict] = {}
    for measured, info in data.get("files", {}).items():
        canon, module = _canonical(measured, roots)
        executed = set(info.get("executed_lines", []))
        missing = set(info.get("missing_lines", []))
        entry = files.setdefault(canon, {"stmts": set(), "covered": set(), "module": module})
        entry["stmts"] |= executed | missing
        entry["covered"] |= executed
        if entry["module"] in ("?", "(uninstalled source)"):
            entry["module"] = module

    # Aggregate per module.
    modules: dict[str, list[int]] = {}  # module -> [covered, total]
    per_file: list[tuple[float, str, int, int]] = []
    for canon, e in files.items():
        total = len(e["stmts"])
        covered = len(e["covered"] & e["stmts"])
        if total == 0:
            continue
        agg = modules.setdefault(e["module"], [0, 0])
        agg[0] += covered
        agg[1] += total
        per_file.append((100.0 * covered / total, canon, covered, total))

    grand_cov = sum(v[0] for v in modules.values())
    grand_tot = sum(v[1] for v in modules.values())
    overall = 100.0 * grand_cov / grand_tot if grand_tot else 0.0

    # ---- print module table ----
    name_w = max((len(m) for m in modules), default=20)
    name_w = max(name_w, len("Module"))
    print()
    print(f"{'Module':<{name_w}}  {'Stmts':>7}  {'Miss':>6}  {'Cover':>7}")
    print("-" * (name_w + 26))
    weakest = (101.0, "")
    for module in sorted(modules):
        cov, tot = modules[module]
        pct = 100.0 * cov / tot if tot else 0.0
        if pct < weakest[0]:
            weakest = (pct, module)
        print(f"{module:<{name_w}}  {tot:>7}  {tot - cov:>6}  {pct:>6.1f}%")
    print("-" * (name_w + 26))
    print(f"{'TOTAL':<{name_w}}  {grand_tot:>7}  {grand_tot - grand_cov:>6}  {overall:>6.1f}%")

    if args.show_files:
        print("\nLeast-covered files:")
        for pct, canon, cov, tot in sorted(per_file)[:25]:
            print(f"  {pct:>5.1f}%  ({cov}/{tot})  {canon}")

    # ---- gate ----
    rc = 0
    if args.fail_under > 0 and overall < args.fail_under:
        print(
            f"\nFAIL: overall line coverage {overall:.1f}% < required {args.fail_under:.1f}%",
            file=sys.stderr,
        )
        rc = 1
    if args.fail_under_module > 0 and weakest[0] < args.fail_under_module:
        print(
            f"FAIL: module '{weakest[1]}' at {weakest[0]:.1f}% "
            f"< required {args.fail_under_module:.1f}%",
            file=sys.stderr,
        )
        rc = 1
    if rc == 0 and (args.fail_under > 0 or args.fail_under_module > 0):
        print(f"\nOK: coverage gate passed (overall {overall:.1f}%).")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
