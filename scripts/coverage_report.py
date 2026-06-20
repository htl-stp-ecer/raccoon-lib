#!/usr/bin/env python3
"""Summarise raccoon Python coverage, grouped by owning source module.

`coverage`/pytest-cov measure the *installed* package, so the raw report is
keyed by `.venv/.../site-packages/raccoon/...` paths. That is noisy and hides
the fact that the sources actually live split across `modules/libstp-*/python`
and `python/`. This script reads ``coverage.json``, rewrites every measured
file to its canonical source path, merges any duplicates (e.g. a module that
gets re-imported from the workspace by conftest), and prints a per-module
table with **line, branch and function** coverage plus an overall total.

The gate is enforced on *line* coverage only (branches/functions are shown for
insight): ``--fail-under N`` exits non-zero when overall line coverage drops
below ``N`` and ``--fail-under-module N`` gates the weakest module. Run via
``scripts/coverage.sh``.
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

    Falls back to the measured tail (and module ``"(uninstalled source)"``)
    when no source file matches — e.g. compiled extensions or files deleted
    from the tree.
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


def _pct(cov: int, tot: int) -> float:
    return 100.0 * cov / tot if tot else 0.0


def _fmt_pct(cov: int, tot: int) -> str:
    """Percent cell — a dash when the metric does not apply (no branches/funcs)."""
    return f"{_pct(cov, tot):>5.1f}%" if tot else "    —"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", required=True, type=Path, help="coverage.json path")
    ap.add_argument(
        "--fail-under",
        type=float,
        default=0.0,
        help="fail if overall LINE coverage is below this percent",
    )
    ap.add_argument(
        "--fail-under-module",
        type=float,
        default=0.0,
        help="fail if any single module's LINE coverage is below this percent",
    )
    ap.add_argument(
        "--show-files",
        action="store_true",
        help="also list the least line-covered files (with branch/func %)",
    )
    args = ap.parse_args()

    if not args.json.exists():
        print(f"error: coverage data not found: {args.json}", file=sys.stderr)
        print("       (did the test run produce any coverage?)", file=sys.stderr)
        return 2

    data = json.loads(args.json.read_text())
    roots = _source_roots()

    # canonical path -> merged metric sets
    files: dict[str, dict] = {}
    for measured, info in data.get("files", {}).items():
        canon, module = _canonical(measured, roots)
        e = files.setdefault(
            canon,
            {
                "module": module,
                "stmts": set(),
                "lhit": set(),  # line numbers: all / executed
                "branch": set(),
                "bhit": set(),  # branch arcs: all / executed
                "funcs": {},  # qualified name -> hit (OR-merged)
            },
        )
        if e["module"] in ("?", "(uninstalled source)"):
            e["module"] = module

        executed = set(info.get("executed_lines", []))
        missing = set(info.get("missing_lines", []))
        e["stmts"] |= executed | missing
        e["lhit"] |= executed

        eb = {tuple(a[:2]) for a in info.get("executed_branches", [])}
        mb = {tuple(a[:2]) for a in info.get("missing_branches", [])}
        e["branch"] |= eb | mb
        e["bhit"] |= eb

        for name, fd in info.get("functions", {}).items():
            hit = (
                bool(fd.get("executed_lines")) or fd.get("summary", {}).get("covered_lines", 0) > 0
            )
            e["funcs"][name] = e["funcs"].get(name, False) or hit

    # module -> [lcov, ltot, bcov, btot, fcov, ftot]
    modules: dict[str, list[int]] = {}
    per_file: list[tuple] = []  # (line_pct, canon, lcov, ltot, bcov, btot, fcov, ftot)
    for canon, e in files.items():
        ltot = len(e["stmts"])
        if ltot == 0:
            continue
        lcov = len(e["lhit"] & e["stmts"])
        btot, bcov = len(e["branch"]), len(e["bhit"])
        ftot = len(e["funcs"])
        fcov = sum(1 for v in e["funcs"].values() if v)
        agg = modules.setdefault(e["module"], [0, 0, 0, 0, 0, 0])
        agg[0] += lcov
        agg[1] += ltot
        agg[2] += bcov
        agg[3] += btot
        agg[4] += fcov
        agg[5] += ftot
        per_file.append((_pct(lcov, ltot), canon, lcov, ltot, bcov, btot, fcov, ftot))

    g = [sum(modules[m][i] for m in modules) for i in range(6)]
    overall = _pct(g[0], g[1])

    # ---- module table ----
    name_w = max((len(m) for m in modules), default=20)
    name_w = max(name_w, len("Module"))
    print()
    print(f"{'Module':<{name_w}}  {'Stmts':>6}  {'Lines':>6}  {'Branch':>6}  {'Funcs':>6}")
    print("-" * (name_w + 34))
    weakest = (101.0, "")
    for module in sorted(modules):
        lcov, ltot, bcov, btot, fcov, ftot = modules[module]
        lpct = _pct(lcov, ltot)
        if lpct < weakest[0]:
            weakest = (lpct, module)
        print(
            f"{module:<{name_w}}  {ltot:>6}  {_fmt_pct(lcov, ltot)}  "
            f"{_fmt_pct(bcov, btot)}  {_fmt_pct(fcov, ftot)}"
        )
    print("-" * (name_w + 34))
    print(
        f"{'TOTAL':<{name_w}}  {g[1]:>6}  {_fmt_pct(g[0], g[1])}  "
        f"{_fmt_pct(g[2], g[3])}  {_fmt_pct(g[4], g[5])}"
    )
    print(f"\n  Lines:     {g[0]:>6}/{g[1]:<6} ({overall:.1f}%)   [gate metric]")
    print(f"  Branches:  {g[2]:>6}/{g[3]:<6} ({_pct(g[2], g[3]):.1f}%)")
    print(f"  Functions: {g[4]:>6}/{g[5]:<6} ({_pct(g[4], g[5]):.1f}%)")

    if args.show_files:
        print("\nLeast line-covered files:")
        for lpct, canon, lcov, ltot, bcov, btot, fcov, ftot in sorted(per_file)[:25]:
            print(
                f"  L {lpct:>5.1f}% ({lcov}/{ltot})  "
                f"B {_pct(bcov, btot):>5.1f}%  F {_pct(fcov, ftot):>5.1f}%  {canon}"
            )

    # ---- gate (line coverage only) ----
    rc = 0
    if args.fail_under > 0 and overall < args.fail_under:
        print(
            f"\nFAIL: overall line coverage {overall:.1f}% < required {args.fail_under:.1f}%",
            file=sys.stderr,
        )
        rc = 1
    if args.fail_under_module > 0 and weakest[0] < args.fail_under_module:
        print(
            f"FAIL: module '{weakest[1]}' at {weakest[0]:.1f}% line coverage "
            f"< required {args.fail_under_module:.1f}%",
            file=sys.stderr,
        )
        rc = 1
    if rc == 0 and (args.fail_under > 0 or args.fail_under_module > 0):
        print(f"\nOK: line-coverage gate passed (overall {overall:.1f}%).")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
