#!/usr/bin/env python3
"""Scan all @dsl-decorated functions/classes and emit a JSON catalog.

Runs at docs build time (no imports needed — uses AST parsing only).
Outputs docs/_build/dsl-steps.json consumed by the Hugo documentation site.
"""
from __future__ import annotations

import ast
import glob
import json
import os
import sys
import textwrap


def _parse_dsl_decorator(node: ast.expr) -> dict | None:
    """Extract @dsl(...) keyword arguments. Returns None if not a @dsl call."""
    # @dsl  (bare)
    if isinstance(node, ast.Name) and node.id == "dsl":
        return {"hidden": False, "tags": [], "name": None}

    # @dsl(...)
    if isinstance(node, ast.Call):
        func = node.func
        if isinstance(func, ast.Name) and func.id == "dsl":
            kwargs: dict = {"hidden": False, "tags": [], "name": None}
            for kw in node.keywords:
                if kw.arg == "hidden" and isinstance(kw.value, ast.Constant):
                    kwargs["hidden"] = bool(kw.value.value)
                elif kw.arg == "name" and isinstance(kw.value, ast.Constant):
                    kwargs["name"] = kw.value.value
                elif kw.arg == "tags" and isinstance(kw.value, ast.List):
                    kwargs["tags"] = [
                        elt.value
                        for elt in kw.value.elts
                        if isinstance(elt, ast.Constant)
                    ]
            return kwargs

    return None


def _format_signature(node: ast.FunctionDef) -> str:
    """Build a human-readable function signature."""
    args = node.args
    parts: list[str] = []

    # positional args (skip 'self'/'cls')
    all_args = args.args
    defaults = args.defaults
    n_no_default = len(all_args) - len(defaults)

    for i, arg in enumerate(all_args):
        if arg.arg in ("self", "cls"):
            continue
        annotation = ""
        if arg.annotation:
            annotation = f": {ast.unparse(arg.annotation)}"
        if i >= n_no_default:
            default = ast.unparse(defaults[i - n_no_default])
            parts.append(f"{arg.arg}{annotation} = {default}")
        else:
            parts.append(f"{arg.arg}{annotation}")

    # keyword-only args
    for i, arg in enumerate(args.kwonlyargs):
        annotation = ""
        if arg.annotation:
            annotation = f": {ast.unparse(arg.annotation)}"
        default = args.kw_defaults[i]
        if default is not None:
            parts.append(f"{arg.arg}{annotation} = {ast.unparse(default)}")
        else:
            parts.append(f"{arg.arg}{annotation}")

    ret = ""
    if node.returns:
        ret = f" -> {ast.unparse(node.returns)}"

    return f"{node.name}({', '.join(parts)}){ret}"


def _get_docstring(node: ast.AST) -> str:
    """Extract docstring from a class or function node."""
    ds = ast.get_docstring(node)
    return textwrap.dedent(ds).strip() if ds else ""


def _module_path(filepath: str, base_dirs: list[str]) -> str:
    """Convert a file path to a dotted module path."""
    for base in base_dirs:
        if filepath.startswith(base):
            rel = os.path.relpath(filepath, base)
            parts = rel.replace(os.sep, "/").removesuffix(".py").split("/")
            if parts[-1] == "__init__":
                parts = parts[:-1]
            return ".".join(parts)
    return filepath


def scan_file(filepath: str, base_dirs: list[str]) -> list[dict]:
    """Parse a single .py file and return DSL entries."""
    with open(filepath, "r") as f:
        source = f.read()

    try:
        tree = ast.parse(source, filename=filepath)
    except SyntaxError:
        return []

    entries: list[dict] = []
    module = _module_path(filepath, base_dirs)

    for node in ast.walk(tree):
        decorators = getattr(node, "decorator_list", [])
        for dec in decorators:
            meta = _parse_dsl_decorator(dec)
            if meta is None:
                continue

            if meta["hidden"]:
                break

            kind = "class" if isinstance(node, ast.ClassDef) else "function"
            name = meta["name"] or node.name

            entry: dict = {
                "name": name,
                "kind": kind,
                "module": module,
                "tags": meta["tags"],
                "docstring": _get_docstring(node),
            }

            if isinstance(node, ast.FunctionDef):
                entry["signature"] = _format_signature(node)

            entries.append(entry)
            break  # only process first @dsl decorator

    return entries


def main():
    base = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(base)

    # Same dirs as conf.py
    module_python_dirs = sorted(
        p
        for p in glob.glob(os.path.join(root, "modules/*/python"))
        if os.path.isdir(p)
    )

    all_entries: list[dict] = []
    for src_dir in module_python_dirs:
        for py_file in glob.glob(
            os.path.join(src_dir, "**/*.py"), recursive=True
        ):
            all_entries.extend(scan_file(py_file, module_python_dirs))

    # Sort by tag then name
    all_entries.sort(key=lambda e: (e["tags"] or ["zzz"], e["name"]))

    out_dir = os.path.join(base, "_build")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "dsl-steps.json")

    with open(out_path, "w") as f:
        json.dump(all_entries, f, indent=2)

    print(f"DSL catalog: {len(all_entries)} public steps -> {out_path}")


if __name__ == "__main__":
    main()
