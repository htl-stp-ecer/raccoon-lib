#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generate defs.py (class Defs) from a YAML motor definitions file.

- Validates YAML keys against real constructor signatures via inspect.
- Infers nested constructors from __init__ type annotations (incl. Optional[...] Unions).
- Emits clean Python with automatically computed imports.
- Fails fast with actionable errors.

Usage:
  python gen_defs.py config.yml -o defs.py --class-name Defs
"""

from __future__ import annotations

import argparse
import inspect
import os.path
from pathlib import Path
from typing import Any, Dict, Tuple, Set, Union, get_origin, get_args

import yaml  # pip install pyyaml

# Root class for each definition entry. Keep this constant minimal.
ROOT_QUALNAME = "libstp.hal.Motor"


# --------------------------- Introspection utils ---------------------------

def resolve_class(qualname: str) -> type:
    mod_name, cls_name = qualname.rsplit(".", 1)
    module = __import__(mod_name, fromlist=[cls_name])
    return getattr(module, cls_name)


def qualname_of(cls: type) -> str:
    return f"{cls.__module__}.{cls.__name__}"


def is_variadic(p: inspect.Parameter) -> bool:
    return p.kind in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD)


def get_init_params(cls: type) -> Dict[str, inspect.Parameter]:
    sig = inspect.signature(cls.__init__)
    params: Dict[str, inspect.Parameter] = {}
    for name, p in sig.parameters.items():
        if name == "self" or is_variadic(p):
            continue
        if p.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY):
            params[name] = p
    return params


def unwrap_optional(tp: Any) -> Any:
    """Optional[T] -> T; Union[T, None] -> T; otherwise unchanged."""
    origin = get_origin(tp)
    if origin is Union:
        args = tuple(a for a in get_args(tp) if a is not type(None))  # noqa: E721
        if len(args) == 1:
            return args[0]
    return tp


def is_class_annotation(tp: Any) -> bool:
    """True if annotation looks like a concrete class type (not typing, not builtins container)."""
    if tp is inspect._empty:
        return False
    tp = unwrap_optional(tp)
    if isinstance(tp, type):
        # Exclude builtins
        return tp.__module__ not in ("builtins", "typing")
    # If it's a typing construct like list[T], dict[…], etc., we don't treat it as a nested class.
    return False


# --------------------------- Expression building ---------------------------

def build_literal_expr(v: Any) -> str:
    if isinstance(v, str):
        return repr(v)
    if isinstance(v, (int, float, bool)) or v is None:
        return repr(v)
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(build_literal_expr(x) for x in v) + "]"
    if isinstance(v, dict):
        items = ", ".join(f"{repr(k)}: {build_literal_expr(val)}" for k, val in v.items())
        return "{" + items + "}"
    return repr(v)


class ImportSet:
    def __init__(self) -> None:
        self._entries: Set[Tuple[str, str]] = set()

    def add(self, cls: type) -> None:
        if cls.__module__ == "builtins":
            return
        self._entries.add((cls.__module__, cls.__name__))

    def render(self) -> str:
        # group by module
        by_mod: Dict[str, Set[str]] = {}
        for mod, name in self._entries:
            by_mod.setdefault(mod, set()).add(name)
        lines = []
        for mod in sorted(by_mod.keys()):
            names = ", ".join(sorted(by_mod[mod]))
            lines.append(f"from {mod} import {names}")
        return "\n".join(lines)


def validate_kwargs(cls: type, provided: Dict[str, Any], context: str) -> None:
    params = get_init_params(cls)
    unknown = [k for k in provided.keys() if k not in params]
    if unknown:
        raise ValueError(
            f"{context}: unknown argument(s) for {cls.__name__}: {unknown}\n"
            f"Allowed: {sorted(params.keys())}"
        )
    missing = [name for name, p in params.items() if p.default is inspect._empty and name not in provided]
    if missing:
        raise ValueError(
            f"{context}: missing required argument(s) for {cls.__name__}: {missing}"
        )


def build_constructor_expr(
        cls: type,
        data: Dict[str, Any],
        context: str,
        imports: ImportSet,
) -> str:
    """
    Turn dict into 'ClassName(kw=..., nested=NestedClass(...))' by reading __init__ annotations.
    """
    if not isinstance(data, dict):
        raise ValueError(f"{context}: expected mapping for {cls.__name__}, got {type(data).__name__}")

    validate_kwargs(cls, data, context)
    imports.add(cls)

    params = get_init_params(cls)

    pieces = []
    for name, value in data.items():
        ann = params[name].annotation
        ann = unwrap_optional(ann)

        if is_class_annotation(ann) and isinstance(value, dict):
            nested_cls = ann  # type: ignore[assignment]
            nested_expr = build_constructor_expr(nested_cls, value, f"{context}.{name}", imports)
            pieces.append(f"{name}={nested_expr}")
        else:
            pieces.append(f"{name}={build_literal_expr(value)}")

    return f"{cls.__name__}(" + ", ".join(pieces) + ")"


# --------------------------- Source generation ---------------------------

def generate_defs_source(config: Dict[str, Any], class_name: str = "Defs") -> str:
    """
    Build the Python source for defs.py from the config dictionary,
    inferring nested types & imports from annotations only.
    """
    Root = resolve_class(ROOT_QUALNAME)

    imports = ImportSet()
    fields = []

    definitions = config.get("definitions")
    if not isinstance(definitions, dict):
        raise SystemExit("Top-level YAML must be a mapping with key 'definitions:'")

    for field_name, motor_cfg in definitions.items():
        if not isinstance(motor_cfg, dict):
            raise ValueError(f"definitions.{field_name} must be a mapping")
        motor_expr = build_constructor_expr(Root, motor_cfg, f"definitions.{field_name}", imports)
        fields.append((field_name, motor_expr))

    # Compose final source
    lines = []
    lines.append("# This file was generated by gen_defs.py — do not edit by hand.")
    lines.append(imports.render())
    lines.append("")
    lines.append(f"class {class_name}:")
    if not fields:
        lines.append("    pass")
    else:
        for name, expr in fields:
            # ensure valid identifier (basic safeguard; keep your YAML keys sane)
            if not name.isidentifier():
                raise ValueError(f"definitions.{name}: not a valid Python identifier")
            lines.append(f"    {name} = {expr}")
    lines.append("")
    lines.append(f"__all__ = ['{class_name}']")
    lines.append("")
    return "\n".join(lines)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("yaml_path", type=Path, help="Path to YAML config")
    p.add_argument("-o", "--out", type=Path, default=Path("defs.py"))
    p.add_argument("--class-name", default="Defs")
    args = p.parse_args(argv)

    data = yaml.safe_load(args.yaml_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SystemExit("Top-level YAML must be a mapping with key 'definitions:'")

    src = generate_defs_source(data, class_name=args.class_name)

    args.out.write_text(src, encoding="utf-8")
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
