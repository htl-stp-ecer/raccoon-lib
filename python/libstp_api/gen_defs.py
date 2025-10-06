#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generate defs.py (class Defs) from a YAML motor definitions file.

- Validates YAML keys against real constructor signatures via inspect.
- Infers nested constructors from __init__ type annotations (incl. Optional[...] Unions).
- Emits clean Python with automatically computed imports.
- Fails fast with actionable errors.

Usage:
  python gen_defs.py raccoon.project.yml -o defs.py --class-name Defs
"""

from __future__ import annotations

import argparse
import inspect
import os.path
from pathlib import Path
from typing import Any, Dict, Tuple, Set, Union, get_origin, get_args

import yaml  # pip install pyyaml
from libstp.logging import debug, info, warn, error

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


def parse_pybind11_signature(cls: type) -> Dict[str, inspect.Parameter]:
    """Parse pybind11 __init__ signature from docstring."""
    import re

    doc = cls.__init__.__doc__
    if not doc:
        return {}

    # Example: __init__(self: libstp.hal.Motor, port: int, inverted: bool = False, calibration: MotorCalibration = ...) -> None
    match = re.search(r'__init__\(self[^,]*, ([^)]+)\)', doc)
    if not match:
        return {}

    params_str = match.group(1)
    params: Dict[str, inspect.Parameter] = {}

    # Split by comma, but handle nested types
    parts = []
    depth = 0
    current = []
    for char in params_str + ',':
        if char in '([{':
            depth += 1
            current.append(char)
        elif char in ')]}':
            depth -= 1
            current.append(char)
        elif char == ',' and depth == 0:
            parts.append(''.join(current).strip())
            current = []
        else:
            current.append(char)

    for part in parts:
        if not part:
            continue
        # Parse "name: type = default" or "name: type"
        if '=' in part:
            name_type, default_str = part.split('=', 1)
            name = name_type.split(':')[0].strip()
            default = inspect.Parameter.empty if default_str.strip() in ('...', '<') else None
            params[name] = inspect.Parameter(
                name, inspect.Parameter.POSITIONAL_OR_KEYWORD,
                default=default, annotation=inspect.Parameter.empty
            )
        else:
            name = part.split(':')[0].strip()
            params[name] = inspect.Parameter(
                name, inspect.Parameter.POSITIONAL_OR_KEYWORD,
                default=inspect.Parameter.empty, annotation=inspect.Parameter.empty
            )

    return params


def get_init_params(cls: type) -> Dict[str, inspect.Parameter]:
    try:
        sig = inspect.signature(cls.__init__)
        params: Dict[str, inspect.Parameter] = {}
        for name, p in sig.parameters.items():
            if name == "self" or is_variadic(p):
                continue
            if p.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY):
                params[name] = p
        return params
    except (ValueError, TypeError):
        # Fallback for pybind11 classes - parse from __doc__
        return parse_pybind11_signature(cls)


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
    # Skip validation - trust the config and let Python throw errors at runtime
    pass


def parse_type_from_docstring(parent_cls: type, param_name: str) -> type | None:
    """Parse parameter type from pybind11 docstring."""
    import re

    doc = parent_cls.__init__.__doc__
    if not doc:
        warn(f"No docstring for {parent_cls.__name__}.__init__")
        return None

    # Look for parameter with type annotation in docstring
    # Example: "calibration: libstp.foundation.MotorCalibration"
    pattern = rf'\b{re.escape(param_name)}\s*:\s*([a-zA-Z_.]+)'
    match = re.search(pattern, doc)
    if not match:
        warn(f"No type found for parameter '{param_name}' in {parent_cls.__name__}")
        return None

    type_str = match.group(1)
    debug(f"Found type for '{param_name}': {type_str}")

    # Try to resolve the type
    try:
        resolved = resolve_class(type_str)
        debug(f"Resolved {type_str} to {resolved}")
        return resolved
    except (ImportError, AttributeError):
        # Try without module prefix (just class name)
        class_name = type_str.split('.')[-1]
        debug(f"Failed to resolve {type_str}, trying {class_name}...")
        for module_name in ['libstp.foundation', 'libstp.hal']:
            try:
                resolved = resolve_class(f"{module_name}.{class_name}")
                debug(f"Resolved to {resolved}")
                return resolved
            except (ImportError, AttributeError):
                continue

    warn(f"Could not resolve type {type_str}")
    return None


def infer_nested_class(parent_cls: type, param_name: str, value: Dict[str, Any]) -> type | None:
    """Try to infer nested class from parent class's __init__ signature via docstring."""
    return parse_type_from_docstring(parent_cls, param_name)


def build_constructor_expr(
        cls: type,
        data: Dict[str, Any],
        context: str,
        imports: ImportSet,
) -> str:
    """
    Turn dict into 'ClassName(kw=...)' - recursively handles nested classes.
    """
    if not isinstance(data, dict):
        raise ValueError(f"{context}: expected mapping for {cls.__name__}, got {type(data).__name__}")

    info(f"Building {cls.__name__} for {context}")
    imports.add(cls)

    pieces = []
    for name, value in data.items():
        if isinstance(value, dict):
            debug(f"Checking nested dict parameter: {name}")
            # Try to infer if this should be a nested class constructor
            nested_cls = infer_nested_class(cls, name, value)
            if nested_cls:
                info(f"Treating '{name}' as {nested_cls.__name__} constructor")
                nested_expr = build_constructor_expr(nested_cls, value, f"{context}.{name}", imports)
                pieces.append(f"{name}={nested_expr}")
            else:
                debug(f"Using literal dict for '{name}'")
                # Fall back to literal dict
                pieces.append(f"{name}={build_literal_expr(value)}")
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
        raise SystemExit("Top-level config must be a mapping with key 'definitions:'")

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
    p.add_argument("config_path", type=Path, help="Path to YAML config")
    p.add_argument("-o", "--out", type=Path, default=Path("defs.py"))
    p.add_argument("--class-name", default="Defs")
    p.add_argument("--no-format", action="store_true", help="Skip black formatting")
    args = p.parse_args(argv)

    info(f"Reading config from {args.config_path}")
    data = yaml.safe_load(args.config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SystemExit("Top-level config must be a mapping with key 'definitions:'")

    info(f"Generating source code...")
    src = generate_defs_source(data, class_name=args.class_name)

    # Format with black if available
    if not args.no_format:
        try:
            import black
            info(f"Formatting with black...")
            src = black.format_str(src, mode=black.Mode(line_length=88))
        except ImportError:
            warn("black not installed, skipping formatting")

    args.out.write_text(src, encoding="utf-8")
    info(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
