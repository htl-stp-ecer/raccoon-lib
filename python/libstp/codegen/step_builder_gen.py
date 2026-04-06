"""Core logic for generating StepBuilder classes from @dsl_step annotations.

Scans Python source files via AST (no imports needed) and generates
companion ``_dsl.py`` files containing:

- A ``<ClassName>Builder(StepBuilder)`` with fluent setter methods
- A ``snake_case()`` factory function decorated with ``@dsl(tags=...)``

This module uses only the Python standard library.
"""

from __future__ import annotations

import ast
import logging
import re
import sys
import textwrap
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

logger = logging.getLogger("libstp.codegen")

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class ParamInfo:
    """Information about a single ``__init__`` parameter."""

    name: str
    annotation: str  # source-level type annotation, or ""
    default: str  # source-level default expression, or ""
    has_default: bool = False
    default_refs: List[str] = field(default_factory=list)  # top-level names in default


@dataclass
class StepClassInfo:
    """Information about a ``@dsl_step`` class extracted from AST."""

    name: str
    params: List[ParamInfo]
    source_file: Path
    bases: List[str] = field(default_factory=list)
    tags: List[str] = field(default_factory=list)
    docstring: str = ""


# ---------------------------------------------------------------------------
# Docstring parsing and generation
# ---------------------------------------------------------------------------

# Fallback descriptions for very common parameter names.
# Class docstring ``Args:`` sections are the PRIMARY source —
# these only kick in when the class doesn't document a param.
_COMMON_PARAM_DEFAULTS: Dict[str, str] = {
    "cm": "Distance in centimeters. Omit to use condition-only mode.",
    "degrees": "Angle in degrees. Omit to use condition-only mode.",
    "speed": "Fraction of max speed, 0.0 to 1.0.",
    "until": (
        "Stop condition for early termination "
        "(e.g., ``on_black(sensor)``). Can also be chained "
        "via the ``.until()`` builder method."
    ),
}


@dataclass
class _ParsedDocstring:
    """Sections extracted from a class docstring."""

    summary: str = ""
    body: str = ""
    args: Dict[str, str] = field(default_factory=dict)
    example: str = ""


def _parse_class_docstring(raw: str) -> _ParsedDocstring:
    """Split a class docstring into summary, body, Args, and Example sections."""
    if not raw:
        return _ParsedDocstring()

    text = textwrap.dedent(raw).strip()
    result = _ParsedDocstring()

    # Split into lines and extract summary (first paragraph)
    lines = text.split("\n")
    summary_lines: List[str] = []
    rest_start = 0
    for i, line in enumerate(lines):
        if line.strip() == "" and summary_lines:
            rest_start = i + 1
            break
        summary_lines.append(line.strip())
        rest_start = i + 1

    result.summary = " ".join(summary_lines)

    rest = "\n".join(lines[rest_start:]).strip()
    if not rest:
        return result

    # Extract Example:: section (everything after "Example::")
    example_match = re.search(r"^Example::[ \t]*\n", rest, re.MULTILINE)
    if example_match:
        result.example = textwrap.dedent(rest[example_match.end():]).strip()
        rest = rest[: example_match.start()].strip()

    # Extract Args: section
    args_match = re.search(r"^Args:\s*\n", rest, re.MULTILINE)
    if args_match:
        args_text = rest[args_match.end():]
        rest = rest[: args_match.start()].strip()
        result.args = _parse_args_section(args_text)

    result.body = rest.strip()
    return result


def _parse_args_section(text: str) -> Dict[str, str]:
    """Parse an Args: section into a name→description mapping."""
    args: Dict[str, str] = {}
    current_name: Optional[str] = None
    current_lines: List[str] = []

    for line in text.split("\n"):
        # Check for a new arg: "    name: description" or "    name (type): desc"
        m = re.match(r"^    (\w+)(?:\s*\([^)]*\))?\s*:\s*(.*)", line)
        if m:
            if current_name:
                args[current_name] = " ".join(current_lines).strip()
            current_name = m.group(1)
            current_lines = [m.group(2).strip()] if m.group(2).strip() else []
        elif current_name and line.startswith("        "):
            current_lines.append(line.strip())
        elif not line.strip():
            if current_name:
                args[current_name] = " ".join(current_lines).strip()
                current_name = None
                current_lines = []
        else:
            # Non-indented non-empty line means end of Args section
            break

    if current_name:
        args[current_name] = " ".join(current_lines).strip()
    return args


def _build_factory_docstring(info: StepClassInfo) -> str:
    """Build a complete docstring for the generated factory function."""
    parsed = _parse_class_docstring(info.docstring)
    fn = camel_to_snake(info.name)
    builder = f"{info.name}Builder"

    sections: List[str] = []

    # Summary
    summary = parsed.summary or f"Create a {info.name} step."
    sections.append(summary)

    # Body (how it works, prerequisites, etc.)
    if parsed.body:
        sections.append("")
        sections.append(parsed.body)

    # Args — class docstring Args: is primary, _COMMON_PARAM_DEFAULTS is fallback
    if info.params:
        sections.append("")
        sections.append("Args:")
        for p in info.params:
            desc = parsed.args.get(p.name) or _COMMON_PARAM_DEFAULTS.get(p.name, "")
            if not desc:
                raise ValueError(
                    f"{info.name}.__init__ param '{p.name}' has no documentation. "
                    f"Add an Args: section to the class docstring or add "
                    f"'{p.name}' to _COMMON_PARAM_DEFAULTS in step_builder_gen.py"
                )
            sections.append(f"    {p.name}: {desc}")

    # Returns
    fluent_methods = ", ".join(f"``.{p.name}()``" for p in info.params)
    sections.append("")
    sections.append("Returns:")
    sections.append(
        f"    A {builder} (chainable via {fluent_methods},"
        f" ``.on_anomaly()``, ``.skip_timing()``)."
    )

    # Example
    if parsed.example:
        sections.append("")
        sections.append("Example::")
        sections.append("")
        # Indent example content
        for line in parsed.example.split("\n"):
            sections.append(f"    {line}" if line.strip() else "")
    else:
        # Auto-generate a minimal example
        sections.append("")
        sections.append("Example::")
        sections.append("")
        sections.append(f"    from libstp.step.motion import {fn}")
        sections.append("")
        # Generate a simple call with the first non-until param
        call_args = []
        for p in info.params:
            if p.name == "until":
                continue
            if p.name == "cm":
                call_args.append("25")
                break
            if p.name == "degrees":
                call_args.append("90")
                break
        sections.append(f"    {fn}({', '.join(call_args)})")

    return "\n".join(sections)


# ---------------------------------------------------------------------------
# AST scanning
# ---------------------------------------------------------------------------


def _is_dsl_step_decorator(node: ast.expr) -> bool:
    """Check if an AST decorator node is ``@dsl_step`` or ``@dsl_step(...)``."""
    if isinstance(node, ast.Name) and node.id == "dsl_step":
        return True
    if isinstance(node, ast.Attribute) and node.attr == "dsl_step":
        return True
    if isinstance(node, ast.Call):
        return _is_dsl_step_decorator(node.func)
    return False


def _extract_dsl_step_tags(decorator_list: List[ast.expr]) -> List[str]:
    """Extract tags from a ``@dsl_step(tags=[...])`` decorator."""
    for node in decorator_list:
        if isinstance(node, ast.Call) and _is_dsl_step_decorator(node.func):
            for kw in node.keywords:
                if kw.arg == "tags" and isinstance(kw.value, (ast.List, ast.Tuple)):
                    return [
                        elt.value
                        for elt in kw.value.elts
                        if isinstance(elt, ast.Constant) and isinstance(elt.value, str)
                    ]
    return []


def _ast_to_source(node: ast.expr) -> str:
    return ast.unparse(node)


def _extract_init_params(cls_node: ast.ClassDef) -> List[ParamInfo]:
    for item in cls_node.body:
        if isinstance(item, ast.FunctionDef) and item.name == "__init__":
            return _parse_function_params(item)
    return []


def _extract_default_refs(node: ast.expr) -> List[str]:
    """Extract top-level names referenced in a default-value AST node.

    For example, ``LineSide.LEFT`` yields ``["LineSide"]``, while
    literal values like ``0.5`` or ``None`` yield nothing.
    """
    refs: List[str] = []
    for child in ast.walk(node):
        if isinstance(child, ast.Name):
            # Skip builtins and common literals
            if child.id not in (
                "None", "True", "False", "int", "float", "str", "bool",
                "list", "dict", "tuple", "set", "Optional", "Union", "List",
            ):
                refs.append(child.id)
    return refs


_ANNOTATION_BUILTINS = frozenset({
    "None", "True", "False", "int", "float", "str", "bool", "bytes", "object",
    "Optional", "Union", "List", "Dict", "Tuple", "Set", "Any", "Callable",
    "Type", "ClassVar", "Final", "Literal",
})


def _extract_annotation_refs(annotation: str) -> List[str]:
    """Extract top-level names referenced in a type annotation string.

    Parses the annotation as an expression and returns every bare ``Name``
    node that isn't a builtin or typing construct.  For example,
    ``"Servo | ServoPreset"`` yields ``["Servo", "ServoPreset"]``.
    """
    if not annotation:
        return []
    try:
        tree = ast.parse(annotation, mode="eval")
    except SyntaxError:
        return []
    return [
        node.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Name) and node.id not in _ANNOTATION_BUILTINS
    ]


def _find_imports_for_names(source_file: Path, needed_names: set) -> List[str]:
    """Return import statements from *source_file* that provide *needed_names*.

    Scans the entire AST of the source file (including function bodies) for
    ``from X import Y`` statements and returns the subset that covers the
    requested names.  Each returned string is a ready-to-emit import line.
    """
    if not needed_names:
        return []
    try:
        tree = ast.parse(source_file.read_text(encoding="utf-8"))
    except (SyntaxError, OSError):
        return []

    # Map: provided name → import line (first occurrence wins)
    found: Dict[str, str] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        module = node.module or ""
        dots = "." * (node.level or 0)
        for alias in node.names:
            exported = alias.asname or alias.name
            if exported in needed_names and exported not in found:
                fragment = alias.name + (f" as {alias.asname}" if alias.asname else "")
                found[exported] = f"from {dots}{module} import {fragment}"

    # Return in stable order (same as needed_names iteration would give)
    return [found[n] for n in sorted(needed_names) if n in found]


def _parse_function_params(func: ast.FunctionDef) -> List[ParamInfo]:
    params: List[ParamInfo] = []
    args = func.args
    all_args = args.args[1:]  # skip self
    num_defaults = len(args.defaults)
    num_args = len(all_args)

    for i, arg in enumerate(all_args):
        annotation = _ast_to_source(arg.annotation) if arg.annotation else ""
        default_index = i - (num_args - num_defaults)
        has_default = default_index >= 0
        default_node = args.defaults[default_index] if has_default else None
        default = _ast_to_source(default_node) if default_node else ""
        default_refs = _extract_default_refs(default_node) if default_node else []
        params.append(ParamInfo(arg.arg, annotation, default, has_default, default_refs))

    for i, arg in enumerate(args.kwonlyargs):
        annotation = _ast_to_source(arg.annotation) if arg.annotation else ""
        kw_default = args.kw_defaults[i]
        has_default = kw_default is not None
        default = _ast_to_source(kw_default) if kw_default else ""
        default_refs = _extract_default_refs(kw_default) if kw_default else []
        params.append(ParamInfo(arg.arg, annotation, default, has_default, default_refs))

    return params


def scan_file(source_file: Path) -> List[StepClassInfo]:
    """Scan a Python source file for ``@dsl_step`` classes."""
    try:
        tree = ast.parse(
            source_file.read_text(encoding="utf-8"),
            filename=str(source_file),
        )
    except (SyntaxError, OSError) as e:
        logger.warning("Could not parse %s: %s", source_file, e)
        return []

    results = []
    for node in ast.iter_child_nodes(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        if not any(_is_dsl_step_decorator(d) for d in node.decorator_list):
            continue
        params = _extract_init_params(node)
        bases = [_ast_to_source(b) for b in node.bases]
        tags = _extract_dsl_step_tags(node.decorator_list)
        docstring = ast.get_docstring(node) or ""
        results.append(StepClassInfo(
            node.name, params, source_file, bases, tags, docstring,
        ))

    return results


# ---------------------------------------------------------------------------
# Code generation
# ---------------------------------------------------------------------------

_SENTINEL = "_UNSET"


def camel_to_snake(name: str) -> str:
    """Convert CamelCase to snake_case."""
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", name)
    s = re.sub(r"([a-z\d])([A-Z])", r"\1_\2", s)
    return s.lower()


def _generate_builder_class(info: StepClassInfo) -> str:
    cls = info.name
    builder = f"{cls}Builder"
    lines = [
        f"class {builder}(StepBuilder):",
        f'    """Builder for {cls}. Auto-generated — do not edit."""',
        "",
        "    def __init__(self):",
        "        super().__init__()",
    ]
    for p in info.params:
        val = p.default if p.has_default else _SENTINEL
        lines.append(f"        self._{p.name} = {val}")
    lines.append("")

    for p in info.params:
        hint = f": {p.annotation}" if p.annotation else ""
        lines += [
            f"    def {p.name}(self, value{hint}):",
            f"        self._{p.name} = value",
            "        return self",
            "",
        ]

    lines.append("    def _build(self):")
    lines.append("        kwargs = {}")
    for p in info.params:
        if p.has_default:
            lines.append(f"        kwargs['{p.name}'] = self._{p.name}")
        else:
            lines.append(f"        if self._{p.name} is not {_SENTINEL}:")
            lines.append(f"            kwargs['{p.name}'] = self._{p.name}")
    lines.append(f"        return {cls}(**kwargs)")
    lines.append("")
    return "\n".join(lines)


def _generate_dsl_function(info: StepClassInfo) -> str:
    cls = info.name
    builder = f"{cls}Builder"
    fn = camel_to_snake(cls)

    sig_parts = []
    for p in info.params:
        hint = f": {p.annotation}" if p.annotation else ""
        default = p.default if p.has_default else _SENTINEL
        sig_parts.append(f"{p.name}{hint} = {default}")

    # Build the docstring
    raw_doc = _build_factory_docstring(info)

    lines = []
    if info.tags:
        lines.append(f"@dsl(tags={info.tags!r})")
    lines.append(f"def {fn}({', '.join(sig_parts)}):")

    # Emit docstring — indent each line by 4 spaces (function body)
    lines.append('    """')
    for dl in raw_doc.split("\n"):
        lines.append(f"    {dl}" if dl.strip() else "")
    lines.append('    """')

    lines.append(f"    b = {builder}()")
    for p in info.params:
        if p.has_default:
            lines.append(f"    b._{p.name} = {p.name}")
        else:
            lines.append(f"    if {p.name} is not {_SENTINEL}:")
            lines.append(f"        b._{p.name} = {p.name}")
    lines.append("    return b")
    lines.append("")
    return "\n".join(lines)


def generate_file(classes: List[StepClassInfo], source_file: Path) -> str:
    """Generate a complete ``_dsl.py`` file for the given step classes."""
    has_tags = any(c.tags for c in classes)
    lines = [
        '"""Auto-generated step builders and DSL functions — DO NOT EDIT.',
        "",
        f"Source: {source_file.name}",
        '"""',
        "",
        "from __future__ import annotations",
        "",
        f"{_SENTINEL} = object()",
        "",
        "from libstp.step.step_builder import StepBuilder",
        "from libstp.step.condition import StopCondition",
    ]
    if has_tags:
        lines.append("from libstp.step.annotation import dsl")
    # Collect names from the source module needed for default values
    class_names = [c.name for c in classes]
    default_ref_names: List[str] = []
    for c in classes:
        for p in c.params:
            for ref in p.default_refs:
                if ref not in class_names and ref not in default_ref_names:
                    default_ref_names.append(ref)
    all_imports = class_names + default_ref_names
    lines += [
        f"from .{source_file.stem} import {', '.join(all_imports)}",
        "",
    ]
    # Collect names used only in type annotations that aren't already imported.
    # These are needed so that get_type_hints() (used by stubgen/mypy) can resolve
    # them at runtime; without this the stub generator silently skips the module.
    _already_covered = set(all_imports) | {
        "StepBuilder", "StopCondition", "dsl",
        # builtins / typing — already in _ANNOTATION_BUILTINS, but be explicit
        "Optional", "Union", "List", "Dict", "Tuple", "Set", "Any", "Callable",
    }
    annotation_only_refs: List[str] = []
    for c in classes:
        for p in c.params:
            for ref in _extract_annotation_refs(p.annotation):
                if ref not in _already_covered and ref not in annotation_only_refs:
                    annotation_only_refs.append(ref)
    if annotation_only_refs:
        # Names that come from an external module — re-emit that import line.
        found_via_import = _find_imports_for_names(source_file, set(annotation_only_refs))
        found_names = {
            line.rsplit("import ", 1)[-1].split(" as ")[0].strip()
            for line in found_via_import
        }
        for import_line in found_via_import:
            lines.append(import_line)
        # Names defined directly in the source file (not imported there) —
        # add to the source-module import so get_type_hints() can resolve them.
        locally_defined = [r for r in annotation_only_refs if r not in found_names]
        if locally_defined:
            # Rewrite the already-emitted source-module import to include them.
            src_import_idx = next(
                i for i, l in enumerate(lines) if l.startswith(f"from .{source_file.stem} import ")
            )
            existing = lines[src_import_idx]
            lines[src_import_idx] = existing.rstrip() + ", " + ", ".join(locally_defined)
    lines.append("")

    exports: List[str] = []
    for info in classes:
        builder = f"{info.name}Builder"
        fn = camel_to_snake(info.name)
        lines.append(_generate_builder_class(info))
        lines.append("")
        lines.append(_generate_dsl_function(info))
        lines.append("")
        exports.extend([builder, fn])

    lines.append(f"__all__ = {exports!r}")
    lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# High-level API
# ---------------------------------------------------------------------------


def generate_for_source_dirs(
    source_dirs: List[Path],
    *,
    dry_run: bool = False,
    format_code: bool = True,
) -> Dict[Path, str]:
    """Scan source directories for ``@dsl_step`` classes and generate builders.

    Args:
        source_dirs: Directories to scan recursively for Python files.
        dry_run: If True, return generated code without writing files.
        format_code: If True, format output with ``black`` (if available).

    Returns:
        Mapping of output file paths to generated source code.
    """
    results: Dict[Path, str] = {}

    for source_dir in source_dirs:
        if not source_dir.is_dir():
            logger.warning("Skipping %s (not a directory)", source_dir)
            continue

        by_file: Dict[Path, List[StepClassInfo]] = {}
        for py_file in sorted(source_dir.rglob("*.py")):
            if py_file.name.startswith("_") or "_dsl.py" in py_file.name:
                continue
            for info in scan_file(py_file):
                by_file.setdefault(info.source_file, []).append(info)

        for source_file, classes in by_file.items():
            output_file = source_file.with_name(f"{source_file.stem}_dsl.py")
            source = generate_file(classes, source_file)

            if format_code and not dry_run:
                try:
                    import black

                    source = black.format_str(
                        source, mode=black.Mode(line_length=88)
                    )
                except ImportError:
                    pass

            if not dry_run:
                output_file.write_text(source, encoding="utf-8")
                logger.info(
                    "Generated %s (%d builder(s))",
                    output_file.name,
                    len(classes),
                )

            results[output_file] = source

    return results
