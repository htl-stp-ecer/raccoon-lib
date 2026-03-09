"""
Include-aware YAML reader/writer.

Handles ``!include`` and ``!include-merge`` tags at **any nesting depth** so
that callers can read and write values by property path without caring whether
the config is one file or split across dozens.

The resolution is fully recursive: an included file may itself contain further
``!include`` / ``!include-merge`` references.

Core API (works with any YAML file)::

    from pathlib import Path
    from libstp.project_yaml import yaml_read, yaml_write

    # Read a deeply nested value, following includes automatically
    val = yaml_read(Path("config.yml"), ["robot", "motion_pid", "angular", "max_velocity"])

    # Write — the value lands in whichever split file owns that path
    yaml_write(Path("config.yml"), ["robot", "motion_pid", "angular", "max_velocity"], 1.23)

Convenience wrappers for ``raccoon.project.yml``::

    from libstp.project_yaml import find_project_root, read_project_value, update_project_value

    root = find_project_root()
    update_project_value(root, ["robot", "motion_pid", "angular", "max_velocity"], 1.23)
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import yaml

_PROJECT_FILENAME = "raccoon.project.yml"

_SENTINEL = object()


# ---------------------------------------------------------------------------
# Include-aware YAML loader
# ---------------------------------------------------------------------------

class _IncludeRef:
    """Marker object that records an ``!include`` or ``!include-merge`` tag."""
    __slots__ = ("path", "merge")

    def __init__(self, path: str, *, merge: bool = False) -> None:
        self.path = path
        self.merge = merge

    def __repr__(self) -> str:
        tag = "!include-merge" if self.merge else "!include"
        return f"{tag} {self.path}"


def _make_include_loader():
    """Create a YAML loader subclass that turns include tags into markers."""
    class IncludeLoader(yaml.SafeLoader):
        pass

    def _include(loader, node):
        return _IncludeRef(loader.construct_scalar(node), merge=False)

    def _include_merge(loader, node):
        return _IncludeRef(loader.construct_scalar(node), merge=True)

    IncludeLoader.add_constructor("!include", _include)
    IncludeLoader.add_constructor("!include-merge", _include_merge)
    return IncludeLoader


_IncludeLoader = _make_include_loader()


def _load_with_includes(path: Path) -> Any:
    """Load a YAML file, preserving ``!include`` / ``!include-merge`` as markers."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.load(f, Loader=_IncludeLoader)  # noqa: S506
    except (yaml.YAMLError, OSError):
        return {}


def _strip_refs(data: Any) -> Any:
    """Recursively remove ``_IncludeRef`` values from a loaded YAML tree."""
    if isinstance(data, dict):
        return {k: _strip_refs(v) for k, v in data.items()
                if not isinstance(v, _IncludeRef)}
    if isinstance(data, list):
        return [_strip_refs(v) for v in data if not isinstance(v, _IncludeRef)]
    return data


def _load_plain(path: Path) -> dict:
    """Load a YAML file, stripping include refs, returning ``{}`` on failure.

    Uses the include-aware loader so files containing ``!include`` tags don't
    cause parse errors.  ``_IncludeRef`` values are dropped from the result.
    """
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.load(f, Loader=_IncludeLoader)  # noqa: S506
        if not isinstance(data, dict):
            return {}
        return _strip_refs(data)
    except (yaml.YAMLError, OSError):
        return {}


def _make_include_dumper():
    """Create a YAML dumper that serializes ``_IncludeRef`` back to tags."""
    class IncludeDumper(yaml.SafeDumper):
        pass

    def _represent_ref(dumper, ref):
        tag = "!include-merge" if ref.merge else "!include"
        return dumper.represent_scalar(tag, ref.path)

    IncludeDumper.add_representer(_IncludeRef, _represent_ref)
    return IncludeDumper


_IncludeDumper = _make_include_dumper()


def _save_yaml(path: Path, data: dict) -> bool:
    """Write *data* to *path*, preserving existing ``!include`` directives.

    Loads the original file (with include markers intact), merges the updated
    plain *data* back in, then serializes with a dumper that can write
    ``_IncludeRef`` objects as proper YAML ``!include`` / ``!include-merge``
    tags.
    """
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        # Load original to preserve _IncludeRef entries
        original = _load_with_includes(path)
        if isinstance(original, dict):
            # Re-insert include refs that were stripped during _load_plain
            merged = dict(original)
            merged.update(data)
        else:
            merged = data
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(merged, f, Dumper=_IncludeDumper,
                      sort_keys=False, default_flow_style=False)
        return True
    except OSError:
        return False


# ---------------------------------------------------------------------------
# Recursive path resolution
# ---------------------------------------------------------------------------

def _resolve_path(
    file_path: Path,
    key_path: list[str],
    *,
    _depth: int = 0,
) -> tuple[Path, list[str]]:
    """Walk *key_path* segment-by-segment, following includes at each level.

    Returns ``(owning_file, remaining_key_path)`` where *owning_file* is the
    YAML file that contains the deepest reachable ancestor of the target, and
    *remaining_key_path* is the portion of the path local to that file.

    The resolution is recursive (up to 20 levels deep to prevent loops).
    """
    if _depth > 20:
        return file_path, key_path  # safety valve

    if not key_path:
        return file_path, key_path

    data = _load_with_includes(file_path)
    if not isinstance(data, dict):
        return file_path, key_path

    remaining = list(key_path)
    consumed: list[str] = []

    while remaining:
        segment = remaining[0]

        # --- Direct key match ---
        if segment in data:
            val = data[segment]

            if isinstance(val, _IncludeRef) and not val.merge:
                # !include — the value lives entirely in the referenced file.
                include_path = (file_path.parent / val.path).resolve()
                remaining.pop(0)
                # Recurse into the included file with the rest of the path.
                return _resolve_path(include_path, remaining, _depth=_depth + 1)

            if isinstance(val, _IncludeRef) and val.merge:
                # Key matched but its value is !include-merge — the merged
                # keys live in the other file.  This shouldn't normally happen
                # (merge keys are typically prefixed with _), but handle it.
                include_path = (file_path.parent / val.path).resolve()
                remaining.pop(0)
                return _resolve_path(include_path, remaining, _depth=_depth + 1)

            # Regular value — this segment is local to this file.
            consumed.append(remaining.pop(0))
            if isinstance(val, dict):
                data = val
                continue
            else:
                # Leaf value — remaining path (if any) can't be navigated further.
                break

        # --- Check !include-merge sources ---
        # The segment might be a key that was merged from another file.
        found_in_merge = False
        for key, val in data.items():
            if not isinstance(val, _IncludeRef) or not val.merge:
                continue
            include_path = (file_path.parent / val.path).resolve()
            merge_data = _load_with_includes(include_path)
            if isinstance(merge_data, dict) and segment in merge_data:
                # The key lives in this merge file. Recurse into it.
                return _resolve_path(include_path, remaining, _depth=_depth + 1)

        # Segment not found anywhere — it might be a new key to create in
        # the current file.
        break

    return file_path, consumed + remaining


# ---------------------------------------------------------------------------
# Core public API (file-based, no project assumptions)
# ---------------------------------------------------------------------------

def yaml_read(
    file_path: Path,
    key_path: list[str],
    default: Any = _SENTINEL,
) -> Any:
    """Read a value from a YAML file, recursively resolving ``!include`` refs.

    Args:
        file_path: Path to the root YAML file.
        key_path: Property path segments, e.g. ``["robot", "motion_pid"]``.
        default: Value to return when the path doesn't exist.
            Raises ``KeyError`` if omitted and the path is missing.

    Returns:
        The resolved value, or *default*.
    """
    target_file, remaining = _resolve_path(file_path, key_path)
    data = _load_plain(target_file)

    for segment in remaining:
        if not isinstance(data, dict) or segment not in data:
            if default is _SENTINEL:
                raise KeyError(f"Path not found: {'.'.join(key_path)}")
            return default
        data = data[segment]
    return data


def yaml_write(
    file_path: Path,
    key_path: list[str],
    value: Any,
) -> bool:
    """Write a value to a YAML file, following ``!include`` refs to the correct file.

    Parent dicts are created as needed.  Only the resolved target file is
    rewritten — all other files (and their ``!include`` directives) are
    untouched.

    Args:
        file_path: Path to the root YAML file.
        key_path: Property path, e.g. ``["robot", "motion_pid", "angular", "max_velocity"]``.
        value: The value to set.

    Returns:
        ``True`` on success.
    """
    if not key_path:
        return False

    target_file, remaining = _resolve_path(file_path, key_path)

    if not remaining:
        if not isinstance(value, dict):
            return False
        return _save_yaml(target_file, value)

    data = _load_plain(target_file)

    cursor = data
    for segment in remaining[:-1]:
        if segment not in cursor or not isinstance(cursor[segment], dict):
            cursor[segment] = {}
        cursor = cursor[segment]

    cursor[remaining[-1]] = value
    return _save_yaml(target_file, data)


def yaml_write_many(
    file_path: Path,
    updates: dict[tuple[str, ...], Any],
) -> bool:
    """Batch-write multiple values, minimizing file I/O.

    Args:
        file_path: Path to the root YAML file.
        updates: Mapping from key-path tuples to values.

    Returns:
        ``True`` if all writes succeeded.
    """
    grouped: dict[Path, list[tuple[list[str], Any]]] = {}
    for key_tuple, value in updates.items():
        target, remaining = _resolve_path(file_path, list(key_tuple))
        grouped.setdefault(target, []).append((remaining, value))

    all_ok = True
    for target, ops in grouped.items():
        data = _load_plain(target)
        for remaining, value in ops:
            if not remaining:
                continue
            cursor = data
            for segment in remaining[:-1]:
                if segment not in cursor or not isinstance(cursor[segment], dict):
                    cursor[segment] = {}
                cursor = cursor[segment]
            cursor[remaining[-1]] = value
        if not _save_yaml(target, data):
            all_ok = False
    return all_ok


# ---------------------------------------------------------------------------
# Project convenience wrappers (raccoon.project.yml)
# ---------------------------------------------------------------------------

def find_project_root(start: Path | None = None) -> Optional[Path]:
    """Search upward from *start* (default: cwd) for ``raccoon.project.yml``."""
    try:
        current = (start or Path.cwd()).resolve()
    except (FileNotFoundError, OSError):
        return None
    while current != current.parent:
        if (current / _PROJECT_FILENAME).exists():
            return current
        current = current.parent
    return None


def read_project_value(
    project_root: Path,
    key_path: list[str],
    default: Any = None,
) -> Any:
    """Read from ``raccoon.project.yml``, following includes. See :func:`yaml_read`."""
    return yaml_read(project_root / _PROJECT_FILENAME, key_path, default)


def update_project_value(
    project_root: Path,
    key_path: list[str],
    value: Any,
) -> bool:
    """Write to ``raccoon.project.yml``, following includes. See :func:`yaml_write`."""
    return yaml_write(project_root / _PROJECT_FILENAME, key_path, value)


def update_project_values(
    project_root: Path,
    updates: dict[tuple[str, ...], Any],
) -> bool:
    """Batch-write to ``raccoon.project.yml``. See :func:`yaml_write_many`."""
    return yaml_write_many(project_root / _PROJECT_FILENAME, updates)
