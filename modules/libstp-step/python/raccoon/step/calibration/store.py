"""YAML-backed calibration store.

Persists arbitrary calibration data to ``racoon.calibration.yml`` alongside
the C++ CalibrationStore (which owns the ``ir-calibration`` section).

Each section/set pair maps to a flat dict of values::

    root:
      range-finder:
        first_pipe:
          t_enter: 1200.0
          t_exit: 800.0
      drum-collector:
        default:
          blocked: 3200.0
          pocket: 900.0
"""

from __future__ import annotations

import warnings
from pathlib import Path

import yaml

from raccoon.log import warn as log_warn

CALIBRATION_FILE = "raccoon.calibration.yml"
#: The name that projects used while the file name had a typo. It is still read (and written, when it is the
#: only one that exists) and gives a deprecation warning.
LEGACY_CALIBRATION_FILE = "racoon.calibration.yml"

_warned_legacy = False


def resolve_calibration_path(directory: Path | None = None) -> Path:
    """The calibration file of a project: ``raccoon.calibration.yml``, or the legacy ``racoon.calibration.yml``.

    The new name wins when both exist. The legacy file is used when it is the only one, with a deprecation
    warning (once per process). With neither, the new name is used.
    """
    global _warned_legacy
    base = directory if directory is not None else Path()
    current, legacy = base / CALIBRATION_FILE, base / LEGACY_CALIBRATION_FILE
    if current.exists() or not legacy.exists():
        return current
    if not _warned_legacy:
        _warned_legacy = True
        message = (
            f"{LEGACY_CALIBRATION_FILE} is deprecated: rename it to {CALIBRATION_FILE} "
            "(`raccoon migrate` does that and updates .gitignore)"
        )
        warnings.warn(message, DeprecationWarning, stacklevel=3)
        log_warn(message)
    return legacy


class CalibrationStore:
    """Read and write named calibration sections in the calibration YAML."""

    def __init__(self, path: Path | None = None) -> None:
        self._explicit_path = path

    @property
    def _path(self) -> Path:
        # Looked up on every use: the working directory is the project, and the file may appear or be renamed.
        return self._explicit_path or resolve_calibration_path()

    def _read_root(self) -> dict:
        if not self._path.exists():
            return {}
        with self._path.open() as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            return {}
        return data.get("root", {})

    def _write_root(self, root: dict) -> None:
        with self._path.open("w") as f:
            yaml.safe_dump({"root": root}, f, default_flow_style=False)

    def load(self, section: str, set_name: str = "default") -> dict | None:
        """Load a calibration dict, or *None* if nothing is stored."""
        section_data = self._read_root().get(section, {})
        entry = section_data.get(set_name)
        return dict(entry) if isinstance(entry, dict) else None

    def store(self, section: str, data: dict, set_name: str = "default") -> None:
        """Persist a calibration dict (merges with existing file)."""
        root = self._read_root()
        root.setdefault(section, {})[set_name] = data
        self._write_root(root)

    def has_data(self, section: str, set_name: str = "default") -> bool:
        """Check whether calibration data exists for the given section/set."""
        return self.load(section, set_name) is not None
