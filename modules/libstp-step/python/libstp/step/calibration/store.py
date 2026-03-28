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

from pathlib import Path

import yaml

CALIBRATION_FILE = "racoon.calibration.yml"


class CalibrationStore:
    """Read and write named calibration sections in the calibration YAML."""

    def __init__(self, path: Path | None = None) -> None:
        self._path = path or Path(CALIBRATION_FILE)

    def _read_root(self) -> dict:
        if not self._path.exists():
            return {}
        with open(self._path) as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            return {}
        return data.get("root", {})

    def _write_root(self, root: dict) -> None:
        with open(self._path, "w") as f:
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
