"""The calibration file is ``raccoon.calibration.yml``; the old name ``racoon.calibration.yml`` is still read."""

from __future__ import annotations

import importlib.util
import sys
import types
import warnings
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
_STORE_PATH = REPO_ROOT / "modules/libstp-step/python/raccoon/step/calibration/store.py"


@pytest.fixture
def store_module(monkeypatch):
    """store.py on its own, with a recording stand-in for the lib's logger."""
    logged: list[str] = []
    log = types.ModuleType("raccoon.log")
    log.warn = lambda message, **_: logged.append(message)
    package = sys.modules.get("raccoon") or types.ModuleType("raccoon")
    monkeypatch.setitem(sys.modules, "raccoon", package)
    monkeypatch.setitem(sys.modules, "raccoon.log", log)
    spec = importlib.util.spec_from_file_location("calibration_store_under_test", _STORE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.logged = logged
    return module


def test_a_new_project_uses_the_new_name(store_module, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    store = store_module.CalibrationStore()
    store.store("ir-calibration", {"white_tresh": 1.0}, "default")
    assert (tmp_path / "raccoon.calibration.yml").is_file() and not (tmp_path / "racoon.calibration.yml").exists()
    assert store.load("ir-calibration") == {"white_tresh": 1.0}
    assert store_module.logged == []


def test_the_old_name_is_read_and_written_with_a_deprecation_warning(store_module, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "racoon.calibration.yml").write_text("root:\n  ir-calibration:\n    default:\n      white_tresh: 2.0\n")
    store = store_module.CalibrationStore()
    with pytest.warns(DeprecationWarning, match="racoon.calibration.yml is deprecated"):
        assert store.load("ir-calibration") == {"white_tresh": 2.0}
    store.store("ir-calibration", {"white_tresh": 3.0}, "other")  # goes to the file that exists
    assert not (tmp_path / "raccoon.calibration.yml").exists()
    assert store.load("ir-calibration", "other") == {"white_tresh": 3.0}
    assert len(store_module.logged) == 1  # once per process, also in the log


def test_the_new_name_wins_when_both_exist(store_module, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "racoon.calibration.yml").write_text("root:\n  x:\n    default: {v: 1}\n")
    (tmp_path / "raccoon.calibration.yml").write_text("root:\n  x:\n    default: {v: 2}\n")
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert store_module.CalibrationStore().load("x") == {"v": 2}
    assert store_module.logged == []


def test_an_explicit_path_is_used_as_it_is(store_module, tmp_path):
    path = tmp_path / "mine.yml"
    store = store_module.CalibrationStore(path)
    store.store("x", {"v": 1})
    assert path.is_file()
