"""Tests for the typed, string-free mission-parameter framework (raccoon.step.params).

Covers the parts that carry logic and don't need a live UI/transport: the
runtime + persistent store, and the NumberParam descriptor (key derivation via
__set_name__, default fallback, persistence round-trip).  The
AskNumber UI step is intentionally not exercised here — it only wires
input_number() into param.set(), which requires a robot/transport.
"""

from __future__ import annotations

import importlib.util

import pytest


def libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not libstp_available(), reason="raccoon native module not installed"
)

pytestmark = requires_libstp


@pytest.fixture(autouse=True)
def _clean_runtime():
    """Every test starts with an empty runtime cache."""
    from raccoon.step.params import reset_params

    reset_params()
    yield
    reset_params()


# --- key derivation --------------------------------------------------------


def test_key_is_derived_from_attribute_name():
    from raccoon import NumberParam, ParamSet

    class P(ParamSet):
        cube_offset = NumberParam(default=0.0)

    assert P.cube_offset.key == "cube_offset"


def test_explicit_key_overrides_attribute_name():
    from raccoon import NumberParam, ParamSet

    param = NumberParam(default=0.0, key="explicit")

    class P(ParamSet):
        renamed = param  # __set_name__ must NOT clobber an explicit key

    assert P.renamed.key == "explicit"


def test_standalone_param_without_key_raises():
    from raccoon import NumberParam

    param = NumberParam(default=1.0)
    with pytest.raises(RuntimeError):
        _ = param.key


# --- get / set / default ---------------------------------------------------


def test_get_returns_default_when_unset():
    from raccoon import NumberParam, ParamSet

    class P(ParamSet):
        speed = NumberParam(default=0.5)

    assert P.speed.get() == 0.5
    assert P.speed.is_set() is False


def test_set_then_get_roundtrips():
    from raccoon import NumberParam, ParamSet

    class P(ParamSet):
        speed = NumberParam(default=0.5)

    P.speed.set(0.8)
    assert P.speed.get() == 0.8
    assert P.speed.is_set() is True


def test_set_stores_value_unclamped():
    from raccoon import NumberParam, ParamSet

    class P(ParamSet):
        offset = NumberParam(default=0.0)

    P.offset.set(999)
    assert P.offset.get() == 999.0
    P.offset.set(-999)
    assert P.offset.get() == -999.0


def test_reset_params_clears_runtime_value():
    from raccoon import NumberParam, ParamSet
    from raccoon.step.params import reset_params

    class P(ParamSet):
        speed = NumberParam(default=0.5)

    P.speed.set(0.9)
    reset_params()
    assert P.speed.get() == 0.5
    assert P.speed.is_set() is False


# --- ParamSet.all ----------------------------------------------------------


def test_paramset_all_lists_declared_params():
    from raccoon import NumberParam, ParamSet

    class P(ParamSet):
        a = NumberParam(default=1.0)
        b = NumberParam(default=2.0)
        not_a_param = 3

    assert set(P.all()) == {P.a, P.b}


# --- persistence (store level) --------------------------------------------


def test_store_persists_to_yaml_and_reads_back(tmp_path):
    from raccoon.step.calibration.store import CalibrationStore
    from raccoon.step.params import _ParamStore

    yml = tmp_path / "cal.yml"
    store_a = _ParamStore(cal_store=CalibrationStore(path=yml))
    store_a.set("offset", 3.5, persist=True)

    # A fresh store (fresh runtime cache) still recovers the value from disk.
    store_b = _ParamStore(cal_store=CalibrationStore(path=yml))
    assert store_b.get("offset", 0.0, persisted=True) == 3.5
    assert store_b.has("offset", persisted=True) is True


def test_store_runtime_only_does_not_touch_yaml(tmp_path):
    from raccoon.step.calibration.store import CalibrationStore
    from raccoon.step.params import _ParamStore

    yml = tmp_path / "cal.yml"
    store_a = _ParamStore(cal_store=CalibrationStore(path=yml))
    store_a.set("speed", 0.7, persist=False)

    store_b = _ParamStore(cal_store=CalibrationStore(path=yml))
    assert store_b.get("speed", 0.1, persisted=True) == 0.1  # falls back to default
    assert store_b.has("speed", persisted=True) is False


def test_non_persisted_get_ignores_existing_yaml(tmp_path):
    from raccoon.step.calibration.store import CalibrationStore
    from raccoon.step.params import _ParamStore

    yml = tmp_path / "cal.yml"
    # Value is on disk, but the param opts out of persistence...
    _ParamStore(cal_store=CalibrationStore(path=yml)).set("k", 9.0, persist=True)
    store = _ParamStore(cal_store=CalibrationStore(path=yml))
    # ...so a non-persisted read must not see it.
    assert store.get("k", 0.0, persisted=False) == 0.0


# --- profile scoping (game-table / side) -----------------------------------


def test_unscoped_param_ignores_active_profile():
    from raccoon import NumberParam, ParamSet
    from raccoon.step.params import set_active_profile

    class P(ParamSet):
        speed = NumberParam(default=0.5)  # not scoped

    set_active_profile("Table 1", "A", persist=False)
    P.speed.set(0.8)
    set_active_profile("Table 2", "B", persist=False)
    # A global param sees the same value regardless of the active profile.
    assert P.speed.get() == 0.8


def test_scoped_param_isolated_per_profile():
    from raccoon import NumberParam, ParamSet
    from raccoon.step.params import set_active_profile

    class P(ParamSet):
        offset = NumberParam(default=0.0, unit="cm", scoped=True)

    set_active_profile("Table 1", "A", persist=False)
    P.offset.set(3.0)

    # Same table, other side — independent value.
    set_active_profile("Table 1", "B", persist=False)
    assert P.offset.is_set() is False
    assert P.offset.get() == 0.0  # default
    P.offset.set(7.0)

    # Different table — independent again.
    set_active_profile("Table 2", "A", persist=False)
    assert P.offset.get() == 0.0

    # Back to the first profile — original value survives.
    set_active_profile("Table 1", "A", persist=False)
    assert P.offset.get() == 3.0
    assert P.offset.is_set() is True


def test_scoped_param_without_profile_falls_back_to_global():
    from raccoon import NumberParam, ParamSet

    class P(ParamSet):
        offset = NumberParam(default=1.0, scoped=True)

    # No profile active: behaves like a plain global param.
    P.offset.set(2.5)
    assert P.offset.get() == 2.5


def test_active_profile_persist_roundtrip(tmp_path, monkeypatch):
    import raccoon.step.params as params_mod
    from raccoon.step.calibration.store import CalibrationStore
    from raccoon.step.params import (
        _ParamStore,
        clear_active_profile,
        get_active_profile,
        load_persisted_profile,
        set_active_profile,
    )

    yml = tmp_path / "cal.yml"
    monkeypatch.setattr(params_mod, "_store", _ParamStore(cal_store=CalibrationStore(path=yml)))

    set_active_profile("Table 3", "B", persist=True)
    clear_active_profile()
    assert get_active_profile() is None

    # A fresh process (cleared runtime) restores the profile from disk.
    assert load_persisted_profile() == ("Table 3", "B")
    assert get_active_profile() == ("Table 3", "B")


def test_reset_params_clears_active_profile():
    from raccoon import NumberParam, ParamSet
    from raccoon.step.params import get_active_profile, reset_params, set_active_profile

    class P(ParamSet):
        offset = NumberParam(default=0.0, scoped=True)

    set_active_profile("Table 1", "A", persist=False)
    P.offset.set(4.0)
    reset_params()
    assert get_active_profile() is None
    assert P.offset.get() == 0.0


# --- profile-entry screen (keypad "1B") ------------------------------------
#
# The screen's refresh() is a no-op without a bound _step, so the real async
# keypad/click handlers can be driven directly to exercise the actual logic.


def _type(screen, *keys):
    import asyncio

    for k in keys:
        asyncio.run(screen.on_key(k))


def _make_screen():
    from raccoon.step.params import _ProfileEntryScreen

    return _ProfileEntryScreen()


def test_profile_entry_parses_number_and_side():
    import asyncio

    s = _make_screen()
    assert s._is_valid() is False  # nothing entered yet

    _type(s, "1")
    asyncio.run(s.on_side_b())
    assert s._is_valid() is True
    assert s._entry_label() == "1B"

    # Submit closes with the entered (table, side).
    asyncio.run(s.on_submit())
    assert s._closed is True
    assert s._result == ("1", "B")


def test_profile_entry_accepts_any_number():
    import asyncio

    # No range protection: any typed number is accepted verbatim.
    s = _make_screen()
    _type(s, "9", "9")
    asyncio.run(s.on_side_a())
    assert s._is_valid() is True
    asyncio.run(s.on_submit())
    assert s._result == ("99", "A")


def test_profile_entry_requires_number_and_side():
    import asyncio

    s = _make_screen()
    _type(s, "2")
    assert s._is_valid() is False  # side still missing
    assert s._entry_label() == "2_"
    # A premature submit must not close on a partial entry.
    asyncio.run(s.on_submit())
    assert s._closed is False


def test_profile_entry_backspace_and_cancel():
    import asyncio

    s = _make_screen()
    _type(s, "1", "back")
    assert s._number_str == ""
    assert s._is_valid() is False

    asyncio.run(s.on_cancel())
    assert s._closed is True
    assert s._result is None
