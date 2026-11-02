"""The import-time transport contract check (run against a transport that has it)."""

import importlib.util
import json
from pathlib import Path

import pytest

pytest.importorskip("raccoon_transport.contract_check")

GATE = Path(__file__).parents[1] / "python" / "raccoon" / "_contract_gate.py"


def load_gate():
    spec = importlib.util.spec_from_file_location("raccoon_contract_gate_under_test", GATE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def device(tmp_path, monkeypatch):
    (tmp_path / "image").mkdir()
    monkeypatch.setenv("RACCOON_DEVICE_MARKER", str(tmp_path / "image"))
    monkeypatch.setenv("RACCOON_BUNDLE_FILE", str(tmp_path / "bundle.json"))
    monkeypatch.setenv("RACCOON_STATUS_DIR", str(tmp_path / "run"))
    from raccoon_transport import CONTRACT_HASH

    def write(contract_hash, lib_version="155"):
        (tmp_path / "bundle.json").write_text(json.dumps(
            {"bundle": "b", "contract_hash": contract_hash, "components": {"raccoon-lib": lib_version}}))

    write.hash = CONTRACT_HASH
    write.status = tmp_path / "run" / "raccoon-lib.json"
    return write


def test_matching_contract_imports_and_records_status(device):
    device(device.hash)
    result = load_gate().check_transport_contract("155")
    assert result.result == "ok"
    assert json.loads(device.status.read_text())["persistent"] is False


def test_mismatch_raises_a_clear_error(device):
    device("1111111111111111")
    with pytest.raises(RuntimeError) as raised:
        load_gate().check_transport_contract("155")
    assert "transport contract mismatch" in str(raised.value)
    assert json.loads(device.status.read_text())["result"] == "mismatch"


def test_wrong_library_version_raises(device):
    device(device.hash, lib_version="v156")
    with pytest.raises(RuntimeError) as raised:
        load_gate().check_transport_contract("155")
    assert "component version mismatch" in str(raised.value)


def test_v_prefix_does_not_matter(device):
    device(device.hash, lib_version="v155")
    assert load_gate().check_transport_contract("155").result == "ok"


def test_dev_entry_omits_the_version_check(device):
    device(device.hash, lib_version="dev")
    result = load_gate().check_transport_contract("999")
    assert result.result == "dev-warnung" and result.reason == "version-omitted"


def test_missing_bundle_file_raises_on_a_device(device, tmp_path):
    (tmp_path / "bundle.json").unlink(missing_ok=True)
    with pytest.raises(RuntimeError) as raised:
        load_gate().check_transport_contract("155")
    assert "bundle file is missing" in str(raised.value)


def test_dev_mode_does_not_raise(device):
    device("dev")
    assert load_gate().check_transport_contract("155").result == "dev-warnung"


def test_laptop_is_skipped(tmp_path, monkeypatch):
    monkeypatch.setenv("RACCOON_DEVICE_MARKER", str(tmp_path / "nope"))
    monkeypatch.setenv("RACCOON_STATUS_DIR", str(tmp_path / "run"))
    assert load_gate().check_transport_contract("155").result == "skipped"
    assert not (tmp_path / "run").exists()


def test_transport_without_the_check_is_ignored(monkeypatch):
    import sys

    import raccoon_transport

    monkeypatch.setitem(sys.modules, "raccoon_transport.contract_check", None)
    monkeypatch.delattr(raccoon_transport, "contract_check", raising=False)
    assert load_gate().check_transport_contract("155") is None


# -- the library's own version check (specified in raccoon-transport contract/COMPONENT_VERSION_CHECK.md) --

CASES = [
    ('"v58"', "ok", None), ('"58"', "ok", None), ('" v58 "', "ok", None), ('"V58"', "ok", None),
    ('"v59"', "version-mismatch", None), ('"v058"', "version-mismatch", None),
    ('"dev"', "dev-warnung", "version-omitted"),
    ('""', "bundle-fehler", "version-field-empty"), ('"  "', "bundle-fehler", "version-field-empty"),
    ("58", "bundle-fehler", "version-field-missing"),
]


@pytest.mark.parametrize("entry,result,reason", CASES)
def test_version_check_cases(tmp_path, entry, result, reason):
    bundle = tmp_path / "b.json"
    bundle.write_text('{"bundle":"b","contract_hash":"x","components":{"raccoon-lib":%s}}' % entry)
    check = load_gate().version_check("v58", str(bundle))
    assert (check.result, check.reason) == (result, reason)


def test_version_check_missing_entry_and_components(tmp_path):
    bundle = tmp_path / "b.json"
    for text in ('{"components":{"raccoon-cli":"v1"}}', '{"contract_hash":"x"}'):
        bundle.write_text(text)
        check = load_gate().version_check("58", str(bundle))
        assert (check.result, check.reason) == ("bundle-fehler", "version-field-missing")
        assert "no version for raccoon-lib" in check.message


def test_version_check_message_names_both_sides(tmp_path):
    bundle = tmp_path / "b.json"
    bundle.write_text('{"bundle":"2026.11.02","components":{"raccoon-lib":"v59"}}')
    check = load_gate().version_check("v58", str(bundle))
    assert check.message == ("component version mismatch: raccoon-lib is v58 but bundle 2026.11.02 expects v59; "
                             "update or reflash so all components match")


def test_version_check_without_a_readable_bundle_returns_none(tmp_path):
    assert load_gate().version_check("58", str(tmp_path / "missing.json")) is None
    (tmp_path / "bad.json").write_text("{nope")
    assert load_gate().version_check("58", str(tmp_path / "bad.json")) is None
