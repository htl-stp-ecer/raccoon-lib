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

    def write(contract_hash):
        (tmp_path / "bundle.json").write_text(json.dumps({"bundle": "b", "contract_hash": contract_hash}))

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
