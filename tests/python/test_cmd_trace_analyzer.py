"""Tests for scripts/analyze_cmd_trace.py — the command-ordering analyzer.

These lock in the core correlation + inversion logic so the diagnostic stays
trustworthy: a false "no problem" here would hide the very servo-reordering bug
the tool exists to catch.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

_SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "analyze_cmd_trace.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("analyze_cmd_trace", _SCRIPT)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    # Register before exec so the module's dataclasses can resolve their
    # ``from __future__ import annotations`` string hints via sys.modules.
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


mod = _load_module()


def _write(path: Path, rows: list[dict]) -> Path:
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    return path


def _send_row(seq, ts_us, ch, kind, port, v):
    return {"t_ns": seq, "seq": seq, "ts_us": ts_us, "ch": ch, "kind": kind,
            "port": port, "v": [v]}


def _recv_row(rseq, w_us, ch, kind, port, v, ts_us, stage="recv"):
    return {"t_ns": rseq, "w_us": w_us, "rseq": rseq, "stage": stage, "kind": kind,
            "ch": ch, "port": port, "v": v, "ts_us": ts_us}


def test_correlate_matches_dropped_and_orphans(tmp_path: Path) -> None:
    send = _write(tmp_path / "s.jsonl", [
        _send_row(0, 1000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0),
        _send_row(1, 1001, "raccoon/servo/1/position_cmd", "servo_pos", 1, 90.0),
    ])
    recv = _write(tmp_path / "r.jsonl", [
        _recv_row(0, 2000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0, 1000),
        # port 1 never received -> dropped; an orphan recv with no matching send:
        _recv_row(1, 2001, "raccoon/servo/3/position_cmd", "servo_pos", 3, 5.0, 9999),
    ])
    a = mod.correlate(mod.load_send(send), mod.load_recv(recv)[0])
    assert len(a.matched) == 1
    assert [s.port for s in a.dropped] == [1]
    assert [r.port for r in a.orphan_recv] == [3]


def test_detects_cross_servo_inversion(tmp_path: Path) -> None:
    send = _write(tmp_path / "s.jsonl", [
        _send_row(0, 1000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0),
        _send_row(1, 1001, "raccoon/servo/1/position_cmd", "servo_pos", 1, 90.0),
    ])
    # port 1 (sent second) arrives first -> inversion.
    recv = _write(tmp_path / "r.jsonl", [
        _recv_row(0, 2000, "raccoon/servo/1/position_cmd", "servo_pos", 1, 90.0, 1001),
        _recv_row(1, 2001, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0, 1000),
    ])
    a = mod.correlate(mod.load_send(send), mod.load_recv(recv)[0])
    inv = mod.find_inversions(a.matched)
    assert len(inv) == 1
    prev, cur = inv[0]
    # cur was sent before prev but received after it
    assert cur.send.seq < prev.send.seq


def test_in_order_delivery_has_no_inversions(tmp_path: Path) -> None:
    send = _write(tmp_path / "s.jsonl", [
        _send_row(0, 1000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0),
        _send_row(1, 1001, "raccoon/servo/1/position_cmd", "servo_pos", 1, 90.0),
    ])
    recv = _write(tmp_path / "r.jsonl", [
        _recv_row(0, 2000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0, 1000),
        _recv_row(1, 2001, "raccoon/servo/1/position_cmd", "servo_pos", 1, 90.0, 1001),
    ])
    a = mod.correlate(mod.load_send(send), mod.load_recv(recv)[0])
    assert mod.find_inversions(a.matched) == []


def test_spi_stage_is_separated(tmp_path: Path) -> None:
    recv = _write(tmp_path / "r.jsonl", [
        _recv_row(0, 2000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0, 1000),
        _recv_row(1, 2001, "", "servo_pos", 0, 30.0, 0, stage="spi"),
    ])
    recv_recs, spi_recs = mod.load_recv(recv)
    assert len(recv_recs) == 1
    assert len(spi_recs) == 1
    assert spi_recs[0].stage == "spi"


def test_latency_is_recv_minus_send(tmp_path: Path) -> None:
    send = _write(tmp_path / "s.jsonl", [
        _send_row(0, 1000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0),
    ])
    recv = _write(tmp_path / "r.jsonl", [
        _recv_row(0, 1500, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0, 1000),
    ])
    a = mod.correlate(mod.load_send(send), mod.load_recv(recv)[0])
    assert a.matched[0].latency_us == 500


def test_main_writes_trace_and_flags_inversion(tmp_path: Path) -> None:
    send = _write(tmp_path / "s.jsonl", [
        _send_row(0, 1000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0),
        _send_row(1, 1001, "raccoon/servo/1/position_cmd", "servo_pos", 1, 90.0),
    ])
    recv = _write(tmp_path / "r.jsonl", [
        _recv_row(0, 2000, "raccoon/servo/1/position_cmd", "servo_pos", 1, 90.0, 1001),
        _recv_row(1, 2001, "raccoon/servo/0/position_cmd", "servo_pos", 0, 30.0, 1000),
    ])
    trace = tmp_path / "t.json"
    rc = mod.main([
        "--send", str(send), "--recv", str(recv),
        "--trace", str(trace), "--fail-on-inversion",
    ])
    assert rc == 1  # cross-servo inversion present
    data = json.loads(trace.read_text())
    assert data["traceEvents"]
    assert any(e.get("ph") == "X" for e in data["traceEvents"])


def test_truncated_trailing_line_is_tolerated(tmp_path: Path, capsys) -> None:
    p = tmp_path / "s.jsonl"
    p.write_text(
        json.dumps(_send_row(0, 1000, "raccoon/servo/0/position_cmd", "servo_pos", 0, 1.0))
        + "\n{ this is a truncated cra"
    )
    rows = mod.load_send(p)
    assert len(rows) == 1  # bad final line skipped, good one kept
