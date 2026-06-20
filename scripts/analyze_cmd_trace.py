#!/usr/bin/env python3
"""Correlate send-side and receive-side command traces and flag reordering.

Two opt-in tracers feed this tool:

* **send side** — raccoon-lib's ``CommandTrace`` (enable with
  ``RACCOON_CMD_TRACE=/path/send.jsonl``) logs every motor/servo/chassis command
  as it is published, with a process-global monotonic ``seq`` (the *intended*
  order across all channels) and the wire ``ts_us`` system-clock timestamp.
* **receive side** — stm32-data-reader's ``CmdTrace`` (enable with
  ``WOMBAT_CMD_TRACE=/path/recv.jsonl``) logs each command when its handler fires
  (stage ``recv``) and when a servo position is staged to the SPI buffer (stage
  ``spi``), with its own monotonic ``rseq`` and a system-clock ``w_us``.

Both sides run on the same Pi, so ``ts_us`` (send) and the echoed ``ts_us``
(recv) share a clock and form the correlation key ``(channel, ts_us)``. This
script matches the two streams and reports:

* dropped commands (sent but never received — a transport overrun),
* send -> recv latency distribution,
* **ordering inversions**: commands applied in a different order than they were
  sent — the "servo that should move first moves second" bug. Cross-port servo
  inversions are called out specifically.

With ``--trace out.json`` it also writes a ``chrome://tracing`` file where send
and recv sit on one shared (system-clock) timeline, so crossing bars show a
reordering at a glance — open it next to the StepProfiler trace.

Usage::

    python3 scripts/analyze_cmd_trace.py --send send.jsonl --recv recv.jsonl \
        --trace cmd-trace.json
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class SendRec:
    seq: int
    ts_us: int
    ch: str
    kind: str
    port: int
    v: list[float]
    t_ns: int


@dataclass
class RecvRec:
    rseq: int
    w_us: int
    ts_us: int
    ch: str
    kind: str
    port: int
    v: float
    stage: str
    t_ns: int


def _load_jsonl(path: Path) -> list[dict]:
    records: list[dict] = []
    with path.open() as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                # A crash can truncate the final line; skip it loudly but keep going.
                sys.stderr.write(f"warning: {path}:{lineno}: bad JSON skipped ({exc})\n")
    return records


def load_send(path: Path) -> list[SendRec]:
    out: list[SendRec] = []
    for r in _load_jsonl(path):
        out.append(
            SendRec(
                seq=int(r["seq"]),
                ts_us=int(r["ts_us"]),
                ch=str(r["ch"]),
                kind=str(r["kind"]),
                port=int(r["port"]),
                v=[float(x) for x in r.get("v", [])],
                t_ns=int(r.get("t_ns", 0)),
            )
        )
    out.sort(key=lambda s: s.seq)
    return out


def load_recv(path: Path) -> tuple[list[RecvRec], list[RecvRec]]:
    recv: list[RecvRec] = []
    spi: list[RecvRec] = []
    for r in _load_jsonl(path):
        rec = RecvRec(
            rseq=int(r["rseq"]),
            w_us=int(r.get("w_us", 0)),
            ts_us=int(r.get("ts_us", 0)),
            ch=str(r.get("ch", "")),
            kind=str(r["kind"]),
            port=int(r["port"]),
            v=float(r.get("v", 0.0)),
            stage=str(r["stage"]),
            t_ns=int(r.get("t_ns", 0)),
        )
        (spi if rec.stage == "spi" else recv).append(rec)
    recv.sort(key=lambda s: s.rseq)
    spi.sort(key=lambda s: s.rseq)
    return recv, spi


@dataclass
class Matched:
    send: SendRec
    recv: RecvRec

    @property
    def latency_us(self) -> int:
        return self.recv.w_us - self.send.ts_us


@dataclass
class Analysis:
    matched: list[Matched] = field(default_factory=list)
    dropped: list[SendRec] = field(default_factory=list)  # sent, never received
    orphan_recv: list[RecvRec] = field(default_factory=list)  # received, no send seen


def correlate(send: list[SendRec], recv: list[RecvRec]) -> Analysis:
    """Match send<->recv on (channel, ts_us). Duplicate keys match in order."""
    buckets: dict[tuple[str, int], list[RecvRec]] = defaultdict(list)
    for r in recv:
        buckets[(r.ch, r.ts_us)].append(r)
    # keep each bucket time-ordered so duplicates pair deterministically
    for v in buckets.values():
        v.sort(key=lambda r: r.rseq)

    a = Analysis()
    used: set[int] = set()
    for s in send:
        bucket = buckets.get((s.ch, s.ts_us), [])
        chosen = next((r for r in bucket if id(r) not in used), None)
        if chosen is None:
            a.dropped.append(s)
        else:
            used.add(id(chosen))
            a.matched.append(Matched(s, chosen))
    for r in recv:
        if id(r) not in used:
            a.orphan_recv.append(r)
    return a


def find_inversions(matched: list[Matched]) -> list[tuple[Matched, Matched]]:
    """Pairs (A, B) where A was sent before B but received/applied after B.

    Compared on the *received* order (``rseq``) against the *sent* order
    (``seq``). Only adjacent inversions in received order are reported so the
    list stays the size of the problem, not its square.
    """
    by_recv = sorted(matched, key=lambda m: m.recv.rseq)
    inversions: list[tuple[Matched, Matched]] = []
    for prev, cur in zip(by_recv, by_recv[1:]):
        # cur arrived right after prev; if cur was *sent* before prev, the bus
        # delivered them in the wrong order.
        if cur.send.seq < prev.send.seq:
            inversions.append((prev, cur))
    return inversions


def _pct(values: list[int], q: float) -> int:
    if not values:
        return 0
    s = sorted(values)
    return s[min(int(q * len(s)), len(s) - 1)]


def format_report(send: list[SendRec], recv: list[RecvRec], spi: list[RecvRec],
                  a: Analysis) -> str:
    lines: list[str] = []
    w = lines.append
    w("=" * 78)
    w("  COMMAND TRACE ANALYSIS (send -> receive ordering)")
    w("=" * 78)
    w(f"  commands sent               : {len(send)}")
    w(f"  commands received (recv)    : {len(recv)}")
    w(f"  servo SPI stagings (spi)    : {len(spi)}")
    w(f"  matched send<->recv         : {len(a.matched)}")
    w(f"  DROPPED (sent, never recvd) : {len(a.dropped)}")
    w(f"  orphan recv (no send seen)  : {len(a.orphan_recv)}")

    lat = [m.latency_us for m in a.matched if m.latency_us >= 0]
    if lat:
        w("")
        w("  SEND -> RECV LATENCY (us)")
        w("  " + "-" * 74)
        w(f"  min / p50 / p99 / max       : {min(lat)} / {_pct(lat, 0.5)} / "
          f"{_pct(lat, 0.99)} / {max(lat)}")

    inversions = find_inversions(a.matched)
    cross_port = [
        (p, c) for (p, c) in inversions
        if p.send.ch != c.send.ch or p.send.port != c.send.port
    ]
    servo_cross = [
        (p, c) for (p, c) in cross_port
        if p.send.kind.startswith("servo") and c.send.kind.startswith("servo")
    ]
    w("")
    w("  ORDERING INVERSIONS (received in a different order than sent)")
    w("  " + "-" * 74)
    w(f"  total inversions            : {len(inversions)}")
    w(f"  cross-channel/port          : {len(cross_port)}")
    w(f"  cross-SERVO (the bug)       : {len(servo_cross)}")
    if servo_cross:
        w("")
        w("  servo commands applied OUT OF ORDER (sent A before B, B ran first):")
        for prev, cur in servo_cross[:20]:
            a_s, b_s = cur.send, prev.send  # cur was sent first but ran second
            w(f"      sent#{a_s.seq} {a_s.kind} port{a_s.port}={a_s.v} (ts={a_s.ts_us})"
              f"  ran AFTER  sent#{b_s.seq} {b_s.kind} port{b_s.port}={b_s.v}")
    elif inversions:
        w("")
        w("  (inversions exist but none are cross-servo; sample:)")
        for prev, cur in inversions[:10]:
            w(f"      sent#{cur.send.seq} {cur.send.kind} p{cur.send.port}"
              f"  after  sent#{prev.send.seq} {prev.send.kind} p{prev.send.port}")

    if a.dropped:
        w("")
        w("  DROPPED COMMANDS (published but never seen by the reader)")
        w("  " + "-" * 74)
        for s in a.dropped[:20]:
            w(f"      seq#{s.seq} {s.kind} port{s.port}={s.v} ch={s.ch} ts={s.ts_us}")

    # SPI apply order vs intended servo send order.
    if spi:
        intended_servo = [m for m in sorted(a.matched, key=lambda m: m.send.seq)
                          if m.send.kind.startswith("servo")]
        applied = sorted(spi, key=lambda r: r.rseq)
        w("")
        w("  SERVO APPLY ORDER AT SPI (first 20, in the order they hit the wire)")
        w("  " + "-" * 74)
        for r in applied[:20]:
            w(f"      rseq#{r.rseq} servo port{r.port} = {r.v} deg")
        w(f"  intended servo commands     : {len(intended_servo)}")

    w("=" * 78)
    return "\n".join(lines)


def write_trace(path: Path, a: Analysis, spi: list[RecvRec]) -> None:
    """chrome://tracing: send + recv on one shared system-clock timeline."""
    events: list[dict] = []
    all_us = [m.send.ts_us for m in a.matched] + [m.recv.w_us for m in a.matched]
    all_us += [s.ts_us for s in a.dropped] + [r.w_us for r in spi if r.w_us]
    t0 = min(all_us) if all_us else 0

    def lane(name: str, tid: int) -> None:
        events.append({"name": "thread_name", "ph": "M", "pid": 1, "tid": tid,
                       "args": {"name": name}})

    # One lane per channel for matched commands; dropped + spi get their own.
    channels = sorted({m.send.ch for m in a.matched})
    ch_tid = {ch: i + 1 for i, ch in enumerate(channels)}
    for ch, tid in ch_tid.items():
        lane(ch, tid)
    dropped_tid = len(ch_tid) + 1
    spi_tid = len(ch_tid) + 2
    lane("DROPPED", dropped_tid)
    lane("servo->SPI", spi_tid)

    for m in a.matched:
        dur = max(m.recv.w_us - m.send.ts_us, 1)
        events.append({
            "name": f"{m.send.kind} p{m.send.port}={m.send.v}",
            "cat": "cmd", "ph": "X", "pid": 1, "tid": ch_tid[m.send.ch],
            "ts": m.send.ts_us - t0, "dur": dur,
            "args": {"seq": m.send.seq, "rseq": m.recv.rseq,
                     "latency_us": m.latency_us, "ts_us": m.send.ts_us},
        })
    for s in a.dropped:
        events.append({
            "name": f"DROP {s.kind} p{s.port}={s.v}", "cat": "drop", "ph": "i",
            "s": "g", "pid": 1, "tid": dropped_tid, "ts": s.ts_us - t0,
            "args": {"seq": s.seq, "ch": s.ch},
        })
    for r in spi:
        if not r.w_us:
            continue
        events.append({
            "name": f"servo p{r.port}={r.v}", "cat": "spi", "ph": "i", "s": "g",
            "pid": 1, "tid": spi_tid, "ts": r.w_us - t0,
            "args": {"rseq": r.rseq},
        })

    with path.open("w") as f:
        json.dump({"traceEvents": events, "displayTimeUnit": "ms"}, f)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--send", required=True, type=Path, help="send-side JSONL (RACCOON_CMD_TRACE)")
    p.add_argument("--recv", required=True, type=Path, help="receive-side JSONL (WOMBAT_CMD_TRACE)")
    p.add_argument("--trace", type=Path, default=None, help="write a chrome://tracing JSON here")
    p.add_argument("--fail-on-inversion", action="store_true",
                   help="exit non-zero if any cross-servo ordering inversion is found")
    args = p.parse_args(argv)

    send = load_send(args.send)
    recv, spi = load_recv(args.recv)
    analysis = correlate(send, recv)

    sys.stdout.write(format_report(send, recv, spi, analysis) + "\n")

    if args.trace is not None:
        write_trace(args.trace, analysis, spi)
        sys.stdout.write(f"\nChrome trace written: {args.trace}\n")
        sys.stdout.write("Open it at chrome://tracing (or https://ui.perfetto.dev).\n")

    if args.fail_on_inversion:
        inv = find_inversions(analysis.matched)
        servo_cross = [
            (a, b) for (a, b) in inv
            if (a.send.ch != b.send.ch or a.send.port != b.send.port)
            and a.send.kind.startswith("servo") and b.send.kind.startswith("servo")
        ]
        if servo_cross:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
