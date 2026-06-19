from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from collections import deque
from pathlib import Path

import aiosqlite

from .models import AnomalyDetection

logger = logging.getLogger(__name__)

# How many recent non-anomalous durations to keep per signature in memory.
# The anomaly window (TimingConfig.window_size, default 20) reads from this, so
# the cap just needs comfortable headroom over any realistic window.
_RECENT_CAP = 128

_INSERT_SQL = """
    INSERT INTO step_executions (
        step_signature,
        duration_seconds,
        timestamp_unix,
        anomaly,
        expected_mean,
        expected_stddev,
        deviation_sigma
    )
    VALUES (?, ?, ?, ?, ?, ?, ?);
"""


class StepTimingDatabase:
    """Async SQLite store for raw step execution history and anomaly flags.

    Three things keep persistence off the per-step *hot path*, which is where
    the "standstill between steps" came from on the Pi's SD card:

    1. The anomaly baseline is served from an in-memory rolling window (seeded
       once from disk per signature), so a step's read does no disk I/O.
    2. A single **persistent** connection is held for the lifetime of the
       process. The old code opened a fresh connection per write, and closing
       the last WAL connection forces a checkpoint (fsync) — so per-step
       connection churn re-introduced a ~40 ms fsync on the SD card even in WAL
       mode. Keeping one connection open means commits never checkpoint inline.
    3. WAL mode + ``synchronous=NORMAL``: each commit appends to the WAL without
       an fsync; the fsync is deferred to SQLite's background auto-checkpoint
       (every ~1000 pages), not the step transition.

    Writes stay synchronous, so read-after-write is always consistent (the tests
    and the live watchdog both rely on it) — but the expensive part of the write
    no longer lands on the critical path. A hard power-loss can drop the last
    few rows, which is fine for diagnostic timing data.
    """

    def __init__(self, db_path: str) -> None:
        """Create a database wrapper for the configured SQLite path."""
        self.db_path = db_path
        self._initialized = False
        self._init_lock = asyncio.Lock()
        self._conn: aiosqlite.Connection | None = None
        self._recent: dict[str, deque[float]] = {}
        self._seeded: set[str] = set()
        self._write_queue: asyncio.Queue[tuple] = asyncio.Queue()
        self._writer_task: asyncio.Task[None] | None = None

    async def _check_integrity(self) -> bool:
        """Return True if the database passes an integrity check."""
        try:
            async with aiosqlite.connect(self.db_path) as db:
                cursor = await db.execute("PRAGMA integrity_check")
                result = await cursor.fetchone()
                return result is not None and result[0] == "ok"
        except Exception:
            return False

    async def initialize(self) -> None:
        """Open the persistent connection and create the schema if needed.

        If the existing database file is corrupted, it is deleted and recreated
        automatically. The connection is opened in WAL mode with
        ``synchronous=NORMAL`` and kept open for the process lifetime so commits
        never trigger a checkpoint fsync on the hot path.
        """
        if self._initialized:
            return

        async with self._init_lock:
            if self._initialized:
                return

            db_file = Path(self.db_path)
            db_file.parent.mkdir(parents=True, exist_ok=True)

            if db_file.exists() and not await self._check_integrity():
                logger.warning(
                    "Timing database %s is corrupted — deleting and recreating",
                    self.db_path,
                )
                db_file.unlink()

            conn = await aiosqlite.connect(self.db_path)
            await conn.execute("PRAGMA journal_mode=WAL;")
            await conn.execute("PRAGMA synchronous=NORMAL;")
            await conn.execute("PRAGMA busy_timeout=5000;")
            await conn.execute(
                """
                CREATE TABLE IF NOT EXISTS step_executions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    step_signature TEXT NOT NULL,
                    duration_seconds REAL NOT NULL,
                    timestamp_unix REAL NOT NULL,
                    anomaly BOOLEAN DEFAULT 0,
                    expected_mean REAL,
                    expected_stddev REAL,
                    deviation_sigma REAL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
                """
            )
            await conn.execute(
                """
                CREATE INDEX IF NOT EXISTS idx_step_signature
                    ON step_executions(step_signature, timestamp_unix DESC);
                """
            )
            await conn.execute(
                """
                CREATE INDEX IF NOT EXISTS idx_anomalies
                    ON step_executions(anomaly)
                    WHERE anomaly = 1;
                """
            )
            await conn.commit()

            self._conn = conn
            self._writer_task = asyncio.ensure_future(self._writer_loop())
            self._initialized = True

    async def _ensure_seeded(self, signature: str) -> deque[float]:
        """Load a signature's recent non-anomalous durations into memory once.

        After seeding, the in-memory window is authoritative for the baseline:
        new executions are appended in :meth:`insert_execution` and never
        re-read from disk on the hot path.
        """
        recent = self._recent.get(signature)
        if recent is None:
            recent = deque(maxlen=_RECENT_CAP)
            self._recent[signature] = recent
        if signature in self._seeded:
            return recent

        assert self._conn is not None
        cursor = await self._conn.execute(
            """
            SELECT duration_seconds
            FROM step_executions
            WHERE step_signature = ? AND anomaly = 0
            ORDER BY timestamp_unix DESC
            LIMIT ?
            """,
            (signature, _RECENT_CAP),
        )
        rows = await cursor.fetchall()
        await cursor.close()
        # rows are newest-first; store oldest-first so appends extend naturally.
        recent.extend(row[0] for row in reversed(rows))
        self._seeded.add(signature)
        return recent

    async def fetch_recent_durations(self, signature: str, limit: int) -> list[float]:
        """Return the most recent non-anomalous durations for a signature.

        Served from the in-memory window (newest first), so this never touches
        disk on the hot path after the one-time seed.
        """
        await self.initialize()
        recent = await self._ensure_seeded(signature)
        if limit <= 0:
            return []
        # Newest-first, capped at `limit` — matches the old ORDER BY ... DESC.
        return list(reversed(recent))[:limit]

    async def insert_execution(
        self,
        signature: str,
        duration: float,
        anomaly: AnomalyDetection | None,
    ) -> None:
        """Record a completed execution.

        Updates the in-memory baseline synchronously (so a subsequent read sees
        it) and hands the durable write to the background writer. Returns without
        touching disk — the step never waits on the commit.
        """
        await self.initialize()
        recent = await self._ensure_seeded(signature)
        if anomaly is None:
            # Only non-anomalous samples feed the rolling baseline.
            recent.append(duration)

        self._write_queue.put_nowait(
            (
                signature,
                duration,
                time.time(),
                1 if anomaly else 0,
                anomaly.expected_mean if anomaly else None,
                anomaly.expected_stddev if anomaly else None,
                anomaly.deviation_sigma if anomaly else None,
            )
        )

    async def _writer_loop(self) -> None:
        """Persist queued executions in batches on the persistent connection.

        Owns the connection's teardown: the ``finally`` runs when the task is
        cancelled (explicitly via :meth:`aclose`, or by ``asyncio.run`` tearing
        the loop down at the end of a test), which closes the connection and
        lets aiosqlite's worker thread exit cleanly — otherwise a persistent
        connection would dangle a thread per event loop.
        """
        assert self._conn is not None
        try:
            while True:
                first = await self._write_queue.get()
                batch = [first]
                while not self._write_queue.empty() and len(batch) < 256:
                    batch.append(self._write_queue.get_nowait())
                try:
                    await self._conn.executemany(_INSERT_SQL, batch)
                    await self._conn.commit()
                except Exception:
                    logger.exception("Timing writer failed to persist %d row(s)", len(batch))
                finally:
                    for _ in batch:
                        self._write_queue.task_done()
        except asyncio.CancelledError:
            pass
        finally:
            if self._conn is not None:
                try:
                    await self._conn.close()
                except Exception:
                    logger.debug("Timing DB close failed", exc_info=True)
                self._conn = None

    async def flush(self) -> None:
        """Block until all queued executions have been committed (shutdown/tests)."""
        if self._initialized:
            await self._write_queue.join()

    async def aclose(self) -> None:
        """Flush pending writes and stop the background writer (closes the DB)."""
        if not self._initialized:
            return
        await self.flush()
        if self._writer_task is not None:
            self._writer_task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._writer_task
            self._writer_task = None
        self._initialized = False
