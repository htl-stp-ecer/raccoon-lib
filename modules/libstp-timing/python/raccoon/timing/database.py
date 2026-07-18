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

    The per-step *hot path* does **zero disk I/O**. On the Pi's SD card the disk
    was the sole source of the "standstill between steps": a per-step
    ``record_execution`` SELECT that could stall behind the background writer's
    WAL-checkpoint fsync (measured up to ~1.4 s on a single step). So the design
    is now "all in RAM during the run, persist once after it":

    1. **Reads never touch disk during the run.** The whole history is
       bulk-seeded into per-signature in-memory windows *once* at
       :meth:`initialize` (before the run starts). After that the in-memory
       window is authoritative — :meth:`fetch_recent_durations` and the anomaly
       baseline are pure RAM.
    2. **Writes never touch disk during the run.** :meth:`insert_execution`
       appends to an in-RAM buffer and updates the in-memory baseline
       synchronously (so read-after-write stays consistent). Nothing is
       committed on the step transition.
    3. **The durable write happens once, after the run**, when :meth:`flush`
       (or :meth:`aclose`) is called from the run-completion path. All buffered
       rows go out in a single ``executemany`` + ``commit`` while the robot is
       already idle, so the WAL-checkpoint fsync never lands mid-run.

    A hard power-loss (process killed before ``flush``) drops that run's timing
    rows, which is fine for diagnostic data — the trade the caller explicitly
    accepts to keep the run's motion smooth.
    """

    def __init__(self, db_path: str) -> None:
        """Create a database wrapper for the configured SQLite path."""
        self.db_path = db_path
        self._initialized = False
        self._init_lock = asyncio.Lock()
        self._conn: aiosqlite.Connection | None = None
        self._recent: dict[str, deque[float]] = {}
        self._seeded: set[str] = set()
        self._bulk_seeded = False
        # Durable writes are deferred to :meth:`flush`; rows accumulate here in
        # RAM during the run so the step transition never waits on disk.
        self._pending: list[tuple] = []
        # Background task whose sole job is to own the connection's teardown, so
        # the persistent connection (and aiosqlite's worker thread) is closed
        # cleanly when the event loop is torn down without an explicit aclose().
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
        automatically. The whole history is bulk-loaded into the in-memory
        windows here (once, before the run) so no per-signature SELECT ever runs
        on the hot path, and durable writes are deferred to :meth:`flush`.
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
            await self._bulk_seed()
            self._writer_task = asyncio.ensure_future(self._conn_teardown_guard())
            self._initialized = True

    async def _bulk_seed(self) -> None:
        """Load the entire non-anomalous history into memory in one scan.

        Done once at startup (before the run) so the hot path never issues a
        per-signature SELECT. Rows come oldest-first and extend the capped
        per-signature deques, so each window keeps its most recent
        ``_RECENT_CAP`` samples — exactly what the lazy per-signature seed used
        to produce, minus the mid-run disk reads.
        """
        assert self._conn is not None
        cursor = await self._conn.execute(
            """
            SELECT step_signature, duration_seconds
            FROM step_executions
            WHERE anomaly = 0
            ORDER BY timestamp_unix ASC
            """
        )
        rows = await cursor.fetchall()
        await cursor.close()
        for signature, duration in rows:
            recent = self._recent.get(signature)
            if recent is None:
                recent = deque(maxlen=_RECENT_CAP)
                self._recent[signature] = recent
            recent.append(duration)
        # From here on every signature is authoritative in RAM; a signature not
        # present above simply starts with an empty window on first use.
        self._bulk_seeded = True

    async def _ensure_seeded(self, signature: str) -> deque[float]:
        """Return a signature's in-memory window, creating an empty one if new.

        After the one-time :meth:`_bulk_seed`, the in-memory window is
        authoritative for the baseline and this never touches disk: a
        first-seen signature just gets a fresh empty deque.
        """
        recent = self._recent.get(signature)
        if recent is None:
            recent = deque(maxlen=_RECENT_CAP)
            self._recent[signature] = recent
        if self._bulk_seeded or signature in self._seeded:
            self._seeded.add(signature)
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
        it) and buffers the durable write in RAM. Returns without touching disk
        — the step never waits on a commit; persistence happens in :meth:`flush`
        after the run.
        """
        await self.initialize()
        recent = await self._ensure_seeded(signature)
        if anomaly is None:
            # Only non-anomalous samples feed the rolling baseline.
            recent.append(duration)

        self._pending.append(
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

    async def _conn_teardown_guard(self) -> None:
        """Hold the connection open and close it cleanly on teardown.

        The connection is kept for the process lifetime (bulk seed + the final
        flush use it). This task's only job is to close it — and let aiosqlite's
        worker thread exit — when it is cancelled, either explicitly via
        :meth:`aclose` or by ``asyncio.run`` tearing the loop down at the end of
        a test. Without it, a persistent connection would dangle a thread per
        event loop.
        """
        try:
            await asyncio.Event().wait()  # sleep until cancelled
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
        """Persist all buffered executions to disk in one batched commit.

        Called from the run-completion path (and by :meth:`aclose`/tests). This
        is the *only* place the step-timing history touches the disk on the hot
        connection, so the WAL-checkpoint fsync it may trigger lands while the
        robot is already idle, never between two steps.
        """
        if not self._initialized or self._conn is None or not self._pending:
            return
        batch, self._pending = self._pending, []
        try:
            await self._conn.executemany(_INSERT_SQL, batch)
            await self._conn.commit()
        except Exception:
            logger.exception("Timing writer failed to persist %d row(s)", len(batch))

    async def aclose(self) -> None:
        """Flush pending writes and close the DB (explicit shutdown/tests)."""
        if not self._initialized:
            return
        await self.flush()
        # Close the connection here, in a normal (non-cancelled) context, so
        # aiosqlite's worker thread has fully drained before the loop tears
        # down — closing it from inside the guard's cancelled ``finally`` can
        # leave an in-flight result callback firing on an already-closed loop.
        if self._conn is not None:
            with contextlib.suppress(Exception):
                await self._conn.close()
            self._conn = None
        if self._writer_task is not None:
            self._writer_task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._writer_task
            self._writer_task = None
        self._initialized = False
