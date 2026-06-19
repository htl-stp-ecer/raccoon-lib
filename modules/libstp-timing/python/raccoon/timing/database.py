from __future__ import annotations

import asyncio
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

    Two things keep persistence off the per-step *hot path*, which is where the
    "standstill between steps" came from on the Pi's SD card:

    1. The anomaly baseline is served from an in-memory rolling window (seeded
       once from disk per signature), so a step's read does no disk I/O.
    2. The database is opened in WAL mode with ``synchronous=NORMAL``, so the
       per-execution ``commit`` appends to the WAL without an fsync — the fsync
       is deferred to SQLite's own background checkpoint instead of blocking the
       step transition. (A hard power-loss can drop the last few rows, which is
       fine for diagnostic timing data.)

    Writes stay synchronous, so read-after-write is always consistent (the tests
    and the live watchdog both rely on it) — but the expensive part of the write
    no longer lands on the critical path.
    """

    def __init__(self, db_path: str) -> None:
        """Create a database wrapper for the configured SQLite path."""
        self.db_path = db_path
        self._initialized = False
        self._init_lock = asyncio.Lock()
        self._recent: dict[str, deque[float]] = {}
        self._seeded: set[str] = set()

    async def _connect(self) -> aiosqlite.Connection:
        """Open a connection with the off-hot-path pragmas applied."""
        conn = await aiosqlite.connect(self.db_path)
        await conn.execute("PRAGMA synchronous=NORMAL;")
        await conn.execute("PRAGMA busy_timeout=5000;")
        return conn

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
        """Create database and schema if needed.

        If the existing database file is corrupted, it is deleted and recreated
        automatically. WAL mode is enabled once (a persistent property of the
        file) so subsequent commits append to the WAL without a per-txn fsync.
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

            async with aiosqlite.connect(self.db_path) as db:
                await db.execute("PRAGMA journal_mode=WAL;")
                await db.execute(
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
                await db.execute(
                    """
                    CREATE INDEX IF NOT EXISTS idx_step_signature
                        ON step_executions(step_signature, timestamp_unix DESC);
                    """
                )
                await db.execute(
                    """
                    CREATE INDEX IF NOT EXISTS idx_anomalies
                        ON step_executions(anomaly)
                        WHERE anomaly = 1;
                    """
                )
                await db.commit()

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

        async with aiosqlite.connect(self.db_path) as db:
            cursor = await db.execute(
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
        """Insert a completed execution into the database.

        Updates the in-memory baseline and persists the row. The commit is cheap
        (WAL + ``synchronous=NORMAL``: no per-txn fsync), so it stays synchronous
        — keeping read-after-write consistent — without the SD-card fsync stall.
        """
        await self.initialize()
        recent = await self._ensure_seeded(signature)
        if anomaly is None:
            # Only non-anomalous samples feed the rolling baseline.
            recent.append(duration)

        row = (
            signature,
            duration,
            time.time(),
            1 if anomaly else 0,
            anomaly.expected_mean if anomaly else None,
            anomaly.expected_stddev if anomaly else None,
            anomaly.deviation_sigma if anomaly else None,
        )
        db = await self._connect()
        try:
            await db.execute(_INSERT_SQL, row)
            await db.commit()
        finally:
            await db.close()
