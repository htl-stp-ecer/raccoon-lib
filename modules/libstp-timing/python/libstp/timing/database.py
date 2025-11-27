import asyncio
import time
from pathlib import Path
from typing import List, Optional

import aiosqlite

from .models import AnomalyDetection


class StepTimingDatabase:
    def __init__(self, db_path: str) -> None:
        self.db_path = db_path
        self._initialized = False
        self._init_lock = asyncio.Lock()

    async def initialize(self) -> None:
        """Create database and schema if needed."""
        if self._initialized:
            return

        async with self._init_lock:
            if self._initialized:
                return

            Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)

            async with aiosqlite.connect(self.db_path) as db:
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

    async def fetch_recent_durations(self, signature: str, limit: int) -> List[float]:
        """Return the most recent non-anomalous durations for a signature."""
        await self.initialize()
        query = """
            SELECT duration_seconds
            FROM step_executions
            WHERE step_signature = ? AND anomaly = 0
            ORDER BY timestamp_unix DESC
            LIMIT ?
        """

        async with aiosqlite.connect(self.db_path) as db:
            cursor = await db.execute(query, (signature, limit))
            rows = await cursor.fetchall()

        return [row[0] for row in rows]

    async def insert_execution(
        self,
        signature: str,
        duration: float,
        anomaly: Optional[AnomalyDetection],
    ) -> None:
        """Insert a completed execution into the database."""
        await self.initialize()

        async with aiosqlite.connect(self.db_path) as db:
            await db.execute(
                """
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
                """,
                (
                    signature,
                    duration,
                    time.time(),
                    1 if anomaly else 0,
                    anomaly.expected_mean if anomaly else None,
                    anomaly.expected_stddev if anomaly else None,
                    anomaly.deviation_sigma if anomaly else None,
                ),
            )
            await db.commit()
