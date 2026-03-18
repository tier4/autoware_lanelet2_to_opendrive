"""Timeout-based scenario fail condition."""

from __future__ import annotations

from typing import Any, Optional

from ..tick_snapshot import TickSnapshot
from .base import BaseCondition, ScenarioResult


class TimeoutCondition(BaseCondition):
    """Fail condition that triggers when elapsed time exceeds the timeout."""

    def __init__(self, timeout_seconds: float = 60.0, *, label: str) -> None:
        """Initialize the timeout condition.

        Args:
            timeout_seconds: Number of seconds before failing. Defaults to 60.0.
            label: Human-readable label identifying this condition.
        """
        super().__init__(label=label)
        self.timeout_seconds = timeout_seconds

    def get_details(self) -> dict[str, Any]:
        return {"timeout_seconds": self.timeout_seconds}

    def check(self, snapshot: TickSnapshot) -> Optional[ScenarioResult]:
        """Return a failure result if elapsed time exceeds the timeout.

        Args:
            snapshot: Immutable snapshot of the current tick state.

        Returns:
            ScenarioResult with passed=False if timed out, None otherwise.
        """
        if snapshot.elapsed >= self.timeout_seconds:
            return ScenarioResult(
                passed=False,
                message=f"Timeout after {snapshot.elapsed:.2f}s (limit: {self.timeout_seconds}s)",
                elapsed_seconds=snapshot.elapsed,
            )
        return None
