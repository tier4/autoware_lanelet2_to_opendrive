"""Timeout-based scenario fail condition."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


class TimeoutCondition(BaseCondition):
    """Fail condition that triggers when elapsed time exceeds the timeout."""

    def __init__(self, timeout_seconds: float = 60.0) -> None:
        """Initialize the timeout condition.

        Args:
            timeout_seconds: Number of seconds before failing. Defaults to 60.0.
        """
        self.timeout_seconds = timeout_seconds

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a failure result if elapsed time exceeds the timeout.

        Args:
            world: The CARLA world instance (unused).
            elapsed: Elapsed time in seconds.

        Returns:
            ScenarioResult with passed=False if timed out, None otherwise.
        """
        if elapsed >= self.timeout_seconds:
            return ScenarioResult(
                passed=False,
                message=f"Timeout after {elapsed:.2f}s (limit: {self.timeout_seconds}s)",
                elapsed_seconds=elapsed,
            )
        return None
