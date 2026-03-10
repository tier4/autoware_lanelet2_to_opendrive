"""Elapsed-time-based scenario pass condition."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


class ElapsedTimeCondition(BaseCondition):
    """Pass condition that triggers when a specified duration has elapsed.

    Unlike :class:`TimeoutCondition` (which returns a *failure*), this condition
    returns a *success* result, making it suitable for scenarios that should pass
    after a certain amount of time (e.g. "drive for 30 seconds without issues").
    """

    def __init__(self, duration_seconds: float) -> None:
        """Initialize the elapsed time condition.

        Args:
            duration_seconds: Number of seconds that must elapse before the
                condition succeeds.  Must be positive.

        Raises:
            ValueError: If *duration_seconds* is not positive.
        """
        if duration_seconds <= 0:
            raise ValueError("duration_seconds must be positive")
        self.duration_seconds = duration_seconds

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a success result if the elapsed time reaches the duration.

        Args:
            world: The CARLA world instance (unused).
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            ScenarioResult with passed=True if duration reached, None otherwise.
        """
        if elapsed >= self.duration_seconds:
            return ScenarioResult(
                passed=True,
                message=(
                    f"Elapsed time {elapsed:.2f}s reached target "
                    f"duration {self.duration_seconds:.2f}s"
                ),
                elapsed_seconds=elapsed,
            )
        return None
